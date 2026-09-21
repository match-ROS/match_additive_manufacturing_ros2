import io
import json
import signal
import subprocess
import sys
import threading
import time

import pytest

from am_operator_gui.remote_supervisor import RemoteSupervisor


def events(stream):
    return [json.loads(line) for line in stream.getvalue().splitlines()]


def wait_until(predicate, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError('condition did not become true')


def test_process_output_exit_status_and_logs_request():
    output = io.StringIO()
    supervisor = RemoteSupervisor(stdin=io.StringIO(), stdout=output)
    supervisor.start('worker', [sys.executable, '-c', 'print("hello remote")'])
    wait_until(lambda: 'worker' not in supervisor.children)
    supervisor._handle({'op': 'logs', 'id': 7, 'name': 'worker', 'limit': 10})

    emitted = events(output)
    assert any(item.get('event') == 'started' and item.get('name') == 'worker' for item in emitted)
    assert any(item.get('event') == 'log' and item.get('message') == 'hello remote' for item in emitted)
    assert any(item.get('event') == 'exited' and item.get('return_code') == 0 for item in emitted)
    log_reply = next(item for item in emitted if item.get('event') == 'logs')
    assert log_reply['messages'] == ['hello remote']


def test_duplicate_start_is_rejected_and_stop_is_confirmed():
    output = io.StringIO()
    supervisor = RemoteSupervisor(stdin=io.StringIO(), stdout=output)
    command = [sys.executable, '-c', 'import time; time.sleep(30)']
    supervisor.start('worker', command)
    with pytest.raises(RuntimeError, match='already running'):
        supervisor.start('worker', command)
    supervisor.stop('worker', 'test')
    wait_until(lambda: 'worker' not in supervisor.children)
    exited = [item for item in events(output) if item.get('event') == 'exited']
    assert exited[-1]['expected'] is True
    assert exited[-1]['return_code'] != 0


def test_log_buffer_is_bounded():
    supervisor = RemoteSupervisor(stdin=io.StringIO(), stdout=io.StringIO())
    command = [sys.executable, '-c', 'for i in range(520): print(i)']
    supervisor.start('noisy', command)
    wait_until(lambda: 'noisy' not in supervisor.children)
    assert len(supervisor.logs['noisy']) == 500
    assert supervisor.logs['noisy'][-1] == '519'


def test_watchdog_stops_once_after_heartbeat_timeout(monkeypatch):
    import am_operator_gui.remote_supervisor as module

    monkeypatch.setattr(module, 'HEARTBEAT_TIMEOUT_S', 0.01)
    output = io.StringIO()
    supervisor = RemoteSupervisor(stdin=io.StringIO(), stdout=output)
    calls = []

    def stopped(reason):
        calls.append(reason)
        supervisor.stopping = True

    supervisor.stop_all = stopped
    thread = threading.Thread(target=supervisor._watchdog)
    thread.start()
    thread.join(timeout=1.0)
    assert calls == ['GUI heartbeat timeout']
    assert any(item.get('event') == 'watchdog' for item in events(output))


def test_forced_cleanup_escalates_from_sigint_to_sigterm_and_sigkill(monkeypatch):
    sent = []

    class StubbornProcess:
        pid = 1234
        return_code = None

        def poll(self):
            return self.return_code

        def wait(self, timeout=None):
            if self.return_code is None:
                raise subprocess.TimeoutExpired('stubborn', timeout)
            return self.return_code

    process = StubbornProcess()

    def kill_group(pid, sig):
        sent.append((pid, sig))
        if sig == signal.SIGKILL:
            process.return_code = -signal.SIGKILL

    monkeypatch.setattr('am_operator_gui.remote_supervisor.os.killpg', kill_group)
    RemoteSupervisor._terminate_group(process)
    assert [sig for _, sig in sent] == [signal.SIGINT, signal.SIGTERM, signal.SIGKILL]
