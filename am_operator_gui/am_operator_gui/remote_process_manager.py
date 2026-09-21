"""Persistent SSH client for the robot-side process supervisor."""

from __future__ import annotations

import json
import os
import shlex
import subprocess
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Callable, Optional


OutputCallback = Callable[[str, str], None]
StateCallback = Callable[[str], None]
REMOTE_PACKAGES = (
    'am_operator_gui', 'am_jparse_controller', 'ur_trajectory_follower',
    'base_trajectory_follower', 'move_to_path_idx',
)


class RemoteProcessError(ValueError):
    pass


@dataclass
class RemoteManagedProcess:
    name: str
    command: list[str] = field(default_factory=list)
    output: deque[str] = field(default_factory=lambda: deque(maxlen=500))
    running: bool = False
    return_code: Optional[int] = None

    def is_running(self) -> bool:
        return self.running

    def poll(self) -> Optional[int]:
        return None if self.running else self.return_code


class RemoteProcessManager:
    def __init__(
        self,
        output_callback: Optional[OutputCallback] = None,
        state_callback: Optional[StateCallback] = None,
        target: str = 'robot@192.168.0.200',
        workspace: str = '/home/robot/b04_gui_ws',
        ros_domain_id: int = 38,
    ) -> None:
        self.output_callback = output_callback
        self.state_callback = state_callback
        self.target = target
        self.workspace = workspace.rstrip('/')
        self.ros_domain_id = int(ros_domain_id)
        self._process: Optional[subprocess.Popen] = None
        self._processes: dict[str, RemoteManagedProcess] = {}
        self._write_lock = threading.Lock()
        self._state_lock = threading.RLock()
        self._hello = threading.Event()
        self._hello_error: Optional[str] = None
        self._closed = False
        self._connected = False
        self._unknown = False
        self._request_id = 0
        self._responses: dict[int, dict] = {}
        self._response_condition = threading.Condition()

    @property
    def connected(self) -> bool:
        with self._state_lock:
            return self._connected and self._process is not None and self._process.poll() is None

    @property
    def status_unknown(self) -> bool:
        with self._state_lock:
            return self._unknown

    def get(self, name: str) -> Optional[RemoteManagedProcess]:
        with self._state_lock:
            return self._processes.get(name)

    def snapshots(self) -> dict[str, RemoteManagedProcess]:
        with self._state_lock:
            return dict(self._processes)

    def connect(self, timeout: float = 10.0) -> None:
        if self.connected:
            return
        local_domain = os.environ.get('ROS_DOMAIN_ID', '0').strip() or '0'
        if local_domain != str(self.ros_domain_id):
            message = f'ROS_DOMAIN_ID mismatch (expected {self.ros_domain_id}, got {local_domain})'
            self._log('remote:preflight', f'ERROR: {message}')
            raise RemoteProcessError(message)
        self.close(stop_remote=False)
        self._closed = False
        self._hello.clear()
        self._hello_error = None
        remote = self._remote_supervisor_command()
        command = [
            'ssh', '-T', '-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=yes',
            '-o', 'ConnectTimeout=8', self.target, 'bash', '-lc', remote,
        ]
        self._log('remote:preflight', f'connecting to {self.target}')
        try:
            self._process = subprocess.Popen(
                command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                text=True, bufsize=1, start_new_session=True,
            )
        except OSError as exc:
            self._mark_disconnected(f'could not start ssh: {exc}')
            raise RemoteProcessError(str(exc)) from exc
        threading.Thread(target=self._read_protocol, daemon=True).start()
        threading.Thread(target=self._read_stderr, daemon=True).start()
        threading.Thread(target=self._wait_ssh, daemon=True).start()
        if not self._hello.wait(timeout):
            code = self._process.poll() if self._process is not None else None
            self.close(stop_remote=False)
            message = self._hello_error or 'SSH timeout waiting for remote supervisor'
            if code is not None:
                message += f' (exit code {code})'
            self._log('remote:preflight', f'ERROR: {message}')
            raise RemoteProcessError(message)
        if self._hello_error:
            message = self._hello_error
            self.close(stop_remote=False)
            raise RemoteProcessError(message)
        with self._state_lock:
            self._connected = True
            self._unknown = False
        self._notify_state('connected')
        threading.Thread(target=self._heartbeat, daemon=True).start()
        try:
            self.request('status', timeout=3.0)
        except RemoteProcessError as exc:
            self._mark_disconnected(f'preflight status failed: {exc}')
            self.close(stop_remote=False)
            raise
        self._log('remote:preflight', 'remote supervisor ready')

    def _remote_supervisor_command(self) -> str:
        setup = f'{self.workspace}/install/setup.bash'
        packages = ' '.join(shlex.quote(item) for item in REMOTE_PACKAGES)
        script = (
            'set -e; '
            'test -r /opt/ros/jazzy/setup.bash || { echo "ERROR: ROS Jazzy setup missing" >&2; exit 20; }; '
            f'test -r {shlex.quote(setup)} || {{ echo "ERROR: remote setup.bash missing: {setup}" >&2; exit 21; }}; '
            'source /opt/ros/jazzy/setup.bash 1>&2; '
            'test ! -r /home/robot/ros_config.sh || source /home/robot/ros_config.sh 1>&2; '
            f'source {shlex.quote(setup)} 1>&2; '
            f'test "${{ROS_DOMAIN_ID:-0}}" = "{self.ros_domain_id}" || '
            f'{{ echo "ERROR: ROS_DOMAIN_ID mismatch (expected {self.ros_domain_id}, got ${{ROS_DOMAIN_ID:-0}})" >&2; exit 23; }}; '
            f'export ROS_DOMAIN_ID={self.ros_domain_id}; '
            f'for package in {packages}; do ros2 pkg prefix "$package" >/dev/null || '
            '{ echo "ERROR: package not built: $package" >&2; exit 22; }; done; '
            'exec ros2 run am_operator_gui remote_process_supervisor'
        )
        return shlex.quote(script)

    def start(self, name: str, command: list[str], replace: bool = True) -> RemoteManagedProcess:
        with self._state_lock:
            existing = self._processes.get(name)
        if existing is not None and existing.running:
            if not replace:
                return existing
            self.stop(name)
        managed = RemoteManagedProcess(name=name, command=list(command), running=False)
        with self._state_lock:
            self._processes[name] = managed
        try:
            self.connect()
            self.request('start', name=name, command=command)
        except Exception:
            managed.running = False
            managed.return_code = 1
            self._notify_state(name)
            raise
        return managed

    def stop(self, name: str) -> None:
        if not self.connected:
            raise RemoteProcessError('remote supervisor is not connected; stop is unconfirmed')
        self.request('stop', name=name, timeout=7.0)

    def stop_all(self) -> None:
        if not self.connected:
            with self._state_lock:
                running = any(item.running for item in self._processes.values())
            if running:
                raise RemoteProcessError('remote supervisor is not connected; remote stop is unconfirmed')
            return
        self.request('stop_all', timeout=40.0)

    def request(self, operation: str, timeout: float = 5.0, **payload) -> dict:
        if not self.connected and operation != 'status':
            raise RemoteProcessError('remote supervisor is not connected')
        with self._response_condition:
            self._request_id += 1
            request_id = self._request_id
        self._send(dict(payload, op=operation, id=request_id))
        deadline = time.monotonic() + timeout
        with self._response_condition:
            while request_id not in self._responses:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise RemoteProcessError(f'remote {operation} timed out')
                self._response_condition.wait(remaining)
            response = self._responses.pop(request_id)
        if response.get('event') == 'error' or response.get('ok') is False:
            raise RemoteProcessError(str(response.get('message', f'remote {operation} failed')))
        return response

    def _send(self, payload: dict) -> None:
        process = self._process
        if process is None or process.stdin is None or process.poll() is not None:
            raise RemoteProcessError('SSH channel is not available')
        try:
            with self._write_lock:
                process.stdin.write(json.dumps(payload, separators=(',', ':')) + '\n')
                process.stdin.flush()
        except (BrokenPipeError, OSError) as exc:
            self._mark_disconnected(f'SSH write failed: {exc}')
            raise RemoteProcessError(str(exc)) from exc

    def _read_protocol(self) -> None:
        process = self._process
        if process is None or process.stdout is None:
            return
        for raw in process.stdout:
            try:
                event = json.loads(raw)
            except json.JSONDecodeError:
                self._log('remote:ssh', f'ERROR: invalid supervisor output: {raw.rstrip()}')
                continue
            self._handle_event(event)

    def _handle_event(self, event: dict) -> None:
        kind = event.get('event')
        request_id = event.get('id')
        if request_id is not None and kind in {'ack', 'status', 'logs', 'error'}:
            with self._response_condition:
                self._responses[int(request_id)] = event
                self._response_condition.notify_all()
        if kind == 'hello':
            version = int(event.get('version', -1))
            if version != 1:
                self._hello_error = f'remote supervisor protocol version mismatch (expected 1, got {version})'
                self._log('remote:preflight', f'ERROR: {self._hello_error}')
                self._hello.set()
                return
            self._hello.set()
        elif kind == 'started':
            with self._state_lock:
                item = self._processes.setdefault(event['name'], RemoteManagedProcess(event['name']))
                item.running, item.return_code = True, None
            self._notify_state(event['name'])
        elif kind == 'stopping':
            self._log(f"remote:{event['name']}", f"stopping: {event.get('reason', '')}")
        elif kind == 'log':
            message = str(event.get('message', ''))
            with self._state_lock:
                item = self._processes.setdefault(event['name'], RemoteManagedProcess(event['name']))
                item.output.append(message)
            self._log(f"remote:{event['name']}", message)
        elif kind == 'exited':
            actual_code = int(event.get('return_code', -1))
            with self._state_lock:
                item = self._processes.setdefault(event['name'], RemoteManagedProcess(event['name']))
                item.running = False
                item.return_code = 0 if event.get('expected') else actual_code
            prefix = '' if event.get('expected') else 'ERROR: unexpected '
            self._log(f"remote:{event['name']}", f'{prefix}exit with code {actual_code}')
            self._notify_state(event['name'])
        elif kind == 'watchdog':
            self._log('remote:ssh', f"ERROR: {event.get('message', 'remote watchdog fired')}")
        elif kind == 'status':
            states = event.get('processes', {})
            with self._state_lock:
                for name, item in self._processes.items():
                    if name not in states:
                        was_running = item.running
                        item.running = False
                        item.return_code = 1 if was_running else item.return_code
                for name, state in states.items():
                    item = self._processes.setdefault(name, RemoteManagedProcess(name))
                    item.running = bool(state.get('running'))
                    item.return_code = state.get('return_code')
                    item.command = list(state.get('command', item.command))
            self._notify_state('status')

    def _read_stderr(self) -> None:
        process = self._process
        if process is None or process.stderr is None:
            return
        for line in process.stderr:
            message = line.rstrip()
            if message:
                self._log('remote:ssh', ('ERROR: ' if 'error' not in message.lower() else '') + message)

    def _wait_ssh(self) -> None:
        process = self._process
        if process is None:
            return
        code = process.wait()
        if self._process is process and not self._closed:
            self._mark_disconnected(f'SSH connection exited with code {code}; remote status unknown')

    def _heartbeat(self) -> None:
        while self.connected and not self._closed:
            try:
                self._send({'op': 'ping'})
            except RemoteProcessError:
                return
            time.sleep(1.0)

    def _mark_disconnected(self, message: str) -> None:
        with self._state_lock:
            self._connected = False
            self._unknown = True
        self._log('remote:ssh', f'ERROR: {message}')
        self._notify_state('unknown')

    def close(self, stop_remote: bool = True) -> None:
        process = self._process
        if process is None:
            self._closed = True
            with self._state_lock:
                self._connected = False
            return
        if stop_remote and self.connected:
            try:
                self.stop_all()
            except RemoteProcessError as exc:
                self._log('remote:ssh', f'ERROR: {exc}')
        self._closed = True
        try:
            if process.stdin is not None:
                process.stdin.close()
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=1.0)
        self._process = None
        with self._state_lock:
            self._connected = False

    def _log(self, source: str, message: str) -> None:
        if self.output_callback is not None:
            self.output_callback(source, message)

    def _notify_state(self, state: str) -> None:
        if self.state_callback is not None:
            self.state_callback(state)
