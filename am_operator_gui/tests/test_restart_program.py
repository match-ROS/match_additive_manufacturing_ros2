"""Program restart must only play after a successful dashboard stop."""

import sys
from types import ModuleType, SimpleNamespace

import pytest

from am_operator_gui.ur_dashboard import DASHBOARD_NAMESPACE, dashboard_command


@pytest.mark.parametrize('failure', [None, 'rejected', 'unavailable', 'timeout'])
def test_restart_checks_stop_before_play(monkeypatch, failure):
    calls = []
    cleanup = []

    def create_client(service_type, name):
        calls.append(name)
        response = SimpleNamespace(success=failure != 'rejected', message='dashboard response')
        future = SimpleNamespace(done=lambda: failure != 'timeout', result=lambda: response)
        return SimpleNamespace(
            wait_for_service=lambda **kwargs: failure != 'unavailable',
            call_async=lambda request: future,
        )

    node = SimpleNamespace(create_client=create_client, destroy_client=lambda client: None,
                           destroy_node=lambda: cleanup.append('node'))
    rclpy = ModuleType('rclpy')
    rclpy.init = lambda: None
    rclpy.create_node = lambda name: node
    rclpy.spin_until_future_complete = lambda *args, **kwargs: None
    rclpy.shutdown = lambda: cleanup.append('shutdown')
    srv = ModuleType('std_srvs.srv')
    srv.Trigger = SimpleNamespace(Request=lambda: object())
    monkeypatch.setitem(sys.modules, 'rclpy', rclpy)
    monkeypatch.setitem(sys.modules, 'std_srvs.srv', srv)
    command = dashboard_command('restart_program')
    monkeypatch.setattr(sys, 'argv', ['-c', command[3]])

    if failure:
        with pytest.raises(SystemExit) as exc:
            exec(command[2], {})
        assert exc.value.code == 1
        assert calls == [f'{DASHBOARD_NAMESPACE}/stop']
    else:
        exec(command[2], {})
        assert calls == [f'{DASHBOARD_NAMESPACE}/stop', f'{DASHBOARD_NAMESPACE}/play']
    assert cleanup == ['node', 'shutdown']


def test_restart_action_dispatch_and_status(tmp_path, monkeypatch):
    from am_operator_gui.operator_service import OperatorService
    from am_operator_gui.web_app import ACTION_ENDPOINTS

    service = OperatorService(config_path=tmp_path / 'config.json')
    calls = []
    monkeypatch.setattr(service.processes, 'start', lambda name, command: calls.append((name, command)))
    service.action('restart_program')
    assert calls == [('restart_program', dashboard_command('restart_program'))]
    assert 'restart_program' in ACTION_ENDPOINTS
    process = SimpleNamespace(is_running=lambda: False, poll=lambda: 1)
    monkeypatch.setattr(service.processes, 'get', lambda name: process if name == 'restart_program' else None)
    assert service._action_states()['restart_program']['state'] == 'error'
