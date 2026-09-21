import json

import pytest

from am_operator_gui.operator_service import OperatorService, REMOTE_CONTROL_PROCESSES
from am_operator_gui.remote_process_manager import (
    RemoteManagedProcess, RemoteProcessError, RemoteProcessManager,
)


class FakeRegistry:
    def __init__(self, fail_name=None):
        self.items = {}
        self._processes = self.items
        self.started = []
        self.stop_all_calls = 0
        self.fail_name = fail_name

    def get(self, name):
        return self.items.get(name)

    def start(self, name, command, replace=True):
        item = RemoteManagedProcess(name, list(command))
        self.items[name] = item
        self.started.append((name, list(command), replace))
        if name == self.fail_name:
            item.return_code = 1
            raise RemoteProcessError(f'{name} failed')
        item.running = True
        return item

    def stop(self, name):
        if name in self.items:
            self.items[name].running = False

    def stop_all(self):
        self.stop_all_calls += 1
        for item in self.items.values():
            item.running = False

    def snapshots(self):
        return dict(self.items)


class FakeRemote(FakeRegistry):
    connected = False
    status_unknown = False

    def connect(self):
        self.connected = True

    def close(self, stop_remote=True):
        self.connected = False


def service(tmp_path, *, remote=True, simulation=False):
    config = tmp_path / 'operator.json'
    config.write_text(json.dumps({
        'simulation': simulation,
        'control_execution_target': 'robot',
        'trajectory_directory': '',
    }))
    return OperatorService(config_path=config, allow_remote_execution=remote)


def test_hardware_controls_route_remote_but_simulation_and_qt_stay_local(tmp_path):
    web = service(tmp_path, remote=True)
    assert all(web._use_remote_process(name) for name in REMOTE_CONTROL_PROCESSES)

    web.config['simulation'] = True
    assert not any(web._use_remote_process(name) for name in REMOTE_CONTROL_PROCESSES)

    qt = service(tmp_path, remote=False)
    assert not any(qt._use_remote_process(name) for name in REMOTE_CONTROL_PROCESSES)


def test_remote_start_failure_does_not_fall_back_local_and_stays_error(tmp_path):
    operator = service(tmp_path)
    operator.remote_processes = FakeRemote(fail_name='path_index')
    operator.processes = FakeRegistry()

    with pytest.raises(RemoteProcessError):
        operator.action('path_index')

    assert not operator.processes.started
    assert operator._action_states()['path_index']['state'] == 'error'
    assert any(log['source'] == 'remote:path_index' and log['level'] == 'error'
               for log in operator.snapshot()['logs'])


def test_launch_all_rolls_back_partial_remote_and_local_starts(tmp_path):
    operator = service(tmp_path)
    operator.remote_processes = FakeRemote(fail_name='base_follower')
    operator.processes = FakeRegistry()
    operator.start_pose_adapters = lambda: None

    with pytest.raises(RemoteProcessError):
        operator.action('launch_all')

    assert operator.remote_processes.stop_all_calls == 1
    assert operator.processes.stop_all_calls == 1
    assert operator._action_states()['launch_all']['state'] == 'error'


def test_target_change_and_remote_build_require_stopped_controls(tmp_path):
    operator = service(tmp_path)
    remote = FakeRemote()
    operator.remote_processes = remote
    operator.processes = FakeRegistry()
    remote.start('path_index', ['ros2'])

    with pytest.raises(ValueError, match='before changing'):
        operator.update_config({'control_execution_target': 'local'})
    with pytest.raises(ValueError, match='before building'):
        operator.action('build_remote')


def test_build_remote_is_separate_ssh_colcon_action(tmp_path):
    operator = service(tmp_path)
    operator.remote_processes = FakeRemote()
    operator.processes = FakeRegistry()
    operator.action('build_remote')

    name, command, _ = operator.processes.started[-1]
    assert name == 'remote:build'
    assert command[0] == 'ssh'
    assert 'colcon build --symlink-install --packages-up-to' in ' '.join(command)


def test_path_publisher_is_one_shot_transient_source(tmp_path):
    operator = service(tmp_path)
    assert 'publish_once:=true' in operator.command_for('publish_path')


def test_received_static_paths_do_not_expire_from_gui_readiness():
    from am_operator_gui.ros_bridge import OperatorGuiNode

    class Clock:
        def now(self):
            return object()

    class State:
        _last_robot_pose_time = None
        _last_arm_pose_time = None
        _last_base_path_time = object()
        _last_arm_path_time = object()
        _has_base_path = True
        _has_arm_path = True
        _has_path = False
        _last_jparse_ready_time = None

        def get_clock(self):
            return Clock()

        def _emit_status(self):
            pass

    state = State()
    OperatorGuiNode._freshness_tick(state)
    assert state._has_path is True


def test_preflight_rejects_local_domain_mismatch_without_opening_ssh(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '17')
    manager = RemoteProcessManager(ros_domain_id=38)
    with pytest.raises(RemoteProcessError, match='expected 38, got 17'):
        manager.connect()
    assert manager.connected is False


def test_ssh_loss_is_reported_as_unknown_and_unexpected_exit_is_error():
    messages = []
    manager = RemoteProcessManager(output_callback=lambda source, message: messages.append((source, message)))
    manager._handle_event({'event': 'started', 'name': 'arm_follower', 'pid': 1})
    manager._mark_disconnected('SSH connection exited with code 255; remote status unknown')
    manager._handle_event({
        'event': 'exited', 'name': 'arm_follower', 'return_code': 7, 'expected': False,
    })

    assert manager.status_unknown is True
    assert manager.get('arm_follower').return_code == 7
    assert ('remote:ssh', 'ERROR: SSH connection exited with code 255; remote status unknown') in messages
    assert ('remote:arm_follower', 'ERROR: unexpected exit with code 7') in messages


def test_web_exposes_remote_actions_and_execution_selector():
    from am_operator_gui.web_app import ACTION_ENDPOINTS, WEB_ROOT

    page = (WEB_ROOT / 'templates' / 'index.html').read_text()
    assert {'check_remote', 'build_remote'} <= set(ACTION_ENDPOINTS)
    assert 'data-setting="control_execution_target"' in page
    assert 'data-action="check_remote"' in page
    assert 'data-action="build_remote"' in page
