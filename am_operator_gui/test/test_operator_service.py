from pathlib import Path

from am_operator_gui.operator_service import OperatorService


class FakeProcess:
    def __init__(self, running=False):
        self._running = running
        self.return_code = None

    def is_running(self):
        return self._running

    def poll(self):
        return self.return_code


class FakeProcesses:
    def __init__(self):
        self._processes = {}
        self.started = []
        self.stopped = []

    def start(self, name, command, replace=True):
        self.started.append((name, command, replace))
        self._processes[name] = FakeProcess(True)

    def get(self, name):
        return self._processes.get(name)

    def stop(self, name):
        self.stopped.append(name)
        if name in self._processes:
            self._processes[name]._running = False

    def stop_all(self):
        self.stopped.extend(self._processes)
        for process in self._processes.values():
            process._running = False


class FakeRosBridge:
    def __init__(self):
        self.calls = []

    def publish_path_index(self, value):
        self.calls.append(('path_index', value))

    def publish_start_condition(self, value):
        self.calls.append(('start_condition', value))

    def publish_stop_commands(self, frame):
        self.calls.append(('stop_commands', frame))

    def lookup_tool_offset(self, *_args):
        return None

    def latest_base_path_pose(self, _index):
        return None

    def latest_robot_pose(self):
        return None

    def original_arm_index_for_tracking_index(self, index):
        return index // 2

    def publish_velocity_override(self, _value):
        pass

    def publish_spray_distance(self, _value):
        pass

    def publish_desired_arm_speed(self, _value):
        pass


def make_service(tmp_path: Path) -> OperatorService:
    service = OperatorService(tmp_path / 'operator.json')
    service.processes = FakeProcesses()
    return service


def test_service_persists_explicit_web_settings_only(tmp_path: Path) -> None:
    service = make_service(tmp_path)

    config = service.update_config({'platform': 'bunker', 'path_index': 7, 'ignored': 'value'})

    assert config['platform'] == 'bunker'
    assert config['path_index'] == 7
    assert 'ignored' not in config
    assert 'cmd_vel_topic:=/diff_drive_controller/cmd_vel' in service.command_for('move_base')


def test_hardware_launch_all_uses_pose_adapters_not_simulator(tmp_path: Path) -> None:
    service = make_service(tmp_path)
    service.update_config({'simulation': False})

    service.action('launch_all')

    names = [name for name, _command, _replace in service.processes.started]
    assert 'simulation' not in names
    assert 'base_pose_adapter' in names
    assert {'publish_path', 'controllers', 'path_index', 'base_follower', 'arm_follower'} <= set(names)


def test_action_toggle_and_stop_all(tmp_path: Path) -> None:
    service = make_service(tmp_path)

    service.action('publish_path')
    service.action('publish_path')
    service.action('stop_all')

    assert service.processes.stopped[0] == 'publish_path'
    assert service._launch_all_active is False


def test_path_transform_is_forwarded_to_path_publisher(tmp_path: Path) -> None:
    service = make_service(tmp_path)
    service.update_config({'path_transform': {'x': 1, 'y': -2, 'z': 0.5, 'yaw_deg': 90}})

    command = service.command_for('publish_path')

    assert 'path_transform_xyz:=[1.000000, -2.000000, 0.500000]' in command
    assert 'path_transform_yaw_deg:=90.000000' in command


def test_arm_and_index_commands_include_selected_speed_mode(tmp_path: Path) -> None:
    service = make_service(tmp_path)
    service.update_config({'simulation': True, 'default_velocity_enabled': True, 'default_velocity': 0.05})

    arm_command = service.command_for('arm_follower')
    index_command = service.command_for('path_index')

    assert 'progress_mode:=desired_speed' in arm_command
    assert 'default_velocity:=0.050000' in arm_command
    assert 'desired_arm_speed:=0.050000' in index_command


def test_every_web_action_has_a_dry_run_service_path(tmp_path: Path) -> None:
    service = make_service(tmp_path)
    service.ros_bridge = FakeRosBridge()

    command_actions = (
        'simulation', 'publish_path', 'path_index', 'base_follower', 'arm_follower',
        'transformations', 'controllers', 'switch_arm_velocity', 'base_accuracy',
        'tcp_accuracy', 'accuracy_report', 'move_base', 'move_arm', 'rviz',
        'sync_workspace',
    )
    for action in command_actions:
        assert service.command_for(action), action
        service.action(action)

    for action in ('pose_adapters', 'capture_tool_offset', 'calculate_path_transform',
                   'start_following', 'stop_following', 'launch_all', 'stop_all'):
        service.action(action)

    assert service.processes.started
    assert ('path_index', 0) in service.ros_bridge.calls


def test_web_action_states_match_process_lifecycle(tmp_path: Path) -> None:
    service = make_service(tmp_path)

    initial = service.snapshot()['actions']
    assert initial['publish_path']['state'] == 'idle'
    assert initial['publish_path']['label'] == 'Publish Path'
    assert initial['launch_all']['state'] == 'idle'

    service.action('publish_path')
    running = service.snapshot()['actions']
    assert running['publish_path']['state'] == 'running'
    assert running['publish_path']['label'] == 'Stop Path'

    service.action('move_base')
    moving = service.snapshot()['actions']
    assert moving['move_base']['state'] == 'progress'
    assert moving['move_base']['label'] == 'Stop Base Move'


def test_launch_all_exposes_each_started_component_as_running(tmp_path: Path) -> None:
    service = make_service(tmp_path)
    service.update_config({'simulation': False})

    service.action('launch_all')
    states = service.snapshot()['actions']

    assert states['launch_all']['state'] == 'running'
    for action in ('publish_path', 'controllers', 'path_index', 'base_follower', 'arm_follower'):
        assert states[action]['state'] == 'running', action
    assert states['transformations']['state'] == 'running'


def test_ros_path_index_is_reflected_in_live_web_snapshot_without_persisting(tmp_path: Path) -> None:
    observed = []
    service = OperatorService(tmp_path / 'operator.json', path_index_callback=observed.append)
    service.ros_bridge = FakeRosBridge()

    service._on_path_index(42)
    snapshot = service.snapshot()

    assert snapshot['config']['path_index'] == 42
    assert snapshot['config']['original_arm_index'] == 21
    assert observed == [42]
    assert not (tmp_path / 'operator.json').exists()


def test_console_log_records_include_ros_level_and_time(tmp_path: Path) -> None:
    service = make_service(tmp_path)

    service.log('ros', '[WARN] Controller response is delayed')
    service.log('launch', '[ERROR] Child process exited')

    logs = service.snapshot()['logs']
    assert [item['level'] for item in logs] == ['warning', 'error']
    assert all(item['timestamp'].endswith('+00:00') for item in logs)


def test_following_resumes_from_live_path_index_after_stop(tmp_path: Path) -> None:
    service = make_service(tmp_path)
    service.ros_bridge = FakeRosBridge()
    service.update_config({'path_index': 0})
    service._on_path_index(37)

    service.action('stop_following')
    service.action('start_following')

    assert ('path_index', 37) in service.ros_bridge.calls
