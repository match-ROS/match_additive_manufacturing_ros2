from pathlib import Path
from types import SimpleNamespace

import pytest

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

    def tracking_arm_index_for_original_index(self, index):
        return index * 2

    def publish_velocity_override(self, value):
        self.calls.append(('velocity_override', value))

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


def test_one_shot_moves_share_the_tracking_index_space(tmp_path: Path) -> None:
    service = make_service(tmp_path)
    service.update_config({'path_index': 100, 'original_arm_index': 28})

    base_command = service.command_for('move_base')
    arm_command = service.command_for('move_arm')

    assert 'path_topic:=/base_path_tracking' in base_command
    assert 'path_index:=100' in base_command
    assert 'path_topic:=/ur_path_tracking' in arm_command
    assert 'path_index:=100' in arm_command
    assert 'path_index:=28' not in arm_command


def test_original_arm_index_selects_its_matching_tracking_index_for_web_gui(tmp_path: Path) -> None:
    service = make_service(tmp_path)
    service.ros_bridge = FakeRosBridge()

    service.update_config({'original_arm_index': 28})

    assert service.config['original_arm_index'] == 28
    assert service.config['path_index'] == 56
    assert ('path_index', 56) in service.ros_bridge.calls


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


def test_platform_tuning_overrides_global_values_and_reaches_each_controller(tmp_path: Path) -> None:
    service = make_service(tmp_path)
    service.update_config({
        'platform': 'robotnik',
        'trajectory_directory': str(tmp_path / 'trajectory'),
        'pid_gains': {'base_follower.max_vx': 0.1},
    })
    settings = service.update_platform_settings('robotnik', {
        'pid_gains': {
            'base_follower.max_vx': 0.42,
            'base_follower.max_vy': 0.31,
            'base_follower.max_wz': 0.73,
            'arm_direction.max_tracking_linear_velocity': 0.18,
            'arm_move.max_linear_velocity': 0.16,
        },
        'base_smoothing': {
            'enabled': True, 'method': 'accel_limit', 'max_accel_x': 0.7,
            'max_accel_y': 0.6, 'max_accel_wz': 1.1,
            'moving_average_window_size': 7, 'external_path_index_stride': 3,
        },
        'jparse_limits': {
            'max_joint_velocity': 1.2,
            'max_cartesian_linear_velocity': 0.33,
            'max_cartesian_angular_velocity': 0.66,
        },
        'path_transform': {'x': 1.0, 'y': -2.0, 'z': 0.3, 'yaw_deg': 45.0},
    })

    assert settings['pid_gains']['base_follower.max_vx'] == 0.42
    assert settings['path_transform']['yaw_deg'] == 45.0
    assert 'max_vx:=0.420000' in service.command_for('base_follower')
    assert 'max_vy:=0.310000' in service.command_for('base_follower')
    assert 'max_wz:=0.730000' in service.command_for('base_follower')
    assert 'velocity_smoothing_method:=accel_limit' in service.command_for('base_follower')
    assert 'max_tracking_linear_velocity:=0.180000' in service.command_for('arm_follower')
    assert 'max_linear_velocity:=0.160000' in service.command_for('move_arm')
    assert 'jparse_max_joint_velocity:=1.200000' in service.command_for('controllers')
    assert 'jparse_max_cartesian_linear_velocity:=0.330000' in service.command_for('controllers')
    assert 'jparse_max_cartesian_angular_velocity:=0.660000' in service.command_for('controllers')
    assert 'path_transform_xyz:=[1.000000, -2.000000, 0.300000]' in service.command_for('publish_path')


def test_platform_tuning_keeps_path_transforms_separate_per_platform(tmp_path: Path) -> None:
    service = make_service(tmp_path)
    service.update_config({'platform': 'robotnik', 'trajectory_directory': str(tmp_path / 'trajectory')})
    service.update_platform_settings('robotnik', {'path_transform': {'x': 1, 'y': 0, 'z': 0, 'yaw_deg': 0}})
    service.update_platform_settings('bunker', {'path_transform': {'x': 2, 'y': 0, 'z': 0, 'yaw_deg': 0}})

    assert 'path_transform_xyz:=[1.000000, 0.000000, 0.000000]' in service.command_for('publish_path')
    service.update_config({'platform': 'bunker'})
    assert 'path_transform_xyz:=[2.000000, 0.000000, 0.000000]' in service.command_for('publish_path')


def test_platform_tuning_rejects_invalid_values_without_persisting(tmp_path: Path) -> None:
    service = make_service(tmp_path)

    with pytest.raises(ValueError, match='max_vx'):
        service.update_platform_settings('robotnik', {'pid_gains': {'base_follower.max_vx': -0.1}})
    with pytest.raises(ValueError, match='method'):
        service.update_platform_settings('robotnik', {'base_smoothing': {'method': 'invalid'}})
    with pytest.raises(ValueError, match='Unknown platform'):
        service.update_platform_settings('mur620_sim', {})
    assert 'platform_control_settings' not in service.config


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
    service.update_config({'path_index': 0, 'velocity_override': 50})
    service.ros_bridge.calls.clear()
    service._on_path_index(37)

    service.action('stop_following')
    service.action('start_following')

    assert ('path_index', 37) in service.ros_bridge.calls
    assert ('velocity_override', 0.5) in service.ros_bridge.calls


def test_web_simulation_window_setting_is_forwarded_for_robotnik_and_bunker(tmp_path: Path) -> None:
    service = make_service(tmp_path)

    service.update_config({'platform': 'robotnik', 'simulation_gui': True})
    assert 'gui:=true' in service.command_for('simulation')

    service.update_config({'platform': 'bunker', 'simulation_gui': True})
    assert 'headless:=false' in service.command_for('simulation')

    service.update_config({'simulation_gui': False})
    assert 'headless:=true' in service.command_for('simulation')


def test_web_tool_offset_is_platform_scoped_and_used_for_arm_launch(tmp_path: Path) -> None:
    service = make_service(tmp_path)
    service.update_config({
        'platform': 'robotnik',
        'fixed_tool_offset': {
            'xyz': [-0.25, 0.0, 0.015],
            'quaternion_xyzw': [0.0, -0.7071067812, 0.0, 0.7071067812],
        },
        'fixed_tool_offsets_by_platform': {
            'robotnik': {
                'xyz': [0.1, 0.2, 0.3],
                'quaternion_xyzw': [0.0, 0.0, 0.0, 1.0],
            },
            'bunker': {
                'xyz': [0.4, 0.5, 0.6],
                'quaternion_xyzw': [0.0, 1.0, 0.0, 0.0],
            },
        },
        'fixed_tool_offset_input_mode': 'rpy',
    })

    robotnik_command = service.command_for('arm_follower')
    assert 'fixed_tool_offset_xyz:=[0.100000, 0.200000, 0.300000]' in robotnik_command
    assert 'fixed_tool_offset_quaternion_xyzw:=[0.000000, 0.000000, 0.000000, 1.000000]' in robotnik_command
    assert service.snapshot()['config']['fixed_tool_offset_input_mode'] == 'rpy'

    service.update_config({'platform': 'bunker'})
    bunker_command = service.command_for('arm_follower')
    assert 'fixed_tool_offset_xyz:=[0.400000, 0.500000, 0.600000]' in bunker_command


def test_capture_tool_offset_updates_the_selected_platform_offset(tmp_path: Path) -> None:
    service = make_service(tmp_path)
    service.update_config({'platform': 'bunker'})
    bridge = FakeRosBridge()
    bridge.lookup_tool_offset = lambda *_args: SimpleNamespace(
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=0.01, y=-0.02, z=0.03),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        )
    )
    service.ros_bridge = bridge

    service.action('capture_tool_offset')

    offset = service.snapshot()['config']['fixed_tool_offsets_by_platform']['bunker']
    assert offset == {'xyz': [0.01, -0.02, 0.03], 'quaternion_xyzw': [0.0, 0.0, 0.0, 1.0]}
    assert 'fixed_tool_offset_xyz:=[0.010000, -0.020000, 0.030000]' in service.command_for('arm_follower')
