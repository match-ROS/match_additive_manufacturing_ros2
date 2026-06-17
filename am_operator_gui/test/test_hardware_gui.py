from pathlib import Path
from types import SimpleNamespace

from am_operator_gui.gui import (
    ARM_FOLLOWER_NAME,
    BASE_FOLLOWER_NAME,
    DEFAULT_PID_GAINS,
    OperatorWindow,
    PATH_INDEX_NAME,
    SYNC_REMOTE_TARGET,
    SYNC_WORKSPACE_NAME,
    WORKSPACE_SRC_ROOT,
)


class FakeCheckBox:

    def __init__(self, checked: bool) -> None:
        self._checked = checked

    def isChecked(self) -> bool:
        return self._checked


class FakeLineEdit:

    def __init__(self, text: str) -> None:
        self._text = text

    def text(self) -> str:
        return self._text


class FakeComboBox:

    def __init__(self, text: str = '', data: str = '') -> None:
        self._text = text
        self._data = data

    def currentText(self) -> str:
        return self._text

    def currentData(self) -> str:
        return self._data


class FakeSpinBox:

    def __init__(self, value: int) -> None:
        self._value = value

    def value(self) -> int:
        return self._value


class FakeProcess:

    def is_running(self) -> bool:
        return True


class FakeProcesses:

    def __init__(self, running=None) -> None:
        self.started = []
        self.running = set(running or (PATH_INDEX_NAME, BASE_FOLLOWER_NAME, ARM_FOLLOWER_NAME))

    def start(self, name, command):
        self.started.append((name, command))

    def get(self, name):
        return FakeProcess() if name in self.running else None


def test_hardware_launch_all_never_starts_move_to_start() -> None:
    calls = []
    fake = SimpleNamespace(
        _launch_all_active=False,
        simulation_checkbox=FakeCheckBox(False),
        control_frame=FakeLineEdit('robotnik_simple'),
        ros_bridge=SimpleNamespace(
            publish_start_condition=lambda value: calls.append(('start_condition', value)),
            publish_stop_commands=lambda frame: calls.append(('stop', frame)),
        ),
        _append_process_output=lambda *_args: None,
        _start_pose_adapters=lambda: calls.append('pose_adapters'),
        _start_publish_path=lambda: calls.append('publish_path'),
        _start_arm_controllers=lambda: calls.append('arm_controllers'),
        _start_path_index=lambda: calls.append('path_index'),
        _start_base_follower=lambda: calls.append('base_follower'),
        _start_arm_follower=lambda **kwargs: calls.append(('arm_follower', kwargs)),
        _start_sim=lambda: calls.append('sim'),
        _start_current_tcp_pose=lambda: calls.append('tcp_from_tf'),
        _start_move_arm_to_start=lambda **kwargs: calls.append(('move_arm', kwargs)),
        _start_move_base_to_start=lambda **kwargs: calls.append(('move_base', kwargs)),
        _schedule_launch_all_action=lambda *_args: calls.append('scheduled_move'),
    )

    OperatorWindow._start_launch_all_components(fake)

    assert 'pose_adapters' in calls
    assert 'sim' not in calls
    assert not any(
        call == 'scheduled_move'
        or (isinstance(call, tuple) and call[0] in {'move_arm', 'move_base'})
        for call in calls
    )
    assert ('start_condition', False) in calls
    assert ('start_condition', True) not in calls
    assert ('stop', 'robotnik_simple') in calls


def test_hardware_arm_stack_uses_jparse_and_forward_velocity_controller() -> None:
    processes = FakeProcesses()
    fake = SimpleNamespace(
        simulation_checkbox=FakeCheckBox(False),
        control_frame=FakeLineEdit('robotnik_simple'),
        processes=processes,
        _use_sim_time=lambda: 'false',
        _arm_velocity_command_topic=lambda: '/robot/arm/forward_velocity_controller/commands',
        _arm_controller_manager=lambda: '/robot/arm/controller_manager',
        _append_process_output=lambda *_args: None,
    )

    OperatorWindow._start_arm_controllers(fake)

    command = processes.started[0][1]
    assert 'use_sim_time:=false' in command
    assert 'controller_manager:=/robot/arm/controller_manager' in command
    assert 'activate_controller:=forward_velocity_controller' in command
    assert (
        'velocity_command_topic:=/robot/arm/forward_velocity_controller/commands'
        in command
    )
    assert 'source_twist_topic:=/jparse_velocity_controller_ur/twist_cmd_world' in command
    assert 'controller_twist_topic:=/jparse_velocity_controller_ur/twist_cmd' in command
    joint_argument = next(
        item for item in command if item.startswith('command_joint_names_csv:=')
    )
    assert not joint_argument.endswith(']')
    assert joint_argument.count(',') == 5


def test_simulation_arm_stack_keeps_sim_controller_names() -> None:
    processes = FakeProcesses()
    fake = SimpleNamespace(
        simulation_checkbox=FakeCheckBox(True),
        control_frame=FakeLineEdit('robotnik_simple'),
        processes=processes,
        _use_sim_time=lambda: 'true',
        _arm_velocity_command_topic=lambda: '/robot/arm_forward_velocity_controller/commands',
        _arm_controller_manager=lambda: '/robot/controller_manager',
        _append_process_output=lambda *_args: None,
    )

    OperatorWindow._start_arm_controllers(fake)

    command = processes.started[0][1]
    assert 'use_sim_time:=true' in command
    assert 'controller_manager:=/robot/controller_manager' in command
    assert 'activate_controller:=arm_forward_velocity_controller' in command
    assert (
        'velocity_command_topic:=/robot/arm_forward_velocity_controller/commands'
        in command
    )


def test_base_follower_uses_configured_pid_gains() -> None:
    processes = FakeProcesses()
    fake = SimpleNamespace(
        processes=processes,
        platform_combo=FakeComboBox(data='robotnik'),
        diff_drive_checkbox=FakeCheckBox(False),
        follower_type_combo=FakeComboBox(data='pid'),
        _use_sim_time=lambda: 'false',
        _default_velocity_param=lambda: -1.0,
        _pid_gain=lambda key: {
            'base_follower.kp_x': 1.1,
            'base_follower.kp_y': 1.2,
            'base_follower.kp_yaw': 1.3,
        }.get(key, DEFAULT_PID_GAINS[key]),
        _ros_float_literal=OperatorWindow._ros_float_literal,
        _append_process_output=lambda *_args: None,
    )
    fake._current_platform_key = lambda: OperatorWindow._current_platform_key(fake)
    fake._current_platform_profile = lambda: OperatorWindow._current_platform_profile(fake)
    fake._diff_drive_mode = lambda: OperatorWindow._diff_drive_mode(fake)
    fake._current_follower_type = lambda: OperatorWindow._current_follower_type(fake)

    OperatorWindow._start_base_follower(fake)

    command = processes.started[0][1]
    assert 'kp_x:=1.100000' in command
    assert 'kp_y:=1.200000' in command
    assert 'kp_yaw:=1.300000' in command


def test_hardware_pose_adapters_start_external_base_reference() -> None:
    processes = FakeProcesses()
    fake = SimpleNamespace(
        processes=processes,
        base_pose_topic=FakeLineEdit('/vicon/Base_RB/Base_RB'),
        arm_pose_topic=FakeLineEdit('/vicon/Tool_Flange/Tool_Flange'),
        control_frame=FakeLineEdit('map'),
        external_map_frame=FakeLineEdit('map'),
        robot_base_frame=FakeLineEdit('base_link'),
        robot_tree_root_frame=FakeLineEdit('odom'),
        _use_sim_time=lambda: 'false',
        _append_process_output=lambda *_args: None,
    )

    OperatorWindow._start_pose_adapters(fake)

    base_command = processes.started[0][1]
    arm_command = processes.started[1][1]
    assert 'external_base_reference' in base_command
    assert 'input_topic:=/vicon/Base_RB/Base_RB' in base_command
    assert 'output_topic:=/robot_pose' in base_command
    assert 'map_frame:=map' in base_command
    assert 'robot_base_frame:=base_link' in base_command
    assert 'robot_tree_root_frame:=odom' in base_command
    assert 'pose_stamped_adapter' in arm_command
    assert 'input_topic:=/vicon/Tool_Flange/Tool_Flange' in arm_command
    assert 'target_frame:=map' in arm_command


def test_arm_follower_uses_configured_pid_gains() -> None:
    processes = FakeProcesses()
    fake = SimpleNamespace(
        processes=processes,
        control_frame=FakeLineEdit('robotnik_simple'),
        direction_mode=FakeComboBox(text='goal_direction'),
        index_spin=FakeSpinBox(0),
        _use_sim_time=lambda: 'false',
        _arm_velocity_command_topic=lambda: '/robot/arm/forward_velocity_controller/commands',
        _arm_trajectory_topic=lambda: '/robot/arm/joint_trajectory_controller/joint_trajectory',
        _default_velocity_param=lambda: -1.0,
        _pid_gain=lambda key: {
            'arm_direction.kp_z': 2.1,
            'arm_direction.ki_z': 2.2,
            'arm_direction.kd_z': 2.3,
            'arm_direction.orthogonal_kp': 2.4,
            'arm_pid_twist.Kp_linear_x': 3.1,
            'arm_orientation.kp_orientation': 4.1,
            'arm_orientation.ki_orientation': 4.2,
            'arm_orientation.kd_orientation': 4.3,
        }.get(key, DEFAULT_PID_GAINS[key]),
        _pid_launch_arguments=lambda prefix, names: OperatorWindow._pid_launch_arguments(fake, prefix, names),
        _ros_float_literal=OperatorWindow._ros_float_literal,
        _append_process_output=lambda *_args: None,
    )

    OperatorWindow._start_arm_follower(fake, move_to_start_pose=False)

    command = processes.started[0][1]
    assert 'kp_z:=2.100000' in command
    assert 'ki_z:=2.200000' in command
    assert 'kd_z:=2.300000' in command
    assert 'orthogonal_kp:=2.400000' in command
    assert 'Kp_linear_x:=3.100000' in command
    assert 'kp_orientation:=4.100000' in command
    assert 'ki_orientation:=4.200000' in command
    assert 'kd_orientation:=4.300000' in command


def test_motion_readiness_requires_every_gate() -> None:
    fake = SimpleNamespace(
        _has_path=True,
        _has_robot_pose=True,
        _has_arm_pose=True,
        _jparse_ready=True,
        _controller_ready=True,
        processes=FakeProcesses(),
    )
    fake._control_processes_running = lambda: OperatorWindow._control_processes_running(fake)

    assert OperatorWindow._motion_ready(fake)
    for attribute in (
        '_has_path',
        '_has_robot_pose',
        '_has_arm_pose',
        '_jparse_ready',
        '_controller_ready',
    ):
        setattr(fake, attribute, False)
        assert not OperatorWindow._motion_ready(fake)
        setattr(fake, attribute, True)

    for process_name in (PATH_INDEX_NAME, BASE_FOLLOWER_NAME, ARM_FOLLOWER_NAME):
        fake.processes.running.remove(process_name)
        assert not OperatorWindow._motion_ready(fake)
        fake.processes.running.add(process_name)


def test_control_frame_defaults_from_path_json(tmp_path: Path) -> None:
    (tmp_path / 'base_path.json').write_text(
        '{"frame_id": "vicon_world", "poses": []}',
        encoding='utf-8',
    )
    assert OperatorWindow._path_frame_from_folder(tmp_path) == 'vicon_world'


def test_sync_workspace_uses_rsync_to_remote_src() -> None:
    processes = FakeProcesses(running=())
    fake = SimpleNamespace(
        processes=processes,
        _append_process_output=lambda *_args: None,
    )

    OperatorWindow._start_sync_workspace(fake)

    assert processes.started == [(
        SYNC_WORKSPACE_NAME,
        [
            'rsync',
            '-az',
            '-e',
            'ssh',
            f'{WORKSPACE_SRC_ROOT}/',
            SYNC_REMOTE_TARGET,
        ],
    )]
