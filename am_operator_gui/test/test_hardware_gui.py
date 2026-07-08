from pathlib import Path
import math
from types import SimpleNamespace

from am_operator_gui.gui import (
    ARM_FOLLOWER_NAME,
    BASE_FOLLOWER_NAME,
    CURRENT_TCP_POSE_NAME,
    DEFAULT_TRAJECTORY_DIR,
    DEFAULT_PID_GAINS,
    MOVE_BASE_NAME,
    ODOMETRY_POSE_ADAPTER_NAME,
    OperatorWindow,
    PATH_INDEX_NAME,
    SIM_NAME,
    SYNC_REMOTE_TARGET,
    SYNC_WORKSPACE_NAME,
    VICON_BASE_REFERENCE_FRAME,
    VICON_BASE_STATIC_TF,
    VICON_BASE_STATIC_TF_NAME,
    VICON_EE_STATIC_TF_NAME,
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

    def setValue(self, value) -> None:
        self._value = value

    def blockSignals(self, _blocked: bool) -> None:
        pass


class FakeProcess:

    def is_running(self) -> bool:
        return True


class FakeProcesses:

    def __init__(self, running=None) -> None:
        self.started = []
        self.stopped = []
        self.running = set(running or (PATH_INDEX_NAME, BASE_FOLLOWER_NAME, ARM_FOLLOWER_NAME))

    def start(self, name, command):
        self.started.append((name, command))

    def get(self, name):
        return FakeProcess() if name in self.running else None

    def stop(self, name):
        self.stopped.append(name)
        self.running.discard(name)


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

def test_robotnik_sim_launch_uses_real_robot_arm_type() -> None:
    processes = FakeProcesses(running=())
    fake = SimpleNamespace(
        simulation_checkbox=FakeCheckBox(True),
        odometry_pose_checkbox=FakeCheckBox(False),
        processes=processes,
        _current_platform_key=lambda: 'robotnik',
        _append_process_output=lambda *_args: None,
    )
    fake._sim_publish_robot_pose = lambda: OperatorWindow._sim_publish_robot_pose(fake)

    OperatorWindow._start_sim(fake)

    assert processes.started == [(
        'launch_sim',
        [
            'ros2',
            'launch',
            'rbvogui_ur_sim_setup',
            'rbvogui_ur_standard_control.launch.py',
            'gui:=true',
            'robot_id:=robot',
            'arm_type:=ur20',
            'publish_robot_pose:=true',
        ],
    )]


def test_sim_launch_disables_builtin_robot_pose_when_odometry_pose_is_used() -> None:
    processes = FakeProcesses()
    fake = SimpleNamespace(
        processes=processes,
        simulation_checkbox=FakeCheckBox(True),
        odometry_pose_checkbox=FakeCheckBox(True),
        platform_combo=FakeComboBox(data='robotnik'),
        _append_process_output=lambda *_args: None,
    )
    fake._current_platform_key = lambda: OperatorWindow._current_platform_key(fake)
    fake._sim_publish_robot_pose = lambda: OperatorWindow._sim_publish_robot_pose(fake)

    OperatorWindow._start_sim(fake)

    name, command = processes.started[0]
    assert name == SIM_NAME
    assert 'rbvogui_ur_standard_control.launch.py' in command
    assert 'publish_robot_pose:=false' in command


def test_sim_transformations_start_odometry_pose_when_checked() -> None:
    calls = []
    fake = SimpleNamespace(
        processes=FakeProcesses(running=()),
        simulation_checkbox=FakeCheckBox(True),
        odometry_pose_checkbox=FakeCheckBox(True),
        _start_current_tcp_pose=lambda: calls.append(CURRENT_TCP_POSE_NAME),
        _start_odometry_pose_adapter=lambda: calls.append(ODOMETRY_POSE_ADAPTER_NAME),
        _refresh_process_states=lambda: calls.append('refresh'),
        _append_process_output=lambda *_args: None,
    )

    OperatorWindow._toggle_current_tcp_pose(fake)

    assert CURRENT_TCP_POSE_NAME in calls
    assert ODOMETRY_POSE_ADAPTER_NAME in calls


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
        _base_smoothing=lambda key: {
            'enabled': False,
            'method': 'moving_average',
            'max_accel_x': 0.11,
            'max_accel_y': 0.12,
            'max_accel_wz': 0.13,
            'moving_average_window_size': 7,
            'external_path_index_stride': 10,
        }[key],
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
    assert 'smooth_velocity_commands:=false' in command
    assert 'velocity_smoothing_method:=moving_average' in command
    assert 'max_accel_x:=0.110000' in command
    assert 'max_accel_y:=0.120000' in command
    assert 'max_accel_wz:=0.130000' in command
    assert 'moving_average_window_size:=7' in command
    assert 'external_path_index_stride:=10' in command


def test_move_base_to_start_uses_configured_velocity_limits() -> None:
    processes = FakeProcesses()
    fake = SimpleNamespace(
        processes=processes,
        platform_combo=FakeComboBox(data='robotnik'),
        diff_drive_checkbox=FakeCheckBox(False),
        index_spin=FakeSpinBox(4),
        base_move_linear_velocity_spin=FakeSpinBox(0.12),
        base_move_lateral_velocity_spin=FakeSpinBox(0.07),
        base_move_angular_velocity_spin=FakeSpinBox(0.34),
        _use_sim_time=lambda: 'false',
        _pid_gain=lambda key: {
            'base_move.kp_linear': 0.6,
            'base_move.kp_lateral': 0.6,
            'base_move.kp_angular_to_point': 1.5,
            'base_move.kp_angular_reorient': 1.2,
        }[key],
        _ros_float_literal=OperatorWindow._ros_float_literal,
        _append_process_output=lambda *_args: None,
    )
    fake._current_platform_key = lambda: OperatorWindow._current_platform_key(fake)
    fake._current_platform_profile = lambda: OperatorWindow._current_platform_profile(fake)
    fake._diff_drive_mode = lambda: OperatorWindow._diff_drive_mode(fake)
    fake._base_move_velocity = lambda key: OperatorWindow._base_move_velocity(fake, key)

    OperatorWindow._start_move_base_to_start(fake)

    name, command = processes.started[0]
    assert name == MOVE_BASE_NAME
    assert 'path_index:=4' in command
    assert 'max_linear_velocity:=0.120000' in command
    assert 'max_lateral_velocity:=0.070000' in command
    assert 'max_angular_velocity:=0.340000' in command


def test_hardware_pose_adapters_start_external_base_reference() -> None:
    processes = FakeProcesses()
    fake = SimpleNamespace(
        processes=processes,
        base_pose_topic=FakeLineEdit('/vicon/Base_RB/Base_RB'),
        arm_pose_topic=FakeLineEdit('/vicon/tool_transformed'),
        odometry_pose_checkbox=FakeCheckBox(False),
        control_frame=FakeLineEdit('map'),
        external_map_frame=FakeLineEdit('map'),
        robot_base_frame=FakeLineEdit('base_link'),
        robot_tree_root_frame=FakeLineEdit('odom'),
        _use_sim_time=lambda: 'false',
        _append_process_output=lambda *_args: None,
    )

    OperatorWindow._start_pose_adapters(fake)

    static_command = processes.started[0][1]
    vicon_command = processes.started[1][1]
    base_command = processes.started[2][1]
    arm_command = processes.started[3][1]
    assert processes.started[0][0] == VICON_BASE_STATIC_TF_NAME
    assert static_command == [
        'ros2',
        'run',
        'tf2_ros',
        'static_transform_publisher',
        *VICON_BASE_STATIC_TF,
    ]
    assert static_command[-2:] == ['robot_base_footprint', VICON_BASE_REFERENCE_FRAME]
    assert processes.started[1][0] == VICON_EE_STATIC_TF_NAME
    assert 'vicon_ee_static_tf' in vicon_command
    assert 'input_topic:=/vicon/Tool_Flange/Tool_Flange' in vicon_command
    assert 'output_topic:=/vicon/tool_transformed' in vicon_command
    assert 'external_base_reference' in base_command
    assert 'input_topic:=/vicon/Base_RB/Base_RB' in base_command
    assert f'input_pose_frame:={VICON_BASE_REFERENCE_FRAME}' in base_command
    assert 'output_topic:=/robot_pose' in base_command
    assert 'map_frame:=map' in base_command
    assert 'robot_base_frame:=base_link' in base_command
    assert 'robot_tree_root_frame:=odom' in base_command
    assert 'pose_stamped_adapter' in arm_command
    assert 'input_topic:=/vicon/tool_transformed' in arm_command
    assert 'target_frame:=map' in arm_command


def test_hardware_pose_adapters_can_use_odometry_for_robot_pose() -> None:
    processes = FakeProcesses()
    fake = SimpleNamespace(
        processes=processes,
        base_pose_topic=FakeLineEdit('/vicon/Base_RB/Base_RB'),
        arm_pose_topic=FakeLineEdit('/vicon/tool_transformed'),
        odometry_pose_checkbox=FakeCheckBox(True),
        control_frame=FakeLineEdit('map'),
        external_map_frame=FakeLineEdit('map'),
        robot_base_frame=FakeLineEdit('base_link'),
        robot_tree_root_frame=FakeLineEdit('odom'),
        platform_combo=FakeComboBox(data='robotnik'),
        index_spin=FakeSpinBox(7),
        _use_sim_time=lambda: 'false',
        _append_process_output=lambda *_args: None,
    )
    fake._current_platform_key = lambda: OperatorWindow._current_platform_key(fake)
    fake._current_platform_profile = lambda: OperatorWindow._current_platform_profile(fake)
    fake._start_odometry_pose_adapter = lambda: OperatorWindow._start_odometry_pose_adapter(fake)

    OperatorWindow._start_pose_adapters(fake)

    started_names = [name for name, _command in processes.started]
    assert VICON_BASE_STATIC_TF_NAME not in started_names
    assert ODOMETRY_POSE_ADAPTER_NAME in started_names
    odom_command = next(
        command for name, command in processes.started
        if name == ODOMETRY_POSE_ADAPTER_NAME
    )
    assert 'odometry_robot_pose' in odom_command
    assert 'odom_topic:=/robot/robotnik_base_control/odom' in odom_command
    assert 'path_topic:=/base_path' in odom_command
    assert 'output_topic:=/robot_pose' in odom_command
    assert 'initial_path_index:=7' in odom_command
    assert 'map_frame:=map' in odom_command
    assert 'odom_frame:=odom' in odom_command
    assert 'robot_base_frame:=base_link' in odom_command


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
        assert OperatorWindow._motion_ready(fake)
        fake.processes.running.add(process_name)


def test_start_following_warns_but_publishes_when_follower_missing() -> None:
    calls = []
    processes = FakeProcesses(running=(PATH_INDEX_NAME, BASE_FOLLOWER_NAME))
    fake = SimpleNamespace(
        _has_path=True,
        _has_robot_pose=True,
        _has_arm_pose=True,
        _jparse_ready=True,
        _controller_ready=True,
        processes=processes,
        index_spin=FakeSpinBox(12),
        ros_bridge=SimpleNamespace(
            publish_path_index=lambda value: calls.append(('path_index', value)),
        ),
        start_following_button=object(),
        _append_process_output=lambda *args: calls.append(args),
        _publish_start_condition_once=lambda value: calls.append(('start_condition', value)),
        _style_button=lambda *_args: None,
    )
    fake._motion_ready = lambda: OperatorWindow._motion_ready(fake)
    fake._motion_not_ready_reason = lambda: OperatorWindow._motion_not_ready_reason(fake)
    fake._missing_control_process_names = lambda: OperatorWindow._missing_control_process_names(fake)

    OperatorWindow._start_following(fake)

    assert ('path_index', 12) in calls
    assert ('start_condition', True) in calls
    assert any(
        call[0] == 'safety' and ARM_FOLLOWER_NAME in call[1]
        for call in calls
        if isinstance(call, tuple) and len(call) == 2
    )


def test_path_index_advancer_uses_base_path_topic() -> None:
    processes = FakeProcesses()
    fake = SimpleNamespace(
        processes=processes,
        platform_combo=FakeComboBox(data='robotnik'),
        index_spin=FakeSpinBox(12),
        path_index_rate_spin=FakeSpinBox(5.0),
        _use_sim_time=lambda: 'false',
        _ros_float_literal=OperatorWindow._ros_float_literal,
        _append_process_output=lambda *_args: None,
    )
    fake._current_platform_key = lambda: OperatorWindow._current_platform_key(fake)
    fake._current_platform_profile = lambda: OperatorWindow._current_platform_profile(fake)

    OperatorWindow._start_path_index(fake)

    command = processes.started[0][1]
    assert 'path_topic:=/base_path' in command
    assert 'path_topic:=/ur_path_transformed' not in command


def test_control_processes_running_reports_missing_followers() -> None:
    fake = SimpleNamespace(processes=FakeProcesses())
    fake._missing_control_process_names = lambda: OperatorWindow._missing_control_process_names(fake)
    for process_name in (PATH_INDEX_NAME, BASE_FOLLOWER_NAME, ARM_FOLLOWER_NAME):
        fake.processes.running.remove(process_name)
        assert not OperatorWindow._control_processes_running(fake)
        assert process_name in OperatorWindow._missing_control_process_names(fake)
        fake.processes.running.add(process_name)


def test_control_frame_defaults_from_path_json(tmp_path: Path) -> None:
    (tmp_path / 'base_path.json').write_text(
        '{"frame_id": "vicon_world", "poses": []}',
        encoding='utf-8',
    )
    assert OperatorWindow._path_frame_from_folder(tmp_path) == 'vicon_world'


def test_trajectory_directory_defaults_and_saves_with_hardware_topics(tmp_path: Path) -> None:
    default_directory = OperatorWindow._configured_trajectory_directory(
        SimpleNamespace(_config={})
    )
    assert default_directory == DEFAULT_TRAJECTORY_DIR

    fake = SimpleNamespace(
        _config={'trajectory_directory': str(tmp_path / 'previous')},
        path_folder=FakeLineEdit(str(tmp_path / 'selected')),
        base_pose_topic=FakeLineEdit('/base_pose'),
        arm_pose_topic=FakeLineEdit('/arm_pose'),
        control_frame=FakeLineEdit('map'),
        external_map_frame=FakeLineEdit('world'),
        robot_base_frame=FakeLineEdit('base_footprint'),
        robot_tree_root_frame=FakeLineEdit('odom'),
        _save_config=lambda: None,
    )

    assert OperatorWindow._configured_trajectory_directory(fake) == tmp_path / 'previous'

    OperatorWindow._save_hardware_topics(fake)

    assert fake._config['trajectory_directory'] == str(tmp_path / 'selected')


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


def _pose(x: float, y: float, z: float, yaw_rad: float):
    return SimpleNamespace(
        pose=SimpleNamespace(
            position=SimpleNamespace(x=x, y=y, z=z),
            orientation=SimpleNamespace(
                x=0.0,
                y=0.0,
                z=math.sin(yaw_rad / 2.0),
                w=math.cos(yaw_rad / 2.0),
            ),
        )
    )


def test_composed_path_transform_aligns_path_index_to_robot_pose() -> None:
    path_pose = _pose(1.0, 0.0, 0.2, 0.0)
    robot_pose = _pose(2.0, 3.0, 0.5, math.pi / 2.0)

    x, y, z, yaw_deg = OperatorWindow._composed_path_transform(
        {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw_deg': 0.0},
        path_pose,
        robot_pose,
    )

    assert round(x, 6) == 2.0
    assert round(y, 6) == 2.0
    assert round(z, 6) == 0.3
    assert round(yaw_deg, 6) == 90.0


def test_calculate_path_transform_updates_fields_and_restarts_publisher() -> None:
    outputs = []
    processes = FakeProcesses(running=('publish_path',))
    fake = SimpleNamespace(
        index_spin=FakeSpinBox(4),
        path_transform_x_spin=FakeSpinBox(0.0),
        path_transform_y_spin=FakeSpinBox(0.0),
        path_transform_z_spin=FakeSpinBox(0.0),
        path_transform_yaw_spin=FakeSpinBox(0.0),
        ros_bridge=SimpleNamespace(
            latest_base_path_pose=lambda index: _pose(1.0, 0.0, 0.0, 0.0) if index == 4 else None,
            latest_robot_pose=lambda: _pose(3.0, 0.0, 0.0, 0.0),
        ),
        processes=processes,
        _append_process_output=lambda *args: outputs.append(args),
        _set_path_transform=lambda: outputs.append(('config', 'saved')),
        _start_publish_path=lambda: processes.start('publish_path', ['republished']),
        _refresh_process_states=lambda: None,
    )
    fake._composed_path_transform = OperatorWindow._composed_path_transform
    fake._set_path_transform_values = (
        lambda x, y, z, yaw: OperatorWindow._set_path_transform_values(fake, x, y, z, yaw)
    )

    OperatorWindow._calculate_path_transform(fake)

    assert fake.path_transform_x_spin.value() == 2.0
    assert fake.path_transform_y_spin.value() == 0.0
    assert fake.path_transform_z_spin.value() == 0.0
    assert fake.path_transform_yaw_spin.value() == 0.0
    assert processes.stopped == ['publish_path']
    assert processes.started[-1] == ('publish_path', ['republished'])
    assert any(call[0] == 'gui' and 'calculated path transform' in call[1] for call in outputs)
