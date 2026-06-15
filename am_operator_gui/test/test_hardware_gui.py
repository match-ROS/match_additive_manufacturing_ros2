from pathlib import Path
from types import SimpleNamespace

from am_operator_gui.gui import (
    ARM_FOLLOWER_NAME,
    BASE_FOLLOWER_NAME,
    OperatorWindow,
    PATH_INDEX_NAME,
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
