import signal
import sys
from pathlib import Path
from typing import Callable, Optional

from ament_index_python.packages import get_package_share_directory
from PyQt5.QtCore import QTimer, Qt, pyqtSignal
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QFileDialog,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QPushButton,
    QPlainTextEdit,
    QSlider,
    QSpinBox,
    QDoubleSpinBox,
    QVBoxLayout,
    QWidget,
)

from am_operator_gui.process_manager import ProcessRegistry
from am_operator_gui.ros_bridge import RosBridge


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_COMPONENTS_DIR = REPO_ROOT / 'components'
DEFAULT_TRAJECTORY_DIR = DEFAULT_COMPONENTS_DIR / 'robotnik_paired_demo'
LAUNCH_ALL_NAME = 'launch_all'
SIM_NAME = 'launch_sim'
PUBLISH_PATH_NAME = 'publish_path'
BASE_FOLLOWER_NAME = 'base_follower'
ARM_FOLLOWER_NAME = 'arm_follower'
PATH_INDEX_NAME = 'path_index'
MOVE_BASE_NAME = 'move_base_to_start'
MOVE_ARM_NAME = 'move_arm_to_start'
SWITCH_ARM_VELOCITY_NAME = 'switch_arm_velocity_controller'
RVIZ_NAME = 'rviz'


class OperatorWindow(QMainWindow):
    ros_status_changed = pyqtSignal(bool, bool)
    path_index_changed = pyqtSignal(int)
    process_output = pyqtSignal(str, str)

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle('AM Operator GUI')
        self.resize(980, 720)
        self._has_path = False
        self._has_robot_pose = False
        self._launch_all_active = False
        self._launch_all_timers: list[QTimer] = []

        self.processes = ProcessRegistry(output_callback=self._on_process_output)
        self.ros_bridge = RosBridge(
            status_callback=self._on_ros_status,
            path_index_callback=self._on_path_index,
        )

        self.ros_status_changed.connect(self._set_ros_status)
        self.path_index_changed.connect(self._set_path_index_from_ros)
        self.process_output.connect(self._append_process_output)

        self._build_ui()
        self._connect_signals()
        self.ros_bridge.start()

        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self._refresh_process_states)
        self.status_timer.start(500)

        self._publish_overrides()
        self._refresh_process_states()

    def _build_ui(self) -> None:
        root = QWidget()
        layout = QVBoxLayout(root)
        layout.setSpacing(10)

        launch_group = QGroupBox('System')
        launch_layout = QGridLayout(launch_group)
        self.simulation_checkbox = QCheckBox('Simulation')
        self.direction_mode = QComboBox()
        self.direction_mode.addItems(['goal_direction', 'speed_orthogonal'])
        self.direction_mode.setCurrentText('goal_direction')
        self.index_spin = QSpinBox()
        self.index_spin.setRange(0, 100000)
        self.index_spin.setValue(0)

        self.path_folder = QLineEdit(str(DEFAULT_TRAJECTORY_DIR))
        self.path_folder.setReadOnly(True)
        self.browse_button = QPushButton('Browse')

        self.launch_button = QPushButton('Launch All')
        self.launch_sim_button = QPushButton('Launch Sim')
        self.publish_path_button = QPushButton('Publish Path')
        self.rviz_button = QPushButton('Open RViz')

        launch_layout.addWidget(self.simulation_checkbox, 0, 0)
        launch_layout.addWidget(QLabel('Direction'), 0, 1)
        launch_layout.addWidget(self.direction_mode, 0, 2)
        launch_layout.addWidget(QLabel('Current index'), 0, 3)
        launch_layout.addWidget(self.index_spin, 0, 4)
        launch_layout.addWidget(QLabel('Path folder'), 1, 0)
        launch_layout.addWidget(self.path_folder, 1, 1, 1, 3)
        launch_layout.addWidget(self.browse_button, 1, 4)
        launch_layout.addWidget(self.launch_button, 2, 0)
        launch_layout.addWidget(self.launch_sim_button, 2, 1)
        launch_layout.addWidget(self.rviz_button, 2, 2)

        component_group = QGroupBox('Components')
        component_layout = QGridLayout(component_group)
        self.base_follower_button = QPushButton('Launch Base Follower')
        self.arm_follower_button = QPushButton('Launch Arm Follower')
        self.path_index_button = QPushButton('Launch Path Index')
        self.switch_arm_velocity_button = QPushButton('Switch Arm Velocity')

        component_layout.addWidget(self.publish_path_button, 0, 0)
        component_layout.addWidget(self.path_index_button, 0, 1)
        component_layout.addWidget(self.base_follower_button, 1, 0)
        component_layout.addWidget(self.arm_follower_button, 1, 1)
        component_layout.addWidget(self.switch_arm_velocity_button, 2, 0)

        motion_group = QGroupBox('Motion')
        motion_layout = QGridLayout(motion_group)
        self.move_base_button = QPushButton('Move Base To Start')
        self.move_arm_button = QPushButton('Move Arm To Start')
        self.start_following_button = QPushButton('Start Following')
        self.stop_following_button = QPushButton('Stop Following')

        motion_layout.addWidget(self.move_base_button, 0, 0)
        motion_layout.addWidget(self.move_arm_button, 0, 1)
        motion_layout.addWidget(self.start_following_button, 1, 0)
        motion_layout.addWidget(self.stop_following_button, 1, 1)

        override_group = QGroupBox('Overrides')
        override_layout = QGridLayout(override_group)
        self.velocity_slider = QSlider(Qt.Horizontal)
        self.velocity_slider.setRange(0, 200)
        self.velocity_slider.setValue(100)
        self.velocity_value = QLabel('100%')

        self.nozzle_reference = QDoubleSpinBox()
        self.nozzle_reference.setRange(-10000.0, 10000.0)
        self.nozzle_reference.setDecimals(1)
        self.nozzle_reference.setSuffix(' mm')
        self.nozzle_reference.setValue(0.0)
        self.nozzle_offset = QSlider(Qt.Horizontal)
        self.nozzle_offset.setRange(-100, 100)
        self.nozzle_offset.setValue(0)
        self.nozzle_offset_value = QLabel('+0 mm')
        self.nozzle_effective_value = QLabel('0.0 mm effective')

        override_layout.addWidget(QLabel('Velocity override'), 0, 0)
        override_layout.addWidget(self.velocity_slider, 0, 1)
        override_layout.addWidget(self.velocity_value, 0, 2)
        override_layout.addWidget(QLabel('Nozzle reference'), 1, 0)
        override_layout.addWidget(self.nozzle_reference, 1, 1)
        override_layout.addWidget(self.nozzle_effective_value, 1, 2)
        override_layout.addWidget(QLabel('Nozzle offset'), 2, 0)
        override_layout.addWidget(self.nozzle_offset, 2, 1)
        override_layout.addWidget(self.nozzle_offset_value, 2, 2)

        status_group = QGroupBox('Status')
        status_layout = QHBoxLayout(status_group)
        self.path_status = QLabel('/base_path: waiting')
        self.pose_status = QLabel('/robot_pose: waiting')
        status_layout.addWidget(self.path_status)
        status_layout.addWidget(self.pose_status)
        status_layout.addStretch(1)

        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(1000)

        layout.addWidget(launch_group)
        layout.addWidget(component_group)
        layout.addWidget(motion_group)
        layout.addWidget(override_group)
        layout.addWidget(status_group)
        layout.addWidget(self.log, 1)
        self.setCentralWidget(root)

    def _connect_signals(self) -> None:
        self.browse_button.clicked.connect(self._choose_path_folder)
        self.launch_button.clicked.connect(self._toggle_launch_all)
        self.launch_sim_button.clicked.connect(self._toggle_sim)
        self.publish_path_button.clicked.connect(self._publish_path)
        self.base_follower_button.clicked.connect(self._toggle_base_follower)
        self.arm_follower_button.clicked.connect(self._toggle_arm_follower)
        self.path_index_button.clicked.connect(self._toggle_path_index)
        self.switch_arm_velocity_button.clicked.connect(self._switch_arm_velocity_controller)
        self.move_base_button.clicked.connect(self._move_base_to_start)
        self.move_arm_button.clicked.connect(self._move_arm_to_start)
        self.start_following_button.clicked.connect(self._start_following)
        self.stop_following_button.clicked.connect(self._stop_following)
        self.rviz_button.clicked.connect(self._open_rviz)
        self.index_spin.valueChanged.connect(self._publish_path_index)
        self.velocity_slider.valueChanged.connect(self._publish_overrides)
        self.nozzle_reference.valueChanged.connect(self._publish_overrides)
        self.nozzle_offset.valueChanged.connect(self._publish_overrides)

    def _choose_path_folder(self) -> None:
        folder = QFileDialog.getExistingDirectory(
            self,
            'Select trajectory folder',
            str(DEFAULT_COMPONENTS_DIR),
        )
        if folder:
            self.path_folder.setText(folder)

    def _toggle_launch_all(self) -> None:
        if self._launch_all_active:
            self._stop_launch_all_components()
            return

        self._start_launch_all_components()
        self._refresh_process_states()

    def _start_launch_all_components(self) -> None:
        self._launch_all_active = True
        self._append_process_output(LAUNCH_ALL_NAME, 'starting managed component set')
        if self.simulation_checkbox.isChecked():
            self._start_sim()
        self._start_publish_path()
        self._start_move_arm_to_start(wait_for_start_condition=True)
        self._start_path_index()
        self._start_base_follower()
        self._start_arm_follower(move_to_start_pose=True)
        self._schedule_launch_all_action(13000, lambda: self._start_move_base_to_start(
            publish_start_condition=True,
        ))
        self._schedule_launch_all_action(13000, self._start_switch_arm_velocity_controller)

    def _stop_launch_all_components(self) -> None:
        for timer in self._launch_all_timers:
            timer.stop()
            timer.deleteLater()
        self._launch_all_timers.clear()
        for name in self._launch_all_process_names():
            self.processes.stop(name)
        self._launch_all_active = False
        self._append_process_output(LAUNCH_ALL_NAME, 'stopped managed component set')
        self._refresh_process_states()

    def _schedule_launch_all_action(self, delay_ms: int, callback: Callable[[], None]) -> None:
        timer = QTimer(self)
        timer.setSingleShot(True)

        def _run() -> None:
            if timer in self._launch_all_timers:
                self._launch_all_timers.remove(timer)
            if self._launch_all_active:
                callback()
            timer.deleteLater()
            self._refresh_process_states()

        timer.timeout.connect(_run)
        self._launch_all_timers.append(timer)
        timer.start(delay_ms)

    def _launch_all_process_names(self) -> list[str]:
        return [
            SIM_NAME,
            PUBLISH_PATH_NAME,
            MOVE_ARM_NAME,
            PATH_INDEX_NAME,
            BASE_FOLLOWER_NAME,
            ARM_FOLLOWER_NAME,
            MOVE_BASE_NAME,
            SWITCH_ARM_VELOCITY_NAME,
        ]

    def _toggle_sim(self) -> None:
        process = self.processes.get(SIM_NAME)
        if process is not None and process.is_running():
            self.processes.stop(SIM_NAME)
            self._append_process_output(SIM_NAME, 'stopped by operator')
            self._refresh_process_states()
            return

        self._start_sim()
        self._refresh_process_states()

    def _start_sim(self) -> None:
        gui_value = 'true' if self.simulation_checkbox.isChecked() else 'false'
        command = [
            'ros2',
            'launch',
            'rbvogui_ur_sim_setup',
            'rbvogui_ur_standard_control.launch.py',
            f'gui:={gui_value}',
            'robot_id:=robot',
        ]
        self._append_process_output(SIM_NAME, ' '.join(command))
        self.processes.start(SIM_NAME, command)

    def _publish_path(self) -> None:
        process = self.processes.get(PUBLISH_PATH_NAME)
        if process is not None and process.is_running():
            self.processes.stop(PUBLISH_PATH_NAME)
            self._append_process_output(PUBLISH_PATH_NAME, 'stopped by operator')
            self._refresh_process_states()
            return

        self._start_publish_path()
        self._refresh_process_states()

    def _start_publish_path(self) -> None:
        command = [
            'ros2',
            'launch',
            'parse_paths',
            'robotnik_base_arm_paths.launch.py',
            'load_exported_trajectories:=true',
            f'trajectory_directory:={self.path_folder.text()}',
            'publish_once:=false',
        ]
        self._append_process_output(PUBLISH_PATH_NAME, ' '.join(command))
        self.processes.start(PUBLISH_PATH_NAME, command)

    def _toggle_base_follower(self) -> None:
        process = self.processes.get(BASE_FOLLOWER_NAME)
        if process is not None and process.is_running():
            self.processes.stop(BASE_FOLLOWER_NAME)
            self._append_process_output(BASE_FOLLOWER_NAME, 'stopped by operator')
            self._refresh_process_states()
            return

        self._start_base_follower()
        self._refresh_process_states()

    def _start_base_follower(self) -> None:
        command = [
            'ros2',
            'run',
            'base_trajectory_follower',
            'simple_base_follower',
            '--ros-args',
            '-p', 'use_sim_time:=true',
            '-p', 'path_topic:=/base_path',
            '-p', 'robot_pose_topic:=/robot_pose',
            '-p', 'robot_pose_type:=pose_stamped',
            '-p', 'cmd_vel_topic:=/robot/robotnik_base_control/cmd_vel_unstamped',
            '-p', 'output_stamped:=false',
            '-p', 'use_external_path_index:=true',
            '-p', 'path_index_topic:=/path_index',
            '-p', 'wait_for_start_condition:=true',
            '-p', 'start_condition_topic:=/start_condition',
            '-p', 'lookahead_distance:=0.3',
            '-p', 'max_vx:=0.25',
            '-p', 'max_vy:=0.25',
            '-p', 'max_wz:=0.5',
        ]
        self._append_process_output(BASE_FOLLOWER_NAME, ' '.join(command))
        self.processes.start(BASE_FOLLOWER_NAME, command)

    def _toggle_arm_follower(self) -> None:
        process = self.processes.get(ARM_FOLLOWER_NAME)
        if process is not None and process.is_running():
            self.processes.stop(ARM_FOLLOWER_NAME)
            self._append_process_output(ARM_FOLLOWER_NAME, 'stopped by operator')
            self._refresh_process_states()
            return

        self._start_arm_follower(move_to_start_pose=False)
        self._refresh_process_states()

    def _start_arm_follower(self, move_to_start_pose: bool = False) -> None:
        command = [
            'ros2',
            'launch',
            'ur_trajectory_follower',
            'sideways_arm_control.launch.py',
            'use_sim_time:=true',
            'robot_name:=robot',
            'arm:=arm',
            'joint_prefix:=robot_arm_',
            'base_link:=robot_arm_base_link',
            'tip_link:=robot_arm_tool0',
            'path_frame:=robotnik_simple',
            'robot_description_topic:=/robot/robot_description',
            'joint_states_topic:=/robot/joint_states',
            'velocity_command_topic:=/robot/arm_forward_velocity_controller/commands',
            'start_jparse_controller:=true',
            'publish_current_pose_from_tf:=false',
            'publish_path:=false',
            'publish_path_index:=false',
            f'move_to_start_pose:={str(move_to_start_pose).lower()}',
            'start_pose_trajectory_topic:=/robot/joint_trajectory_controller/joint_trajectory',
            'start_pose_publish_delay:=8.0',
            'current_pose_topic:=/current_tcp_pose',
            'path_topic:=/ur_path_transformed',
            'original_path_topic:=/ur_path_original',
            'normal_topic:=/normal_vector',
            'path_index_topic:=/path_index',
            'next_goal_topic:=/next_goal',
            'wait_for_start_condition:=true',
            'start_condition_topic:=/start_condition',
            f'initial_path_index:={self.index_spin.value()}',
            f'direction_control_mode:={self.direction_mode.currentText()}',
        ]
        self._append_process_output(ARM_FOLLOWER_NAME, ' '.join(command))
        self.processes.start(ARM_FOLLOWER_NAME, command)

    def _toggle_path_index(self) -> None:
        process = self.processes.get(PATH_INDEX_NAME)
        if process is not None and process.is_running():
            self.processes.stop(PATH_INDEX_NAME)
            self._append_process_output(PATH_INDEX_NAME, 'stopped by operator')
            self._refresh_process_states()
            return

        self._start_path_index()
        self._refresh_process_states()

    def _start_path_index(self) -> None:
        command = [
            'ros2',
            'run',
            'ur_trajectory_follower',
            'increment_path_index',
            '--ros-args',
            '-p', 'use_sim_time:=true',
            '-p', 'path_index_topic:=/path_index',
            '-p', 'next_goal_topic:=/next_goal',
            '-p', 'normal_topic:=/normal_vector',
            '-p', f'initial_path_index:={self.index_spin.value()}',
            '-p', 'path_topic:=/ur_path_transformed',
            '-p', 'publish_rate:=5.0',
            '-p', 'start_condition_topic:=/start_condition',
            '-p', 'wait_for_start_condition:=true',
        ]
        self._append_process_output(PATH_INDEX_NAME, ' '.join(command))
        self.processes.start(PATH_INDEX_NAME, command)

    def _move_base_to_start(self) -> None:
        process = self.processes.get(MOVE_BASE_NAME)
        if process is not None and process.is_running():
            self.processes.stop(MOVE_BASE_NAME)
            self._append_process_output(MOVE_BASE_NAME, 'stopped by operator')
            self._refresh_process_states()
            return

        self._start_move_base_to_start(publish_start_condition=False)
        self._refresh_process_states()

    def _start_move_base_to_start(self, publish_start_condition: bool = False) -> None:
        command = [
            'ros2',
            'run',
            'move_to_path_idx',
            'move_to_path_idx',
            '--ros-args',
            '-p', 'use_sim_time:=true',
            '-p', 'path_topic:=/base_path',
            '-p', 'robot_pose_topic:=/robot_pose',
            '-p', 'robot_pose_type:=pose_stamped',
            '-p', 'cmd_vel_topic:=/robot/robotnik_base_control/cmd_vel_unstamped',
            '-p', f'path_index:={self.index_spin.value()}',
            '-p', f'publish_start_condition:={str(publish_start_condition).lower()}',
            '-p', 'start_condition_topic:=/start_pose_reached',
            '-p', 'distance_tolerance:=0.06',
            '-p', 'yaw_tolerance:=0.08',
            '-p', 'max_linear_velocity:=0.2',
            '-p', 'max_angular_velocity:=0.5',
        ]
        self._append_process_output(MOVE_BASE_NAME, ' '.join(command))
        self.processes.start(MOVE_BASE_NAME, command)

    def _move_arm_to_start(self) -> None:
        process = self.processes.get(MOVE_ARM_NAME)
        if process is not None and process.is_running():
            self.processes.stop(MOVE_ARM_NAME)
            self._append_process_output(MOVE_ARM_NAME, 'stopped by operator')
            self._refresh_process_states()
            return

        self._start_move_arm_to_start(wait_for_start_condition=False)
        self._refresh_process_states()

    def _start_move_arm_to_start(self, wait_for_start_condition: bool = False) -> None:
        command = [
            'ros2',
            'launch',
            'move_to_path_idx',
            'move_ur_to_path_idx.launch.py',
            'path_topic:=/ur_path_transformed',
            'current_pose_topic:=/current_tcp_pose',
            f'path_index:={self.index_spin.value()}',
            f'wait_for_start_condition:={str(wait_for_start_condition).lower()}',
            'start_condition_topic:=/start_pose_reached',
            'cmd_vel_topic:=/jparse_velocity_controller_ur/twist_cmd_world',
            'path_frame:=robotnik_simple',
        ]
        self._append_process_output(MOVE_ARM_NAME, ' '.join(command))
        self.processes.start(MOVE_ARM_NAME, command)

    def _switch_arm_velocity_controller(self) -> None:
        self._start_switch_arm_velocity_controller()
        self._refresh_process_states()

    def _start_switch_arm_velocity_controller(self) -> None:
        command = [
            'ros2',
            'control',
            'switch_controllers',
            '--controller-manager',
            '/robot/controller_manager',
            '--deactivate',
            'joint_trajectory_controller',
            '--activate',
            'arm_forward_velocity_controller',
        ]
        self._append_process_output(SWITCH_ARM_VELOCITY_NAME, ' '.join(command))
        self.processes.start(SWITCH_ARM_VELOCITY_NAME, command)

    def _start_following(self) -> None:
        self.ros_bridge.publish_path_index(self.index_spin.value())
        self._append_process_output('ros', f'published /path_index {self.index_spin.value()}')
        self._start_condition_publish_count = 5
        self._publish_start_condition_once(True)
        self._style_button(self.start_following_button, 'green')

    def _stop_following(self) -> None:
        self._start_condition_publish_count = 5
        self._publish_start_condition_once(False)
        self._style_button(self.stop_following_button, 'red')

    def _publish_start_condition_once(self, value: bool) -> None:
        self.ros_bridge.publish_start_condition(value)
        self._append_process_output('ros', f'published /start_condition {str(value).lower()}')
        self._start_condition_publish_count -= 1
        if self._start_condition_publish_count > 0:
            QTimer.singleShot(200, lambda: self._publish_start_condition_once(value))

    def _open_rviz(self) -> None:
        rviz_config = Path(get_package_share_directory('am_operator_gui')) / 'rviz' / 'robotnik_operator.rviz'
        command = ['rviz2', '-d', str(rviz_config)]
        self._append_process_output(RVIZ_NAME, ' '.join(command))
        self.processes.start(RVIZ_NAME, command)
        self._refresh_process_states()

    def _publish_path_index(self, value: int) -> None:
        self.ros_bridge.publish_path_index(value)
        self._append_process_output('ros', f'published /path_index {value}')

    def _publish_overrides(self) -> None:
        velocity_percent = self.velocity_slider.value()
        velocity_scale = velocity_percent / 100.0
        reference_mm = self.nozzle_reference.value()
        offset_mm = self.nozzle_offset.value()
        effective_mm = reference_mm + offset_mm
        self.velocity_value.setText(f'{velocity_percent}%')
        self.nozzle_offset_value.setText(f'{offset_mm:+d} mm')
        self.nozzle_effective_value.setText(f'{effective_mm:.1f} mm effective')
        self.ros_bridge.publish_velocity_override(velocity_scale)
        self.ros_bridge.publish_nozzle_height(effective_mm / 1000.0)

    def _on_process_output(self, name: str, line: str) -> None:
        self.process_output.emit(name, line)

    def _append_process_output(self, name: str, line: str) -> None:
        self.log.appendPlainText(f'[{name}] {line}')

    def _on_ros_status(self, has_path: bool, has_robot_pose: bool) -> None:
        self.ros_status_changed.emit(has_path, has_robot_pose)

    def _on_path_index(self, value: int) -> None:
        self.path_index_changed.emit(value)

    def _set_ros_status(self, has_path: bool, has_robot_pose: bool) -> None:
        self._has_path = has_path
        self._has_robot_pose = has_robot_pose
        self.path_status.setText('/base_path: ready' if has_path else '/base_path: waiting')
        self.pose_status.setText('/robot_pose: ready' if has_robot_pose else '/robot_pose: waiting')
        self._refresh_process_states()

    def _set_path_index_from_ros(self, value: int) -> None:
        self.index_spin.blockSignals(True)
        self.index_spin.setValue(max(0, value))
        self.index_spin.blockSignals(False)

    def _refresh_process_states(self) -> None:
        self._set_launch_button_state()
        self._set_sim_button_state()
        self._set_publish_path_state()
        self._set_path_index_button_state()
        self._set_base_follower_button_state()
        self._set_arm_follower_button_state()
        self._set_move_base_state()
        self._set_move_arm_state()
        self._set_switch_arm_velocity_state()
        self._set_start_following_state()
        self._set_rviz_state()

    def _set_launch_button_state(self) -> None:
        running = any(
            (process := self.processes.get(name)) is not None and process.is_running()
            for name in self._launch_all_process_names()
        )
        if self._launch_all_active and not running and not self._launch_all_timers:
            self._launch_all_active = False
        active = self._launch_all_active or running or bool(self._launch_all_timers)
        self.launch_button.setText('Stop All' if active else 'Launch All')
        self._style_button(self.launch_button, 'green' if active else 'grey')

    def _set_sim_button_state(self) -> None:
        self._set_process_toggle_button(self.launch_sim_button, SIM_NAME, 'Stop Sim', 'Launch Sim')

    def _set_publish_path_state(self) -> None:
        self._set_process_toggle_button(self.publish_path_button, PUBLISH_PATH_NAME, 'Stop Path', 'Publish Path')

    def _set_path_index_button_state(self) -> None:
        self._set_process_toggle_button(self.path_index_button, PATH_INDEX_NAME, 'Stop Path Index', 'Launch Path Index')

    def _set_base_follower_button_state(self) -> None:
        self._set_process_toggle_button(
            self.base_follower_button,
            BASE_FOLLOWER_NAME,
            'Stop Base Follower',
            'Launch Base Follower',
        )

    def _set_arm_follower_button_state(self) -> None:
        self._set_process_toggle_button(
            self.arm_follower_button,
            ARM_FOLLOWER_NAME,
            'Stop Arm Follower',
            'Launch Arm Follower',
        )

    def _set_move_base_state(self) -> None:
        process = self.processes.get(MOVE_BASE_NAME)
        if process is None:
            color = 'grey'
            text = 'Move Base To Start'
        elif process.is_running() and (not self._has_path or not self._has_robot_pose):
            color = 'orange'
            text = 'Stop Base Move'
        elif process.is_running():
            color = 'yellow'
            text = 'Stop Base Move'
        elif process.return_code == 0:
            color = 'green'
            text = 'Move Base To Start'
        else:
            color = 'red'
            text = 'Move Base To Start'
        self.move_base_button.setText(text)
        self._style_button(self.move_base_button, color)

    def _set_move_arm_state(self) -> None:
        process = self.processes.get(MOVE_ARM_NAME)
        if process is None:
            color = 'grey'
            text = 'Move Arm To Start'
        elif process.is_running():
            color = 'yellow'
            text = 'Stop Arm Move'
        elif process.return_code == 0:
            color = 'green'
            text = 'Move Arm To Start'
        else:
            color = 'red'
            text = 'Move Arm To Start'
        self.move_arm_button.setText(text)
        self._style_button(self.move_arm_button, color)

    def _set_switch_arm_velocity_state(self) -> None:
        process = self.processes.get(SWITCH_ARM_VELOCITY_NAME)
        if process is None:
            color = 'grey'
        elif process.is_running():
            color = 'yellow'
        elif process.return_code == 0:
            color = 'green'
        else:
            color = 'red'
        self._style_button(self.switch_arm_velocity_button, color)

    def _set_start_following_state(self) -> None:
        if getattr(self, '_start_condition_publish_count', 0) > 0:
            return
        self._style_button(self.start_following_button, 'grey')
        self._style_button(self.stop_following_button, 'grey')

    def _set_rviz_state(self) -> None:
        process = self.processes.get(RVIZ_NAME)
        running = process is not None and process.is_running()
        self._style_button(self.rviz_button, 'green' if running else 'grey')

    def _set_process_toggle_button(
        self,
        button: QPushButton,
        process_name: str,
        stop_text: str,
        start_text: str,
    ) -> None:
        process = self.processes.get(process_name)
        if process is None:
            button.setText(start_text)
            self._style_button(button, 'grey')
            return
        if process.is_running():
            button.setText(stop_text)
            self._style_button(button, 'green')
            return
        button.setText(start_text)
        self._style_button(button, 'green' if process.return_code == 0 else 'red')

    @staticmethod
    def _style_button(button: QPushButton, color: str) -> None:
        colors = {
            'grey': '#d6d6d6',
            'green': '#53a653',
            'yellow': '#f0d35b',
            'orange': '#f2a541',
            'red': '#d95c5c',
        }
        background = colors.get(color, colors['grey'])
        button.setStyleSheet(
            f'QPushButton {{ background-color: {background}; color: #101010; padding: 6px; }}'
        )

    def closeEvent(self, event) -> None:
        self.processes.stop_all()
        self.ros_bridge.stop()
        event.accept()


def run(args: Optional[list[str]] = None) -> int:
    app = QApplication(args if args is not None else sys.argv)
    window = OperatorWindow()
    window.show()

    def _handle_sigint(_sig, _frame):
        window.close()

    signal.signal(signal.SIGINT, _handle_sigint)
    return app.exec_()
