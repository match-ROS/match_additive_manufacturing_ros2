import json
import math
import signal
import statistics
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
DEFAULT_PATH_INDEX_RATE = 5.0
DEFAULT_DEFAULT_VELOCITY = 0.1
CONFIG_PATH = Path.home() / '.config' / 'am_operator_gui' / 'operator_gui_config.json'
LAUNCH_ALL_NAME = 'launch_all'
SIM_NAME = 'launch_sim'
PUBLISH_PATH_NAME = 'publish_path'
BASE_FOLLOWER_NAME = 'base_follower'
ARM_FOLLOWER_NAME = 'arm_follower'
PATH_INDEX_NAME = 'path_index'
CURRENT_TCP_POSE_NAME = 'current_tcp_pose'
BASE_POSE_ADAPTER_NAME = 'base_pose_adapter'
ARM_POSE_ADAPTER_NAME = 'arm_pose_adapter'
ARM_CONTROLLERS_NAME = 'arm_controllers'
MOVE_BASE_NAME = 'move_base_to_start'
MOVE_ARM_NAME = 'move_arm_to_start'
SWITCH_ARM_VELOCITY_NAME = 'switch_arm_velocity_controller'
RVIZ_NAME = 'rviz'

PLATFORM_PROFILES = {
    'robotnik': {
        'label': 'Robotnik',
        'path_topic': '/base_path',
        'robot_pose_topic': '/robot_pose',
        'cmd_vel_topic': '/robot/robotnik_base_control/cmd_vel_unstamped',
        'output_stamped': False,
        'command_frame_id': 'base_link',
        'path_frame': 'robotnik_simple',
        'max_vx': 0.25,
        'max_vy': 0.25,
        'max_wz': 0.5,
        'move_max_linear': 0.2,
        'move_max_lateral': 0.2,
        'move_max_angular': 0.5,
    },
    'bunker': {
        'label': 'Bunker',
        'path_topic': '/base_path',
        'robot_pose_topic': '/robot_pose',
        'cmd_vel_topic': '/diff_drive_controller/cmd_vel',
        'output_stamped': True,
        'command_frame_id': 'base_footprint',
        'path_frame': 'map',
        'max_vx': 0.25,
        'max_vy': 0.0,
        'max_wz': 0.6,
        'move_max_linear': 0.2,
        'move_max_lateral': 0.0,
        'move_max_angular': 0.5,
    },
}


class OperatorWindow(QMainWindow):
    ros_status_changed = pyqtSignal(bool, bool, bool, bool, bool)
    path_index_changed = pyqtSignal(int)
    process_output = pyqtSignal(str, str)

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle('AM Operator GUI')
        self.resize(980, 720)
        self._has_path = False
        self._has_robot_pose = False
        self._has_arm_pose = False
        self._jparse_ready = False
        self._controller_ready = False
        self._launch_all_active = False
        self._launch_all_timers: list[QTimer] = []
        self._config = self._load_config()

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

    def _load_config(self) -> dict:
        try:
            with CONFIG_PATH.open('r', encoding='utf-8') as config_file:
                data = json.load(config_file)
        except (OSError, json.JSONDecodeError):
            return {}
        return data if isinstance(data, dict) else {}

    def _save_config(self) -> None:
        try:
            CONFIG_PATH.parent.mkdir(parents=True, exist_ok=True)
            with CONFIG_PATH.open('w', encoding='utf-8') as config_file:
                json.dump(self._config, config_file, indent=2, sort_keys=True)
                config_file.write('\n')
        except OSError as exc:
            self._append_process_output('gui', f'failed to save config: {exc}')

    def _configured_path_index_rate(self) -> float:
        try:
            rate = float(self._config.get('path_index_rate', DEFAULT_PATH_INDEX_RATE))
        except (TypeError, ValueError):
            return DEFAULT_PATH_INDEX_RATE
        return max(0.01, min(1000.0, rate))

    def _configured_default_velocity(self) -> float:
        try:
            velocity = float(self._config.get('default_velocity', DEFAULT_DEFAULT_VELOCITY))
        except (TypeError, ValueError):
            return DEFAULT_DEFAULT_VELOCITY
        return max(0.001, min(10.0, velocity))

    def _configured_default_velocity_enabled(self) -> bool:
        return bool(self._config.get('default_velocity_enabled', False))

    def _configured_platform(self) -> str:
        value = str(self._config.get('platform', 'robotnik')).strip().lower()
        return value if value in PLATFORM_PROFILES else 'robotnik'

    def _configured_follower_type(self) -> str:
        value = str(self._config.get('follower_type', 'pid')).strip().lower()
        return value if value in {'pid', 'pure_pursuit'} else 'pid'

    def _configured_diff_drive_mode(self) -> bool:
        if self._configured_platform() == 'bunker':
            return True
        return bool(self._config.get('diff_drive_mode', False))

    def _configured_base_pose_topic(self) -> str:
        return str(self._config.get('base_pose_topic', '/vicon/Base_RB/Base_RB'))

    def _configured_arm_pose_topic(self) -> str:
        return str(self._config.get('arm_pose_topic', '/vicon/robot_ee/robot_ee'))

    def _configured_control_frame(self) -> str:
        configured = str(self._config.get('control_frame', '')).strip()
        return configured or self._path_frame_from_folder(Path(DEFAULT_TRAJECTORY_DIR))

    def _build_ui(self) -> None:
        root = QWidget()
        layout = QVBoxLayout(root)
        layout.setSpacing(10)

        launch_group = QGroupBox('System')
        launch_layout = QGridLayout(launch_group)
        self.simulation_checkbox = QCheckBox('Simulation')
        self.platform_combo = QComboBox()
        for key, profile in PLATFORM_PROFILES.items():
            self.platform_combo.addItem(str(profile['label']), key)
        self.platform_combo.setCurrentIndex(self.platform_combo.findData(self._configured_platform()))
        self.follower_type_combo = QComboBox()
        self.follower_type_combo.addItem('PID', 'pid')
        self.follower_type_combo.addItem('Pure Pursuit', 'pure_pursuit')
        self.follower_type_combo.setCurrentIndex(
            self.follower_type_combo.findData(self._configured_follower_type())
        )
        self.diff_drive_checkbox = QCheckBox('Diff drive mode')
        self.diff_drive_checkbox.setChecked(self._configured_diff_drive_mode())
        self._sync_diff_drive_checkbox()
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
        launch_layout.addWidget(QLabel('Platform'), 0, 1)
        launch_layout.addWidget(self.platform_combo, 0, 2)
        launch_layout.addWidget(QLabel('Follower'), 0, 3)
        launch_layout.addWidget(self.follower_type_combo, 0, 4)
        launch_layout.addWidget(self.diff_drive_checkbox, 0, 5)
        launch_layout.addWidget(QLabel('Direction'), 1, 0)
        launch_layout.addWidget(self.direction_mode, 1, 1)
        launch_layout.addWidget(QLabel('Current index'), 1, 2)
        launch_layout.addWidget(self.index_spin, 1, 3)
        launch_layout.addWidget(QLabel('Path folder'), 2, 0)
        launch_layout.addWidget(self.path_folder, 2, 1, 1, 4)
        launch_layout.addWidget(self.browse_button, 2, 5)
        self.base_pose_topic = QLineEdit(self._configured_base_pose_topic())
        self.arm_pose_topic = QLineEdit(self._configured_arm_pose_topic())
        self.control_frame = QLineEdit(self._configured_control_frame())
        launch_layout.addWidget(QLabel('Base pose topic'), 3, 0)
        launch_layout.addWidget(self.base_pose_topic, 3, 1, 1, 2)
        launch_layout.addWidget(QLabel('EE pose topic'), 3, 3)
        launch_layout.addWidget(self.arm_pose_topic, 3, 4, 1, 2)
        launch_layout.addWidget(QLabel('Control frame'), 4, 0)
        launch_layout.addWidget(self.control_frame, 4, 1, 1, 2)
        launch_layout.addWidget(self.launch_button, 5, 0)
        launch_layout.addWidget(self.launch_sim_button, 5, 1)
        launch_layout.addWidget(self.rviz_button, 5, 2)

        component_group = QGroupBox('Components')
        component_layout = QGridLayout(component_group)
        self.base_follower_button = QPushButton('Launch Base Follower')
        self.arm_follower_button = QPushButton('Launch Arm Follower')
        self.path_index_button = QPushButton('Launch Path Index')
        self.current_tcp_pose_button = QPushButton('Launch TCP Pose')
        self.arm_controllers_button = QPushButton('Start Controllers')
        self.switch_arm_velocity_button = QPushButton('Switch Arm Velocity')
        self.path_index_rate_spin = QDoubleSpinBox()
        self.path_index_rate_spin.setRange(0.01, 1000.0)
        self.path_index_rate_spin.setDecimals(3)
        self.path_index_rate_spin.setSuffix(' Hz')
        self.path_index_rate_spin.setValue(self._configured_path_index_rate())
        self.calculate_path_index_rate_button = QPushButton('Calculate Index Rate')
        self.default_velocity_checkbox = QCheckBox('Default velocity')
        self.default_velocity_checkbox.setChecked(self._configured_default_velocity_enabled())
        self.default_velocity_spin = QDoubleSpinBox()
        self.default_velocity_spin.setRange(0.001, 10.0)
        self.default_velocity_spin.setDecimals(3)
        self.default_velocity_spin.setSingleStep(0.01)
        self.default_velocity_spin.setSuffix(' m/s')
        self.default_velocity_spin.setValue(self._configured_default_velocity())
        self.default_velocity_spin.setEnabled(self.default_velocity_checkbox.isChecked())

        component_layout.addWidget(self.publish_path_button, 0, 0)
        component_layout.addWidget(self.path_index_button, 0, 1)
        component_layout.addWidget(self.base_follower_button, 1, 0)
        component_layout.addWidget(self.arm_follower_button, 1, 1)
        component_layout.addWidget(self.current_tcp_pose_button, 2, 0)
        component_layout.addWidget(self.arm_controllers_button, 2, 1)
        component_layout.addWidget(self.switch_arm_velocity_button, 2, 2)
        component_layout.addWidget(QLabel('Index rate'), 3, 0)
        component_layout.addWidget(self.path_index_rate_spin, 3, 1)
        component_layout.addWidget(self.calculate_path_index_rate_button, 3, 2)
        component_layout.addWidget(self.default_velocity_checkbox, 4, 0)
        component_layout.addWidget(self.default_velocity_spin, 4, 1)

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
        self.arm_pose_status = QLabel('/current_tcp_pose: waiting')
        self.arm_control_status = QLabel('arm control: waiting')
        status_layout.addWidget(self.path_status)
        status_layout.addWidget(self.pose_status)
        status_layout.addWidget(self.arm_pose_status)
        status_layout.addWidget(self.arm_control_status)
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
        self.base_pose_topic.editingFinished.connect(self._save_hardware_topics)
        self.arm_pose_topic.editingFinished.connect(self._save_hardware_topics)
        self.control_frame.editingFinished.connect(self._save_hardware_topics)
        self.simulation_checkbox.toggled.connect(self._simulation_mode_changed)
        self.platform_combo.currentIndexChanged.connect(self._set_platform)
        self.follower_type_combo.currentIndexChanged.connect(self._set_follower_type)
        self.diff_drive_checkbox.toggled.connect(self._set_diff_drive_mode)
        self.launch_button.clicked.connect(self._toggle_launch_all)
        self.launch_sim_button.clicked.connect(self._toggle_sim)
        self.publish_path_button.clicked.connect(self._publish_path)
        self.base_follower_button.clicked.connect(self._toggle_base_follower)
        self.arm_follower_button.clicked.connect(self._toggle_arm_follower)
        self.path_index_button.clicked.connect(self._toggle_path_index)
        self.current_tcp_pose_button.clicked.connect(self._toggle_current_tcp_pose)
        self.arm_controllers_button.clicked.connect(self._toggle_arm_controllers)
        self.calculate_path_index_rate_button.clicked.connect(self._calculate_path_index_rate)
        self.switch_arm_velocity_button.clicked.connect(self._switch_arm_velocity_controller)
        self.move_base_button.clicked.connect(self._move_base_to_start)
        self.move_arm_button.clicked.connect(self._move_arm_to_start)
        self.start_following_button.clicked.connect(self._start_following)
        self.stop_following_button.clicked.connect(self._stop_following)
        self.rviz_button.clicked.connect(self._open_rviz)
        self.index_spin.valueChanged.connect(self._publish_path_index)
        self.velocity_slider.valueChanged.connect(self._publish_overrides)
        self.path_index_rate_spin.valueChanged.connect(self._set_path_index_rate)
        self.default_velocity_checkbox.toggled.connect(self._set_default_velocity_enabled)
        self.default_velocity_spin.valueChanged.connect(self._set_default_velocity)
        self.nozzle_reference.valueChanged.connect(self._publish_overrides)
        self.nozzle_offset.valueChanged.connect(self._publish_overrides)

    def _current_platform_key(self) -> str:
        value = self.platform_combo.currentData()
        key = str(value).strip().lower()
        return key if key in PLATFORM_PROFILES else 'robotnik'

    def _current_platform_profile(self) -> dict:
        return PLATFORM_PROFILES[self._current_platform_key()]

    def _current_follower_type(self) -> str:
        value = self.follower_type_combo.currentData()
        follower_type = str(value).strip().lower()
        return follower_type if follower_type in {'pid', 'pure_pursuit'} else 'pid'

    def _diff_drive_mode(self) -> bool:
        return self._current_platform_key() == 'bunker' or self.diff_drive_checkbox.isChecked()

    def _sync_diff_drive_checkbox(self) -> None:
        is_bunker = self._current_platform_key() == 'bunker'
        self.diff_drive_checkbox.blockSignals(True)
        if is_bunker:
            self.diff_drive_checkbox.setChecked(True)
        self.diff_drive_checkbox.setEnabled(not is_bunker)
        self.diff_drive_checkbox.blockSignals(False)

    def _set_platform(self, *_args) -> None:
        platform = self._current_platform_key()
        previous_platform = str(self._config.get('platform', 'robotnik')).strip().lower()
        if platform == 'robotnik' and previous_platform == 'bunker':
            self.diff_drive_checkbox.blockSignals(True)
            self.diff_drive_checkbox.setChecked(False)
            self.diff_drive_checkbox.blockSignals(False)
        self._config['platform'] = platform
        self._sync_diff_drive_checkbox()
        self._config['diff_drive_mode'] = self._diff_drive_mode()
        self._save_config()
        profile = self._current_platform_profile()
        self.path_status.setText(f"{profile['path_topic']}: ready" if self._has_path else f"{profile['path_topic']}: waiting")

    def _set_follower_type(self, *_args) -> None:
        self._config['follower_type'] = self._current_follower_type()
        self._save_config()

    def _set_diff_drive_mode(self, enabled: bool) -> None:
        self._config['diff_drive_mode'] = bool(enabled)
        self._save_config()

    def _choose_path_folder(self) -> None:
        folder = QFileDialog.getExistingDirectory(
            self,
            'Select trajectory folder',
            str(DEFAULT_COMPONENTS_DIR),
        )
        if folder:
            self.path_folder.setText(folder)
            frame = self._path_frame_from_folder(Path(folder))
            if frame:
                self.control_frame.setText(frame)
            self._save_hardware_topics()

    @staticmethod
    def _path_frame_from_folder(folder: Path) -> str:
        for filename in ('base_path.json', 'arm_path.json'):
            try:
                data = json.loads((folder / filename).read_text(encoding='utf-8'))
            except (OSError, json.JSONDecodeError, TypeError):
                continue
            frame = str(data.get('frame_id', '')).strip() if isinstance(data, dict) else ''
            if frame:
                return frame
        return 'robotnik_simple'

    def _save_hardware_topics(self) -> None:
        self._config['base_pose_topic'] = self.base_pose_topic.text().strip()
        self._config['arm_pose_topic'] = self.arm_pose_topic.text().strip()
        self._config['control_frame'] = self.control_frame.text().strip()
        self._save_config()

    def _simulation_mode_changed(self, _enabled: bool) -> None:
        self._refresh_process_states()

    def _use_sim_time(self) -> str:
        return 'true' if self.simulation_checkbox.isChecked() else 'false'

    def _toggle_launch_all(self) -> None:
        if self._launch_all_active:
            self._stop_launch_all_components()
            return

        self._start_launch_all_components()
        self._refresh_process_states()

    def _start_launch_all_components(self) -> None:
        self._launch_all_active = True
        self._append_process_output(LAUNCH_ALL_NAME, 'starting managed component set')
        self.ros_bridge.publish_start_condition(False)
        self.ros_bridge.publish_stop_commands(self.control_frame.text().strip())
        if self.simulation_checkbox.isChecked():
            self._start_sim()
            self._start_current_tcp_pose()
        else:
            self._start_pose_adapters()
        self._start_publish_path()
        self._start_arm_controllers()
        self._start_path_index()
        self._start_base_follower()
        self._start_arm_follower(move_to_start_pose=False)
        if self.simulation_checkbox.isChecked():
            self._start_move_arm_to_start(wait_for_start_condition=True)
            self._schedule_launch_all_action(13000, lambda: self._start_move_base_to_start(
                publish_start_condition=True,
            ))

    def _stop_launch_all_components(self) -> None:
        self.ros_bridge.publish_start_condition(False)
        self.ros_bridge.publish_stop_commands(self.control_frame.text().strip())
        for timer in self._launch_all_timers:
            timer.stop()
            timer.deleteLater()
        self._launch_all_timers.clear()
        for name in self._launch_all_process_names():
            self.processes.stop(name)
        self._launch_all_active = False
        self._append_process_output(LAUNCH_ALL_NAME, 'stopped managed component set')
        self._refresh_process_states()

    def _schedule_launch_all_action(
        self,
        delay_ms: int,
        callback: Callable[[], None],
        require_launch_all_active: bool = True,
    ) -> None:
        timer = QTimer(self)
        timer.setSingleShot(True)

        def _run() -> None:
            if timer in self._launch_all_timers:
                self._launch_all_timers.remove(timer)
            if self._launch_all_active or not require_launch_all_active:
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
            CURRENT_TCP_POSE_NAME,
            BASE_POSE_ADAPTER_NAME,
            ARM_POSE_ADAPTER_NAME,
            ARM_CONTROLLERS_NAME,
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
        if self._current_platform_key() == 'bunker':
            headless_value = 'false' if self.simulation_checkbox.isChecked() else 'true'
            command = [
                'ros2',
                'launch',
                'bunker_description',
                'spawn_with_controllers.launch.py',
                f'headless:={headless_value}',
                'launch_rviz:=false',
            ]
        else:
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
            f'use_sim_time:={self._use_sim_time()}',
            f'frame_id:={self.control_frame.text().strip()}',
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
        profile = self._current_platform_profile()
        diff_drive = self._diff_drive_mode()
        command = [
            'ros2',
            'run',
            'base_trajectory_follower',
            'simple_base_follower',
            '--ros-args',
            '-p', f'use_sim_time:={self._use_sim_time()}',
            '-p', f"path_topic:={profile['path_topic']}",
            '-p', f"robot_pose_topic:={profile['robot_pose_topic']}",
            '-p', 'robot_pose_type:=pose_stamped',
            '-p', f"cmd_vel_topic:={profile['cmd_vel_topic']}",
            '-p', f"output_stamped:={str(bool(profile['output_stamped'])).lower()}",
            '-p', f"command_frame_id:={profile['command_frame_id']}",
            '-p', f'follower_type:={self._current_follower_type()}',
            '-p', f'diff_drive_mode:={str(diff_drive).lower()}',
            '-p', 'use_external_path_index:=true',
            '-p', 'path_index_topic:=/path_index',
            '-p', 'wait_for_start_condition:=true',
            '-p', 'start_condition_topic:=/start_condition',
            '-p', 'velocity_override_topic:=/velocity_override',
            '-p', 'lookahead_distance:=0.3',
            '-p', f"max_vx:={self._ros_float_literal(float(profile['max_vx']))}",
            '-p', f"max_vy:={self._ros_float_literal(float(profile['max_vy']))}",
            '-p', f"max_wz:={self._ros_float_literal(float(profile['max_wz']))}",
            '-p', f'default_linear_velocity:={self._ros_float_literal(self._default_velocity_param())}',
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
            f'use_sim_time:={self._use_sim_time()}',
            'robot_name:=robot',
            'arm:=arm',
            'joint_prefix:=robot_arm_',
            'base_link:=robot_arm_base_link',
            'tip_link:=robot_arm_tool0',
            f'path_frame:={self.control_frame.text().strip()}',
            'robot_description_topic:=/robot/robot_description',
            'joint_states_topic:=/robot/joint_states',
            f"velocity_command_topic:={self._arm_velocity_command_topic()}",
            'start_jparse_controller:=false',
            'start_command_transform:=false',
            'publish_current_pose_from_tf:=false',
            'publish_path:=false',
            'publish_path_index:=false',
            f'move_to_start_pose:={str(move_to_start_pose).lower()}',
            f"start_pose_trajectory_topic:={self._arm_trajectory_topic()}",
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
            f'default_velocity:={self._ros_float_literal(self._default_velocity_param())}',
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
            '-p', f'use_sim_time:={self._use_sim_time()}',
            '-p', 'path_index_topic:=/path_index',
            '-p', 'next_goal_topic:=/next_goal',
            '-p', 'normal_topic:=/normal_vector',
            '-p', f'initial_path_index:={self.index_spin.value()}',
            '-p', 'path_topic:=/ur_path_transformed',
            '-p', f'publish_rate:={self._ros_float_literal(self.path_index_rate_spin.value())}',
            '-p', 'velocity_override_topic:=/velocity_override',
            '-p', 'start_condition_topic:=/start_condition',
            '-p', 'wait_for_start_condition:=true',
        ]
        self._append_process_output(PATH_INDEX_NAME, ' '.join(command))
        self.processes.start(PATH_INDEX_NAME, command)

    def _toggle_current_tcp_pose(self) -> None:
        if not self.simulation_checkbox.isChecked():
            running = any(
                (process := self.processes.get(name)) is not None and process.is_running()
                for name in (BASE_POSE_ADAPTER_NAME, ARM_POSE_ADAPTER_NAME)
            )
            if running:
                self.processes.stop(BASE_POSE_ADAPTER_NAME)
                self.processes.stop(ARM_POSE_ADAPTER_NAME)
            else:
                self._start_pose_adapters()
            self._refresh_process_states()
            return
        process = self.processes.get(CURRENT_TCP_POSE_NAME)
        if process is not None and process.is_running():
            self.processes.stop(CURRENT_TCP_POSE_NAME)
            self._append_process_output(CURRENT_TCP_POSE_NAME, 'stopped by operator')
            self._refresh_process_states()
            return

        self._start_current_tcp_pose()
        self._refresh_process_states()

    def _start_current_tcp_pose(self) -> None:
        command = [
            'ros2',
            'run',
            'ur_trajectory_follower',
            'current_pose_from_tf',
            '--ros-args',
            '-p', f'use_sim_time:={self._use_sim_time()}',
            '-p', f'target_frame:={self.control_frame.text().strip()}',
            '-p', 'source_frame:=robot_arm_tool0',
            '-p', 'pose_topic:=/current_tcp_pose',
            '-p', 'publish_rate:=20.0',
        ]
        self._append_process_output(CURRENT_TCP_POSE_NAME, ' '.join(command))
        self.processes.start(CURRENT_TCP_POSE_NAME, command)

    def _start_pose_adapters(self) -> None:
        adapters = (
            (
                BASE_POSE_ADAPTER_NAME,
                self.base_pose_topic.text().strip(),
                '/robot_pose',
                '/am/base_pose_ready',
            ),
            (
                ARM_POSE_ADAPTER_NAME,
                self.arm_pose_topic.text().strip(),
                '/current_tcp_pose',
                '/am/arm_pose_ready',
            ),
        )
        for name, input_topic, output_topic, ready_topic in adapters:
            command = [
                'ros2',
                'run',
                'am_operator_gui',
                'pose_stamped_adapter',
                '--ros-args',
                '-r', f'__node:={name}',
                '-p', f'use_sim_time:={self._use_sim_time()}',
                '-p', f'input_topic:={input_topic}',
                '-p', f'output_topic:={output_topic}',
                '-p', f'target_frame:={self.control_frame.text().strip()}',
                '-p', f'ready_topic:={ready_topic}',
                '-p', 'stale_timeout:=0.5',
            ]
            self._append_process_output(name, ' '.join(command))
            self.processes.start(name, command)

    def _toggle_arm_controllers(self) -> None:
        process = self.processes.get(ARM_CONTROLLERS_NAME)
        if process is not None and process.is_running():
            self.processes.stop(ARM_CONTROLLERS_NAME)
            self._append_process_output(ARM_CONTROLLERS_NAME, 'stopped by operator')
            self._refresh_process_states()
            return

        self._start_arm_controllers()
        self._refresh_process_states()

    def _start_arm_controllers(self) -> None:
        simulation = self.simulation_checkbox.isChecked()
        command = [
            'ros2',
            'launch',
            'am_operator_gui',
            'arm_velocity_controller_stack.launch.py',
            f'use_sim_time:={self._use_sim_time()}',
            'robot_name:=robot',
            'arm:=arm',
            'base_link:=robot_arm_base_link',
            'tip_link:=robot_arm_tool0',
            f'path_frame:={self.control_frame.text().strip()}',
            'robot_description_topic:=/robot/robot_description',
            'joint_states_topic:=/robot/joint_states',
            'source_twist_topic:=/jparse_velocity_controller_ur/twist_cmd_world',
            'controller_twist_topic:=/jparse_velocity_controller_ur/twist_cmd',
            f"velocity_command_topic:={self._arm_velocity_command_topic()}",
            f"controller_manager:={self._arm_controller_manager()}",
            'deactivate_controller:=joint_trajectory_controller',
            f"activate_controller:={'arm_forward_velocity_controller' if simulation else 'forward_velocity_controller'}",
            'jparse_readiness_topic:=/am/jparse_ready',
            'controller_readiness_topic:=/am/arm_controller_ready',
            'command_joint_names_csv:=robot_arm_shoulder_pan_joint,robot_arm_shoulder_lift_joint,'
            'robot_arm_elbow_joint,robot_arm_wrist_1_joint,robot_arm_wrist_2_joint,'
            'robot_arm_wrist_3_joint',
        ]
        self._append_process_output(ARM_CONTROLLERS_NAME, ' '.join(command))
        self.processes.start(ARM_CONTROLLERS_NAME, command)

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
        profile = self._current_platform_profile()
        diff_drive = self._diff_drive_mode()
        command = [
            'ros2',
            'run',
            'move_to_path_idx',
            'move_to_path_idx',
            '--ros-args',
            '-p', f'use_sim_time:={self._use_sim_time()}',
            '-p', f"path_topic:={profile['path_topic']}",
            '-p', f"robot_pose_topic:={profile['robot_pose_topic']}",
            '-p', 'robot_pose_type:=pose_stamped',
            '-p', f"cmd_vel_topic:={profile['cmd_vel_topic']}",
            '-p', f"output_stamped:={str(bool(profile['output_stamped'])).lower()}",
            '-p', f"command_frame_id:={profile['command_frame_id']}",
            '-p', f'diff_drive_mode:={str(diff_drive).lower()}',
            '-p', f'path_index:={self.index_spin.value()}',
            '-p', f'publish_start_condition:={str(publish_start_condition).lower()}',
            '-p', 'start_condition_topic:=/start_pose_reached',
            '-p', 'distance_tolerance:=0.06',
            '-p', 'yaw_tolerance:=0.08',
            '-p', f"max_linear_velocity:={self._ros_float_literal(float(profile['move_max_linear']))}",
            '-p', f"max_lateral_velocity:={self._ros_float_literal(float(profile['move_max_lateral']))}",
            '-p', f"max_angular_velocity:={self._ros_float_literal(float(profile['move_max_angular']))}",
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
            f'use_sim_time:={self._use_sim_time()}',
            'path_topic:=/ur_path_transformed',
            'current_pose_topic:=/current_tcp_pose',
            f'path_index:={self.index_spin.value()}',
            f'wait_for_start_condition:={str(wait_for_start_condition).lower()}',
            'start_condition_topic:=/start_pose_reached',
            'cmd_vel_topic:=/jparse_velocity_controller_ur/twist_cmd_world',
            f'path_frame:={self.control_frame.text().strip()}',
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
            self._arm_controller_manager(),
            '--deactivate',
            'joint_trajectory_controller',
            '--activate',
            'arm_forward_velocity_controller' if self.simulation_checkbox.isChecked() else 'forward_velocity_controller',
        ]
        self._append_process_output(SWITCH_ARM_VELOCITY_NAME, ' '.join(command))
        self.processes.start(SWITCH_ARM_VELOCITY_NAME, command)

    def _start_following(self) -> None:
        if not self._motion_ready():
            self._append_process_output('safety', self._motion_not_ready_reason())
            return
        self.ros_bridge.publish_path_index(self.index_spin.value())
        self._append_process_output('ros', f'published /path_index {self.index_spin.value()}')
        self._start_condition_publish_count = 5
        self._publish_start_condition_once(True)
        self._style_button(self.start_following_button, 'green')

    def _stop_following(self) -> None:
        self._start_condition_publish_count = 5
        self._publish_start_condition_once(False)
        for delay_ms in range(0, 1000, 100):
            QTimer.singleShot(
                delay_ms,
                lambda: self.ros_bridge.publish_stop_commands(self.control_frame.text().strip()),
            )
        self._style_button(self.stop_following_button, 'red')

    def _publish_start_condition_once(self, value: bool) -> None:
        self.ros_bridge.publish_start_condition(value)
        self._append_process_output('ros', f'published /start_condition {str(value).lower()}')
        self._start_condition_publish_count -= 1
        if self._start_condition_publish_count > 0:
            QTimer.singleShot(200, lambda: self._publish_start_condition_once(value))

    def _open_rviz(self) -> None:
        rviz_config = Path(get_package_share_directory('am_operator_gui')) / 'rviz' / 'robotnik_operator.rviz'
        command = ['rviz2', '-d', str(rviz_config), '-f', self.control_frame.text().strip()]
        self._append_process_output(RVIZ_NAME, ' '.join(command))
        self.processes.start(RVIZ_NAME, command)
        self._refresh_process_states()

    def _publish_path_index(self, value: int) -> None:
        self.ros_bridge.publish_path_index(value)
        self._append_process_output('ros', f'published /path_index {value}')

    def _calculate_path_index_rate(self) -> None:
        if self.default_velocity_checkbox.isChecked():
            velocity = self.default_velocity_spin.value()
            median_segment_length = self.ros_bridge.latest_ur_path_median_segment_length
            source = '/ur_path_transformed median segment length'
            if median_segment_length is None or median_segment_length <= 0.0:
                median_segment_length = self._arm_path_median_segment_length_from_file()
                source = f'{Path(self.path_folder.text()) / "arm_path.json"} median segment length'
            if median_segment_length is None or median_segment_length <= 0.0:
                self._append_process_output(
                    'gui',
                    'cannot calculate index rate: no valid arm path segment lengths found',
                )
                return

            rate = velocity / median_segment_length
            self.path_index_rate_spin.setValue(rate)
            self._append_process_output(
                'gui',
                f'calculated path index rate {rate:.3f} Hz from {velocity:.3f} m/s and {source}',
            )
            return

        rate = self.ros_bridge.latest_ur_path_rate
        source = '/ur_path_transformed'
        if rate is None or rate <= 0.0:
            rate = self._path_index_rate_from_file()
            source = str(Path(self.path_folder.text()) / 'arm_path.json')
        if rate is None or rate <= 0.0:
            self._append_process_output(
                'gui',
                'cannot calculate index rate: no valid /ur_path_transformed or arm_path.json timestamps found',
            )
            return

        self.path_index_rate_spin.setValue(rate)
        self._append_process_output('gui', f'calculated path index rate {rate:.3f} Hz from {source}')

    def _arm_path_median_segment_length_from_file(self) -> Optional[float]:
        path_file = Path(self.path_folder.text()) / 'arm_path.json'
        try:
            data = json.loads(path_file.read_text(encoding='utf-8'))
        except (OSError, json.JSONDecodeError, TypeError):
            return None

        poses = data.get('poses') if isinstance(data, dict) else None
        if not isinstance(poses, list) or len(poses) < 2:
            return None

        lengths = []
        previous_position = self._pose_position(poses[0])
        if previous_position is None:
            return None
        for pose in poses[1:]:
            current_position = self._pose_position(pose)
            if current_position is None:
                return None
            length = math.dist(previous_position, current_position)
            if length > 0.0:
                lengths.append(length)
            previous_position = current_position

        if not lengths:
            return None
        return float(statistics.median(lengths))

    def _path_index_rate_from_file(self) -> Optional[float]:
        path_file = Path(self.path_folder.text()) / 'arm_path.json'
        try:
            data = json.loads(path_file.read_text(encoding='utf-8'))
        except (OSError, json.JSONDecodeError, TypeError):
            return None

        poses = data.get('poses') if isinstance(data, dict) else None
        if not isinstance(poses, list) or len(poses) < 2:
            return None

        deltas = []
        previous_time = self._stamp_seconds(poses[0].get('stamp') if isinstance(poses[0], dict) else None)
        if previous_time is None:
            return None
        for pose in poses[1:]:
            stamp = pose.get('stamp') if isinstance(pose, dict) else None
            current_time = self._stamp_seconds(stamp)
            if current_time is None:
                return None
            delta = current_time - previous_time
            if delta > 0.0:
                deltas.append(delta)
            previous_time = current_time

        if not deltas:
            return None
        return len(deltas) / sum(deltas)

    @staticmethod
    def _pose_position(pose: object) -> Optional[tuple[float, float, float]]:
        if not isinstance(pose, dict):
            return None
        position = pose.get('position')
        if not isinstance(position, dict):
            return None
        try:
            return (
                float(position.get('x', 0.0)),
                float(position.get('y', 0.0)),
                float(position.get('z', 0.0)),
            )
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _stamp_seconds(stamp: object) -> Optional[float]:
        if not isinstance(stamp, dict):
            return None
        try:
            return float(stamp.get('sec', 0.0)) + float(stamp.get('nanosec', 0.0)) / 1e9
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _ros_float_literal(value: float) -> str:
        return f'{float(value):.6f}'

    def _set_path_index_rate(self, value: float) -> None:
        self._config['path_index_rate'] = float(value)
        self._save_config()

    def _set_default_velocity_enabled(self, enabled: bool) -> None:
        self.default_velocity_spin.setEnabled(enabled)
        self._config['default_velocity_enabled'] = bool(enabled)
        self._save_config()

    def _set_default_velocity(self, value: float) -> None:
        self._config['default_velocity'] = float(value)
        self._save_config()

    def _default_velocity_param(self) -> float:
        if not self.default_velocity_checkbox.isChecked():
            return -1.0
        return self.default_velocity_spin.value()

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

    def _on_ros_status(
        self,
        has_path: bool,
        has_robot_pose: bool,
        has_arm_pose: bool,
        jparse_ready: bool,
        controller_ready: bool,
    ) -> None:
        self.ros_status_changed.emit(
            has_path,
            has_robot_pose,
            has_arm_pose,
            jparse_ready,
            controller_ready,
        )

    def _on_path_index(self, value: int) -> None:
        self.path_index_changed.emit(value)

    def _set_ros_status(
        self,
        has_path: bool,
        has_robot_pose: bool,
        has_arm_pose: bool,
        jparse_ready: bool,
        controller_ready: bool,
    ) -> None:
        self._has_path = has_path
        self._has_robot_pose = has_robot_pose
        self._has_arm_pose = has_arm_pose
        self._jparse_ready = jparse_ready
        self._controller_ready = controller_ready
        profile = self._current_platform_profile()
        path_topic = str(profile['path_topic'])
        pose_topic = str(profile['robot_pose_topic'])
        self.path_status.setText(f'{path_topic}: ready' if has_path else f'{path_topic}: waiting')
        self.pose_status.setText(f'{pose_topic}: ready' if has_robot_pose else f'{pose_topic}: waiting')
        self.arm_pose_status.setText(
            '/current_tcp_pose: ready' if has_arm_pose else '/current_tcp_pose: waiting'
        )
        arm_ready = jparse_ready and controller_ready
        self.arm_control_status.setText(
            'arm control: ready' if arm_ready else 'arm control: waiting'
        )
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
        self._set_current_tcp_pose_button_state()
        self._set_arm_controllers_button_state()
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

    def _set_current_tcp_pose_button_state(self) -> None:
        self._set_process_toggle_button(
            self.current_tcp_pose_button,
            CURRENT_TCP_POSE_NAME,
            'Stop TCP Pose',
            'Launch TCP Pose',
        )

    def _set_arm_controllers_button_state(self) -> None:
        self._set_process_toggle_button(
            self.arm_controllers_button,
            ARM_CONTROLLERS_NAME,
            'Stop Controllers',
            'Start Controllers',
        )

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
        self.start_following_button.setEnabled(self._motion_ready())
        self._style_button(self.start_following_button, 'green' if self._motion_ready() else 'grey')
        self._style_button(self.stop_following_button, 'grey')

    def _motion_ready(self) -> bool:
        return (
            self._has_path
            and self._has_robot_pose
            and self._has_arm_pose
            and self._jparse_ready
            and self._controller_ready
            and self._control_processes_running()
        )

    def _motion_not_ready_reason(self) -> str:
        missing = []
        if not self._has_path:
            missing.append('base and arm paths')
        if not self._has_robot_pose:
            missing.append('fresh transformed base pose')
        if not self._has_arm_pose:
            missing.append('fresh transformed end-effector pose')
        if not self._jparse_ready:
            missing.append('J-PARSE chain/joint states')
        if not self._controller_ready:
            missing.append('active arm velocity controller')
        if not self._control_processes_running():
            missing.append('running path index and base/arm followers')
        return 'motion blocked; waiting for ' + ', '.join(missing)

    def _control_processes_running(self) -> bool:
        return all(
            (process := self.processes.get(name)) is not None and process.is_running()
            for name in (PATH_INDEX_NAME, BASE_FOLLOWER_NAME, ARM_FOLLOWER_NAME)
        )

    def _arm_controller_manager(self) -> str:
        return '/robot/controller_manager' if self.simulation_checkbox.isChecked() else '/robot/arm/controller_manager'

    def _arm_velocity_command_topic(self) -> str:
        if self.simulation_checkbox.isChecked():
            return '/robot/arm_forward_velocity_controller/commands'
        return '/robot/arm/forward_velocity_controller/commands'

    def _arm_trajectory_topic(self) -> str:
        if self.simulation_checkbox.isChecked():
            return '/robot/joint_trajectory_controller/joint_trajectory'
        return '/robot/arm/joint_trajectory_controller/joint_trajectory'

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
