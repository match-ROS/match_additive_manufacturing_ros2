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
    QDialog,
    QDialogButtonBox,
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
    QSplitter,
    QSpinBox,
    QDoubleSpinBox,
    QVBoxLayout,
    QWidget,
)

from am_operator_gui.process_manager import ProcessRegistry
from am_operator_gui.ros_bridge import RosBridge


REPO_ROOT = Path(__file__).resolve().parents[2]
PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_SRC_ROOT = REPO_ROOT.parent
DEFAULT_COMPONENTS_DIR = REPO_ROOT / 'components'
DEFAULT_TRAJECTORY_DIR = DEFAULT_COMPONENTS_DIR / 'robotnik_paired_demo'
DEFAULT_PATH_INDEX_RATE = 5.0
DEFAULT_DEFAULT_VELOCITY = 0.1
DEFAULT_PATH_TRANSFORM = {
    'x': 0.0,
    'y': 0.0,
    'z': 0.0,
    'yaw_deg': 0.0,
}
DEFAULT_BASE_SMOOTHING = {
    'enabled': True,
    'method': 'moving_average',
    'max_accel_x': 0.25,
    'max_accel_y': 0.25,
    'max_accel_wz': 0.5,
    'moving_average_window_size': 5,
    'external_path_index_stride': 10,
}
BASE_SMOOTHING_METHODS = (
    ('moving_average', 'Moving average'),
    ('accel_limit', 'Acceleration limit'),
)
VICON_BASE_MARKER_FRAME = 'Base_RB_Base_RB'
VICON_BASE_REFERENCE_FRAME = 'robot_base_vicon_reference'
VICON_BASE_STATIC_TF = (
    '0.022595781',
    '-0.008234146',
    '-0.007327516',
    '0.004459784',
    '-0.006515752',
    '0.009033290',
    '0.999928025',
    'robot_base_footprint',
    VICON_BASE_REFERENCE_FRAME,
)
CONFIG_PATH = PACKAGE_ROOT / 'config' / 'operator_gui_config.json'
LEGACY_CONFIG_PATH = Path.home() / '.config' / 'am_operator_gui' / 'operator_gui_config.json'
LAUNCH_ALL_NAME = 'launch_all'
SIM_NAME = 'launch_sim'
PUBLISH_PATH_NAME = 'publish_path'
BASE_FOLLOWER_NAME = 'base_follower'
ARM_FOLLOWER_NAME = 'arm_follower'
PATH_INDEX_NAME = 'path_index'
CURRENT_TCP_POSE_NAME = 'current_tcp_pose'
BASE_POSE_ADAPTER_NAME = 'base_pose_adapter'
ODOMETRY_POSE_ADAPTER_NAME = 'odometry_pose_adapter'
ARM_POSE_ADAPTER_NAME = 'arm_pose_adapter'
VICON_EE_STATIC_TF_NAME = 'vicon_ee_static_tf'
VICON_BASE_STATIC_TF_NAME = 'vicon_base_static_tf'
ARM_CONTROLLERS_NAME = 'arm_controllers'
MOVE_BASE_NAME = 'move_base_to_start'
MOVE_ARM_NAME = 'move_arm_to_start'
SWITCH_ARM_VELOCITY_NAME = 'switch_arm_velocity_controller'
RVIZ_NAME = 'rviz'
SYNC_WORKSPACE_NAME = 'sync_workspace'
SYNC_REMOTE_TARGET = 'ite-dcs@192.168.0.222:~/workspaces/print_wattle_daub/src/'

DEFAULT_PID_GAINS = {
    'base_follower.kp_x': 0.8,
    'base_follower.kp_y': 0.8,
    'base_follower.kp_yaw': 1.2,
    'base_move.kp_linear': 0.6,
    'base_move.kp_lateral': 0.6,
    'base_move.kp_angular_to_point': 1.5,
    'base_move.kp_angular_reorient': 1.2,
    'arm_direction.kp_z': 0.7,
    'arm_direction.ki_z': 0.0,
    'arm_direction.kd_z': 0.0,
    'arm_direction.orthogonal_kp': 1.0,
    'arm_pid_twist.Kp_linear_x': 1.0,
    'arm_pid_twist.Ki_linear_x': 0.0,
    'arm_pid_twist.Kd_linear_x': 0.0,
    'arm_pid_twist.Kp_linear_y': 1.0,
    'arm_pid_twist.Ki_linear_y': 0.0,
    'arm_pid_twist.Kd_linear_y': 0.0,
    'arm_pid_twist.Kp_linear_z': 1.0,
    'arm_pid_twist.Ki_linear_z': 0.0,
    'arm_pid_twist.Kd_linear_z': 0.0,
    'arm_pid_twist.Kp_angular_x': 1.0,
    'arm_pid_twist.Ki_angular_x': 0.0,
    'arm_pid_twist.Kd_angular_x': 0.0,
    'arm_pid_twist.Kp_angular_y': 1.0,
    'arm_pid_twist.Ki_angular_y': 0.0,
    'arm_pid_twist.Kd_angular_y': 0.0,
    'arm_pid_twist.Kp_angular_z': 1.0,
    'arm_pid_twist.Ki_angular_z': 0.0,
    'arm_pid_twist.Kd_angular_z': 0.0,
    'arm_orientation.kp_orientation': 1.0,
    'arm_orientation.ki_orientation': 0.0,
    'arm_orientation.kd_orientation': 0.0,
    'arm_move.kp_linear': 0.8,
    'arm_move.kp_angular': 1.0,
}

PID_GAIN_GROUPS = (
    (
        'Mobile Base Follower',
        (
            ('base_follower.kp_x', 'Kp X'),
            ('base_follower.kp_y', 'Kp Y'),
            ('base_follower.kp_yaw', 'Kp yaw'),
        ),
    ),
    (
        'Mobile Base Move To Start',
        (
            ('base_move.kp_linear', 'Kp linear'),
            ('base_move.kp_lateral', 'Kp lateral'),
            ('base_move.kp_angular_to_point', 'Kp angular to point'),
            ('base_move.kp_angular_reorient', 'Kp angular reorient'),
        ),
    ),
    (
        'Arm Path Direction',
        (
            ('arm_direction.kp_z', 'Kp Z'),
            ('arm_direction.ki_z', 'Ki Z'),
            ('arm_direction.kd_z', 'Kd Z'),
            ('arm_direction.orthogonal_kp', 'Orthogonal Kp'),
        ),
    ),
    (
        'Arm Twist PID',
        (
            ('arm_pid_twist.Kp_linear_x', 'Kp linear X'),
            ('arm_pid_twist.Ki_linear_x', 'Ki linear X'),
            ('arm_pid_twist.Kd_linear_x', 'Kd linear X'),
            ('arm_pid_twist.Kp_linear_y', 'Kp linear Y'),
            ('arm_pid_twist.Ki_linear_y', 'Ki linear Y'),
            ('arm_pid_twist.Kd_linear_y', 'Kd linear Y'),
            ('arm_pid_twist.Kp_linear_z', 'Kp linear Z'),
            ('arm_pid_twist.Ki_linear_z', 'Ki linear Z'),
            ('arm_pid_twist.Kd_linear_z', 'Kd linear Z'),
            ('arm_pid_twist.Kp_angular_x', 'Kp angular X'),
            ('arm_pid_twist.Ki_angular_x', 'Ki angular X'),
            ('arm_pid_twist.Kd_angular_x', 'Kd angular X'),
            ('arm_pid_twist.Kp_angular_y', 'Kp angular Y'),
            ('arm_pid_twist.Ki_angular_y', 'Ki angular Y'),
            ('arm_pid_twist.Kd_angular_y', 'Kd angular Y'),
            ('arm_pid_twist.Kp_angular_z', 'Kp angular Z'),
            ('arm_pid_twist.Ki_angular_z', 'Ki angular Z'),
            ('arm_pid_twist.Kd_angular_z', 'Kd angular Z'),
        ),
    ),
    (
        'Arm Orientation',
        (
            ('arm_orientation.kp_orientation', 'Kp orientation'),
            ('arm_orientation.ki_orientation', 'Ki orientation'),
            ('arm_orientation.kd_orientation', 'Kd orientation'),
        ),
    ),
    (
        'Arm Move To Start',
        (
            ('arm_move.kp_linear', 'Kp linear'),
            ('arm_move.kp_angular', 'Kp angular'),
        ),
    ),
)

PLATFORM_PROFILES = {
    'robotnik': {
        'label': 'Robotnik',
        'path_topic': '/base_path',
        'robot_pose_topic': '/robot_pose',
        'odom_topic': '/robot/robotnik_base_control/odom',
        'cmd_vel_topic': '/robot/robotnik_base_control/cmd_vel_unstamped',
        'output_stamped': False,
        'command_frame_id': 'base_link',
        'path_frame': 'robotnik_simple',
        'external_map_frame': 'map',
        'robot_base_frame': 'base_link',
        'robot_tree_root_frame': 'odom',
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
        'odom_topic': '/odom',
        'cmd_vel_topic': '/diff_drive_controller/cmd_vel',
        'output_stamped': True,
        'command_frame_id': 'base_footprint',
        'path_frame': 'map',
        'external_map_frame': 'map',
        'robot_base_frame': 'base_footprint',
        'robot_tree_root_frame': 'odom',
        'max_vx': 0.25,
        'max_vy': 0.0,
        'max_wz': 0.6,
        'move_max_linear': 0.2,
        'move_max_lateral': 0.0,
        'move_max_angular': 0.5,
    },
}


class PidGainsDialog(QDialog):

    def __init__(self, parent: 'OperatorWindow', gains: dict[str, float]) -> None:
        super().__init__(parent)
        self.setWindowTitle('PID Gains')
        self.resize(640, 720)
        self._parent = parent
        self._spins: dict[str, QDoubleSpinBox] = {}

        layout = QVBoxLayout(self)
        for group_name, fields in PID_GAIN_GROUPS:
            group = QGroupBox(group_name)
            grid = QGridLayout(group)
            for index, (key, label) in enumerate(fields):
                spin = QDoubleSpinBox()
                spin.setRange(-10000.0, 10000.0)
                spin.setDecimals(4)
                spin.setSingleStep(0.05)
                spin.setValue(float(gains.get(key, DEFAULT_PID_GAINS[key])))
                self._spins[key] = spin
                row = index // 3
                column = (index % 3) * 2
                grid.addWidget(QLabel(label), row, column)
                grid.addWidget(spin, row, column + 1)
            layout.addWidget(group)

        button_box = QDialogButtonBox(
            QDialogButtonBox.Save | QDialogButtonBox.Cancel | QDialogButtonBox.Reset
        )
        button_box.accepted.connect(self._save)
        button_box.rejected.connect(self.reject)
        reset_button = button_box.button(QDialogButtonBox.Reset)
        if reset_button is not None:
            reset_button.clicked.connect(self._reset_defaults)
        layout.addWidget(button_box)

    def configured_gains(self) -> dict[str, float]:
        return {key: float(spin.value()) for key, spin in self._spins.items()}

    def _reset_defaults(self) -> None:
        for key, spin in self._spins.items():
            spin.setValue(DEFAULT_PID_GAINS[key])

    def _save(self) -> None:
        self._parent._set_pid_gains(self.configured_gains())
        self.accept()


class BaseSmoothingDialog(QDialog):

    def __init__(self, parent: 'OperatorWindow', settings: dict[str, float | bool | int | str]) -> None:
        super().__init__(parent)
        self.setWindowTitle('Base Velocity Smoothing')
        self.resize(420, 320)
        self._parent = parent

        layout = QVBoxLayout(self)
        group = QGroupBox('Base Follower Command Limits')
        grid = QGridLayout(group)

        self.enabled_checkbox = QCheckBox('Smooth velocity commands')
        self.enabled_checkbox.setChecked(bool(settings.get('enabled', DEFAULT_BASE_SMOOTHING['enabled'])))
        grid.addWidget(self.enabled_checkbox, 0, 0, 1, 2)

        self.method_combo = QComboBox()
        for method, label in BASE_SMOOTHING_METHODS:
            self.method_combo.addItem(label, method)
        method = str(settings.get('method', DEFAULT_BASE_SMOOTHING['method']))
        method_index = self.method_combo.findData(method)
        self.method_combo.setCurrentIndex(max(0, method_index))

        self.max_accel_x_spin = self._make_accel_spin(
            float(settings.get('max_accel_x', DEFAULT_BASE_SMOOTHING['max_accel_x'])),
            ' m/s^2',
        )
        self.max_accel_y_spin = self._make_accel_spin(
            float(settings.get('max_accel_y', DEFAULT_BASE_SMOOTHING['max_accel_y'])),
            ' m/s^2',
        )
        self.max_accel_wz_spin = self._make_accel_spin(
            float(settings.get('max_accel_wz', DEFAULT_BASE_SMOOTHING['max_accel_wz'])),
            ' rad/s^2',
        )
        self.moving_average_window_spin = QSpinBox()
        self.moving_average_window_spin.setRange(1, 100)
        self.moving_average_window_spin.setValue(
            int(settings.get(
                'moving_average_window_size',
                DEFAULT_BASE_SMOOTHING['moving_average_window_size'],
            ))
        )
        self.external_path_index_stride_spin = QSpinBox()
        self.external_path_index_stride_spin.setRange(1, 1000)
        self.external_path_index_stride_spin.setValue(
            int(settings.get(
                'external_path_index_stride',
                DEFAULT_BASE_SMOOTHING['external_path_index_stride'],
            ))
        )

        grid.addWidget(QLabel('Method'), 1, 0)
        grid.addWidget(self.method_combo, 1, 1)
        grid.addWidget(QLabel('Moving average samples'), 2, 0)
        grid.addWidget(self.moving_average_window_spin, 2, 1)
        grid.addWidget(QLabel('Base path index stride'), 3, 0)
        grid.addWidget(self.external_path_index_stride_spin, 3, 1)
        grid.addWidget(QLabel('Max accel X'), 4, 0)
        grid.addWidget(self.max_accel_x_spin, 4, 1)
        grid.addWidget(QLabel('Max accel Y'), 5, 0)
        grid.addWidget(self.max_accel_y_spin, 5, 1)
        grid.addWidget(QLabel('Max accel yaw'), 6, 0)
        grid.addWidget(self.max_accel_wz_spin, 6, 1)
        layout.addWidget(group)

        button_box = QDialogButtonBox(
            QDialogButtonBox.Save | QDialogButtonBox.Cancel | QDialogButtonBox.Reset
        )
        button_box.accepted.connect(self._save)
        button_box.rejected.connect(self.reject)
        reset_button = button_box.button(QDialogButtonBox.Reset)
        if reset_button is not None:
            reset_button.clicked.connect(self._reset_defaults)
        layout.addWidget(button_box)

    @staticmethod
    def _make_accel_spin(value: float, suffix: str) -> QDoubleSpinBox:
        spin = QDoubleSpinBox()
        spin.setRange(0.0, 10.0)
        spin.setDecimals(3)
        spin.setSingleStep(0.05)
        spin.setSuffix(suffix)
        spin.setValue(value)
        return spin

    def configured_settings(self) -> dict[str, float | bool | int | str]:
        return {
            'enabled': self.enabled_checkbox.isChecked(),
            'method': str(self.method_combo.currentData()),
            'max_accel_x': float(self.max_accel_x_spin.value()),
            'max_accel_y': float(self.max_accel_y_spin.value()),
            'max_accel_wz': float(self.max_accel_wz_spin.value()),
            'moving_average_window_size': int(self.moving_average_window_spin.value()),
            'external_path_index_stride': int(self.external_path_index_stride_spin.value()),
        }

    def _reset_defaults(self) -> None:
        self.enabled_checkbox.setChecked(bool(DEFAULT_BASE_SMOOTHING['enabled']))
        method_index = self.method_combo.findData(DEFAULT_BASE_SMOOTHING['method'])
        self.method_combo.setCurrentIndex(max(0, method_index))
        self.max_accel_x_spin.setValue(float(DEFAULT_BASE_SMOOTHING['max_accel_x']))
        self.max_accel_y_spin.setValue(float(DEFAULT_BASE_SMOOTHING['max_accel_y']))
        self.max_accel_wz_spin.setValue(float(DEFAULT_BASE_SMOOTHING['max_accel_wz']))
        self.moving_average_window_spin.setValue(
            int(DEFAULT_BASE_SMOOTHING['moving_average_window_size'])
        )
        self.external_path_index_stride_spin.setValue(
            int(DEFAULT_BASE_SMOOTHING['external_path_index_stride'])
        )

    def _save(self) -> None:
        self._parent._set_base_smoothing(self.configured_settings())
        self.accept()


class OperatorWindow(QMainWindow):
    ros_status_changed = pyqtSignal(bool, bool, bool, bool, bool)
    path_index_changed = pyqtSignal(int)
    process_output = pyqtSignal(str, str)

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle('AM Operator GUI')
        self.resize(1280, 720)
        self._has_path = False
        self._has_robot_pose = False
        self._has_arm_pose = False
        self._jparse_ready = False
        self._controller_ready = False
        self._launch_all_active = False
        self._launch_all_timers: list[QTimer] = []
        self._config = self._load_config()
        self._pid_gains_dialog: Optional[PidGainsDialog] = None
        self._base_smoothing_dialog: Optional[BaseSmoothingDialog] = None

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
        for config_path in (CONFIG_PATH, LEGACY_CONFIG_PATH):
            try:
                with config_path.open('r', encoding='utf-8') as config_file:
                    data = json.load(config_file)
            except FileNotFoundError:
                continue
            except (OSError, json.JSONDecodeError):
                return {}
            return data if isinstance(data, dict) else {}
        return {}

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

    def _configured_base_move_velocity(self, key: str) -> float:
        profile = self._current_platform_profile()
        defaults = {
            'max_linear': float(profile['move_max_linear']),
            'max_lateral': float(profile['move_max_lateral']),
            'max_angular': float(profile['move_max_angular']),
        }
        configured_by_platform = self._config.get('base_move_velocity', {})
        configured = {}
        if isinstance(configured_by_platform, dict):
            platform_config = configured_by_platform.get(self._current_platform_key(), {})
            if isinstance(platform_config, dict):
                configured = platform_config
        try:
            value = float(configured.get(key, defaults[key]))
        except (KeyError, TypeError, ValueError):
            value = defaults[key]
        return max(0.0, min(10.0, value))

    def _configured_path_transform(self) -> dict[str, float]:
        configured = self._config.get('path_transform', {})
        transform = dict(DEFAULT_PATH_TRANSFORM)
        if not isinstance(configured, dict):
            return transform
        for key, default in DEFAULT_PATH_TRANSFORM.items():
            try:
                transform[key] = float(configured.get(key, default))
            except (TypeError, ValueError):
                transform[key] = default
        return transform

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

    def _configured_use_odometry_robot_pose(self) -> bool:
        return bool(self._config.get('use_odometry_robot_pose', False))

    def _configured_base_pose_topic(self) -> str:
        return str(self._config.get('base_pose_topic', '/vicon/Base_RB/Base_RB'))

    def _configured_arm_pose_topic(self) -> str:
        topic = str(self._config.get('arm_pose_topic', '/vicon/tool_transformed')).strip()
        if topic in {'vicon/Tool_Flange/Tool_Flange', '/vicon/Tool_Flange/Tool_Flange'}:
            return '/vicon/tool_transformed'
        return topic

    def _configured_external_map_frame(self) -> str:
        configured = str(self._config.get('external_map_frame', '')).strip()
        return configured or str(self._current_platform_profile()['external_map_frame'])

    def _configured_robot_base_frame(self) -> str:
        configured = str(self._config.get('robot_base_frame', '')).strip()
        return configured or str(self._current_platform_profile()['robot_base_frame'])

    def _configured_robot_tree_root_frame(self) -> str:
        configured = str(self._config.get('robot_tree_root_frame', '')).strip()
        return configured or str(self._current_platform_profile()['robot_tree_root_frame'])

    def _configured_control_frame(self) -> str:
        configured = str(self._config.get('control_frame', '')).strip()
        return configured or self._path_frame_from_folder(self._configured_trajectory_directory())

    def _configured_trajectory_directory(self) -> Path:
        configured = str(self._config.get('trajectory_directory', '')).strip()
        return Path(configured).expanduser() if configured else DEFAULT_TRAJECTORY_DIR

    def _configured_pid_gains(self) -> dict[str, float]:
        configured = self._config.get('pid_gains', {})
        gains = dict(DEFAULT_PID_GAINS)
        if not isinstance(configured, dict):
            return gains
        for key, default in DEFAULT_PID_GAINS.items():
            try:
                gains[key] = float(configured.get(key, default))
            except (TypeError, ValueError):
                gains[key] = default
        return gains

    def _configured_base_smoothing(self) -> dict[str, float | bool | int | str]:
        configured = self._config.get('base_smoothing', {})
        settings = dict(DEFAULT_BASE_SMOOTHING)
        if not isinstance(configured, dict):
            return settings
        settings['enabled'] = bool(configured.get('enabled', DEFAULT_BASE_SMOOTHING['enabled']))
        method = str(configured.get('method', DEFAULT_BASE_SMOOTHING['method'])).strip().lower()
        valid_methods = {method_key for method_key, _label in BASE_SMOOTHING_METHODS}
        settings['method'] = method if method in valid_methods else DEFAULT_BASE_SMOOTHING['method']
        for key in ('max_accel_x', 'max_accel_y', 'max_accel_wz'):
            try:
                value = float(configured.get(key, DEFAULT_BASE_SMOOTHING[key]))
            except (TypeError, ValueError):
                value = float(DEFAULT_BASE_SMOOTHING[key])
            settings[key] = max(0.0, min(10.0, value))
        try:
            window_size = int(configured.get(
                'moving_average_window_size',
                DEFAULT_BASE_SMOOTHING['moving_average_window_size'],
            ))
        except (TypeError, ValueError):
            window_size = int(DEFAULT_BASE_SMOOTHING['moving_average_window_size'])
        settings['moving_average_window_size'] = max(1, min(100, window_size))
        try:
            stride = int(configured.get(
                'external_path_index_stride',
                DEFAULT_BASE_SMOOTHING['external_path_index_stride'],
            ))
        except (TypeError, ValueError):
            stride = int(DEFAULT_BASE_SMOOTHING['external_path_index_stride'])
        settings['external_path_index_stride'] = max(1, min(1000, stride))
        return settings

    def _build_ui(self) -> None:
        root = QWidget()
        layout = QHBoxLayout(root)
        layout.setSpacing(10)

        splitter = QSplitter(Qt.Horizontal)
        controls_panel = QWidget()
        controls_layout = QVBoxLayout(controls_panel)
        controls_layout.setSpacing(10)
        controls_layout.setContentsMargins(0, 0, 0, 0)

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
        self.odometry_pose_checkbox = QCheckBox('Use odometry for /robot_pose')
        self.odometry_pose_checkbox.setChecked(self._configured_use_odometry_robot_pose())
        self.direction_mode = QComboBox()
        self.direction_mode.addItems(['goal_direction', 'speed_orthogonal'])
        self.direction_mode.setCurrentText('goal_direction')
        self.index_spin = QSpinBox()
        self.index_spin.setRange(0, 100000)
        self.index_spin.setValue(0)

        self.path_folder = QLineEdit(str(self._configured_trajectory_directory()))
        self.path_folder.setReadOnly(True)
        self.browse_button = QPushButton('Browse')
        path_transform = self._configured_path_transform()
        self.path_transform_x_spin = QDoubleSpinBox()
        self.path_transform_y_spin = QDoubleSpinBox()
        self.path_transform_z_spin = QDoubleSpinBox()
        self.path_transform_yaw_spin = QDoubleSpinBox()
        for spin, key in (
            (self.path_transform_x_spin, 'x'),
            (self.path_transform_y_spin, 'y'),
            (self.path_transform_z_spin, 'z'),
        ):
            spin.setRange(-10000.0, 10000.0)
            spin.setDecimals(4)
            spin.setSingleStep(0.01)
            spin.setSuffix(' m')
            spin.setValue(path_transform[key])
        self.path_transform_yaw_spin.setRange(-36000.0, 36000.0)
        self.path_transform_yaw_spin.setDecimals(3)
        self.path_transform_yaw_spin.setSingleStep(1.0)
        self.path_transform_yaw_spin.setSuffix(' deg')
        self.path_transform_yaw_spin.setValue(path_transform['yaw_deg'])
        self.calculate_path_transform_button = QPushButton('Calculate Path Transform')

        self.launch_button = QPushButton('Launch All')
        self.launch_sim_button = QPushButton('Launch Sim')
        self.publish_path_button = QPushButton('Publish Path')
        self.rviz_button = QPushButton('Open RViz')
        self.pid_gains_button = QPushButton('PID Gains...')
        self.base_smoothing_button = QPushButton('Base Smoothing...')
        self.sync_workspace_button = QPushButton('Sync Workspace')

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
        launch_layout.addWidget(self.odometry_pose_checkbox, 1, 4, 1, 2)
        launch_layout.addWidget(QLabel('Path folder'), 2, 0)
        launch_layout.addWidget(self.path_folder, 2, 1, 1, 4)
        launch_layout.addWidget(self.browse_button, 2, 5)
        launch_layout.addWidget(QLabel('Path transform X'), 3, 0)
        launch_layout.addWidget(self.path_transform_x_spin, 3, 1)
        launch_layout.addWidget(QLabel('Y'), 3, 2)
        launch_layout.addWidget(self.path_transform_y_spin, 3, 3)
        launch_layout.addWidget(QLabel('Z'), 3, 4)
        launch_layout.addWidget(self.path_transform_z_spin, 3, 5)
        launch_layout.addWidget(QLabel('Path rotation'), 4, 0)
        launch_layout.addWidget(self.path_transform_yaw_spin, 4, 1)
        launch_layout.addWidget(self.calculate_path_transform_button, 4, 2, 1, 2)
        self.base_pose_topic = QLineEdit(self._configured_base_pose_topic())
        self.arm_pose_topic = QLineEdit(self._configured_arm_pose_topic())
        self.control_frame = QLineEdit(self._configured_control_frame())
        self.external_map_frame = QLineEdit(self._configured_external_map_frame())
        self.robot_base_frame = QLineEdit(self._configured_robot_base_frame())
        self.robot_tree_root_frame = QLineEdit(self._configured_robot_tree_root_frame())
        launch_layout.addWidget(QLabel('Vicon base marker topic'), 5, 0)
        launch_layout.addWidget(self.base_pose_topic, 5, 1, 1, 2)
        launch_layout.addWidget(QLabel('EE pose topic'), 5, 3)
        launch_layout.addWidget(self.arm_pose_topic, 5, 4, 1, 2)
        launch_layout.addWidget(QLabel('Control frame'), 6, 0)
        launch_layout.addWidget(self.control_frame, 6, 1, 1, 2)
        launch_layout.addWidget(QLabel('External map'), 6, 3)
        launch_layout.addWidget(self.external_map_frame, 6, 4, 1, 2)
        launch_layout.addWidget(QLabel('Robot base frame'), 7, 0)
        launch_layout.addWidget(self.robot_base_frame, 7, 1, 1, 2)
        launch_layout.addWidget(QLabel('Robot TF root'), 7, 3)
        launch_layout.addWidget(self.robot_tree_root_frame, 7, 4, 1, 2)
        launch_layout.addWidget(self.launch_button, 8, 0)
        launch_layout.addWidget(self.launch_sim_button, 8, 1)
        launch_layout.addWidget(self.rviz_button, 8, 2)
        launch_layout.addWidget(self.pid_gains_button, 8, 3)
        launch_layout.addWidget(self.base_smoothing_button, 8, 4)
        launch_layout.addWidget(self.sync_workspace_button, 8, 5)

        component_group = QGroupBox('Components')
        component_layout = QGridLayout(component_group)
        self.base_follower_button = QPushButton('Launch Base Follower')
        self.arm_follower_button = QPushButton('Launch Arm Follower')
        self.path_index_button = QPushButton('Launch Path Index')
        self.current_tcp_pose_button = QPushButton('Launch Transformations')
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
        self.base_move_linear_velocity_spin = QDoubleSpinBox()
        self.base_move_lateral_velocity_spin = QDoubleSpinBox()
        self.base_move_angular_velocity_spin = QDoubleSpinBox()
        for spin in (
            self.base_move_linear_velocity_spin,
            self.base_move_lateral_velocity_spin,
            self.base_move_angular_velocity_spin,
        ):
            spin.setRange(0.0, 10.0)
            spin.setDecimals(3)
            spin.setSingleStep(0.01)
        self.base_move_linear_velocity_spin.setSuffix(' m/s')
        self.base_move_lateral_velocity_spin.setSuffix(' m/s')
        self.base_move_angular_velocity_spin.setSuffix(' rad/s')
        self._load_base_move_velocity_controls()

        motion_layout.addWidget(self.move_base_button, 0, 0)
        motion_layout.addWidget(self.move_arm_button, 0, 1)
        motion_layout.addWidget(self.start_following_button, 1, 0)
        motion_layout.addWidget(self.stop_following_button, 1, 1)
        motion_layout.addWidget(QLabel('Base start linear'), 2, 0)
        motion_layout.addWidget(self.base_move_linear_velocity_spin, 2, 1)
        motion_layout.addWidget(QLabel('Base start lateral'), 3, 0)
        motion_layout.addWidget(self.base_move_lateral_velocity_spin, 3, 1)
        motion_layout.addWidget(QLabel('Base start angular'), 4, 0)
        motion_layout.addWidget(self.base_move_angular_velocity_spin, 4, 1)

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
        self.log.setMinimumWidth(420)

        log_group = QGroupBox('Console')
        log_layout = QVBoxLayout(log_group)
        log_layout.addWidget(self.log)

        controls_layout.addWidget(launch_group)
        controls_layout.addWidget(component_group)
        controls_layout.addWidget(motion_group)
        controls_layout.addWidget(override_group)
        controls_layout.addWidget(status_group)
        controls_layout.addStretch(1)

        splitter.addWidget(controls_panel)
        splitter.addWidget(log_group)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)
        splitter.setSizes([760, 520])

        layout.addWidget(splitter)
        self.setCentralWidget(root)

    def _connect_signals(self) -> None:
        self.browse_button.clicked.connect(self._choose_path_folder)
        self.base_pose_topic.editingFinished.connect(self._save_hardware_topics)
        self.arm_pose_topic.editingFinished.connect(self._save_hardware_topics)
        self.control_frame.editingFinished.connect(self._save_hardware_topics)
        self.external_map_frame.editingFinished.connect(self._save_hardware_topics)
        self.robot_base_frame.editingFinished.connect(self._save_hardware_topics)
        self.robot_tree_root_frame.editingFinished.connect(self._save_hardware_topics)
        self.simulation_checkbox.toggled.connect(self._simulation_mode_changed)
        self.platform_combo.currentIndexChanged.connect(self._set_platform)
        self.follower_type_combo.currentIndexChanged.connect(self._set_follower_type)
        self.diff_drive_checkbox.toggled.connect(self._set_diff_drive_mode)
        self.odometry_pose_checkbox.toggled.connect(self._set_use_odometry_robot_pose)
        self.launch_button.clicked.connect(self._toggle_launch_all)
        self.launch_sim_button.clicked.connect(self._toggle_sim)
        self.pid_gains_button.clicked.connect(self._open_pid_gains_window)
        self.base_smoothing_button.clicked.connect(self._open_base_smoothing_window)
        self.calculate_path_transform_button.clicked.connect(self._calculate_path_transform)
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
        self.sync_workspace_button.clicked.connect(self._sync_workspace)
        self.index_spin.valueChanged.connect(self._publish_path_index)
        self.velocity_slider.valueChanged.connect(self._publish_overrides)
        self.path_index_rate_spin.valueChanged.connect(self._set_path_index_rate)
        self.default_velocity_checkbox.toggled.connect(self._set_default_velocity_enabled)
        self.default_velocity_spin.valueChanged.connect(self._set_default_velocity)
        self.base_move_linear_velocity_spin.valueChanged.connect(self._set_base_move_velocity)
        self.base_move_lateral_velocity_spin.valueChanged.connect(self._set_base_move_velocity)
        self.base_move_angular_velocity_spin.valueChanged.connect(self._set_base_move_velocity)
        self.path_transform_x_spin.valueChanged.connect(self._set_path_transform)
        self.path_transform_y_spin.valueChanged.connect(self._set_path_transform)
        self.path_transform_z_spin.valueChanged.connect(self._set_path_transform)
        self.path_transform_yaw_spin.valueChanged.connect(self._set_path_transform)
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
        self._load_base_move_velocity_controls()
        profile = self._current_platform_profile()
        self.path_status.setText(f"{profile['path_topic']}: ready" if self._has_path else f"{profile['path_topic']}: waiting")

    def _set_follower_type(self, *_args) -> None:
        self._config['follower_type'] = self._current_follower_type()
        self._save_config()

    def _set_diff_drive_mode(self, enabled: bool) -> None:
        self._config['diff_drive_mode'] = bool(enabled)
        self._save_config()

    def _set_use_odometry_robot_pose(self, enabled: bool) -> None:
        self._config['use_odometry_robot_pose'] = bool(enabled)
        self._save_config()

    def _set_path_transform(self, *_args) -> None:
        self._config['path_transform'] = {
            'x': float(self.path_transform_x_spin.value()),
            'y': float(self.path_transform_y_spin.value()),
            'z': float(self.path_transform_z_spin.value()),
            'yaw_deg': float(self.path_transform_yaw_spin.value()),
        }
        self._save_config()

    def _set_path_transform_values(self, x: float, y: float, z: float, yaw_deg: float) -> None:
        for spin, value in (
            (self.path_transform_x_spin, x),
            (self.path_transform_y_spin, y),
            (self.path_transform_z_spin, z),
            (self.path_transform_yaw_spin, yaw_deg),
        ):
            spin.blockSignals(True)
            spin.setValue(float(value))
            spin.blockSignals(False)
        self._set_path_transform()

    def _calculate_path_transform(self) -> None:
        index = self.index_spin.value()
        path_pose = self.ros_bridge.latest_base_path_pose(index)
        robot_pose = self.ros_bridge.latest_robot_pose()
        if path_pose is None:
            self._append_process_output(
                'gui',
                f'cannot calculate path transform: no /base_path pose at index {index}',
            )
            return
        if robot_pose is None:
            self._append_process_output(
                'gui',
                'cannot calculate path transform: no fresh /robot_pose available',
            )
            return

        x, y, z, yaw_deg = self._composed_path_transform(
            {
                'x': self.path_transform_x_spin.value(),
                'y': self.path_transform_y_spin.value(),
                'z': self.path_transform_z_spin.value(),
                'yaw_deg': self.path_transform_yaw_spin.value(),
            },
            path_pose,
            robot_pose,
        )
        self._set_path_transform_values(x, y, z, yaw_deg)
        self._append_process_output(
            'gui',
            (
                f'calculated path transform from /base_path[{index}] to current base pose: '
                f'x={x:.4f} m y={y:.4f} m z={z:.4f} m yaw={yaw_deg:.3f} deg'
            ),
        )

        process = self.processes.get(PUBLISH_PATH_NAME)
        if process is not None and process.is_running():
            self.processes.stop(PUBLISH_PATH_NAME)
            self._start_publish_path()
            self._refresh_process_states()

    @classmethod
    def _composed_path_transform(
        cls,
        current_transform: dict[str, float],
        path_pose,
        robot_pose,
    ) -> tuple[float, float, float, float]:
        path_position = path_pose.pose.position
        robot_position = robot_pose.pose.position
        path_yaw = cls._yaw_from_orientation(path_pose.pose.orientation)
        robot_yaw = cls._yaw_from_orientation(robot_pose.pose.orientation)
        delta_yaw = robot_yaw - path_yaw
        delta_x, delta_y, delta_z = cls._inverse_transformed_pose_delta(
            path_position,
            robot_position,
            delta_yaw,
        )

        current_yaw = math.radians(float(current_transform.get('yaw_deg', 0.0)))
        current_x = float(current_transform.get('x', 0.0))
        current_y = float(current_transform.get('y', 0.0))
        current_z = float(current_transform.get('z', 0.0))
        rotated_x, rotated_y = cls._rotate_xy(current_x, current_y, delta_yaw)
        yaw_deg = math.degrees(cls._normalize_angle(current_yaw + delta_yaw))
        return (
            rotated_x + delta_x,
            rotated_y + delta_y,
            current_z + delta_z,
            yaw_deg,
        )

    @staticmethod
    def _inverse_transformed_pose_delta(path_position, robot_position, delta_yaw: float) -> tuple[float, float, float]:
        rotated_path_x, rotated_path_y = OperatorWindow._rotate_xy(
            float(path_position.x),
            float(path_position.y),
            delta_yaw,
        )
        return (
            float(robot_position.x) - rotated_path_x,
            float(robot_position.y) - rotated_path_y,
            float(robot_position.z) - float(path_position.z),
        )

    @staticmethod
    def _rotate_xy(x: float, y: float, yaw: float) -> tuple[float, float]:
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        return (
            cos_yaw * float(x) - sin_yaw * float(y),
            sin_yaw * float(x) + cos_yaw * float(y),
        )

    @staticmethod
    def _yaw_from_orientation(orientation) -> float:
        x = float(orientation.x)
        y = float(orientation.y)
        z = float(orientation.z)
        w = float(orientation.w)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def _choose_path_folder(self) -> None:
        folder = QFileDialog.getExistingDirectory(
            self,
            'Select trajectory folder',
            self.path_folder.text().strip() or str(DEFAULT_COMPONENTS_DIR),
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
        self._config['trajectory_directory'] = self.path_folder.text().strip()
        self._config['base_pose_topic'] = self.base_pose_topic.text().strip()
        self._config['arm_pose_topic'] = self.arm_pose_topic.text().strip()
        self._config['control_frame'] = self.control_frame.text().strip()
        self._config['external_map_frame'] = self.external_map_frame.text().strip()
        self._config['robot_base_frame'] = self.robot_base_frame.text().strip()
        self._config['robot_tree_root_frame'] = self.robot_tree_root_frame.text().strip()
        self._save_config()

    def _open_pid_gains_window(self) -> None:
        if self._pid_gains_dialog is not None and self._pid_gains_dialog.isVisible():
            self._pid_gains_dialog.raise_()
            self._pid_gains_dialog.activateWindow()
            return
        self._pid_gains_dialog = PidGainsDialog(self, self._configured_pid_gains())
        self._pid_gains_dialog.show()

    def _set_pid_gains(self, gains: dict[str, float]) -> None:
        self._config['pid_gains'] = {
            key: float(gains.get(key, DEFAULT_PID_GAINS[key]))
            for key in DEFAULT_PID_GAINS
        }
        self._save_config()
        self._append_process_output('gui', 'saved PID gains; restart affected controllers to apply')

    def _pid_gain(self, key: str) -> float:
        return self._configured_pid_gains()[key]

    def _open_base_smoothing_window(self) -> None:
        if self._base_smoothing_dialog is not None and self._base_smoothing_dialog.isVisible():
            self._base_smoothing_dialog.raise_()
            self._base_smoothing_dialog.activateWindow()
            return

        self._base_smoothing_dialog = BaseSmoothingDialog(self, self._configured_base_smoothing())
        self._base_smoothing_dialog.show()

    def _set_base_smoothing(self, settings: dict[str, float | bool | int | str]) -> None:
        current = self._configured_base_smoothing()
        current['enabled'] = bool(settings.get('enabled', current['enabled']))
        method = str(settings.get('method', current['method'])).strip().lower()
        valid_methods = {method_key for method_key, _label in BASE_SMOOTHING_METHODS}
        current['method'] = method if method in valid_methods else current['method']
        for key in ('max_accel_x', 'max_accel_y', 'max_accel_wz'):
            try:
                value = float(settings.get(key, current[key]))
            except (TypeError, ValueError):
                value = float(current[key])
            current[key] = max(0.0, min(10.0, value))
        try:
            window_size = int(settings.get('moving_average_window_size', current['moving_average_window_size']))
        except (TypeError, ValueError):
            window_size = int(current['moving_average_window_size'])
        current['moving_average_window_size'] = max(1, min(100, window_size))
        try:
            stride = int(settings.get('external_path_index_stride', current['external_path_index_stride']))
        except (TypeError, ValueError):
            stride = int(current['external_path_index_stride'])
        current['external_path_index_stride'] = max(1, min(1000, stride))
        self._config['base_smoothing'] = current
        self._save_config()
        self._append_process_output(
            'gui',
            'saved base smoothing settings; restart base follower to apply',
        )

    def _base_smoothing(self, key: str) -> float | bool | int | str:
        return self._configured_base_smoothing()[key]

    def _pid_launch_arguments(self, key_prefix: str, names: tuple[str, ...]) -> list[str]:
        return [
            f'{name}:={self._ros_float_literal(self._pid_gain(f"{key_prefix}.{name}"))}'
            for name in names
        ]

    def _pid_ros_parameters(self, key_prefix: str, names: tuple[str, ...]) -> list[str]:
        parameters = []
        for name in names:
            parameters.extend([
                '-p',
                f'{name}:={self._ros_float_literal(self._pid_gain(f"{key_prefix}.{name}"))}',
            ])
        return parameters

    def _path_transform_launch_arguments(self) -> list[str]:
        return [
            (
                'path_transform_xyz:='
                f'[{self._ros_float_literal(self.path_transform_x_spin.value())}, '
                f'{self._ros_float_literal(self.path_transform_y_spin.value())}, '
                f'{self._ros_float_literal(self.path_transform_z_spin.value())}]'
            ),
            (
                'path_transform_yaw_deg:='
                f'{self._ros_float_literal(self.path_transform_yaw_spin.value())}'
            ),
        ]

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
        if self.simulation_checkbox.isChecked() and self.odometry_pose_checkbox.isChecked():
            self._start_odometry_pose_adapter()
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
            VICON_BASE_STATIC_TF_NAME,
            BASE_POSE_ADAPTER_NAME,
            ODOMETRY_POSE_ADAPTER_NAME,
            ARM_POSE_ADAPTER_NAME,
            VICON_EE_STATIC_TF_NAME,
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
                f'publish_robot_pose:={self._sim_publish_robot_pose()}',
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
                f'publish_robot_pose:={self._sim_publish_robot_pose()}',
            ]
        self._append_process_output(SIM_NAME, ' '.join(command))
        self.processes.start(SIM_NAME, command)

    def _sim_publish_robot_pose(self) -> str:
        return 'false' if self.odometry_pose_checkbox.isChecked() else 'true'

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
        command.extend(self._path_transform_launch_arguments())
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
            '-p', f"external_path_index_stride:={int(self._base_smoothing('external_path_index_stride'))}",
            '-p', 'wait_for_start_condition:=true',
            '-p', 'start_condition_topic:=/start_condition',
            '-p', 'velocity_override_topic:=/velocity_override',
            '-p', 'lookahead_distance:=0.3',
            '-p', f"kp_x:={self._ros_float_literal(self._pid_gain('base_follower.kp_x'))}",
            '-p', f"kp_y:={self._ros_float_literal(self._pid_gain('base_follower.kp_y'))}",
            '-p', f"kp_yaw:={self._ros_float_literal(self._pid_gain('base_follower.kp_yaw'))}",
            '-p', f"max_vx:={self._ros_float_literal(float(profile['max_vx']))}",
            '-p', f"max_vy:={self._ros_float_literal(float(profile['max_vy']))}",
            '-p', f"max_wz:={self._ros_float_literal(float(profile['max_wz']))}",
            '-p', f"smooth_velocity_commands:={str(bool(self._base_smoothing('enabled'))).lower()}",
            '-p', f"velocity_smoothing_method:={self._base_smoothing('method')}",
            '-p', f"max_accel_x:={self._ros_float_literal(float(self._base_smoothing('max_accel_x')))}",
            '-p', f"max_accel_y:={self._ros_float_literal(float(self._base_smoothing('max_accel_y')))}",
            '-p', f"max_accel_wz:={self._ros_float_literal(float(self._base_smoothing('max_accel_wz')))}",
            '-p', f"moving_average_window_size:={int(self._base_smoothing('moving_average_window_size'))}",
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
        command.extend(self._pid_launch_arguments(
            'arm_direction',
            ('kp_z', 'ki_z', 'kd_z', 'orthogonal_kp'),
        ))
        command.extend(self._pid_launch_arguments(
            'arm_pid_twist',
            (
                'Kp_linear_x',
                'Ki_linear_x',
                'Kd_linear_x',
                'Kp_linear_y',
                'Ki_linear_y',
                'Kd_linear_y',
                'Kp_linear_z',
                'Ki_linear_z',
                'Kd_linear_z',
                'Kp_angular_x',
                'Ki_angular_x',
                'Kd_angular_x',
                'Kp_angular_y',
                'Ki_angular_y',
                'Kd_angular_y',
                'Kp_angular_z',
                'Ki_angular_z',
                'Kd_angular_z',
            ),
        ))
        command.extend(self._pid_launch_arguments(
            'arm_orientation',
            ('kp_orientation', 'ki_orientation', 'kd_orientation'),
        ))
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
        profile = self._current_platform_profile()
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
            '-p', f"path_topic:={profile['path_topic']}",
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
                for name in (
                    VICON_BASE_STATIC_TF_NAME,
                    VICON_EE_STATIC_TF_NAME,
                    BASE_POSE_ADAPTER_NAME,
                    ODOMETRY_POSE_ADAPTER_NAME,
                    ARM_POSE_ADAPTER_NAME,
                )
            )
            if running:
                self.processes.stop(VICON_BASE_STATIC_TF_NAME)
                self.processes.stop(VICON_EE_STATIC_TF_NAME)
                self.processes.stop(BASE_POSE_ADAPTER_NAME)
                self.processes.stop(ODOMETRY_POSE_ADAPTER_NAME)
                self.processes.stop(ARM_POSE_ADAPTER_NAME)
            else:
                self._start_pose_adapters()
            self._refresh_process_states()
            return
        process = self.processes.get(CURRENT_TCP_POSE_NAME)
        odom_process = self.processes.get(ODOMETRY_POSE_ADAPTER_NAME)
        odom_running = odom_process is not None and odom_process.is_running()
        if (process is not None and process.is_running()) or odom_running:
            self.processes.stop(CURRENT_TCP_POSE_NAME)
            self.processes.stop(ODOMETRY_POSE_ADAPTER_NAME)
            self._append_process_output(CURRENT_TCP_POSE_NAME, 'stopped by operator')
            self._refresh_process_states()
            return

        self._start_current_tcp_pose()
        if self.odometry_pose_checkbox.isChecked():
            self._start_odometry_pose_adapter()
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
        use_odometry_pose = self.odometry_pose_checkbox.isChecked()
        if not use_odometry_pose:
            base_static_command = [
                'ros2',
                'run',
                'tf2_ros',
                'static_transform_publisher',
                *VICON_BASE_STATIC_TF,
            ]
            self._append_process_output(VICON_BASE_STATIC_TF_NAME, ' '.join(base_static_command))
            self.processes.start(VICON_BASE_STATIC_TF_NAME, base_static_command)

        vicon_transform_command = [
            'ros2',
            'run',
            'am_operator_gui',
            'vicon_ee_static_tf',
            '--ros-args',
            '-p', f'use_sim_time:={self._use_sim_time()}',
            '-p', 'input_topic:=/vicon/Tool_Flange/Tool_Flange',
            '-p', 'output_topic:=/vicon/tool_transformed',
        ]
        self._append_process_output(VICON_EE_STATIC_TF_NAME, ' '.join(vicon_transform_command))
        self.processes.start(VICON_EE_STATIC_TF_NAME, vicon_transform_command)

        if use_odometry_pose:
            self._start_odometry_pose_adapter()
        else:
            base_command = [
                'ros2',
                'run',
                'am_operator_gui',
                'external_base_reference',
                '--ros-args',
                '-r', f'__node:={BASE_POSE_ADAPTER_NAME}',
                '-p', f'use_sim_time:={self._use_sim_time()}',
                '-p', f'input_topic:={self.base_pose_topic.text().strip()}',
                '-p', f'input_pose_frame:={VICON_BASE_REFERENCE_FRAME}',
                '-p', 'output_topic:=/robot_pose',
                '-p', f'map_frame:={self.external_map_frame.text().strip()}',
                '-p', f'robot_base_frame:={self.robot_base_frame.text().strip()}',
                '-p', f'robot_tree_root_frame:={self.robot_tree_root_frame.text().strip()}',
                '-p', 'ready_topic:=/am/base_pose_ready',
                '-p', 'stale_timeout:=0.5',
            ]
            self._append_process_output(BASE_POSE_ADAPTER_NAME, ' '.join(base_command))
            self.processes.start(BASE_POSE_ADAPTER_NAME, base_command)

        arm_command = [
            'ros2',
            'run',
            'am_operator_gui',
            'pose_stamped_adapter',
            '--ros-args',
            '-r', f'__node:={ARM_POSE_ADAPTER_NAME}',
            '-p', f'use_sim_time:={self._use_sim_time()}',
            '-p', f'input_topic:={self.arm_pose_topic.text().strip()}',
            '-p', 'output_topic:=/current_tcp_pose',
            '-p', f'target_frame:={self.control_frame.text().strip()}',
            '-p', 'ready_topic:=/am/arm_pose_ready',
            '-p', 'stale_timeout:=0.5',
        ]
        self._append_process_output(ARM_POSE_ADAPTER_NAME, ' '.join(arm_command))
        self.processes.start(ARM_POSE_ADAPTER_NAME, arm_command)

    def _start_odometry_pose_adapter(self) -> None:
        profile = self._current_platform_profile()
        base_command = [
            'ros2',
            'run',
            'am_operator_gui',
            'odometry_robot_pose',
            '--ros-args',
            '-r', f'__node:={ODOMETRY_POSE_ADAPTER_NAME}',
            '-p', f'use_sim_time:={self._use_sim_time()}',
            '-p', f"odom_topic:={profile['odom_topic']}",
            '-p', f"path_topic:={profile['path_topic']}",
            '-p', 'output_topic:=/robot_pose',
            '-p', f'initial_path_index:={self.index_spin.value()}',
            '-p', f'map_frame:={self.external_map_frame.text().strip()}',
            '-p', f'odom_frame:={self.robot_tree_root_frame.text().strip()}',
            '-p', f'robot_base_frame:={self.robot_base_frame.text().strip()}',
            '-p', 'ready_topic:=/am/base_pose_ready',
            '-p', 'stale_timeout:=0.5',
            '-p', 'publish_tf:=true',
        ]
        self._append_process_output(ODOMETRY_POSE_ADAPTER_NAME, ' '.join(base_command))
        self.processes.start(ODOMETRY_POSE_ADAPTER_NAME, base_command)

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
            '-p', f"kp_linear:={self._ros_float_literal(self._pid_gain('base_move.kp_linear'))}",
            '-p', f"kp_lateral:={self._ros_float_literal(self._pid_gain('base_move.kp_lateral'))}",
            '-p', f"kp_angular_to_point:={self._ros_float_literal(self._pid_gain('base_move.kp_angular_to_point'))}",
            '-p', f"kp_angular_reorient:={self._ros_float_literal(self._pid_gain('base_move.kp_angular_reorient'))}",
            '-p', f"max_linear_velocity:={self._ros_float_literal(self._base_move_velocity('max_linear'))}",
            '-p', f"max_lateral_velocity:={self._ros_float_literal(self._base_move_velocity('max_lateral'))}",
            '-p', f"max_angular_velocity:={self._ros_float_literal(self._base_move_velocity('max_angular'))}",
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
        command.extend(self._pid_launch_arguments(
            'arm_move',
            ('kp_linear', 'kp_angular'),
        ))
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
        missing_processes = self._missing_control_process_names()
        if missing_processes:
            self._append_process_output(
                'safety',
                'starting following with missing control process(es): '
                + ', '.join(missing_processes),
            )
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

    def _sync_workspace(self) -> None:
        process = self.processes.get(SYNC_WORKSPACE_NAME)
        if process is not None and process.is_running():
            self.processes.stop(SYNC_WORKSPACE_NAME)
            self._append_process_output(SYNC_WORKSPACE_NAME, 'stopped by operator')
            self._refresh_process_states()
            return

        self._start_sync_workspace()
        self._refresh_process_states()

    def _start_sync_workspace(self) -> None:
        command = [
            'rsync',
            '-az',
            '-e',
            'ssh',
            f'{WORKSPACE_SRC_ROOT}/',
            SYNC_REMOTE_TARGET,
        ]
        self._append_process_output(SYNC_WORKSPACE_NAME, ' '.join(command))
        self.processes.start(SYNC_WORKSPACE_NAME, command)

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

    def _load_base_move_velocity_controls(self) -> None:
        controls = (
            (self.base_move_linear_velocity_spin, 'max_linear'),
            (self.base_move_lateral_velocity_spin, 'max_lateral'),
            (self.base_move_angular_velocity_spin, 'max_angular'),
        )
        for spin, key in controls:
            spin.blockSignals(True)
            spin.setValue(self._configured_base_move_velocity(key))
            spin.blockSignals(False)

    def _set_base_move_velocity(self, *_args) -> None:
        configured_by_platform = self._config.get('base_move_velocity', {})
        if not isinstance(configured_by_platform, dict):
            configured_by_platform = {}
        configured_by_platform[self._current_platform_key()] = {
            'max_linear': float(self.base_move_linear_velocity_spin.value()),
            'max_lateral': float(self.base_move_lateral_velocity_spin.value()),
            'max_angular': float(self.base_move_angular_velocity_spin.value()),
        }
        self._config['base_move_velocity'] = configured_by_platform
        self._save_config()

    def _base_move_velocity(self, key: str) -> float:
        return {
            'max_linear': self.base_move_linear_velocity_spin.value(),
            'max_lateral': self.base_move_lateral_velocity_spin.value(),
            'max_angular': self.base_move_angular_velocity_spin.value(),
        }[key]

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
        self._set_sync_workspace_state()

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
        if not self.simulation_checkbox.isChecked():
            running = any(
                (process := self.processes.get(name)) is not None and process.is_running()
                for name in (
                    VICON_BASE_STATIC_TF_NAME,
                    VICON_EE_STATIC_TF_NAME,
                    BASE_POSE_ADAPTER_NAME,
                    ODOMETRY_POSE_ADAPTER_NAME,
                    ARM_POSE_ADAPTER_NAME,
                )
            )
            self.current_tcp_pose_button.setText(
                'Stop Transformations' if running else 'Launch Transformations'
            )
            self._style_button(self.current_tcp_pose_button, 'green' if running else 'grey')
            return
        if self.odometry_pose_checkbox.isChecked():
            running = any(
                (process := self.processes.get(name)) is not None and process.is_running()
                for name in (CURRENT_TCP_POSE_NAME, ODOMETRY_POSE_ADAPTER_NAME)
            )
            self.current_tcp_pose_button.setText(
                'Stop Transformations' if running else 'Launch Transformations'
            )
            self._style_button(self.current_tcp_pose_button, 'green' if running else 'grey')
            return
        self._set_process_toggle_button(
            self.current_tcp_pose_button,
            CURRENT_TCP_POSE_NAME,
            'Stop Transformations',
            'Launch Transformations',
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
        motion_ready = self._motion_ready()
        self.start_following_button.setEnabled(True)
        if motion_ready:
            color = 'green' if self._control_processes_running() else 'orange'
        else:
            color = 'orange'
        self._style_button(self.start_following_button, color)
        self._style_button(self.stop_following_button, 'grey')

    def _motion_ready(self) -> bool:
        return (
            self._has_path
            and self._has_robot_pose
            and self._has_arm_pose
            and self._jparse_ready
            and self._controller_ready
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
        return 'motion blocked; waiting for ' + ', '.join(missing)

    def _control_processes_running(self) -> bool:
        return not self._missing_control_process_names()

    def _missing_control_process_names(self) -> list[str]:
        return [
            name for name in (PATH_INDEX_NAME, BASE_FOLLOWER_NAME, ARM_FOLLOWER_NAME)
            if (process := self.processes.get(name)) is None or not process.is_running()
        ]

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

    def _set_sync_workspace_state(self) -> None:
        self._set_process_toggle_button(
            self.sync_workspace_button,
            SYNC_WORKSPACE_NAME,
            'Stop Sync',
            'Sync Workspace',
        )

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
