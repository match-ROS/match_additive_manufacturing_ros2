import json
import math
import os
import signal
import statistics
import sys
from datetime import datetime
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
    QMessageBox,
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
from am_operator_gui.config_store import ConfigStore
from am_operator_gui.operator_service import OperatorService


REPO_ROOT = Path(__file__).resolve().parents[2]
PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_SRC_ROOT = REPO_ROOT.parent
DEFAULT_COMPONENTS_DIR = REPO_ROOT / 'components'
DEFAULT_TRAJECTORY_DIR = DEFAULT_COMPONENTS_DIR / 'robotnik_paired_demo'
DEFAULT_DEFAULT_VELOCITY = 0.1
DEFAULT_PATH_TRANSFORM = {
    'x': 0.0,
    'y': 0.0,
    'z': 0.0,
    'yaw_deg': 0.0,
}
DEFAULT_FIXED_TOOL_OFFSET = {
    # Matches the modeled robot_arm_tool0 -> robot_arm_nozzle_tip transform.
    'xyz': [-0.25, 0.0, 0.015],
    'quaternion_xyzw': [0.0, -0.7071067812, 0.0, 0.7071067812],
}


def _normalize_quaternion(quaternion: list[float] | tuple[float, ...]) -> list[float]:
    norm = math.sqrt(sum(float(value) ** 2 for value in quaternion))
    if norm < 1e-12:
        raise ValueError('quaternion norm must be greater than zero')
    return [float(value) / norm for value in quaternion]


def _rpy_degrees_to_quaternion(roll: float, pitch: float, yaw: float) -> list[float]:
    roll, pitch, yaw = (math.radians(value) for value in (roll, pitch, yaw))
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return _normalize_quaternion([
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ])


def _quaternion_to_rpy_degrees(quaternion: list[float] | tuple[float, ...]) -> list[float]:
    x, y, z, w = _normalize_quaternion(quaternion)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch_value = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, pitch_value)))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return [math.degrees(value) for value in (roll, pitch, yaw)]
TOOL_OFFSET_COMPARISON_TOLERANCE = 1e-6
DEFAULT_SPRAY_DISTANCE_MAX_RATE = 0.02
DEFAULT_SPRAY_DISTANCE_MM = 100.0
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
VICON_TCP_POSE_BACKUP_NAME = 'vicon_tcp_pose_backup'
ARM_POSE_ADAPTER_NAME = 'arm_pose_adapter'
VICON_EE_STATIC_TF_NAME = 'vicon_ee_static_tf'
VICON_BASE_STATIC_TF_NAME = 'vicon_base_static_tf'
ARM_CONTROLLERS_NAME = 'arm_controllers'
BASE_ACCURACY_MONITOR_NAME = 'base_accuracy_monitor'
TCP_ACCURACY_MONITOR_NAME = 'tcp_accuracy_monitor'
ACCURACY_REPORT_NAME = 'accuracy_report'
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
    'base_follower.max_vx': 0.25,
    'base_follower.max_vy': 0.25,
    'base_follower.max_wz': 0.5,
    'base_move.kp_linear': 0.6,
    'base_move.kp_lateral': 0.6,
    'base_move.kp_angular_to_point': 1.5,
    'base_move.kp_angular_reorient': 1.2,
    'base_move.max_linear_velocity': 0.2,
    'base_move.max_lateral_velocity': 0.2,
    'base_move.max_angular_velocity': 0.5,
    'arm_direction.kp_z': 0.7,
    'arm_direction.along_track_kp': 2.0,
    'arm_direction.orthogonal_kp': 1.0,
    'arm_direction.max_along_track_correction': 0.03,
    'arm_direction.max_spray_axis_correction': 0.03,
    'arm_direction.max_tracking_linear_velocity': 0.12,
    'arm_direction.final_position_tolerance': 0.005,
    'arm_orientation.kp_orientation': 1.0,
    'arm_orientation.ki_orientation': 0.0,
    'arm_orientation.kd_orientation': 0.0,
    'arm_move.kp_linear': 0.8,
    'arm_move.kp_angular': 1.0,
    'arm_move.max_linear_velocity': 0.12,
    'arm_move.max_angular_velocity': 0.5,
}

PID_GAIN_GROUPS = (
    (
        'Mobile Base Follower',
        (
            ('base_follower.kp_x', 'Kp X'),
            ('base_follower.kp_y', 'Kp Y'),
            ('base_follower.kp_yaw', 'Kp yaw'),
            ('base_follower.max_vx', 'Max velocity X'),
            ('base_follower.max_vy', 'Max velocity Y'),
            ('base_follower.max_wz', 'Max velocity yaw'),
        ),
    ),
    (
        'Mobile Base Move To Start',
        (
            ('base_move.kp_linear', 'Kp linear'),
            ('base_move.kp_lateral', 'Kp lateral'),
            ('base_move.kp_angular_to_point', 'Kp angular to point'),
            ('base_move.kp_angular_reorient', 'Kp angular reorient'),
            ('base_move.max_linear_velocity', 'Max linear velocity'),
            ('base_move.max_lateral_velocity', 'Max lateral velocity'),
            ('base_move.max_angular_velocity', 'Max angular velocity'),
        ),
    ),
    (
        'Arm Path Direction',
        (
            ('arm_direction.kp_z', 'Kp Z'),
            ('arm_direction.along_track_kp', 'Along-track Kp'),
            ('arm_direction.orthogonal_kp', 'Orthogonal Kp'),
            ('arm_direction.max_along_track_correction', 'Max along correction'),
            ('arm_direction.max_spray_axis_correction', 'Max spray correction'),
            ('arm_direction.max_tracking_linear_velocity', 'Max tracking velocity'),
            ('arm_direction.final_position_tolerance', 'Final position tolerance'),
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
            ('arm_move.max_linear_velocity', 'Max linear velocity'),
            ('arm_move.max_angular_velocity', 'Max angular velocity'),
        ),
    ),
)

PLATFORM_PROFILES = {
    'robotnik': {
        'label': 'Robotnik',
        'follower_type': 'pid',
        'diff_drive_mode': False,
        'path_topic': '/base_path',
        'robot_pose_topic': '/robot_pose',
        'odom_topic': '/robot/robotnik_base_control/odom',
        'cmd_vel_topic': '/robot/robotnik_base_control/cmd_vel_unstamped',
        'output_stamped': False,
        'command_frame_id': 'base_link',
        'path_frame': 'map',
        'external_map_frame': 'map',
        'robot_base_frame': 'base_link',
        'robot_tree_root_frame': 'odom',
        'max_vx': 0.25,
        'max_vy': 0.25,
        'max_wz': 0.5,
        'move_max_linear': 0.2,
        'move_max_lateral': 0.2,
        'move_max_angular': 0.5,
        'arm_control_supported': True,
    },
    'bunker': {
        'label': 'Bunker',
        'follower_type': 'pure_pursuit',
        'diff_drive_mode': True,
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
        'arm_control_supported': True,
    },
    'mur620_sim': {
        'label': 'MuR620 (simulation)',
        'follower_type': 'pure_pursuit',
        'diff_drive_mode': True,
        'path_topic': '/base_path',
        # `mur_launch_sim` publishes this ground-truth PoseStamped endpoint
        # without requiring hardware localization or a mocap system.
        'robot_pose_topic': '/mur620a/ground_truth/pose',
        'odom_topic': '/mur620a/ground_truth/odom',
        'cmd_vel_topic': '/mur620a/mobile_base_controller/cmd_vel',
        'output_stamped': True,
        'command_frame_id': 'mur620a/base_footprint',
        'path_frame': 'map',
        'external_map_frame': 'map',
        'robot_base_frame': 'mur620a/base_footprint',
        'robot_tree_root_frame': 'mur620a/odom',
        'max_vx': 0.2,
        'max_vy': 0.0,
        'max_wz': 0.5,
        'move_max_linear': 0.15,
        'move_max_lateral': 0.0,
        'move_max_angular': 0.4,
        # The MuR arm needs its own controller integration.  Do not route it
        # through the Robotnik UR controller stack while that work is pending.
        'arm_control_supported': False,
    },
    'mur620_left_arm_sim': {
        # This is intentionally a separate profile from ``mur620_sim``.  The
        # latter remains the low-risk base-only validation path.
        'label': 'MuR620 left arm (simulation)',
        'follower_type': 'pure_pursuit',
        'diff_drive_mode': True,
        'path_topic': '/base_path',
        'robot_pose_topic': '/mur620a/ground_truth/pose',
        'odom_topic': '/mur620a/ground_truth/odom',
        'cmd_vel_topic': '/mur620a/mobile_base_controller/cmd_vel',
        'output_stamped': True,
        'command_frame_id': 'mur620a/base_footprint',
        'path_frame': 'map',
        'external_map_frame': 'map',
        'robot_base_frame': 'mur620a/base_footprint',
        'robot_tree_root_frame': 'mur620a/odom',
        'max_vx': 0.2,
        'max_vy': 0.0,
        'max_wz': 0.5,
        'move_max_linear': 0.15,
        'move_max_lateral': 0.0,
        'move_max_angular': 0.4,
        'arm_control_supported': True,
        'mur_native_arm': True,
        'arm_base_link': 'mur620a/UR10_l/base_link',
        'arm_command_frame': 'UR10_l/base_link',
        'arm_tip_link': 'mur620a/UR10_l/tool0',
        'arm_command_tip_link': 'UR10_l/tool0',
        'arm_joint_prefix': 'UR10_l/',
        'robot_description_topic': '/mur620a/robot_description',
        'joint_states_topic': '/mur620a/joint_states',
        'arm_velocity_command_topic': '/mur620a/forward_velocity_controller_l/commands',
        'arm_controller_manager': '/mur620a/controller_manager',
        'arm_trajectory_topic': '/mur620a/joint_trajectory_controller/joint_trajectory',
        'arm_world_twist_topic': '/mur620a/arm_following/twist_world',
        'start_base_motion_compensation': True,
        'base_velocity_topic': '/mur620a/ground_truth/odom',
        'base_velocity_type': 'odometry',
        'compensation_base_frame': 'mur620a/base_footprint',
        'base_compensation_topic': '/mur620a/arm_following/twist_base_compensation_world',
        'arm_stop_topic': '/mur620a/jparse_velocity_controller_l/twist_cmd',
        'arm_stop_frame': 'UR10_l/base_link',
        'arm_move_to_start_supported': True,
    },
}

MUR_ARMS = {'none': 'None', 'l': 'Left', 'r': 'Right'}
MUR_NATIVE_ARMS = {'l', 'r'}
LEGACY_MUR_PLATFORM = 'mur620_left_arm_sim'


class PidGainsDialog(QDialog):

    def __init__(
        self,
        parent: 'OperatorWindow',
        gains: dict[str, float],
        defaults: dict[str, float],
    ) -> None:
        super().__init__(parent)
        self.setWindowTitle('PID Gains')
        self.resize(640, 720)
        self._parent = parent
        self._defaults = defaults
        self._spins: dict[str, QDoubleSpinBox] = {}

        layout = QVBoxLayout(self)
        for group_name, fields in PID_GAIN_GROUPS:
            group = QGroupBox(group_name)
            grid = QGridLayout(group)
            for index, (key, label) in enumerate(fields):
                spin = QDoubleSpinBox()
                if '.max_' in key:
                    spin.setRange(0.0, 10.0)
                else:
                    spin.setRange(-10000.0, 10000.0)
                spin.setDecimals(4)
                spin.setSingleStep(0.05)
                spin.setValue(float(gains.get(key, self._defaults[key])))
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
            spin.setValue(self._defaults[key])

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
        self._last_safety_gate_ready = False
        self._launch_all_active = False
        self._launch_all_timers: list[QTimer] = []
        self._updating_path_indices = False
        # The reference GUI and web GUI deliberately share persistence, process
        # ownership and ROS bridge lifecycle through this toolkit-neutral layer.
        self.service = OperatorService(
            output_callback=self._on_process_output,
            status_callback=self._on_ros_status,
            path_index_callback=self._on_path_index,
        )
        self._config = self.service.config
        self._pid_gains_dialog: Optional[PidGainsDialog] = None
        self._base_smoothing_dialog: Optional[BaseSmoothingDialog] = None

        self.processes = self.service.processes
        if not self.service.ensure_ros():
            raise RuntimeError(self.service.ros_error or 'failed to start ROS bridge')
        self.ros_bridge = self.service.ros_bridge

        self.ros_status_changed.connect(self._set_ros_status)
        self.path_index_changed.connect(self._set_path_index_from_ros)
        self.process_output.connect(self._append_process_output)

        self._build_ui()
        self._connect_signals()
        # Give the TF listener a moment to receive the static tool transform before
        # checking that the persisted calibration still matches the robot.
        QTimer.singleShot(1000, self._check_configured_tool_offset)

        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self._refresh_process_states)
        self.status_timer.start(500)

        self._publish_overrides()
        self._refresh_process_states()

    def _load_config(self) -> dict:
        return self.service.store.load()

    def _save_config(self) -> None:
        try:
            self.service.store.save(self._config)
        except OSError as exc:
            self._append_process_output('gui', f'failed to save config: {exc}')

    def _configured_default_velocity(self) -> float:
        try:
            velocity = float(self._config.get('default_velocity', DEFAULT_DEFAULT_VELOCITY))
        except (TypeError, ValueError):
            return DEFAULT_DEFAULT_VELOCITY
        return max(0.001, min(10.0, velocity))

    def _configured_default_velocity_enabled(self) -> bool:
        return bool(self._config.get('default_velocity_enabled', False))

    def _configured_contour_control_enabled(self) -> bool:
        """Return the explicitly persisted, default-off contour correction choice."""
        return bool(self._config.get('contour_control_enabled', False))

    def _configured_path_transform(self) -> dict[str, float]:
        return self._path_transform_for_directory(self._configured_trajectory_directory())

    @staticmethod
    def _path_directory_key(directory: Path) -> str:
        return str(directory.expanduser().resolve())

    def _path_transform_for_directory(self, directory: Path) -> dict[str, float]:
        directory_key = self._path_directory_key(directory)
        platform_combo = getattr(self, 'platform_combo', None)
        platform = self._current_platform_key() if platform_combo is not None else ''
        platform_transforms = self._config.get('path_transforms_by_platform_directory', {})
        if platform and isinstance(platform_transforms, dict):
            configured = platform_transforms.get(platform, {})
            if isinstance(configured, dict) and isinstance(configured.get(directory_key), dict):
                return configured[directory_key]
        transforms = self._config.get('path_transforms_by_directory', {})
        configured = {}
        if isinstance(transforms, dict):
            configured = transforms.get(directory_key, {})
        # Support older configuration files until the transform is next saved.
        if not isinstance(configured, dict):
            configured = {}
        if not configured and not isinstance(transforms, dict):
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
        if value == LEGACY_MUR_PLATFORM:
            return 'mur620_sim'
        return value if value in PLATFORM_PROFILES else 'robotnik'

    def _configured_mur_arm(self) -> str:
        default = 'l' if str(self._config.get('platform', '')).strip().lower() == LEGACY_MUR_PLATFORM else 'none'
        value = str(self._config.get('mur_arm', default)).strip().lower()
        return value if value in MUR_ARMS else default

    def _configured_follower_type(self) -> str:
        # Keep platform-specific drive settings separate.  Old configurations
        # may have a global follower_type, which remains a Robotnik-only
        # fallback for backwards compatibility.
        platform = self._configured_platform()
        configured = self._config.get('platform_control_settings', {})
        if isinstance(configured, dict):
            settings = configured.get(platform, {})
            if isinstance(settings, dict):
                value = str(settings.get('follower_type', '')).strip().lower()
                if value in {'pid', 'pure_pursuit'}:
                    return value
        if platform == 'robotnik':
            value = str(self._config.get('follower_type', 'pid')).strip().lower()
            if value in {'pid', 'pure_pursuit'}:
                return value
        return str(PLATFORM_PROFILES[platform]['follower_type'])

    def _configured_diff_drive_mode(self) -> bool:
        platform = self._configured_platform()
        configured = self._config.get('platform_control_settings', {})
        if isinstance(configured, dict):
            settings = configured.get(platform, {})
            if isinstance(settings, dict) and 'diff_drive_mode' in settings:
                return bool(settings['diff_drive_mode'])
        return bool(PLATFORM_PROFILES[platform]['diff_drive_mode'])

    def _configured_platform_text(self, key: str, fallback: str) -> str:
        """Read a text setting for the configured platform, with legacy fallback."""
        configured = self._config.get('platform_control_settings', {})
        if isinstance(configured, dict):
            settings = configured.get(self._configured_platform(), {})
            if isinstance(settings, dict):
                value = str(settings.get(key, '')).strip()
                if value:
                    return value
        return fallback

    def _configured_use_odometry_robot_pose(self) -> bool:
        return bool(self._config.get('use_odometry_robot_pose', False))

    def _configured_use_vicon_tcp_base_pose_fallback(self) -> bool:
        return bool(self._config.get('use_vicon_tcp_base_pose_fallback', False))

    def _use_vicon_tcp_base_pose_fallback(self) -> bool:
        checkbox = getattr(self, 'vicon_tcp_base_pose_fallback_checkbox', None)
        return bool(checkbox is not None and checkbox.isChecked())

    def _configured_simulation(self) -> bool:
        return bool(self._config.get('simulation', False))

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
        fallback = configured or str(self._current_platform_profile()['robot_base_frame'])
        return self._configured_platform_text('robot_base_frame', fallback)

    def _configured_robot_tree_root_frame(self) -> str:
        configured = str(self._config.get('robot_tree_root_frame', '')).strip()
        fallback = configured or str(self._current_platform_profile()['robot_tree_root_frame'])
        return self._configured_platform_text('robot_tree_root_frame', fallback)

    def _configured_control_frame(self) -> str:
        configured = str(self._config.get('control_frame', '')).strip()
        return configured or self._path_frame_from_folder(self._configured_trajectory_directory())

    def _configured_trajectory_directory(self) -> Path:
        configured = str(self._config.get('trajectory_directory', '')).strip()
        return Path(configured).expanduser() if configured else DEFAULT_TRAJECTORY_DIR

    def _configured_fixed_tool_offset(self) -> dict[str, list[float]]:
        platform_combo = getattr(self, 'platform_combo', None)
        platform = self._current_platform_key() if platform_combo is not None else self._configured_platform()
        platform_offsets = self._config.get('fixed_tool_offsets_by_platform', {})
        configured = {}
        if platform and isinstance(platform_offsets, dict):
            candidate = platform_offsets.get(platform, {})
            if isinstance(candidate, dict):
                configured = candidate
        # Support the original global setting for existing configurations and
        # platforms that have not been calibrated yet.
        if not configured:
            configured = self._config.get('fixed_tool_offset', {})
        if not isinstance(configured, dict):
            return dict(DEFAULT_FIXED_TOOL_OFFSET)
        xyz = configured.get('xyz', DEFAULT_FIXED_TOOL_OFFSET['xyz'])
        quaternion = configured.get('quaternion_xyzw', DEFAULT_FIXED_TOOL_OFFSET['quaternion_xyzw'])
        try:
            if len(xyz) != 3 or len(quaternion) != 4:
                raise ValueError
            return {
                'xyz': [float(value) for value in xyz],
                'quaternion_xyzw': [float(value) for value in quaternion],
            }
        except (TypeError, ValueError):
            return dict(DEFAULT_FIXED_TOOL_OFFSET)

    def _configured_fixed_tool_offset_input_mode(self) -> str:
        mode = str(self._config.get('fixed_tool_offset_input_mode', 'quaternion')).strip().lower()
        return mode if mode in {'rpy', 'quaternion'} else 'quaternion'

    def _sync_fixed_tool_offset_widgets(self) -> None:
        offset = self._configured_fixed_tool_offset()
        quaternion = _normalize_quaternion(offset['quaternion_xyzw'])
        rpy = _quaternion_to_rpy_degrees(quaternion)
        widgets = [*self.fixed_tool_offset_xyz, *self.fixed_tool_offset_rotation]
        for widget in widgets:
            widget.blockSignals(True)
        for widget, value in zip(self.fixed_tool_offset_xyz, offset['xyz']):
            widget.setValue(value)
        mode = self._configured_fixed_tool_offset_input_mode()
        rotation = [*rpy, quaternion[3]] if mode == 'rpy' else quaternion
        labels = ['Roll', 'Pitch', 'Yaw', 'Qw'] if mode == 'rpy' else ['Qx', 'Qy', 'Qz', 'Qw']
        for index, (label, widget, value) in enumerate(zip(
            self.fixed_tool_offset_rotation_labels, self.fixed_tool_offset_rotation, rotation
        )):
            label.setText(f'{labels[index]}{" (deg)" if mode == "rpy" and index < 3 else ""}')
            widget.setSuffix(' deg' if mode == 'rpy' and index < 3 else '')
            widget.setDecimals(3 if mode == 'rpy' and index < 3 else 8)
            widget.setSingleStep(0.1 if mode == 'rpy' and index < 3 else 0.01)
            widget.setEnabled(not (mode == 'rpy' and index == 3))
            widget.setValue(value)
        self.fixed_tool_offset_mode.blockSignals(True)
        self.fixed_tool_offset_mode.setCurrentIndex(self.fixed_tool_offset_mode.findData(mode))
        self.fixed_tool_offset_mode.blockSignals(False)
        for widget in widgets:
            widget.blockSignals(False)

    def _set_fixed_tool_offset_input_mode(self, *_args) -> None:
        self._config['fixed_tool_offset_input_mode'] = str(
            self.fixed_tool_offset_mode.currentData()
        )
        self._save_config()
        self._sync_fixed_tool_offset_widgets()

    def _save_fixed_tool_offset(self) -> None:
        try:
            xyz = [float(widget.value()) for widget in self.fixed_tool_offset_xyz]
            if self.fixed_tool_offset_mode.currentData() == 'rpy':
                quaternion = _rpy_degrees_to_quaternion(*(
                    float(widget.value()) for widget in self.fixed_tool_offset_rotation[:3]
                ))
            else:
                quaternion = _normalize_quaternion([
                    float(widget.value()) for widget in self.fixed_tool_offset_rotation
                ])
        except (TypeError, ValueError) as exc:
            self._append_process_output('gui', f'cannot save flange-to-nozzle transform: {exc}')
            return
        offset = {
            'xyz': xyz,
            'quaternion_xyzw': quaternion,
        }
        platform = self._current_platform_key() or self._configured_platform()
        platform_offsets = self._config.setdefault('fixed_tool_offsets_by_platform', {})
        if not isinstance(platform_offsets, dict):
            platform_offsets = {}
            self._config['fixed_tool_offsets_by_platform'] = platform_offsets
        platform_offsets[platform] = offset
        self._config['fixed_tool_offset_input_mode'] = str(self.fixed_tool_offset_mode.currentData())
        self._save_config()
        self._sync_fixed_tool_offset_widgets()
        self._append_process_output(
            'gui',
            'saved flange-to-nozzle transform; restart the arm follower and controller stack to apply',
        )

    def _configured_spray_distance_max_rate(self) -> float:
        try:
            value = float(self._config.get('spray_distance_max_rate', DEFAULT_SPRAY_DISTANCE_MAX_RATE))
        except (TypeError, ValueError):
            value = DEFAULT_SPRAY_DISTANCE_MAX_RATE
        return max(0.001, min(1.0, value))

    def _configured_spray_distance_mm(self) -> float:
        try:
            value = float(self._config.get('spray_distance_mm', DEFAULT_SPRAY_DISTANCE_MM))
        except (TypeError, ValueError):
            value = DEFAULT_SPRAY_DISTANCE_MM
        return max(-10000.0, min(10000.0, value))

    def _tool_offset_launch_arguments(self) -> list[str]:
        offset = (
            self._configured_fixed_tool_offset()
            if hasattr(self, '_configured_fixed_tool_offset') else dict(DEFAULT_FIXED_TOOL_OFFSET)
        )
        xyz = ', '.join(OperatorWindow._ros_float_literal(value) for value in offset['xyz'])
        quaternion = ', '.join(
            OperatorWindow._ros_float_literal(value) for value in offset['quaternion_xyzw'])
        return [
            f'fixed_tool_offset_xyz:=[{xyz}]',
            f'fixed_tool_offset_quaternion_xyzw:=[{quaternion}]',
        ]

    def _effective_spray_distance_m(self) -> float:
        if not hasattr(self, 'nozzle_reference') or not hasattr(self, 'nozzle_offset'):
            return 0.0
        return (self.nozzle_reference.value() + self.nozzle_offset.value()) / 1000.0

    def _configured_pid_gains(self) -> dict[str, float]:
        configured = self._config.get('pid_gains', {})
        gains = self._pid_gain_defaults()
        if not isinstance(configured, dict):
            return gains
        for key, default in DEFAULT_PID_GAINS.items():
            try:
                gains[key] = float(configured.get(key, default))
            except (TypeError, ValueError):
                gains[key] = default
        return gains

    def _pid_gain_defaults(self) -> dict[str, float]:
        """Return gains with velocity limits appropriate for the active platform."""
        gains = dict(DEFAULT_PID_GAINS)
        profile = self._current_platform_profile()
        gains.update({
            'base_follower.max_vx': float(profile['max_vx']),
            'base_follower.max_vy': float(profile['max_vy']),
            'base_follower.max_wz': float(profile['max_wz']),
            'base_move.max_linear_velocity': float(profile['move_max_linear']),
            'base_move.max_lateral_velocity': float(profile['move_max_lateral']),
            'base_move.max_angular_velocity': float(profile['move_max_angular']),
        })
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
        self.simulation_checkbox.setChecked(self._configured_simulation())
        self.platform_combo = QComboBox()
        for key, profile in PLATFORM_PROFILES.items():
            if key == LEGACY_MUR_PLATFORM:
                continue
            self.platform_combo.addItem(str(profile['label']), key)
        self.platform_combo.setCurrentIndex(self.platform_combo.findData(self._configured_platform()))
        self.mur_arm_combo = QComboBox()
        for arm, label in MUR_ARMS.items():
            self.mur_arm_combo.addItem(label, arm)
        self.mur_arm_combo.setCurrentIndex(self.mur_arm_combo.findData(self._configured_mur_arm()))
        self.mur_arm_combo.setEnabled(self._configured_platform() == 'mur620_sim')
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
        self.vicon_tcp_base_pose_fallback_checkbox = QCheckBox('Base pose via tool TF')
        self.vicon_tcp_base_pose_fallback_checkbox.setToolTip(
            'Estimate the base pose from the Vicon tool pose and robot TF when the base marker is unavailable.'
        )
        self.vicon_tcp_base_pose_fallback_checkbox.setChecked(
            self._configured_use_vicon_tcp_base_pose_fallback()
            and not self.odometry_pose_checkbox.isChecked()
        )
        self.direction_mode = QComboBox()
        self.direction_mode.addItems(['goal_direction', 'speed_orthogonal'])
        self.direction_mode.setCurrentText('goal_direction')
        self.index_spin = QSpinBox()
        self.index_spin.setRange(0, 100000)
        self.index_spin.setValue(0)
        self.original_arm_index_spin = QSpinBox()
        self.original_arm_index_spin.setRange(0, 100000)
        self.original_arm_index_spin.setValue(0)

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
        launch_layout.addWidget(QLabel('MuR arm'), 0, 3)
        launch_layout.addWidget(self.mur_arm_combo, 0, 4)
        launch_layout.addWidget(QLabel('Follower'), 0, 5)
        launch_layout.addWidget(self.follower_type_combo, 0, 6)
        launch_layout.addWidget(self.diff_drive_checkbox, 0, 7)
        launch_layout.addWidget(QLabel('Direction'), 1, 0)
        launch_layout.addWidget(self.direction_mode, 1, 1)
        launch_layout.addWidget(QLabel('Interpolated index'), 1, 2)
        launch_layout.addWidget(self.index_spin, 1, 3)
        launch_layout.addWidget(QLabel('Non-interpolated arm index'), 1, 4)
        launch_layout.addWidget(self.original_arm_index_spin, 1, 5)
        launch_layout.addWidget(self.odometry_pose_checkbox, 2, 4)
        launch_layout.addWidget(self.vicon_tcp_base_pose_fallback_checkbox, 2, 5)
        launch_layout.addWidget(QLabel('Path folder'), 2, 0)
        launch_layout.addWidget(self.path_folder, 2, 1, 1, 3)
        launch_layout.addWidget(self.browse_button, 2, 3)
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
        self.capture_tool_offset_button = QPushButton('Capture UR TCP Offset')
        self._set_tool_offset_capture_button_state(matches=True)
        self.base_accuracy_monitor_button = QPushButton('Record Base Accuracy')
        self.tcp_accuracy_monitor_button = QPushButton('Record TCP Accuracy')
        self.accuracy_phase_combo = QComboBox()
        self.accuracy_phase_combo.addItem('Baseline', 'baseline')
        self.accuracy_phase_combo.addItem('Tuned', 'tuned')
        self.accuracy_report_button = QPushButton('Summarize Accuracy Runs')
        self.default_velocity_checkbox = QCheckBox('Default velocity')
        self.default_velocity_checkbox.setChecked(self._configured_default_velocity_enabled())
        self.contour_control_checkbox = QCheckBox('Enable bounded contour correction')
        self.contour_control_checkbox.setToolTip(
            'Off by default. Enables only the bounded Y/Z correction input; '
            'it still requires fresh Keyence errors and /start_condition.'
        )
        self.contour_control_checkbox.setChecked(self._configured_contour_control_enabled())
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
        component_layout.addWidget(self.capture_tool_offset_button, 2, 3)
        component_layout.addWidget(self.base_accuracy_monitor_button, 5, 0)
        component_layout.addWidget(self.tcp_accuracy_monitor_button, 5, 1)
        component_layout.addWidget(QLabel('Accuracy phase'), 6, 0)
        component_layout.addWidget(self.accuracy_phase_combo, 6, 1)
        component_layout.addWidget(self.accuracy_report_button, 6, 2)
        component_layout.addWidget(self.default_velocity_checkbox, 4, 0)
        component_layout.addWidget(self.default_velocity_spin, 4, 1)
        component_layout.addWidget(self.contour_control_checkbox, 4, 2, 1, 2)

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
        self.nozzle_reference.setValue(self._configured_spray_distance_mm())
        self.nozzle_offset = QSlider(Qt.Horizontal)
        self.nozzle_offset.setRange(-100, 100)
        self.nozzle_offset.setValue(0)
        self.nozzle_offset_value = QLabel('+0 mm')
        self.nozzle_effective_value = QLabel('0.0 mm effective')

        fixed_offset = self._configured_fixed_tool_offset()
        self.fixed_tool_offset_xyz = []
        for value in fixed_offset['xyz']:
            spin = QDoubleSpinBox()
            spin.setRange(-1000.0, 1000.0)
            spin.setDecimals(6)
            spin.setSingleStep(0.001)
            spin.setSuffix(' m')
            spin.setValue(value)
            self.fixed_tool_offset_xyz.append(spin)
        self.fixed_tool_offset_mode = QComboBox()
        self.fixed_tool_offset_mode.addItem('RPY (degrees)', 'rpy')
        self.fixed_tool_offset_mode.addItem('Quaternion (x, y, z, w)', 'quaternion')
        self.fixed_tool_offset_mode.setCurrentIndex(
            self.fixed_tool_offset_mode.findData(self._configured_fixed_tool_offset_input_mode())
        )
        self.fixed_tool_offset_rotation = []
        for _ in range(4):
            spin = QDoubleSpinBox()
            spin.setRange(-10000.0, 10000.0)
            spin.setDecimals(8)
            spin.setSingleStep(0.01)
            self.fixed_tool_offset_rotation.append(spin)
        self.fixed_tool_offset_save_button = QPushButton('Save flange-to-nozzle transform')

        override_layout.addWidget(QLabel('Velocity override'), 0, 0)
        override_layout.addWidget(self.velocity_slider, 0, 1)
        override_layout.addWidget(self.velocity_value, 0, 2)
        override_layout.addWidget(QLabel('Spray distance'), 1, 0)
        override_layout.addWidget(self.nozzle_reference, 1, 1)
        override_layout.addWidget(self.nozzle_effective_value, 1, 2)
        override_layout.addWidget(QLabel('Spray distance offset'), 2, 0)
        override_layout.addWidget(self.nozzle_offset, 2, 1)
        override_layout.addWidget(self.nozzle_offset_value, 2, 2)
        offset_group = QGroupBox('Flange-to-nozzle transform')
        offset_layout = QGridLayout(offset_group)
        offset_layout.addWidget(QLabel('XYZ'), 0, 0)
        for column, spin in enumerate(self.fixed_tool_offset_xyz, start=1):
            offset_layout.addWidget(spin, 0, column)
        offset_layout.addWidget(QLabel('Rotation input'), 1, 0)
        offset_layout.addWidget(self.fixed_tool_offset_mode, 1, 1, 1, 3)
        self.fixed_tool_offset_rotation_labels = [QLabel() for _ in range(4)]
        for column, (label, spin) in enumerate(zip(
            self.fixed_tool_offset_rotation_labels, self.fixed_tool_offset_rotation
        ), start=1):
            offset_layout.addWidget(label, 2, column)
            offset_layout.addWidget(spin, 3, column)
        self._sync_fixed_tool_offset_widgets()
        offset_layout.addWidget(self.fixed_tool_offset_save_button, 4, 0, 1, 4)
        override_layout.addWidget(offset_group, 3, 0, 1, 3)

        status_group = QGroupBox('Status')
        status_layout = QHBoxLayout(status_group)
        self.path_status = QLabel('/base_path: waiting')
        self.pose_status = QLabel('/robot_pose: waiting')
        self.arm_pose_status = QLabel('/current_deposition_pose: waiting')
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
        self.mur_arm_combo.currentIndexChanged.connect(self._set_mur_arm)
        self.follower_type_combo.currentIndexChanged.connect(self._set_follower_type)
        self.diff_drive_checkbox.toggled.connect(self._set_diff_drive_mode)
        self.odometry_pose_checkbox.toggled.connect(self._set_use_odometry_robot_pose)
        self.vicon_tcp_base_pose_fallback_checkbox.toggled.connect(
            self._set_use_vicon_tcp_base_pose_fallback
        )
        self.launch_button.clicked.connect(lambda: self._invoke_service_action('launch_all'))
        self.launch_sim_button.clicked.connect(lambda: self._invoke_service_action('simulation'))
        self.pid_gains_button.clicked.connect(self._open_pid_gains_window)
        self.base_smoothing_button.clicked.connect(self._open_base_smoothing_window)
        self.calculate_path_transform_button.clicked.connect(lambda: self._invoke_service_action('calculate_path_transform'))
        self.publish_path_button.clicked.connect(lambda: self._invoke_service_action('publish_path'))
        self.base_follower_button.clicked.connect(lambda: self._invoke_service_action('base_follower'))
        self.arm_follower_button.clicked.connect(lambda: self._invoke_service_action('arm_follower'))
        self.path_index_button.clicked.connect(lambda: self._invoke_service_action('path_index'))
        self.current_tcp_pose_button.clicked.connect(lambda: self._invoke_service_action('transformations'))
        self.arm_controllers_button.clicked.connect(lambda: self._invoke_service_action('controllers'))
        self.switch_arm_velocity_button.clicked.connect(lambda: self._invoke_service_action('switch_arm_velocity'))
        self.capture_tool_offset_button.clicked.connect(lambda: self._invoke_service_action('capture_tool_offset'))
        self.base_accuracy_monitor_button.clicked.connect(lambda: self._invoke_service_action('base_accuracy'))
        self.tcp_accuracy_monitor_button.clicked.connect(lambda: self._invoke_service_action('tcp_accuracy'))
        self.accuracy_report_button.clicked.connect(lambda: self._invoke_service_action('accuracy_report'))
        self.move_base_button.clicked.connect(lambda: self._invoke_service_action('move_base'))
        self.move_arm_button.clicked.connect(lambda: self._invoke_service_action('move_arm'))
        self.start_following_button.clicked.connect(lambda: self._invoke_service_action('start_following'))
        self.stop_following_button.clicked.connect(lambda: self._invoke_service_action('stop_following'))
        self.rviz_button.clicked.connect(lambda: self._invoke_service_action('rviz'))
        self.sync_workspace_button.clicked.connect(lambda: self._invoke_service_action('sync_workspace'))
        self.index_spin.valueChanged.connect(self._publish_path_index)
        self.index_spin.valueChanged.connect(self._set_original_arm_index_from_tracking)
        self.original_arm_index_spin.valueChanged.connect(self._set_tracking_index_from_original_arm)
        self.velocity_slider.valueChanged.connect(self._publish_overrides)
        self.default_velocity_checkbox.toggled.connect(self._set_default_velocity_enabled)
        self.contour_control_checkbox.toggled.connect(self._set_contour_control_enabled)
        self.default_velocity_spin.valueChanged.connect(self._set_default_velocity)
        self.path_transform_x_spin.valueChanged.connect(self._set_path_transform)
        self.path_transform_y_spin.valueChanged.connect(self._set_path_transform)
        self.path_transform_z_spin.valueChanged.connect(self._set_path_transform)
        self.path_transform_yaw_spin.valueChanged.connect(self._set_path_transform)
        self.nozzle_reference.valueChanged.connect(self._set_spray_distance_mm)
        self.nozzle_reference.valueChanged.connect(self._publish_overrides)
        self.nozzle_offset.valueChanged.connect(self._publish_overrides)
        self.fixed_tool_offset_mode.currentIndexChanged.connect(self._set_fixed_tool_offset_input_mode)
        self.fixed_tool_offset_save_button.clicked.connect(self._save_fixed_tool_offset)

    def _invoke_service_action(self, action: str) -> None:
        """Adapter from the retained Qt view to the shared control layer."""
        if action in {'base_accuracy', 'tcp_accuracy'}:
            self._config['accuracy_phase'] = str(self.accuracy_phase_combo.currentData())
        self._config['path_index'] = int(self.index_spin.value())
        self._config['original_arm_index'] = int(self._original_arm_path_index())
        self.service.action(action)
        self._launch_all_active = self.service._launch_all_active
        if action == 'calculate_path_transform':
            transform = self._path_transform_for_directory(
                Path(self.path_folder.text().strip()).expanduser()
            )
            if isinstance(transform, dict):
                self._set_path_transform_widget_values(**{
                    key: float(transform.get(key, 0.0))
                    for key in ('x', 'y', 'z', 'yaw_deg')
                })
        self._refresh_process_states()

    def _current_platform_key(self) -> str:
        value = self.platform_combo.currentData()
        key = str(value).strip().lower()
        # Do not silently reinterpret a malformed selection as Robotnik: that
        # could start a substantially different simulator than the operator
        # selected.  Callers that only need display defaults handle the empty
        # value safely; launch paths reject it explicitly.
        return key if key in PLATFORM_PROFILES else ''

    def _current_platform_profile(self) -> dict:
        platform = self._current_platform_key()
        profile = dict(PLATFORM_PROFILES.get(platform, PLATFORM_PROFILES['robotnik']))
        if platform == 'mur620_sim':
            arm = self._current_mur_arm()
            if arm not in MUR_NATIVE_ARMS:
                return profile
            arm_profile = PLATFORM_PROFILES[LEGACY_MUR_PLATFORM]
            profile.update({
                key: arm_profile[key]
                for key in (
                    'arm_control_supported', 'mur_native_arm', 'arm_joint_prefix',
                    'robot_description_topic', 'joint_states_topic',
                    'arm_velocity_command_topic', 'arm_controller_manager',
                    'arm_trajectory_topic', 'arm_world_twist_topic',
                    'start_base_motion_compensation', 'base_velocity_topic',
                    'base_velocity_type', 'compensation_base_frame', 'base_compensation_topic',
                    'arm_stop_topic', 'arm_stop_frame', 'arm_move_to_start_supported',
                )
            })
            prefix = f'UR10_{arm}'
            suffix = f'_{arm}'
            profile.update({
                'arm_base_link': f'mur620a/{prefix}/base_link',
                'arm_command_frame': f'{prefix}/base_link',
                'arm_tip_link': f'mur620a/{prefix}/tool0',
                'arm_command_tip_link': f'{prefix}/tool0',
                'arm_joint_prefix': f'{prefix}/',
                'arm_velocity_command_topic': f'/mur620a/forward_velocity_controller{suffix}/commands',
                'arm_trajectory_topic': f'/mur620a/joint_trajectory_controller{suffix}/joint_trajectory',
                'arm_stop_topic': f'/mur620a/jparse_velocity_controller{suffix}/twist_cmd',
                'arm_stop_frame': f'{prefix}/base_link',
                'arm_selected': arm,
            })
        return profile

    def _current_mur_arm(self) -> str:
        combo = getattr(self, 'mur_arm_combo', None)
        value = combo.currentData() if combo is not None else self._configured_mur_arm()
        value = str(value).strip().lower()
        return value if value in MUR_ARMS else self._configured_mur_arm()

    def _arm_platform_profile(self) -> dict:
        """Return a profile without making lightweight command tests build widgets."""
        profile_getter = getattr(self, '_current_platform_profile', None)
        return profile_getter() if profile_getter is not None else PLATFORM_PROFILES['robotnik']

    def _arm_control_supported(self) -> bool:
        """Whether this platform has a verified arm-control integration."""
        # Keeping a default here also makes the individual command helpers
        # usable in headless/unit-test contexts that do not construct widgets.
        platform_combo = getattr(self, 'platform_combo', None)
        if platform_combo is None:
            return True
        return bool(self._current_platform_profile().get('arm_control_supported', False))

    def _report_unsupported_arm_control(self, action: str) -> None:
        self._append_process_output(
            'safety',
            f'{action} is unavailable for {self._current_platform_profile()["label"]}: '
            'no verified arm controller integration is configured',
        )

    def _current_follower_type(self) -> str:
        value = self.follower_type_combo.currentData()
        follower_type = str(value).strip().lower()
        return follower_type if follower_type in {'pid', 'pure_pursuit'} else 'pid'

    def _diff_drive_mode(self) -> bool:
        return bool(OperatorWindow._arm_platform_profile(self).get('diff_drive_mode', False)) or self.diff_drive_checkbox.isChecked()

    def _sync_diff_drive_checkbox(self) -> None:
        is_bunker = self._current_platform_key() == 'bunker'
        self.diff_drive_checkbox.blockSignals(True)
        if is_bunker:
            self.diff_drive_checkbox.setChecked(True)
        self.diff_drive_checkbox.setEnabled(not is_bunker)
        self.diff_drive_checkbox.blockSignals(False)

    def _sync_platform_control_widgets(self) -> None:
        """Load the selected platform's drive settings without cross-contamination."""
        follower_type = self._configured_follower_type()
        self.follower_type_combo.blockSignals(True)
        self.follower_type_combo.setCurrentIndex(self.follower_type_combo.findData(follower_type))
        self.follower_type_combo.setEnabled(self._current_platform_key() != 'bunker')
        self.follower_type_combo.blockSignals(False)
        self._sync_diff_drive_checkbox()
        self.mur_arm_combo.blockSignals(True)
        self.mur_arm_combo.setCurrentIndex(self.mur_arm_combo.findData(self._configured_mur_arm()))
        self.mur_arm_combo.setEnabled(self._current_platform_key() == 'mur620_sim')
        self.mur_arm_combo.blockSignals(False)

    def _sync_platform_frame_widgets(self) -> None:
        for widget, value in (
            (self.robot_base_frame, self._configured_robot_base_frame()),
            (self.robot_tree_root_frame, self._configured_robot_tree_root_frame()),
        ):
            widget.blockSignals(True)
            widget.setText(value)
            widget.blockSignals(False)

    def _save_platform_control_setting(self, key: str, value) -> None:
        platform = self._current_platform_key()
        settings = self._config.setdefault('platform_control_settings', {})
        if not isinstance(settings, dict):
            settings = {}
            self._config['platform_control_settings'] = settings
        platform_settings = settings.setdefault(platform, {})
        if not isinstance(platform_settings, dict):
            platform_settings = {}
            settings[platform] = platform_settings
        platform_settings[key] = value

    def _set_platform(self, *_args) -> None:
        platform = self._current_platform_key()
        previous_platform = str(self._config.get('platform', '')).strip().lower()
        self._config['platform'] = platform
        if platform == 'mur620_sim' and previous_platform == LEGACY_MUR_PLATFORM and 'mur_arm' not in self._config:
            self._config['mur_arm'] = 'l'
        self._sync_platform_control_widgets()
        self._sync_platform_frame_widgets()
        self._sync_fixed_tool_offset_widgets()
        self._save_config()
        profile = OperatorWindow._arm_platform_profile(self)
        self.path_status.setText(f"{profile['path_topic']}: ready" if self._has_path else f"{profile['path_topic']}: waiting")

    def _set_mur_arm(self, *_args) -> None:
        self._config['mur_arm'] = self._current_mur_arm()
        self._save_config()
        profile = OperatorWindow._arm_platform_profile(self)
        self.arm_pose_status.setText(
            f"{profile.get('arm_tip_link', '/current_deposition_pose')}: ready"
            if self._has_arm_pose else 'Deposition pose: waiting'
        )

    def _set_follower_type(self, *_args) -> None:
        self._save_platform_control_setting('follower_type', self._current_follower_type())
        self._save_config()

    def _set_diff_drive_mode(self, enabled: bool) -> None:
        self._save_platform_control_setting('diff_drive_mode', bool(enabled))
        self._save_config()

    def _set_use_odometry_robot_pose(self, enabled: bool) -> None:
        if enabled and self.vicon_tcp_base_pose_fallback_checkbox.isChecked():
            self.vicon_tcp_base_pose_fallback_checkbox.setChecked(False)
        self._config['use_odometry_robot_pose'] = bool(enabled)
        self._save_config()

    def _set_use_vicon_tcp_base_pose_fallback(self, enabled: bool) -> None:
        if enabled and self.odometry_pose_checkbox.isChecked():
            self.odometry_pose_checkbox.setChecked(False)
        self._config['use_vicon_tcp_base_pose_fallback'] = bool(enabled)
        self._save_config()

    def _set_path_transform(self, *_args) -> None:
        directory = Path(self.path_folder.text().strip()).expanduser()
        transform = {
            'x': float(self.path_transform_x_spin.value()),
            'y': float(self.path_transform_y_spin.value()),
            'z': float(self.path_transform_z_spin.value()),
            'yaw_deg': float(self.path_transform_yaw_spin.value()),
        }
        platform_combo = getattr(self, 'platform_combo', None)
        platform = self._current_platform_key() if platform_combo is not None else ''
        if platform:
            platform_transforms = self._config.setdefault('path_transforms_by_platform_directory', {})
            if not isinstance(platform_transforms, dict):
                platform_transforms = {}
                self._config['path_transforms_by_platform_directory'] = platform_transforms
            per_platform = platform_transforms.setdefault(platform, {})
            if not isinstance(per_platform, dict):
                per_platform = {}
                platform_transforms[platform] = per_platform
            per_platform[self._path_directory_key(directory)] = transform
            self._save_config()
            return
        transforms = self._config.setdefault('path_transforms_by_directory', {})
        if not isinstance(transforms, dict):
            transforms = {}
            self._config['path_transforms_by_directory'] = transforms
        transforms[self._path_directory_key(directory)] = transform
        self._save_config()

    def _set_path_transform_values(self, x: float, y: float, z: float, yaw_deg: float) -> None:
        self._set_path_transform_widget_values(x, y, z, yaw_deg)
        self._set_path_transform()

    def _set_path_transform_widget_values(self, x: float, y: float, z: float, yaw_deg: float) -> None:
        for spin, value in (
            (self.path_transform_x_spin, x),
            (self.path_transform_y_spin, y),
            (self.path_transform_z_spin, z),
            (self.path_transform_yaw_spin, yaw_deg),
        ):
            spin.blockSignals(True)
            spin.setValue(float(value))
            spin.blockSignals(False)

    def _capture_tool_offset(self) -> None:
        if self.simulation_checkbox.isChecked():
            self._append_process_output('gui', 'UR TCP offset capture is available in hardware mode only')
            return
        transform = self.ros_bridge.lookup_tool_offset(
            'robot_arm_tool0', 'robot_arm_tool0_controller')
        if transform is None:
            self._append_process_output(
                'gui', 'cannot capture UR TCP offset: TF robot_arm_tool0 <- robot_arm_tool0_controller unavailable')
            return
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        offset = {
            'xyz': [float(translation.x), float(translation.y), float(translation.z)],
            'quaternion_xyzw': [float(rotation.x), float(rotation.y), float(rotation.z), float(rotation.w)],
        }
        platform = self._current_platform_key() or self._configured_platform()
        platform_offsets = self._config.setdefault('fixed_tool_offsets_by_platform', {})
        if not isinstance(platform_offsets, dict):
            platform_offsets = {}
            self._config['fixed_tool_offsets_by_platform'] = platform_offsets
        platform_offsets[platform] = offset
        self._save_config()
        self._sync_fixed_tool_offset_widgets()
        # The captured transform has just been saved, so it is now the configured value.
        self._set_tool_offset_capture_button_state(matches=True)
        self._append_process_output(
            'gui',
            'saved fixed tool offset from robot_arm_tool0 -> robot_arm_tool0_controller; '
            'restart arm controllers and follower to apply',
        )

    @staticmethod
    def _tool_offsets_match(
        configured: dict[str, list[float]],
        transform,
        tolerance: float = TOOL_OFFSET_COMPARISON_TOLERANCE,
    ) -> bool:
        """Compare a persisted tool offset with a TF transform.

        Quaternions q and -q describe the same orientation, so compare their
        normalized absolute dot product rather than their individual components.
        """
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        captured_xyz = (float(translation.x), float(translation.y), float(translation.z))
        captured_quaternion = (
            float(rotation.x), float(rotation.y), float(rotation.z), float(rotation.w),
        )
        configured_xyz = configured['xyz']
        configured_quaternion = configured['quaternion_xyzw']
        if any(
            not math.isclose(expected, actual, abs_tol=tolerance, rel_tol=0.0)
            for expected, actual in zip(configured_xyz, captured_xyz)
        ):
            return False

        configured_norm = math.sqrt(sum(value * value for value in configured_quaternion))
        captured_norm = math.sqrt(sum(value * value for value in captured_quaternion))
        if configured_norm == 0.0 or captured_norm == 0.0:
            return False
        quaternion_dot = sum(
            expected * actual
            for expected, actual in zip(configured_quaternion, captured_quaternion)
        ) / (configured_norm * captured_norm)
        return math.isclose(abs(quaternion_dot), 1.0, abs_tol=tolerance, rel_tol=0.0)

    def _tool_offset_matches_transform(self, transform) -> bool:
        return self._tool_offsets_match(self._configured_fixed_tool_offset(), transform)

    def _set_tool_offset_capture_button_state(self, matches: bool) -> None:
        if not hasattr(self, 'capture_tool_offset_button'):
            return
        color = '#c62828' if not matches else '#2e7d32'
        self.capture_tool_offset_button.setStyleSheet(
            f'QPushButton {{ background-color: {color}; color: white; }}'
        )
        self.capture_tool_offset_button.setToolTip(
            'Configured UR TCP offset differs from the robot TF; capture it to update the configuration.'
            if not matches else
            'Configured UR TCP offset matches the robot TF.'
        )

    def _check_configured_tool_offset(self) -> None:
        """Warn at startup if the hardware TCP calibration differs from configuration."""
        if self.simulation_checkbox.isChecked():
            return
        transform = self.ros_bridge.lookup_tool_offset(
            'robot_arm_tool0', 'robot_arm_tool0_controller')
        if transform is None:
            return
        matches = self._tool_offset_matches_transform(transform)
        self._set_tool_offset_capture_button_state(matches)
        if matches:
            return
        message = (
            'The configured UR TCP offset differs from the transform currently published by '
            'robot_arm_tool0 -> robot_arm_tool0_controller. Capture the UR TCP offset before '
            'starting the arm controllers or follower.'
        )
        self._append_process_output('gui', f'WARNING: {message}')
        QMessageBox.warning(self, 'UR TCP Offset Mismatch', message)

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
            transform = self._path_transform_for_directory(Path(folder))
            self._set_path_transform_widget_values(**transform)
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
        return 'map'

    def _save_hardware_topics(self) -> None:
        self._config['trajectory_directory'] = self.path_folder.text().strip()
        self._config['base_pose_topic'] = self.base_pose_topic.text().strip()
        self._config['arm_pose_topic'] = self.arm_pose_topic.text().strip()
        self._config['control_frame'] = self.control_frame.text().strip()
        self._config['external_map_frame'] = self.external_map_frame.text().strip()
        self._save_platform_control_setting('robot_base_frame', self.robot_base_frame.text().strip())
        self._save_platform_control_setting('robot_tree_root_frame', self.robot_tree_root_frame.text().strip())
        self._save_config()

    def _open_pid_gains_window(self) -> None:
        if self._pid_gains_dialog is not None and self._pid_gains_dialog.isVisible():
            self._pid_gains_dialog.raise_()
            self._pid_gains_dialog.activateWindow()
            return
        self._pid_gains_dialog = PidGainsDialog(
            self,
            self._configured_pid_gains(),
            self._pid_gain_defaults(),
        )
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
        arguments = [
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
        return arguments

    def _simulation_mode_changed(self, _enabled: bool) -> None:
        self._config['simulation'] = bool(_enabled)
        self._save_config()
        self._refresh_process_states()

    def _set_contour_control_enabled(self, enabled: bool) -> None:
        """Persist an explicit operator choice for the bounded correction path."""
        self._config['contour_control_enabled'] = bool(enabled)
        self._save_config()

    def _contour_control_enabled(self) -> bool:
        checkbox = getattr(self, 'contour_control_checkbox', None)
        if checkbox is not None:
            return bool(checkbox.isChecked())
        return bool(getattr(self, '_config', {}).get('contour_control_enabled', False))

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
        OperatorWindow._publish_stop_commands(self)
        if self.simulation_checkbox.isChecked():
            self._start_sim()
        else:
            self._start_pose_adapters()
        self._start_publish_path()
        if self.simulation_checkbox.isChecked() and self.odometry_pose_checkbox.isChecked():
            self._start_odometry_pose_adapter()
        arm_supported = OperatorWindow._arm_control_supported(self)
        if arm_supported:
            self._start_arm_controllers()
        self._start_path_index()
        self._start_base_follower()
        if arm_supported:
            self._start_arm_follower(move_to_start_pose=False)
        if self.simulation_checkbox.isChecked():
            if arm_supported:
                arm_ready_topic = '/am/move_arm_ready'
                self._start_move_arm_to_start(
                    wait_for_start_condition=True, ready_topic=arm_ready_topic)
                # Each mover waits for its own path and pose inputs.  The arm is
                # still gated by /start_pose_reached, without an arbitrary delay.
                self._start_move_base_to_start(
                    publish_start_condition=True, wait_for_ready_topic=arm_ready_topic)
            else:
                self._start_move_base_to_start(publish_start_condition=True)

    def _stop_launch_all_components(self) -> None:
        self.ros_bridge.publish_start_condition(False)
        OperatorWindow._publish_stop_commands(self)
        for timer in self._launch_all_timers:
            timer.stop()
            timer.deleteLater()
        self._launch_all_timers.clear()
        for name in self._launch_all_process_names():
            self.processes.stop(name)
        # Restarted simulators reset /clock. Clear the GUI's TF cache after all
        # transform publishers have stopped so the next run cannot be rejected
        # as TF_OLD_DATA from the previous clock epoch.
        self.ros_bridge.reset_tf_buffer()
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
            VICON_TCP_POSE_BACKUP_NAME,
            ARM_POSE_ADAPTER_NAME,
            VICON_EE_STATIC_TF_NAME,
            ARM_CONTROLLERS_NAME,
            BASE_FOLLOWER_NAME,
            ARM_FOLLOWER_NAME,
            MOVE_BASE_NAME,
            SWITCH_ARM_VELOCITY_NAME,
            BASE_ACCURACY_MONITOR_NAME,
            TCP_ACCURACY_MONITOR_NAME,
            ACCURACY_REPORT_NAME,
            RVIZ_NAME,
            SYNC_WORKSPACE_NAME,
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
        platform = self._current_platform_key()
        if platform == 'bunker':
            headless_value = 'false' if self._simulation_gui_enabled() else 'true'
            command = [
                'ros2',
                'launch',
                'bunker_description',
                'spawn_with_controllers.launch.py',
                f'headless:={headless_value}',
                'launch_rviz:=false',
                f'publish_robot_pose:={self._sim_publish_robot_pose()}',
            ]
        elif platform in {'mur620_sim', 'mur620_left_arm_sim'}:
            mur_native_arm = bool(self._current_platform_profile().get('mur_native_arm', False))
            gui_value = 'true' if self._simulation_gui_enabled() else 'false'
            command = [
                'ros2',
                'launch',
                'mur_launch_sim',
                'mur620.launch.py',
                'robot_name:=mur620a',
                'world:=empty',
                'x:=44.0',
                'y:=44.0',
                'z:=0.07',
                'Y:=0.0',
                'include_gz:=true',
                f'gazebo_gui:={gui_value}',
                f'use_camera:={gui_value}',
                f'enable_sensors:={gui_value}',
                f'use_simple_collisions:={str(not self._simulation_gui_enabled()).lower()}',
                'ground_truth:=true',
                'fake_localization:=true',
                'navigation:=false',
                f'load_arm_controllers:={str(mur_native_arm).lower()}',
                'load_lift_controllers:=false',
                'launch_moveit:=false',
                # AM launches its shared controller stack below. Keep MuR's
                # native J-PARSE enabled by default for standalone use only.
                'launch_jparse_idk:=false',
                'auto_switch_arm_controllers:=false',
            ]
        elif platform == 'robotnik':
            gui_value = 'true' if OperatorWindow._simulation_gui_enabled(self) else 'false'
            command = [
                'ros2',
                'launch',
                'robotnik_rbvogui_tum',
                'rbvogui_ur_standard_control.launch.py',
                f'gui:={gui_value}',
                'robot_id:=robot',
                'arm_type:=ur20',
                f'publish_robot_pose:={self._sim_publish_robot_pose()}',
            ]
        else:
            self._append_process_output(
                SIM_NAME,
                'refusing to launch simulation: select a valid platform explicitly',
            )
            return
        self._append_process_output(SIM_NAME, ' '.join(command))
        self.processes.start(SIM_NAME, command)

    def _simulation_gui_enabled(self) -> bool:
        if not self.simulation_checkbox.isChecked():
            return False
        configured = getattr(self, '_config', {}).get('simulation_gui', None)
        if configured is not None:
            return str(configured).strip().lower() in {'true', '1', 'yes', 'on'}
        env_value = os.environ.get('AM_OPERATOR_GUI_SIM_GUI')
        if env_value is not None:
            return env_value.strip().lower() in {'true', '1', 'yes', 'on'}
        return False

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
        profile = OperatorWindow._arm_platform_profile(self)
        diff_drive = self._diff_drive_mode()
        command = [
            'ros2',
            'run',
            'base_trajectory_follower',
            'simple_base_follower',
            '--ros-args',
            '-p', f'use_sim_time:={self._use_sim_time()}',
            '-p', 'path_topic:=/base_path_tracking',
            '-p', f"robot_pose_topic:={profile['robot_pose_topic']}",
            '-p', 'robot_pose_type:=pose_stamped',
            '-p', f"cmd_vel_topic:={profile['cmd_vel_topic']}",
            '-p', f"output_stamped:={str(bool(profile['output_stamped'])).lower()}",
            '-p', f"command_frame_id:={profile['command_frame_id']}",
            '-p', f'follower_type:={self._current_follower_type()}',
            '-p', f'diff_drive_mode:={str(diff_drive).lower()}',
            '-p', 'use_external_path_index:=true',
            '-p', 'path_index_topic:=/path_index',
            '-p', 'reference_pose_topic:=/base_trajectory_reference',
            '-p', f"external_path_index_stride:={int(self._base_smoothing('external_path_index_stride'))}",
            '-p', 'wait_for_start_condition:=true',
            '-p', 'start_condition_topic:=/start_condition',
            '-p', 'velocity_override_topic:=/velocity_override',
            '-p', 'lookahead_distance:=0.3',
            '-p', f"kp_x:={self._ros_float_literal(self._pid_gain('base_follower.kp_x'))}",
            '-p', f"kp_y:={self._ros_float_literal(self._pid_gain('base_follower.kp_y'))}",
            '-p', f"kp_yaw:={self._ros_float_literal(self._pid_gain('base_follower.kp_yaw'))}",
            '-p', f"max_vx:={self._ros_float_literal(self._pid_gain('base_follower.max_vx'))}",
            '-p', f"max_vy:={self._ros_float_literal(self._pid_gain('base_follower.max_vy'))}",
            '-p', f"max_wz:={self._ros_float_literal(self._pid_gain('base_follower.max_wz'))}",
            '-p', f"smooth_velocity_commands:={str(bool(self._base_smoothing('enabled'))).lower()}",
            '-p', f"velocity_smoothing_method:={self._base_smoothing('method')}",
            '-p', f"max_accel_x:={self._ros_float_literal(float(self._base_smoothing('max_accel_x')))}",
            '-p', f"max_accel_y:={self._ros_float_literal(float(self._base_smoothing('max_accel_y')))}",
            '-p', f"max_accel_wz:={self._ros_float_literal(float(self._base_smoothing('max_accel_wz')))}",
            '-p', f"moving_average_window_size:={int(self._base_smoothing('moving_average_window_size'))}",
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
        if not OperatorWindow._arm_control_supported(self):
            OperatorWindow._report_unsupported_arm_control(self, 'Arm follower')
            return
        simulation_checkbox = getattr(self, 'simulation_checkbox', None)
        simulation = simulation_checkbox is not None and simulation_checkbox.isChecked()
        profile = OperatorWindow._arm_platform_profile(self)
        mur_native_arm = bool(profile.get('mur_native_arm', False))
        command = [
            'ros2',
            'launch',
            'ur_trajectory_follower',
            'sideways_arm_control.launch.py',
            f'use_sim_time:={self._use_sim_time()}',
            f'robot_name:={"mur620a" if mur_native_arm else "robot"}',
            f'arm:={profile.get("arm_selected", "arm") if mur_native_arm else "arm"}',
            f'joint_prefix:={profile.get("arm_joint_prefix", "robot_arm_")}',
            f'base_link:={profile.get("arm_base_link", "robot_arm_base_link")}',
            f'tip_link:={profile.get("arm_tip_link", "robot_arm_tool0")}',
            f'path_frame:={self.control_frame.text().strip()}',
            f'robot_description_topic:={profile.get("robot_description_topic", "/robot/robot_description")}',
            f'joint_states_topic:={profile.get("joint_states_topic", "/robot/joint_states")}',
            f"velocity_command_topic:={self._arm_velocity_command_topic()}",
            'start_jparse_controller:=false',
            'start_command_transform:=false',
            f'publish_current_pose_from_tf:={str(mur_native_arm).lower()}',
            'publish_path:=false',
            'publish_path_index:=false',
            f'move_to_start_pose:={str(move_to_start_pose).lower()}',
            f"start_pose_trajectory_topic:={self._arm_trajectory_topic()}",
            'start_pose_publish_delay:=8.0',
            f'derive_nozzle_pose_from_tcp:={str(simulation).lower()}',
            'tcp_pose_topic:=/current_tcp_pose',
            'nozzle_pose_topic:=/current_nozzle_tip_pose',
            'current_pose_topic:=/current_deposition_pose',
            'spray_distance_topic:=/spray_distance',
            'smoothed_spray_distance_topic:=/spray_distance_smoothed',
            f'spray_distance_initial:={self._ros_float_literal(OperatorWindow._effective_spray_distance_m(self))}',
            f'spray_distance_max_rate:={self._ros_float_literal(OperatorWindow._configured_spray_distance_max_rate(self) if hasattr(self, "_configured_spray_distance_max_rate") else DEFAULT_SPRAY_DISTANCE_MAX_RATE)}',
            'path_topic:=/ur_path_transformed',
            'original_path_topic:=/ur_path_original',
            'normal_topic:=/normal_vector',
            'path_index_topic:=/path_index',
            'next_goal_topic:=/next_goal',
            'wait_for_start_condition:=true',
            'start_condition_topic:=/start_condition',
            f'initial_path_index:={self.index_spin.value()}',
            'progress_mode:=desired_speed' if getattr(self, 'default_velocity_checkbox', None) is not None and self.default_velocity_checkbox.isChecked() else 'progress_mode:=timestamp',
            'arm_reference_topic:=/arm_trajectory_reference',
            'desired_speed_topic:=/desired_arm_speed',
            f'default_velocity:={self._ros_float_literal(self._default_velocity_param())}',
            'contour_control_enabled:=' + str(OperatorWindow._contour_control_enabled(self)).lower(),
        ]
        if mur_native_arm:
            command.append(f'combined_twist_source_topic:={profile["arm_world_twist_topic"]}')
            if bool(profile.get('start_base_motion_compensation', False)):
                command.extend([
                    'start_base_motion_compensation:=true',
                    f'base_velocity_topic:={profile["base_velocity_topic"]}',
                    f'base_velocity_type:={profile["base_velocity_type"]}',
                    f'compensation_base_frame:={profile["compensation_base_frame"]}',
                    f'base_compensation_topic:={profile["base_compensation_topic"]}',
                ])
        command.extend(OperatorWindow._tool_offset_launch_arguments(self))
        command.extend(self._pid_launch_arguments(
            'arm_direction',
            ('kp_z', 'along_track_kp', 'orthogonal_kp',
             'max_along_track_correction', 'max_spray_axis_correction',
             'max_tracking_linear_velocity', 'final_position_tolerance'),
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
        publish_speed = getattr(self, '_publish_desired_arm_speed', None)
        if publish_speed is not None:
            publish_speed()
        profile = self._current_platform_profile()
        command = [
            'ros2',
            'run',
            'ur_trajectory_follower',
            'increment_path_index',
            '--ros-args',
            '-p', f'use_sim_time:={self._use_sim_time()}',
            '-p', 'path_index_topic:=/path_index',
            '-p', 'path_index_command_topic:=/path_index_command',
            '-p', 'next_goal_topic:=/next_goal',
            '-p', 'normal_topic:=/normal_vector',
            '-p', f'initial_path_index:={self.index_spin.value()}',
            '-p', 'path_topic:=/ur_path_transformed',
            '-p', f"base_path_topic:={profile['path_topic']}",
            '-p', f"progress_mode:={'desired_speed' if getattr(self, 'default_velocity_checkbox', None) is not None and self.default_velocity_checkbox.isChecked() else 'timestamp'}",
            '-p', 'arm_reference_topic:=/arm_trajectory_reference',
            '-p', 'base_reference_topic:=/base_trajectory_reference',
            '-p', 'processed_path_topic:=/ur_path_tracking',
            '-p', 'processed_base_path_topic:=/base_path_tracking',
            '-p', 'desired_speed_topic:=/desired_arm_speed',
            '-p', f"desired_arm_speed:={self._ros_float_literal(getattr(self, '_default_velocity_param', lambda: -1.0)())}",
            '-p', 'enable_path_resampling:=true',
            '-p', 'resample_spacing:=0.005',
            '-p', 'velocity_override_topic:=/velocity_override',
            '-p', 'start_condition_topic:=/start_condition',
            '-p', 'wait_for_start_condition:=true',
        ]
        self._append_process_output(PATH_INDEX_NAME, ' '.join(command))
        self.processes.start(PATH_INDEX_NAME, command)

    def _toggle_current_tcp_pose(self) -> None:
        sim_process = self.processes.get(SIM_NAME)
        if self.simulation_checkbox.isChecked() and sim_process is not None and sim_process.is_running():
            self._append_process_output(
                CURRENT_TCP_POSE_NAME,
                'simulation launch already publishes /current_tcp_pose; no second publisher started',
            )
            self._refresh_process_states()
            return
        if not self.simulation_checkbox.isChecked():
            running = any(
                (process := self.processes.get(name)) is not None and process.is_running()
                for name in (
                    VICON_BASE_STATIC_TF_NAME,
                    VICON_EE_STATIC_TF_NAME,
                    BASE_POSE_ADAPTER_NAME,
                    ODOMETRY_POSE_ADAPTER_NAME,
                    VICON_TCP_POSE_BACKUP_NAME,
                    ARM_POSE_ADAPTER_NAME,
                )
            )
            if running:
                self.processes.stop(VICON_BASE_STATIC_TF_NAME)
                self.processes.stop(VICON_EE_STATIC_TF_NAME)
                self.processes.stop(BASE_POSE_ADAPTER_NAME)
                self.processes.stop(ODOMETRY_POSE_ADAPTER_NAME)
                self.processes.stop(VICON_TCP_POSE_BACKUP_NAME)
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
            '-p', 'source_frame:=robot_arm_nozzle_tip',
            '-p', 'pose_topic:=/current_nozzle_tip_pose',
            '-p', 'publish_rate:=20.0',
        ]
        self._append_process_output(CURRENT_TCP_POSE_NAME, ' '.join(command))
        self.processes.start(CURRENT_TCP_POSE_NAME, command)

    def _toggle_base_accuracy_monitor(self) -> None:
        self._toggle_accuracy_monitor('base')

    def _toggle_tcp_accuracy_monitor(self) -> None:
        self._toggle_accuracy_monitor('tcp')

    def _toggle_accuracy_monitor(self, mode: str) -> None:
        name = BASE_ACCURACY_MONITOR_NAME if mode == 'base' else TCP_ACCURACY_MONITOR_NAME
        process = self.processes.get(name)
        if process is not None and process.is_running():
            self.processes.stop(name)
            self._append_process_output(name, 'stopped; CSV and JSON summary were written')
            self._refresh_process_states()
            return
        if not self._has_path or not self._has_robot_pose or (mode == 'tcp' and not self._has_arm_pose):
            self._append_process_output(name, 'not started: wait for fresh path and pose topics')
            return
        if mode == 'base':
            actual_topic, path_topic, reference_topic = (
                '/robot_pose', self._current_platform_profile()['path_topic'], '/base_trajectory_reference',
            )
        else:
            actual_topic, path_topic, reference_topic = (
                '/current_deposition_pose', '/ur_path_tracking', '/arm_trajectory_reference',
            )
        phase = str(self.accuracy_phase_combo.currentData())
        run_name = f'{mode}_{phase}_{datetime.now().strftime("%Y%m%dT%H%M%S")}'
        snapshot_path = Path('/tmp/am_trajectory_runs') / f'{run_name}_config.json'
        snapshot_path.parent.mkdir(parents=True, exist_ok=True)
        snapshot_path.write_text(json.dumps(self._config, indent=2, sort_keys=True) + '\n', encoding='utf-8')
        command = [
            'ros2', 'run', 'print_path_monitoring', 'trajectory_accuracy_monitor', '--ros-args',
            '-p', f'use_sim_time:={self._use_sim_time()}',
            '-p', f'mode:={mode}',
            '-p', f'actual_pose_topic:={actual_topic}',
            '-p', f'reference_path_topic:={path_topic}',
            '-p', f'reference_pose_topic:={reference_topic}',
            '-p', 'path_index_topic:=/path_index',
            '-p', 'output_directory:=/tmp/am_trajectory_runs',
            '-p', f'run_name:={run_name}',
            '-p', f'phase:={phase}',
            '-p', 'required_frame:=map',
            '-p', f"start_condition_topic:={'/start_pose_reached' if mode == 'base' else '/start_condition'}",
        ]
        if mode == 'tcp':
            command.extend([
                '-p', 'base_reference_path_topic:=/base_path',
                '-p', 'arm_base_offset:=[0.26,0.0,1.046]',
                '-p', 'command_twist_topic:=/ur_twist_world',
                '-p', 'joint_states_topic:=/robot/joint_states',
                '-p', f"max_tracking_linear_velocity:={self._ros_float_literal(self._pid_gain('arm_direction.max_tracking_linear_velocity'))}",
            ])
        self._append_process_output(name, ' '.join(command))
        self.processes.start(name, command)
        self._refresh_process_states()

    def _summarize_accuracy_runs(self) -> None:
        command = [
            'ros2', 'run', 'print_path_monitoring', 'trajectory_accuracy_report',
            '--input-directory', '/tmp/am_trajectory_runs',
            '--trajectory-directory', self.path_folder.text().strip(),
            '--arm-base-offset', '0.26,0,1.046',
        ]
        self._append_process_output(ACCURACY_REPORT_NAME, ' '.join(command))
        self.processes.start(ACCURACY_REPORT_NAME, command)

    def _start_pose_adapters(self) -> None:
        use_odometry_pose = self.odometry_pose_checkbox.isChecked()
        fallback_checkbox = getattr(self, 'vicon_tcp_base_pose_fallback_checkbox', None)
        use_vicon_tcp_fallback = bool(
            fallback_checkbox is not None and fallback_checkbox.isChecked()
        )
        if not use_odometry_pose and not use_vicon_tcp_fallback:
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
        elif use_vicon_tcp_fallback:
            base_command = [
                'ros2',
                'run',
                'am_operator_gui',
                'vicon_tcp_robot_pose_backup',
                '--ros-args',
                '-r', f'__node:={VICON_TCP_POSE_BACKUP_NAME}',
                '-p', f'use_sim_time:={self._use_sim_time()}',
                '-p', 'input_topic:=/vicon/tool_transformed',
                '-p', 'output_topic:=/robot_pose',
                '-p', f'map_frame:={self.external_map_frame.text().strip()}',
                '-p', f'robot_base_frame:={self.robot_base_frame.text().strip()}',
                '-p', 'robot_tcp_frame:=robot_arm_nozzle_tip',
                '-p', f'robot_tree_root_frame:={self.robot_tree_root_frame.text().strip()}',
                '-p', 'ready_topic:=/am/base_pose_ready',
                '-p', 'stale_timeout:=0.5',
            ]
            self._append_process_output(VICON_TCP_POSE_BACKUP_NAME, ' '.join(base_command))
            self.processes.start(VICON_TCP_POSE_BACKUP_NAME, base_command)
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
            '-p', 'output_topic:=/current_nozzle_tip_pose',
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
        if not OperatorWindow._arm_control_supported(self):
            OperatorWindow._report_unsupported_arm_control(self, 'Arm controller stack')
            return
        simulation = self.simulation_checkbox.isChecked()
        profile = OperatorWindow._arm_platform_profile(self)
        if profile.get('mur_native_arm', False):
            command = [
                'ros2', 'launch', 'am_operator_gui', 'mur_arm_velocity_stack.launch.py',
                f'use_sim_time:={self._use_sim_time()}', 'robot_name:=mur620a', f'arm:={profile["arm_selected"]}',
                f'path_frame:={self.control_frame.text().strip()}',
                f'arm_base_link:={profile["arm_base_link"]}',
                f'controller_frame:={profile.get("arm_command_frame", profile["arm_base_link"])}',
                f'source_twist_topic:={profile["arm_world_twist_topic"]}',
                f'controller_twist_topic:={profile["arm_stop_topic"]}',
                f'velocity_command_topic:={profile["arm_velocity_command_topic"]}',
                f'tip_link:={profile["arm_tip_link"]}',
                f'controller_tip_link:={profile.get("arm_command_tip_link", profile["arm_tip_link"])}',
                f'robot_description_topic:={profile["robot_description_topic"]}',
                f'joint_states_topic:={profile["joint_states_topic"]}',
                'spray_distance_topic:=/spray_distance_smoothed',
                'jparse_readiness_topic:=/am/jparse_ready',
                *self._tool_offset_launch_arguments(),
            ]
            self._append_process_output(ARM_CONTROLLERS_NAME, ' '.join(command))
            self.processes.start(ARM_CONTROLLERS_NAME, command)
            return
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
        command.extend(OperatorWindow._tool_offset_launch_arguments(self))
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

    def _start_move_base_to_start(
        self,
        publish_start_condition: bool = False,
        wait_for_ready_topic: str = '',
    ) -> None:
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
            '-p', f"target_yaw_mode:={profile.get('start_target_yaw_mode', 'auto')}",
            '-p', f'path_index:={self.index_spin.value()}',
            *( ['-p', f'wait_for_ready_topic:={wait_for_ready_topic}'] if wait_for_ready_topic else [] ),
            '-p', f'publish_start_condition:={str(publish_start_condition).lower()}',
            '-p', 'start_condition_topic:=/start_pose_reached',
            '-p', 'distance_tolerance:=0.06',
            '-p', 'yaw_tolerance:=0.08',
            '-p', f"kp_linear:={self._ros_float_literal(self._pid_gain('base_move.kp_linear'))}",
            '-p', f"kp_lateral:={self._ros_float_literal(self._pid_gain('base_move.kp_lateral'))}",
            '-p', f"kp_angular_to_point:={self._ros_float_literal(self._pid_gain('base_move.kp_angular_to_point'))}",
            '-p', f"kp_angular_reorient:={self._ros_float_literal(self._pid_gain('base_move.kp_angular_reorient'))}",
            '-p', f"max_linear_velocity:={self._ros_float_literal(self._pid_gain('base_move.max_linear_velocity'))}",
            '-p', f"max_lateral_velocity:={self._ros_float_literal(self._pid_gain('base_move.max_lateral_velocity'))}",
            '-p', f"max_angular_velocity:={self._ros_float_literal(self._pid_gain('base_move.max_angular_velocity'))}",
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

    def _start_move_arm_to_start(
        self,
        wait_for_start_condition: bool = False,
        ready_topic: str = '',
    ) -> None:
        if not OperatorWindow._arm_control_supported(self):
            OperatorWindow._report_unsupported_arm_control(self, 'Move arm to start')
            return
        profile = OperatorWindow._arm_platform_profile(self)
        command = [
            'ros2',
            'launch',
            'move_to_path_idx',
            'move_ur_to_path_idx.launch.py',
            f'use_sim_time:={self._use_sim_time()}',
            'path_topic:=/ur_path_transformed',
            'current_pose_topic:=/current_deposition_pose',
            f'path_index:={OperatorWindow._original_arm_path_index(self)}',
            f'wait_for_start_condition:={str(wait_for_start_condition).lower()}',
            'start_condition_topic:=/start_pose_reached',
            *( [f'ready_topic:={ready_topic}'] if ready_topic else [] ),
            f"cmd_vel_topic:={profile.get('arm_world_twist_topic', '/jparse_velocity_controller_ur/twist_cmd_world')}",
            f'path_frame:={self.control_frame.text().strip()}',
        ]
        command.extend(self._pid_launch_arguments(
            'arm_move',
            ('kp_linear', 'kp_angular', 'max_linear_velocity', 'max_angular_velocity'),
        ))
        self._append_process_output(MOVE_ARM_NAME, ' '.join(command))
        self.processes.start(MOVE_ARM_NAME, command)

    def _switch_arm_velocity_controller(self) -> None:
        self._start_switch_arm_velocity_controller()
        self._refresh_process_states()

    def _start_switch_arm_velocity_controller(self) -> None:
        if not OperatorWindow._arm_control_supported(self):
            OperatorWindow._report_unsupported_arm_control(self, 'Switch arm velocity controller')
            return
        command = [
            'ros2',
            'control',
            'switch_controllers',
            '--controller-manager',
            self._arm_controller_manager(),
            '--deactivate',
            f'joint_trajectory_controller_{self._current_mur_arm()}'
            if OperatorWindow._arm_platform_profile(self).get('mur_native_arm', False)
            else 'joint_trajectory_controller',
            '--activate',
            (
                f'forward_velocity_controller_{self._current_mur_arm()}'
                if OperatorWindow._arm_platform_profile(self).get('mur_native_arm', False)
                else 'arm_forward_velocity_controller' if self.simulation_checkbox.isChecked()
                else 'forward_velocity_controller'
            ),
        ]
        self._append_process_output(SWITCH_ARM_VELOCITY_NAME, ' '.join(command))
        self.processes.start(SWITCH_ARM_VELOCITY_NAME, command)

    def _start_following(self) -> None:
        if not self._motion_ready():
            self._append_process_output('safety', self._motion_not_ready_reason())
            return
        missing_processes = self._missing_control_process_names()
        if missing_processes:
            self._append_process_output(
                'safety',
                'following blocked; missing control process(es): '
                + ', '.join(missing_processes),
            )
            return
        self.ros_bridge.publish_path_index(self.index_spin.value())
        self._append_process_output(
            'ros',
            f'published /path_index_command {self.index_spin.value()}',
        )
        self._start_condition_publish_count = 5
        self._publish_start_condition_once(True)
        self._style_button(self.start_following_button, 'green')

    def _stop_following(self) -> None:
        self._start_condition_publish_count = 5
        self._publish_start_condition_once(False)
        for delay_ms in range(0, 1000, 100):
            QTimer.singleShot(
                delay_ms,
                lambda: OperatorWindow._publish_stop_commands(self),
            )
        self._style_button(self.stop_following_button, 'red')

    def _publish_start_condition_once(self, value: bool) -> None:
        self.ros_bridge.publish_start_condition(value)
        self._append_process_output('ros', f'published /start_condition {str(value).lower()}')
        self._start_condition_publish_count -= 1
        if self._start_condition_publish_count > 0:
            QTimer.singleShot(200, lambda: self._publish_start_condition_once(value))

    def _open_rviz(self) -> None:
        rviz_name = (
            'bunker_operator.rviz'
            if self._current_platform_key() == 'bunker'
            else 'robotnik_operator.rviz'
        )
        rviz_config = Path(get_package_share_directory('am_operator_gui')) / 'rviz' / rviz_name
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
        self._config['path_index'] = int(value)
        self.ros_bridge.publish_path_index(value)
        self._append_process_output('ros', f'published /path_index_command {value}')

    def _original_arm_path_index(self) -> int:
        spin = getattr(self, 'original_arm_index_spin', self.index_spin)
        return int(spin.value())

    def _set_original_arm_index_from_tracking(self, value: int) -> None:
        if self._updating_path_indices:
            return
        mapper = getattr(self.ros_bridge, 'original_arm_index_for_tracking_index', None)
        original_index = int(mapper(value)) if mapper is not None else int(value)
        self._updating_path_indices = True
        self.original_arm_index_spin.setValue(max(0, original_index))
        self._updating_path_indices = False

    def _set_tracking_index_from_original_arm(self, value: int) -> None:
        if self._updating_path_indices:
            return
        mapper = getattr(self.ros_bridge, 'tracking_arm_index_for_original_index', None)
        tracking_index = int(mapper(value)) if mapper is not None else int(value)
        self._updating_path_indices = True
        self.index_spin.setValue(max(0, tracking_index))
        config = getattr(self, '_config', None)
        if isinstance(config, dict):
            config['original_arm_index'] = int(value)
        self._updating_path_indices = False

    @staticmethod
    def _ros_float_literal(value: float) -> str:
        return f'{float(value):.6f}'

    def _set_default_velocity_enabled(self, enabled: bool) -> None:
        self.default_velocity_spin.setEnabled(enabled)
        self._config['default_velocity_enabled'] = bool(enabled)
        self._save_config()
        self._publish_desired_arm_speed()

    def _set_default_velocity(self, value: float) -> None:
        self._config['default_velocity'] = float(value)
        self._save_config()
        self._publish_desired_arm_speed()

    def _default_velocity_param(self) -> float:
        if not self.default_velocity_checkbox.isChecked():
            return -1.0
        return self.default_velocity_spin.value()

    def _publish_desired_arm_speed(self) -> None:
        publisher = getattr(self.ros_bridge, 'publish_desired_arm_speed', None)
        if publisher is not None:
            publisher(max(0.0, self._default_velocity_param()))

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
        self.ros_bridge.publish_spray_distance(effective_mm / 1000.0)

    def _set_spray_distance_mm(self, value: float) -> None:
        self._config['spray_distance_mm'] = float(value)
        self._save_config()

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
        was_safety_ready = getattr(self, '_last_safety_gate_ready', False)
        safety_ready = all((has_path, has_robot_pose, has_arm_pose, jparse_ready, controller_ready))
        self._last_safety_gate_ready = safety_ready
        self._has_path = has_path
        self._has_robot_pose = has_robot_pose
        self._has_arm_pose = has_arm_pose
        self._jparse_ready = jparse_ready
        self._controller_ready = controller_ready
        profile = OperatorWindow._arm_platform_profile(self)
        path_topic = str(profile['path_topic'])
        pose_topic = str(profile['robot_pose_topic'])
        self.path_status.setText(f'{path_topic}: ready' if has_path else f'{path_topic}: waiting')
        self.pose_status.setText(f'{pose_topic}: ready' if has_robot_pose else f'{pose_topic}: waiting')
        self.arm_pose_status.setText(
            '/current_deposition_pose: ready' if has_arm_pose else '/current_deposition_pose: waiting'
        )
        arm_ready = jparse_ready and controller_ready
        self.arm_control_status.setText(
            'arm control: ready' if arm_ready else 'arm control: waiting'
        )
        if was_safety_ready and not safety_ready:
            self._append_process_output('safety', 'readiness lost; publishing stop commands')
            self.ros_bridge.publish_start_condition(False)
            OperatorWindow._publish_stop_commands(self)
        self._refresh_process_states()

    def _publish_stop_commands(self) -> None:
        """Stop the selected platform directly, even if controller bridges died."""
        profile = OperatorWindow._arm_platform_profile(self)
        arm_topic = str(profile.get('arm_stop_topic', '/jparse_velocity_controller_ur/twist_cmd_world'))
        arm_frame = str(profile.get('arm_stop_frame', self.control_frame.text().strip()))
        try:
            self.ros_bridge.publish_stop_commands(
                arm_frame,
                base_topic=str(profile['cmd_vel_topic']),
                base_stamped=bool(profile['output_stamped']),
                base_frame=str(profile['command_frame_id']),
                arm_topic=arm_topic,
            )
        except TypeError:
            # Lightweight test doubles and third-party bridge integrations may
            # still implement the historic one-argument method.
            self.ros_bridge.publish_stop_commands(arm_frame)

    def _set_path_index_from_ros(self, value: int) -> None:
        self.index_spin.blockSignals(True)
        self.index_spin.setValue(max(0, value))
        self.index_spin.blockSignals(False)
        if hasattr(self, 'original_arm_index_spin'):
            self._set_original_arm_index_from_tracking(value)

    def _refresh_process_states(self) -> None:
        self._set_launch_button_state()
        self._set_sim_button_state()
        self._set_publish_path_state()
        self._set_path_index_button_state()
        self._set_current_tcp_pose_button_state()
        self._set_accuracy_monitor_button_states()
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
                    VICON_TCP_POSE_BACKUP_NAME,
                    ARM_POSE_ADAPTER_NAME,
                )
            )
            self.current_tcp_pose_button.setText(
                'Stop Transformations' if running else 'Launch Transformations'
            )
            self._style_button(self.current_tcp_pose_button, 'green' if running else 'grey')
            return
        sim_process = self.processes.get(SIM_NAME)
        if sim_process is not None and sim_process.is_running():
            self.current_tcp_pose_button.setText('TCP Pose from Sim')
            self._style_button(self.current_tcp_pose_button, 'green')
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

    def _set_accuracy_monitor_button_states(self) -> None:
        self._set_process_toggle_button(
            self.base_accuracy_monitor_button,
            BASE_ACCURACY_MONITOR_NAME,
            'Stop Base Recording',
            'Record Base Accuracy',
        )
        self._set_process_toggle_button(
            self.tcp_accuracy_monitor_button,
            TCP_ACCURACY_MONITOR_NAME,
            'Stop TCP Recording',
            'Record TCP Accuracy',
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
        profile = OperatorWindow._arm_platform_profile(self)
        if profile.get('mur_native_arm', False):
            return str(profile['arm_controller_manager'])
        return '/robot/controller_manager' if self.simulation_checkbox.isChecked() else '/robot/arm/controller_manager'

    def _arm_velocity_command_topic(self) -> str:
        profile = OperatorWindow._arm_platform_profile(self)
        if profile.get('mur_native_arm', False):
            return str(profile['arm_velocity_command_topic'])
        if self.simulation_checkbox.isChecked():
            return '/robot/arm_forward_velocity_controller/commands'
        return '/robot/arm/forward_velocity_controller/commands'

    def _arm_trajectory_topic(self) -> str:
        profile = self._current_platform_profile()
        if profile.get('mur_native_arm', False):
            return str(profile['arm_trajectory_topic'])
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

    def _detach_process_output_callbacks(self) -> None:
        self.processes._output_callback = None
        for process in self.processes._processes.values():
            process.output_callback = None

    def closeEvent(self, event) -> None:
        self._detach_process_output_callbacks()
        service = getattr(self, 'service', None)
        if service is not None:
            service.close()
        else:  # compatibility for lightweight reference tests/adapters
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
