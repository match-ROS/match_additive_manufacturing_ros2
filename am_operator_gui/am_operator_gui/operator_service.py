"""Toolkit-neutral control and process layer shared by local operator UIs.

This deliberately owns neither Qt nor FastAPI.  A presentation layer supplies
settings and invokes named actions; status and bounded logs are read back as data.
"""

from __future__ import annotations

import math
import os
import shlex
import sys
from collections import deque
from copy import deepcopy
from datetime import datetime, timezone
from pathlib import Path
from threading import Lock, Timer
from typing import Any, Callable, Optional

from .tool_transforms import (
    DEFAULT_VICON_INPUT_TOPIC, DEFAULT_VICON_NOZZLE_TRANSFORM,
    VICON_NOZZLE_TOPIC, validated_transform,
)
from .config_store import ConfigStore
from .robot_debug import RobotDebugInfo
from .process_manager import ProcessRegistry
from .ur_dashboard import (
    ForwardVelocityControllerInfo, UrDashboardInfo, UrStatusMonitor, dashboard_command, enable_command,
)


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[2]
SYNC_REMOTE_TARGET = 'robot@192.168.0.200:~/b04_gui_ws/src/'


def _installed_config_path() -> Path:
    """Use the ROS share directory after installation, source-tree path otherwise."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory('am_operator_gui')) / 'config' / 'operator_gui_config.json'
    except Exception:
        return PACKAGE_ROOT / 'config' / 'operator_gui_config.json'


CONFIG_PATH = _installed_config_path()
ASSET_ROOT = CONFIG_PATH.parent.parent
LEGACY_CONFIG_PATH = Path.home() / '.config' / 'am_operator_gui' / 'operator_gui_config.json'

PROFILES = {
    'robotnik': {'cmd_vel': '/robot/robotnik_base_control/cmd_vel_unstamped', 'stamped': False,
                 'frame': 'base_link', 'odom': '/robot/robotnik_base_control/odom',
                 'robot_pose': '/robot_pose', 'path': '/base_path'},
    'bunker': {'cmd_vel': '/diff_drive_controller/cmd_vel', 'stamped': True,
               'frame': 'base_footprint', 'odom': '/odom', 'robot_pose': '/robot_pose', 'path': '/base_path'},
}

DEFAULT_BATTERY_TOPICS = {
    'robotnik': '/robot/battery_estimator/data',
    'bunker': '',
}

DEFAULT_BASE_HARDWARE = {
    'robotnik': {'expected_motor_ids': list(range(1, 9))},
    'bunker': {'expected_motor_ids': []},
}

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

DEFAULT_BASE_SMOOTHING = {
    'enabled': True,
    'method': 'moving_average',
    'max_accel_x': 0.25,
    'max_accel_y': 0.25,
    'max_accel_wz': 0.5,
    'moving_average_window_size': 5,
    'external_path_index_stride': 10,
}

DEFAULT_JPARSE_LIMITS = {
    'max_joint_velocity': 1.5,
    'max_joint_acceleration': 0.5,
    'max_cartesian_linear_velocity': 0.25,
    'max_cartesian_angular_velocity': 0.8,
}

DEFAULT_PATH_TRANSFORM = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw_deg': 0.0}

POSE_ADAPTER_PROCESSES = (
    'nozzle_static_tf',
    'vicon_base_static_tf', 'vicon_ee_static_tf', 'vicon_fallback_nozzle_tf', 'base_pose_adapter',
    'odometry_pose_adapter', 'vicon_tcp_pose_backup', 'arm_pose_adapter',
)

TOGGLE_ACTIONS = {
    'vicon': ('vicon', 'Start Vicon', 'Stop Vicon'),
    'simulation': ('simulation', 'Launch Sim', 'Stop Sim'),
    'publish_path': ('publish_path', 'Publish Path', 'Stop Path'),
    'index_pose_preview': ('index_pose_preview', 'Show Index Poses', 'Stop Index Poses'),
    'path_index': ('path_index', 'Launch Path Index', 'Stop Path Index'),
    'base_follower': ('base_follower', 'Launch Base Follower', 'Stop Base Follower'),
    'arm_follower': ('arm_follower', 'Launch Arm Follower', 'Stop Arm Follower'),
    'controllers': ('controllers', 'Start Controllers', 'Stop Controllers'),
    'base_accuracy': ('base_accuracy', 'Record Base Accuracy', 'Stop Base Recording'),
    'tcp_accuracy': ('tcp_accuracy', 'Record TCP Accuracy', 'Stop TCP Recording'),
    'sync_workspace': ('sync_workspace', 'Sync Workspace', 'Stop Sync'),
}

ONE_SHOT_ACTIONS = {
    'move_base': ('move_base', 'Move Base To Start', 'Stop Base Move'),
    'move_arm': ('move_arm', 'Move Arm To Start', 'Stop Arm Move'),
    'switch_arm_velocity': ('switch_arm_velocity', 'Switch Arm Velocity', 'Switching Arm Velocity'),
    'accuracy_report': ('accuracy_report', 'Summarize Accuracy', 'Summarizing Accuracy'),
    'check_hardware_topics': ('check_hardware_topics', 'Check Hardware Topics', 'Checking Hardware Topics'),
    'restart_arm_controllers': (
        'restart_arm_controllers', 'Restart Controllers', 'Restarting Controllers'
    ),
}

# These descriptions are returned with every action state and become the
# browser button tooltips.  Keep them operational so hovering explains the
# action in addition to reporting its current process state.
ACTION_DESCRIPTIONS = {
    'index_pose_preview': ('Publiziert die gewählten Arm-/Base-Trackingindexposen auf '
                           '/selected_arm_index_pose und /selected_base_index_pose für RViz.'),
    'simulation': (
        'Startet ausschließlich die Simulation des gewählten Bunker- oder '
        'Robotnik-Profils; Show simulator window steuert das Gazebo-Fenster.'
    ),
    'publish_path': (
        'Lädt die exportierten Arm-, Base- und Normalenpfade und publiziert sie nach '
        'Anwendung der gespeicherten Translation und Gierrotation.'
    ),
    'path_index': (
        'Resampelt den gekoppelten Arm-/Base-Pfad auf 5 mm und publiziert Trackingpfade, '
        'Referenzposen und den gemeinsamen /path_index.'
    ),
    'base_follower': (
        'Folgt /base_path_tracking mit /robot_pose und dem plattformspezifischen '
        'cmd_vel-Topic; Bewegung beginnt erst mit /start_condition=true.'
    ),
    'arm_follower': (
        'Regelt den Arm entlang von /ur_path_transformed mit Normalen, /path_index und '
        'Arm-Referenzpose und sendet Welt-Twist-Kommandos.'
    ),
    'controllers': (
        'Transformiert Arm-Welt-Twist in den Controller-Rahmen und schaltet vom '
        'Trajectory- auf den passenden Velocity-Controller.'
    ),
    'base_accuracy': (
        'Zeichnet die Abweichung von /robot_pose zu Base-Pfad und Base-Referenzpose '
        'ab /start_pose_reached nach /tmp/am_trajectory_runs auf.'
    ),
    'tcp_accuracy': (
        'Zeichnet die Abweichung von /current_deposition_pose zum Tracking-Armpfad '
        'und zur Arm-Referenzpose ab /start_condition auf.'
    ),
    'sync_workspace': 'Copies workspace sources to robot@192.168.0.200:~/b04_gui_ws/src/ via rsync. Build and launch on the robot separately.',
    'move_base': (
        'Fährt die Base einmalig zur Pose des gemeinsamen Trackingindex in /base_path_tracking mit '
        '/robot_pose und plattformspezifischem cmd_vel-Topic.'
    ),
    'move_arm': (
        'Fährt den Arm einmalig zur Pose des gemeinsamen Trackingindex in '
        '/ur_path_tracking; dies ist dieselbe Quelle wie /next_goal.'
    ),
    'switch_arm_velocity': (
        'Deaktiviert per ros2 control den Joint-Trajectory-Controller und aktiviert '
        'den passenden Velocity-Controller für Simulation oder Hardware.'
    ),
    'accuracy_report': (
        'Erstellt aus den Läufen in /tmp/am_trajectory_runs einen Genauigkeitsbericht '
        'für das ausgewählte Pfadverzeichnis.'
    ),
    'check_hardware_topics': (
        'Prüft die ROS-Graph-Verträge der externen Hardware-Eingänge und Kommando-Endpunkte; '
        'dies ist kein Frische-, Controllerzustands- oder Sicherheitstest.'
    ),
    'restart_arm_controllers': (
        'Wartet auf den UR-Controller-Manager, deaktiviert strikt alle momentan aktiven '
        'Arm-Controller und aktiviert anschließend die Status-Controller sowie ausschließlich '
        'forward_velocity_controller. Trajectory-, position- und weitere Bewegungscontroller '
        'bleiben deaktiviert.'
    ),
    'transformations': (
        'Simulation: leitet TCP-/Nozzle-Pose aus Robot-TF, Werkzeugoffset und '
        'Sprühabstand ab. Hardware: startet die Vicon-/Odometrie-Posekette.'
    ),
    'vicon': 'Starts the Vicon receiver at 192.168.0.30:8802 using the configured Control frame as world_frame. Stop and restart to apply frame changes.',
    'pose_adapters': (
        'Erzeugt auf Hardware Base- und Nozzle-Pose aus Vicon, Odometry oder Tool-TF '
        'und publiziert die standardisierten Pose-Topics.'
    ),
    'rviz': 'Öffnet RViz mit der zum Plattformprofil passenden Konfiguration.',
    'capture_tool_offset': (
        'Liest den TF robot_arm_tool0 → robot_arm_tool0_controller und speichert ihn '
        'als Flansch-zu-Nozzle-Offset.'
    ),
    'calculate_path_transform': (
        'Berechnet aus /robot_pose und /base_path am gewählten Index die starre '
        'Pfadtranslation und Gierrotation.'
    ),
}


class OperatorService:
    """Configuration, process lifecycle and ROS command construction.

    ROS publishing is optional so that the web app can still show diagnostics on a
    machine where ROS has not been sourced yet.
    """

    def __init__(
        self,
        config_path: Path | None = None,
        output_callback: Optional[Callable[[str, str], None]] = None,
        status_callback: Optional[Callable[[bool, bool, bool, bool, bool], None]] = None,
        path_index_callback: Optional[Callable[[int], None]] = None,
    ) -> None:
        if config_path is None:
            configured_path = os.environ.get('AM_OPERATOR_GUI_CONFIG', '').strip()
            config_path = Path(configured_path).expanduser() if configured_path else CONFIG_PATH
        self.store = ConfigStore(config_path, LEGACY_CONFIG_PATH if config_path == CONFIG_PATH else None)
        self.config: dict[str, Any] = self.store.load()
        # The former EE field selected a downstream pose. Preserve a custom
        # measured topic as the input to the now explicit calibration chain.
        legacy_topic = str(self.config.get('arm_pose_topic', VICON_NOZZLE_TOPIC))
        self.config.setdefault('vicon_input_topic',
                               legacy_topic if legacy_topic != VICON_NOZZLE_TOPIC else DEFAULT_VICON_INPUT_TOPIC)
        self.config['arm_pose_topic'] = VICON_NOZZLE_TOPIC
        self.logs: deque[dict[str, str]] = deque(maxlen=1000)
        self._lock = Lock()
        self._output_callback = output_callback
        self._status_callback = status_callback
        self._external_path_index_callback = path_index_callback
        self.processes = ProcessRegistry(output_callback=self._on_output)
        self.ros_bridge = None
        self.ros_error: str | None = None
        self._battery_level: float | None = None
        self._base_hardware: dict[str, Any] = {
            'supported': False, 'level': 'neutral', 'summary': 'Warte auf ROS',
            'any_motor_disabled': None, 'emergency_stop': None, 'motors': {},
        }
        self.robot_debug = RobotDebugInfo()
        self.ur_status_monitor = UrStatusMonitor()
        self.ur_dashboard = UrDashboardInfo(self.ur_status_monitor)
        self.forward_velocity_controller = ForwardVelocityControllerInfo(self.ur_status_monitor)
        self._remote_bringup_run = 0
        self._status = {'path': False, 'robot_pose': False, 'arm_pose': False,
                        'jparse_ready': False, 'controller_ready': False}
        self._launch_all_active = False
        self._timers: list[Timer] = []
        self._following_active = False
        self._last_action_messages: dict[str, str] = {}
        self._last_action_success: dict[str, bool] = {}
        self._hardware_topic_results: list[str] = []
        self._live_path_index: int | None = None
        self._live_original_arm_index: int | None = None

    def _on_output(self, source: str, message: str) -> None:
        with self._lock:
            self.logs.append({
                'source': source,
                'message': message,
                'level': self._log_level(message),
                'timestamp': datetime.now(timezone.utc).isoformat(),
            })
        if self._output_callback is not None:
            self._output_callback(source, message)

    def log(self, source: str, message: str) -> None:
        self._on_output(source, message)

    @staticmethod
    def _log_level(message: str) -> str:
        """Classify regular ROS/launch output for the web console.

        ROS 2 normally emits ``[INFO]``, ``[WARN]`` and ``[ERROR]``.  Some
        launch tools omit brackets, so recognise both forms while retaining
        unlabelled output as information.
        """
        upper = message.upper()
        if any(token in upper for token in ('[FATAL]', '[ERROR]', ' FATAL', ' ERROR')):
            return 'error'
        if any(token in upper for token in ('[WARN]', '[WARNING]', ' WARN', ' WARNING')):
            return 'warning'
        if any(token in upper for token in ('[DEBUG]', '[TRACE]', ' DEBUG', ' TRACE')):
            return 'debug'
        return 'info'

    def ensure_ros(self) -> bool:
        if self.ros_bridge is not None:
            return True
        try:
            from .ros_bridge import RosBridge
            self.ros_bridge = RosBridge(
                status_callback=self._on_ros_status,
                path_index_callback=self._on_path_index,
                battery_callback=self._on_battery_level,
                battery_topic=self._battery_topic(),
                control_frame=str(self._setting('control_frame', 'map')),
                base_hardware_callback=self._on_base_hardware,
                expected_motor_ids=() if bool(self._setting('simulation', False)) else tuple(self._base_hardware_config()['expected_motor_ids']),
                base_odom_topic=str(self._profile()['odom']),
            )
            self.ros_bridge.start()
            # The status monitor shares rclpy with RosBridge but owns its own
            # node/executor and its long-lived service clients.
            self.ur_status_monitor.start()
            return True
        except Exception as exc:  # ROS is intentionally an optional web-server dependency
            self.ros_error = str(exc)
            self.log('ros', f'ROS bridge unavailable: {exc}')
            return False

    def _on_ros_status(self, path: bool, robot: bool, arm: bool, jparse: bool, controller: bool) -> None:
        self._status.update(path=path, robot_pose=robot, arm_pose=arm,
                            jparse_ready=jparse, controller_ready=controller)
        if self._status_callback is not None:
            self._status_callback(path, robot, arm, jparse, controller)

    def _on_path_index(self, value: int) -> None:
        """Keep the ROS-published progress visible without persisting every tick."""
        index = max(0, int(value))
        self._live_path_index = index
        mapper = getattr(self.ros_bridge, 'original_arm_index_for_tracking_index', None)
        try:
            self._live_original_arm_index = int(mapper(index)) if mapper is not None else index
        except Exception:
            self._live_original_arm_index = index
        if self._external_path_index_callback is not None:
            self._external_path_index_callback(index)

    def _on_battery_level(self, level: float | None) -> None:
        self._battery_level = level

    def _on_base_hardware(self, status: dict) -> None:
        self._base_hardware = deepcopy(status)

    def update_config(self, values: dict[str, Any]) -> dict[str, Any]:
        # Keep the public web endpoint deliberately explicit; unknown keys cannot
        # accidentally become command-line arguments.
        allowed = {'simulation', 'simulation_gui', 'platform', 'trajectory_directory', 'control_frame',
                   'base_pose_topic', 'vicon_input_topic', 'vicon_nozzle_transform',
                   'vicon_fallback_nozzle_transform',
                   'vicon_nozzle_transform_input_mode', 'external_map_frame',
                   'robot_base_frame', 'robot_tree_root_frame', 'use_odometry_robot_pose',
                   'use_vicon_tcp_base_pose_fallback', 'default_velocity',
                   'default_velocity_enabled', 'spray_distance_mm', 'path_transform',
                   'path_transforms_by_directory', 'platform_control_settings', 'pid_gains',
                   'base_smoothing', 'fixed_tool_offset', 'fixed_tool_offsets_by_platform',
                   'fixed_tool_offset_input_mode', 'path_index', 'original_arm_index',
                   'velocity_override', 'nozzle_offset_mm', 'follower_type', 'diff_drive_mode',
                   'direction_mode', 'accuracy_phase', 'battery_topic', 'battery_topics_by_platform',
                   'base_hardware_by_platform'}
        accepted = {key: value for key, value in values.items() if key in allowed}
        try:
            if 'battery_topics_by_platform' in accepted:
                accepted['battery_topics_by_platform'] = self._validated_battery_topics(
                    accepted['battery_topics_by_platform'])
            if 'battery_topic' in accepted:
                battery_topic = self._validated_battery_topic(accepted.pop('battery_topic'))
                topics = accepted.get('battery_topics_by_platform', self._battery_topics())
                platform = self._platform_name(accepted.get('platform', self._platform_key()))
                topics[platform] = battery_topic
                accepted['battery_topics_by_platform'] = topics
            if 'vicon_input_topic' in accepted:
                topic = str(accepted['vicon_input_topic']).strip()
                if not topic.startswith('/') or any(char.isspace() for char in topic):
                    raise ValueError('Vicon input must be an absolute ROS topic without whitespace')
                if topic.rstrip('/') in {VICON_NOZZLE_TOPIC, '/current_nozzle_tip_pose', '/current_deposition_pose'}:
                    raise ValueError('Vicon input must be an external measurement, not a local pose output')
                accepted['vicon_input_topic'] = topic
            if 'vicon_nozzle_transform' in accepted:
                accepted['vicon_nozzle_transform'] = validated_transform(accepted['vicon_nozzle_transform'])
            if 'vicon_fallback_nozzle_transform' in accepted:
                accepted['vicon_fallback_nozzle_transform'] = validated_transform(accepted['vicon_fallback_nozzle_transform'])
            if 'path_index' in accepted:
                accepted['path_index'] = max(0, int(accepted['path_index']))
            if 'original_arm_index' in accepted:
                accepted['original_arm_index'] = max(0, int(accepted['original_arm_index']))
            for key in ('default_velocity', 'velocity_override', 'spray_distance_mm', 'nozzle_offset_mm'):
                if key in accepted:
                    accepted[key] = self._finite_number(key, accepted[key])
        except (TypeError, ValueError) as exc:
            raise ValueError(f'Invalid setting: {exc}') from exc
        if 'original_arm_index' in accepted and 'path_index' not in accepted:
            # The manual source-path field is a selection aid.  All managed
            # motions use the resampled tracking paths, so convert it before
            # launching a mover or publishing the shared progress index.
            mapper = getattr(self.ros_bridge, 'tracking_arm_index_for_original_index', None)
            try:
                accepted['path_index'] = max(0, int(mapper(accepted['original_arm_index']))) if mapper else accepted['original_arm_index']
            except Exception:
                accepted['path_index'] = accepted['original_arm_index']
        self.config.update(accepted)
        if 'control_frame' in accepted and self.ros_bridge is not None:
            self.ros_bridge.set_control_frame(str(accepted['control_frame']))
        if self.ros_bridge is not None and ({'platform', 'battery_topics_by_platform'} & set(accepted)):
            self._battery_level = None
            setter = getattr(self.ros_bridge, 'set_battery_topic', None)
            if setter is not None:
                setter(self._battery_topic())
        if self.ros_bridge is not None and ({'platform', 'simulation', 'base_hardware_by_platform'} & set(accepted)):
            setter = getattr(self.ros_bridge, 'set_base_hardware_expectations', None)
            if setter is not None:
                setter(() if bool(self._setting('simulation', False)) else tuple(self._base_hardware_config()['expected_motor_ids']))
        if 'path_index' in accepted:
            self._live_path_index = accepted['path_index']
        if 'original_arm_index' in accepted:
            self._live_original_arm_index = accepted['original_arm_index']
        self.store.save(self.config)
        if self.ros_bridge is not None:
            if 'use_odometry_robot_pose' in accepted or 'use_vicon_tcp_base_pose_fallback' in accepted:
                self.ros_bridge.publish_robot_pose_modes(
                    bool(self._setting('use_odometry_robot_pose', False)),
                    bool(self._setting('use_vicon_tcp_base_pose_fallback', False)))
            if 'path_index' in accepted:
                self.ros_bridge.publish_path_index(int(self._setting('path_index', 0)))
            if 'velocity_override' in accepted:
                self.ros_bridge.publish_velocity_override(float(self._setting('velocity_override', 100.0)) / 100.0)
            if 'spray_distance_mm' in accepted or 'nozzle_offset_mm' in accepted:
                effective = float(self._setting('spray_distance_mm', 100.0)) + float(self._setting('nozzle_offset_mm', 0.0))
                self.ros_bridge.publish_spray_distance(effective / 1000.0)
            if 'default_velocity' in accepted or 'default_velocity_enabled' in accepted:
                desired = float(self._setting('default_velocity', 0.1)) if self._setting('default_velocity_enabled', False) else 0.0
                self.ros_bridge.publish_desired_arm_speed(desired)
        return self.config

    @staticmethod
    def _finite_number(name: str, value: Any, *, minimum: float | None = None) -> float:
        try:
            number = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f'{name} must be a number') from exc
        if not math.isfinite(number):
            raise ValueError(f'{name} must be finite')
        if minimum is not None and number < minimum:
            raise ValueError(f'{name} must be at least {minimum}')
        return number

    @staticmethod
    def _platform_name(value: str) -> str:
        platform = str(value).strip().lower()
        if platform not in PROFILES:
            raise ValueError(f'Unknown platform: {value}')
        return platform

    def update_platform_settings(self, platform: str, values: dict[str, Any]) -> dict[str, Any]:
        """Persist validated tuning settings for exactly one platform.

        This intentionally merges only the named platform block, so selecting
        Robotnik in one browser cannot overwrite Bunker's tuning values.
        """
        platform = self._platform_name(platform)
        if not isinstance(values, dict):
            raise ValueError('Platform settings must be a JSON object')
        configured = self.config.get('platform_control_settings', {})
        configured = configured if isinstance(configured, dict) else {}
        current = configured.get(platform, {})
        current = deepcopy(current) if isinstance(current, dict) else {}

        if 'pid_gains' in values:
            gains = values['pid_gains']
            if not isinstance(gains, dict):
                raise ValueError('pid_gains must be an object')
            target = current.setdefault('pid_gains', {})
            if not isinstance(target, dict):
                target = {}
                current['pid_gains'] = target
            for key, value in gains.items():
                if key not in DEFAULT_PID_GAINS:
                    raise ValueError(f'Unknown PID setting: {key}')
                target[key] = self._finite_number(key, value, minimum=0.0)

        if 'base_smoothing' in values:
            smoothing = values['base_smoothing']
            if not isinstance(smoothing, dict):
                raise ValueError('base_smoothing must be an object')
            target = current.setdefault('base_smoothing', {})
            if not isinstance(target, dict):
                target = {}
                current['base_smoothing'] = target
            allowed = set(DEFAULT_BASE_SMOOTHING)
            unknown = set(smoothing) - allowed
            if unknown:
                raise ValueError(f'Unknown smoothing setting: {sorted(unknown)[0]}')
            if 'enabled' in smoothing:
                if not isinstance(smoothing['enabled'], bool):
                    raise ValueError('base_smoothing.enabled must be a boolean')
                target['enabled'] = bool(smoothing['enabled'])
            if 'method' in smoothing:
                method = str(smoothing['method']).strip().lower()
                if method not in {'moving_average', 'accel_limit'}:
                    raise ValueError('base_smoothing.method must be moving_average or accel_limit')
                target['method'] = method
            for key in ('max_accel_x', 'max_accel_y', 'max_accel_wz'):
                if key in smoothing:
                    target[key] = self._finite_number(key, smoothing[key], minimum=0.0)
            for key, maximum in (('moving_average_window_size', 100), ('external_path_index_stride', 1000)):
                if key in smoothing:
                    try:
                        number = self._finite_number(key, smoothing[key])
                    except (TypeError, ValueError) as exc:
                        raise ValueError(f'{key} must be an integer') from exc
                    if not number.is_integer():
                        raise ValueError(f'{key} must be an integer')
                    integer = int(number)
                    if integer < 1 or integer > maximum:
                        raise ValueError(f'{key} must be between 1 and {maximum}')
                    target[key] = integer

        if 'jparse_limits' in values:
            limits = values['jparse_limits']
            if not isinstance(limits, dict):
                raise ValueError('jparse_limits must be an object')
            target = current.setdefault('jparse_limits', {})
            if not isinstance(target, dict):
                target = {}
                current['jparse_limits'] = target
            unknown = set(limits) - set(DEFAULT_JPARSE_LIMITS)
            if unknown:
                raise ValueError(f'Unknown J-PARSE setting: {sorted(unknown)[0]}')
            for key, value in limits.items():
                target[key] = self._finite_number(key, value, minimum=0.000001)

        if 'path_transform' in values:
            transform = values['path_transform']
            if not isinstance(transform, dict):
                raise ValueError('path_transform must be an object')
            unknown = set(transform) - set(DEFAULT_PATH_TRANSFORM)
            if unknown:
                raise ValueError(f'Unknown path transform setting: {sorted(unknown)[0]}')
            target = current.setdefault('path_transforms_by_directory', {})
            if not isinstance(target, dict):
                target = {}
                current['path_transforms_by_directory'] = target
            trajectory = str(self._setting('trajectory_directory', ''))
            if not trajectory:
                raise ValueError('A trajectory directory is required for the path transform')
            directory = str(Path(trajectory).expanduser().resolve())
            existing = target.get(directory, {})
            existing = existing if isinstance(existing, dict) else {}
            target[directory] = {
                key: self._finite_number(key, transform.get(key, existing.get(key, default)))
                for key, default in DEFAULT_PATH_TRANSFORM.items()
            }

        configured[platform] = current
        self.config['platform_control_settings'] = configured
        self.store.save(self.config)
        return self.platform_settings_snapshot(platform)

    def snapshot(self) -> dict[str, Any]:
        processes = {}
        for name, managed in self.processes._processes.items():
            processes[name] = {'running': managed.is_running(), 'return_code': managed.poll()}
        config = dict(self.config)
        defaults = {
            'platform': 'robotnik',
            'simulation': False,
            'trajectory_directory': str(REPO_ROOT / 'components' / 'robotnik_paired_demo'),
            'control_frame': 'map',
            'base_pose_topic': '/vicon/Base_RB/Base_RB',
            'arm_pose_topic': VICON_NOZZLE_TOPIC,
            'vicon_input_topic': DEFAULT_VICON_INPUT_TOPIC,
            'vicon_nozzle_transform': deepcopy(DEFAULT_VICON_NOZZLE_TRANSFORM),
            'vicon_nozzle_transform_input_mode': 'quaternion',
            'external_map_frame': 'map',
            'robot_base_frame': 'base_link',
            'robot_tree_root_frame': 'odom',
            'path_index': 0,
            'original_arm_index': 0,
            'default_velocity_enabled': False,
            'default_velocity': 0.1,
            'spray_distance_mm': 100.0,
            'simulation_gui': False,
            'battery_topics_by_platform': deepcopy(DEFAULT_BATTERY_TOPICS),
            'base_hardware_by_platform': deepcopy(DEFAULT_BASE_HARDWARE),
        }
        for key, value in defaults.items():
            config.setdefault(key, value)
        if self._live_path_index is not None:
            config['path_index'] = self._live_path_index
        if self._live_original_arm_index is not None:
            config['original_arm_index'] = self._live_original_arm_index
        config.setdefault('follower_type', self._control_setting('follower_type', 'pid'))
        config.setdefault('diff_drive_mode', self._control_setting('diff_drive_mode', False))
        config.setdefault('direction_mode', 'goal_direction')
        config.setdefault('accuracy_phase', 'baseline')
        config.setdefault('velocity_override', 100)
        config.setdefault('nozzle_offset_mm', 0)
        config.setdefault('fixed_tool_offset_input_mode', 'quaternion')
        config['battery_topic'] = self._battery_topic()
        platform_settings = {
            platform: self.platform_settings_snapshot(platform)
            for platform in PROFILES
        }
        return {'config': config, 'platform_settings': platform_settings, 'status': self._status, 'processes': processes,
                'actions': self._action_states(),
                'move_start_distances_cm': self.move_start_distances_cm(),
                'logs': list(self.logs), 'ros_error': self.ros_error,
                'battery': {'level': self._battery_level, 'topic': self._battery_topic()},
                'base_hardware': self._base_hardware,
                'ur_dashboard': self.ur_dashboard.snapshot(),
                'forward_velocity_controller': self.forward_velocity_controller.snapshot(),
                'hardware_topic_results': self._hardware_topic_results}

    def move_start_distances_cm(self) -> dict:
        if self.ros_bridge is None:
            return {'base': None, 'arm': None}
        # The GUI displays the latest /path_index immediately.  Use that same
        # live selection here so its distance label and selected index cannot
        # diverge while the persisted start index remains unchanged.
        index = self._live_path_index if self._live_path_index is not None else int(
            self._setting('path_index', 0))
        return self.ros_bridge.move_start_distances_cm(index)

    def _process_state(
        self,
        names: tuple[str, ...],
        start_label: str,
        stop_label: str,
        description: str,
        one_shot: bool = False,
    ) -> dict[str, str]:
        managed = [self.processes.get(name) for name in names]
        present = [process for process in managed if process is not None]
        running = any(process.is_running() for process in present)
        if running:
            return {'label': stop_label, 'state': 'progress' if one_shot else 'running',
                    'detail': f'{description}\n\nStatus: aktiv.'}
        if not present:
            return {'label': start_label, 'state': 'idle',
                    'detail': f'{description}\n\nStatus: noch nicht gestartet.'}
        return_codes = [process.poll() for process in present]
        if any(code not in (None, 0) for code in return_codes):
            return {'label': start_label, 'state': 'error',
                    'detail': f'{description}\n\nStatus: letzter Prozess endete mit Fehler; Details stehen in der Konsole.'}
        if one_shot:
            return {'label': start_label, 'state': 'success',
                    'detail': f'{description}\n\nStatus: letzte Ausführung erfolgreich beendet.'}
        return {'label': start_label, 'state': 'success',
                'detail': f'{description}\n\nStatus: Prozess wurde erfolgreich beendet.'}

    def _action_states(self) -> dict[str, dict[str, str]]:
        states = {
            action: self._process_state((process,), start, stop, ACTION_DESCRIPTIONS[action])
            for action, (process, start, stop) in TOGGLE_ACTIONS.items()
        }
        states.update({
            action: self._process_state(
                (process,), start, stop, ACTION_DESCRIPTIONS[action], one_shot=True
            )
            for action, (process, start, stop) in ONE_SHOT_ACTIONS.items()
        })
        launch_processes = tuple(self.processes.get(name) for name in self._launch_all_process_names())
        launch_running = self._launch_all_active or any(process is not None and process.is_running() for process in launch_processes)
        states['launch_all'] = {
            'label': 'Stop All' if launch_running else 'Launch All',
            'state': 'running' if launch_running else 'idle',
            'detail': (
                'Simulation: startet Simulator, Transformation, Pfad-Publisher, Index, '
                'Follower und Velocity-Stack. Hardware: startet die externe Posekette statt '
                'des Simulators.\n\nStatus: aktiv.'
                if launch_running else
                'Simulation: startet Simulator, Transformation, Pfad-Publisher, Index, '
                'Follower und Velocity-Stack. Hardware: startet die externe Posekette statt '
                'des Simulators.\n\nStatus: noch nicht gestartet.'
            ),
        }
        states['stop_all'] = {
            'label': 'Stop All', 'state': 'danger',
            'detail': 'Stoppt alle von der GUI verwalteten Prozesse und publiziert Stop-Kommandos an Base und Arm.',
        }
        transformations = self._process_state(
            POSE_ADAPTER_PROCESSES if not bool(self._setting('simulation', False)) else ('transformations',),
            'Launch Transformations',
            'Stop Transformations',
            ACTION_DESCRIPTIONS['transformations'],
        )
        if bool(self._setting('simulation', False)) and self._is_running('simulation'):
            transformations = {
                'label': 'TCP Pose from Sim', 'state': 'running',
                'detail': f"{ACTION_DESCRIPTIONS['transformations']}\n\nStatus: Die Simulation publiziert die TCP-Pose.",
            }
        states['transformations'] = transformations
        states['pose_adapters'] = self._process_state(
            POSE_ADAPTER_PROCESSES, 'Pose Adapters', 'Stop Pose Adapters', ACTION_DESCRIPTIONS['pose_adapters']
        )
        states['rviz'] = self._process_state(
            ('rviz',), 'Open RViz', 'Open RViz', ACTION_DESCRIPTIONS['rviz']
        )
        states['capture_tool_offset'] = self._message_state(
            'capture_tool_offset', 'Capture UR TCP Offset', ACTION_DESCRIPTIONS['capture_tool_offset']
        )
        states['calculate_path_transform'] = self._message_state(
            'calculate_path_transform', 'Calculate Path Transform', ACTION_DESCRIPTIONS['calculate_path_transform']
        )
        states['check_hardware_topics'] = self._message_state(
            'check_hardware_topics', 'Check Hardware Topics', ACTION_DESCRIPTIONS['check_hardware_topics']
        )
        restart_process = self.processes.get('restart_arm_controllers')
        if restart_process is None and self._last_action_success.get('restart_arm_controllers') is False:
            states['restart_arm_controllers'] = self._message_state(
                'restart_arm_controllers', 'Restart Controllers', ACTION_DESCRIPTIONS['restart_arm_controllers']
            )
        else:
            states['restart_arm_controllers'] = self._process_state(
                ('restart_arm_controllers',), 'Restart Controllers', 'Restarting Controllers',
                ACTION_DESCRIPTIONS['restart_arm_controllers'], one_shot=True,
            )
        states['remote_bringup'] = self._message_state(
            'remote_bringup', 'Start remote Bringup',
            'Führt ~/bringup.sh auf robot@192.168.0.200 aus. Jeder Klick startet eine neue Ausführung.'
        )
        states['enable_ur'] = self._message_state(
            'enable_ur', 'Enable UR',
            'Quittiert bei Protective Stop und führt anschließend Power on sowie Brake release aus.'
        )
        states['release_brakes'] = self._message_state(
            'release_brakes', 'Release brakes', 'Löst die Bremsen des eingeschalteten UR.'
        )
        states['unlock_protective_stop'] = self._message_state(
            'unlock_protective_stop', 'Unlock Protective Stop',
            'Quittiert einen Protective Stop nach Prüfung der Ursache.'
        )
        states['play_program'] = self._message_state(
            'play_program', 'Run Program', 'Startet das aktuell im UR geladene Programm.'
        )
        ready = all(self._status.values())
        controls = all(self._is_running(name) for name in ('path_index', 'base_follower', 'arm_follower'))
        states['start_following'] = {
            'label': 'Following active' if self._following_active else 'Start Following',
            'state': 'running' if self._following_active else ('ready' if ready and controls else 'warning'),
            'detail': (
                'Publiziert /path_index_command und mehrfach /start_condition=true; dadurch '
                'starten Fortschritt sowie Base- und Armfolger.\n\nStatus: Following aktiv.'
                if self._following_active else (
                    'Publiziert /path_index_command und mehrfach /start_condition=true; dadurch '
                    'starten Fortschritt sowie Base- und Armfolger.\n\nStatus: bereit.'
                    if ready and controls else
                    'Publiziert /path_index_command und mehrfach /start_condition=true; dadurch '
                    'starten Fortschritt sowie Base- und Armfolger.\n\nStatus: wartet auf ROS-Status oder Steuerprozesse.'
                )
            ),
        }
        states['stop_following'] = {
            'label': 'Stop Following',
            'state': 'danger' if self._following_active else 'idle',
            'detail': (
                'Publiziert mehrfach /start_condition=false und wiederholt Null-Kommandos an '
                'Base und Arm.\n\nStatus: Following wird gestoppt.'
                if self._following_active else
                'Publiziert mehrfach /start_condition=false und wiederholt Null-Kommandos an '
                'Base und Arm.\n\nStatus: Following ist nicht aktiv.'
            ),
        }
        return states

    def _is_running(self, name: str) -> bool:
        process = self.processes.get(name)
        return process is not None and process.is_running()

    def _message_state(self, action: str, label: str, description: str) -> dict[str, str]:
        message = self._last_action_messages.get(action)
        return {
            'label': label,
            'state': ('success' if self._last_action_success.get(action, True) else 'error') if message else 'idle',
            'detail': f'{description}\n\nStatus: {message}' if message else f'{description}\n\nStatus: noch nicht ausgeführt.',
        }

    def _setting(self, name: str, default: Any) -> Any:
        return self.config.get(name, default)

    def _use_sim_time(self) -> str:
        return str(bool(self._setting('simulation', False))).lower()

    def _profile(self) -> dict[str, Any]:
        return PROFILES.get(str(self._setting('platform', 'robotnik')).lower(), PROFILES['robotnik'])

    def _platform_key(self) -> str:
        """Return the canonical key for platform-scoped operator settings."""
        return str(self._setting('platform', 'robotnik')).strip().lower() or 'robotnik'

    @staticmethod
    def _validated_battery_topic(value: Any) -> str:
        topic = str(value).strip()
        if not topic:
            return ''
        if not topic.startswith('/') or any(character.isspace() for character in topic):
            raise ValueError('Battery topic must be an absolute ROS topic without whitespace')
        return topic

    def _validated_battery_topics(self, value: Any) -> dict[str, str]:
        if not isinstance(value, dict):
            raise ValueError('battery_topics_by_platform must be an object')
        unknown = set(value) - set(PROFILES)
        if unknown:
            raise ValueError(f'Unknown platform: {sorted(unknown)[0]}')
        topics = deepcopy(DEFAULT_BATTERY_TOPICS)
        topics.update({platform: self._validated_battery_topic(topic) for platform, topic in value.items()})
        return topics

    def _battery_topics(self) -> dict[str, str]:
        configured = self.config.get('battery_topics_by_platform', {})
        try:
            return self._validated_battery_topics(configured)
        except ValueError:
            return deepcopy(DEFAULT_BATTERY_TOPICS)

    def _battery_topic(self, platform: str | None = None) -> str:
        key = self._platform_key() if platform is None else self._platform_name(platform)
        return self._battery_topics()[key]

    def _base_hardware_config(self) -> dict[str, list[int]]:
        configured = self.config.get('base_hardware_by_platform', {})
        candidate = configured.get(self._platform_key(), {}) if isinstance(configured, dict) else {}
        values = candidate.get('expected_motor_ids', []) if isinstance(candidate, dict) else []
        try:
            ids = sorted(set(int(item) for item in values if int(item) > 0))
        except (TypeError, ValueError):
            ids = []
        if not ids:
            ids = list(DEFAULT_BASE_HARDWARE[self._platform_key()]['expected_motor_ids'])
        return {'expected_motor_ids': ids}

    def _platform_settings(self, platform: str | None = None) -> dict[str, Any]:
        key = self._platform_key() if platform is None else str(platform).strip().lower()
        configured = self._setting('platform_control_settings', {})
        if not isinstance(configured, dict):
            return {}
        settings = configured.get(key, {})
        return settings if isinstance(settings, dict) else {}

    def _control_setting(self, name: str, default: Any) -> Any:
        # The web form stores an explicit current value; the older Qt GUI stores
        # values per platform. Honour both representations during migration.
        if name in self.config:
            return self.config[name]
        settings = self._platform_settings()
        if settings:
            return settings.get(name, self._setting(name, default))
        return self._setting(name, default)

    def _pid(self, name: str, default: float, platform: str | None = None) -> float:
        platform_settings = self._platform_settings(platform)
        gains = platform_settings.get('pid_gains', {})
        if not isinstance(gains, dict) or name not in gains:
            gains = self._setting('pid_gains', {})
        try:
            return float(gains.get(name, default)) if isinstance(gains, dict) else default
        except (TypeError, ValueError):
            return default

    def _smoothing(self, name: str, default: Any, platform: str | None = None) -> Any:
        platform_settings = self._platform_settings(platform)
        settings = platform_settings.get('base_smoothing', {})
        if not isinstance(settings, dict) or name not in settings:
            settings = self._setting('base_smoothing', {})
        return settings.get(name, default) if isinstance(settings, dict) else default

    def _jparse_limit(self, name: str, default: float, platform: str | None = None) -> float:
        platform_settings = self._platform_settings(platform)
        limits = platform_settings.get('jparse_limits', {})
        try:
            return float(limits.get(name, default)) if isinstance(limits, dict) else default
        except (TypeError, ValueError):
            return default

    def _path_transform(self, trajectory: str, platform: str | None = None) -> dict[str, float]:
        directory = str(Path(trajectory).expanduser().resolve())
        platform_settings = self._platform_settings(platform)
        by_directory = platform_settings.get('path_transforms_by_directory', {})
        transform = by_directory.get(directory, {}) if isinstance(by_directory, dict) else {}
        if not isinstance(transform, dict) or not transform:
            by_directory = self._setting('path_transforms_by_directory', {})
            transform = by_directory.get(directory, {}) if isinstance(by_directory, dict) else {}
        if not isinstance(transform, dict) or not transform:
            transform = self._setting('path_transform', {})
        transform = transform if isinstance(transform, dict) else {}
        result = {}
        for key, default in DEFAULT_PATH_TRANSFORM.items():
            try:
                result[key] = float(transform.get(key, default))
            except (TypeError, ValueError):
                result[key] = default
        return result

    def platform_settings_snapshot(self, platform: str) -> dict[str, Any]:
        """Return all effective tuning values used by the selected platform."""
        platform = self._platform_name(platform)
        trajectory = str(self._setting('trajectory_directory', ''))
        return {
            'pid_gains': {
                key: self._pid(key, default, platform)
                for key, default in DEFAULT_PID_GAINS.items()
            },
            'base_smoothing': {
                key: self._smoothing(key, default, platform)
                for key, default in DEFAULT_BASE_SMOOTHING.items()
            },
            'jparse_limits': {
                key: self._jparse_limit(key, default, platform)
                for key, default in DEFAULT_JPARSE_LIMITS.items()
            },
            'path_transform': self._path_transform(trajectory, platform),
        }

    def _default_velocity_parameter(self) -> float:
        if not bool(self._setting('default_velocity_enabled', False)):
            return -1.0
        try:
            return float(self._setting('default_velocity', 0.1))
        except (TypeError, ValueError):
            return 0.1

    def _fixed_tool_offset(self) -> dict:
        platform_offsets = self._setting('fixed_tool_offsets_by_platform', {})
        offset = {}
        if isinstance(platform_offsets, dict):
            candidate = platform_offsets.get(self._platform_key(), {})
            if isinstance(candidate, dict):
                offset = candidate
        if not offset:
            offset = self._setting('fixed_tool_offset', {})
        offset = offset if isinstance(offset, dict) else {}
        return {
            'xyz': offset.get('xyz', [-0.25, 0.0, 0.015]),
            'quaternion_xyzw': offset.get('quaternion_xyzw', [0.0, -0.7071067812, 0.0, 0.7071067812]),
        }

    def _fixed_tool_arguments(self) -> list[str]:
        offset = self._fixed_tool_offset()
        try:
            xyz = ', '.join(f'{float(value):.6f}' for value in offset.get('xyz', [-0.25, 0.0, 0.015]))
            quat = ', '.join(f'{float(value):.6f}' for value in offset.get('quaternion_xyzw', [0.0, -0.7071067812, 0.0, 0.7071067812]))
        except (TypeError, ValueError):
            return []
        return [f'fixed_tool_offset_xyz:=[{xyz}]', f'fixed_tool_offset_quaternion_xyzw:=[{quat}]']

    def _path_transform_arguments(self, trajectory: str) -> list[str]:
        transform = self._path_transform(trajectory)
        x = transform['x']
        y = transform['y']
        z = transform['z']
        yaw = transform['yaw_deg']
        return [f'path_transform_xyz:=[{x:.6f}, {y:.6f}, {z:.6f}]', f'path_transform_yaw_deg:={yaw:.6f}']

    def _toggle(self, name: str, command: list[str]) -> None:
        running = self.processes.get(name)
        if running and running.is_running():
            self.processes.stop(name)
            self.log(name, 'stopped by operator')
        else:
            self.log(name, ' '.join(command))
            self.processes.start(name, command)

    def _start(self, name: str) -> None:
        command = self.command_for(name)
        if command is None:
            raise ValueError(f'unknown action: {name}')
        self.log(name, ' '.join(command))
        self.processes.start(name, command, replace=False)

    def _schedule(self, seconds: float, callback: Callable[[], None]) -> None:
        timer = Timer(seconds, callback)
        timer.daemon = True
        self._timers.append(timer)
        timer.start()

    @staticmethod
    def _launch_all_process_names() -> tuple[str, ...]:
        return (
            'simulation', 'publish_path', 'move_arm', 'path_index', 'transformations',
            *POSE_ADAPTER_PROCESSES, 'controllers', 'base_follower', 'arm_follower',
            'move_base', 'switch_arm_velocity', 'base_accuracy', 'tcp_accuracy',
            'accuracy_report', 'rviz', 'sync_workspace',
        )

    def action(self, name: str) -> None:
        if name == 'remote_bringup':
            self._remote_bringup_run += 1
            run_name = f'remote_bringup_{self._remote_bringup_run}'
            command = ['ssh', '-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=yes',
                       '-o', 'ConnectTimeout=8', 'robot@192.168.0.200', 'cd "$HOME" && ./bringup.sh']
            self.log(run_name, 'starting ~/bringup.sh on robot@192.168.0.200')
            self.processes.start(run_name, command, replace=False)
            self._last_action_messages[name] = f'Bringup-Ausführung {self._remote_bringup_run} gestartet'
            self._last_action_success[name] = True
            return
        if name in {'play_program', 'unlock_protective_stop', 'enable_ur', 'release_brakes'}:
            if name == 'enable_ur':
                command = enable_command(
                    self.ur_dashboard.snapshot().get('safety_mode') == 'PROTECTIVE_STOP'
                )
                self._start_dashboard_command(name, command)
            else:
                self._start_dashboard_command(name)
            return
        if name == 'stop_all':
            self.stop_all()
            return
        if name == 'launch_all':
            if self._launch_all_active:
                self.stop_all()
                return
            self._launch_all_active = True
            if bool(self._setting('simulation', False)):
                self._start('simulation')
            else:
                self.start_pose_adapters()
            for item in ('publish_path', 'controllers', 'path_index', 'base_follower', 'arm_follower'):
                self._start(item)
            if bool(self._setting('simulation', False)):
                self._start('move_arm')
                self._schedule(13.0, lambda: self._start('move_base'))
            return
        if name == 'start_following':
            if self.ensure_ros():
                # The progress topic is live state.  Do not resume from the
                # last value saved in the configuration merely because the
                # operator paused following in between.
                index = self._live_path_index if self._live_path_index is not None else int(self._setting('path_index', 0))
                self.ros_bridge.publish_path_index(index)
                # ``/velocity_override`` is deliberately non-latched.  The
                # follower processes can therefore start after the GUI has
                # last published it and would otherwise retain their 1.0
                # default.  Publish the persisted value after all components
                # are up and immediately before opening the start gate.
                self.ros_bridge.publish_velocity_override(
                    float(self._setting('velocity_override', 100.0)) / 100.0
                )
                self._publish_start_condition_repeatedly(True)
                self._following_active = True
            return
        if name == 'capture_tool_offset':
            if not self.ensure_ros():
                return
            transform = self.ros_bridge.lookup_tool_offset('robot_arm_tool0', 'robot_arm_tool0_controller')
            if transform is None:
                self.log('calibration', 'TF robot_arm_tool0 -> robot_arm_tool0_controller is unavailable')
                return
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            offset = {
                'xyz': [translation.x, translation.y, translation.z],
                'quaternion_xyzw': [rotation.x, rotation.y, rotation.z, rotation.w],
            }
            platform_offsets = self.config.setdefault('fixed_tool_offsets_by_platform', {})
            if not isinstance(platform_offsets, dict):
                platform_offsets = {}
                self.config['fixed_tool_offsets_by_platform'] = platform_offsets
            platform_offsets[self._platform_key()] = offset
            self.store.save(self.config)
            self.log('calibration', f'captured UR TCP offset for {self._platform_key()}')
            self._last_action_messages['capture_tool_offset'] = 'UR TCP offset captured'
            return
        if name == 'calculate_path_transform':
            self.calculate_path_transform()
            return
        if name == 'check_hardware_topics':
            self.check_hardware_topics()
            return
        if name == 'restart_arm_controllers':
            self._restart_arm_controllers()
            return
        if name == 'pose_adapters':
            self._toggle_pose_adapters()
            return
        if name == 'transformations' and not bool(self._setting('simulation', False)):
            self._toggle_pose_adapters()
            return
        if name == 'stop_following':
            self._following_active = False
            if self.ensure_ros():
                self._publish_start_condition_repeatedly(False)
                for delay in range(0, 1000, 100):
                    self._schedule(delay / 1000.0, lambda: self.ros_bridge.publish_stop_commands(str(self._setting('control_frame', 'map'))))
            return
        command = self.command_for(name)
        if command is None:
            raise ValueError(f'unknown action: {name}')
        self._toggle(name, command)

    def _start_dashboard_command(self, name: str, command: list[str] | None = None) -> None:
        command = dashboard_command(name) if command is None else command
        self.log(name, ' '.join(command))
        self.processes.start(name, command)
        self.ur_status_monitor.request_refresh(dashboard=True, controller=False)
        self._last_action_messages[name] = 'Dashboard-Kommando gesendet; Status wird aktualisiert'
        self._last_action_success[name] = True

    @staticmethod
    def _restart_arm_controllers_command() -> list[str]:
        """Return the remote, fail-closed UR controller restart procedure.

        This is deliberately executed where the controller manager runs.  It
        therefore uses the robot's ROS/DDS settings, rather than assuming that
        the workstation running the web GUI can make ROS service calls to it.
        """
        script = r'''set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /home/robot/ros_config.sh >/dev/null
set -u
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

manager=/robot/arm/controller_manager
required=(
  joint_state_broadcaster
  io_and_status_controller
  speed_scaling_state_broadcaster
  force_torque_sensor_broadcaster
  tcp_pose_broadcaster
  ur_configuration_controller
  forward_velocity_controller
)

list_controllers() {
  local deadline result
  deadline=$((SECONDS + 90))
  while true; do
    if result="$(timeout 20s ros2 control list_controllers --controller-manager "$manager")"; then
      printf '%s\n' "$result"
      return 0
    fi
    if (( SECONDS >= deadline )); then
      echo "ERROR: $manager/list_controllers did not answer within 90 seconds." >&2
      return 1
    fi
    echo "Controller-manager response delayed; retrying ..." >&2
    sleep 2
  done
}

echo "Waiting for $manager/list_controllers ..."
deadline=$((SECONDS + 90))
while ! controllers="$(list_controllers)"; do
  if (( SECONDS >= deadline )); then
    echo "ERROR: $manager/list_controllers did not answer within 90 seconds." >&2
    exit 1
  fi
  sleep 2
done

mapfile -t active < <(printf '%s\n' "$controllers" | awk '$3 == "active" { print $1 }')
if (( ${#active[@]} )); then
  echo "Deactivating all active controllers: ${active[*]}"
  if ! timeout 60s ros2 control switch_controllers --controller-manager "$manager" \
    --deactivate "${active[@]}" --strict --switch-timeout 50; then
    echo "Switch response delayed; checking resulting controller state ..." >&2
  fi
fi

controllers="$(list_controllers)"
if printf '%s\n' "$controllers" | awk '$3 == "active" { found = 1 } END { exit !found }'; then
  echo "ERROR: At least one controller is still active after the strict stop." >&2
  printf '%s\n' "$controllers" >&2
  exit 1
fi

for controller in "${required[@]}"; do
  if ! printf '%s\n' "$controllers" | awk -v name="$controller" '$1 == name { found = 1 } END { exit found ? 0 : 1 }'; then
    echo "Loading $controller"
    if ! timeout 60s ros2 control load_controller --controller-manager "$manager" "$controller"; then
      echo "Load response delayed; checking whether $controller was loaded ..." >&2
    fi
    controllers="$(list_controllers)"
    if ! printf '%s\n' "$controllers" | awk -v name="$controller" '$1 == name { found = 1 } END { exit found ? 0 : 1 }'; then
      echo "ERROR: $controller was not loaded." >&2
      exit 1
    fi
  fi
  state="$(printf '%s\n' "$controllers" | awk -v name="$controller" '$1 == name { print $3 }')"
  if [[ "$state" == "unconfigured" ]]; then
    echo "Configuring $controller"
    if ! timeout 60s ros2 control set_controller_state --controller-manager "$manager" "$controller" inactive; then
      echo "Configure response delayed; checking resulting controller state ..." >&2
    fi
    controllers="$(list_controllers)"
    state="$(printf '%s\n' "$controllers" | awk -v name="$controller" '$1 == name { print $3 }')"
    if [[ "$state" != "inactive" ]]; then
      echo "ERROR: $controller was not configured to inactive (state: '$state')." >&2
      exit 1
    fi
  elif [[ "$state" != "inactive" ]]; then
    echo "ERROR: $controller has unexpected state '$state' after the strict stop." >&2
    exit 1
  fi
done

echo "Activating required status controllers and forward_velocity_controller"
if ! timeout 60s ros2 control switch_controllers --controller-manager "$manager" \
  --activate "${required[@]}" --strict --switch-timeout 50; then
  echo "Activation response delayed; checking resulting controller state ..." >&2
fi

controllers="$(list_controllers)"
printf '%s\n' "$controllers"
if ! printf '%s\n' "$controllers" | awk '$1 == "forward_velocity_controller" && $3 == "active" { found = 1 } END { exit found ? 0 : 1 }'; then
  echo "ERROR: forward_velocity_controller is not active." >&2
  exit 1
fi
if printf '%s\n' "$controllers" | awk '$3 == "active" && $1 ~ /^(joint_trajectory_controller|scaled_joint_trajectory_controller|forward_position_controller|force_mode_controller|passthrough_trajectory_controller|freedrive_mode_controller|tool_contact_controller)$/ { found = 1 } END { exit found ? 0 : 1 }'; then
  echo "ERROR: a conflicting arm motion controller is still active." >&2
  exit 1
fi
echo "Controller restart complete: forward_velocity_controller is active."
'''
        return [
            'ssh', '-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=yes',
            '-o', 'ConnectTimeout=8', 'robot@192.168.0.200',
            f'bash -lc {shlex.quote(script)}',
        ]

    def _restart_arm_controllers(self) -> None:
        """Run the bounded, remote controller recovery without moving the arm."""
        name = 'restart_arm_controllers'
        if bool(self._setting('simulation', False)):
            message = 'Controller restart skipped: disable Simulation before changing the physical UR controllers'
            self.log(name, message)
            self._last_action_messages[name] = message
            self._last_action_success[name] = False
            return
        command = self._restart_arm_controllers_command()
        self.log(name, 'restarting remote arm controllers; no motion command is sent')
        self.processes.start(name, command)
        self.ur_status_monitor.request_refresh(dashboard=False, controller=True)
        self._last_action_messages[name] = (
            'Controller restart started: all active controllers will be stopped before '
            'forward_velocity_controller is activated'
        )
        self._last_action_success[name] = True

    def _toggle_pose_adapters(self) -> None:
        if any(self._is_running(name) for name in POSE_ADAPTER_PROCESSES):
            for name in POSE_ADAPTER_PROCESSES:
                self.processes.stop(name)
            self.log('transformations', 'stopped by operator')
            return
        self.start_pose_adapters()

    def check_hardware_topics(self) -> list[str]:
        """Check the selected hardware input and command graph endpoints."""
        if bool(self._setting('simulation', False)):
            message = 'Hardware topic check skipped: disable Simulation first'
            self.log('hardware_check', message)
            self._last_action_messages['check_hardware_topics'] = message
            self._last_action_success['check_hardware_topics'] = False
            self._hardware_topic_results = [message]
            return [message]
        if not self.ensure_ros():
            message = 'Hardware topic check failed: ROS bridge unavailable'
            self._last_action_messages['check_hardware_topics'] = message
            self._last_action_success['check_hardware_topics'] = False
            self._hardware_topic_results = [message]
            return [message]
        profile = self._profile()
        base_topic = str(self._setting('base_pose_topic', '/vicon/Base_RB/Base_RB'))
        requirements = [
            # This is the bridge input.  /vicon/tool_transformed is generated
            # locally by vicon_ee_static_tf during Launch All, so checking it
            # before launch would always create a misleading failure.
            (str(self._setting('vicon_input_topic', DEFAULT_VICON_INPUT_TOPIC)),
             'geometry_msgs/msg/PoseStamped', 'publisher'),
            ('/robot/robot_description', 'std_msgs/msg/String', 'publisher'),
            ('/robot/joint_states', 'sensor_msgs/msg/JointState', 'publisher'),
            (str(profile['cmd_vel']),
             'geometry_msgs/msg/TwistStamped' if bool(profile['stamped']) else 'geometry_msgs/msg/Twist',
             'subscriber'),
            ('/robot/arm/forward_velocity_controller/commands',
             'std_msgs/msg/Float64MultiArray', 'subscriber'),
        ]
        if bool(self._setting('use_odometry_robot_pose', False)):
            requirements.insert(0, (str(profile['odom']), 'nav_msgs/msg/Odometry', 'publisher'))
        if not bool(self._setting('use_vicon_tcp_base_pose_fallback', False)):
            requirements.insert(0, (base_topic, 'geometry_msgs/msg/PoseStamped', 'publisher'))
        messages = self.ros_bridge.check_topic_contract(requirements)
        for message in messages:
            self.log('hardware_check', message)
        failures = sum(message.startswith('FAIL') for message in messages)
        summary = f'Hardware topic check: {len(messages) - failures}/{len(messages)} OK'
        self.log('hardware_check', summary)
        self._last_action_messages['check_hardware_topics'] = summary
        self._last_action_success['check_hardware_topics'] = failures == 0
        self._hardware_topic_results = [*messages, summary]
        return self._hardware_topic_results

    def command_for(self, name: str) -> list[str] | None:
        simulation = bool(self._setting('simulation', False))
        profile = self._profile()
        frame = str(self._setting('control_frame', 'map'))
        trajectory = str(self._setting('trajectory_directory', REPO_ROOT / 'components' / 'robotnik_paired_demo'))
        index = int(self._setting('path_index', 0))
        if name == 'vicon':
            command = ['ros2', 'launch', 'vicon_receiver', 'client.launch.py',
                       'hostname:=192.168.0.30:8802', 'topic_namespace:=vicon',
                       f'world_frame:={frame}', 'vicon_frame:=vicon']
            setup = Path.home() / 'vicon_receiver_ws' / 'install' / 'setup.bash'
            if setup.is_file():
                # Source only in the managed child session. Positional arguments
                # preserve paths/frame values without shell interpolation.
                return ['bash', '-c', 'source "$1" && shift && exec "$@"',
                        'vicon', str(setup), *command]
            return command
        if name == 'simulation':
            show_window = str(bool(self._setting('simulation_gui', False))).lower()
            if self._platform_key() == 'bunker':
                return ['ros2', 'launch', 'bunker_description', 'spawn_with_controllers.launch.py',
                        f'headless:={str(not bool(self._setting("simulation_gui", False))).lower()}',
                        'launch_rviz:=false']
            return ['ros2', 'launch', 'robotnik_rbvogui_tum', 'rbvogui_ur_standard_control.launch.py',
                    f'gui:={show_window}', 'robot_id:=robot', 'arm_type:=ur20']
        if name == 'publish_path':
            return ['ros2', 'launch', 'parse_paths', 'robotnik_base_arm_paths.launch.py',
                    f'use_sim_time:={self._use_sim_time()}', f'frame_id:={frame}',
                    'load_exported_trajectories:=true', f'trajectory_directory:={trajectory}', 'publish_once:=false',
                    *self._path_transform_arguments(trajectory)]
        if name == 'index_pose_preview':
            return [sys.executable, '-m', 'am_operator_gui.index_pose_preview', '--ros-args',
                    '-p', f'use_sim_time:={self._use_sim_time()}',
                    '-p', f'initial_path_index:={index}',
                    '-p', f"base_path_topic:={profile['path']}"]
        if name == 'path_index':
            return ['ros2', 'run', 'ur_trajectory_follower', 'increment_path_index', '--ros-args',
                    '-p', f'use_sim_time:={self._use_sim_time()}', '-p', 'path_index_topic:=/path_index',
                    '-p', 'path_index_command_topic:=/path_index_command', '-p', 'next_goal_topic:=/next_goal',
                    '-p', 'normal_topic:=/normal_vector', '-p', f'initial_path_index:={index}',
                    '-p', 'path_topic:=/ur_path_transformed', '-p', f"base_path_topic:={profile['path']}",
                    '-p', f"progress_mode:={'desired_speed' if bool(self._setting('default_velocity_enabled', False)) else 'timestamp'}",
                    '-p', 'arm_reference_topic:=/arm_trajectory_reference', '-p', 'base_reference_topic:=/base_trajectory_reference',
                    '-p', 'processed_path_topic:=/ur_path_tracking', '-p', 'processed_base_path_topic:=/base_path_tracking',
                    '-p', 'desired_speed_topic:=/desired_arm_speed', '-p', f'desired_arm_speed:={self._default_velocity_parameter():.6f}',
                    '-p', 'enable_path_resampling:=true', '-p', 'resample_spacing:=0.005',
                    '-p', 'velocity_override_topic:=/velocity_override', '-p', 'start_condition_topic:=/start_condition',
                    '-p', 'wait_for_start_condition:=true']
        if name == 'base_follower':
            follower = str(self._control_setting('follower_type', 'pid'))
            diff_drive = bool(self._control_setting('diff_drive_mode', False)) or str(self._setting('platform', 'robotnik')) == 'bunker'
            return ['ros2', 'run', 'base_trajectory_follower', 'simple_base_follower', '--ros-args',
                    '-p', f'use_sim_time:={self._use_sim_time()}', '-p', 'path_topic:=/base_path_tracking',
                    '-p', f"robot_pose_topic:={profile['robot_pose']}", '-p', 'robot_pose_type:=pose_stamped',
                    '-p', f"cmd_vel_topic:={profile['cmd_vel']}", '-p', f"output_stamped:={str(profile['stamped']).lower()}",
                    '-p', f"command_frame_id:={profile['frame']}", '-p', f'follower_type:={follower}',
                    '-p', f'diff_drive_mode:={str(diff_drive).lower()}', '-p', 'use_external_path_index:=true',
                    '-p', 'path_index_topic:=/path_index', '-p', 'reference_pose_topic:=/base_trajectory_reference',
                    '-p', f"external_path_index_stride:={int(self._smoothing('external_path_index_stride', 10))}",
                    '-p', 'wait_for_start_condition:=true', '-p', 'start_condition_topic:=/start_condition',
                    '-p', 'velocity_override_topic:=/velocity_override', '-p', 'lookahead_distance:=0.3',
                    '-p', f'kp_x:={self._pid("base_follower.kp_x", 0.8):.6f}', '-p', f'kp_y:={self._pid("base_follower.kp_y", 0.8):.6f}',
                    '-p', f'kp_yaw:={self._pid("base_follower.kp_yaw", 1.2):.6f}', '-p', f'max_vx:={self._pid("base_follower.max_vx", 0.25):.6f}',
                    '-p', f'max_vy:={self._pid("base_follower.max_vy", 0.25):.6f}', '-p', f'max_wz:={self._pid("base_follower.max_wz", 0.5):.6f}',
                    '-p', f"smooth_velocity_commands:={str(bool(self._smoothing('enabled', True))).lower()}",
                    '-p', f"velocity_smoothing_method:={self._smoothing('method', 'moving_average')}",
                    '-p', f"max_accel_x:={float(self._smoothing('max_accel_x', 0.25)):.6f}", '-p', f"max_accel_y:={float(self._smoothing('max_accel_y', 0.25)):.6f}",
                    '-p', f"max_accel_wz:={float(self._smoothing('max_accel_wz', 0.5)):.6f}", '-p', f"moving_average_window_size:={int(self._smoothing('moving_average_window_size', 5))}"]
        if name == 'arm_follower':
            nozzle_pose = '/current_tcp_pose' if simulation else '/current_nozzle_tip_pose'
            direction_gains = [
                f'{key}:={self._pid(f"arm_direction.{key}", default):.6f}'
                for key, default in (
                    ('kp_z', 0.7), ('along_track_kp', 2.0), ('orthogonal_kp', 1.0),
                    ('max_along_track_correction', 0.03), ('max_spray_axis_correction', 0.03),
                    ('max_tracking_linear_velocity', 0.12), ('final_position_tolerance', 0.005),
                )
            ]
            orientation_gains = [
                f'{key}:={self._pid(f"arm_orientation.{key}", default):.6f}'
                for key, default in (('kp_orientation', 1.0), ('ki_orientation', 0.0), ('kd_orientation', 0.0))
            ]
            return ['ros2', 'launch', 'ur_trajectory_follower', 'sideways_arm_control.launch.py',
                    f'use_sim_time:={self._use_sim_time()}', f'path_frame:={frame}',
                    'robot_name:=robot', 'arm:=arm', 'joint_prefix:=robot_arm_', 'base_link:=robot_arm_base_link', 'tip_link:=robot_arm_tool0',
                    'robot_description_topic:=/robot/robot_description', 'joint_states_topic:=/robot/joint_states',
                    f"velocity_command_topic:={'/robot/arm_forward_velocity_controller/commands' if simulation else '/robot/arm/forward_velocity_controller/commands'}",
                    'start_jparse_controller:=false', 'start_command_transform:=false', 'publish_current_pose_from_tf:=false',
                    'publish_path:=false', 'publish_path_index:=false', 'move_to_start_pose:=false',
                    f"start_pose_trajectory_topic:={'/robot/joint_trajectory_controller/joint_trajectory' if simulation else '/robot/arm/joint_trajectory_controller/joint_trajectory'}",
                    'start_pose_publish_delay:=8.0', f'nozzle_pose_topic:={nozzle_pose}', 'current_pose_topic:=/current_deposition_pose',
                    'spray_distance_topic:=/spray_distance', 'smoothed_spray_distance_topic:=/spray_distance_smoothed',
                    f"spray_distance_initial:={(float(self._setting('spray_distance_mm', 100.0)) + float(self._setting('nozzle_offset_mm', 0.0))) / 1000.0:.6f}",
                    'spray_distance_max_rate:=0.020000', 'path_topic:=/ur_path_transformed', 'original_path_topic:=/ur_path_original',
                    'normal_topic:=/normal_vector', 'path_index_topic:=/path_index', 'next_goal_topic:=/next_goal',
                    'wait_for_start_condition:=true', 'start_condition_topic:=/start_condition', f'initial_path_index:={index}',
                    f"progress_mode:={'desired_speed' if bool(self._setting('default_velocity_enabled', False)) else 'timestamp'}",
                    'arm_reference_topic:=/arm_trajectory_reference', 'desired_speed_topic:=/desired_arm_speed',
                    f'default_velocity:={self._default_velocity_parameter():.6f}', *self._fixed_tool_arguments(), *direction_gains, *orientation_gains]
        if name == 'controllers':
            controller_manager = '/robot/controller_manager' if simulation else '/robot/arm/controller_manager'
            velocity_topic = '/robot/arm_forward_velocity_controller/commands' if simulation else '/robot/arm/forward_velocity_controller/commands'
            active_controller = 'arm_forward_velocity_controller' if simulation else 'forward_velocity_controller'
            return ['ros2', 'launch', 'am_operator_gui', 'arm_velocity_controller_stack.launch.py',
                    f'use_sim_time:={self._use_sim_time()}', 'robot_name:=robot', 'arm:=arm',
                    'base_link:=robot_arm_base_link', 'tip_link:=robot_arm_tool0', f'path_frame:={frame}',
                    'robot_description_topic:=/robot/robot_description', 'joint_states_topic:=/robot/joint_states',
                    'source_twist_topic:=/jparse_velocity_controller_ur/twist_cmd_world',
                    'controller_twist_topic:=/jparse_velocity_controller_ur/twist_cmd',
                    f'velocity_command_topic:={velocity_topic}', f'controller_manager:={controller_manager}',
                    'deactivate_controller:=joint_trajectory_controller', f'activate_controller:={active_controller}',
                    'jparse_readiness_topic:=/am/jparse_ready', 'controller_readiness_topic:=/am/arm_controller_ready',
                    'command_joint_names_csv:=robot_arm_shoulder_pan_joint,robot_arm_shoulder_lift_joint,robot_arm_elbow_joint,robot_arm_wrist_1_joint,robot_arm_wrist_2_joint,robot_arm_wrist_3_joint',
                    f'jparse_max_joint_velocity:={self._jparse_limit("max_joint_velocity", 1.5):.6f}',
                    f'jparse_max_joint_acceleration:={self._jparse_limit("max_joint_acceleration", 0.5):.6f}',
                    f'jparse_max_cartesian_linear_velocity:={self._jparse_limit("max_cartesian_linear_velocity", 0.25):.6f}',
                    f'jparse_max_cartesian_angular_velocity:={self._jparse_limit("max_cartesian_angular_velocity", 0.8):.6f}',
                    *self._fixed_tool_arguments()]
        if name == 'transformations':
            return ['ros2', 'run', 'ur_trajectory_follower', 'current_pose_from_tf', '--ros-args',
                    '-p', f'target_frame:={frame}', '-p', 'source_frame:=robot_arm_nozzle_tip', '-p', 'pose_topic:=/current_nozzle_tip_pose']
        if name in {'base_accuracy', 'tcp_accuracy'}:
            mode = 'base' if name == 'base_accuracy' else 'tcp'
            actual = '/robot_pose' if mode == 'base' else '/current_deposition_pose'
            path = '/base_path' if mode == 'base' else '/ur_path_tracking'
            reference = '/base_trajectory_reference' if mode == 'base' else '/arm_trajectory_reference'
            phase = str(self._setting('accuracy_phase', 'baseline'))
            return ['ros2', 'run', 'print_path_monitoring', 'trajectory_accuracy_monitor', '--ros-args',
                    '-p', f'use_sim_time:={self._use_sim_time()}', '-p', f'mode:={mode}',
                    '-p', f'actual_pose_topic:={actual}', '-p', f'reference_path_topic:={path}',
                    '-p', f'reference_pose_topic:={reference}', '-p', 'path_index_topic:=/path_index',
                    '-p', 'output_directory:=/tmp/am_trajectory_runs', '-p', f'phase:={phase}',
                    '-p', 'required_frame:=map',
                    '-p', f"start_condition_topic:={'/start_pose_reached' if mode == 'base' else '/start_condition'}"]
        if name == 'accuracy_report':
            return ['ros2', 'run', 'print_path_monitoring', 'trajectory_accuracy_report',
                    '--input-directory', '/tmp/am_trajectory_runs', '--trajectory-directory', trajectory,
                    '--arm-base-offset', '0.26,0,1.046']
        if name == 'move_base':
            diff_drive = bool(self._control_setting('diff_drive_mode', False)) or str(self._setting('platform', 'robotnik')) == 'bunker'
            return ['ros2', 'run', 'move_to_path_idx', 'move_to_path_idx', '--ros-args',
                    '-p', f'use_sim_time:={self._use_sim_time()}', '-p', 'path_topic:=/base_path_tracking',
                    '-p', f"robot_pose_topic:={profile['robot_pose']}", '-p', 'robot_pose_type:=pose_stamped',
                    '-p', f"cmd_vel_topic:={profile['cmd_vel']}", '-p', f"output_stamped:={str(profile['stamped']).lower()}",
                    '-p', f"command_frame_id:={profile['frame']}", '-p', f'diff_drive_mode:={str(diff_drive).lower()}',
                    '-p', f'path_index:={move_index}', '-p', 'publish_start_condition:=false', '-p', 'start_condition_topic:=/start_pose_reached',
                    '-p', 'distance_tolerance:=0.06', '-p', 'yaw_tolerance:=0.08',
                    '-p', f'kp_linear:={self._pid("base_move.kp_linear", 0.6):.6f}', '-p', f'kp_lateral:={self._pid("base_move.kp_lateral", 0.6):.6f}',
                    '-p', f'kp_angular_to_point:={self._pid("base_move.kp_angular_to_point", 1.5):.6f}', '-p', f'kp_angular_reorient:={self._pid("base_move.kp_angular_reorient", 1.2):.6f}',
                    '-p', f'max_linear_velocity:={self._pid("base_move.max_linear_velocity", 0.2):.6f}', '-p', f'max_lateral_velocity:={self._pid("base_move.max_lateral_velocity", 0.2):.6f}',
                    '-p', f'max_angular_velocity:={self._pid("base_move.max_angular_velocity", 0.5):.6f}']
        if name == 'move_arm':
            return ['ros2', 'launch', 'move_to_path_idx', 'move_ur_to_path_idx.launch.py',
                    f'use_sim_time:={self._use_sim_time()}', 'path_topic:=/ur_path_tracking',
                    'current_pose_topic:=/current_deposition_pose', f'path_index:={move_index}',
                    'wait_for_start_condition:=false', 'start_condition_topic:=/start_pose_reached',
                    'cmd_vel_topic:=/jparse_velocity_controller_ur/twist_cmd_world', f'path_frame:={frame}',
                    f'kp_linear:={self._pid("arm_move.kp_linear", 0.8):.6f}', f'kp_angular:={self._pid("arm_move.kp_angular", 1.0):.6f}',
                    f'max_linear_velocity:={self._pid("arm_move.max_linear_velocity", 0.12):.6f}', f'max_angular_velocity:={self._pid("arm_move.max_angular_velocity", 0.5):.6f}']
        if name == 'switch_arm_velocity':
            manager = '/robot/controller_manager' if simulation else '/robot/arm/controller_manager'
            controller = 'arm_forward_velocity_controller' if simulation else 'forward_velocity_controller'
            return ['ros2', 'control', 'switch_controllers', '--controller-manager', manager, '--deactivate', 'joint_trajectory_controller', '--activate', controller]
        if name == 'rviz':
            rviz = 'bunker_operator.rviz' if str(self._setting('platform', 'robotnik')) == 'bunker' else 'robotnik_operator.rviz'
            return ['rviz2', '-d', str(ASSET_ROOT / 'rviz' / rviz), '-f', frame]
        if name == 'sync_workspace':
            move_index = self._live_path_index if self._live_path_index is not None else index
            return ['rsync', '-az', '-e', 'ssh', f'{REPO_ROOT.parent}/', SYNC_REMOTE_TARGET]
        return None

    def start_pose_adapters(self) -> None:
        """Start the same Vicon/odometry adapter set as the reference GUI."""
        # Simulation provides the nozzle through its URDF. Hardware uses the
        # same platform calibration as the controller's tool0-based KDL chain.
        if not bool(self._setting('simulation', False)):
            offset = validated_transform(self._fixed_tool_offset())
            arguments = []
            for flag, value in zip(
                ('--x', '--y', '--z', '--qx', '--qy', '--qz', '--qw'),
            move_index = self._live_path_index if self._live_path_index is not None else index
                [*offset['xyz'], *offset['quaternion_xyzw']],
            ):
                arguments.extend([flag, str(value)])
            self.processes.start('nozzle_static_tf', [
                'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                *arguments, '--frame-id', 'robot_arm_tool0',
                '--child-frame-id', 'robot_arm_nozzle_tip',
            ])
        frame = str(self._setting('control_frame', 'map'))
        base_frame = str(self._setting('robot_base_frame', self._profile()['frame']))
        root_frame = str(self._setting('robot_tree_root_frame', 'odom'))
        external_map = str(self._setting('external_map_frame', 'map'))
        self.processes.start('vicon_base_static_tf', ['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                '0.022595781', '-0.008234146', '-0.007327516', '0.004459784', '-0.006515752', '0.009033290', '0.999928025', 'robot_base_footprint', 'robot_base_vicon_reference'])
        vicon_offset = validated_transform(self._setting('vicon_nozzle_transform', DEFAULT_VICON_NOZZLE_TRANSFORM))
        self.processes.start('vicon_ee_static_tf', ['ros2', 'run', 'am_operator_gui', 'vicon_ee_static_tf', '--ros-args',
            '-p', f'use_sim_time:={self._use_sim_time()}',
            '-p', f"input_topic:={self._setting('vicon_input_topic', DEFAULT_VICON_INPUT_TOPIC)}",
            '-p', f'output_topic:={VICON_NOZZLE_TOPIC}',
            '-p', f"marker_to_nozzle_xyz:={vicon_offset['xyz']}",
            '-p', f"marker_to_nozzle_quaternion_xyzw:={vicon_offset['quaternion_xyzw']}"])
        # Independent measured reference for base reconstruction. The deposition
        # adapter above retains its original calibration and output topic.
        fallback_offset = validated_transform(self._setting(
            'vicon_fallback_nozzle_transform', vicon_offset))
        self.processes.start('vicon_fallback_nozzle_tf', [
            'ros2', 'run', 'am_operator_gui', 'vicon_ee_static_tf', '--ros-args',
            '-r', '__node:=vicon_fallback_nozzle_transform',
            '-p', f'use_sim_time:={self._use_sim_time()}',
            '-p', f"input_topic:={self._setting('vicon_input_topic', DEFAULT_VICON_INPUT_TOPIC)}",
            '-p', 'output_topic:=/vicon/nozzle_fallback',
            '-p', 'tcp_frame:=vicon_nozzle_fallback',
            '-p', 'publish_marker_tf:=false',
            '-p', f"marker_to_nozzle_xyz:={fallback_offset['xyz']}",
            '-p', f"marker_to_nozzle_quaternion_xyzw:={fallback_offset['quaternion_xyzw']}",
        ])
        command = ['ros2', 'run', 'am_operator_gui', 'external_base_reference', '--ros-args',
            '-p', f'use_sim_time:={self._use_sim_time()}', '-p', f"input_topic:={self._setting('base_pose_topic', '/vicon/Base/root')}",
            '-p', 'input_pose_frame:=robot_base_vicon_reference', '-p', 'output_topic:=/robot_pose',
            '-p', 'tcp_topic:=/vicon/nozzle_fallback',
            '-p', 'robot_tcp_frame:=robot_arm_nozzle_tip',
            '-p', f'map_frame:={external_map}', '-p', f'robot_base_frame:={base_frame}',
            '-p', f'robot_tree_root_frame:={root_frame}', '-p', f"odom_topic:={self._profile()['odom']}",
            '-p', f"use_odometry_robot_pose:={str(bool(self._setting('use_odometry_robot_pose', False))).lower()}",
            '-p', f"use_vicon_tcp_base_pose_fallback:={str(bool(self._setting('use_vicon_tcp_base_pose_fallback', False))).lower()}"]
        self.processes.stop('odometry_pose_adapter')
        self.processes.stop('vicon_tcp_pose_backup')
        self.processes.start('base_pose_adapter', command)
        self.processes.start('arm_pose_adapter', ['ros2', 'run', 'am_operator_gui', 'pose_stamped_adapter', '--ros-args',
            '-p', f'use_sim_time:={self._use_sim_time()}', '-p', f'input_topic:={VICON_NOZZLE_TOPIC}',
            '-p', 'output_topic:=/current_nozzle_tip_pose', '-p', f'target_frame:={frame}'])

    def _publish_start_condition_repeatedly(self, value: bool) -> None:
        """Mirror the reference GUI's transient-local start/stop safety pulses."""
        for attempt in range(5):
            self._schedule(0.2 * attempt, lambda value=value: self.ros_bridge.publish_start_condition(value))
        self.log('ros', f'published /start_condition {str(value).lower()} five times')

    @staticmethod
    def _yaw(orientation: Any) -> float:
        return math.atan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y), 1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2))

    def calculate_path_transform(self) -> None:
        if not self.ensure_ros():
            return
        index = int(self._setting('path_index', 0))
        path_pose = self.ros_bridge.latest_base_path_pose(index)
        robot_pose = self.ros_bridge.latest_robot_pose()
        if path_pose is None or robot_pose is None:
            self.log('calibration', 'path transform requires /base_path at the selected index and a fresh /robot_pose')
            return
        trajectory = str(self._setting('trajectory_directory', ''))
        current = self._path_transform(trajectory)
        path, robot = path_pose.pose, robot_pose.pose
        delta_yaw = self._yaw(robot.orientation) - self._yaw(path.orientation)
        cos_yaw, sin_yaw = math.cos(delta_yaw), math.sin(delta_yaw)
        path_x = cos_yaw * path.position.x - sin_yaw * path.position.y
        path_y = sin_yaw * path.position.x + cos_yaw * path.position.y
        old_x, old_y = float(current.get('x', 0.0)), float(current.get('y', 0.0))
        transform = {
            'x': cos_yaw * old_x - sin_yaw * old_y + robot.position.x - path_x,
            'y': sin_yaw * old_x + cos_yaw * old_y + robot.position.y - path_y,
            'z': float(current.get('z', 0.0)) + robot.position.z - path.position.z,
            'yaw_deg': math.degrees(math.atan2(math.sin(math.radians(float(current.get('yaw_deg', 0.0))) + delta_yaw), math.cos(math.radians(float(current.get('yaw_deg', 0.0))) + delta_yaw))),
        }
        self.config['path_transform'] = transform  # Legacy fallback.
        if trajectory:
            settings = self._platform_settings()
            # _platform_settings returns the mutable nested dictionary from
            # self.config, so this preserves the transform per platform.
            transforms = settings.setdefault('path_transforms_by_directory', {})
            if not isinstance(transforms, dict):
                transforms = {}
                settings['path_transforms_by_directory'] = transforms
            transforms[str(Path(trajectory).expanduser().resolve())] = transform
        self.store.save(self.config)
        self._last_action_messages['calculate_path_transform'] = (
            f'Path transform calculated at index {index}'
        )
        self.log('calibration', f'calculated path transform at index {index}')

    def stop_all(self) -> None:
        # Stopping processes must be safe even when this controller has never
        # connected to ROS (for example in a configuration-only web session).
        if self.ros_bridge is not None:
            self.ros_bridge.publish_start_condition(False)
            self.ros_bridge.publish_stop_commands(str(self._setting('control_frame', 'map')))
        self.processes.stop_all()
        for timer in self._timers:
            timer.cancel()
        self._timers.clear()
        self._launch_all_active = False
        self._following_active = False
        self.log('system', 'all managed processes stopped')

    def close(self) -> None:
        self.stop_all()
        self.ur_status_monitor.close()
        if self.ros_bridge is not None:
            self.ros_bridge.stop()
