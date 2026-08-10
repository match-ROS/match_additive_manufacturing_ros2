"""Toolkit-neutral control and process layer shared by local operator UIs.

This deliberately owns neither Qt nor FastAPI.  A presentation layer supplies
settings and invokes named actions; status and bounded logs are read back as data.
"""

from __future__ import annotations

import math
import os
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
from threading import Lock, Timer
from typing import Any, Callable, Optional

from .config_store import ConfigStore
from .process_manager import ProcessRegistry


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[2]


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
    # Keep the browser/operator-service path in sync with the Qt GUI.  These
    # are simulation profiles, not aliases for the older Robotnik platform.
    'mur620_sim': {'cmd_vel': '/mur620a/mobile_base_controller/cmd_vel', 'stamped': True,
                   'frame': 'mur620a/base_footprint', 'odom': '/mur620a/ground_truth/odom',
                   'robot_pose': '/mur620a/ground_truth/pose', 'path': '/base_path',
                   'follower_type': 'pure_pursuit', 'diff_drive_mode': True,
                   'arm_control_supported': False},
    'mur620_left_arm_sim': {'cmd_vel': '/mur620a/mobile_base_controller/cmd_vel', 'stamped': True,
                            'frame': 'mur620a/base_footprint', 'odom': '/mur620a/ground_truth/odom',
                            'robot_pose': '/mur620a/ground_truth/pose', 'path': '/base_path',
                            'mur_native_arm': True,
                            'follower_type': 'pure_pursuit', 'diff_drive_mode': True,
                            'arm_base_link': 'mur620a/UR10_l/base_link',
                            'arm_command_frame': 'UR10_l/base_link',
                            'arm_tip_link': 'mur620a/UR10_l/tool0',
                            'arm_joint_prefix': 'UR10_l/',
                            'robot_description_topic': '/mur620a/robot_description',
                            'joint_states_topic': '/mur620a/joint_states',
                            'arm_velocity_command_topic': '/mur620a/forward_velocity_controller_l/commands',
                            'arm_controller_manager': '/mur620a/controller_manager',
                            'arm_trajectory_topic': '/mur620a/joint_trajectory_controller/joint_trajectory',
                            'arm_world_twist_topic': '/mur620a/arm_following/twist_world',
                            'arm_stop_topic': '/mur620a/jparse_velocity_controller_l/twist_cmd'},
}

MUR_ARMS = {'none': 'None', 'l': 'Left', 'r': 'Right'}
MUR_NATIVE_ARMS = {'l', 'r'}
LEGACY_MUR_PLATFORM = 'mur620_left_arm_sim'

POSE_ADAPTER_PROCESSES = (
    'vicon_base_static_tf', 'vicon_ee_static_tf', 'base_pose_adapter',
    'odometry_pose_adapter', 'vicon_tcp_pose_backup', 'arm_pose_adapter',
)

TOGGLE_ACTIONS = {
    'simulation': ('simulation', 'Launch Sim', 'Stop Sim'),
    'publish_path': ('publish_path', 'Publish Path', 'Stop Path'),
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
        self.logs: deque[dict[str, str]] = deque(maxlen=1000)
        self._lock = Lock()
        self._output_callback = output_callback
        self._status_callback = status_callback
        self._external_path_index_callback = path_index_callback
        self.processes = ProcessRegistry(output_callback=self._on_output)
        self.ros_bridge = None
        self.ros_error: str | None = None
        self._status = {'path': False, 'robot_pose': False, 'arm_pose': False,
                        'jparse_ready': False, 'controller_ready': False}
        self._launch_all_active = False
        self._timers: list[Timer] = []
        self._following_active = False
        self._last_action_messages: dict[str, str] = {}
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
                robot_pose_topic=str(self._profile()['robot_pose']),
            )
            self.ros_bridge.start()
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

    def update_config(self, values: dict[str, Any]) -> dict[str, Any]:
        # Keep the public web endpoint deliberately explicit; unknown keys cannot
        # accidentally become command-line arguments.
        allowed = {'simulation', 'simulation_gui', 'platform', 'trajectory_directory', 'control_frame',
                   'base_pose_topic', 'arm_pose_topic', 'external_map_frame',
                   'robot_base_frame', 'robot_tree_root_frame', 'use_odometry_robot_pose',
                   'use_vicon_tcp_base_pose_fallback', 'default_velocity',
                   'default_velocity_enabled', 'spray_distance_mm', 'path_transform',
                   'path_transforms_by_directory', 'platform_control_settings', 'pid_gains',
                   'path_transforms_by_platform_directory',
                   'base_smoothing', 'fixed_tool_offset', 'path_index', 'original_arm_index',
                   'velocity_override', 'nozzle_offset_mm', 'follower_type', 'diff_drive_mode',
                   'direction_mode', 'accuracy_phase', 'mur_arm', 'fixed_tool_offset_input_mode'}
        if 'mur_arm' in values:
            arm = str(values['mur_arm']).strip().lower()
            if arm not in MUR_ARMS:
                values = dict(values)
                values.pop('mur_arm')
            else:
                values = dict(values)
                values['mur_arm'] = arm
        self.config.update({key: value for key, value in values.items() if key in allowed})
        if 'platform' in values and self.ros_bridge is not None:
            configure_pose = getattr(self.ros_bridge, 'set_robot_pose_topic', None)
            if configure_pose is not None:
                configure_pose(str(self._profile()['robot_pose']))
        if 'path_index' in values:
            self._live_path_index = max(0, int(values['path_index']))
        if 'original_arm_index' in values:
            self._live_original_arm_index = max(0, int(values['original_arm_index']))
        self.store.save(self.config)
        if self.ros_bridge is not None:
            if 'path_index' in values:
                self.ros_bridge.publish_path_index(int(self._setting('path_index', 0)))
            if 'velocity_override' in values:
                self.ros_bridge.publish_velocity_override(float(self._setting('velocity_override', 100.0)) / 100.0)
            if 'spray_distance_mm' in values or 'nozzle_offset_mm' in values:
                effective = float(self._setting('spray_distance_mm', 100.0)) + float(self._setting('nozzle_offset_mm', 0.0))
                self.ros_bridge.publish_spray_distance(effective / 1000.0)
            if 'default_velocity' in values or 'default_velocity_enabled' in values:
                desired = float(self._setting('default_velocity', 0.1)) if self._setting('default_velocity_enabled', False) else 0.0
                self.ros_bridge.publish_desired_arm_speed(desired)
        return self.config

    def snapshot(self) -> dict[str, Any]:
        processes = {}
        for name, managed in self.processes._processes.items():
            processes[name] = {'running': managed.is_running(), 'return_code': managed.poll()}
        config = dict(self.config)
        # Present the former left-arm-only profile as the unified MUR profile
        # without rewriting an existing operator configuration on read.
        if config.get('platform') == LEGACY_MUR_PLATFORM:
            config['platform'] = 'mur620_sim'
            config.setdefault('mur_arm', 'l')
        defaults = {
            'platform': 'robotnik',
            'simulation': False,
            'trajectory_directory': str(REPO_ROOT / 'components' / 'robotnik_paired_demo'),
            'control_frame': 'map',
            'base_pose_topic': '/vicon/Base_RB/Base_RB',
            'arm_pose_topic': '/vicon/tool_transformed',
            'external_map_frame': 'map',
            'robot_base_frame': 'base_link',
            'robot_tree_root_frame': 'odom',
            'path_index': 0,
            'original_arm_index': 0,
            'default_velocity_enabled': False,
            'default_velocity': 0.1,
            'spray_distance_mm': 100.0,
            'mur_arm': 'none',
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
        config.setdefault('contour_control_enabled', False)
        config.setdefault('nozzle_offset_mm', 0)
        config.setdefault('fixed_tool_offset_input_mode', 'quaternion')
        return {'config': config, 'status': self._status, 'processes': processes,
                'actions': self._action_states(),
                'logs': list(self.logs), 'ros_error': self.ros_error}

    def _process_state(self, names: tuple[str, ...], start_label: str, stop_label: str, one_shot: bool = False) -> dict[str, str]:
        managed = [self.processes.get(name) for name in names]
        present = [process for process in managed if process is not None]
        running = any(process.is_running() for process in present)
        if running:
            return {'label': stop_label, 'state': 'progress' if one_shot else 'running', 'detail': 'Prozess läuft'}
        if not present:
            return {'label': start_label, 'state': 'idle', 'detail': 'Noch nicht gestartet'}
        return_codes = [process.poll() for process in present]
        if any(code not in (None, 0) for code in return_codes):
            return {'label': start_label, 'state': 'error', 'detail': 'Letzter Prozess endete mit Fehler'}
        if one_shot:
            return {'label': start_label, 'state': 'success', 'detail': 'Letzte Aktion erfolgreich beendet'}
        return {'label': start_label, 'state': 'success', 'detail': 'Letzter Prozess erfolgreich beendet'}

    def _action_states(self) -> dict[str, dict[str, str]]:
        states = {
            action: self._process_state((process,), start, stop)
            for action, (process, start, stop) in TOGGLE_ACTIONS.items()
        }
        states.update({
            action: self._process_state((process,), start, stop, one_shot=True)
            for action, (process, start, stop) in ONE_SHOT_ACTIONS.items()
        })
        launch_processes = tuple(self.processes.get(name) for name in self._launch_all_process_names())
        launch_running = self._launch_all_active or any(process is not None and process.is_running() for process in launch_processes)
        states['launch_all'] = {
            'label': 'Stop All' if launch_running else 'Launch All',
            'state': 'running' if launch_running else 'idle',
            'detail': 'Verwalteter Komponentensatz aktiv' if launch_running else 'Komponentensatz nicht gestartet',
        }
        states['stop_all'] = {'label': 'Stop All', 'state': 'danger', 'detail': 'Alle verwalteten Prozesse stoppen'}
        transformations = self._process_state(
            POSE_ADAPTER_PROCESSES if not bool(self._setting('simulation', False)) else ('transformations',),
            'Launch Transformations',
            'Stop Transformations',
        )
        if bool(self._setting('simulation', False)) and self._is_running('simulation'):
            transformations = {'label': 'TCP Pose from Sim', 'state': 'running', 'detail': 'Simulation publiziert die TCP-Pose'}
        states['transformations'] = transformations
        states['pose_adapters'] = self._process_state(POSE_ADAPTER_PROCESSES, 'Pose Adapters', 'Stop Pose Adapters')
        states['rviz'] = self._process_state(('rviz',), 'Open RViz', 'Open RViz')
        states['capture_tool_offset'] = self._message_state('capture_tool_offset', 'Capture UR TCP Offset')
        states['calculate_path_transform'] = self._message_state('calculate_path_transform', 'Calculate Path Transform')
        ready = all(self._status.values())
        controls = all(self._is_running(name) for name in ('path_index', 'base_follower', 'arm_follower'))
        states['start_following'] = {
            'label': 'Following active' if self._following_active else 'Start Following',
            'state': 'running' if self._following_active else ('ready' if ready and controls else 'warning'),
            'detail': 'Following aktiv' if self._following_active else ('Bereit für Following' if ready and controls else 'Wartet auf ROS-Status oder Steuerprozesse'),
        }
        states['stop_following'] = {
            'label': 'Stop Following',
            'state': 'danger' if self._following_active else 'idle',
            'detail': 'Following stoppen' if self._following_active else 'Following ist nicht aktiv',
        }
        return states

    def _is_running(self, name: str) -> bool:
        process = self.processes.get(name)
        return process is not None and process.is_running()

    def _message_state(self, action: str, label: str) -> dict[str, str]:
        message = self._last_action_messages.get(action)
        return {
            'label': label,
            'state': 'success' if message else 'idle',
            'detail': message or 'Noch nicht ausgeführt',
        }

    def _setting(self, name: str, default: Any) -> Any:
        return self.config.get(name, default)

    def _use_sim_time(self) -> str:
        return str(bool(self._setting('simulation', False))).lower()

    def _profile(self) -> dict[str, Any]:
        platform = str(self._setting('platform', 'robotnik')).lower()
        profile = dict(PROFILES.get(platform, PROFILES['robotnik']))
        if platform in {'mur620_sim', LEGACY_MUR_PLATFORM}:
            default_arm = 'l' if platform == LEGACY_MUR_PLATFORM else 'none'
            arm = str(self._setting('mur_arm', default_arm)).strip().lower()
            if arm not in MUR_ARMS:
                arm = default_arm
            if arm not in MUR_NATIVE_ARMS:
                return profile
            prefix = f'UR10_{arm}'
            suffix = f'_{arm}'
            profile.update({
                'mur_native_arm': True,
                'arm_control_supported': True,
                'arm_base_link': f'mur620a/{prefix}/base_link',
                'arm_command_frame': f'{prefix}/base_link',
                'arm_tip_link': f'mur620a/{prefix}/tool0',
                'arm_joint_prefix': f'{prefix}/',
                'robot_description_topic': '/mur620a/robot_description',
                'joint_states_topic': '/mur620a/joint_states',
                'arm_velocity_command_topic': f'/mur620a/forward_velocity_controller{suffix}/commands',
                'arm_controller_manager': '/mur620a/controller_manager',
                'arm_trajectory_topic': f'/mur620a/joint_trajectory_controller{suffix}/joint_trajectory',
                'arm_world_twist_topic': '/mur620a/arm_following/twist_world',
                'arm_stop_topic': f'/mur620a/jparse_velocity_controller{suffix}/twist_cmd',
                'arm_stop_frame': f'{prefix}/base_link',
                'arm_selected': arm,
            })
        return profile

    def _control_setting(self, name: str, default: Any) -> Any:
        # The web form stores an explicit current value; the older Qt GUI stores
        # values per platform. Honour both representations during migration.
        if name in self.config:
            return self.config[name]
        profile_default = self._profile().get(name, default)
        settings = self._setting('platform_control_settings', {})
        platform = str(self._setting('platform', 'robotnik')).lower()
        if isinstance(settings, dict) and isinstance(settings.get(platform), dict):
            return settings[platform].get(name, profile_default)
        return profile_default

    def _pid(self, name: str, default: float) -> float:
        gains = self._setting('pid_gains', {})
        try:
            return float(gains.get(name, default)) if isinstance(gains, dict) else default
        except (TypeError, ValueError):
            return default

    def _smoothing(self, name: str, default: Any) -> Any:
        settings = self._setting('base_smoothing', {})
        return settings.get(name, default) if isinstance(settings, dict) else default

    def _default_velocity_parameter(self) -> float:
        if not bool(self._setting('default_velocity_enabled', False)):
            return -1.0
        try:
            return float(self._setting('default_velocity', 0.1))
        except (TypeError, ValueError):
            return 0.1

    def _fixed_tool_arguments(self) -> list[str]:
        offset = self._setting('fixed_tool_offset', {})
        offset = offset if isinstance(offset, dict) else {}
        try:
            xyz = ', '.join(f'{float(value):.6f}' for value in offset.get('xyz', [-0.25, 0.0, 0.015]))
            quat = ', '.join(f'{float(value):.6f}' for value in offset.get('quaternion_xyzw', [0.0, -0.7071067812, 0.0, 0.7071067812]))
        except (TypeError, ValueError):
            return []
        return [f'fixed_tool_offset_xyz:=[{xyz}]', f'fixed_tool_offset_quaternion_xyzw:=[{quat}]']

    def _path_transform_for_trajectory(self, trajectory: str) -> dict[str, Any]:
        transform = self._setting('path_transform', {})
        try:
            trajectory_key = str(Path(trajectory).expanduser().resolve())
        except OSError:
            trajectory_key = trajectory
        platform_transforms = self._setting('path_transforms_by_platform_directory', {})
        platform = str(self._setting('platform', 'robotnik')).lower()
        has_platform_transform = False
        if isinstance(platform_transforms, dict):
            configured = platform_transforms.get(platform, {})
            if isinstance(configured, dict) and isinstance(configured.get(trajectory_key), dict):
                transform = configured.get(trajectory_key, transform)
                has_platform_transform = True
        by_directory = self._setting('path_transforms_by_directory', {})
        if not has_platform_transform and isinstance(by_directory, dict):
            try:
                transform = by_directory.get(trajectory_key, transform)
            except OSError:
                pass
        return transform if isinstance(transform, dict) else {}

    def _path_transform_arguments(self, trajectory: str) -> list[str]:
        transform = self._path_transform_for_trajectory(trajectory)
        x = float(transform.get('x', 0.0))
        y = float(transform.get('y', 0.0))
        z = float(transform.get('z', 0.0))
        yaw = float(transform.get('yaw_deg', 0.0))
        return [f'path_transform_xyz:=[{x:.6f}, {y:.6f}, {z:.6f}]', f'path_transform_yaw_deg:={yaw:.6f}']

    def _toggle(self, name: str, command: list[str]) -> None:
        running = self.processes.get(name)
        if running and running.is_running():
            self.processes.stop(name)
            self.log(name, 'stopped by operator')
        else:
            self.log(name, ' '.join(command))
            self.processes.start(name, command)

    def _start(self, name: str, command: list[str] | None = None) -> None:
        command = command if command is not None else self.command_for(name)
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
            arm_supported = bool(self._profile().get('arm_control_supported', True))
            components = ['publish_path', 'path_index', 'base_follower']
            if arm_supported:
                components[1:1] = ['controllers']
                components.append('arm_follower')
            for item in components:
                self._start(item)
            if bool(self._setting('simulation', False)):
                # The arm waits for the base's completion signal, preventing a
                # world-frame target from being pursued before the mobile base
                # has reached its selected path index.  Both one-shot movers
                # wait for their own ROS inputs, so no fixed startup sleep is
                # needed to synchronize them.
                if arm_supported:
                    arm_ready_topic = '/am/move_arm_ready'
                    self._start('move_arm', self.command_for(
                        'move_arm', wait_for_start_condition=True, ready_topic=arm_ready_topic))
                    self._start('move_base', self.command_for(
                        'move_base', publish_start_condition=True, wait_for_ready_topic=arm_ready_topic))
                else:
                    self._start('move_base', self.command_for('move_base', publish_start_condition=True))
            return
        if name == 'start_following':
            if self.ensure_ros():
                # The progress topic is live state.  Do not resume from the
                # last value saved in the configuration merely because the
                # operator paused following in between.
                index = self._live_path_index if self._live_path_index is not None else int(self._setting('path_index', 0))
                self.ros_bridge.publish_path_index(index)
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
            self.config['fixed_tool_offset'] = {
                'xyz': [translation.x, translation.y, translation.z],
                'quaternion_xyzw': [rotation.x, rotation.y, rotation.z, rotation.w],
            }
            self.store.save(self.config)
            self.log('calibration', 'captured UR TCP offset')
            self._last_action_messages['capture_tool_offset'] = 'UR TCP offset captured'
            return
        if name == 'calculate_path_transform':
            self.calculate_path_transform()
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
                    self._schedule(delay / 1000.0, self._publish_stop_commands)
            return
        if name == 'move_base' and not self._is_running('publish_path'):
            # A one-shot move is useful on its own, not only after Launch All.
            # Start its path dependency first; the mover waits until it has
            # received both that path and the selected platform pose.
            self._start('publish_path')
        if name in {'controllers', 'arm_follower', 'move_arm', 'switch_arm_velocity'}:
            if not bool(self._profile().get('arm_control_supported', True)):
                self.log('safety', f'{name} is unavailable: no MuR arm is selected')
                return
        command = self.command_for(name)
        if command is None:
            raise ValueError(f'unknown action: {name}')
        self._toggle(name, command)

    def _toggle_pose_adapters(self) -> None:
        if any(self._is_running(name) for name in POSE_ADAPTER_PROCESSES):
            for name in POSE_ADAPTER_PROCESSES:
                self.processes.stop(name)
            self.log('transformations', 'stopped by operator')
            return
        self.start_pose_adapters()

    def command_for(
        self,
        name: str,
        *,
        wait_for_start_condition: bool = False,
        publish_start_condition: bool = False,
        ready_topic: str = '',
        wait_for_ready_topic: str = '',
    ) -> list[str] | None:
        simulation = bool(self._setting('simulation', False))
        profile = self._profile()
        frame = str(self._setting('control_frame', 'map'))
        trajectory = str(self._setting('trajectory_directory', REPO_ROOT / 'components' / 'robotnik_paired_demo'))
        index = int(self._setting('path_index', 0))
        mur_native_arm = bool(profile.get('mur_native_arm', False))
        if name == 'simulation':
            platform = str(self._setting('platform', 'robotnik')).strip().lower()
            if platform == 'bunker':
                return ['ros2', 'launch', 'bunker_description', 'spawn_with_controllers.launch.py', 'headless:=true', 'launch_rviz:=false']
            if platform in {'mur620_sim', LEGACY_MUR_PLATFORM}:
                mur_native_arm = bool(self._profile().get('mur_native_arm', False))
                return [
                    'ros2', 'launch', 'mur_launch_sim', 'mur620.launch.py',
                    'robot_name:=mur620a', 'world:=empty', 'x:=44.0', 'y:=44.0',
                    'z:=0.07', 'Y:=0.0', 'include_gz:=true',
                    f"gazebo_gui:={str(bool(self._setting('simulation_gui', False))).lower()}",
                    f"use_camera:={str(bool(self._setting('simulation_gui', False))).lower()}",
                    f"enable_sensors:={str(bool(self._setting('simulation_gui', False))).lower()}",
                    f"use_simple_collisions:={str(not bool(self._setting('simulation_gui', False))).lower()}",
                    'ground_truth:=true', 'fake_localization:=true', 'navigation:=false',
                    f'load_arm_controllers:={str(mur_native_arm).lower()}',
                    'load_lift_controllers:=false', 'launch_moveit:=false',
                    # The AM stack starts am_jparse_controller. The native
                    # MuR controller remains the default outside this stack.
                    'launch_jparse_idk:=false',
                    'auto_switch_arm_controllers:=false',
                ]
            if platform == 'robotnik':
                return ['ros2', 'launch', 'robotnik_rbvogui_tum', 'rbvogui_ur_standard_control.launch.py', 'gui:=false', 'robot_id:=robot', 'arm_type:=ur20']
            self.log('simulation', f'refusing to launch simulation: unknown platform {platform!r}')
            return None
        if name == 'publish_path':
            return ['ros2', 'launch', 'parse_paths', 'robotnik_base_arm_paths.launch.py',
                    f'use_sim_time:={self._use_sim_time()}', f'frame_id:={frame}',
                    'load_exported_trajectories:=true', f'trajectory_directory:={trajectory}', 'publish_once:=false',
                    *self._path_transform_arguments(trajectory)]
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
            diff_drive = bool(self._control_setting('diff_drive_mode', False)) or bool(profile.get('diff_drive_mode', False))
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
                    f'robot_name:={"mur620a" if mur_native_arm else "robot"}', f'arm:={profile.get("arm_selected", "arm") if mur_native_arm else "arm"}',
                    f'joint_prefix:={profile.get("arm_joint_prefix", "robot_arm_")}',
                    f'base_link:={profile.get("arm_base_link", "robot_arm_base_link")}',
                    f'tip_link:={profile.get("arm_tip_link", "robot_arm_tool0")}',
                    f'robot_description_topic:={profile.get("robot_description_topic", "/robot/robot_description")}',
                    f'joint_states_topic:={profile.get("joint_states_topic", "/robot/joint_states")}',
                    f"velocity_command_topic:={profile.get('arm_velocity_command_topic', '/robot/arm_forward_velocity_controller/commands' if simulation else '/robot/arm/forward_velocity_controller/commands')}",
                    'start_jparse_controller:=false', 'start_command_transform:=false',
                    f'publish_current_pose_from_tf:={str(mur_native_arm).lower()}',
                    'publish_path:=false', 'publish_path_index:=false', 'move_to_start_pose:=false',
                    f"start_pose_trajectory_topic:={profile.get('arm_trajectory_topic', '/robot/joint_trajectory_controller/joint_trajectory' if simulation else '/robot/arm/joint_trajectory_controller/joint_trajectory')}",
                    'start_pose_publish_delay:=8.0',
                    f'derive_nozzle_pose_from_tcp:={str(simulation).lower()}',
                    'tcp_pose_topic:=/current_tcp_pose',
                    'nozzle_pose_topic:=/current_nozzle_tip_pose',
                    'current_pose_topic:=/current_deposition_pose',
                    'spray_distance_topic:=/spray_distance', 'smoothed_spray_distance_topic:=/spray_distance_smoothed',
                    f"spray_distance_initial:={(float(self._setting('spray_distance_mm', 100.0)) + float(self._setting('nozzle_offset_mm', 0.0))) / 1000.0:.6f}",
                    'spray_distance_max_rate:=0.020000', 'path_topic:=/ur_path_transformed', 'original_path_topic:=/ur_path_original',
                    'normal_topic:=/normal_vector', 'path_index_topic:=/path_index', 'next_goal_topic:=/next_goal',
                    'wait_for_start_condition:=true', 'start_condition_topic:=/start_condition', f'initial_path_index:={index}',
                    f"progress_mode:={'desired_speed' if bool(self._setting('default_velocity_enabled', False)) else 'timestamp'}",
                    'arm_reference_topic:=/arm_trajectory_reference', 'desired_speed_topic:=/desired_arm_speed',
                    f'default_velocity:={self._default_velocity_parameter():.6f}',
                    *( [f'combined_twist_source_topic:={profile["arm_world_twist_topic"]}'] if mur_native_arm else [] ),
                    *self._fixed_tool_arguments(), *direction_gains, *orientation_gains]
        if name == 'controllers':
            if mur_native_arm:
                return ['ros2', 'launch', 'am_operator_gui', 'mur_arm_velocity_stack.launch.py',
                        f'use_sim_time:={self._use_sim_time()}', 'robot_name:=mur620a', f'arm:={profile["arm_selected"]}',
                        f'path_frame:={frame}',
                        f'arm_base_link:={profile["arm_base_link"]}',
                        f'controller_frame:={profile.get("arm_command_frame", profile["arm_base_link"])}',
                        f'source_twist_topic:={profile["arm_world_twist_topic"]}',
                        f'controller_twist_topic:={profile["arm_stop_topic"]}',
                        f'velocity_command_topic:={profile["arm_velocity_command_topic"]}',
                        f'tip_link:={profile["arm_tip_link"]}',
                        f'robot_description_topic:={profile["robot_description_topic"]}',
                        f'joint_states_topic:={profile["joint_states_topic"]}',
                        'spray_distance_topic:=/spray_distance_smoothed',
                        'jparse_readiness_topic:=/am/jparse_ready',
                        *self._fixed_tool_arguments()]
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
            diff_drive = bool(self._control_setting('diff_drive_mode', False)) or bool(profile.get('diff_drive_mode', False))
            start_target_yaw_mode = str(profile.get('start_target_yaw_mode', 'auto'))
            return ['ros2', 'run', 'move_to_path_idx', 'move_to_path_idx', '--ros-args',
                    '-p', f'use_sim_time:={self._use_sim_time()}', '-p', f"path_topic:={profile['path']}",
                    '-p', f"robot_pose_topic:={profile['robot_pose']}", '-p', 'robot_pose_type:=pose_stamped',
                    '-p', f"cmd_vel_topic:={profile['cmd_vel']}", '-p', f"output_stamped:={str(profile['stamped']).lower()}",
                    '-p', f"command_frame_id:={profile['frame']}", '-p', f'diff_drive_mode:={str(diff_drive).lower()}',
                    '-p', f'target_yaw_mode:={start_target_yaw_mode}', '-p', f'path_index:={index}',
                    *( ['-p', f'wait_for_ready_topic:={wait_for_ready_topic}'] if wait_for_ready_topic else [] ),
                    '-p', f'publish_start_condition:={str(publish_start_condition).lower()}',
                    '-p', 'start_condition_topic:=/start_pose_reached',
                    '-p', 'distance_tolerance:=0.06', '-p', 'yaw_tolerance:=0.08',
                    '-p', f'kp_linear:={self._pid("base_move.kp_linear", 0.6):.6f}', '-p', f'kp_lateral:={self._pid("base_move.kp_lateral", 0.6):.6f}',
                    '-p', f'kp_angular_to_point:={self._pid("base_move.kp_angular_to_point", 1.5):.6f}', '-p', f'kp_angular_reorient:={self._pid("base_move.kp_angular_reorient", 1.2):.6f}',
                    '-p', f'max_linear_velocity:={self._pid("base_move.max_linear_velocity", 0.2):.6f}', '-p', f'max_lateral_velocity:={self._pid("base_move.max_lateral_velocity", 0.2):.6f}',
                    '-p', f'max_angular_velocity:={self._pid("base_move.max_angular_velocity", 0.5):.6f}']
        if name == 'move_arm':
            original_index = int(self._setting('original_arm_index', index))
            command_topic = profile.get('arm_world_twist_topic', '/jparse_velocity_controller_ur/twist_cmd_world')
            return ['ros2', 'launch', 'move_to_path_idx', 'move_ur_to_path_idx.launch.py',
                    f'use_sim_time:={self._use_sim_time()}', 'path_topic:=/ur_path_transformed',
                    'current_pose_topic:=/current_deposition_pose', f'path_index:={original_index}',
                    f'wait_for_start_condition:={str(wait_for_start_condition).lower()}',
                    'start_condition_topic:=/start_pose_reached',
                    *( [f'ready_topic:={ready_topic}'] if ready_topic else [] ),
                    f'cmd_vel_topic:={command_topic}', f'path_frame:={frame}',
                    f'kp_linear:={self._pid("arm_move.kp_linear", 0.8):.6f}', f'kp_angular:={self._pid("arm_move.kp_angular", 1.0):.6f}',
                    f'max_linear_velocity:={self._pid("arm_move.max_linear_velocity", 0.12):.6f}', f'max_angular_velocity:={self._pid("arm_move.max_angular_velocity", 0.5):.6f}']
        if name == 'switch_arm_velocity':
            if mur_native_arm:
                arm = profile['arm_selected']
                return ['ros2', 'control', 'switch_controllers', '--controller-manager', profile['arm_controller_manager'],
                        '--deactivate', f'joint_trajectory_controller_{arm}', '--activate', f'forward_velocity_controller_{arm}']
            manager = '/robot/controller_manager' if simulation else '/robot/arm/controller_manager'
            controller = 'arm_forward_velocity_controller' if simulation else 'forward_velocity_controller'
            return ['ros2', 'control', 'switch_controllers', '--controller-manager', manager, '--deactivate', 'joint_trajectory_controller', '--activate', controller]
        if name == 'rviz':
            rviz = 'bunker_operator.rviz' if str(self._setting('platform', 'robotnik')) == 'bunker' else 'robotnik_operator.rviz'
            return ['rviz2', '-d', str(ASSET_ROOT / 'rviz' / rviz), '-f', frame]
        if name == 'sync_workspace':
            return ['rsync', '-az', '-e', 'ssh', f'{REPO_ROOT.parent}/', 'ite-dcs@192.168.0.222:~/workspaces/print_wattle_daub/src/']
        return None

    def start_pose_adapters(self) -> None:
        """Start the same Vicon/odometry adapter set as the reference GUI."""
        frame = str(self._setting('control_frame', 'map'))
        base_frame = str(self._setting('robot_base_frame', self._profile()['frame']))
        root_frame = str(self._setting('robot_tree_root_frame', 'odom'))
        external_map = str(self._setting('external_map_frame', 'map'))
        if not bool(self._setting('use_odometry_robot_pose', False)) and not bool(self._setting('use_vicon_tcp_base_pose_fallback', False)):
            self.processes.start('vicon_base_static_tf', ['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                '0.022595781', '-0.008234146', '-0.007327516', '0.004459784', '-0.006515752', '0.009033290', '0.999928025', 'robot_base_footprint', 'robot_base_vicon_reference'])
        self.processes.start('vicon_ee_static_tf', ['ros2', 'run', 'am_operator_gui', 'vicon_ee_static_tf', '--ros-args',
            '-p', f'use_sim_time:={self._use_sim_time()}', '-p', 'input_topic:=/vicon/Tool_Flange/Tool_Flange', '-p', 'output_topic:=/vicon/tool_transformed'])
        if bool(self._setting('use_odometry_robot_pose', False)):
            command = ['ros2', 'run', 'am_operator_gui', 'odometry_robot_pose', '--ros-args',
                '-p', f'use_sim_time:={self._use_sim_time()}', '-p', f"odom_topic:={self._profile()['odom']}",
                '-p', 'path_topic:=/base_path', '-p', 'output_topic:=/robot_pose',
                '-p', f"initial_path_index:={int(self._setting('path_index', 0))}", '-p', f'map_frame:={external_map}',
                '-p', f'odom_frame:={root_frame}', '-p', f'robot_base_frame:={base_frame}', '-p', 'publish_tf:=true']
            self.processes.start('odometry_pose_adapter', command)
        elif bool(self._setting('use_vicon_tcp_base_pose_fallback', False)):
            command = ['ros2', 'run', 'am_operator_gui', 'vicon_tcp_robot_pose_backup', '--ros-args',
                '-p', f'use_sim_time:={self._use_sim_time()}', '-p', 'input_topic:=/vicon/tool_transformed',
                '-p', 'output_topic:=/robot_pose', '-p', f'map_frame:={external_map}',
                '-p', f'robot_base_frame:={base_frame}', '-p', f'robot_tree_root_frame:={root_frame}']
            self.processes.start('vicon_tcp_pose_backup', command)
        else:
            command = ['ros2', 'run', 'am_operator_gui', 'external_base_reference', '--ros-args',
                '-p', f'use_sim_time:={self._use_sim_time()}', '-p', f"input_topic:={self._setting('base_pose_topic', '/vicon/Base_RB/Base_RB')}",
                '-p', 'input_pose_frame:=robot_base_vicon_reference', '-p', 'output_topic:=/robot_pose',
                '-p', f'map_frame:={external_map}', '-p', f'robot_base_frame:={base_frame}', '-p', f'robot_tree_root_frame:={root_frame}']
            self.processes.start('base_pose_adapter', command)
        self.processes.start('arm_pose_adapter', ['ros2', 'run', 'am_operator_gui', 'pose_stamped_adapter', '--ros-args',
            '-p', f'use_sim_time:={self._use_sim_time()}', '-p', f"input_topic:={self._setting('arm_pose_topic', '/vicon/tool_transformed')}",
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
        current = self._path_transform_for_trajectory(trajectory)
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
        if trajectory:
            trajectory_key = str(Path(trajectory).expanduser().resolve())
            platform = str(self._setting('platform', 'robotnik')).lower()
            platform_transforms = self.config.setdefault('path_transforms_by_platform_directory', {})
            if isinstance(platform_transforms, dict):
                per_platform = platform_transforms.setdefault(platform, {})
                if isinstance(per_platform, dict):
                    per_platform[trajectory_key] = transform
        self.store.save(self.config)
        if self._is_running('publish_path'):
            # The publisher applies its registration only at launch.  Replace
            # it immediately so Calculate Path Transform changes the live path
            # that Move Base/Arm To Start will use.
            self.processes.stop('publish_path')
            self._start('publish_path')
        self._last_action_messages['calculate_path_transform'] = (
            f'Path transform calculated at index {index}'
        )
        self.log('calibration', f'calculated path transform at index {index}')

    def stop_all(self) -> None:
        # Stopping processes must be safe even when this controller has never
        # connected to ROS (for example in a configuration-only web session).
        if self.ros_bridge is not None:
            self.ros_bridge.publish_start_condition(False)
            self._publish_stop_commands()
        self.processes.stop_all()
        for timer in self._timers:
            timer.cancel()
        self._timers.clear()
        self._launch_all_active = False
        self._following_active = False
        self.log('system', 'all managed processes stopped')

    def _publish_stop_commands(self) -> None:
        """Stop the active platform's base and arm without its follower stack."""
        if self.ros_bridge is None:
            return
        profile = self._profile()
        arm_frame = str(profile.get('arm_command_frame', self._setting('control_frame', 'map')))
        try:
            self.ros_bridge.publish_stop_commands(
                arm_frame,
                base_topic=str(profile['cmd_vel']),
                base_stamped=bool(profile['stamped']),
                base_frame=str(profile['frame']),
                arm_topic=str(profile.get('arm_stop_topic', '/jparse_velocity_controller_ur/twist_cmd_world')),
            )
        except TypeError:
            # Retain compatibility with minimal bridges used by integrations.
            self.ros_bridge.publish_stop_commands(arm_frame)

    def close(self) -> None:
        self.stop_all()
        if self.ros_bridge is not None:
            self.ros_bridge.stop()
