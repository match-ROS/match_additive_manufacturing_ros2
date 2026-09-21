"""Low-frequency, non-blocking UR status monitoring over one ROS node."""

from __future__ import annotations

import time
from threading import Lock, Thread
from typing import Any, Callable


DASHBOARD_NAMESPACE = '/robot/arm/dashboard_client'
ARM_CONTROLLER_MANAGER = '/robot/arm/controller_manager'
FORWARD_VELOCITY_CONTROLLER = 'forward_velocity_controller'
DASHBOARD_PERIOD_S = 10.0
CONTROLLER_PERIOD_S = 30.0
REQUEST_TIMEOUT_S = 5.0
SAFETY_MODES = {1: 'NORMAL', 2: 'REDUCED', 3: 'PROTECTIVE_STOP', 4: 'RECOVERY', 5: 'SAFEGUARD_STOP',
                6: 'SYSTEM_EMERGENCY_STOP', 7: 'ROBOT_EMERGENCY_STOP', 8: 'VIOLATION', 9: 'FAULT'}
ROBOT_MODES = {-1: 'NO_CONTROLLER', 0: 'DISCONNECTED', 1: 'CONFIRM_SAFETY', 2: 'BOOTING',
               3: 'POWER_OFF', 4: 'POWER_ON', 5: 'IDLE', 6: 'BACKDRIVE', 7: 'RUNNING'}


def dashboard_command(action: str) -> list[str]:
    """Return a fixed dashboard command; the UI never accepts arbitrary ROS commands."""
    services = {
        'play_program': ('play', 'std_srvs/srv/Trigger'),
        'unlock_protective_stop': ('unlock_protective_stop', 'std_srvs/srv/Trigger'),
        'release_brakes': ('brake_release', 'std_srvs/srv/Trigger'),
    }
    service, service_type = services[action]
    return ['ros2', 'service', 'call', f'{DASHBOARD_NAMESPACE}/{service}', service_type, '{}']


def enable_command(unlock_protective_stop: bool) -> list[str]:
    """Power on and release brakes in order after an explicit Enable UR click."""
    trigger = lambda service: (
        f'ros2 service call {DASHBOARD_NAMESPACE}/{service} std_srvs/srv/Trigger {{}}'
    )
    commands = ([trigger('unlock_protective_stop')] if unlock_protective_stop else [])
    commands.extend((trigger('power_on'), trigger('brake_release')))
    return ['bash', '-c', ' && '.join(commands)]


class UrStatusMonitor:
    """Own one ROS node and cache low-frequency dashboard/controller status.

    The web UI may render every second, but rendering only reads this cache.  ROS
    clients are created once, so ordinary polling does not repeatedly churn DDS
    discovery endpoints.
    """

    def __init__(self, enabled: bool = True) -> None:
        self._lock = Lock()
        self._dashboard: dict[str, Any] = {
            'available': None, 'checked_at': None, 'error': None, 'stale': False,
        }
        self._controller: dict[str, Any] = {
            'available': None, 'loaded': None, 'state': None,
            'checked_at': None, 'error': None, 'stale': False,
        }
        self._started = False
        self._stopped = False
        self._enabled = bool(enabled)
        self._dashboard_in_flight = False
        self._controller_in_flight = False
        self._dashboard_refresh_requested = False
        self._controller_refresh_requested = False
        self._dashboard_deadline = 0.0
        self._controller_deadline = 0.0
        self._next_dashboard = 0.0
        self._next_controller = 0.0
        self._dashboard_index = 0
        self._dashboard_values: dict[str, Any] = {}
        self._node = None
        self._executor = None
        self._thread: Thread | None = None
        self._dashboard_queries: tuple[tuple[str, str, Any, Any, Callable[[Any], Any]], ...] = ()
        self._controller_client = None
        self._controller_service_type = None

    def start(self) -> bool:
        """Start the monitor after the application's ROS bridge has initialized rclpy."""
        with self._lock:
            if self._started:
                return True
            if self._stopped:
                return False
        try:
            import rclpy
            from controller_manager_msgs.srv import ListControllers
            from rclpy.executors import SingleThreadedExecutor
            from ur_dashboard_msgs.srv import (
                GetLoadedProgram, GetProgramState, GetRobotMode, GetSafetyMode, IsInRemoteControl,
            )
            if not rclpy.ok():
                rclpy.init(args=None)
            node = rclpy.create_node('am_operator_ur_status_monitor')
            executor = SingleThreadedExecutor()
            executor.add_node(node)
            queries = (
                ('loaded_program', 'get_loaded_program', GetLoadedProgram, lambda response: response.program_name),
                ('program_state', 'program_state', GetProgramState, lambda response: response.state.state),
                ('safety_mode', 'get_safety_mode', GetSafetyMode, lambda response: response.safety_mode.mode),
                ('robot_mode', 'get_robot_mode', GetRobotMode, lambda response: response.robot_mode.mode),
                ('remote_control', 'is_in_remote_control', IsInRemoteControl, lambda response: response.remote_control),
            )
            self._dashboard_queries = tuple(
                (key, service, service_type, node.create_client(service_type, f'{DASHBOARD_NAMESPACE}/{service}'), value)
                for key, service, service_type, value in queries
            )
            self._controller_client = node.create_client(
                ListControllers, f'{ARM_CONTROLLER_MANAGER}/list_controllers',
            )
            self._controller_service_type = ListControllers
            node.create_timer(0.25, self._tick)
            self._node = node
            self._executor = executor
            self._thread = Thread(target=self._spin, daemon=True, name='ur-status-monitor')
            with self._lock:
                self._started = True
                if self._enabled:
                    self._next_dashboard = time.monotonic()
                    self._next_controller = time.monotonic()
            self._thread.start()
            return True
        except (ImportError, RuntimeError, ValueError) as exc:
            self._set_dashboard_error(str(exc))
            self._set_controller_error(str(exc))
            return False

    def close(self) -> None:
        """Stop only this monitor; the shared application ROS context stays owned by RosBridge."""
        with self._lock:
            self._stopped = True
            executor, node, thread = self._executor, self._node, self._thread
            self._executor = None
            self._node = None
            self._thread = None
        if executor is not None:
            try:
                executor.shutdown()
            except Exception:
                pass
        if thread is not None:
            thread.join(timeout=1.0)
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass

    def dashboard_snapshot(self) -> dict[str, Any]:
        with self._lock:
            return dict(self._dashboard, checking=self._dashboard_in_flight, enabled=self._enabled)

    def controller_snapshot(self) -> dict[str, Any]:
        with self._lock:
            return dict(self._controller, checking=self._controller_in_flight, enabled=self._enabled)

    def set_enabled(self, enabled: bool) -> None:
        """Enable or suspend all periodic dashboard and controller queries together."""
        now = time.monotonic()
        with self._lock:
            self._enabled = bool(enabled)
            self._dashboard_refresh_requested = False
            self._controller_refresh_requested = False
            if not self._enabled:
                # An already sent request cannot be revoked, but its response
                # is discarded and no subsequent request is started.
                self._dashboard_in_flight = False
                self._controller_in_flight = False
                return
            self._next_dashboard = now
            self._next_controller = now

    def request_refresh(self, *, dashboard: bool = True, controller: bool = True) -> None:
        """Request one prompt refresh after an operator action without overlapping calls."""
        with self._lock:
            if not self._enabled:
                return
            if dashboard:
                self._dashboard_refresh_requested = True
            if controller:
                self._controller_refresh_requested = True

    def _spin(self) -> None:
        try:
            self._executor.spin()
        except Exception:
            # RosBridge may shut down the shared rclpy context during application exit.
            pass

    def _tick(self) -> None:
        now = time.monotonic()
        with self._lock:
            if not self._enabled:
                return
            dashboard_due = self._dashboard_refresh_requested or now >= self._next_dashboard
            controller_due = self._controller_refresh_requested or now >= self._next_controller
            if dashboard_due and not self._dashboard_in_flight:
                self._dashboard_refresh_requested = False
                self._next_dashboard = now + DASHBOARD_PERIOD_S
                start_dashboard = True
            else:
                start_dashboard = False
            if controller_due and not self._controller_in_flight:
                self._controller_refresh_requested = False
                self._next_controller = now + CONTROLLER_PERIOD_S
                start_controller = True
            else:
                start_controller = False
            dashboard_timed_out = self._dashboard_in_flight and now >= self._dashboard_deadline
            controller_timed_out = self._controller_in_flight and now >= self._controller_deadline
        if dashboard_timed_out:
            self._finish_dashboard_error('Dashboard service timed out')
        if controller_timed_out:
            self._finish_controller_error('Controller-manager service timed out')
        if start_dashboard:
            self._start_dashboard()
        if start_controller:
            self._start_controller()

    def _start_dashboard(self) -> None:
        with self._lock:
            self._dashboard_in_flight = True
            self._dashboard_deadline = time.monotonic() + REQUEST_TIMEOUT_S
            self._dashboard_index = 0
            self._dashboard_values = {}
        self._next_dashboard_query()

    def _next_dashboard_query(self) -> None:
        with self._lock:
            if not self._dashboard_in_flight or self._dashboard_index >= len(self._dashboard_queries):
                return
            key, service, service_type, client, value = self._dashboard_queries[self._dashboard_index]
        if not client.service_is_ready():
            self._finish_dashboard_error(f'Dashboard service unavailable: {service}')
            return
        try:
            future = client.call_async(service_type.Request())
            future.add_done_callback(
                lambda completed, query_key=key, value_fn=value: self._dashboard_response(
                    completed, query_key, value_fn,
                )
            )
        except Exception as exc:
            self._finish_dashboard_error(f'Dashboard service failed: {exc}')

    def _dashboard_response(self, future: Any, key: str, value_fn: Callable[[Any], Any]) -> None:
        try:
            response = future.result()
            if response is None:
                raise RuntimeError('empty response')
            value = value_fn(response)
        except Exception as exc:
            self._finish_dashboard_error(f'Dashboard service failed: {exc}')
            return
        with self._lock:
            if not self._dashboard_in_flight:
                return
            self._dashboard_values[key] = value
            self._dashboard_index += 1
            complete = self._dashboard_index == len(self._dashboard_queries)
            values = dict(self._dashboard_values) if complete else None
        if complete:
            try:
                values['safety_mode'] = SAFETY_MODES.get(int(values['safety_mode']), str(values['safety_mode']))
                values['robot_mode'] = ROBOT_MODES.get(int(values['robot_mode']), str(values['robot_mode']))
            except (KeyError, TypeError, ValueError) as exc:
                self._finish_dashboard_error(f'Dashboard response invalid: {exc}')
                return
            with self._lock:
                self._dashboard.update(values, available=True, checked_at=time.time(), error=None, stale=False)
                self._dashboard_in_flight = False
        else:
            self._next_dashboard_query()

    def _start_controller(self) -> None:
        with self._lock:
            self._controller_in_flight = True
            self._controller_deadline = time.monotonic() + REQUEST_TIMEOUT_S
        if not self._controller_client.service_is_ready():
            self._finish_controller_error('Controller-manager service unavailable')
            return
        try:
            future = self._controller_client.call_async(self._controller_service_type.Request())
            future.add_done_callback(self._controller_response)
        except Exception as exc:
            self._finish_controller_error(f'Controller-manager service failed: {exc}')

    def _controller_response(self, future: Any) -> None:
        try:
            response = future.result()
            if response is None:
                raise RuntimeError('empty response')
            states = {controller.name: controller.state for controller in response.controller}
            state = states.get(FORWARD_VELOCITY_CONTROLLER)
        except Exception as exc:
            self._finish_controller_error(f'Controller-manager service failed: {exc}')
            return
        with self._lock:
            if not self._controller_in_flight:
                return
            self._controller.update(
                available=True, loaded=state is not None, state=state,
                checked_at=time.time(), error=None, stale=False,
            )
            self._controller_in_flight = False

    def _set_dashboard_error(self, error: str) -> None:
        with self._lock:
            self._dashboard.update(available=False, checked_at=time.time(), error=error, stale=True)

    def _set_controller_error(self, error: str) -> None:
        with self._lock:
            self._controller.update(available=False, checked_at=time.time(), error=error, stale=True)

    def _finish_dashboard_error(self, error: str) -> None:
        self._set_dashboard_error(error)
        with self._lock:
            self._dashboard_in_flight = False

    def _finish_controller_error(self, error: str) -> None:
        self._set_controller_error(error)
        with self._lock:
            self._controller_in_flight = False


class UrDashboardInfo:
    """Compatibility view of the shared :class:`UrStatusMonitor` cache."""

    def __init__(self, monitor: UrStatusMonitor) -> None:
        self._monitor = monitor

    def snapshot(self) -> dict[str, Any]:
        return self._monitor.dashboard_snapshot()

    def request_refresh(self) -> None:
        self._monitor.request_refresh(dashboard=True, controller=False)


class ForwardVelocityControllerInfo:
    """Compatibility view of the shared :class:`UrStatusMonitor` cache."""

    def __init__(self, monitor: UrStatusMonitor) -> None:
        self._monitor = monitor

    def snapshot(self) -> dict[str, Any]:
        return self._monitor.controller_snapshot()

    def request_refresh(self) -> None:
        self._monitor.request_refresh(dashboard=False, controller=True)
