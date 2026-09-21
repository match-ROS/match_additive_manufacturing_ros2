#!/usr/bin/env python3
from typing import Optional

from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import (
    ConfigureController,
    ListControllers,
    LoadController,
    SwitchController,
)
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


class ControllerSwitchGuard(Node):

    def __init__(self) -> None:
        super().__init__('controller_switch_guard')
        self.declare_parameter('controller_manager', '/robot/arm/controller_manager')
        self.declare_parameter('activate_controller', 'forward_velocity_controller')
        self.declare_parameter('deactivate_controller', 'joint_trajectory_controller')
        self.declare_parameter('ready_topic', '/robot/arm/forward_velocity_controller_ready')
        self.declare_parameter('switch_on_start', True)
        self.declare_parameter('poll_period', 1.0)

        manager = str(self.get_parameter('controller_manager').value).rstrip('/')
        self.activate_controller = str(self.get_parameter('activate_controller').value)
        self.deactivate_controller = str(self.get_parameter('deactivate_controller').value)
        self.switch_on_start = bool(self.get_parameter('switch_on_start').value)
        self.list_client = self.create_client(ListControllers, f'{manager}/list_controllers')
        self.load_client = self.create_client(LoadController, f'{manager}/load_controller')
        self.configure_client = self.create_client(
            ConfigureController,
            f'{manager}/configure_controller',
        )
        self.switch_client = self.create_client(SwitchController, f'{manager}/switch_controller')
        self.pending: Optional[object] = None
        self.switch_requested = False
        self.ready = False
        # Startup is allowed to repair the controller configuration.  Once it
        # is confirmed, ordinary operation must not continuously exercise the
        # controller manager.  A manual GUI check remains available below.
        self._preparing = True
        self._check_only = False

        ready_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.ready_pub = self.create_publisher(
            Bool,
            str(self.get_parameter('ready_topic').value),
            ready_qos,
        )
        self.ready_pub.publish(Bool(data=False))
        self._poll_timer = self.create_timer(
            max(0.2, float(self.get_parameter('poll_period').value)), self._poll)
        self.create_service(Trigger, 'check', self._check_service)

    def _poll(self) -> None:
        """Prepare the controller only until the first successful confirmation."""
        if self.pending is not None:
            return
        if not self.list_client.service_is_ready():
            # During initial setup there is no prior confirmation.  Later a
            # temporarily unreachable manager means "unknown", not "false".
            if self._preparing:
                self._set_ready(False)
            return
        self.pending = self.list_client.call_async(ListControllers.Request())
        self.pending.add_done_callback(self._controllers_cb)

    def _check_service(self, _request, response):
        """Run one read-only controller check requested by the operator GUI."""
        if self.pending is not None:
            response.success = False
            response.message = 'Controller check already in progress'
            return response
        if not self.list_client.service_is_ready():
            response.success = False
            response.message = 'Controller manager is unavailable; previous confirmation is unchanged'
            return response
        self._check_only = True
        self.pending = self.list_client.call_async(ListControllers.Request())
        self.pending.add_done_callback(self._controllers_cb)
        response.success = True
        response.message = 'Controller check started'
        return response

    def _controllers_cb(self, future) -> None:
        self.pending = None
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Controller query failed: {exc}', throttle_duration_sec=2.0)
            self._set_ready(False)
            return

        states = {controller.name: controller.state for controller in response.controller}
        if states.get(self.activate_controller) == 'active':
            self._set_ready(True)
            if self._preparing:
                self._preparing = False
                self._poll_timer.cancel()
                self.get_logger().info(
                    f'{self.activate_controller} is active; controller-manager polling stopped.'
                )
            self._check_only = False
            return

        self._set_ready(False)
        if self._check_only:
            self._check_only = False
            self.get_logger().warn(
                f'{self.activate_controller} is not active.', throttle_duration_sec=2.0)
            return
        if (
            self.deactivate_controller
            and self.deactivate_controller not in states
        ):
            self.get_logger().warn(
                f'Waiting for driver controller {self.deactivate_controller}.',
                throttle_duration_sec=2.0,
            )
            return
        if self.activate_controller not in states:
            if self.load_client.service_is_ready():
                request = LoadController.Request()
                request.name = self.activate_controller
                self.pending = self.load_client.call_async(request)
                self.pending.add_done_callback(self._management_cb)
            return
        if states.get(self.activate_controller) == 'unconfigured':
            if self.configure_client.service_is_ready():
                request = ConfigureController.Request()
                request.name = self.activate_controller
                self.pending = self.configure_client.call_async(request)
                self.pending.add_done_callback(self._management_cb)
            return
        if (
            not self.switch_on_start
            or self.switch_requested
            or not self.switch_client.service_is_ready()
        ):
            return

        request = SwitchController.Request()
        request.activate_controllers = [self.activate_controller]
        request.deactivate_controllers = (
            [self.deactivate_controller]
            if states.get(self.deactivate_controller) == 'active'
            else []
        )
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = True
        request.timeout = Duration(sec=5)
        self.switch_requested = True
        self.pending = self.switch_client.call_async(request)
        self.pending.add_done_callback(self._switch_cb)

    def _switch_cb(self, future) -> None:
        self.pending = None
        self.switch_requested = False
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Controller switch failed: {exc}')
            return
        if not response.ok:
            self.get_logger().error(
                f'Failed to activate {self.activate_controller}; will retry after rechecking.'
            )

    def _management_cb(self, future) -> None:
        self.pending = None
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Controller management request failed: {exc}')
            return
        if not response.ok:
            self.get_logger().error(
                f'Could not prepare controller {self.activate_controller}; will retry.'
            )

    def _set_ready(self, ready: bool) -> None:
        self.ready = ready
        self.ready_pub.publish(Bool(data=ready))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControllerSwitchGuard()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
