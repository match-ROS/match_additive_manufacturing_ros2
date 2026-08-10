"""ROS 2 wrapper around the fail-closed process policy.

This node has no serial or Dynamixel dependency.  A later hardware adapter must
consume `/process/valve_target`; it must not bypass this policy.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32

from .policy import ProcessPolicy


class ProcessSafetyNode(Node):
    def __init__(self):
        super().__init__('process_safety_node')
        self._policy = ProcessPolicy(
            max_target=float(self.declare_parameter('max_target', 1.0).value),
            max_rate_per_second=float(self.declare_parameter('max_rate_per_second', 0.25).value),
            feedback_timeout=float(self.declare_parameter('feedback_timeout', 0.5).value),
        )
        self._last_step = self.get_clock().now().nanoseconds / 1e9
        self._target_sub = self.create_subscription(Float32, '/process/target', self._on_target, 10)
        self._feedback_sub = self.create_subscription(Float32, '/process/flow_measurement', self._on_feedback, 10)
        self._armed_sub = self.create_subscription(Bool, '/process/armed', self._on_armed, 10)
        self._ack_sub = self.create_subscription(Bool, '/process/acknowledged', self._on_acknowledged, 10)
        self._print_sub = self.create_subscription(Bool, '/start_condition', self._on_print_enabled, 10)
        self._output_pub = self.create_publisher(Float32, '/process/valve_target', 10)
        self.create_timer(0.05, self._publish)

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _on_target(self, message):
        try:
            self._policy.set_target(float(message.data))
        except ValueError as exc:
            self.get_logger().warning(f'rejected process target: {exc}')

    def _on_feedback(self, _message):
        self._policy.observe_feedback(self._now())

    def _on_armed(self, message):
        self._policy.armed = message.data

    def _on_acknowledged(self, message):
        self._policy.acknowledged = message.data

    def _on_print_enabled(self, message):
        self._policy.print_enabled = message.data

    def _publish(self):
        now = self._now()
        output = self._policy.step(now, now - self._last_step)
        self._last_step = now
        self._output_pub.publish(Float32(data=output))


def main(args=None):
    rclpy.init(args=args)
    node = ProcessSafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
