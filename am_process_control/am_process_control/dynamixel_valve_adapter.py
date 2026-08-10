"""Publish guarded valve goals using the ROS 1 Dynamixel topic contract."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int16

from .flow_protocol import ValveMapper
from .valve_adapter import ValveCommandGate


class DynamixelValveAdapter(Node):
    def __init__(self) -> None:
        super().__init__('dynamixel_valve_adapter')
        for name, default in (
            ('input_topic', '/process/valve_target'), ('left_topic', 'servo_target_pos_left'),
            ('right_topic', 'servo_target_pos_right'), ('enable_output', False), ('target_timeout', 0.25),
            ('left_closed_position', 0), ('left_open_position', 1023),
            ('right_closed_position', 0), ('right_open_position', 1023), ('publish_rate', 20.0),
        ):
            self.declare_parameter(name, default)
        self._gate = ValveCommandGate(
            left_mapper=ValveMapper(int(self.get_parameter('left_closed_position').value), int(self.get_parameter('left_open_position').value)),
            right_mapper=ValveMapper(int(self.get_parameter('right_closed_position').value), int(self.get_parameter('right_open_position').value)),
            timeout=float(self.get_parameter('target_timeout').value),
            enabled=bool(self.get_parameter('enable_output').value),
        )
        self._left_pub = self.create_publisher(Int16, str(self.get_parameter('left_topic').value), 10)
        self._right_pub = self.create_publisher(Int16, str(self.get_parameter('right_topic').value), 10)
        self.create_subscription(Float32, str(self.get_parameter('input_topic').value), self._target_callback, 10)
        self.create_timer(1.0 / max(1.0, float(self.get_parameter('publish_rate').value)), self._publish)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _target_callback(self, message: Float32) -> None:
        try:
            self._gate.observe_target(float(message.data), self._now())
        except ValueError as error:
            self.get_logger().warning(f'rejected valve target: {error}')

    def _publish(self) -> None:
        left, right = self._gate.command(self._now())
        self._left_pub.publish(Int16(data=left))
        self._right_pub.publish(Int16(data=right))


def main(args=None):
    rclpy.init(args=args)
    node = DynamixelValveAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
