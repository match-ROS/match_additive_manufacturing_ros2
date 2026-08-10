"""ROS 2 serial bridge for the deployed Arduino foam-flow CSV stream."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from .flow_protocol import ExponentialFlowFilter, parse_flow_line


class FlowSerialBridge(Node):
    def __init__(self) -> None:
        super().__init__('flow_serial_bridge')
        for name, default in (
            ('port', ''), ('baud', 115200), ('timeout', 0.1), ('line_prefix', 'FLOW,'),
            ('left_channel', 1), ('right_channel', 2), ('safety_channel', 1), ('filter_alpha', 0.25),
            ('left_topic', '/process/flow_left'), ('right_topic', '/process/flow_right'),
            ('flow_measurement_topic', '/process/flow_measurement'),
        ):
            self.declare_parameter(name, default)
        self._serial = None
        self._filters = {}
        self._left_pub = self.create_publisher(Float32, str(self.get_parameter('left_topic').value), 10)
        self._right_pub = self.create_publisher(Float32, str(self.get_parameter('right_topic').value), 10)
        self._safety_pub = self.create_publisher(Float32, str(self.get_parameter('flow_measurement_topic').value), 10)
        self.create_timer(0.02, self._tick)

    def _connect(self) -> bool:
        try:
            from serial import Serial
        except ImportError:
            self.get_logger().error('pyserial is required for flow_serial_bridge')
            return False
        port = str(self.get_parameter('port').value)
        if not port:
            self.get_logger().warning('flow serial bridge requires a port parameter', throttle_duration_sec=5.0)
            return False
        try:
            self._serial = Serial(port, int(self.get_parameter('baud').value), timeout=float(self.get_parameter('timeout').value))
            return True
        except Exception as error:
            self.get_logger().warning(f'flow serial open failed: {error}', throttle_duration_sec=5.0)
            return False

    def _tick(self) -> None:
        if self._serial is None and not self._connect():
            return
        try:
            line = self._serial.readline().decode('ascii', errors='strict')
            if not line.strip():
                return
            sample = parse_flow_line(line, str(self.get_parameter('line_prefix').value))
        except (UnicodeError, ValueError) as error:
            self.get_logger().warning(f'ignored flow line: {error}', throttle_duration_sec=2.0)
            return
        except Exception as error:
            self.get_logger().warning(f'flow serial read failed: {error}')
            try:
                self._serial.close()
            finally:
                self._serial = None
            return
        filter_ = self._filters.setdefault(sample.channel, ExponentialFlowFilter(float(self.get_parameter('filter_alpha').value)))
        value = filter_.update(sample.percent)
        message = Float32(data=value)
        if sample.channel == int(self.get_parameter('left_channel').value):
            self._left_pub.publish(message)
        if sample.channel == int(self.get_parameter('right_channel').value):
            self._right_pub.publish(message)
        if sample.channel == int(self.get_parameter('safety_channel').value):
            self._safety_pub.publish(message)

    def destroy_node(self):
        if self._serial is not None:
            self._serial.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FlowSerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
