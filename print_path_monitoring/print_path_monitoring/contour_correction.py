"""Fail-closed bounded contour-error to Cartesian-twist policy."""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float32


def bounded_correction(lateral_error, height_error, *, enabled, started,
                       lateral_gain, height_gain, max_lateral, max_height):
    """Return lateral/height corrections, or zero unless every gate is valid."""
    values = (lateral_error, height_error, lateral_gain, height_gain, max_lateral, max_height)
    if not enabled or not started or not all(math.isfinite(float(value)) for value in values):
        return 0.0, 0.0
    if min(lateral_gain, height_gain, max_lateral, max_height) < 0.0:
        return 0.0, 0.0
    lateral = max(-max_lateral, min(max_lateral, -lateral_gain * lateral_error))
    height = max(-max_height, min(max_height, -height_gain * height_error))
    return lateral, height


class ContourCorrection(Node):
    def __init__(self):
        super().__init__('contour_correction')
        for name, default in (
            ('enabled', False), ('lateral_error_topic', '/contour/lateral_error'),
            ('height_error_topic', '/contour/height_error'), ('start_condition_topic', '/start_condition'),
            ('output_topic', '/contour/twist_world'), ('max_input_age', 0.25),
            ('lateral_gain', 1.0), ('height_gain', 1.0),
            ('max_lateral_velocity', 0.01), ('max_height_velocity', 0.01),
            ('publish_rate', 50.0),
        ):
            self.declare_parameter(name, default)
        self._lateral = self._height = None
        self._lateral_time = self._height_time = None
        self._started = False
        self._pub = self.create_publisher(Twist, str(self.get_parameter('output_topic').value), 10)
        self.create_subscription(Float32, str(self.get_parameter('lateral_error_topic').value), self._lateral_cb, 10)
        self.create_subscription(Float32, str(self.get_parameter('height_error_topic').value), self._height_cb, 10)
        self.create_subscription(Bool, str(self.get_parameter('start_condition_topic').value), self._start_cb, 10)
        self.create_timer(1.0 / max(1.0, float(self.get_parameter('publish_rate').value)), self._publish)

    def _lateral_cb(self, msg):
        self._lateral, self._lateral_time = float(msg.data), time.monotonic()

    def _height_cb(self, msg):
        self._height, self._height_time = float(msg.data), time.monotonic()

    def _start_cb(self, msg):
        self._started = bool(msg.data)

    def _publish(self):
        now, age = time.monotonic(), float(self.get_parameter('max_input_age').value)
        fresh = self._lateral_time is not None and self._height_time is not None and now - self._lateral_time <= age and now - self._height_time <= age
        lateral, height = bounded_correction(
            self._lateral, self._height, enabled=bool(self.get_parameter('enabled').value) and fresh,
            started=self._started, lateral_gain=float(self.get_parameter('lateral_gain').value),
            height_gain=float(self.get_parameter('height_gain').value),
            max_lateral=float(self.get_parameter('max_lateral_velocity').value),
            max_height=float(self.get_parameter('max_height_velocity').value))
        message = Twist()
        message.linear.y, message.linear.z = lateral, height
        self._pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = ContourCorrection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
