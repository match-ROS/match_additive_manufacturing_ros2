"""Monitoring-only comparison of Keyence profiles against a recorded reference."""

import json
import math
import struct
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32

from .contour_metrics import estimate_contour_error


class ContourProfileMonitor(Node):
    def __init__(self) -> None:
        super().__init__('contour_profile_monitor')
        self.declare_parameter('profiles_topic', '/profiles')
        self.declare_parameter('reference_profile_file', '')
        self.declare_parameter('max_shift_samples', 20)
        self.declare_parameter('min_overlap', 12)
        self.declare_parameter('max_profile_age', 0.5)
        self.declare_parameter('lateral_error_topic', '/contour/lateral_error')
        self.declare_parameter('height_error_topic', '/contour/height_error')
        path = str(self.get_parameter('reference_profile_file').value)
        self._reference_z, self._reference_pitch = self._load_reference(path)
        self._last_profile_time = None
        self._last_error = 'waiting for profile'
        self._lateral_pub = self.create_publisher(Float32, str(self.get_parameter('lateral_error_topic').value), 10)
        self._height_pub = self.create_publisher(Float32, str(self.get_parameter('height_error_topic').value), 10)
        self._diagnostics = self.create_publisher(DiagnosticArray, '~/diagnostics', 10)
        self.create_subscription(PointCloud2, str(self.get_parameter('profiles_topic').value), self._profile_callback, 10)
        self.create_timer(0.2, self._publish_diagnostics)

    @staticmethod
    def _load_reference(path: str):
        if not path:
            raise ValueError('reference_profile_file is required')
        record = json.loads(Path(path).read_text())
        pitch = float(record['x_pitch_m'])
        heights = tuple(math.nan if value is None else float(value) for value in record['z_m'])
        if not heights or pitch <= 0.0:
            raise ValueError('reference profile requires non-empty z_m and positive x_pitch_m')
        return heights, pitch

    @staticmethod
    def _z_values(message: PointCloud2):
        z_field = next((field for field in message.fields if field.name == 'z'), None)
        x_field = next((field for field in message.fields if field.name == 'x'), None)
        if z_field is None or x_field is None or z_field.datatype != 7 or x_field.datatype != 7:
            raise ValueError('profile requires float32 x and z PointCloud2 fields')
        if message.height != 1 or message.point_step < max(x_field.offset, z_field.offset) + 4:
            raise ValueError('profile PointCloud2 layout is invalid')
        count = message.width
        xs = [struct.unpack_from('<f', message.data, index * message.point_step + x_field.offset)[0] for index in range(count)]
        zs = [struct.unpack_from('<f', message.data, index * message.point_step + z_field.offset)[0] for index in range(count)]
        if len(xs) < 2:
            raise ValueError('profile needs at least two points')
        pitch = statistics_median([xs[index + 1] - xs[index] for index in range(len(xs) - 1)])
        return tuple(zs), pitch

    def _profile_callback(self, message: PointCloud2) -> None:
        try:
            observed_z, pitch = self._z_values(message)
            if abs(pitch - self._reference_pitch) > max(1e-9, self._reference_pitch * 0.01):
                raise ValueError('observed profile pitch differs from reference by more than 1%')
            error = estimate_contour_error(
                self._reference_z, observed_z, self._reference_pitch,
                int(self.get_parameter('max_shift_samples').value), int(self.get_parameter('min_overlap').value))
            if not error.valid:
                raise ValueError(error.reason)
        except (ValueError, struct.error) as exc:
            self._last_error = str(exc)
            return
        self._lateral_pub.publish(Float32(data=error.lateral_error_m))
        self._height_pub.publish(Float32(data=error.height_error_m))
        self._last_profile_time = self.get_clock().now()
        self._last_error = f'valid overlap={error.overlap}'

    def _publish_diagnostics(self) -> None:
        status = DiagnosticStatus(name='contour profile monitor', hardware_id='keyence')
        if self._last_profile_time is None:
            age, status.level, status.message = float('inf'), DiagnosticStatus.WARN, self._last_error
        else:
            age = (self.get_clock().now() - self._last_profile_time).nanoseconds / 1e9
            fresh = age <= float(self.get_parameter('max_profile_age').value)
            status.level = DiagnosticStatus.OK if fresh else DiagnosticStatus.WARN
            status.message = self._last_error if fresh else f'stale profile: {self._last_error}'
        status.values = [KeyValue(key='age_sec', value=str(age))]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self._diagnostics.publish(array)


def statistics_median(values):
    values = sorted(values)
    return values[len(values) // 2] if len(values) % 2 else (values[len(values) // 2 - 1] + values[len(values) // 2]) / 2.0


def main(args=None):
    rclpy.init(args=args)
    node = ContourProfileMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
