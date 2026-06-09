#!/usr/bin/env python3
from typing import List

import numpy as np
import rclpy
from geometry_msgs.msg import Quaternion, Vector3
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from tf_transformations import quaternion_from_euler

from parse_paths.path_utils import as_bool, as_float_list, make_pose
from parse_paths.test_path_shapes import (
    generate_circle,
    generate_line,
    generate_rectangle,
    generate_waypoints,
    tangent_yaw,
)


class PathGeneratorNode(Node):
    def __init__(self) -> None:
        super().__init__('test_path_generator')
        self.declare_parameter('path_type', 'line')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('path_topic', '/test_path')
        self.declare_parameter('original_path_topic', '')
        self.declare_parameter('normal_topic', '')
        self.declare_parameter('normal_vector', [0.0, 0.0, 1.0])
        self.declare_parameter('num_points', 50)
        self.declare_parameter('time_step', 0.1)
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('publish_once', False)
        self.declare_parameter('orientation_mode', 'tangent')
        self.declare_parameter('fixed_yaw', 0.0)

        self.declare_parameter('line_start', [0.0, 0.0, 0.0])
        self.declare_parameter('line_end', [1.0, 0.0, 0.0])
        self.declare_parameter('rectangle_center', [0.0, 0.0, 0.0])
        self.declare_parameter('rectangle_width', 1.0)
        self.declare_parameter('rectangle_height', 1.0)
        self.declare_parameter('circle_center', [0.0, 0.0, 0.0])
        self.declare_parameter('circle_radius', 0.5)
        self.declare_parameter('closed_path', True)
        self.declare_parameter('waypoints', [0.0, 0.0, 0.0, 1.0, 0.0, 0.0])
        self.declare_parameter('interpolate_waypoints', True)

        self.frame_id = str(self.get_parameter('frame_id').value)
        self.path_topic = str(self.get_parameter('path_topic').value)
        self.original_path_topic = str(self.get_parameter('original_path_topic').value).strip()
        self.normal_topic = str(self.get_parameter('normal_topic').value).strip()
        self.num_points = max(2, int(self.get_parameter('num_points').value))
        self.time_step = float(self.get_parameter('time_step').value)
        self.publish_once = as_bool(self.get_parameter('publish_once').value)

        latch_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.path_pub = self.create_publisher(Path, self.path_topic, latch_qos)
        self.original_pub = (
            self.create_publisher(Path, self.original_path_topic, latch_qos)
            if self.original_path_topic else None
        )
        self.normal_pub = (
            self.create_publisher(Vector3, self.normal_topic, latch_qos)
            if self.normal_topic else None
        )

        self.path_msg = self._build_path()
        self.normal_msg = self._build_normal()
        self.has_published_once = False
        rate = max(0.1, float(self.get_parameter('publish_rate').value))
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"Publishing {len(self.path_msg.poses)} point {self.get_parameter('path_type').value} "
            f"test path on {self.path_topic}."
        )

    def _build_path(self) -> Path:
        points = self._generate_points()
        orientation_mode = str(self.get_parameter('orientation_mode').value).strip().lower()
        fixed_yaw = float(self.get_parameter('fixed_yaw').value)
        path_msg = Path()
        path_msg.header.frame_id = self.frame_id
        start_time = self.get_clock().now()

        for index, point in enumerate(points):
            yaw = tangent_yaw(points, index, fixed_yaw) if orientation_mode == 'tangent' else fixed_yaw
            orientation = self._orientation_from_yaw(yaw)
            stamp = (start_time + Duration(seconds=self.time_step * index)).to_msg()
            path_msg.poses.append(make_pose(self.frame_id, stamp, point, orientation))
        return path_msg

    def _generate_points(self) -> List[np.ndarray]:
        path_type = str(self.get_parameter('path_type').value).strip().lower()
        if path_type == 'line':
            return generate_line(
                as_float_list(self.get_parameter('line_start').value, [0.0, 0.0, 0.0]),
                as_float_list(self.get_parameter('line_end').value, [1.0, 0.0, 0.0]),
                self.num_points,
            )
        if path_type == 'rectangle':
            return generate_rectangle(
                as_float_list(self.get_parameter('rectangle_center').value, [0.0, 0.0, 0.0]),
                float(self.get_parameter('rectangle_width').value),
                float(self.get_parameter('rectangle_height').value),
                self.num_points,
                as_bool(self.get_parameter('closed_path').value),
            )
        if path_type == 'circle':
            return generate_circle(
                as_float_list(self.get_parameter('circle_center').value, [0.0, 0.0, 0.0]),
                float(self.get_parameter('circle_radius').value),
                self.num_points,
                as_bool(self.get_parameter('closed_path').value),
            )
        if path_type == 'waypoints':
            return generate_waypoints(
                as_float_list(self.get_parameter('waypoints').value, [0.0, 0.0, 0.0, 1.0, 0.0, 0.0]),
                self.num_points,
                as_bool(self.get_parameter('interpolate_waypoints').value),
            )

        self.get_logger().warn(f"Unknown path_type '{path_type}', falling back to line.")
        return generate_line([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], self.num_points)

    def _build_normal(self) -> Vector3:
        normal = as_float_list(self.get_parameter('normal_vector').value, [0.0, 0.0, 1.0])
        normal = (normal + [0.0, 0.0, 1.0])[:3]
        return Vector3(x=float(normal[0]), y=float(normal[1]), z=float(normal[2]))

    def _tick(self) -> None:
        if self.publish_once and self.has_published_once:
            return
        stamp = self.get_clock().now().to_msg()
        self.path_msg.header.stamp = stamp
        self.path_pub.publish(self.path_msg)
        if self.original_pub is not None:
            self.original_pub.publish(self.path_msg)
        if self.normal_pub is not None:
            self.normal_pub.publish(self.normal_msg)
        self.has_published_once = True

    @staticmethod
    def _orientation_from_yaw(yaw: float) -> Quaternion:
        quat = quaternion_from_euler(0.0, 0.0, yaw)
        return Quaternion(x=float(quat[0]), y=float(quat[1]), z=float(quat[2]), w=float(quat[3]))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PathGeneratorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
