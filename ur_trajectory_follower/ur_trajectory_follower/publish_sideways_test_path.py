#!/usr/bin/env python3
from typing import Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Vector3
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool
from tf_transformations import quaternion_from_matrix

from ur_trajectory_follower.ros2_utils import as_bool, as_float_list


def _normalize(vec: np.ndarray, fallback: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(vec)
    if norm < 1e-6:
        return fallback
    return vec / norm


def _build_orientation(nozzle_axis: np.ndarray, x_axis_hint: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    z_axis = _normalize(nozzle_axis, np.array([0.0, 0.0, 1.0]))
    ref_axis = _normalize(x_axis_hint, np.array([1.0, 0.0, 0.0]))
    if abs(float(np.dot(ref_axis, z_axis))) > 0.95:
        ref_axis = np.array([1.0, 0.0, 0.0]) if abs(z_axis[0]) < 0.95 else np.array([0.0, 1.0, 0.0])

    x_axis = _normalize(np.cross(ref_axis, z_axis), np.array([1.0, 0.0, 0.0]))
    y_axis = _normalize(np.cross(z_axis, x_axis), np.array([0.0, 1.0, 0.0]))
    x_axis = _normalize(np.cross(y_axis, z_axis), np.array([1.0, 0.0, 0.0]))

    rotation = np.eye(4)
    rotation[0:3, 0] = x_axis
    rotation[0:3, 1] = y_axis
    rotation[0:3, 2] = z_axis
    return quaternion_from_matrix(rotation), z_axis


class SidewaysTestPathPublisher(Node):
    def __init__(self) -> None:
        super().__init__('ur_sideways_test_path_publisher')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('path_topic', '/ur_path_transformed')
        self.declare_parameter('original_path_topic', '/ur_path_original')
        self.declare_parameter('normal_topic', '/normal_vector')
        self.declare_parameter('use_current_pose', True)
        self.declare_parameter('current_pose_topic', '/current_tcp_pose')
        self.declare_parameter('wait_for_home_pose', False)
        self.declare_parameter('home_pose_ready_topic', '/home_pose_ready')
        self.declare_parameter('start_offset', [0.0, 0.0, 0.0])
        self.declare_parameter('start_xyz', [0.6, 0.0, 0.45])
        self.declare_parameter('direction', [1.0, 0.0, 0.0])
        self.declare_parameter('nozzle_axis', [0.0, 1.0, 0.0])
        self.declare_parameter('x_axis_hint', [1.0, 0.0, 0.0])
        self.declare_parameter('path_length', 0.3)
        self.declare_parameter('num_points', 50)
        self.declare_parameter('time_step', 0.1)
        self.declare_parameter('publish_rate', 1.0)

        self.frame_id = str(self.get_parameter('frame_id').value)
        self.path_topic = str(self.get_parameter('path_topic').value)
        self.original_path_topic = str(self.get_parameter('original_path_topic').value)
        self.normal_topic = str(self.get_parameter('normal_topic').value)
        self.use_current_pose = as_bool(self.get_parameter('use_current_pose').value)
        self.wait_for_home_pose = as_bool(self.get_parameter('wait_for_home_pose').value)
        self.current_pose_topic = str(self.get_parameter('current_pose_topic').value)
        self.home_pose_ready_topic = str(self.get_parameter('home_pose_ready_topic').value)
        self.start_offset = np.array(as_float_list(self.get_parameter('start_offset').value, [0.0, 0.0, 0.0]), dtype=float)
        self.start_xyz = np.array(as_float_list(self.get_parameter('start_xyz').value, [0.6, 0.0, 0.45]), dtype=float)
        self.direction = np.array(as_float_list(self.get_parameter('direction').value, [1.0, 0.0, 0.0]), dtype=float)
        self.nozzle_axis = np.array(as_float_list(self.get_parameter('nozzle_axis').value, [0.0, 1.0, 0.0]), dtype=float)
        self.x_axis_hint = np.array(as_float_list(self.get_parameter('x_axis_hint').value, [1.0, 0.0, 0.0]), dtype=float)
        self.path_length = float(self.get_parameter('path_length').value)
        self.num_points = max(2, int(self.get_parameter('num_points').value))
        self.time_step = float(self.get_parameter('time_step').value)

        latch_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, reliability=QoSReliabilityPolicy.RELIABLE)
        self.path_pub = self.create_publisher(Path, self.path_topic, latch_qos)
        self.original_pub = self.create_publisher(Path, self.original_path_topic, latch_qos)
        self.normal_pub = self.create_publisher(Vector3, self.normal_topic, latch_qos)
        self.current_pose: Optional[PoseStamped] = None
        self.home_ready = not self.wait_for_home_pose
        self.path_msg: Optional[Path] = None
        self.normal_msg = Vector3()

        if self.use_current_pose:
            self.create_subscription(PoseStamped, self.current_pose_topic, self._current_pose_cb, 10)
        if self.wait_for_home_pose:
            self.create_subscription(Bool, self.home_pose_ready_topic, self._home_pose_cb, 10)

        rate = max(0.1, float(self.get_parameter('publish_rate').value))
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info("Sideways test path publisher waiting for start pose inputs.")

    def _current_pose_cb(self, msg: PoseStamped) -> None:
        self.current_pose = msg
        self._ensure_path()

    def _home_pose_cb(self, msg: Bool) -> None:
        if msg.data:
            self.home_ready = True
            self._ensure_path()

    def _ensure_path(self) -> None:
        if self.path_msg is not None or not self.home_ready:
            return
        if self.use_current_pose and self.current_pose is None:
            return

        if self.use_current_pose:
            pos = self.current_pose.pose.position
            start = np.array([pos.x, pos.y, pos.z], dtype=float)
        else:
            start = self.start_xyz
        self.path_msg, self.normal_msg = self._build_messages(start + self.start_offset)
        self.get_logger().info(
            f"Publishing sideways path with {self.num_points} points on {self.path_topic}."
        )

    def _build_messages(self, start_point: np.ndarray) -> Tuple[Path, Vector3]:
        direction = _normalize(self.direction, np.array([1.0, 0.0, 0.0]))
        step = self.path_length / max(self.num_points - 1, 1)
        quat, nozzle_axis = _build_orientation(self.nozzle_axis, self.x_axis_hint)
        path_msg = Path()
        path_msg.header.frame_id = self.frame_id
        start_time = self.get_clock().now()

        for i in range(self.num_points):
            pose = PoseStamped()
            pose.header.frame_id = self.frame_id
            pose.header.stamp = (start_time + Duration(seconds=self.time_step * i)).to_msg()
            position = start_point + direction * (step * i)
            pose.pose.position.x = float(position[0])
            pose.pose.position.y = float(position[1])
            pose.pose.position.z = float(position[2])
            pose.pose.orientation.x = float(quat[0])
            pose.pose.orientation.y = float(quat[1])
            pose.pose.orientation.z = float(quat[2])
            pose.pose.orientation.w = float(quat[3])
            path_msg.poses.append(pose)

        normal = Vector3(x=float(nozzle_axis[0]), y=float(nozzle_axis[1]), z=float(nozzle_axis[2]))
        return path_msg, normal

    def _tick(self) -> None:
        self._ensure_path()
        if self.path_msg is None:
            return
        stamp = self.get_clock().now().to_msg()
        self.path_msg.header.stamp = stamp
        self.path_pub.publish(self.path_msg)
        self.original_pub.publish(self.path_msg)
        self.normal_pub.publish(self.normal_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SidewaysTestPathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
