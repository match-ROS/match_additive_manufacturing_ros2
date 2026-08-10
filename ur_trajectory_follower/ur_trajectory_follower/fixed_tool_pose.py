"""Derive a nozzle-tip pose from a measured TCP pose and a fixed tool offset."""

from __future__ import annotations

import math

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from tf_transformations import quaternion_matrix, quaternion_multiply


def nozzle_pose_from_tcp(
    tcp_pose: PoseStamped,
    offset_xyz: tuple[float, float, float] | list[float],
    offset_quaternion_xyzw: tuple[float, float, float, float] | list[float],
) -> PoseStamped:
    """Compose the TCP-to-nozzle offset with a TCP pose in its parent frame."""
    if len(offset_xyz) != 3 or len(offset_quaternion_xyzw) != 4:
        raise ValueError('fixed tool offset must contain XYZ and XYZW values')

    tcp_q = tcp_pose.pose.orientation
    tcp_quaternion = (float(tcp_q.x), float(tcp_q.y), float(tcp_q.z), float(tcp_q.w))
    offset_quaternion = tuple(float(value) for value in offset_quaternion_xyzw)
    tcp_norm = math.sqrt(sum(value * value for value in tcp_quaternion))
    offset_norm = math.sqrt(sum(value * value for value in offset_quaternion))
    if tcp_norm < 1e-9:
        raise ValueError('TCP pose has an invalid orientation quaternion')
    if offset_norm < 1e-9:
        raise ValueError('fixed tool offset has an invalid orientation quaternion')
    tcp_quaternion = tuple(value / tcp_norm for value in tcp_quaternion)
    offset_quaternion = tuple(value / offset_norm for value in offset_quaternion)

    rotated_offset = quaternion_matrix(tcp_quaternion).dot(
        [float(offset_xyz[0]), float(offset_xyz[1]), float(offset_xyz[2]), 0.0])
    result = PoseStamped()
    result.header = tcp_pose.header
    result.pose.position.x = float(tcp_pose.pose.position.x) + float(rotated_offset[0])
    result.pose.position.y = float(tcp_pose.pose.position.y) + float(rotated_offset[1])
    result.pose.position.z = float(tcp_pose.pose.position.z) + float(rotated_offset[2])
    result_quaternion = quaternion_multiply(tcp_quaternion, offset_quaternion)
    result.pose.orientation.x = float(result_quaternion[0])
    result.pose.orientation.y = float(result_quaternion[1])
    result.pose.orientation.z = float(result_quaternion[2])
    result.pose.orientation.w = float(result_quaternion[3])
    return result


class FixedToolPose(Node):
    """Publish the nozzle-tip pose corresponding to an incoming TCP pose."""

    def __init__(self) -> None:
        super().__init__('fixed_tool_pose')
        self.declare_parameter('input_pose_topic', '/current_tcp_pose')
        self.declare_parameter('output_pose_topic', '/current_nozzle_tip_pose')
        self.declare_parameter('fixed_tool_offset_xyz', [0.0, 0.0, 0.0])
        self.declare_parameter('fixed_tool_offset_quaternion_xyzw', [0.0, 0.0, 0.0, 1.0])

        self.offset_xyz = list(self.get_parameter('fixed_tool_offset_xyz').value)
        self.offset_quaternion = list(
            self.get_parameter('fixed_tool_offset_quaternion_xyzw').value)
        if len(self.offset_xyz) != 3 or len(self.offset_quaternion) != 4:
            raise ValueError('fixed_tool_offset_xyz must have 3 values and quaternion_xyzw 4')
        self.pose_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter('output_pose_topic').value), 10)
        self.create_subscription(
            PoseStamped, str(self.get_parameter('input_pose_topic').value), self._tcp_cb, 10)

    def _tcp_cb(self, msg: PoseStamped) -> None:
        try:
            self.pose_pub.publish(nozzle_pose_from_tcp(
                msg, self.offset_xyz, self.offset_quaternion))
        except ValueError as exc:
            self.get_logger().warn(str(exc), throttle_duration_sec=2.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FixedToolPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
