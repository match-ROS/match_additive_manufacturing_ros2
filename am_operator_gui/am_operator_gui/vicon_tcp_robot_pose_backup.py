#!/usr/bin/env python3
from __future__ import annotations

import math
from typing import Optional

from geometry_msgs.msg import PoseStamped, TransformStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener
from tf_transformations import (
    concatenate_matrices,
    inverse_matrix,
    quaternion_from_matrix,
    quaternion_matrix,
    translation_matrix,
)


def _clean_frame(frame: str) -> str:
    return str(frame).strip().lstrip("/")


def _normalized_quaternion_xyzw(x: float, y: float, z: float, w: float) -> tuple[float, float, float, float]:
    values = (float(x), float(y), float(z), float(w))
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f"quaternion contains non-finite values: {values}")
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 1e-9:
        raise ValueError(f"quaternion norm is too small: {values}")
    return tuple(value / norm for value in values)


class ViconTcpRobotPoseBackup(Node):
    """Infer /robot_pose from a Vicon TCP pose when the base marker is unavailable."""

    def __init__(self, **kwargs) -> None:
        super().__init__("vicon_tcp_robot_pose_backup", **kwargs)
        self.declare_parameter("input_topic", "/vicon/tool_transformed")
        self.declare_parameter("output_topic", "/robot_pose")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_base_frame", "base_link")
        self.declare_parameter("robot_tcp_frame", "robot_arm_nozzle_tip")
        self.declare_parameter("robot_tree_root_frame", "odom")
        self.declare_parameter("ready_topic", "~/ready")
        self.declare_parameter("stale_timeout", 0.5)
        self.declare_parameter("publish_direct_base_tf_when_root_missing", False)

        self.map_frame = _clean_frame(str(self.get_parameter("map_frame").value))
        self.robot_base_frame = _clean_frame(
            str(self.get_parameter("robot_base_frame").value)
        )
        self.robot_tcp_frame = _clean_frame(str(self.get_parameter("robot_tcp_frame").value))
        self.robot_tree_root_frame = _clean_frame(
            str(self.get_parameter("robot_tree_root_frame").value)
        )
        self.stale_timeout = max(0.05, float(self.get_parameter("stale_timeout").value))
        self.publish_direct_base_tf_when_root_missing = bool(
            self.get_parameter("publish_direct_base_tf_when_root_missing").value
        )

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.last_output_time: Optional[rclpy.time.Time] = None
        self.ready = False

        ready_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.pose_pub = self.create_publisher(
            PoseStamped,
            str(self.get_parameter("output_topic").value),
            10,
        )
        self.ready_pub = self.create_publisher(
            Bool,
            str(self.get_parameter("ready_topic").value),
            ready_qos,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("input_topic").value),
            self._tcp_pose_cb,
            10,
        )
        self.create_timer(min(0.1, self.stale_timeout / 2.0), self._check_stale)
        self._set_ready(False)

        self.get_logger().info(
            "Publishing backup robot pose from Vicon TCP: "
            f"{self.get_parameter('input_topic').value} -> "
            f"{self.get_parameter('output_topic').value}; "
            f"base={self.robot_base_frame}, tcp={self.robot_tcp_frame}, "
            f"map={self.map_frame}."
        )

    def _tcp_pose_cb(self, msg: PoseStamped) -> None:
        source_frame = _clean_frame(msg.header.frame_id)
        if not source_frame:
            self.get_logger().warn(
                "Ignoring Vicon TCP pose with an empty frame_id.",
                throttle_duration_sec=2.0,
            )
            self._set_ready(False)
            return

        try:
            map_tcp = self._pose_in_map(msg, source_frame)
            map_base = self._tcp_pose_to_base_pose(map_tcp)
            self._publish_map_to_robot_tree(map_base)
            self.pose_pub.publish(map_base)
        except (TransformException, ValueError) as exc:
            self.get_logger().warn(
                f"Waiting for TF needed by Vicon TCP robot pose backup: {exc}",
                throttle_duration_sec=2.0,
            )
            self._set_ready(False)
            return

        self.last_output_time = self.get_clock().now()
        self._set_ready(True)

    def _pose_in_map(self, msg: PoseStamped, source_frame: str) -> PoseStamped:
        if source_frame == self.map_frame:
            map_tcp = PoseStamped()
            map_tcp.pose = msg.pose
        else:
            transform = self.buffer.lookup_transform(
                self.map_frame,
                source_frame,
                rclpy.time.Time(),
            )
            map_tcp = PoseStamped()
            map_tcp.pose = do_transform_pose(msg.pose, transform)
        map_tcp.header.frame_id = self.map_frame
        map_tcp.header.stamp = self.get_clock().now().to_msg()
        return map_tcp

    def _tcp_pose_to_base_pose(self, map_tcp: PoseStamped) -> PoseStamped:
        base_to_tcp = self.buffer.lookup_transform(
            self.robot_base_frame,
            self.robot_tcp_frame,
            rclpy.time.Time(),
        )
        map_base_matrix = concatenate_matrices(
            self._pose_to_matrix(map_tcp),
            inverse_matrix(self._transform_to_matrix(base_to_tcp)),
        )
        map_base = PoseStamped()
        map_base.header = map_tcp.header
        map_base.pose.position.x = float(map_base_matrix[0, 3])
        map_base.pose.position.y = float(map_base_matrix[1, 3])
        map_base.pose.position.z = float(map_base_matrix[2, 3])
        qx, qy, qz, qw = _normalized_quaternion_xyzw(
            *quaternion_from_matrix(map_base_matrix)
        )
        map_base.pose.orientation.x = float(qx)
        map_base.pose.orientation.y = float(qy)
        map_base.pose.orientation.z = float(qz)
        map_base.pose.orientation.w = float(qw)
        return map_base

    def _publish_map_to_robot_tree(self, map_base: PoseStamped) -> None:
        if self.robot_tree_root_frame and self.robot_tree_root_frame != self.robot_base_frame:
            try:
                root_to_base = self.buffer.lookup_transform(
                    self.robot_tree_root_frame,
                    self.robot_base_frame,
                    rclpy.time.Time(),
                )
            except TransformException:
                if not self.publish_direct_base_tf_when_root_missing:
                    raise
            else:
                map_to_base = self._pose_to_matrix(map_base)
                root_to_base_matrix = self._transform_to_matrix(root_to_base)
                map_to_root = concatenate_matrices(
                    map_to_base,
                    inverse_matrix(root_to_base_matrix),
                )
                self.broadcaster.sendTransform(
                    self._transform_from_matrix(
                        map_base.header,
                        self.robot_tree_root_frame,
                        map_to_root,
                    )
                )
                return

        self.broadcaster.sendTransform(
            self._transform_from_matrix(
                map_base.header,
                self.robot_base_frame,
                self._pose_to_matrix(map_base),
            )
        )

    @staticmethod
    def _pose_to_matrix(pose: PoseStamped):
        position = pose.pose.position
        orientation = pose.pose.orientation
        qx, qy, qz, qw = _normalized_quaternion_xyzw(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        return concatenate_matrices(
            translation_matrix((position.x, position.y, position.z)),
            quaternion_matrix((qx, qy, qz, qw)),
        )

    @staticmethod
    def _transform_to_matrix(transform: TransformStamped):
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        qx, qy, qz, qw = _normalized_quaternion_xyzw(
            rotation.x,
            rotation.y,
            rotation.z,
            rotation.w,
        )
        return concatenate_matrices(
            translation_matrix((translation.x, translation.y, translation.z)),
            quaternion_matrix((qx, qy, qz, qw)),
        )

    @staticmethod
    def _transform_from_matrix(header, child_frame: str, matrix) -> TransformStamped:
        qx, qy, qz, qw = _normalized_quaternion_xyzw(*quaternion_from_matrix(matrix))
        transform = TransformStamped()
        transform.header = header
        transform.child_frame_id = child_frame
        transform.transform.translation.x = float(matrix[0, 3])
        transform.transform.translation.y = float(matrix[1, 3])
        transform.transform.translation.z = float(matrix[2, 3])
        transform.transform.rotation.x = float(qx)
        transform.transform.rotation.y = float(qy)
        transform.transform.rotation.z = float(qz)
        transform.transform.rotation.w = float(qw)
        return transform

    def _check_stale(self) -> None:
        if self.last_output_time is None:
            self._set_ready(False)
            return
        if (self.get_clock().now() - self.last_output_time).nanoseconds / 1e9 > self.stale_timeout:
            self._set_ready(False)

    def _set_ready(self, ready: bool) -> None:
        if ready == self.ready and self.last_output_time is not None:
            return
        self.ready = ready
        self.ready_pub.publish(Bool(data=ready))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ViconTcpRobotPoseBackup()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
