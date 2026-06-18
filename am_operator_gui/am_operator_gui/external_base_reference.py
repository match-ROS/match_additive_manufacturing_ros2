#!/usr/bin/env python3
from __future__ import annotations

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
    return str(frame).strip().lstrip('/')


class ExternalBaseReference(Node):

    def __init__(self, **kwargs) -> None:
        super().__init__('external_base_reference', **kwargs)
        self.declare_parameter('input_topic', '/vicon/Base_RB/Base_RB')
        self.declare_parameter('input_pose_frame', '')
        self.declare_parameter('output_topic', '/robot_pose')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('robot_tree_root_frame', 'odom')
        self.declare_parameter('ready_topic', '~/ready')
        self.declare_parameter('stale_timeout', 0.5)
        self.declare_parameter('publish_direct_base_tf_when_root_missing', False)

        self.map_frame = _clean_frame(str(self.get_parameter('map_frame').value))
        self.input_pose_frame = _clean_frame(str(self.get_parameter('input_pose_frame').value))
        self.robot_base_frame = _clean_frame(str(self.get_parameter('robot_base_frame').value))
        self.robot_tree_root_frame = _clean_frame(
            str(self.get_parameter('robot_tree_root_frame').value)
        )
        self.stale_timeout = max(0.05, float(self.get_parameter('stale_timeout').value))
        self.publish_direct_base_tf_when_root_missing = bool(
            self.get_parameter('publish_direct_base_tf_when_root_missing').value
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
            str(self.get_parameter('output_topic').value),
            10,
        )
        self.ready_pub = self.create_publisher(
            Bool,
            str(self.get_parameter('ready_topic').value),
            ready_qos,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter('input_topic').value),
            self._pose_cb,
            10,
        )
        self.create_timer(min(0.1, self.stale_timeout / 2.0), self._check_stale)
        self._set_ready(False)

    def _pose_cb(self, msg: PoseStamped) -> None:
        source_frame = _clean_frame(msg.header.frame_id)
        if not source_frame:
            self.get_logger().warn(
                'Ignoring external base pose with an empty frame_id.',
                throttle_duration_sec=2.0,
            )
            self._set_ready(False)
            return

        try:
            map_base = self._pose_in_map(msg, source_frame)
            if self.input_pose_frame:
                map_base = self._reference_pose_to_base_pose(map_base)
            self._publish_map_to_robot_tree(map_base)
            self.pose_pub.publish(map_base)
        except TransformException as exc:
            self.get_logger().warn(
                f'Waiting for TF needed by external base reference: {exc}',
                throttle_duration_sec=2.0,
            )
            self._set_ready(False)
            return

        self.last_output_time = self.get_clock().now()
        self._set_ready(True)

    def _pose_in_map(self, msg: PoseStamped, source_frame: str) -> PoseStamped:
        if source_frame == self.map_frame:
            map_base = PoseStamped()
            map_base.pose = msg.pose
        else:
            transform = self.buffer.lookup_transform(
                self.map_frame,
                source_frame,
                rclpy.time.Time(),
            )
            map_base = PoseStamped()
            map_base.pose = do_transform_pose(msg.pose, transform)
        map_base.header.frame_id = self.map_frame
        map_base.header.stamp = self.get_clock().now().to_msg()
        return map_base

    def _reference_pose_to_base_pose(self, map_reference: PoseStamped) -> PoseStamped:
        base_to_reference = self.buffer.lookup_transform(
            self.robot_base_frame,
            self.input_pose_frame,
            rclpy.time.Time(),
        )
        map_base_matrix = concatenate_matrices(
            self._pose_to_matrix(map_reference),
            inverse_matrix(self._transform_to_matrix(base_to_reference)),
        )
        map_base = PoseStamped()
        map_base.header = map_reference.header
        map_base.pose.position.x = float(map_base_matrix[0, 3])
        map_base.pose.position.y = float(map_base_matrix[1, 3])
        map_base.pose.position.z = float(map_base_matrix[2, 3])
        qx, qy, qz, qw = quaternion_from_matrix(map_base_matrix)
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
        return concatenate_matrices(
            translation_matrix((position.x, position.y, position.z)),
            quaternion_matrix((orientation.x, orientation.y, orientation.z, orientation.w)),
        )

    @staticmethod
    def _transform_to_matrix(transform: TransformStamped):
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return concatenate_matrices(
            translation_matrix((translation.x, translation.y, translation.z)),
            quaternion_matrix((rotation.x, rotation.y, rotation.z, rotation.w)),
        )

    @staticmethod
    def _transform_from_matrix(header, child_frame: str, matrix) -> TransformStamped:
        transform = TransformStamped()
        transform.header = header
        transform.child_frame_id = child_frame
        transform.transform.translation.x = float(matrix[0, 3])
        transform.transform.translation.y = float(matrix[1, 3])
        transform.transform.translation.z = float(matrix[2, 3])
        qx, qy, qz, qw = quaternion_from_matrix(matrix)
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
    node = ExternalBaseReference()
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
