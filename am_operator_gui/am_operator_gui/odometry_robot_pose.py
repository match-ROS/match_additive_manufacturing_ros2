#!/usr/bin/env python3
from __future__ import annotations

import math
from typing import Optional

from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster
from tf_transformations import (
    concatenate_matrices,
    inverse_matrix,
    quaternion_from_matrix,
    quaternion_matrix,
    translation_matrix,
)


def _clean_frame(frame: str) -> str:
    return str(frame).strip().lstrip('/')


def _normalized_quaternion_xyzw(
    x: float,
    y: float,
    z: float,
    w: float,
) -> tuple[float, float, float, float]:
    values = (float(x), float(y), float(z), float(w))
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f'quaternion contains non-finite values: {values}')
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 1e-9:
        raise ValueError(f'quaternion norm is too small: {values}')
    return tuple(value / norm for value in values)


class OdometryRobotPose(Node):
    """Publish /robot_pose by anchoring odometry to base_path[initial_path_index]."""

    def __init__(self, **kwargs) -> None:
        super().__init__('odometry_robot_pose', **kwargs)
        self.declare_parameter('odom_topic', '/robot/robotnik_base_control/odom')
        self.declare_parameter('path_topic', '/base_path')
        self.declare_parameter('output_topic', '/robot_pose')
        self.declare_parameter('initial_path_index', 0)
        self.declare_parameter('map_frame', '')
        self.declare_parameter('odom_frame', '')
        self.declare_parameter('robot_base_frame', '')
        self.declare_parameter('ready_topic', '~/ready')
        self.declare_parameter('stale_timeout', 0.5)
        self.declare_parameter('publish_tf', True)

        self.initial_path_index = max(0, int(self.get_parameter('initial_path_index').value))
        self.map_frame_override = _clean_frame(str(self.get_parameter('map_frame').value))
        self.odom_frame_override = _clean_frame(str(self.get_parameter('odom_frame').value))
        self.robot_base_frame_override = _clean_frame(
            str(self.get_parameter('robot_base_frame').value)
        )
        self.stale_timeout = max(0.05, float(self.get_parameter('stale_timeout').value))
        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        self.latest_path: Optional[Path] = None
        self.latest_odom: Optional[Odometry] = None
        self.map_to_odom_matrix = None
        self.output_frame = ''
        self.odom_frame = ''
        self.robot_base_frame = ''
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
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Path,
            str(self.get_parameter('path_topic').value),
            self._path_cb,
            10,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self._odom_cb,
            10,
        )
        self.create_timer(min(0.1, self.stale_timeout / 2.0), self._check_stale)
        self._set_ready(False)

        self.get_logger().info(
            'Publishing robot pose from odometry anchored to '
            f"{self.get_parameter('path_topic').value}[{self.initial_path_index}] "
            f"using {self.get_parameter('odom_topic').value}."
        )

    def _path_cb(self, msg: Path) -> None:
        self.latest_path = msg
        self._try_initialize_anchor()

    def _odom_cb(self, msg: Odometry) -> None:
        self.latest_odom = msg
        self._try_initialize_anchor()
        if self.map_to_odom_matrix is None:
            self._set_ready(False)
            return

        try:
            odom_to_base = self._pose_to_matrix(msg.pose.pose)
            map_to_base = concatenate_matrices(self.map_to_odom_matrix, odom_to_base)
            output = self._pose_stamped_from_matrix(
                self.output_frame,
                map_to_base,
                stamp=msg.header.stamp,
            )
            self.pose_pub.publish(output)
            if self.publish_tf and self.odom_frame:
                self.broadcaster.sendTransform(
                    self._transform_from_matrix(
                        self.output_frame,
                        self.odom_frame,
                        self.map_to_odom_matrix,
                        stamp=msg.header.stamp,
                    )
                )
        except ValueError as exc:
            self.get_logger().warn(
                f'Cannot publish odometry-derived robot pose: {exc}',
                throttle_duration_sec=2.0,
            )
            self._set_ready(False)
            return

        self.last_output_time = self.get_clock().now()
        self._set_ready(True)

    def _try_initialize_anchor(self) -> None:
        if self.map_to_odom_matrix is not None:
            return
        if self.latest_path is None or self.latest_odom is None:
            return
        if self.initial_path_index >= len(self.latest_path.poses):
            self.get_logger().warn(
                f'Waiting for base_path[{self.initial_path_index}], '
                f'latest path length is {len(self.latest_path.poses)}.',
                throttle_duration_sec=2.0,
            )
            self._set_ready(False)
            return

        path_pose = self.latest_path.poses[self.initial_path_index]
        output_frame = self.map_frame_override or _clean_frame(path_pose.header.frame_id)
        if not output_frame:
            output_frame = _clean_frame(self.latest_path.header.frame_id)
        if not output_frame:
            self.get_logger().warn(
                'Waiting for a non-empty base path frame before anchoring odometry.',
                throttle_duration_sec=2.0,
            )
            self._set_ready(False)
            return

        odom_frame = self.odom_frame_override or _clean_frame(self.latest_odom.header.frame_id)
        robot_base_frame = (
            self.robot_base_frame_override
            or _clean_frame(self.latest_odom.child_frame_id)
        )
        try:
            map_to_base_initial = self._pose_to_matrix(path_pose.pose)
            odom_to_base_initial = self._pose_to_matrix(self.latest_odom.pose.pose)
            self.map_to_odom_matrix = concatenate_matrices(
                map_to_base_initial,
                inverse_matrix(odom_to_base_initial),
            )
        except ValueError as exc:
            self.get_logger().warn(
                f'Waiting for valid initial odometry anchor: {exc}',
                throttle_duration_sec=2.0,
            )
            self._set_ready(False)
            return

        self.output_frame = output_frame
        self.odom_frame = odom_frame
        self.robot_base_frame = robot_base_frame
        self.get_logger().info(
            f'Anchored odometry at base_path[{self.initial_path_index}] '
            f'in frame {self.output_frame}; odom_frame={self.odom_frame or "<none>"}, '
            f'base_frame={self.robot_base_frame or "<none>"}.'
        )

    @staticmethod
    def _pose_to_matrix(pose: Pose):
        position = pose.position
        orientation = pose.orientation
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
    def _pose_stamped_from_matrix(frame_id: str, matrix, stamp) -> PoseStamped:
        qx, qy, qz, qw = _normalized_quaternion_xyzw(*quaternion_from_matrix(matrix))
        msg = PoseStamped()
        msg.header.frame_id = frame_id
        msg.header.stamp = stamp
        msg.pose.position.x = float(matrix[0, 3])
        msg.pose.position.y = float(matrix[1, 3])
        msg.pose.position.z = float(matrix[2, 3])
        msg.pose.orientation.x = float(qx)
        msg.pose.orientation.y = float(qy)
        msg.pose.orientation.z = float(qz)
        msg.pose.orientation.w = float(qw)
        return msg

    @staticmethod
    def _transform_from_matrix(
        parent_frame: str,
        child_frame: str,
        matrix,
        stamp,
    ) -> TransformStamped:
        qx, qy, qz, qw = _normalized_quaternion_xyzw(*quaternion_from_matrix(matrix))
        transform = TransformStamped()
        transform.header.frame_id = parent_frame
        transform.header.stamp = stamp
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
    node = OdometryRobotPose()
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
