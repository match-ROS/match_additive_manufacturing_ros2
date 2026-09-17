"""Select base/TCP feedback live and bridge gaps with anchored odometry."""
import time

import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import rclpy
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, qos_profile_sensor_data
from tf2_ros import TransformException
from tf_transformations import inverse_matrix

from .external_base_reference import ExternalBaseReference
from .odometry_robot_pose import OdometryRobotPose


class LiveRobotPose(ExternalBaseReference):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.declare_parameter('use_odometry_robot_pose', False)
        self.declare_parameter('use_vicon_tcp_base_pose_fallback', False)
        self.declare_parameter('tcp_topic', '/vicon/tool_transformed')
        self.declare_parameter('robot_tcp_frame', 'robot_arm_nozzle_tip')
        self.declare_parameter('odom_topic', '/robot/robotnik_base_control/odom')
        self.use_odom = bool(self.get_parameter('use_odometry_robot_pose').value)
        self.use_tcp = bool(self.get_parameter('use_vicon_tcp_base_pose_fallback').value)
        self.input_pose_frame = self.input_pose_frame or self.robot_base_frame
        self.primary_time = None
        self.odom_time = None
        self.odom_matrix = None
        self.odom_frame = None
        self.anchor = None
        self.last_primary = None
        self.processing_tcp = False
        self.bridging = False
        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(Bool, '/am/robot_pose/use_odometry', self._odom_mode, qos)
        self.create_subscription(Bool, '/am/robot_pose/use_tcp', self._tcp_mode, qos)
        self.create_subscription(PoseStamped, str(self.get_parameter('tcp_topic').value), self._tcp_cb, 10)
        self.create_subscription(Odometry, str(self.get_parameter('odom_topic').value),
                                 self._odom_cb, qos_profile_sensor_data)

    def _odom_mode(self, msg):
        self.use_odom = msg.data

    def _tcp_mode(self, msg):
        if self.use_tcp != msg.data:
            self.use_tcp = msg.data
            self.primary_time = None

    def _pose_cb(self, msg):
        if self.use_tcp != self.processing_tcp:
            return
        # Base conversion validates input, catches missing TF and only publishes
        # a successful pose. Anchoring happens inside the successful TF path.
        super()._pose_cb(msg)

    def _tcp_cb(self, msg):
        if not self.use_tcp:
            return
        self.processing_tcp = True
        try:
            self._pose_cb(msg)
        finally:
            self.processing_tcp = False

    def _reference_pose_to_base_pose(self, pose):
        if not self.processing_tcp:
            return super()._reference_pose_to_base_pose(pose)
        tf = self.buffer.lookup_transform(self.robot_base_frame,
            str(self.get_parameter('robot_tcp_frame').value), rclpy.time.Time())
        matrix = self._pose_to_matrix(pose) @ inverse_matrix(self._transform_to_matrix(tf))
        return OdometryRobotPose._pose_stamped_from_matrix(self.map_frame, matrix, pose.header.stamp)

    def _publish_map_to_robot_tree(self, pose):
        super()._publish_map_to_robot_tree(pose)
        if not self.bridging:
            self.primary_time = time.monotonic()
            self.last_primary = self._pose_to_matrix(pose)
            if self.odom_time is not None and self.primary_time - self.odom_time <= self.stale_timeout:
                self.anchor = self.last_primary @ inverse_matrix(self.odom_matrix)
            else:
                self.anchor = None

    def _odom_cb(self, msg):
        try:
            sample = PoseStamped()
            sample.pose = msg.pose.pose
            sample = self._validated_pose(sample)
            matrix = self._pose_to_matrix(sample)
            frame = msg.header.frame_id.lstrip('/')
            child = msg.child_frame_id.lstrip('/')
            if not frame or not child:
                raise ValueError('odometry requires parent and child frames')
            if child != self.robot_base_frame:
                tf = self.buffer.lookup_transform(child, self.robot_base_frame, rclpy.time.Time())
                matrix = matrix @ self._transform_to_matrix(tf)
            stamp = rclpy.time.Time.from_msg(msg.header.stamp)
            if stamp.nanoseconds and (self.get_clock().now() - stamp).nanoseconds / 1e9 > self.stale_timeout:
                return
            if self.odom_frame is not None and frame != self.odom_frame:
                self.anchor = None
            self.odom_frame = frame
            self.odom_matrix = matrix
            self.odom_time = time.monotonic()
            fresh = self.primary_time is not None and self.odom_time - self.primary_time <= self.stale_timeout
            # If odometry was first discovered just after a primary sample,
            # establish its baseline there; never initialize from the path.
            if fresh:
                if self.anchor is None and self.last_primary is not None:
                    self.anchor = self.last_primary @ inverse_matrix(matrix)
                return
            if not self.use_odom or self.anchor is None:
                return
            output = OdometryRobotPose._pose_stamped_from_matrix(
                self.map_frame, self.anchor @ matrix, msg.header.stamp)
            output = self._validated_pose(output)
            self.bridging = True
            try:
                self._publish_map_to_robot_tree(output)
                self.pose_pub.publish(output)
            finally:
                self.bridging = False
            self.last_output_time = self.get_clock().now()
            self._set_ready(True)
        except (ValueError, np.linalg.LinAlgError, TransformException) as exc:
            self.get_logger().warn(f'Cannot bridge robot pose with odometry: {exc}', throttle_duration_sec=2.0)
