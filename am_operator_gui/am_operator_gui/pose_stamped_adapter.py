#!/usr/bin/env python3
"""Publish a measured nozzle pose or derive it from the base and arm pose."""
from typing import Optional

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, String
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformException, TransformListener
from tf_transformations import concatenate_matrices, quaternion_from_matrix

from .vicon_tcp_robot_pose_backup import ViconTcpRobotPoseBackup


class PoseStampedAdapter(Node):

    def __init__(self, **kwargs) -> None:
        super().__init__('pose_stamped_adapter', **kwargs)
        self.declare_parameter('input_topic', '/pose_in')
        self.declare_parameter('output_topic', '/pose_out')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('ready_topic', '~/ready')
        self.declare_parameter('stale_timeout', 0.5)
        self.declare_parameter('use_base_tcp_pose_fallback', False)
        self.declare_parameter('base_tcp_pose_fallback_source', 'tf')
        self.declare_parameter('base_pose_topic', '')
        self.declare_parameter('controller_tcp_pose_topic', '/robot/arm/tcp_pose_broadcaster/pose')
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('robot_tcp_frame', 'robot_arm_nozzle_tip')
        self.declare_parameter('controller_tcp_frame', 'robot_arm_tool0_controller_raw')

        self.target_frame = str(self.get_parameter('target_frame').value).lstrip('/')
        self.stale_timeout = max(0.05, float(self.get_parameter('stale_timeout').value))
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.last_output_time: Optional[rclpy.time.Time] = None
        self.ready = False
        selected = str(self.get_parameter('base_tcp_pose_fallback_source').value)
        if selected not in {'tf', 'topic'}:
            raise ValueError('base_tcp_pose_fallback_source must be tf or topic')
        self.source = selected if bool(self.get_parameter('use_base_tcp_pose_fallback').value) else 'measured'
        self.latest_base_pose: Optional[PoseStamped] = None
        # (arm frame, base <- arm mount, controller TCP <- nozzle offset)
        self.topic_geometry = None

        ready_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.pose_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter('output_topic').value), 10)
        self.ready_pub = self.create_publisher(
            Bool, str(self.get_parameter('ready_topic').value), ready_qos)
        self.create_subscription(PoseStamped, str(self.get_parameter('input_topic').value),
                                 self._pose_cb, 10)
        controller_qos = QoSProfile(
            depth=1, durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.BEST_EFFORT)
        base_topic = str(self.get_parameter('base_pose_topic').value)
        if base_topic:
            self.create_subscription(PoseStamped, base_topic, self._base_pose_cb, 10)
            self.create_subscription(PoseStamped,
                                     str(self.get_parameter('controller_tcp_pose_topic').value),
                                     self._controller_tcp_cb, controller_qos)
            self.create_subscription(String, '/am/tcp_pose/source', self._source_mode, ready_qos)
        self.create_timer(min(0.1, self.stale_timeout / 2.0), self._check_stale)
        self._set_ready(False)

    def _publish(self, pose: PoseStamped) -> None:
        self.pose_pub.publish(pose)
        self.last_output_time = self.get_clock().now()
        self._set_ready(True)

    def _pose_cb(self, msg: PoseStamped) -> None:
        if self.source != 'measured':
            return
        source_frame = msg.header.frame_id.strip()
        if not source_frame:
            self.get_logger().warn('Ignoring PoseStamped with an empty frame_id.',
                                   throttle_duration_sec=2.0)
            self._set_ready(False)
            return
        try:
            if source_frame == self.target_frame:
                transformed = PoseStamped()
                transformed.header = msg.header
                transformed.pose = msg.pose
            else:
                transform = self.buffer.lookup_transform(
                    self.target_frame, source_frame, rclpy.time.Time())
                transformed = PoseStamped()
                transformed.pose = do_transform_pose(msg.pose, transform)
            transformed.header.frame_id = self.target_frame
            transformed.header.stamp = self.get_clock().now().to_msg()
        except TransformException as exc:
            self.get_logger().warn(f'Waiting for TF {self.target_frame} <- {source_frame}: {exc}',
                                   throttle_duration_sec=2.0)
            self._set_ready(False)
            return
        self._publish(transformed)

    def _source_mode(self, msg: String) -> None:
        if msg.data not in {'measured', 'tf', 'topic'}:
            self.get_logger().warn(f'Ignoring unknown TCP pose source: {msg.data}')
            return
        if self.source != msg.data:
            self.source = msg.data
            self.last_output_time = None
            self._set_ready(False)

    def _map_base_pose(self, msg: PoseStamped) -> PoseStamped:
        source_frame = msg.header.frame_id.strip().lstrip('/')
        if not source_frame:
            raise ValueError('base pose has an empty frame_id')
        if source_frame == self.target_frame:
            return msg
        transform = self.buffer.lookup_transform(
            self.target_frame, source_frame, rclpy.time.Time())
        mapped = PoseStamped()
        mapped.pose = do_transform_pose(msg.pose, transform)
        mapped.header.frame_id = self.target_frame
        mapped.header.stamp = msg.header.stamp
        return mapped

    def _pose_from_matrix(self, matrix, stamp) -> PoseStamped:
        output = PoseStamped()
        output.header.frame_id = self.target_frame
        output.header.stamp = stamp
        output.pose.position.x = float(matrix[0, 3])
        output.pose.position.y = float(matrix[1, 3])
        output.pose.position.z = float(matrix[2, 3])
        q = quaternion_from_matrix(matrix)
        (output.pose.orientation.x, output.pose.orientation.y,
         output.pose.orientation.z, output.pose.orientation.w) = map(float, q)
        return output

    def _base_pose_cb(self, msg: PoseStamped) -> None:
        # Topic mode only stores the newest base pose. The arm topic drives output.
        self.latest_base_pose = msg
        if self.source != 'tf':
            return
        try:
            base_frame = str(self.get_parameter('robot_base_frame').value).lstrip('/')
            tcp_frame = str(self.get_parameter('robot_tcp_frame').value).lstrip('/')
            map_base = self._map_base_pose(msg)
            base_to_tcp = self.buffer.lookup_transform(base_frame, tcp_frame, rclpy.time.Time())
            matrix = concatenate_matrices(
                ViconTcpRobotPoseBackup._pose_to_matrix(map_base),
                ViconTcpRobotPoseBackup._transform_to_matrix(base_to_tcp))
            output = self._pose_from_matrix(matrix, self.get_clock().now().to_msg())
        except (TransformException, ValueError) as exc:
            self.get_logger().warn(f'Waiting for base-to-TCP FK: {exc}', throttle_duration_sec=2.0)
            self._set_ready(False)
            return
        self._publish(output)

    @staticmethod
    def _stamp_ns(stamp) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def _controller_tcp_cb(self, msg: PoseStamped) -> None:
        if self.source != 'topic':
            return
        base = self.latest_base_pose
        if base is None:
            self.get_logger().warn('Waiting for /robot_pose before controller TCP pose.',
                                   throttle_duration_sec=2.0)
            self._set_ready(False)
            return
        try:
            arm_frame = msg.header.frame_id.strip().lstrip('/')
            if not arm_frame:
                raise ValueError('controller TCP pose has an empty frame_id')
            if self.topic_geometry is None:
                base_frame = str(self.get_parameter('robot_base_frame').value).lstrip('/')
                controller_frame = str(self.get_parameter('controller_tcp_frame').value).lstrip('/')
                nozzle_frame = str(self.get_parameter('robot_tcp_frame').value).lstrip('/')
                base_from_arm = self.buffer.lookup_transform(
                    base_frame, arm_frame, rclpy.time.Time())
                controller_from_nozzle = self.buffer.lookup_transform(
                    controller_frame, nozzle_frame, rclpy.time.Time())
                self.topic_geometry = (
                    arm_frame,
                    ViconTcpRobotPoseBackup._transform_to_matrix(base_from_arm),
                    ViconTcpRobotPoseBackup._transform_to_matrix(controller_from_nozzle))
                self.get_logger().info(
                    f'Captured fixed TCP geometry {base_frame} <- {arm_frame} and '
                    f'{controller_frame} <- {nozzle_frame}; no further geometry TF lookups.')
            captured_arm, base_from_arm, controller_from_nozzle = self.topic_geometry
            if arm_frame != captured_arm:
                raise ValueError(f'controller TCP frame changed from {captured_arm} to {arm_frame}')
            map_base = self._map_base_pose(base)
            base_stamp = self._stamp_ns(base.header.stamp)
            arm_stamp = self._stamp_ns(msg.header.stamp)
            if base_stamp and arm_stamp and abs(base_stamp - arm_stamp) > 300_000_000:
                self.get_logger().warn(
                    f'Base and controller TCP timestamps differ by '
                    f'{abs(base_stamp - arm_stamp) / 1e9:.3f}s (limit 0.300s); publishing anyway.',
                    throttle_duration_sec=2.0)
            matrix = concatenate_matrices(
                ViconTcpRobotPoseBackup._pose_to_matrix(map_base),
                base_from_arm,
                ViconTcpRobotPoseBackup._pose_to_matrix(msg),
                controller_from_nozzle)
            stamp = msg.header.stamp if arm_stamp else self.get_clock().now().to_msg()
            output = self._pose_from_matrix(matrix, stamp)
        except (TransformException, ValueError) as exc:
            self.get_logger().warn(f'Waiting for controller TCP pose geometry: {exc}',
                                   throttle_duration_sec=2.0)
            self._set_ready(False)
            return
        self._publish(output)

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
    node = PoseStampedAdapter()
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
