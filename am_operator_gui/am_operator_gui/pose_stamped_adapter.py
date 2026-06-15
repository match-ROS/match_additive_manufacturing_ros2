#!/usr/bin/env python3
from typing import Optional

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformException, TransformListener


class PoseStampedAdapter(Node):

    def __init__(self, **kwargs) -> None:
        super().__init__('pose_stamped_adapter', **kwargs)
        self.declare_parameter('input_topic', '/pose_in')
        self.declare_parameter('output_topic', '/pose_out')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('ready_topic', '~/ready')
        self.declare_parameter('stale_timeout', 0.5)

        self.target_frame = str(self.get_parameter('target_frame').value)
        self.stale_timeout = max(0.05, float(self.get_parameter('stale_timeout').value))
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
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
        source_frame = msg.header.frame_id.strip()
        if not source_frame:
            self.get_logger().warn(
                'Ignoring PoseStamped with an empty frame_id.',
                throttle_duration_sec=2.0,
            )
            self._set_ready(False)
            return

        try:
            if source_frame == self.target_frame:
                transformed = PoseStamped()
                transformed.header = msg.header
                transformed.pose = msg.pose
            else:
                transform = self.buffer.lookup_transform(
                    self.target_frame,
                    source_frame,
                    rclpy.time.Time(),
                )
                transformed = PoseStamped()
                transformed.pose = do_transform_pose(msg.pose, transform)
            transformed.header.frame_id = self.target_frame
            transformed.header.stamp = self.get_clock().now().to_msg()
        except TransformException as exc:
            self.get_logger().warn(
                f'Waiting for TF {self.target_frame} <- {source_frame}: {exc}',
                throttle_duration_sec=2.0,
            )
            self._set_ready(False)
            return

        self.pose_pub.publish(transformed)
        self.last_output_time = self.get_clock().now()
        self._set_ready(True)

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
