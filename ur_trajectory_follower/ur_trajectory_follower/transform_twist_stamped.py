#!/usr/bin/env python3
import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from tf2_ros import Buffer, TransformException, TransformListener
from tf_transformations import quaternion_matrix


class TransformTwistStamped(Node):
    def __init__(self) -> None:
        super().__init__('transform_twist_stamped')
        self.declare_parameter('input_topic', '/twist_in')
        self.declare_parameter('output_topic', '/twist_out')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('output_frame', '')
        self.declare_parameter('fallback_source_frame', 'map')

        self.target_frame = str(self.get_parameter('target_frame').value)
        # Some vendor controllers use a frame label that differs from the TF
        # frame name. Transform in the real TF frame, then label the command
        # with the controller's expected name.
        self.output_frame = str(self.get_parameter('output_frame').value) or self.target_frame
        self.fallback_source_frame = str(self.get_parameter('fallback_source_frame').value)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.pub = self.create_publisher(
            TwistStamped,
            str(self.get_parameter('output_topic').value),
            QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT),
        )
        self.create_subscription(
            TwistStamped,
            str(self.get_parameter('input_topic').value),
            self._twist_cb,
            10,
        )

    def _twist_cb(self, msg: TwistStamped) -> None:
        source_frame = msg.header.frame_id or self.fallback_source_frame
        if source_frame == self.target_frame:
            out = TwistStamped()
            out.header = msg.header
            out.header.frame_id = self.output_frame
            out.twist = msg.twist
            self.pub.publish(out)
            return

        try:
            transform = self.buffer.lookup_transform(
                self.target_frame,
                source_frame,
                rclpy.time.Time(),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f"Waiting for TF {self.target_frame} <- {source_frame}: {exc}",
                throttle_duration_sec=2.0,
            )
            return

        quat = transform.transform.rotation
        rotation = quaternion_matrix([quat.x, quat.y, quat.z, quat.w])[0:3, 0:3]
        linear = rotation @ np.array(
            [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z],
            dtype=float,
        )
        angular = rotation @ np.array(
            [msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z],
            dtype=float,
        )

        out = TwistStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.output_frame
        out.twist.linear.x = float(linear[0])
        out.twist.linear.y = float(linear[1])
        out.twist.linear.z = float(linear[2])
        out.twist.angular.x = float(angular[0])
        out.twist.angular.y = float(angular[1])
        out.twist.angular.z = float(angular[2])
        self.pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TransformTwistStamped()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
