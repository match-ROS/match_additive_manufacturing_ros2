#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


class CurrentPoseFromTf(Node):
    def __init__(self) -> None:
        super().__init__('current_pose_from_tf')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('source_frame', 'tool0')
        self.declare_parameter('pose_topic', '/current_tcp_pose')
        self.declare_parameter('publish_rate', 50.0)

        self.target_frame = str(self.get_parameter('target_frame').value)
        self.source_frame = str(self.get_parameter('source_frame').value)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.pub = self.create_publisher(PoseStamped, str(self.get_parameter('pose_topic').value), 10)
        rate = max(1.0, float(self.get_parameter('publish_rate').value))
        self.create_timer(1.0 / rate, self._tick)

    def _tick(self) -> None:
        try:
            tf = self.buffer.lookup_transform(self.target_frame, self.source_frame, rclpy.time.Time())
        except TransformException as exc:
            self.get_logger().warn(f"Waiting for TF {self.target_frame} <- {self.source_frame}: {exc}", throttle_duration_sec=5.0)
            return

        pose = PoseStamped()
        pose.header = tf.header
        pose.pose.position.x = tf.transform.translation.x
        pose.pose.position.y = tf.transform.translation.y
        pose.pose.position.z = tf.transform.translation.z
        pose.pose.orientation = tf.transform.rotation
        self.pub.publish(pose)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CurrentPoseFromTf()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
