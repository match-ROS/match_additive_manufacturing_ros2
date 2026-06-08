#!/usr/bin/env python3
from typing import Dict

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node

from ur_trajectory_follower.ros2_utils import as_bool, as_string_list


class TwistCombiner(Node):
    def __init__(self) -> None:
        super().__init__('twist_combiner')
        self.declare_parameter('twist_topics', '')
        self.declare_parameter('combined_twist_topic', '/combined_twist')
        self.declare_parameter('output_stamped', False)
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('publish_rate_hz', 50.0)

        self.twist_topics = as_string_list(self.get_parameter('twist_topics').value)
        self.output_stamped = as_bool(self.get_parameter('output_stamped').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.twists: Dict[str, Twist] = {topic: Twist() for topic in self.twist_topics}

        topic = str(self.get_parameter('combined_twist_topic').value)
        msg_type = TwistStamped if self.output_stamped else Twist
        self.pub = self.create_publisher(msg_type, topic, 10)
        for twist_topic in self.twist_topics:
            self.create_subscription(
                Twist, twist_topic, self._make_callback(twist_topic), 10
            )

        rate_hz = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self.create_timer(1.0 / rate_hz, self.publish_combined_twist)
        self.get_logger().info(
            f"Combining {self.twist_topics} into {topic}"
            + (" as TwistStamped." if self.output_stamped else ".")
        )

    def _make_callback(self, topic: str):
        def callback(msg: Twist) -> None:
            self.twists[topic] = msg

        return callback

    def publish_combined_twist(self) -> None:
        combined = Twist()
        for twist in self.twists.values():
            combined.linear.x += twist.linear.x
            combined.linear.y += twist.linear.y
            combined.linear.z += twist.linear.z
            combined.angular.x += twist.angular.x
            combined.angular.y += twist.angular.y
            combined.angular.z += twist.angular.z

        if self.output_stamped:
            stamped = TwistStamped()
            stamped.header.stamp = self.get_clock().now().to_msg()
            stamped.header.frame_id = self.frame_id
            stamped.twist = combined
            self.pub.publish(stamped)
        else:
            self.pub.publish(combined)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TwistCombiner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
