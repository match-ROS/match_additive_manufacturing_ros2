#!/usr/bin/env python3
from typing import Dict, List, Optional, Set

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Bool
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
        self.declare_parameter('input_timeout', 0.5)
        self.declare_parameter('wait_for_start_condition', False)
        self.declare_parameter('start_condition_topic', '/start_condition')

        self.twist_topics = as_string_list(self.get_parameter('twist_topics').value)
        self.output_stamped = as_bool(self.get_parameter('output_stamped').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.wait_for_start_condition = as_bool(self.get_parameter('wait_for_start_condition').value)
        self.start_condition_received = not self.wait_for_start_condition
        self.input_timeout = max(0.0, float(self.get_parameter('input_timeout').value))
        self.twists: Dict[str, Twist] = {topic: Twist() for topic in self.twist_topics}
        # Twist has no header, so freshness is measured from local receipt time.
        self.last_twist_times: Dict[str, Optional[int]] = {
            topic: None for topic in self.twist_topics
        }
        self._stale_inputs: Set[str] = set()

        topic = str(self.get_parameter('combined_twist_topic').value)
        msg_type = TwistStamped if self.output_stamped else Twist
        self.pub = self.create_publisher(msg_type, topic, 10)
        for twist_topic in self.twist_topics:
            self.create_subscription(
                Twist, twist_topic, self._make_callback(twist_topic), 10
            )

        if self.wait_for_start_condition:
            self.create_subscription(
                Bool,
                str(self.get_parameter('start_condition_topic').value),
                self._start_condition_callback,
                10,
            )

        rate_hz = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self.create_timer(1.0 / rate_hz, self.publish_combined_twist)
        self.get_logger().info(
            f"Combining {self.twist_topics} into {topic}"
            + (" as TwistStamped." if self.output_stamped else ".")
            + (
                " Waiting for start condition."
                if self.wait_for_start_condition
                else ""
            )
        )

    def _make_callback(self, topic: str):
        def callback(msg: Twist) -> None:
            now = self.get_clock().now().nanoseconds
            self.twists[topic] = msg
            self.last_twist_times[topic] = now
            self._update_stale_inputs(self._stale_topics(now))

        return callback

    def _stale_topics(self, now: int) -> List[str]:
        return [
            topic for topic, received_at in self.last_twist_times.items()
            if received_at is None or (now - received_at) / 1e9 > self.input_timeout
        ]

    def _update_stale_inputs(self, stale_topics: List[str]) -> None:
        stale_inputs = set(stale_topics)
        newly_stale = sorted(stale_inputs - self._stale_inputs)
        if newly_stale:
            self.get_logger().warn(
                'Twist input freshness not fulfilled; zeroing stale inputs: '
                + ', '.join(newly_stale)
            )
        self._stale_inputs = stale_inputs
        return stale_topics

    def _start_condition_callback(self, msg: Bool) -> None:
        self.start_condition_received = bool(msg.data)
        if self.start_condition_received:
            self.get_logger().info('Start condition received; publishing combined twist.')

    def publish_combined_twist(self) -> None:
        if not self.start_condition_received:
            return

        now = self.get_clock().now().nanoseconds
        stale_topics = self._stale_topics(now)
        self._update_stale_inputs(stale_topics)
        combined = Twist()
        for topic, cached_twist in self.twists.items():
            # Do not retain a component after its source stops publishing.
            twist = Twist() if topic in stale_topics else cached_twist
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
