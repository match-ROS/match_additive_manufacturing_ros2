#!/usr/bin/env python3
"""
Publish a fixed PoseStamped in the map frame.

Example:
  ./publish_map_pose.py
  ./publish_map_pose.py --ros-args -p output_topic:=/target_pose -p publish_count:=0
"""
from __future__ import annotations

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy


DEFAULT_POSITION = [-1.371411, 3.705373, -0.955608]
DEFAULT_ORIENTATION = [-0.084335, -0.059610, -0.590883, 0.800119]


def _as_float_list(value, expected_len: int, parameter_name: str) -> list[float]:
    result = [float(item) for item in value]
    if len(result) != expected_len:
        raise ValueError(
            f"{parameter_name} must contain exactly {expected_len} values, got {result}"
        )
    return result


class FixedMapPosePublisher(Node):

    def __init__(self) -> None:
        super().__init__("publish_map_pose")
        self.declare_parameter("output_topic", "/goal_pose_start")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("position", DEFAULT_POSITION)
        self.declare_parameter("orientation", DEFAULT_ORIENTATION)
        self.declare_parameter("publish_rate", 2.0)
        self.declare_parameter("publish_count", 5)

        self.output_topic = str(self.get_parameter("output_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.position = _as_float_list(
            self.get_parameter("position").value,
            3,
            "position",
        )
        self.orientation = _as_float_list(
            self.get_parameter("orientation").value,
            4,
            "orientation",
        )
        self.publish_count = int(self.get_parameter("publish_count").value)
        self.published_count = 0

        qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.publisher = self.create_publisher(PoseStamped, self.output_topic, qos)

        rate = max(0.1, float(self.get_parameter("publish_rate").value))
        self.create_timer(1.0 / rate, self._publish_pose)

        count_text = "continuously" if self.publish_count <= 0 else f"{self.publish_count} times"
        self.get_logger().info(
            f"Publishing fixed PoseStamped in frame '{self.frame_id}' on "
            f"{self.output_topic} {count_text}."
        )

    def _publish_pose(self) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.pose.position.x = self.position[0]
        msg.pose.position.y = self.position[1]
        msg.pose.position.z = self.position[2]
        msg.pose.orientation.x = self.orientation[0]
        msg.pose.orientation.y = self.orientation[1]
        msg.pose.orientation.z = self.orientation[2]
        msg.pose.orientation.w = self.orientation[3]

        self.publisher.publish(msg)
        self.published_count += 1

        if self.publish_count > 0 and self.published_count >= self.publish_count:
            self.get_logger().info(
                f"Published pose {self.published_count} times; shutting down."
            )
            rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FixedMapPosePublisher()
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
