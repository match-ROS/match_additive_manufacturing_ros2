#!/usr/bin/env python3
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Vector3
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, Int32

from ur_trajectory_follower.ros2_utils import as_bool


class IncrementPathIndex(Node):
    def __init__(self) -> None:
        super().__init__('increment_path_index')
        self.declare_parameter('path_index_topic', '/path_index')
        self.declare_parameter('next_goal_topic', '/next_goal')
        self.declare_parameter('normal_topic', '/normal_vector')
        self.declare_parameter('initial_path_index', 0)
        self.declare_parameter('path_topic', '/ur_path_transformed')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('start_condition_topic', '/start_condition')
        self.declare_parameter('wait_for_start_condition', True)

        self.path: Optional[Path] = None
        self.path_index = max(0, int(self.get_parameter('initial_path_index').value))
        self.start_enabled = not as_bool(self.get_parameter('wait_for_start_condition').value)
        self.normal = Vector3(x=0.0, y=0.0, z=1.0)

        latch_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, reliability=QoSReliabilityPolicy.RELIABLE)
        self.index_pub = self.create_publisher(Int32, str(self.get_parameter('path_index_topic').value), latch_qos)
        self.goal_pose_pub = self.create_publisher(PoseStamped, str(self.get_parameter('next_goal_topic').value), latch_qos)
        self.normal_pub = self.create_publisher(Vector3, str(self.get_parameter('normal_topic').value), latch_qos)

        self.create_subscription(Path, str(self.get_parameter('path_topic').value), self._path_cb, latch_qos)
        self.create_subscription(Vector3, str(self.get_parameter('normal_topic').value), self._normal_cb, latch_qos)
        self.create_subscription(Bool, str(self.get_parameter('start_condition_topic').value), self._start_cb, 10)
        rate = max(0.1, float(self.get_parameter('publish_rate').value))
        self.create_timer(1.0 / rate, self._tick)

    def _path_cb(self, msg: Path) -> None:
        if not msg.poses:
            self.get_logger().warn("Ignoring empty path.")
            return
        self.path = msg
        self.path_index = min(self.path_index, len(msg.poses) - 1)

    def _normal_cb(self, msg: Vector3) -> None:
        self.normal = msg

    def _start_cb(self, msg: Bool) -> None:
        self.start_enabled = bool(msg.data)

    def _tick(self) -> None:
        if self.path is None or not self.path.poses:
            return
        if self.start_enabled and self.path_index < len(self.path.poses) - 1:
            self.path_index += 1
        self.index_pub.publish(Int32(data=self.path_index))
        self.goal_pose_pub.publish(self.path.poses[self.path_index])
        self.normal_pub.publish(self.normal)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = IncrementPathIndex()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
