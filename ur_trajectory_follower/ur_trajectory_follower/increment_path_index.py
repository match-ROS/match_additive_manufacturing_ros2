#!/usr/bin/env python3
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Vector3
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, Float32, Int32

from ur_trajectory_follower.ros2_utils import as_bool


class IncrementPathIndex(Node):
    def __init__(self) -> None:
        super().__init__('increment_path_index')
        self.declare_parameter('path_index_topic', '/path_index')
        self.declare_parameter('next_goal_topic', '/next_goal')
        self.declare_parameter('additional_goal_path_topic', '')
        self.declare_parameter('additional_goal_topic', '')
        self.declare_parameter('normal_topic', '/normal_vector')
        self.declare_parameter('initial_path_index', 0)
        self.declare_parameter('path_topic', '/ur_path_transformed')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('velocity_override_topic', '/velocity_override')
        self.declare_parameter('start_condition_topic', '/start_condition')
        self.declare_parameter('wait_for_start_condition', True)

        self.path: Optional[Path] = None
        self.additional_goal_path: Optional[Path] = None
        self.path_index = max(0, int(self.get_parameter('initial_path_index').value))
        self._last_published_index: Optional[int] = None
        self.start_enabled = not as_bool(self.get_parameter('wait_for_start_condition').value)
        self.normal = Vector3(x=0.0, y=0.0, z=1.0)
        self.base_publish_rate = max(0.0, float(self.get_parameter('publish_rate').value))
        self.velocity_override = 1.0
        self._timer = None

        latch_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.path_index_topic = str(self.get_parameter('path_index_topic').value)
        self.index_pub = self.create_publisher(Int32, self.path_index_topic, latch_qos)
        self.goal_pose_pub = self.create_publisher(
            PoseStamped,
            str(self.get_parameter('next_goal_topic').value),
            latch_qos,
        )
        self.additional_goal_pose_pub = None
        additional_goal_path_topic = str(
            self.get_parameter('additional_goal_path_topic').value
        ).strip()
        additional_goal_topic = str(
            self.get_parameter('additional_goal_topic').value
        ).strip()
        if additional_goal_path_topic and additional_goal_topic:
            self.additional_goal_pose_pub = self.create_publisher(
                PoseStamped,
                additional_goal_topic,
                latch_qos,
            )
            self.create_subscription(
                Path,
                additional_goal_path_topic,
                self._additional_goal_path_cb,
                latch_qos,
            )
        elif additional_goal_path_topic or additional_goal_topic:
            self.get_logger().warn(
                'Ignoring additional goal publisher: both additional_goal_path_topic '
                'and additional_goal_topic must be set.'
            )
        self.normal_pub = self.create_publisher(
            Vector3,
            str(self.get_parameter('normal_topic').value),
            latch_qos,
        )

        self.create_subscription(Path, str(self.get_parameter('path_topic').value), self._path_cb, latch_qos)
        self.create_subscription(Vector3, str(self.get_parameter('normal_topic').value), self._normal_cb, latch_qos)
        self.create_subscription(Int32, self.path_index_topic, self._external_index_cb, 10)
        self.create_subscription(
            Float32,
            str(self.get_parameter('velocity_override_topic').value),
            self._velocity_override_cb,
            10,
        )
        self.create_subscription(Bool, str(self.get_parameter('start_condition_topic').value), self._start_cb, 10)
        self._update_timer()

    def _path_cb(self, msg: Path) -> None:
        if not msg.poses:
            self.get_logger().warn("Ignoring empty path.")
            return
        had_path = self.path is not None
        self.path = msg
        clamped_index = min(self.path_index, len(msg.poses) - 1)
        index_changed = clamped_index != self.path_index
        self.path_index = clamped_index
        if not had_path or index_changed:
            self._publish_state(force=True)

    def _additional_goal_path_cb(self, msg: Path) -> None:
        if not msg.poses:
            self.get_logger().warn("Ignoring empty additional goal path.")
            return
        self.additional_goal_path = msg
        self._publish_state(force=True)

    def _normal_cb(self, msg: Vector3) -> None:
        self.normal = msg
        self.normal_pub.publish(self.normal)

    def _start_cb(self, msg: Bool) -> None:
        self.start_enabled = bool(msg.data)

    def _velocity_override_cb(self, msg: Float32) -> None:
        new_override = max(0.0, float(msg.data))
        if abs(new_override - self.velocity_override) < 1e-6:
            return
        self.velocity_override = new_override
        self._update_timer()

    def _update_timer(self) -> None:
        if self._timer is not None:
            self._timer.cancel()
            self.destroy_timer(self._timer)
            self._timer = None

        effective_rate = self.base_publish_rate * self.velocity_override
        if effective_rate <= 0.0:
            self.get_logger().info(
                "Path index advancement paused because effective publish rate is zero."
            )
            return

        self._timer = self.create_timer(1.0 / effective_rate, self._tick)
        self.get_logger().info(
            f"Path index advancement rate set to {effective_rate:.3f} Hz "
            f"({self.base_publish_rate:.3f} Hz * velocity_override {self.velocity_override:.3f})."
        )

    def _external_index_cb(self, msg: Int32) -> None:
        requested_index = max(0, int(msg.data))
        if self.path is not None and self.path.poses:
            requested_index = min(requested_index, len(self.path.poses) - 1)
        if requested_index == self.path_index:
            return
        self.path_index = requested_index
        self._publish_state(force=True)

    def _tick(self) -> None:
        if self.path is None or not self.path.poses:
            return
        if self.start_enabled and self.path_index < len(self.path.poses) - 1:
            self.path_index += 1
            self._publish_state()

    def _publish_state(self, force: bool = False) -> None:
        if self.path is None or not self.path.poses:
            return
        if not force and self.path_index == self._last_published_index:
            return
        self.index_pub.publish(Int32(data=self.path_index))
        self.goal_pose_pub.publish(self.path.poses[self.path_index])
        if (
            self.additional_goal_pose_pub is not None
            and self.additional_goal_path is not None
        ):
            additional_index = min(
                self.path_index,
                len(self.additional_goal_path.poses) - 1,
            )
            self.additional_goal_pose_pub.publish(self.additional_goal_path.poses[additional_index])
        self.normal_pub.publish(self.normal)
        self._last_published_index = self.path_index


def main(args=None) -> None:
    rclpy.init(args=args)
    node = IncrementPathIndex()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
