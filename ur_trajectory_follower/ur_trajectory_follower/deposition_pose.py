"""Publish a virtual deposition pose at a configurable nozzle stand-off."""

from __future__ import annotations

import math

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from tf_transformations import quaternion_matrix


def clamp_distance_step(current: float, target: float, max_rate: float, dt: float) -> float:
    """Apply a symmetric slew-rate limit, keeping the operation deterministic."""
    if max_rate <= 0.0 or dt <= 0.0:
        return target
    limit = max_rate * dt
    return current + max(-limit, min(limit, target - current))


def deposition_pose_from_nozzle(nozzle_pose: PoseStamped, distance: float) -> PoseStamped:
    """Return the pose displaced along the nozzle pose's local +Z axis."""
    q = nozzle_pose.pose.orientation
    quaternion = (float(q.x), float(q.y), float(q.z), float(q.w))
    if math.sqrt(sum(value * value for value in quaternion)) < 1e-9:
        raise ValueError('nozzle pose has an invalid orientation quaternion')
    axis = quaternion_matrix(quaternion)[0:3, 2]
    result = PoseStamped()
    result.header = nozzle_pose.header
    # ROS Python message assignment aliases the nested message object.  Copy
    # the fields explicitly so periodic output never modifies the cached input.
    result.pose.position.x = nozzle_pose.pose.position.x + float(axis[0]) * float(distance)
    result.pose.position.y = nozzle_pose.pose.position.y + float(axis[1]) * float(distance)
    result.pose.position.z = nozzle_pose.pose.position.z + float(axis[2]) * float(distance)
    result.pose.orientation = nozzle_pose.pose.orientation
    return result


class DepositionPose(Node):
    """Keep the measured nozzle and virtual deposition poses in one convention."""

    def __init__(self) -> None:
        super().__init__('deposition_pose')
        self.declare_parameter('nozzle_pose_topic', '/current_nozzle_tip_pose')
        self.declare_parameter('deposition_pose_topic', '/current_deposition_pose')
        self.declare_parameter('spray_distance_topic', '/spray_distance')
        self.declare_parameter('smoothed_spray_distance_topic', '/spray_distance_smoothed')
        self.declare_parameter('spray_distance_initial', 0.0)
        self.declare_parameter('spray_distance_max_rate', 0.02)
        self.declare_parameter('publish_rate', 50.0)

        self.target_distance = float(self.get_parameter('spray_distance_initial').value)
        self.smoothed_distance = self.target_distance
        self.max_rate = max(0.0, float(self.get_parameter('spray_distance_max_rate').value))
        self.last_nozzle_pose: PoseStamped | None = None
        self.last_tick = self.get_clock().now()
        self.pose_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter('deposition_pose_topic').value), 10)
        self.distance_pub = self.create_publisher(
            Float32, str(self.get_parameter('smoothed_spray_distance_topic').value), 10)
        self.create_subscription(
            PoseStamped, str(self.get_parameter('nozzle_pose_topic').value), self._nozzle_cb, 10)
        self.create_subscription(
            Float32, str(self.get_parameter('spray_distance_topic').value), self._distance_cb, 10)
        rate = max(1.0, float(self.get_parameter('publish_rate').value))
        self.create_timer(1.0 / rate, self._tick)

    def _nozzle_cb(self, msg: PoseStamped) -> None:
        self.last_nozzle_pose = msg

    def _distance_cb(self, msg: Float32) -> None:
        if math.isfinite(float(msg.data)):
            self.target_distance = float(msg.data)

    def _tick(self) -> None:
        now = self.get_clock().now()
        dt = max(0.0, (now - self.last_tick).nanoseconds / 1e9)
        self.last_tick = now
        self.smoothed_distance = clamp_distance_step(
            self.smoothed_distance, self.target_distance, self.max_rate, dt)
        self.distance_pub.publish(Float32(data=float(self.smoothed_distance)))
        if self.last_nozzle_pose is None:
            return
        try:
            output = deposition_pose_from_nozzle(self.last_nozzle_pose, self.smoothed_distance)
        except ValueError as exc:
            self.get_logger().warn(str(exc), throttle_duration_sec=2.0)
            return
        output.header.stamp = now.to_msg()
        self.pose_pub.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DepositionPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
