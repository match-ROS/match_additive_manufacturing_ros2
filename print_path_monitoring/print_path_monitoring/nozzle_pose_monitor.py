#!/usr/bin/env python3
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Float32, Int32

from print_path_monitoring.error_metrics import compute_pose_error


class NozzlePoseMonitor(Node):
    def __init__(self) -> None:
        super().__init__('nozzle_pose_monitor')
        self.declare_parameter('tcp_pose_topic', '/current_tcp_pose')
        self.declare_parameter('reference_pose_topic', '')
        self.declare_parameter('reference_path_topic', '/ur_path_transformed')
        self.declare_parameter('path_index_topic', '/path_index')
        self.declare_parameter('position_error_topic', '/nozzle_position_error')
        self.declare_parameter('position_error_norm_topic', '/nozzle_position_error_norm')
        self.declare_parameter('yaw_error_topic', '/nozzle_yaw_error')
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('warn_error_distance', 0.02)

        self.tcp_pose: Optional[PoseStamped] = None
        self.reference_pose: Optional[PoseStamped] = None
        self.reference_path: Optional[Path] = None
        self.path_index: Optional[int] = None
        self.last_wait_reason = ''

        latch_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        self.create_subscription(
            PoseStamped,
            str(self.get_parameter('tcp_pose_topic').value),
            self._tcp_pose_cb,
            10,
        )
        reference_pose_topic = str(self.get_parameter('reference_pose_topic').value).strip()
        if reference_pose_topic:
            self.create_subscription(PoseStamped, reference_pose_topic, self._reference_pose_cb, 10)
        self.create_subscription(
            Path,
            str(self.get_parameter('reference_path_topic').value),
            self._reference_path_cb,
            latch_qos,
        )
        self.create_subscription(
            Int32,
            str(self.get_parameter('path_index_topic').value),
            self._path_index_cb,
            10,
        )

        self.error_pub = self.create_publisher(
            Vector3Stamped,
            str(self.get_parameter('position_error_topic').value),
            10,
        )
        self.error_norm_pub = self.create_publisher(
            Float32,
            str(self.get_parameter('position_error_norm_topic').value),
            10,
        )
        self.yaw_error_pub = self.create_publisher(
            Float32,
            str(self.get_parameter('yaw_error_topic').value),
            10,
        )

        rate = max(1.0, float(self.get_parameter('publish_rate').value))
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info("Nozzle pose monitor waiting for TCP pose and reference.")

    def _tcp_pose_cb(self, msg: PoseStamped) -> None:
        self.tcp_pose = msg

    def _reference_pose_cb(self, msg: PoseStamped) -> None:
        self.reference_pose = msg

    def _reference_path_cb(self, msg: Path) -> None:
        self.reference_path = msg

    def _path_index_cb(self, msg: Int32) -> None:
        self.path_index = max(0, int(msg.data))

    def _tick(self) -> None:
        if self.tcp_pose is None:
            self._log_waiting('tcp pose')
            return
        reference = self._select_reference()
        if reference is None:
            self._log_waiting('reference pose/path index')
            return

        error = compute_pose_error(self.tcp_pose.pose, reference.pose)
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = reference.header.frame_id or self.tcp_pose.header.frame_id
        msg.vector.x = float(error.dx)
        msg.vector.y = float(error.dy)
        msg.vector.z = float(error.dz)
        self.error_pub.publish(msg)
        self.error_norm_pub.publish(Float32(data=float(error.distance)))
        self.yaw_error_pub.publish(Float32(data=float(error.yaw_error)))

        warn_distance = float(self.get_parameter('warn_error_distance').value)
        if error.distance > warn_distance:
            self.get_logger().warn(
                f"Nozzle position error {error.distance:.4f} m exceeds {warn_distance:.4f} m.",
                throttle_duration_sec=2.0,
            )

    def _select_reference(self) -> Optional[PoseStamped]:
        if self.reference_pose is not None:
            return self.reference_pose
        if self.reference_path is None or not self.reference_path.poses or self.path_index is None:
            return None
        idx = max(0, min(self.path_index, len(self.reference_path.poses) - 1))
        return self.reference_path.poses[idx]

    def _log_waiting(self, reason: str) -> None:
        if reason == self.last_wait_reason:
            return
        self.last_wait_reason = reason
        self.get_logger().info(f"Nozzle pose monitor waiting for {reason}.")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NozzlePoseMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
