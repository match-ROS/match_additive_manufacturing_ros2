#!/usr/bin/env python3
import math
from enum import Enum
from typing import Optional

import rclpy
from geometry_msgs.msg import Pose, PoseStamped, TwistStamped
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool

from move_to_path_idx.move_to_path_idx import as_bool, clamp


class ControlState(Enum):
    WAITING_FOR_INPUTS = 0
    DRIVE_TO_POINT = 1
    ALIGN_ORIENTATION = 2
    DONE = 3


def vector_norm(x: float, y: float, z: float) -> float:
    return math.sqrt(x * x + y * y + z * z)


def scale_to_limit(x: float, y: float, z: float, limit: float) -> tuple[float, float, float]:
    norm = vector_norm(x, y, z)
    if norm <= 1e-9 or norm <= limit:
        return x, y, z
    scale = limit / norm
    return x * scale, y * scale, z * scale


def normalized_quaternion(pose: Pose) -> tuple[float, float, float, float]:
    q = pose.orientation
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if norm <= 1e-9:
        return 0.0, 0.0, 0.0, 1.0
    return q.x / norm, q.y / norm, q.z / norm, q.w / norm


def quaternion_inverse(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    return -q[0], -q[1], -q[2], q[3]


def quaternion_multiply(
    a: tuple[float, float, float, float],
    b: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def orientation_error_vector(target_pose: Pose, current_pose: Pose) -> tuple[float, float, float, float]:
    q_target = normalized_quaternion(target_pose)
    q_current = normalized_quaternion(current_pose)
    q_error = quaternion_multiply(q_target, quaternion_inverse(q_current))
    if q_error[3] < 0.0:
        q_error = (-q_error[0], -q_error[1], -q_error[2], -q_error[3])

    w = max(-1.0, min(1.0, q_error[3]))
    angle = 2.0 * math.acos(w)
    sin_half = math.sqrt(max(1.0 - w * w, 0.0))
    if sin_half < 1e-6:
        return 0.0, 0.0, 0.0, 0.0

    axis_x = q_error[0] / sin_half
    axis_y = q_error[1] / sin_half
    axis_z = q_error[2] / sin_half
    return axis_x * angle, axis_y * angle, axis_z * angle, abs(angle)


class MoveUrToPathIdx(Node):
    def __init__(self) -> None:
        super().__init__('move_ur_to_path_idx')

        self.declare_parameter('path_topic', '/ur_path_transformed')
        self.declare_parameter('current_pose_topic', '/current_tcp_pose')
        self.declare_parameter('path_index', 0)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('distance_tolerance', 0.005)
        self.declare_parameter('orientation_tolerance', 0.06)
        self.declare_parameter('yaw_tolerance', 0.06)
        self.declare_parameter('kp_linear', 0.8)
        self.declare_parameter('kp_angular', 1.0)
        self.declare_parameter('kp_angular_to_point', 1.2)
        self.declare_parameter('kp_angular_reorient', 1.0)
        self.declare_parameter('max_linear_velocity', 0.12)
        self.declare_parameter('max_angular_velocity', 0.5)
        self.declare_parameter('drive_heading_threshold', 0.6)
        self.declare_parameter('publish_stop_count', 3)
        self.declare_parameter('wait_for_start_condition', True)
        self.declare_parameter('start_condition_topic', '/start_pose_reached')
        self.declare_parameter('ready_topic', '')
        self.declare_parameter('completion_topic', '')
        self.declare_parameter('cmd_vel_topic', '/jparse_velocity_controller_ur/twist_cmd_world')
        self.declare_parameter('path_frame', 'map')

        self.path: Optional[Path] = None
        self.robot_pose: Optional[Pose] = None
        self.path_index = max(0, int(self.get_parameter('path_index').value))
        self.state = ControlState.WAITING_FOR_INPUTS
        self.target_pose: Optional[Pose] = None
        self.has_logged_waiting = False
        self.stop_count_remaining = 0
        self.path_frame = str(self.get_parameter('path_frame').value)
        self.command_frame = self.path_frame

        self.wait_for_start_condition = as_bool(
            self.get_parameter('wait_for_start_condition').value
        )
        self.start_enabled = not self.wait_for_start_condition

        path_topic = str(self.get_parameter('path_topic').value)
        current_pose_topic = str(self.get_parameter('current_pose_topic').value)
        cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)

        path_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(Path, path_topic, self._path_cb, path_qos)
        self.create_subscription(PoseStamped, current_pose_topic, self._pose_cb, 10)
        self.create_subscription(
            Bool,
            str(self.get_parameter('start_condition_topic').value),
            self._start_cb,
            10,
        )
        # The simulated base mover publishes /start_pose_reached with
        # transient-local QoS.  Subscribe with that QoS too, so an arm mover
        # that starts just after a base already at its start pose still gets
        # the completion signal.  The volatile subscription above preserves
        # compatibility with manual/default-QoS publishers.
        self.create_subscription(
            Bool,
            str(self.get_parameter('start_condition_topic').value),
            self._start_cb,
            path_qos,
        )
        ready_topic = str(self.get_parameter('ready_topic').value).strip()
        self.ready_pub = (
            self.create_publisher(Bool, ready_topic, path_qos) if ready_topic else None
        )
        self.cmd_vel_pub = self.create_publisher(TwistStamped, cmd_vel_topic, 10)
        completion_topic = str(self.get_parameter('completion_topic').value).strip()
        self.completion_pub = (
            self.create_publisher(Bool, completion_topic, 10) if completion_topic else None
        )
        self.completion_published = False

        rate = max(1.0, float(self.get_parameter('publish_rate').value))
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f"Waiting for path={path_topic} and current_pose={current_pose_topic}; "
            f"will move to path_index={self.path_index} and publish on {cmd_vel_topic}."
        )
        if self.ready_pub is not None:
            # Publish only after the start-condition subscriptions exist.
            self.ready_pub.publish(Bool(data=True))

    def _path_cb(self, msg: Path) -> None:
        if not msg.poses:
            self.get_logger().warn("Ignoring empty path.")
            return
        self.path = msg
        if msg.header.frame_id:
            self.command_frame = msg.header.frame_id
        self._maybe_start()

    def _pose_cb(self, msg: PoseStamped) -> None:
        self.robot_pose = msg.pose
        self._maybe_start()

    def _start_cb(self, msg: Bool) -> None:
        self.start_enabled = bool(msg.data) or not self.wait_for_start_condition
        if self.start_enabled:
            self._maybe_start()

    def _maybe_start(self) -> None:
        if self.state != ControlState.WAITING_FOR_INPUTS:
            return
        if not self.start_enabled:
            return
        if self.path is None or self.robot_pose is None:
            if not self.has_logged_waiting:
                self.has_logged_waiting = True
                self.get_logger().info("Waiting until both path and current controlled pose are available.")
            return
        if self.path_index >= len(self.path.poses):
            self.get_logger().error(
                f"path_index {self.path_index} is out of range for path with "
                f"{len(self.path.poses)} poses."
            )
            self._publish_stop()
            rclpy.shutdown()
            return

        self.target_pose = self.path.poses[self.path_index].pose
        self.state = ControlState.DRIVE_TO_POINT
        self.get_logger().info(
            f"Received path and controlled pose. Moving to path index {self.path_index}."
        )

    def _publish_stop(self) -> None:
        stop = TwistStamped()
        stop.header.frame_id = self.command_frame
        stop.header.stamp = self.get_clock().now().to_msg()
        self.cmd_vel_pub.publish(stop)

    def _publish_completion(self) -> None:
        if self.completion_pub is not None and not self.completion_published:
            self.completion_pub.publish(Bool(data=True))
            self.completion_published = True

    def _orientation_tolerance(self) -> float:
        value = self.get_parameter('orientation_tolerance').value
        # Keep compatibility with older launch files that only configured yaw_tolerance.
        if value is None:
            value = self.get_parameter('yaw_tolerance').value
        return float(value)

    def _tick(self) -> None:
        if self.state == ControlState.WAITING_FOR_INPUTS:
            self._maybe_start()
            return
        if self.state == ControlState.DONE:
            self._publish_stop()
            if self.stop_count_remaining > 0:
                self.stop_count_remaining -= 1
                return
            rclpy.shutdown()
            return
        if self.target_pose is None or self.robot_pose is None:
            return

        dx = self.target_pose.position.x - self.robot_pose.position.x
        dy = self.target_pose.position.y - self.robot_pose.position.y
        dz = self.target_pose.position.z - self.robot_pose.position.z
        dist = vector_norm(dx, dy, dz)

        ox, oy, oz, orientation_error = orientation_error_vector(self.target_pose, self.robot_pose)

        cmd = TwistStamped()
        cmd.header.frame_id = self.command_frame
        cmd.header.stamp = self.get_clock().now().to_msg()

        if self.state == ControlState.DRIVE_TO_POINT:
            if dist <= float(self.get_parameter('distance_tolerance').value):
                self.state = ControlState.ALIGN_ORIENTATION
                self._publish_stop()
                return

            max_linear = float(self.get_parameter('max_linear_velocity').value)
            vx = float(self.get_parameter('kp_linear').value) * dx
            vy = float(self.get_parameter('kp_linear').value) * dy
            vz = float(self.get_parameter('kp_linear').value) * dz
            vx, vy, vz = scale_to_limit(vx, vy, vz, max_linear)
            cmd.twist.linear.x = float(vx)
            cmd.twist.linear.y = float(vy)
            cmd.twist.linear.z = float(vz)
            # The controlled point is the deposition point, 0.35 m ahead of
            # the flange.  Keeping the flange orientation fixed until that
            # point has reached the target can drive the remote-point Jacobian
            # into a poor configuration.  Start reducing the pose error as a
            # whole, so the tool takes the coupled approach to the target.
            kp_angular = float(self.get_parameter('kp_angular').value)
            max_angular = float(self.get_parameter('max_angular_velocity').value)
            wx, wy, wz = scale_to_limit(
                kp_angular * ox,
                kp_angular * oy,
                kp_angular * oz,
                max_angular,
            )
            cmd.twist.angular.x = float(wx)
            cmd.twist.angular.y = float(wy)
            cmd.twist.angular.z = float(wz)
            self.cmd_vel_pub.publish(cmd)
            return

        if self.state == ControlState.ALIGN_ORIENTATION:
            distance_tolerance = float(self.get_parameter('distance_tolerance').value)
            orientation_tolerance = self._orientation_tolerance()
            if dist <= distance_tolerance and orientation_error <= orientation_tolerance:
                self.state = ControlState.DONE
                self.stop_count_remaining = max(1, int(self.get_parameter('publish_stop_count').value))
                self._publish_stop()
                self.get_logger().info(
                    f"Reached controlled-pose target index {self.path_index}: dist={dist:.3f}, "
                    f"orientation_error={orientation_error:.3f}. Shutting down."
                )
                self._publish_completion()
            else:
                max_linear = float(self.get_parameter('max_linear_velocity').value)
                kp_linear = float(self.get_parameter('kp_linear').value)
                vx, vy, vz = scale_to_limit(
                    kp_linear * dx,
                    kp_linear * dy,
                    kp_linear * dz,
                    max_linear,
                )
                kp_angular = float(self.get_parameter('kp_angular').value)
                max_angular = float(self.get_parameter('max_angular_velocity').value)
                wx, wy, wz = scale_to_limit(
                    kp_angular * ox,
                    kp_angular * oy,
                    kp_angular * oz,
                    max_angular,
                )
                cmd.twist.linear.x = float(vx)
                cmd.twist.linear.y = float(vy)
                cmd.twist.linear.z = float(vz)
                cmd.twist.angular.x = float(wx)
                cmd.twist.angular.y = float(wy)
                cmd.twist.angular.z = float(wz)
                self.cmd_vel_pub.publish(cmd)
            return

        self._publish_stop()


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[MoveUrToPathIdx] = None
    try:
        node = MoveUrToPathIdx()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    finally:
        if node is not None:
            # The controller transitions itself to DONE by publishing a zero
            # command.  Do not try to publish again after its normal
            # rclpy.shutdown(), which turns an otherwise successful
            # move-to-start into an invalid-context traceback.
            if rclpy.ok():
                node._publish_stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
