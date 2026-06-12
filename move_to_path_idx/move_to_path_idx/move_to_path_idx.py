#!/usr/bin/env python3
import math
from enum import Enum
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Pose, PoseStamped, Twist, TwistStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool


class ControlState(Enum):
    WAITING_FOR_INPUTS = 0
    DRIVE_TO_POINT = 1
    REORIENT = 2
    DONE = 3


def wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_pose(pose: Pose) -> float:
    q = pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {'1', 'true', 'yes', 'y', 'on'}


class MoveToPathIdx(Node):
    def __init__(self) -> None:
        super().__init__('move_to_path_idx')

        self.declare_parameter('path_topic', '/mobile_base_path')
        self.declare_parameter('robot_pose_topic', '/robot_pose')
        self.declare_parameter('robot_pose_type', 'pose_stamped')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('output_stamped', False)
        self.declare_parameter('command_frame_id', 'base_link')
        self.declare_parameter('diff_drive_mode', True)
        self.declare_parameter('path_index', 0)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('distance_tolerance', 0.05)
        self.declare_parameter('yaw_tolerance', 0.05)
        self.declare_parameter('kp_linear', 0.6)
        self.declare_parameter('kp_lateral', 0.6)
        self.declare_parameter('kp_angular_to_point', 1.5)
        self.declare_parameter('kp_angular_reorient', 1.2)
        self.declare_parameter('max_linear_velocity', 0.25)
        self.declare_parameter('max_lateral_velocity', 0.25)
        self.declare_parameter('max_angular_velocity', 0.6)
        self.declare_parameter('drive_heading_threshold', 0.6)
        self.declare_parameter('target_yaw_mode', 'auto')
        self.declare_parameter('target_yaw_lookahead_points', 10)
        self.declare_parameter('publish_start_condition', False)
        self.declare_parameter('start_condition_topic', '/start_condition')
        self.declare_parameter('start_condition_publish_count', 5)

        self.path: Optional[Path] = None
        self.robot_pose: Optional[Pose] = None
        self.path_index = max(0, int(self.get_parameter('path_index').value))
        self.state = ControlState.WAITING_FOR_INPUTS
        self.target_pose: Optional[Pose] = None
        self.target_yaw: Optional[float] = None
        self.has_logged_waiting = False
        self.start_condition_remaining = 0
        self.output_stamped = as_bool(self.get_parameter('output_stamped').value)
        self.command_frame_id = str(self.get_parameter('command_frame_id').value)
        self.diff_drive_mode = as_bool(self.get_parameter('diff_drive_mode').value)

        path_topic = str(self.get_parameter('path_topic').value)
        robot_pose_topic = str(self.get_parameter('robot_pose_topic').value)
        cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)

        path_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(Path, path_topic, self._path_cb, path_qos)
        self.start_condition_pub = self.create_publisher(
            Bool,
            str(self.get_parameter('start_condition_topic').value),
            path_qos,
        )

        pose_type = str(self.get_parameter('robot_pose_type').value).strip().lower()
        if pose_type in {'pose', 'geometry_msgs/msg/pose'}:
            self.create_subscription(Pose, robot_pose_topic, self._pose_cb, 10)
        else:
            self.create_subscription(PoseStamped, robot_pose_topic, self._pose_stamped_cb, 10)

        if self.output_stamped:
            self.cmd_vel_pub = self.create_publisher(TwistStamped, cmd_vel_topic, 10)
        else:
            self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        rate = max(1.0, float(self.get_parameter('publish_rate').value))
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f"Waiting for path={path_topic} and robot_pose={robot_pose_topic}; "
            f"will move once to startup path_index={self.path_index} and publish on {cmd_vel_topic}; "
            f"diff_drive={self.diff_drive_mode}."
        )

    def _path_cb(self, msg: Path) -> None:
        if not msg.poses:
            self.get_logger().warn("Ignoring empty path.")
            return
        self.path = msg
        self._maybe_start()

    def _pose_cb(self, msg: Pose) -> None:
        self.robot_pose = msg
        self._maybe_start()

    def _pose_stamped_cb(self, msg: PoseStamped) -> None:
        self.robot_pose = msg.pose
        self._maybe_start()

    def _maybe_start(self) -> None:
        if self.state != ControlState.WAITING_FOR_INPUTS:
            return
        if self.path is None or self.robot_pose is None:
            if not self.has_logged_waiting:
                self.has_logged_waiting = True
                self.get_logger().info("Waiting until both path and robot pose are available.")
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
        self.target_yaw = self._target_yaw_for_index()
        self.state = ControlState.DRIVE_TO_POINT
        self.get_logger().info(
            f"Received path and robot pose. Moving once to path index {self.path_index} "
            f"with target_yaw={self.target_yaw:.3f} rad."
        )

    def _target_yaw_for_index(self) -> float:
        assert self.path is not None
        assert self.target_pose is not None

        mode = str(self.get_parameter('target_yaw_mode').value).strip().lower()
        if mode in {'pose', 'path_pose', 'path'}:
            return yaw_from_pose(self.target_pose)
        if mode == 'auto' and not self.diff_drive_mode:
            return yaw_from_pose(self.target_pose)

        lookahead = max(1, int(self.get_parameter('target_yaw_lookahead_points').value))
        current_idx = max(0, min(self.path_index, len(self.path.poses) - 1))
        next_idx = min(len(self.path.poses) - 1, current_idx + lookahead)
        previous_idx = max(0, current_idx - lookahead)

        start = self.path.poses[current_idx].pose.position
        end = self.path.poses[next_idx].pose.position
        dx = end.x - start.x
        dy = end.y - start.y

        if math.hypot(dx, dy) <= 1e-6 and previous_idx != current_idx:
            previous = self.path.poses[previous_idx].pose.position
            dx = start.x - previous.x
            dy = start.y - previous.y

        if math.hypot(dx, dy) > 1e-6:
            return math.atan2(dy, dx)
        return yaw_from_pose(self.target_pose)

    def _publish_stop(self) -> None:
        self._publish_twist(Twist())

    def _publish_twist(self, twist: Twist) -> None:
        if self.output_stamped:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.command_frame_id
            msg.twist = twist
            self.cmd_vel_pub.publish(msg)
        else:
            self.cmd_vel_pub.publish(twist)

    def _publish_start_condition(self) -> None:
        if as_bool(self.get_parameter('publish_start_condition').value):
            self.start_condition_pub.publish(Bool(data=True))

    def _start_condition_enabled(self) -> bool:
        return as_bool(self.get_parameter('publish_start_condition').value)

    def _tick(self) -> None:
        if self.state == ControlState.WAITING_FOR_INPUTS:
            self._maybe_start()
            return
        if self.state == ControlState.DONE:
            self._publish_stop()
            if self.start_condition_remaining > 0:
                self._publish_start_condition()
                self.start_condition_remaining -= 1
                return
            rclpy.shutdown()
            return
        if self.target_pose is None or self.robot_pose is None:
            return

        robot_yaw = yaw_from_pose(self.robot_pose)
        dx = self.target_pose.position.x - self.robot_pose.position.x
        dy = self.target_pose.position.y - self.robot_pose.position.y
        dist = math.sqrt(dx * dx + dy * dy)
        target_yaw = self.target_yaw if self.target_yaw is not None else yaw_from_pose(self.target_pose)
        angle_diff = wrap_to_pi(target_yaw - robot_yaw)

        cmd = Twist()
        if not self.diff_drive_mode:
            if (
                dist <= float(self.get_parameter('distance_tolerance').value)
                and abs(angle_diff) <= float(self.get_parameter('yaw_tolerance').value)
            ):
                self.state = ControlState.DONE
                self._publish_stop()
                self.start_condition_remaining = max(
                    1,
                    int(self.get_parameter('start_condition_publish_count').value),
                )
                self._publish_start_condition()
                self.start_condition_remaining -= 1
                self.get_logger().info(
                    f"Reached path index {self.path_index}: dist={dist:.3f}, "
                    f"angle_diff={angle_diff:.3f}; shutting down."
                )
                return

            cos_yaw = math.cos(robot_yaw)
            sin_yaw = math.sin(robot_yaw)
            dx_robot = cos_yaw * dx + sin_yaw * dy
            dy_robot = -sin_yaw * dx + cos_yaw * dy
            cmd.linear.x = clamp(
                float(self.get_parameter('kp_linear').value) * dx_robot,
                float(self.get_parameter('max_linear_velocity').value),
            )
            cmd.linear.y = clamp(
                float(self.get_parameter('kp_lateral').value) * dy_robot,
                float(self.get_parameter('max_lateral_velocity').value),
            )
            cmd.angular.z = clamp(
                float(self.get_parameter('kp_angular_reorient').value) * angle_diff,
                float(self.get_parameter('max_angular_velocity').value),
            )
            self._publish_twist(cmd)
            return

        if self.state == ControlState.DRIVE_TO_POINT:
            if dist <= float(self.get_parameter('distance_tolerance').value):
                self.state = ControlState.REORIENT
                self._publish_stop()
                return

            heading_to_target = math.atan2(dy, dx)
            heading_error = wrap_to_pi(heading_to_target - robot_yaw)
            max_linear = float(self.get_parameter('max_linear_velocity').value)
            max_angular = float(self.get_parameter('max_angular_velocity').value)
            heading_threshold = abs(float(self.get_parameter('drive_heading_threshold').value))

            if abs(heading_error) <= heading_threshold:
                cmd.linear.x = min(max_linear, float(self.get_parameter('kp_linear').value) * dist)
            cmd.angular.z = clamp(
                float(self.get_parameter('kp_angular_to_point').value) * heading_error,
                max_angular,
            )
            self._publish_twist(cmd)
            return

        if self.state == ControlState.REORIENT:
            if abs(angle_diff) <= float(self.get_parameter('yaw_tolerance').value):
                self.state = ControlState.DONE
                self._publish_stop()
                self.start_condition_remaining = max(
                    1,
                    int(self.get_parameter('start_condition_publish_count').value),
                )
                self._publish_start_condition()
                self.start_condition_remaining -= 1
                status = (
                    f"Published reached signal on {self.start_condition_pub.topic_name}"
                    if self._start_condition_enabled()
                    else "No reached signal configured"
                )
                self.get_logger().info(
                    f"Reached path index {self.path_index}: dist={dist:.3f}, "
                    f"angle_diff={angle_diff:.3f}. {status}; shutting down."
                )
            else:
                cmd.angular.z = clamp(
                    float(self.get_parameter('kp_angular_reorient').value) * angle_diff,
                    float(self.get_parameter('max_angular_velocity').value),
                )
                self._publish_twist(cmd)
            return

        self._publish_stop()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MoveToPathIdx()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        if rclpy.ok():
            node._publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
