#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Pose, PoseStamped, Twist, TwistStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool

try:
    from move_to_path_idx.move_to_path_idx import (
        ControlState,
        as_bool,
        clamp,
        wrap_to_pi,
        yaw_from_pose,
    )
except ImportError:
    from move_to_path_idx import ControlState, as_bool, clamp, wrap_to_pi, yaw_from_pose


def as_float_list(value, expected_len: int, parameter_name: str) -> list[float]:
    result = [float(item) for item in value]
    if len(result) != expected_len:
        raise ValueError(
            f"{parameter_name} must contain exactly {expected_len} values, got {result}"
        )
    return result


class MoveToPose(Node):
    def __init__(self) -> None:
        super().__init__('move_to_pose')

        self.declare_parameter('robot_pose_topic', '/robot_pose')
        self.declare_parameter('robot_pose_type', 'pose_stamped')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('output_stamped', False)
        self.declare_parameter('command_frame_id', 'base_link')
        self.declare_parameter('diff_drive_mode', True)
        self.declare_parameter('target_position', [0.0, 0.0, 0.0])
        self.declare_parameter('target_orientation', [0.0, 0.0, 0.0, 1.0])
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
        self.declare_parameter('publish_start_condition', False)
        self.declare_parameter('start_condition_topic', '/start_condition')
        self.declare_parameter('start_condition_publish_count', 5)

        target_position = as_float_list(
            self.get_parameter('target_position').value,
            3,
            'target_position',
        )
        target_orientation = as_float_list(
            self.get_parameter('target_orientation').value,
            4,
            'target_orientation',
        )

        self.robot_pose: Optional[Pose] = None
        self.target_pose = Pose()
        self.target_pose.position.x = target_position[0]
        self.target_pose.position.y = target_position[1]
        self.target_pose.position.z = target_position[2]
        self.target_pose.orientation.x = target_orientation[0]
        self.target_pose.orientation.y = target_orientation[1]
        self.target_pose.orientation.z = target_orientation[2]
        self.target_pose.orientation.w = target_orientation[3]
        self.target_yaw = yaw_from_pose(self.target_pose)
        self.state = ControlState.WAITING_FOR_INPUTS
        self.has_logged_waiting = False
        self.start_condition_remaining = 0
        self.output_stamped = as_bool(self.get_parameter('output_stamped').value)
        self.command_frame_id = str(self.get_parameter('command_frame_id').value)
        self.diff_drive_mode = as_bool(self.get_parameter('diff_drive_mode').value)

        robot_pose_topic = str(self.get_parameter('robot_pose_topic').value)
        cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)

        path_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
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
            f"Waiting for robot_pose={robot_pose_topic}; will move once to "
            f"target_position={target_position}, target_yaw={self.target_yaw:.3f} rad "
            f"and publish on {cmd_vel_topic}; diff_drive={self.diff_drive_mode}."
        )

    def _pose_cb(self, msg: Pose) -> None:
        self.robot_pose = msg
        self._maybe_start()

    def _pose_stamped_cb(self, msg: PoseStamped) -> None:
        self.robot_pose = msg.pose
        self._maybe_start()

    def _maybe_start(self) -> None:
        if self.state != ControlState.WAITING_FOR_INPUTS:
            return
        if self.robot_pose is None:
            if not self.has_logged_waiting:
                self.has_logged_waiting = True
                self.get_logger().info("Waiting until robot pose is available.")
            return
        self.state = ControlState.DRIVE_TO_POINT
        self.get_logger().info(
            f"Received robot pose. Moving once to target pose with "
            f"target_yaw={self.target_yaw:.3f} rad."
        )

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

    def _finish(self, dist: float, angle_diff: float) -> None:
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
            f"Reached target pose: dist={dist:.3f}, angle_diff={angle_diff:.3f}. "
            f"{status}; shutting down."
        )

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
        if self.robot_pose is None:
            return

        robot_yaw = yaw_from_pose(self.robot_pose)
        dx = self.target_pose.position.x - self.robot_pose.position.x
        dy = self.target_pose.position.y - self.robot_pose.position.y
        dist = (dx * dx + dy * dy) ** 0.5
        angle_diff = wrap_to_pi(self.target_yaw - robot_yaw)

        cmd = Twist()
        if not self.diff_drive_mode:
            if (
                dist <= float(self.get_parameter('distance_tolerance').value)
                and abs(angle_diff) <= float(self.get_parameter('yaw_tolerance').value)
            ):
                self._finish(dist, angle_diff)
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
                self._finish(dist, angle_diff)
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
    node = MoveToPose()
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
