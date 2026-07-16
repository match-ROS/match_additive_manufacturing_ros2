#!/usr/bin/env python3
import math
from typing import List, Optional

import rclpy
from geometry_msgs.msg import Pose, PoseStamped, Twist, TwistStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, Float32, Int32

from base_trajectory_follower.controller import (
    FollowerGains,
    FollowerLimits,
    FollowerTolerances,
    Pose2D,
    PurePursuitGains,
    compute_pure_pursuit_command,
    compute_velocity_command,
    select_lookahead_index,
)


class SimpleBaseFollower(Node):
    def __init__(self) -> None:
        super().__init__('simple_base_follower')
        self.declare_parameter('path_topic', '/base_path')
        self.declare_parameter('robot_pose_topic', '/robot_pose')
        self.declare_parameter('robot_pose_type', 'pose_stamped')
        self.declare_parameter('cmd_vel_topic', '/robot/robotnik_base_control/cmd_vel_unstamped')
        self.declare_parameter('output_stamped', False)
        self.declare_parameter('command_frame_id', 'base_link')
        self.declare_parameter('use_external_path_index', False)
        self.declare_parameter('path_index_topic', '/path_index')
        self.declare_parameter('reference_pose_topic', '')
        self.declare_parameter('external_path_index_stride', 10)
        self.declare_parameter('wait_for_start_condition', False)
        self.declare_parameter('start_condition_topic', '/start_condition')
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('follower_type', 'pid')
        self.declare_parameter('diff_drive_mode', False)
        self.declare_parameter('velocity_override_topic', '/velocity_override')
        self.declare_parameter('hold_reference_on_pause', True)
        self.declare_parameter('path_time_step', 0.1)
        self.declare_parameter('lookahead_distance', 0.4)
        self.declare_parameter('stale_pose_timeout', 0.5)
        self.declare_parameter('stop_on_goal', True)
        self.declare_parameter('allow_reverse', True)
        self.declare_parameter('kp_x', 0.8)
        self.declare_parameter('kp_y', 0.8)
        self.declare_parameter('kp_yaw', 1.2)
        self.declare_parameter('max_vx', 0.4)
        self.declare_parameter('max_vy', 0.4)
        self.declare_parameter('max_wz', 0.8)
        self.declare_parameter('smooth_velocity_commands', True)
        self.declare_parameter('velocity_smoothing_method', 'accel_limit')
        self.declare_parameter('max_accel_x', 0.25)
        self.declare_parameter('max_accel_y', 0.25)
        self.declare_parameter('max_accel_wz', 0.5)
        self.declare_parameter('moving_average_window_size', 5)
        self.declare_parameter('xy_goal_tolerance', 0.05)
        self.declare_parameter('yaw_goal_tolerance', 0.08)
        self.declare_parameter('pure_pursuit_kv', 1.0)
        self.declare_parameter('pure_pursuit_kw', 1.0)
        self.declare_parameter('pure_pursuit_ky', 0.3)
        self.declare_parameter('pure_pursuit_k_distance', 0.0)
        self.declare_parameter('pure_pursuit_k_orientation', 0.5)
        self.declare_parameter('pure_pursuit_k_index', 0.02)

        self.path: List[Pose2D] = []
        self.path_timestamps: List[float] = []
        self.robot_pose: Optional[Pose2D] = None
        self.last_pose_time = None
        self.current_index = 0
        self.external_path_index: Optional[int] = None
        self.reference_pose: Optional[Pose2D] = None
        self.goal_reached = False
        self.last_stop_reason = ''
        self.velocity_override = 1.0
        self.last_twist = Twist()
        self.last_command_time = None
        self.moving_average_history: List[Twist] = []

        self.path_topic = str(self.get_parameter('path_topic').value)
        self.robot_pose_topic = str(self.get_parameter('robot_pose_topic').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.output_stamped = self._as_bool(self.get_parameter('output_stamped').value)
        self.command_frame_id = str(self.get_parameter('command_frame_id').value)
        self.use_external_path_index = self._as_bool(self.get_parameter('use_external_path_index').value)
        self.follower_type = str(self.get_parameter('follower_type').value).strip().lower()
        self.diff_drive_mode = self._as_bool(self.get_parameter('diff_drive_mode').value)
        self.wait_for_start_condition = self._as_bool(
            self.get_parameter('wait_for_start_condition').value
        )
        self.control_enabled = not self.wait_for_start_condition

        latch_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(Path, self.path_topic, self._path_cb, latch_qos)
        if self.use_external_path_index:
            self.create_subscription(
                Int32,
                str(self.get_parameter('path_index_topic').value),
                self._path_index_cb,
                10,
            )
        reference_pose_topic = str(self.get_parameter('reference_pose_topic').value).strip()
        if reference_pose_topic:
            self.create_subscription(PoseStamped, reference_pose_topic, self._reference_pose_cb, latch_qos)
        self.create_subscription(
            Bool,
            str(self.get_parameter('start_condition_topic').value),
            self._start_condition_cb,
            10,
        )
        self.create_subscription(
            Float32,
            str(self.get_parameter('velocity_override_topic').value),
            self._velocity_override_cb,
            10,
        )

        pose_type = str(self.get_parameter('robot_pose_type').value).strip().lower()
        if pose_type in {'pose', 'geometry_msgs/msg/pose'}:
            self.create_subscription(Pose, self.robot_pose_topic, self._pose_cb, 10)
        else:
            self.create_subscription(PoseStamped, self.robot_pose_topic, self._pose_stamped_cb, 10)

        if self.output_stamped:
            self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_vel_topic, 10)
        else:
            self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        rate = max(1.0, float(self.get_parameter('publish_rate').value))
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"Simple base follower waiting for path={self.path_topic}, "
            f"pose={self.robot_pose_topic}, cmd={self.cmd_vel_topic}, "
            f"type={self.follower_type}, diff_drive={self.diff_drive_mode}."
        )

    def _path_cb(self, msg: Path) -> None:
        poses = [self._pose2d_from_pose_stamped(pose) for pose in msg.poses]
        timestamps = [
            float(pose.header.stamp.sec) + float(pose.header.stamp.nanosec) / 1e9
            for pose in msg.poses
        ]
        self.path = poses
        self.path_timestamps = timestamps
        self.current_index = 0
        if self.external_path_index is not None and self.path:
            self.current_index = self._path_index_from_external_index(self.external_path_index)
        self.goal_reached = False
        self.get_logger().info(f"Received base path with {len(self.path)} poses.")

    def _path_index_cb(self, msg: Int32) -> None:
        self.external_path_index = max(0, int(msg.data))

    def _reference_pose_cb(self, msg: PoseStamped) -> None:
        self.reference_pose = self._pose2d_from_pose(msg.pose)

    def _start_condition_cb(self, msg: Bool) -> None:
        was_enabled = self.control_enabled
        self.control_enabled = bool(msg.data) or not self.wait_for_start_condition
        if self.control_enabled and not was_enabled:
            self.get_logger().info("Base follower start condition received.")
        elif was_enabled and not self.control_enabled:
            self._publish_stop('start condition disabled')

    def _velocity_override_cb(self, msg: Float32) -> None:
        self.velocity_override = max(0.0, float(msg.data))

    def _pose_cb(self, msg: Pose) -> None:
        self.robot_pose = self._pose2d_from_pose(msg)
        self.last_pose_time = self.get_clock().now()

    def _pose_stamped_cb(self, msg: PoseStamped) -> None:
        self.robot_pose = self._pose2d_from_pose(msg.pose)
        self.last_pose_time = self.get_clock().now()

    def _tick(self) -> None:
        if not self.control_enabled:
            return
        if not self.path:
            self._publish_stop('no path')
            return
        if self.robot_pose is None or self.last_pose_time is None:
            self._publish_stop('no pose')
            return
        if self._pose_is_stale():
            self._publish_stop('stale pose')
            return
        if self.goal_reached and self._as_bool(self.get_parameter('stop_on_goal').value):
            self._publish_stop('goal reached')
            return

        if self.use_external_path_index:
            if self.external_path_index is None:
                self._publish_stop('no path index')
                return
            self.current_index = self._path_index_from_external_index(self.external_path_index)
        else:
            lookahead = float(self.get_parameter('lookahead_distance').value)
            self.current_index = select_lookahead_index(
                self.path,
                self.robot_pose,
                lookahead,
                self.current_index,
            )
        lookahead = float(self.get_parameter('lookahead_distance').value)
        target_index = self.current_index
        if self.follower_type == 'pure_pursuit' and self.reference_pose is None:
            target_index = select_lookahead_index(
                self.path,
                self.robot_pose,
                lookahead,
                self.current_index,
            )
            command = compute_pure_pursuit_command(
                self.robot_pose,
                self.path,
                self.current_index,
                target_index,
                self.path_timestamps,
                self._pure_pursuit_gains(),
                self._limits(),
                self._tolerances(),
                self.velocity_override,
                float(self.get_parameter('path_time_step').value),
                self.diff_drive_mode,
            )
        else:
            # With the shared external index, that index is the progress
            # contract.  A PID follower must regulate to this tracking point,
            # not to a lookahead point: otherwise the 0.3 m lookahead creates
            # a large initial P error and immediately drives at max velocity.
            # Pure Pursuit still uses lookahead for its steering direction.
            target_index = self.current_index
            target_pose = self.reference_pose or self.path[target_index]
            command_override = self.velocity_override
            if command_override <= 0.0 and self._as_bool(self.get_parameter('hold_reference_on_pause').value):
                command_override = 1.0
            command = compute_velocity_command(
                self.robot_pose,
                target_pose,
                self.path[-1],
                self._gains(),
                self._limits(),
                self._tolerances(),
                self.diff_drive_mode,
                command_override,
            )
        self.goal_reached = command.reached_goal
        if self.goal_reached:
            self._publish_stop('goal reached')
            self.get_logger().info("Base path goal reached.", throttle_duration_sec=2.0)
            return

        twist = Twist()
        twist.linear.x = command.vx if self._as_bool(self.get_parameter('allow_reverse').value) else max(0.0, command.vx)
        twist.linear.y = 0.0 if self.diff_drive_mode else command.vy
        twist.angular.z = command.wz
        self._publish_twist(self._smooth_twist(twist))
        self.last_stop_reason = ''

    def _pose_is_stale(self) -> bool:
        timeout = float(self.get_parameter('stale_pose_timeout').value)
        age = (self.get_clock().now() - self.last_pose_time).nanoseconds / 1e9
        return age > timeout

    def _path_index_from_external_index(self, external_index: int) -> int:
        stride = max(1, int(self.get_parameter('external_path_index_stride').value))
        return max(0, min(int(external_index) * stride, len(self.path) - 1))

    def _publish_stop(self, reason: str) -> None:
        self._reset_smoother()
        self._publish_twist(Twist())
        if reason != self.last_stop_reason:
            self.get_logger().warn(f"Publishing zero velocity: {reason}.")
            self.last_stop_reason = reason

    def _smooth_twist(self, target: Twist) -> Twist:
        if not self._as_bool(self.get_parameter('smooth_velocity_commands').value):
            self.last_twist = target
            self.last_command_time = self.get_clock().now()
            self.moving_average_history = []
            return target

        method = str(self.get_parameter('velocity_smoothing_method').value).strip().lower()
        if method in {'moving_average', 'moving-average', 'average', 'mean'}:
            return self._smooth_twist_moving_average(target)
        if method not in {'accel_limit', 'acceleration_limit', 'slew_rate', 'slew'}:
            self.get_logger().warn(
                f"Unknown velocity_smoothing_method '{method}', using accel_limit.",
                throttle_duration_sec=5.0,
            )

        now = self.get_clock().now()
        if self.last_command_time is None:
            publish_rate = max(1.0, float(self.get_parameter('publish_rate').value))
            dt = 1.0 / publish_rate
        else:
            dt = (now - self.last_command_time).nanoseconds / 1e9
            if dt <= 0.0:
                publish_rate = max(1.0, float(self.get_parameter('publish_rate').value))
                dt = 1.0 / publish_rate

        out = Twist()
        out.linear.x = self._slew(
            self.last_twist.linear.x,
            target.linear.x,
            float(self.get_parameter('max_accel_x').value),
            dt,
        )
        out.linear.y = self._slew(
            self.last_twist.linear.y,
            target.linear.y,
            float(self.get_parameter('max_accel_y').value),
            dt,
        )
        out.angular.z = self._slew(
            self.last_twist.angular.z,
            target.angular.z,
            float(self.get_parameter('max_accel_wz').value),
            dt,
        )
        self.last_twist = out
        self.last_command_time = now
        return out

    def _smooth_twist_moving_average(self, target: Twist) -> Twist:
        window_size = max(1, int(self.get_parameter('moving_average_window_size').value))
        sample = Twist()
        sample.linear.x = target.linear.x
        sample.linear.y = target.linear.y
        sample.angular.z = target.angular.z

        self.moving_average_history.append(sample)
        self.moving_average_history = self.moving_average_history[-window_size:]

        out = Twist()
        count = float(len(self.moving_average_history))
        out.linear.x = sum(twist.linear.x for twist in self.moving_average_history) / count
        out.linear.y = sum(twist.linear.y for twist in self.moving_average_history) / count
        out.angular.z = sum(twist.angular.z for twist in self.moving_average_history) / count
        self.last_twist = out
        self.last_command_time = self.get_clock().now()
        return out

    def _reset_smoother(self) -> None:
        self.last_twist = Twist()
        self.last_command_time = None
        self.moving_average_history = []

    @staticmethod
    def _slew(previous: float, target: float, max_rate: float, dt: float) -> float:
        max_delta = abs(float(max_rate)) * max(0.0, float(dt))
        if max_delta <= 0.0:
            return float(target)
        delta = float(target) - float(previous)
        if abs(delta) <= max_delta:
            return float(target)
        return float(previous) + math.copysign(max_delta, delta)

    def _publish_twist(self, twist: Twist) -> None:
        if self.output_stamped:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.command_frame_id
            msg.twist = twist
            self.cmd_pub.publish(msg)
        else:
            self.cmd_pub.publish(twist)

    def _gains(self) -> FollowerGains:
        return FollowerGains(
            kp_x=float(self.get_parameter('kp_x').value),
            kp_y=float(self.get_parameter('kp_y').value),
            kp_yaw=float(self.get_parameter('kp_yaw').value),
        )

    def _limits(self) -> FollowerLimits:
        return FollowerLimits(
            max_vx=float(self.get_parameter('max_vx').value),
            max_vy=float(self.get_parameter('max_vy').value),
            max_wz=float(self.get_parameter('max_wz').value),
        )

    def _tolerances(self) -> FollowerTolerances:
        return FollowerTolerances(
            xy_goal_tolerance=float(self.get_parameter('xy_goal_tolerance').value),
            yaw_goal_tolerance=float(self.get_parameter('yaw_goal_tolerance').value),
        )

    def _pure_pursuit_gains(self) -> PurePursuitGains:
        return PurePursuitGains(
            kv=float(self.get_parameter('pure_pursuit_kv').value),
            kw=float(self.get_parameter('pure_pursuit_kw').value),
            ky=float(self.get_parameter('pure_pursuit_ky').value),
            k_distance=float(self.get_parameter('pure_pursuit_k_distance').value),
            k_orientation=float(self.get_parameter('pure_pursuit_k_orientation').value),
            k_index=float(self.get_parameter('pure_pursuit_k_index').value),
        )

    @staticmethod
    def _pose2d_from_pose_stamped(msg: PoseStamped) -> Pose2D:
        return SimpleBaseFollower._pose2d_from_pose(msg.pose)

    @staticmethod
    def _pose2d_from_pose(msg: Pose) -> Pose2D:
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return Pose2D(float(msg.position.x), float(msg.position.y), yaw)

    @staticmethod
    def _as_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in {'1', 'true', 'yes', 'y', 'on'}


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimpleBaseFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node._publish_twist(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
