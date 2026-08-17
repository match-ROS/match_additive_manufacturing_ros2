#!/usr/bin/env python3
"""Cartesian arm trajectory tracking with path feedforward and pose feedback."""

from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, Float32, Int32
from tf_transformations import quaternion_matrix

from ur_trajectory_follower.direction_control import (
    cartesian_tracking_command,
    limit_vector,
    normalize,
    path_feedforward,
    project_onto_plane,
    segment_speed,
)


class DirectionController(Node):
    def __init__(self) -> None:
        super().__init__('ur_direction_controller')
        self.declare_parameter('path_topic', '/ur_path_transformed')
        self.declare_parameter('reference_pose_topic', '/arm_trajectory_reference')
        self.declare_parameter('current_pose_topic', '/current_deposition_pose')
        self.declare_parameter('path_index_topic', '/path_index')
        self.declare_parameter('velocity_override_topic', '/velocity_override')
        self.declare_parameter('desired_speed_topic', '/desired_arm_speed')
        self.declare_parameter('default_velocity', -1.0)
        # Runtime diagnostic switch: leave reference progression and feedback
        # untouched while suppressing only the path-derivative contribution.
        # This makes A/B checks of a suspected frame error possible without
        # changing the initial conditions of the run.
        self.declare_parameter('feedforward_scale', 1.0)
        self.declare_parameter('start_condition_topic', '/start_condition')
        self.declare_parameter('wait_for_start_condition', True)
        self.declare_parameter('initial_path_index', 0)
        self.declare_parameter('spray_axis_source', 'tool_z')
        self.declare_parameter('spray_axis_sign', 1.0)
        self.declare_parameter('along_track_kp', 2.0)
        self.declare_parameter('orthogonal_kp', 1.0)
        self.declare_parameter('kp_z', 0.7)
        self.declare_parameter('max_along_track_correction', 0.03)
        self.declare_parameter('orthogonal_max_velocity', 0.02)
        self.declare_parameter('max_spray_axis_correction', 0.03)
        self.declare_parameter('max_tracking_linear_velocity', 0.12)
        self.declare_parameter('final_position_tolerance', 0.005)
        self.declare_parameter('final_tolerance_cycles', 3)
        self.declare_parameter('output_smoothing_coeff', 0.0)
        # Tracking must not depend on new path or pose messages arriving.  In
        # particular, the final-position correction has to remain active once
        # trajectory progress has reached the last waypoint.
        self.declare_parameter('control_rate', 100.0)

        self.path: Optional[Path] = None
        self.reference_pose: Optional[PoseStamped] = None
        self.current_pose: Optional[PoseStamped] = None
        self.current_index = max(0, int(self.get_parameter('initial_path_index').value))
        self.velocity_override = 1.0
        self.desired_speed = float(self.get_parameter('default_velocity').value)
        self.control_enabled = not self._as_bool(self.get_parameter('wait_for_start_condition').value)
        self.spray_axis_source = str(self.get_parameter('spray_axis_source').value)
        self.spray_axis_sign = float(self.get_parameter('spray_axis_sign').value)
        # Keep smoothing state per channel. Their sum therefore remains equal
        # to smoothing the final combined command.
        self.feedforward_old = np.zeros(3)
        self.control_old = np.zeros(3)
        self.final_cycles = 0
        self.completed = False

        latch_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, reliability=QoSReliabilityPolicy.RELIABLE)
        self.feedforward_pub = self.create_publisher(Twist, 'ur_twist_world_feedforward', 10)
        self.control_pub = self.create_publisher(Twist, 'ur_twist_world_control', 10)
        # Keep the original combined topic available for existing monitoring
        # and external consumers. The command pipeline itself uses the two
        # components via twist_combiner.
        self.pub = self.create_publisher(Twist, 'ur_twist_world', 10)
        self.complete_pub = self.create_publisher(Bool, 'trajectory_complete', latch_qos)
        self.create_subscription(Path, str(self.get_parameter('path_topic').value), self._path_cb, latch_qos)
        self.create_subscription(PoseStamped, str(self.get_parameter('reference_pose_topic').value), self._reference_cb, latch_qos)
        self.create_subscription(PoseStamped, str(self.get_parameter('current_pose_topic').value), self._pose_cb, 10)
        self.create_subscription(Int32, str(self.get_parameter('path_index_topic').value), self._index_cb, latch_qos)
        self.create_subscription(Float32, str(self.get_parameter('velocity_override_topic').value), self._override_cb, 10)
        self.create_subscription(Float32, str(self.get_parameter('desired_speed_topic').value), self._desired_speed_cb, latch_qos)
        self.create_subscription(Bool, str(self.get_parameter('start_condition_topic').value), self._start_cb, 10)
        control_rate = max(1.0, float(self.get_parameter('control_rate').value))
        self.create_timer(1.0 / control_rate, self._calculate)

    @staticmethod
    def _as_bool(value) -> bool:
        return value if isinstance(value, bool) else str(value).strip().lower() in {'1', 'true', 'yes', 'y', 'on'}

    def _path_cb(self, msg: Path) -> None:
        self.path = msg if msg.poses else None
        if self.path is not None:
            self.current_index = min(self.current_index, len(self.path.poses) - 1)
            self.completed, self.final_cycles = False, 0
        self._calculate()

    def _reference_cb(self, msg: PoseStamped) -> None:
        self.reference_pose = msg
        self._calculate()

    def _pose_cb(self, msg: PoseStamped) -> None:
        self.current_pose = msg
        self._calculate()

    def _index_cb(self, msg: Int32) -> None:
        if self.path is not None:
            self.current_index = max(0, min(int(msg.data), len(self.path.poses) - 1))
        else:
            self.current_index = max(0, int(msg.data))
        self.final_cycles = 0
        self._calculate()

    def _override_cb(self, msg: Float32) -> None:
        self.velocity_override = max(0.0, float(msg.data))
        self._calculate()

    def _desired_speed_cb(self, msg: Float32) -> None:
        self.desired_speed = max(0.0, float(msg.data))
        self._calculate()

    def _start_cb(self, msg: Bool) -> None:
        enabled = bool(msg.data) or not self._as_bool(self.get_parameter('wait_for_start_condition').value)
        if enabled != self.control_enabled:
            self.control_enabled = enabled
            self.final_cycles = 0
            if not enabled:
                self.feedforward_old = np.zeros(3)
                self.control_old = np.zeros(3)
                # The twist combiner retains its latest messages, so clear
                # both channels explicitly when control stops.
                self.feedforward_pub.publish(Twist())
                self.control_pub.publish(Twist())
                self.pub.publish(Twist())
            else:
                self._calculate()

    @staticmethod
    def _position(pose: PoseStamped) -> np.ndarray:
        return np.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z], dtype=float)

    def _spray_axis(self, pose: PoseStamped) -> np.ndarray:
        q = pose.pose.orientation
        quat = np.array([q.x, q.y, q.z, q.w], dtype=float)
        if np.linalg.norm(quat) < 1e-9:
            return np.array([0.0, 0.0, 1.0])
        rotation = quaternion_matrix(quat / np.linalg.norm(quat))
        column = {'tool_x': 0, 'tool_y': 1, 'tool_z': 2}.get(self.spray_axis_source, 2)
        return normalize(rotation[:3, column]) * (-1.0 if self.spray_axis_sign < 0.0 else 1.0)

    def _feedforward(self) -> np.ndarray:
        if self.path is None or self.current_index >= len(self.path.poses) - 1:
            return np.zeros(3)
        start, goal = self.path.poses[self.current_index], self.path.poses[self.current_index + 1]
        delta = self._position(goal) - self._position(start)
        if self.desired_speed > 1e-6:
            speed = self.desired_speed
        else:
            speed = segment_speed(self._position(start), self._position(goal), max(0.0, (goal.header.stamp.sec - start.header.stamp.sec) + (goal.header.stamp.nanosec - start.header.stamp.nanosec) / 1e9))
        return path_feedforward(
            delta,
            speed,
            self.velocity_override * max(
                0.0, float(self.get_parameter('feedforward_scale').value)
            ),
        )

    def _smooth_components(
        self, feedforward: np.ndarray, control: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        coeff = max(0.0, min(1.0, float(self.get_parameter('output_smoothing_coeff').value)))
        feedforward_output = coeff * self.feedforward_old + (1.0 - coeff) * feedforward
        control_output = coeff * self.control_old + (1.0 - coeff) * control
        self.feedforward_old = feedforward_output
        self.control_old = control_output
        return feedforward_output, control_output

    def _calculate(self) -> None:
        if not self.control_enabled or self.path is None or self.reference_pose is None or self.current_pose is None:
            return
        reference, measured = self._position(self.reference_pose), self._position(self.current_pose)
        error = reference - measured
        spray_axis = self._spray_axis(self.reference_pose)
        # The GUI override controls trajectory progress and feedforward speed.
        # Keep pose feedback at its configured bounded rate so tracking
        # stiffness and disturbance recovery do not change with print speed.
        correction_scale = 1.0
        feedforward = self._feedforward()
        tracking = cartesian_tracking_command(
            reference=reference,
            measured=measured,
            tangent=self._segment_tangent(),
            spray_axis=spray_axis,
            feedforward=feedforward,
            along_track_kp=float(self.get_parameter('along_track_kp').value),
            orthogonal_kp=float(self.get_parameter('orthogonal_kp').value),
            spray_kp=float(self.get_parameter('kp_z').value),
            max_along=float(self.get_parameter('max_along_track_correction').value),
            max_orthogonal=float(self.get_parameter('orthogonal_max_velocity').value),
            max_spray=float(self.get_parameter('max_spray_axis_correction').value),
            max_linear=float(self.get_parameter('max_tracking_linear_velocity').value),
            correction_scale=correction_scale,
        )
        control = tracking.along + tracking.lateral + tracking.spray
        command = tracking.command

        final = self.current_index >= len(self.path.poses) - 1
        if final and float(np.linalg.norm(error)) <= float(self.get_parameter('final_position_tolerance').value):
            self.final_cycles += 1
        else:
            self.final_cycles = 0
        if self.final_cycles >= max(1, int(self.get_parameter('final_tolerance_cycles').value)):
            command = np.zeros(3)
            if not self.completed:
                self.completed = True
                self.complete_pub.publish(Bool(data=True))
        else:
            self.complete_pub.publish(Bool(data=False))

        command = limit_vector(command, float(self.get_parameter('max_tracking_linear_velocity').value))

        # The limiter applies to the sum, not to each component. Scale both
        # channels by the same factor so their downstream sum is exactly the
        # bounded command this controller previously published.
        unbounded_command = feedforward + control
        unbounded_norm = float(np.linalg.norm(unbounded_command))
        scale = float(np.linalg.norm(command)) / unbounded_norm if unbounded_norm > 1e-12 else 0.0
        feedforward *= scale
        control *= scale
        feedforward, control = self._smooth_components(feedforward, control)

        feedforward_out = Twist()
        feedforward_out.linear.x, feedforward_out.linear.y, feedforward_out.linear.z = map(float, feedforward)
        control_out = Twist()
        control_out.linear.x, control_out.linear.y, control_out.linear.z = map(float, control)
        self.feedforward_pub.publish(feedforward_out)
        self.control_pub.publish(control_out)
        combined_out = Twist()
        combined_out.linear.x, combined_out.linear.y, combined_out.linear.z = map(
            float, feedforward + control
        )
        self.pub.publish(combined_out)

    def _segment_tangent(self) -> np.ndarray:
        if self.path is None or len(self.path.poses) < 2:
            return np.zeros(3)
        # At the endpoint feedforward is disabled, but the last path tangent
        # remains the meaningful along-track axis for bounded final feedback.
        start_index = min(max(0, self.current_index), len(self.path.poses) - 2)
        return self._position(self.path.poses[start_index + 1]) - self._position(self.path.poses[start_index])


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DirectionController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
