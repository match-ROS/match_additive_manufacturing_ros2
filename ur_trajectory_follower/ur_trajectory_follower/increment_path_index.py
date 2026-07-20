#!/usr/bin/env python3
"""Shared, coupled arm/base trajectory progress.

The historical status output is an ``Int32 /path_index``. External index
requests arrive separately on ``/path_index_command`` so this node never
interprets delayed copies of its own status as commands. In segment modes the
index identifies the start of the active segment and ``phase`` is the common
progress through that segment.
"""

from copy import deepcopy
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Vector3
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, Float32, Int32


def stamp_seconds(pose: PoseStamped) -> float:
    return float(pose.header.stamp.sec) + float(pose.header.stamp.nanosec) / 1e9


def interpolate_pose(start: PoseStamped, goal: PoseStamped, phase: float) -> PoseStamped:
    """Linearly interpolate position and SLERP the quaternion without mutating a path."""
    alpha = max(0.0, min(1.0, float(phase)))
    result = deepcopy(start)
    result.pose.position.x = (1.0 - alpha) * start.pose.position.x + alpha * goal.pose.position.x
    result.pose.position.y = (1.0 - alpha) * start.pose.position.y + alpha * goal.pose.position.y
    result.pose.position.z = (1.0 - alpha) * start.pose.position.z + alpha * goal.pose.position.z

    q0 = np.array([start.pose.orientation.x, start.pose.orientation.y, start.pose.orientation.z, start.pose.orientation.w], dtype=float)
    q1 = np.array([goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w], dtype=float)
    q0_norm, q1_norm = np.linalg.norm(q0), np.linalg.norm(q1)
    if q0_norm < 1e-9 or q1_norm < 1e-9:
        return result
    q0, q1 = q0 / q0_norm, q1 / q1_norm
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1, dot = -q1, -dot
    if dot > 0.9995:
        quat = q0 + alpha * (q1 - q0)
        quat /= np.linalg.norm(quat)
    else:
        theta = float(np.arccos(max(-1.0, min(1.0, dot))))
        sin_theta = float(np.sin(theta))
        quat = (np.sin((1.0 - alpha) * theta) / sin_theta) * q0 + (np.sin(alpha * theta) / sin_theta) * q1
    result.pose.orientation.x, result.pose.orientation.y, result.pose.orientation.z, result.pose.orientation.w = map(float, quat)
    interpolated_stamp = (1.0 - alpha) * stamp_seconds(start) + alpha * stamp_seconds(goal)
    result.header.stamp.sec = int(interpolated_stamp)
    result.header.stamp.nanosec = int(round((interpolated_stamp - result.header.stamp.sec) * 1e9))
    if result.header.stamp.nanosec >= 1_000_000_000:
        result.header.stamp.sec += 1
        result.header.stamp.nanosec -= 1_000_000_000
    return result


def position_distance(start: PoseStamped, goal: PoseStamped) -> float:
    return float(np.linalg.norm([
        goal.pose.position.x - start.pose.position.x,
        goal.pose.position.y - start.pose.position.y,
        goal.pose.position.z - start.pose.position.z,
    ]))


def paths_have_same_trajectory(first: Optional[Path], second: Path) -> bool:
    """Compare path content while ignoring its periodically refreshed header stamp."""
    if first is None or first.header.frame_id != second.header.frame_id:
        return False
    if len(first.poses) != len(second.poses):
        return False
    return all(first_pose == second_pose for first_pose, second_pose in zip(first.poses, second.poses))


def resample_coupled_paths(
    arm_path: Path,
    base_path: Optional[Path],
    spacing: float,
) -> tuple[Path, Optional[Path]]:
    """Resample the arm path by arc length and interpolate a coupled base path.

    The source messages are never modified.  Samples are placed at roughly
    ``spacing`` metres along the arm polyline.  Their timestamp and orientation
    are interpolated from the original segment, preserving the exported timing
    profile.  Consecutive zero-length arm segments are retained exactly because
    they can represent a dwell, a base-only movement, or an orientation change.
    """
    if spacing <= 0.0 or len(arm_path.poses) < 2:
        return deepcopy(arm_path), deepcopy(base_path) if base_path is not None else None
    if base_path is not None and len(base_path.poses) != len(arm_path.poses):
        raise ValueError('coupled paths must have equal pose counts before resampling')

    arm_result = Path(header=deepcopy(arm_path.header))
    base_result = Path(header=deepcopy(base_path.header)) if base_path is not None else None

    def append(arm_pose: PoseStamped, base_pose: Optional[PoseStamped]) -> None:
        arm_result.poses.append(deepcopy(arm_pose))
        if base_result is not None and base_pose is not None:
            base_result.poses.append(deepcopy(base_pose))

    append(arm_path.poses[0], base_path.poses[0] if base_path is not None else None)
    cumulative_distance = 0.0
    next_sample_distance = float(spacing)
    epsilon = 1e-9

    for index, (arm_start, arm_goal) in enumerate(zip(arm_path.poses, arm_path.poses[1:])):
        base_start = base_path.poses[index] if base_path is not None else None
        base_goal = base_path.poses[index + 1] if base_path is not None else None
        segment_length = position_distance(arm_start, arm_goal)
        segment_end = cumulative_distance + segment_length

        if segment_length <= epsilon:
            # Do not discard semantic zero-displacement intervals.
            append(arm_goal, base_goal)
            continue

        while next_sample_distance < segment_end - epsilon:
            phase = (next_sample_distance - cumulative_distance) / segment_length
            append(
                interpolate_pose(arm_start, arm_goal, phase),
                interpolate_pose(base_start, base_goal, phase) if base_start is not None and base_goal is not None else None,
            )
            next_sample_distance += spacing
        cumulative_distance = segment_end

    # The final waypoint carries the exact exported endpoint timestamp and pose.
    if stamp_seconds(arm_result.poses[-1]) < stamp_seconds(arm_path.poses[-1]) - epsilon:
        append(arm_path.poses[-1], base_path.poses[-1] if base_path is not None else None)
    return arm_result, base_result


class IncrementPathIndex(Node):
    """Publish coupled index/phase progress and backward-compatible waypoint topics."""

    def __init__(self) -> None:
        super().__init__('increment_path_index')
        self.declare_parameter('path_index_topic', '/path_index')
        self.declare_parameter('path_index_command_topic', '/path_index_command')
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
        # ``fixed_rate`` retains the legacy behaviour for standalone users.
        self.declare_parameter('progress_mode', 'fixed_rate')
        self.declare_parameter('base_path_topic', '')
        self.declare_parameter('arm_reference_topic', '/arm_trajectory_reference')
        self.declare_parameter('base_reference_topic', '/base_trajectory_reference')
        self.declare_parameter('processed_path_topic', '/ur_path_tracking')
        self.declare_parameter('processed_base_path_topic', '/base_path_tracking')
        self.declare_parameter('phase_topic', '/trajectory_phase')
        self.declare_parameter('desired_speed_topic', '/desired_arm_speed')
        self.declare_parameter('desired_arm_speed', -1.0)
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('zero_segment_duration', 0.1)
        self.declare_parameter('enable_path_resampling', False)
        self.declare_parameter('resample_spacing', 0.005)

        self.path: Optional[Path] = None
        self.base_path: Optional[Path] = None
        self._source_path: Optional[Path] = None
        self._source_base_path: Optional[Path] = None
        self.additional_goal_path: Optional[Path] = None
        self.path_index = max(0, int(self.get_parameter('initial_path_index').value))
        self.phase = 0.0
        self.completed = False
        self._last_published_index: Optional[int] = None
        self.start_enabled = not self._as_bool(self.get_parameter('wait_for_start_condition').value)
        self.normal = Vector3(x=0.0, y=0.0, z=1.0)
        self.base_publish_rate = max(0.0, float(self.get_parameter('publish_rate').value))
        self.velocity_override = 1.0
        self.desired_arm_speed = float(self.get_parameter('desired_arm_speed').value)
        self.progress_mode = str(self.get_parameter('progress_mode').value).strip().lower()
        if self.progress_mode not in {'fixed_rate', 'timestamp', 'desired_speed'}:
            self.get_logger().warn(f"Unknown progress_mode '{self.progress_mode}', using fixed_rate.")
            self.progress_mode = 'fixed_rate'
        self._timer = None
        self._base_path_required = bool(str(self.get_parameter('base_path_topic').value).strip())
        self._trajectory_valid = False
        self._last_tick = self.get_clock().now()

        latch_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, reliability=QoSReliabilityPolicy.RELIABLE)
        self.path_index_topic = str(self.get_parameter('path_index_topic').value)
        self.path_index_command_topic = str(
            self.get_parameter('path_index_command_topic').value
        )
        self.index_pub = self.create_publisher(Int32, self.path_index_topic, latch_qos)
        self.goal_pose_pub = self.create_publisher(PoseStamped, str(self.get_parameter('next_goal_topic').value), latch_qos)
        self.normal_pub = self.create_publisher(Vector3, str(self.get_parameter('normal_topic').value), latch_qos)
        self.arm_reference_pub = self.create_publisher(PoseStamped, str(self.get_parameter('arm_reference_topic').value), latch_qos)
        self.base_reference_pub = self.create_publisher(PoseStamped, str(self.get_parameter('base_reference_topic').value), latch_qos)
        self.processed_path_pub = self.create_publisher(Path, str(self.get_parameter('processed_path_topic').value), latch_qos)
        self.processed_base_path_pub = self.create_publisher(Path, str(self.get_parameter('processed_base_path_topic').value), latch_qos)
        self.phase_pub = self.create_publisher(Float32, str(self.get_parameter('phase_topic').value), latch_qos)

        self.additional_goal_pose_pub = None
        additional_goal_path_topic = str(self.get_parameter('additional_goal_path_topic').value).strip()
        additional_goal_topic = str(self.get_parameter('additional_goal_topic').value).strip()
        if additional_goal_path_topic and additional_goal_topic:
            self.additional_goal_pose_pub = self.create_publisher(PoseStamped, additional_goal_topic, latch_qos)
            self.create_subscription(Path, additional_goal_path_topic, self._additional_goal_path_cb, latch_qos)

        self.create_subscription(Path, str(self.get_parameter('path_topic').value), self._path_cb, latch_qos)
        base_path_topic = str(self.get_parameter('base_path_topic').value).strip()
        if base_path_topic:
            self.create_subscription(Path, base_path_topic, self._base_path_cb, latch_qos)
        self.create_subscription(Vector3, str(self.get_parameter('normal_topic').value), self._normal_cb, latch_qos)
        self.create_subscription(
            Int32,
            self.path_index_command_topic,
            self._external_index_cb,
            10,
        )
        self.create_subscription(Float32, str(self.get_parameter('velocity_override_topic').value), self._velocity_override_cb, 10)
        desired_speed_topic = str(self.get_parameter('desired_speed_topic').value).strip()
        if desired_speed_topic:
            self.create_subscription(Float32, desired_speed_topic, self._desired_speed_cb, latch_qos)
        self.create_subscription(Bool, str(self.get_parameter('start_condition_topic').value), self._start_cb, 10)
        self._update_timer()

    @staticmethod
    def _as_bool(value) -> bool:
        return value if isinstance(value, bool) else str(value).strip().lower() in {'1', 'true', 'yes', 'y', 'on'}

    def _path_cb(self, msg: Path) -> None:
        if not msg.poses:
            self.get_logger().warn('Ignoring empty arm path.')
            return
        if paths_have_same_trajectory(self._source_path, msg):
            return
        self._source_path = msg
        self._rebuild_processed_paths()

    def _base_path_cb(self, msg: Path) -> None:
        if not msg.poses:
            self.get_logger().warn('Ignoring empty base path.')
            return
        if paths_have_same_trajectory(self._source_base_path, msg):
            return
        self._source_base_path = msg
        self._rebuild_processed_paths()

    def _rebuild_processed_paths(self) -> None:
        if self._source_path is None:
            return
        if self._base_path_required and self._source_base_path is None:
            self._trajectory_valid = False
            return
        try:
            if self._as_bool(self.get_parameter('enable_path_resampling').value):
                spacing = float(self.get_parameter('resample_spacing').value)
                self.path, self.base_path = resample_coupled_paths(
                    self._source_path,
                    self._source_base_path,
                    spacing,
                )
                self.get_logger().info(
                    f'Resampled arm trajectory: {len(self._source_path.poses)} -> {len(self.path.poses)} poses '
                    f'at {spacing:.4f} m nominal spacing.'
                )
            else:
                self.path = self._source_path
                self.base_path = self._source_base_path
        except ValueError as error:
            self._trajectory_valid = False
            self.get_logger().error(f'Rejecting trajectory before resampling: {error}.')
            return
        self.path_index = min(self.path_index, len(self.path.poses) - 1)
        self.phase, self.completed = 0.0, False
        self._trajectory_valid = self._validate_coupling()
        if self._trajectory_valid:
            self.processed_path_pub.publish(self.path)
            if self.base_path is not None:
                self.processed_base_path_pub.publish(self.base_path)
        self._publish_state(force=True)

    def _validate_coupling(self) -> bool:
        if self.path is None:
            return False
        if self._base_path_required and self.base_path is None:
            return False
        if self.base_path is not None and len(self.path.poses) != len(self.base_path.poses):
            self.get_logger().error(f'Coupled paths have unequal pose counts: arm={len(self.path.poses)}, base={len(self.base_path.poses)}.')
            return False
        if self.base_path is not None:
            for index, (arm_pose, base_pose) in enumerate(zip(self.path.poses, self.base_path.poses)):
                if abs(stamp_seconds(arm_pose) - stamp_seconds(base_pose)) > 1e-6:
                    self.get_logger().error(f'Rejecting coupled trajectory: timestamp mismatch at index {index}.')
                    return False
        for label, path in (('arm', self.path), ('base', self.base_path)):
            if path is None:
                continue
            for previous, current in zip(path.poses, path.poses[1:]):
                if stamp_seconds(current) <= stamp_seconds(previous):
                    self.get_logger().error(f'Rejecting {label} trajectory: timestamps must strictly increase.')
                    return False
            small_segments = sum(position_distance(previous, current) <= 1e-6 for previous, current in zip(path.poses, path.poses[1:]))
            lengths = [position_distance(previous, current) for previous, current in zip(path.poses, path.poses[1:])]
            durations = [stamp_seconds(current) - stamp_seconds(previous) for previous, current in zip(path.poses, path.poses[1:])]
            speeds = [length / duration for length, duration in zip(lengths, durations)]
            if lengths:
                self.get_logger().info(
                    f'{label} trajectory diagnostics: {len(lengths)} segments, '
                    f'length=[{min(lengths):.4f}, {max(lengths):.4f}] m, '
                    f'speed=[{min(speeds):.4f}, {max(speeds):.4f}] m/s.'
                )
            if small_segments:
                self.get_logger().info(f'{label} trajectory contains {small_segments} zero-length position segments; preserving their timing.')
        return True

    def _additional_goal_path_cb(self, msg: Path) -> None:
        if msg.poses:
            self.additional_goal_path = msg
            self._publish_state(force=True)

    def _normal_cb(self, msg: Vector3) -> None:
        self.normal = msg

    def _start_cb(self, msg: Bool) -> None:
        self.start_enabled = bool(msg.data)
        self._last_tick = self.get_clock().now()

    def _velocity_override_cb(self, msg: Float32) -> None:
        self.velocity_override = max(0.0, float(msg.data))
        self._update_timer()

    def _desired_speed_cb(self, msg: Float32) -> None:
        self.desired_arm_speed = max(0.0, float(msg.data))

    def _external_index_cb(self, msg: Int32) -> None:
        if self.path is None:
            return
        requested = max(0, min(int(msg.data), len(self.path.poses) - 1))
        if requested != self.path_index:
            self.path_index, self.phase, self.completed = requested, 0.0, requested == len(self.path.poses) - 1
            self._publish_state(force=True)

    def _update_timer(self) -> None:
        if self._timer is not None:
            self._timer.cancel()
            self.destroy_timer(self._timer)
        if self.progress_mode == 'fixed_rate':
            rate = self.base_publish_rate * self.velocity_override
            self._timer = None if rate <= 0.0 else self.create_timer(1.0 / rate, self._legacy_tick)
        else:
            rate = max(1.0, float(self.get_parameter('control_rate').value))
            self._timer = self.create_timer(1.0 / rate, self._segment_tick)
        self._last_tick = self.get_clock().now()

    def _legacy_tick(self) -> None:
        if self.path is None or not self.path.poses or not self.start_enabled:
            return
        if self.path_index < len(self.path.poses) - 1:
            self.path_index += 1
            self._publish_state()

    def _segment_duration(self) -> float:
        if self.path is None or self.path_index >= len(self.path.poses) - 1:
            return 0.0
        start, goal = self.path.poses[self.path_index], self.path.poses[self.path_index + 1]
        timestamp_duration = stamp_seconds(goal) - stamp_seconds(start)
        if self.progress_mode == 'timestamp':
            return timestamp_duration if timestamp_duration > 0.0 else max(0.001, float(self.get_parameter('zero_segment_duration').value))
        distance = position_distance(start, goal)
        if distance > 1e-6 and self.desired_arm_speed > 1e-6:
            return distance / self.desired_arm_speed
        # A zero arm displacement can still encode base movement, orientation, or a dwell.
        return timestamp_duration if timestamp_duration > 0.0 else max(0.001, float(self.get_parameter('zero_segment_duration').value))

    def _segment_tick(self) -> None:
        now = self.get_clock().now()
        dt = max(0.0, (now - self._last_tick).nanoseconds / 1e9)
        self._last_tick = now
        if not self._trajectory_valid or self.path is None or not self.path.poses or not self.start_enabled or self.completed:
            return
        if self.velocity_override > 0.0:
            remaining = dt * self.velocity_override
            while remaining > 0.0 and not self.completed:
                duration = self._segment_duration()
                phase_remaining = max(0.0, 1.0 - self.phase)
                time_remaining = phase_remaining * duration
                if remaining < time_remaining:
                    self.phase += remaining / duration
                    remaining = 0.0
                else:
                    remaining -= time_remaining
                    self.path_index += 1
                    self.phase = 0.0
                    self.completed = self.path_index >= len(self.path.poses) - 1
        self._publish_state(force=True)

    def _reference(self, path: Optional[Path]) -> Optional[PoseStamped]:
        if path is None or not path.poses:
            return None
        index = min(self.path_index, len(path.poses) - 1)
        if index >= len(path.poses) - 1 or self.completed:
            return deepcopy(path.poses[-1])
        return interpolate_pose(path.poses[index], path.poses[index + 1], self.phase)

    def _publish_state(self, force: bool = False) -> None:
        if not self._trajectory_valid or self.path is None or not self.path.poses:
            return
        arm_reference = self._reference(self.path)
        if arm_reference is None:
            return
        self.arm_reference_pub.publish(arm_reference)
        self.goal_pose_pub.publish(arm_reference)
        self.phase_pub.publish(Float32(data=float(self.phase)))
        self.normal_pub.publish(self.normal)
        base_reference = self._reference(self.base_path)
        if base_reference is not None:
            self.base_reference_pub.publish(base_reference)
        if self.additional_goal_pose_pub is not None and self.additional_goal_path is not None:
            reference = self._reference(self.additional_goal_path)
            if reference is not None:
                self.additional_goal_pose_pub.publish(reference)
        if force or self.path_index != self._last_published_index:
            self.index_pub.publish(Int32(data=self.path_index))
            self._last_published_index = self.path_index


def main(args=None) -> None:
    rclpy.init(args=args)
    node = IncrementPathIndex()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
