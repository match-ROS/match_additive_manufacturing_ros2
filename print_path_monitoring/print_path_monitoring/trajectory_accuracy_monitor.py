#!/usr/bin/env python3
"""Record base or TCP tracking accuracy without commanding the robot."""
from __future__ import annotations

import csv
import json
import math
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Twist, Vector3Stamped
from nav_msgs.msg import Path as RosPath
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, Int32

from print_path_monitoring.error_metrics import (
    compute_planar_error,
    compute_pose_error,
    compute_tracking_error,
    summarize_distances,
    tool_z_axis,
    yaw_from_quaternion,
)


class TrajectoryAccuracyMonitor(Node):
    def __init__(self) -> None:
        super().__init__('trajectory_accuracy_monitor')
        self.declare_parameter('mode', 'tcp')
        self.declare_parameter('actual_pose_topic', '/current_tcp_pose')
        self.declare_parameter('reference_path_topic', '/ur_path_transformed')
        self.declare_parameter('reference_pose_topic', '')
        self.declare_parameter('base_reference_path_topic', '/base_path')
        self.declare_parameter('arm_base_offset', [0.0, 0.0, 0.0])
        self.declare_parameter('min_reachable_radius', 0.25)
        self.declare_parameter('max_reachable_radius', 0.85)
        self.declare_parameter('reach_boundary_margin', 0.05)
        self.declare_parameter('path_index_topic', '/path_index')
        self.declare_parameter('trajectory_phase_topic', '')
        self.declare_parameter('velocity_override_topic', '')
        self.declare_parameter('desired_speed_topic', '')
        self.declare_parameter('fixed_path_index', -1)
        self.declare_parameter('output_directory', '/tmp/am_trajectory_runs')
        self.declare_parameter('run_name', '')
        self.declare_parameter('phase', 'baseline')
        self.declare_parameter('max_pose_age', 0.75)
        self.declare_parameter('required_frame', 'map')
        self.declare_parameter('max_index_offset', 12)
        self.declare_parameter('start_condition_topic', '')
        # The arm remains in bounded final-position feedback after its phase
        # reaches one.  Keep recording after completion to characterize the
        # settling phase instead of treating the first path-end sample as the
        # endpoint result.
        self.declare_parameter('post_end_grace_seconds', 3.0)
        self.declare_parameter('max_post_end_seconds', 30.0)
        self.declare_parameter('error_topic_prefix', '/trajectory_accuracy')
        self.declare_parameter('command_twist_topic', '')
        self.declare_parameter('joint_states_topic', '')
        self.declare_parameter('max_tracking_linear_velocity', 0.12)
        self.declare_parameter('saturation_fraction', 0.99)
        self.declare_parameter('completion_topic', '/trajectory_complete')

        self.mode = str(self.get_parameter('mode').value).strip().lower()
        if self.mode not in {'base', 'tcp'}:
            raise ValueError("mode must be 'base' or 'tcp'")
        self.phase = str(self.get_parameter('phase').value).strip().lower()
        if self.phase not in {'baseline', 'tuned'}:
            raise ValueError("phase must be 'baseline' or 'tuned'")
        self.path: Optional[RosPath] = None
        self.base_path: Optional[RosPath] = None
        self.reference_pose: Optional[PoseStamped] = None
        self.path_index: Optional[int] = None
        self.trajectory_phase: Optional[float] = None
        self.velocity_override: Optional[float] = None
        self.desired_speed: Optional[float] = None
        fixed = int(self.get_parameter('fixed_path_index').value)
        self.fixed_path_index: Optional[int] = fixed if fixed >= 0 else None
        self.invalid = Counter()
        self.samples: list[dict[str, float | int]] = []
        start_topic = str(self.get_parameter('start_condition_topic').value).strip()
        self.recording_enabled = not bool(start_topic)
        self.path_end_time = None
        self.start_time = self.get_clock().now() if self.recording_enabled else None
        self.completion_time = None
        self.last_twist_time = None
        self.last_twist_speed = 0.0
        self.twist_duration = 0.0
        self.twist_saturation_duration = 0.0
        self.twist_speeds: list[float] = []
        self.twist_linear_x: list[float] = []
        self.twist_linear_y: list[float] = []
        self.twist_linear_z: list[float] = []
        self.joint_velocities: dict[str, list[float]] = {}

        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(RosPath, str(self.get_parameter('reference_path_topic').value), self._path_cb, qos)
        reference_pose_topic = str(self.get_parameter('reference_pose_topic').value).strip()
        if reference_pose_topic:
            self.create_subscription(PoseStamped, reference_pose_topic, self._reference_cb, qos)
        if self.mode == 'tcp':
            self.create_subscription(
                RosPath,
                str(self.get_parameter('base_reference_path_topic').value),
                self._base_path_cb,
                qos,
            )
        self.create_subscription(Int32, str(self.get_parameter('path_index_topic').value), self._index_cb, 10)
        trajectory_phase_topic = str(self.get_parameter('trajectory_phase_topic').value).strip()
        if trajectory_phase_topic:
            self.create_subscription(Float32, trajectory_phase_topic, self._trajectory_phase_cb, 10)
        velocity_override_topic = str(self.get_parameter('velocity_override_topic').value).strip()
        if velocity_override_topic:
            self.create_subscription(Float32, velocity_override_topic, self._velocity_override_cb, 10)
        desired_speed_topic = str(self.get_parameter('desired_speed_topic').value).strip()
        if desired_speed_topic:
            self.create_subscription(Float32, desired_speed_topic, self._desired_speed_cb, 10)
        self.create_subscription(PoseStamped, str(self.get_parameter('actual_pose_topic').value), self._pose_cb, 10)
        command_twist_topic = str(self.get_parameter('command_twist_topic').value).strip()
        if command_twist_topic:
            self.create_subscription(Twist, command_twist_topic, self._twist_cb, 10)
        joint_states_topic = str(self.get_parameter('joint_states_topic').value).strip()
        if joint_states_topic:
            self.create_subscription(JointState, joint_states_topic, self._joint_state_cb, 10)
        completion_topic = str(self.get_parameter('completion_topic').value).strip()
        if completion_topic:
            self.create_subscription(Bool, completion_topic, self._completion_cb, 10)
        if start_topic:
            self.create_subscription(Bool, start_topic, self._start_cb, 10)
        prefix = str(self.get_parameter('error_topic_prefix').value).rstrip('/') + '/' + self.mode
        self.vector_pub = self.create_publisher(Vector3Stamped, prefix + '/error_vector', 10)
        self.absolute_pub = self.create_publisher(Float32, prefix + '/absolute_error', 10)
        self.yaw_pub = self.create_publisher(Float32, prefix + '/yaw_error', 10)
        self.tangential_pub = self.create_publisher(Float32, prefix + '/planar_tangential_error', 10)
        self.cross_track_pub = self.create_publisher(Float32, prefix + '/planar_cross_track_error', 10)
        self.along_pub = self.create_publisher(Float32, prefix + '/along_track_error', 10)
        self.lateral_pub = self.create_publisher(Float32, prefix + '/lateral_error', 10)
        self.spray_pub = self.create_publisher(Float32, prefix + '/spray_axis_error', 10)

        output_directory = Path(str(self.get_parameter('output_directory').value)).expanduser()
        run_name = str(self.get_parameter('run_name').value).strip() or (
            f'{self.mode}_{datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")}')
        output_directory.mkdir(parents=True, exist_ok=True)
        self.csv_path = output_directory / f'{run_name}.csv'
        self.summary_path = output_directory / f'{run_name}.json'
        self.csv_file = self.csv_path.open('w', newline='', encoding='utf-8')
        self.writer = csv.DictWriter(self.csv_file, fieldnames=[
            'stamp_sec', 'path_index', 'trajectory_phase', 'velocity_override', 'desired_speed',
            'reference_source', 'actual_x', 'actual_y', 'actual_z',
            'dx', 'dy', 'dz', 'absolute_error', 'yaw_error',
            'planar_tangential_error', 'planar_cross_track_error',
            'along_track_error', 'lateral_error', 'spray_axis_error',
            'reach_class', 'planned_arm_base_x', 'planned_arm_base_y',
            'planned_arm_base_z', 'planned_arm_base_planar_radius',
        ])
        self.writer.writeheader()
        self.get_logger().info(f'Recording {self.mode} trajectory accuracy to {self.csv_path}')

    def _path_cb(self, msg: RosPath) -> None:
        self.path = msg

    def _reference_cb(self, msg: PoseStamped) -> None:
        self.reference_pose = msg

    def _base_path_cb(self, msg: RosPath) -> None:
        self.base_path = msg

    def _index_cb(self, msg: Int32) -> None:
        self.path_index = max(0, int(msg.data))
        if self.path is not None and self.path.poses and self.path_index >= len(self.path.poses) - 1:
            self.path_end_time = self.get_clock().now()

    def _trajectory_phase_cb(self, msg: Float32) -> None:
        self.trajectory_phase = max(0.0, min(1.0, float(msg.data)))

    def _velocity_override_cb(self, msg: Float32) -> None:
        self.velocity_override = max(0.0, float(msg.data))

    def _desired_speed_cb(self, msg: Float32) -> None:
        self.desired_speed = max(0.0, float(msg.data))

    def _start_cb(self, msg: Bool) -> None:
        self.recording_enabled = bool(msg.data)
        if self.recording_enabled and self.start_time is None:
            self.start_time = self.get_clock().now()

    def _completion_cb(self, msg: Bool) -> None:
        if bool(msg.data) and self.completion_time is None:
            self.completion_time = self.get_clock().now()

    def _recording_window_open(self) -> bool:
        if not self.recording_enabled:
            return False
        if self.path_end_time is None:
            return True
        now = self.get_clock().now()
        if self.completion_time is not None:
            elapsed = (now - self.completion_time).nanoseconds / 1e9
            return elapsed <= float(self.get_parameter('post_end_grace_seconds').value)
        # A failed or unreachable trajectory must not record forever.  A
        # campaign run can raise this upper bound when a deliberately slow
        # final approach is under test.
        elapsed = (now - self.path_end_time).nanoseconds / 1e9
        return elapsed <= float(self.get_parameter('max_post_end_seconds').value)

    def _twist_cb(self, msg: Twist) -> None:
        if not self._recording_window_open():
            return
        now = self.get_clock().now()
        speed = math.sqrt(msg.linear.x ** 2 + msg.linear.y ** 2 + msg.linear.z ** 2)
        if self.last_twist_time is not None:
            dt = max(0.0, (now - self.last_twist_time).nanoseconds / 1e9)
            self.twist_duration += dt
            maximum = max(0.0, float(self.get_parameter('max_tracking_linear_velocity').value))
            threshold = maximum * max(0.0, float(self.get_parameter('saturation_fraction').value))
            if maximum > 0.0 and self.last_twist_speed >= threshold:
                self.twist_saturation_duration += dt
        self.last_twist_time = now
        self.last_twist_speed = speed
        self.twist_speeds.append(speed)
        self.twist_linear_x.append(float(msg.linear.x))
        self.twist_linear_y.append(float(msg.linear.y))
        self.twist_linear_z.append(float(msg.linear.z))

    def _joint_state_cb(self, msg: JointState) -> None:
        if not self._recording_window_open():
            return
        for name, velocity in zip(msg.name, msg.velocity):
            self.joint_velocities.setdefault(name, []).append(abs(float(velocity)))

    def _reference_for_sample(self, index: int) -> tuple[PoseStamped, str]:
        if self.reference_pose is not None:
            return self.reference_pose, 'continuous'
        return self.path.poses[index], 'indexed'

    def _nearest_path_index(self, reference: PoseStamped) -> int:
        """Provide a diagnostic tangent when a continuous reference has no index."""
        assert self.path is not None and self.path.poses
        point = reference.pose.position
        return min(
            range(len(self.path.poses)),
            key=lambda index: (
                (self.path.poses[index].pose.position.x - point.x) ** 2
                + (self.path.poses[index].pose.position.y - point.y) ** 2
                + (self.path.poses[index].pose.position.z - point.z) ** 2
            ),
        )

    def _segment_tangent(self, index: int) -> tuple[float, float, float]:
        if self.path is None or len(self.path.poses) < 2:
            return (0.0, 0.0, 0.0)
        start_index = min(max(0, index), len(self.path.poses) - 2)
        start = self.path.poses[start_index].pose.position
        goal = self.path.poses[start_index + 1].pose.position
        return (goal.x - start.x, goal.y - start.y, goal.z - start.z)

    def _pose_cb(self, actual: PoseStamped) -> None:
        if not self.recording_enabled:
            self.invalid['before_start_condition'] += 1
            return
        if not self._recording_window_open():
            self.invalid['after_path_end'] += 1
            return
        max_pose_age = float(self.get_parameter('max_pose_age').value)
        if not (actual.header.stamp.sec or actual.header.stamp.nanosec):
            self.invalid['unstamped_actual_pose'] += 1
            return
        age = (self.get_clock().now() - rclpy.time.Time.from_msg(actual.header.stamp)).nanoseconds / 1e9
        if age > max_pose_age:
            self.invalid['stale_actual_pose'] += 1
            return
        if self.path is None or not self.path.poses:
            self.invalid['missing_path'] += 1
            return
        index = self.path_index if self.path_index is not None else self.fixed_path_index
        if index is None:
            if self.reference_pose is None:
                self.invalid['missing_path_index'] += 1
                return
            index = self._nearest_path_index(self.reference_pose)
        if index >= len(self.path.poses):
            self.invalid['path_index_out_of_range'] += 1
            return
        reference, reference_source = self._reference_for_sample(index)
        actual_frame = actual.header.frame_id.strip().lstrip('/')
        reference_frame = (reference.header.frame_id or self.path.header.frame_id).strip().lstrip('/')
        required_frame = str(self.get_parameter('required_frame').value).strip().lstrip('/')
        if (not actual_frame or not reference_frame or actual_frame != reference_frame or
                (required_frame and actual_frame != required_frame)):
            self.invalid['frame_mismatch'] += 1
            return
        error = compute_pose_error(actual.pose, reference.pose)
        previous = self.path.poses[max(0, index - 1)].pose
        following = self.path.poses[min(len(self.path.poses) - 1, index + 1)].pose
        planar = compute_planar_error(actual.pose, reference.pose, previous, following)
        tracking = compute_tracking_error(
            actual.pose,
            reference.pose,
            self._segment_tangent(index),
            tool_z_axis(reference.pose),
        )
        reach = self._reach_classification(index, reference)
        stamp = actual.header.stamp
        row = {
            'stamp_sec': float(stamp.sec) + float(stamp.nanosec) / 1e9,
            'path_index': index,
            'trajectory_phase': self.trajectory_phase,
            'velocity_override': self.velocity_override,
            'desired_speed': self.desired_speed,
            'reference_source': reference_source,
            'actual_x': actual.pose.position.x,
            'actual_y': actual.pose.position.y,
            'actual_z': actual.pose.position.z,
            'dx': error.dx, 'dy': error.dy, 'dz': error.dz,
            'absolute_error': error.distance, 'yaw_error': error.yaw_error,
            'planar_tangential_error': planar.tangential,
            'planar_cross_track_error': planar.cross_track,
            'along_track_error': tracking.along_track,
            'lateral_error': tracking.lateral,
            'spray_axis_error': tracking.spray_axis,
            **reach,
        }
        self.samples.append(row)
        self.writer.writerow(row)
        self.csv_file.flush()
        vector = Vector3Stamped()
        vector.header = actual.header
        vector.vector.x, vector.vector.y, vector.vector.z = error.dx, error.dy, error.dz
        self.vector_pub.publish(vector)
        self.absolute_pub.publish(Float32(data=error.distance))
        self.yaw_pub.publish(Float32(data=error.yaw_error))
        if self.mode == 'tcp':
            self.tangential_pub.publish(Float32(data=planar.tangential))
            self.cross_track_pub.publish(Float32(data=planar.cross_track))
            self.along_pub.publish(Float32(data=tracking.along_track))
            self.lateral_pub.publish(Float32(data=tracking.lateral))
            self.spray_pub.publish(Float32(data=tracking.spray_axis))

    def write_summary(self) -> None:
        distances = [float(row['absolute_error']) for row in self.samples]
        endpoint_rows = self.samples[-1:] if self.samples else []
        trajectory_duration = None
        if self.start_time is not None and self.path_end_time is not None:
            trajectory_duration = max(0.0, (self.path_end_time - self.start_time).nanoseconds / 1e9)
        summary = {
            'mode': self.mode,
            'phase': self.phase,
            'samples': len(self.samples),
            'invalid_samples': dict(self.invalid),
            'valid_sample_fraction': self._valid_sample_fraction(),
            'absolute_error': summarize_distances(distances),
            'yaw_error': summarize_distances(abs(float(row['yaw_error'])) for row in self.samples),
            'axis_bias': {axis: (sum(float(row[axis]) for row in self.samples) / len(self.samples) if self.samples else 0.0)
                          for axis in ('dx', 'dy', 'dz')},
            'path_index_alignment': self._path_index_alignment(),
            'reference_source': ('continuous' if self.reference_pose is not None else 'indexed'),
            'trajectory_duration_seconds': trajectory_duration,
            'endpoint_error': summarize_distances(float(row['absolute_error']) for row in endpoint_rows),
            'command_twist': {
                'linear_speed': summarize_distances(self.twist_speeds),
                'linear_x': summarize_distances(self.twist_linear_x),
                'linear_y': summarize_distances(self.twist_linear_y),
                'linear_z': summarize_distances(self.twist_linear_z),
                'observed_duration_seconds': self.twist_duration,
                'saturation_duration_seconds': self.twist_saturation_duration,
                'saturation_time_fraction': (
                    self.twist_saturation_duration / self.twist_duration
                    if self.twist_duration > 1e-9 else 0.0
                ),
            },
            'joint_velocity': {
                name: summarize_distances(values)
                for name, values in sorted(self.joint_velocities.items())
            },
        }
        if self.mode == 'tcp':
            summary['planar_tangential_error'] = summarize_distances(
                abs(float(row['planar_tangential_error'])) for row in self.samples)
            summary['planar_cross_track_error'] = summarize_distances(
                abs(float(row['planar_cross_track_error'])) for row in self.samples)
            summary['along_track_error'] = summarize_distances(
                abs(float(row['along_track_error'])) for row in self.samples)
            summary['lateral_error'] = summarize_distances(
                float(row['lateral_error']) for row in self.samples)
            summary['spray_axis_error'] = summarize_distances(
                abs(float(row['spray_axis_error'])) for row in self.samples)
            summary['reachability'] = self._reachability_summary()
        self.summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + '\n', encoding='utf-8')
        self.get_logger().info(f'Wrote accuracy summary to {self.summary_path}')

    def _valid_sample_fraction(self) -> float:
        # The gate deliberately discards pre-start and post-end messages; they
        # are not attempted tracking samples and must not dilute data quality.
        attempted_invalid = sum(
            count for reason, count in self.invalid.items()
            if reason not in {'before_start_condition', 'after_path_end'}
        )
        total = len(self.samples) + attempted_invalid
        return len(self.samples) / total if total else 0.0

    def _reach_classification(self, index: int, tcp_reference: PoseStamped) -> dict[str, float | str]:
        unavailable = {
            'reach_class': 'not_evaluated',
            'planned_arm_base_x': 0.0,
            'planned_arm_base_y': 0.0,
            'planned_arm_base_z': 0.0,
            'planned_arm_base_planar_radius': 0.0,
        }
        if self.mode != 'tcp' or self.base_path is None or index >= len(self.base_path.poses):
            return unavailable
        base = self.base_path.poses[index].pose
        offset = list(self.get_parameter('arm_base_offset').value)
        if len(offset) < 3:
            return unavailable
        yaw = yaw_from_quaternion(
            base.orientation.x, base.orientation.y, base.orientation.z, base.orientation.w,
        )
        ox, oy, oz = (float(offset[0]), float(offset[1]), float(offset[2]))
        arm_x = base.position.x + math.cos(yaw) * ox - math.sin(yaw) * oy
        arm_y = base.position.y + math.sin(yaw) * ox + math.cos(yaw) * oy
        arm_z = base.position.z + oz
        radius = math.hypot(tcp_reference.pose.position.x - arm_x, tcp_reference.pose.position.y - arm_y)
        minimum = float(self.get_parameter('min_reachable_radius').value)
        maximum = float(self.get_parameter('max_reachable_radius').value)
        margin = max(0.0, float(self.get_parameter('reach_boundary_margin').value))
        if radius < minimum or radius > maximum:
            reach_class = 'unreachable_planar'
        elif radius <= minimum + margin or radius >= maximum - margin:
            reach_class = 'poor_reachability'
        else:
            reach_class = 'well_reachable'
        return {
            'reach_class': reach_class,
            'planned_arm_base_x': arm_x,
            'planned_arm_base_y': arm_y,
            'planned_arm_base_z': arm_z,
            'planned_arm_base_planar_radius': radius,
        }

    def _reachability_summary(self) -> dict[str, object]:
        classes: dict[str, dict[str, object]] = {}
        for reach_class in ('well_reachable', 'poor_reachability', 'unreachable_planar', 'not_evaluated'):
            rows = [row for row in self.samples if row.get('reach_class') == reach_class]
            if not rows:
                continue
            classes[reach_class] = {
                'samples': len(rows),
                'absolute_error': summarize_distances(float(row['absolute_error']) for row in rows),
                'axis_bias': {
                    axis: sum(float(row[axis]) for row in rows) / len(rows)
                    for axis in ('dx', 'dy', 'dz')
                },
                'planar_tangential_error': summarize_distances(
                    abs(float(row['planar_tangential_error'])) for row in rows),
                'planar_cross_track_error': summarize_distances(
                    abs(float(row['planar_cross_track_error'])) for row in rows),
            }
        return {
            'arm_base_offset': list(self.get_parameter('arm_base_offset').value),
            'planar_radius_range': [
                float(self.get_parameter('min_reachable_radius').value),
                float(self.get_parameter('max_reachable_radius').value),
            ],
            'boundary_margin': float(self.get_parameter('reach_boundary_margin').value),
            'classes': classes,
        }

    def _path_index_alignment(self) -> dict[str, float | int | bool | str]:
        """Diagnose a constant index lag without overwriting primary metrics."""
        if self.path is None or not self.samples:
            return {'available': False, 'reason': 'No path or valid samples.'}
        limit = max(0, int(self.get_parameter('max_index_offset').value))
        candidates: list[tuple[int, float]] = []
        for offset in range(-limit, limit + 1):
            errors = []
            for row in self.samples:
                index = int(row['path_index']) + offset
                if not 0 <= index < len(self.path.poses):
                    continue
                point = self.path.poses[index].pose.position
                errors.append(((float(row['actual_x']) - point.x) ** 2 +
                               (float(row['actual_y']) - point.y) ** 2 +
                               (float(row['actual_z']) - point.z) ** 2) ** 0.5)
            if errors:
                errors.sort()
                candidates.append((offset, errors[len(errors) // 2]))
        if not candidates:
            return {'available': False, 'reason': 'No aligned path samples.'}
        commanded = next(value for offset, value in candidates if offset == 0)
        offset, median = min(candidates, key=lambda item: item[1])
        return {
            'available': True,
            'best_constant_index_offset': offset,
            'commanded_index_median_error': commanded,
            'best_offset_median_error': median,
            'geometric_error_not_explained_by_index_lag': offset == 0,
        }


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[TrajectoryAccuracyMonitor] = None
    try:
        node = TrajectoryAccuracyMonitor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.write_summary()
            node.csv_file.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
