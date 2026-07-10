#!/usr/bin/env python3
"""Record base or TCP tracking accuracy without commanding the robot."""
from __future__ import annotations

import csv
import json
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from nav_msgs.msg import Path as RosPath
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Float32, Int32

from print_path_monitoring.error_metrics import (
    compute_planar_error,
    compute_pose_error,
    summarize_distances,
)


class TrajectoryAccuracyMonitor(Node):
    def __init__(self) -> None:
        super().__init__('trajectory_accuracy_monitor')
        self.declare_parameter('mode', 'tcp')
        self.declare_parameter('actual_pose_topic', '/current_tcp_pose')
        self.declare_parameter('reference_path_topic', '/ur_path_transformed')
        self.declare_parameter('path_index_topic', '/path_index')
        self.declare_parameter('fixed_path_index', -1)
        self.declare_parameter('output_directory', '/tmp/am_trajectory_runs')
        self.declare_parameter('run_name', '')
        self.declare_parameter('phase', 'baseline')
        self.declare_parameter('max_pose_age', 0.75)
        self.declare_parameter('error_topic_prefix', '/trajectory_accuracy')

        self.mode = str(self.get_parameter('mode').value).strip().lower()
        if self.mode not in {'base', 'tcp'}:
            raise ValueError("mode must be 'base' or 'tcp'")
        self.phase = str(self.get_parameter('phase').value).strip().lower()
        if self.phase not in {'baseline', 'tuned'}:
            raise ValueError("phase must be 'baseline' or 'tuned'")
        self.path: Optional[RosPath] = None
        self.path_index: Optional[int] = None
        fixed = int(self.get_parameter('fixed_path_index').value)
        self.fixed_path_index: Optional[int] = fixed if fixed >= 0 else None
        self.invalid = Counter()
        self.samples: list[dict[str, float | int]] = []

        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(RosPath, str(self.get_parameter('reference_path_topic').value), self._path_cb, qos)
        self.create_subscription(Int32, str(self.get_parameter('path_index_topic').value), self._index_cb, 10)
        self.create_subscription(PoseStamped, str(self.get_parameter('actual_pose_topic').value), self._pose_cb, 10)
        prefix = str(self.get_parameter('error_topic_prefix').value).rstrip('/') + '/' + self.mode
        self.vector_pub = self.create_publisher(Vector3Stamped, prefix + '/error_vector', 10)
        self.absolute_pub = self.create_publisher(Float32, prefix + '/absolute_error', 10)
        self.yaw_pub = self.create_publisher(Float32, prefix + '/yaw_error', 10)
        self.tangential_pub = self.create_publisher(Float32, prefix + '/planar_tangential_error', 10)
        self.cross_track_pub = self.create_publisher(Float32, prefix + '/planar_cross_track_error', 10)

        output_directory = Path(str(self.get_parameter('output_directory').value)).expanduser()
        run_name = str(self.get_parameter('run_name').value).strip() or (
            f'{self.mode}_{datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")}')
        output_directory.mkdir(parents=True, exist_ok=True)
        self.csv_path = output_directory / f'{run_name}.csv'
        self.summary_path = output_directory / f'{run_name}.json'
        self.csv_file = self.csv_path.open('w', newline='', encoding='utf-8')
        self.writer = csv.DictWriter(self.csv_file, fieldnames=[
            'stamp_sec', 'path_index', 'dx', 'dy', 'dz', 'absolute_error', 'yaw_error',
            'planar_tangential_error', 'planar_cross_track_error',
        ])
        self.writer.writeheader()
        self.get_logger().info(f'Recording {self.mode} trajectory accuracy to {self.csv_path}')

    def _path_cb(self, msg: RosPath) -> None:
        self.path = msg

    def _index_cb(self, msg: Int32) -> None:
        self.path_index = max(0, int(msg.data))

    def _pose_cb(self, actual: PoseStamped) -> None:
        max_pose_age = float(self.get_parameter('max_pose_age').value)
        if actual.header.stamp.sec or actual.header.stamp.nanosec:
            age = (self.get_clock().now() - rclpy.time.Time.from_msg(actual.header.stamp)).nanoseconds / 1e9
            if age > max_pose_age:
                self.invalid['stale_actual_pose'] += 1
                return
        if self.path is None or not self.path.poses:
            self.invalid['missing_path'] += 1
            return
        index = self.path_index if self.path_index is not None else self.fixed_path_index
        if index is None:
            self.invalid['missing_path_index'] += 1
            return
        if index >= len(self.path.poses):
            self.invalid['path_index_out_of_range'] += 1
            return
        reference = self.path.poses[index]
        actual_frame = actual.header.frame_id.strip().lstrip('/')
        reference_frame = (reference.header.frame_id or self.path.header.frame_id).strip().lstrip('/')
        if not actual_frame or not reference_frame or actual_frame != reference_frame:
            self.invalid['frame_mismatch'] += 1
            return
        error = compute_pose_error(actual.pose, reference.pose)
        previous = self.path.poses[max(0, index - 1)].pose
        following = self.path.poses[min(len(self.path.poses) - 1, index + 1)].pose
        planar = compute_planar_error(actual.pose, reference.pose, previous, following)
        stamp = actual.header.stamp
        row = {
            'stamp_sec': float(stamp.sec) + float(stamp.nanosec) / 1e9,
            'path_index': index,
            'dx': error.dx, 'dy': error.dy, 'dz': error.dz,
            'absolute_error': error.distance, 'yaw_error': error.yaw_error,
            'planar_tangential_error': planar.tangential,
            'planar_cross_track_error': planar.cross_track,
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

    def write_summary(self) -> None:
        distances = [float(row['absolute_error']) for row in self.samples]
        summary = {
            'mode': self.mode,
            'phase': self.phase,
            'samples': len(self.samples),
            'invalid_samples': dict(self.invalid),
            'valid_sample_fraction': (len(self.samples) / (len(self.samples) + sum(self.invalid.values()))
                                      if self.samples or self.invalid else 0.0),
            'absolute_error': summarize_distances(distances),
            'yaw_error': summarize_distances(abs(float(row['yaw_error'])) for row in self.samples),
            'axis_bias': {axis: (sum(float(row[axis]) for row in self.samples) / len(self.samples) if self.samples else 0.0)
                          for axis in ('dx', 'dy', 'dz')},
        }
        if self.mode == 'tcp':
            summary['planar_tangential_error'] = summarize_distances(
                abs(float(row['planar_tangential_error'])) for row in self.samples)
            summary['planar_cross_track_error'] = summarize_distances(
                abs(float(row['planar_cross_track_error'])) for row in self.samples)
        self.summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + '\n', encoding='utf-8')
        self.get_logger().info(f'Wrote accuracy summary to {self.summary_path}')


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
