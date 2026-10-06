import json
import math
from types import SimpleNamespace

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.parameter import Parameter
from std_msgs.msg import String

from print_path_monitoring.research_data import orientation_error, summarize_window
from print_path_monitoring.trajectory_accuracy_monitor import TrajectoryAccuracyMonitor
from print_path_monitoring.paper_accuracy_report import build_report, run_statistics


@pytest.fixture
def monitor(tmp_path):
    rclpy.init(args=[])
    node = TrajectoryAccuracyMonitor(use_global_arguments=False, parameter_overrides=[
        Parameter('output_directory', value=str(tmp_path)),
        Parameter('run_name', value='trial'),
        Parameter('trajectory_state_topic', value='/test/state'),
        Parameter('start_condition_topic', value='/test/start'),
    ])
    path = Path()
    path.header.frame_id = 'map'
    for i in range(3):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp.sec = i + 1
        pose.pose.position.x = float(i)
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)
    node._path_cb(path)
    yield node
    node.csv_file.close()
    node.destroy_node()
    rclpy.shutdown()


def state(stamp_ns, index=0, phase=0.2, enabled=True, override=1.0):
    reference = {'frame_id': 'map', 'planned_stamp_sec': 1 + index + phase,
                 'position': [index + phase, 0.0, 0.0], 'orientation': [0.0, 0.0, 0.0, 1.0]}
    return String(data=json.dumps({
        'schema_version': 1, 'stamp_ns': stamp_ns, 'path_index': index,
        'path_points': 3, 'segment_phase': phase, 'start_enabled': enabled,
        'path_complete': index == 2, 'velocity_override': override,
        'desired_speed': 0.04, 'arm_reference': reference, 'base_reference': reference,
    }))


def actual(stamp_ns, x=0.1, frame='map'):
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.header.stamp.sec, pose.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
    pose.pose.position.x = x
    pose.pose.orientation.w = 1.0
    return pose


def test_delayed_pose_uses_causal_atomic_target_and_saves_raw_poses(monitor):
    start = monitor.get_clock().now().nanoseconds - 300_000_000
    monitor._state_cb(state(start, phase=0.2))
    monitor._pose_cb(actual(start + 30_000_000))
    assert not monitor.samples  # No upper time bound received yet.
    monitor._state_cb(state(start + 50_000_000, phase=0.4))
    row = monitor.samples[0]
    assert row['target_x'] == 0.2  # Never use the future 0.4 target.
    assert row['dx'] == pytest.approx(0.1)
    assert row['trajectory_phase'] == 0.2
    assert row['reference_age'] == pytest.approx(0.03)
    assert row['reference_planned_stamp_sec'] == 1.2
    assert row['actual_qw'] == row['target_qw'] == 1.0
    assert row['reference_stamp_ns'] == start


def test_start_pause_settling_and_duplicate_samples(monitor):
    start = monitor.get_clock().now().nanoseconds - 300_000_000
    monitor._state_cb(state(start, enabled=False))
    monitor._state_cb(state(start + 20_000_000))
    monitor._pose_cb(actual(start + 10_000_000))
    assert monitor.invalid['before_start_condition'] == 1
    monitor._state_cb(state(start + 40_000_000, override=0.0))
    monitor._pose_cb(actual(start + 30_000_000))
    monitor._state_cb(state(start + 60_000_000, index=2, phase=0.0))
    monitor._pose_cb(actual(start + 50_000_000))
    monitor._state_cb(state(start + 80_000_000, index=2, phase=0.0))
    monitor._pose_cb(actual(start + 70_000_000))
    monitor._pose_cb(actual(start + 70_000_000))
    assert [r['measurement_window'] for r in monitor.samples] == ['active_tracking', 'paused', 'settling']
    assert monitor.invalid['duplicate_or_out_of_order_actual_pose'] == 1
    monitor.write_summary()
    summary = json.loads(monitor.summary_path.read_text())
    assert summary['windows']['active_tracking']['samples'] == 1
    assert summary['windows']['settling']['samples'] == 1
    assert summary['reached_path_end'] is True
    assert summary['synchronization'] == 'causal_source_state'


def test_stale_reference_and_frame_mismatch_are_counted(monitor):
    start = monitor.get_clock().now().nanoseconds - 400_000_000
    monitor._state_cb(state(start))
    monitor._state_cb(state(start + 300_000_000))
    monitor._pose_cb(actual(start + 200_000_000))
    monitor._pose_cb(actual(start + 10_000_000, frame='vicon_world'))
    assert monitor.invalid['stale_reference'] == 1
    assert monitor.invalid['frame_mismatch'] == 1
    assert not monitor.samples


def test_endpoint_timestamp_is_not_reset_by_repeated_state(monitor):
    start = monitor.get_clock().now().nanoseconds - 200_000_000
    monitor._state_cb(state(start, index=2, phase=0.0))
    monitor._state_cb(state(start + 100_000_000, index=2, phase=0.0))
    assert monitor.path_end_time.nanoseconds == start


def test_orientation_metric_detects_tilt_and_quaternion_sign_equivalence():
    q = SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
    tilt = SimpleNamespace(x=math.sqrt(0.5), y=0.0, z=0.0, w=math.sqrt(0.5))
    minus = SimpleNamespace(x=0.0, y=0.0, z=0.0, w=-1.0)
    assert orientation_error(q, tilt) == pytest.approx(math.pi / 2)
    assert orientation_error(q, minus) == 0.0
    q.x = float('nan')
    with pytest.raises(ValueError):
        orientation_error(q, tilt)


def test_time_weighting_and_pause_boundaries():
    rows = []
    for stamp, error, episode in ((0.0, 1.0, 1), (0.01, 3.0, 1),
                                 (0.10, 7.0, 1), (0.11, 100.0, 2), (0.12, 0.0, 2)):
        rows.append({'stamp_sec': stamp, 'absolute_error': error, 'episode': episode,
                     'orientation_error': 0.0, 'dx': error, 'dy': 0.0, 'dz': 0.0,
                     'along_track_error': error, 'lateral_error': 0.0, 'spray_axis_error': 0.0,
                     'planar_cross_track_error': 0.0, 'path_progress': stamp})
    result = summarize_window(rows)
    assert result['observed_duration_seconds'] == pytest.approx(0.11)
    assert result['time_weighted']['mean'] == pytest.approx((0.01 + 0.27 + 1.0) / 0.11)
    assert run_statistics([1])['bootstrap_mean_ci95'] is None
    assert run_statistics([1, 2, 3])['runs'] == 3


def test_report_excludes_incomplete_trials_and_separates_paths(tmp_path):
    for name, digest, complete in (('a', 'path_a', True), ('b', 'path_b', True), ('failed', 'path_a', False)):
        directory = tmp_path / name
        directory.mkdir()
        manifest = {'schema_version': 1, 'session_id': name, 'settings': {}, 'status': 'finished',
                    'bag_metadata_available': True, 'bag_required_topics_present': True,
                    'platform': 'robotnik', 'simulation': False,
                    'control_frame': 'map', 'condition': 'baseline'}
        run = {'reached_path_end': complete, 'path_changed': False, 'valid_sample_fraction': 1.0,
               'path_hashes': {'reference_path': digest}, 'endpoint_window': {},
               'windows': {'active_tracking': {'time_weighted': {'mean': 0.1, 'rmse': 0.1, 'p95': 0.1, 'max': 0.1}},
                           'settling': {}}}
        (directory / 'manifest.json').write_text(json.dumps(manifest))
        for mode in ('base', 'tcp'):
            (directory / f'{mode}.json').write_text(json.dumps(run))
    report = build_report(tmp_path)
    assert len(report['groups']) == 4
    assert len(report['excluded']) == 2
    assert all(group['statistics']['rmse']['runs'] == 1 for group in report['groups'])
