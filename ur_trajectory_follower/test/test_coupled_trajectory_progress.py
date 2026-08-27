import math
import json
from pathlib import Path as FilePath

from geometry_msgs.msg import PoseStamped

from nav_msgs.msg import Path

from ur_trajectory_follower.increment_path_index import (
    interpolate_pose,
    paths_have_same_trajectory,
    position_distance,
    resample_coupled_paths,
    stamp_seconds,
)


def _pose(x: float, yaw_quaternion_z: float = 0.0, yaw_quaternion_w: float = 1.0) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = x
    pose.pose.orientation.z = yaw_quaternion_z
    pose.pose.orientation.w = yaw_quaternion_w
    return pose


def test_interpolated_reference_has_continuous_position_and_orientation():
    start = _pose(0.0)
    goal = _pose(1.0, 1.0, 0.0)

    midpoint = interpolate_pose(start, goal, 0.5)

    assert math.isclose(midpoint.pose.position.x, 0.5)
    assert math.isclose(abs(midpoint.pose.orientation.z), math.sqrt(0.5), rel_tol=1e-6)
    assert math.isclose(abs(midpoint.pose.orientation.w), math.sqrt(0.5), rel_tol=1e-6)


def test_zero_length_arm_segment_is_preserved_for_progress_classification():
    start = _pose(1.0)
    goal = _pose(1.0)

    assert position_distance(start, goal) == 0.0


def test_republished_path_with_only_a_new_header_stamp_is_unchanged():
    first = Path()
    first.header.frame_id = 'map'
    first.header.stamp.sec = 10
    first.poses = [_pose(0.0), _pose(1.0)]
    second = Path()
    second.header.frame_id = 'map'
    second.header.stamp.sec = 11
    second.poses = [_pose(0.0), _pose(1.0)]

    assert paths_have_same_trajectory(first, second)

    second.poses[1].pose.position.x = 2.0
    assert not paths_have_same_trajectory(first, second)


def test_resampling_makes_arm_segments_uniform_and_preserves_timing_profile():
    arm, base = Path(), Path()
    for x, y, stamp in ((0.0, 0.0, 0.0), (0.012, 0.024, 1.0), (0.032, 0.064, 4.0)):
        arm_pose = _pose(x)
        arm_pose.header.stamp.sec = int(stamp)
        base_pose = _pose(y)
        base_pose.header.stamp.sec = int(stamp)
        arm.poses.append(arm_pose)
        base.poses.append(base_pose)

    arm_resampled, base_resampled = resample_coupled_paths(arm, base, 0.01)

    assert [round(p.pose.position.x, 3) for p in arm_resampled.poses] == [0.0, 0.01, 0.02, 0.03, 0.032]
    assert [round(stamp_seconds(p), 3) for p in arm_resampled.poses] == [0.0, 0.833, 2.2, 3.7, 4.0]
    assert base_resampled is not None
    assert len(base_resampled.poses) == len(arm_resampled.poses)
    assert [round(stamp_seconds(p), 3) for p in base_resampled.poses] == [round(stamp_seconds(p), 3) for p in arm_resampled.poses]
    lengths = [position_distance(a, b) for a, b in zip(arm_resampled.poses, arm_resampled.poses[1:])]
    assert all(math.isclose(length, 0.01, abs_tol=1e-9) for length in lengths[:-1])
    assert math.isclose(lengths[-1], 0.002, abs_tol=1e-9)


def test_resampling_keeps_zero_displacement_intervals_for_dwell_semantics():
    arm = Path()
    for x, stamp in ((0.0, 0), (0.0, 1), (0.02, 3)):
        pose = _pose(x)
        pose.header.stamp.sec = stamp
        arm.poses.append(pose)

    result, _ = resample_coupled_paths(arm, None, 0.01)

    assert [round(stamp_seconds(p), 3) for p in result.poses] == [0.0, 1.0, 2.0, 3.0]
    assert [round(p.pose.position.x, 3) for p in result.poses] == [0.0, 0.0, 0.01, 0.02]


def test_david_original_index_28_maps_to_a_tracking_target_within_2cm():
    """The source-index GUI selector must lead to the shared tracking target."""
    trajectory_file = (
        FilePath(__file__).resolve().parents[2]
        / 'components' / 'david_path' / 'arm_path.json'
    )
    data = json.loads(trajectory_file.read_text(encoding='utf-8'))
    original = Path()
    original.header.frame_id = data['frame_id']
    for item in data['poses']:
        pose = PoseStamped()
        pose.header.frame_id = data['frame_id']
        pose.pose.position.x = item['position']['x']
        pose.pose.position.y = item['position']['y']
        pose.pose.position.z = item['position']['z']
        pose.pose.orientation.x = item['orientation']['x']
        pose.pose.orientation.y = item['orientation']['y']
        pose.pose.orientation.z = item['orientation']['z']
        pose.pose.orientation.w = item['orientation']['w']
        original.poses.append(pose)

    tracking, _ = resample_coupled_paths(original, None, 0.005)
    source_lengths = [0.0]
    for previous, current in zip(original.poses, original.poses[1:]):
        source_lengths.append(source_lengths[-1] + position_distance(previous, current))
    tracking_lengths = [0.0]
    for previous, current in zip(tracking.poses, tracking.poses[1:]):
        tracking_lengths.append(tracking_lengths[-1] + position_distance(previous, current))
    tracking_index = min(
        range(len(tracking_lengths)),
        key=lambda candidate: abs(tracking_lengths[candidate] - source_lengths[28]),
    )
    target = tracking.poses[tracking_index].pose.position
    source = original.poses[28].pose.position

    assert tracking_index == 103
    assert math.dist(
        (source.x, source.y, source.z),
        (target.x, target.y, target.z),
    ) < 0.02
