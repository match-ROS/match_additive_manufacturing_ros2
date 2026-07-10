import math

from geometry_msgs.msg import Pose
from tf_transformations import quaternion_from_euler

from print_path_monitoring.error_metrics import (
    compute_planar_error,
    compute_pose_error,
    summarize_distances,
    wrap_to_pi,
)


def _pose(x, y, z, yaw):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    quat = quaternion_from_euler(0.0, 0.0, yaw)
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]
    return pose


def test_compute_pose_error_position_and_yaw():
    current = _pose(1.0, 2.0, 3.0, 0.0)
    reference = _pose(2.0, 4.0, 6.0, math.pi / 2.0)

    error = compute_pose_error(current, reference)

    assert error.dx == 1.0
    assert error.dy == 2.0
    assert error.dz == 3.0
    assert math.isclose(error.distance, math.sqrt(14.0))
    assert math.isclose(error.yaw_error, math.pi / 2.0)


def test_wrap_to_pi_large_angle():
    assert math.isclose(abs(wrap_to_pi(3.0 * math.pi)), math.pi)


def test_planar_error_is_resolved_into_path_axes():
    current = _pose(0.8, 0.3, 2.0, 0.0)
    reference = _pose(1.0, 0.0, 2.0, 0.0)
    following = _pose(2.0, 0.0, 2.0, 0.0)

    planar = compute_planar_error(current, reference, reference, following)

    assert math.isclose(planar.tangential, 0.2)
    assert math.isclose(planar.cross_track, -0.3)


def test_distance_summary_has_rmse_percentile_and_spread():
    summary = summarize_distances([0.0, 1.0, 2.0, 3.0])

    assert summary['count'] == 4.0
    assert math.isclose(summary['mean'], 1.5)
    assert math.isclose(summary['rmse'], math.sqrt(3.5))
    assert summary['p95'] == 3.0
    assert summary['max'] == 3.0
