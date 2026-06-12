import math

from geometry_msgs.msg import Pose
from tf_transformations import quaternion_from_euler

from print_path_monitoring.error_metrics import compute_pose_error, wrap_to_pi


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
