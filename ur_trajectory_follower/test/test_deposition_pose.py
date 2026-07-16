import math

from geometry_msgs.msg import PoseStamped

from ur_trajectory_follower.deposition_pose import (
    clamp_distance_step,
    deposition_pose_from_nozzle,
)


def _identity_pose() -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.orientation.w = 1.0
    return pose


def test_deposition_pose_moves_along_local_nozzle_z() -> None:
    pose = _identity_pose()
    pose.pose.position.x = 1.0

    result = deposition_pose_from_nozzle(pose, 0.2)

    assert result.header.frame_id == 'map'
    assert math.isclose(result.pose.position.x, 1.0)
    assert math.isclose(result.pose.position.z, 0.2)


def test_distance_slew_rate_limits_a_step() -> None:
    assert clamp_distance_step(0.0, 0.3, max_rate=0.02, dt=0.5) == 0.01
    assert clamp_distance_step(0.3, 0.0, max_rate=0.02, dt=0.5) == 0.29
