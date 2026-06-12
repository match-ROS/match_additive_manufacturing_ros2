import numpy as np

from ur_trajectory_follower.direction_control import (
    has_forward_segment,
    segment_speed,
    speed_dependent_orthogonal_command,
)


def test_segment_speed_uses_full_3d_distance():
    speed = segment_speed(
        np.array([0.0, 0.0, 0.0]),
        np.array([0.0, 3.0, 4.0]),
        2.0,
    )

    assert speed == 2.5


def test_forward_segment_stops_at_final_path_index():
    assert has_forward_segment(0, 2)
    assert has_forward_segment(3, 5)
    assert not has_forward_segment(4, 5)
    assert not has_forward_segment(0, 1)


def test_feedforward_uses_trajectory_speed_and_override():
    result = speed_dependent_orthogonal_command(
        current=np.array([0.4, 0.0, 0.0]),
        segment_start=np.array([0.0, 0.0, 0.0]),
        segment_goal=np.array([1.0, 0.0, 0.0]),
        spray_axis=np.array([0.0, 0.0, 1.0]),
        trajectory_speed=0.3,
        velocity_override=0.5,
        orthogonal_kp=1.0,
        orthogonal_max_velocity=0.1,
    )

    assert np.allclose(result.feedforward, [0.15, 0.0, 0.0])
    assert np.allclose(result.correction, [0.0, 0.0, 0.0])
    assert np.allclose(result.command, [0.15, 0.0, 0.0])


def test_cross_track_correction_is_orthogonal_to_path_and_spray_axis():
    result = speed_dependent_orthogonal_command(
        current=np.array([0.4, 0.2, 0.3]),
        segment_start=np.array([0.0, 0.0, 0.0]),
        segment_goal=np.array([1.0, 0.0, 0.0]),
        spray_axis=np.array([0.0, 0.0, 1.0]),
        trajectory_speed=0.2,
        velocity_override=1.0,
        orthogonal_kp=0.5,
        orthogonal_max_velocity=0.2,
    )

    assert np.allclose(result.cross_track_error, [0.0, -0.2, 0.0])
    assert np.allclose(result.correction, [0.0, -0.1, 0.0])
    assert np.isclose(np.dot(result.correction, result.tangent), 0.0)
    assert np.isclose(np.dot(result.correction, [0.0, 0.0, 1.0]), 0.0)
    assert np.allclose(result.command, [0.2, -0.1, 0.0])


def test_cross_track_correction_is_limited():
    result = speed_dependent_orthogonal_command(
        current=np.array([0.0, 1.0, 0.0]),
        segment_start=np.array([0.0, 0.0, 0.0]),
        segment_goal=np.array([1.0, 0.0, 0.0]),
        spray_axis=np.array([0.0, 0.0, 1.0]),
        trajectory_speed=0.2,
        velocity_override=1.0,
        orthogonal_kp=2.0,
        orthogonal_max_velocity=0.05,
    )

    assert np.isclose(np.linalg.norm(result.correction), 0.05)
    assert result.correction[1] < 0.0
