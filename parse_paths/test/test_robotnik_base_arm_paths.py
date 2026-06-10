import math

import numpy as np

from parse_paths.publish_robotnik_base_arm_paths import (
    generate_arm_points,
    generate_sideways_then_diagonal_points,
    rotate_xy,
)


def test_robotnik_base_path_keeps_requested_count():
    points = generate_sideways_then_diagonal_points(
        np.array([0.0, 0.0, 0.0]),
        0.0,
        0.8,
        0.8,
        17,
    )

    assert len(points) == 17
    assert points[0].tolist() == [0.0, 0.0, 0.0]


def test_robotnik_base_path_starts_sideways_then_turns_diagonal():
    points = generate_sideways_then_diagonal_points(
        np.array([0.0, 0.0, 0.0]),
        0.0,
        1.0,
        math.sqrt(2.0),
        20,
    )

    early_delta = points[1] - points[0]
    final_delta = points[-1] - points[-2]
    assert abs(early_delta[1]) > abs(early_delta[0])
    assert np.allclose(final_delta[0], final_delta[1])
    assert np.allclose(points[-1], [1.0, 2.0, 0.0])


def test_robotnik_base_path_respects_start_yaw():
    points = generate_sideways_then_diagonal_points(
        np.array([1.0, 2.0, 0.0]),
        math.pi / 2.0,
        1.0,
        math.sqrt(2.0),
        20,
    )

    assert points[1][0] < points[0][0]
    assert np.allclose(points[-1], [-1.0, 3.0, 0.0])


def test_base_start_offset_can_be_applied_in_robot_frame():
    base_pose = np.array([1.0, 2.0, 0.0])
    start_offset = np.array([0.35, 0.0, 0.0])
    base_yaw = math.pi / 2.0

    points = generate_sideways_then_diagonal_points(
        base_pose + rotate_xy(start_offset, base_yaw),
        base_yaw,
        1.0,
        math.sqrt(2.0),
        20,
    )

    assert np.allclose(points[0], [1.0, 2.35, 0.0])


def test_arm_path_tracks_base_displacement_with_offset_and_height_change():
    base_start = np.array([0.0, 0.0, 0.0])
    arm_start = np.array([0.5, 0.2, 0.8])
    base_points = [
        np.array([0.0, 0.0, 0.0]),
        np.array([0.0, 1.0, 0.0]),
        np.array([1.0, 2.0, 0.0]),
    ]

    arm_points = generate_arm_points(
        arm_start,
        base_points,
        base_start,
        np.array([0.1, -0.1, 0.0]),
        0.2,
    )

    assert len(arm_points) == len(base_points)
    assert np.allclose(arm_points[0], [0.6, 0.1, 0.8])
    assert np.allclose(arm_points[1], [0.6, 1.1, 0.9])
    assert np.allclose(arm_points[2], [1.6, 2.1, 1.0])
