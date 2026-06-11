import math

import numpy as np

from geometry_msgs.msg import PoseStamped, Vector3
from nav_msgs.msg import Path
from parse_paths.publish_robotnik_base_arm_paths import (
    base_to_arm_planar_distances,
    generate_arm_points,
    generate_sideways_then_diagonal_points,
    path_from_dict,
    path_to_dict,
    quaternion_from_yaw,
    rotate_xy,
    vector3_from_dict,
    vector3_to_dict,
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
    assert np.allclose(arm_points[0], [0.5, 0.2, 0.8])
    assert np.allclose(arm_points[1], [0.55, 1.15, 0.9])
    assert np.allclose(arm_points[2], [1.6, 2.1, 1.0])


def test_default_robotnik_arm_path_stays_in_conservative_planar_reach():
    base_start = np.array([0.0, 0.0, 0.0])
    arm_start = np.array([0.5, 0.2, 0.8])
    base_points = generate_sideways_then_diagonal_points(
        base_start,
        0.0,
        0.8,
        0.8,
        50,
    )

    arm_points = generate_arm_points(
        arm_start,
        base_points,
        base_start,
        np.array([0.15, 0.0, 0.0]),
        0.2,
    )
    distances = base_to_arm_planar_distances(base_points, arm_points)

    assert len(distances) == len(base_points)
    assert min(distances) > 0.25
    assert max(distances) < 0.85


def test_path_json_round_trip_preserves_pose_count_and_frame():
    path = Path()
    path.header.frame_id = 'robotnik_simple'
    orientation = quaternion_from_yaw(0.5)
    for x, y, z in [(1.0, 2.0, 0.3), (1.5, 2.5, 0.4)]:
        pose = PoseStamped()
        pose.header.frame_id = 'robotnik_simple'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation = orientation
        path.poses.append(pose)

    restored = path_from_dict(path_to_dict(path), 'fallback')

    assert restored.header.frame_id == 'robotnik_simple'
    assert len(restored.poses) == 2
    assert restored.poses[1].pose.position.x == 1.5
    assert restored.poses[1].pose.orientation.z == orientation.z


def test_normal_vector_json_round_trip():
    restored = vector3_from_dict(vector3_to_dict(Vector3(x=0.0, y=1.0, z=0.0)))

    assert restored.x == 0.0
    assert restored.y == 1.0
    assert restored.z == 0.0
