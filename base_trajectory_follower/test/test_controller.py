import json
import math
from pathlib import Path

from base_trajectory_follower.controller import (
    FollowerGains,
    FollowerLimits,
    FollowerTolerances,
    Pose2D,
    PurePursuitGains,
    advance_geometric_path_index,
    cumulative_path_arc_lengths,
    compute_pure_pursuit_command,
    compute_velocity_command,
    path_arc_length_error,
    progress_index_after_reference_seek,
    select_anchored_lookahead_index,
    select_lookahead_index,
    wrap_to_pi,
)


def _exported_mur_base_path():
    """Load the MuR-right regression trajectory as the controller sees it."""
    exported_path = (
        Path(__file__).resolve().parents[2]
        / 'components' / 'doubleCurvedTElement' / 'base_path.json'
    )
    rows = json.loads(exported_path.read_text(encoding='utf-8'))['poses']
    path = []
    for row in rows:
        position = row['position']
        orientation = row['orientation']
        yaw = math.atan2(
            2.0 * (
                orientation['w'] * orientation['z']
                + orientation['x'] * orientation['y']
            ),
            1.0 - 2.0 * (
                orientation['y'] * orientation['y']
                + orientation['z'] * orientation['z']
            ),
        )
        path.append(Pose2D(position['x'], position['y'], yaw))
    return path


def test_wrap_to_pi_bounds_large_angle():
    wrapped = wrap_to_pi(3.0 * math.pi)

    assert math.isclose(abs(wrapped), math.pi)


def test_select_lookahead_stays_forward_from_current_index():
    path = [Pose2D(float(i), 0.0, 0.0) for i in range(5)]
    robot = Pose2D(1.1, 0.0, 0.0)

    assert select_lookahead_index(path, robot, 1.0, current_index=1) == 3


def test_anchored_lookahead_cannot_skip_to_a_nearby_later_path_section():
    # The final pose is physically close to the anchor but lies far ahead in
    # path progress.  An externally supplied index must not shortcut to it.
    path = [
        Pose2D(0.0, 0.0, 0.0),
        Pose2D(1.0, 0.0, 0.0),
        Pose2D(2.0, 0.0, 0.0),
        Pose2D(2.0, 1.0, 0.0),
        Pose2D(0.0, 0.1, 0.0),
    ]

    assert select_anchored_lookahead_index(path, anchor_index=0, lookahead_distance=1.5) == 2


def test_compute_velocity_uses_robot_frame_lateral_error():
    robot = Pose2D(0.0, 0.0, math.pi / 2.0)
    target = Pose2D(1.0, 0.0, math.pi / 2.0)
    command = compute_velocity_command(
        robot,
        target,
        target,
        FollowerGains(kp_x=1.0, kp_y=1.0, kp_yaw=1.0),
        FollowerLimits(max_vx=2.0, max_vy=2.0, max_wz=2.0),
        FollowerTolerances(xy_goal_tolerance=0.01, yaw_goal_tolerance=0.01),
    )

    assert abs(command.vx) < 1e-6
    assert math.isclose(command.vy, -1.0, abs_tol=1e-6)


def test_compute_velocity_reports_reached_goal():
    pose = Pose2D(1.0, 2.0, 0.1)
    command = compute_velocity_command(
        pose,
        pose,
        pose,
        FollowerGains(),
        FollowerLimits(),
        FollowerTolerances(xy_goal_tolerance=0.05, yaw_goal_tolerance=0.05),
    )

    assert command.reached_goal
    assert command.vx == 0.0
    assert command.vy == 0.0
    assert command.wz == 0.0


def test_compute_velocity_honours_velocity_override():
    command = compute_velocity_command(
        Pose2D(0.0, 0.0, 0.0),
        Pose2D(1.0, 0.0, 0.0),
        Pose2D(10.0, 0.0, 0.0),
        FollowerGains(kp_x=1.0, kp_y=1.0, kp_yaw=1.0),
        FollowerLimits(max_vx=2.0, max_vy=2.0, max_wz=2.0),
        FollowerTolerances(xy_goal_tolerance=0.01, yaw_goal_tolerance=0.01),
        velocity_override=0.25,
    )

    assert math.isclose(command.vx, 0.25, abs_tol=1e-6)


def test_compute_velocity_diff_drive_suppresses_lateral_motion():
    robot = Pose2D(0.0, 0.0, 0.0)
    target = Pose2D(0.0, 1.0, 0.0)
    command = compute_velocity_command(
        robot,
        target,
        Pose2D(10.0, 0.0, 0.0),
        FollowerGains(kp_x=1.0, kp_y=1.0, kp_yaw=1.0),
        FollowerLimits(max_vx=2.0, max_vy=2.0, max_wz=2.0),
        FollowerTolerances(xy_goal_tolerance=0.01, yaw_goal_tolerance=0.01),
        diff_drive_mode=True,
    )

    assert command.vy == 0.0
    assert command.wz > 0.0


def test_pure_pursuit_diff_drive_suppresses_lateral_motion_and_clamps():
    path = [
        Pose2D(0.0, 0.0, 0.0),
        Pose2D(1.0, 1.0, 0.0),
        Pose2D(2.0, 1.0, 0.0),
    ]
    command = compute_pure_pursuit_command(
        Pose2D(0.0, 0.0, 0.0),
        path,
        current_index=1,
        target_index=2,
        timestamps=[0.0, 0.1, 0.2],
        gains=PurePursuitGains(kv=10.0, kw=10.0, ky=10.0),
        limits=FollowerLimits(max_vx=0.2, max_vy=0.2, max_wz=0.3),
        tolerances=FollowerTolerances(xy_goal_tolerance=0.01, yaw_goal_tolerance=0.01),
        diff_drive_mode=True,
    )

    assert math.isclose(command.vx, 0.2)
    assert command.vy == 0.0
    assert abs(command.wz) <= 0.3


def test_pure_pursuit_holonomic_can_publish_lateral_motion():
    path = [
        Pose2D(0.0, 0.0, 0.0),
        Pose2D(0.0, 1.0, 0.0),
        Pose2D(0.0, 2.0, 0.0),
    ]
    command = compute_pure_pursuit_command(
        Pose2D(0.0, 0.0, 0.0),
        path,
        current_index=1,
        target_index=2,
        timestamps=[0.0, 1.0, 2.0],
        gains=PurePursuitGains(kv=1.0),
        limits=FollowerLimits(max_vx=2.0, max_vy=2.0, max_wz=2.0),
        tolerances=FollowerTolerances(xy_goal_tolerance=0.01, yaw_goal_tolerance=0.01),
        diff_drive_mode=False,
    )

    assert abs(command.vx) < 1e-6
    assert command.vy > 0.0


def test_arc_length_error_uses_distance_not_waypoint_count_and_keeps_sign():
    path = [
        Pose2D(0.0, 0.0, 0.0),
        Pose2D(0.1, 0.0, 0.0),
        Pose2D(3.1, 0.0, 0.0),
        Pose2D(3.2, 0.0, 0.0),
    ]
    arc_lengths = cumulative_path_arc_lengths(path)

    assert arc_lengths == [0.0, 0.1, 3.1, 3.2]
    assert math.isclose(path_arc_length_error(arc_lengths, 2, 1), 3.0)
    assert math.isclose(path_arc_length_error(arc_lengths, 1, 2), -3.0)


def test_geometric_progress_advances_sequentially_to_the_closer_dense_waypoint():
    path = [Pose2D(0.1 * index, 0.0, 0.0) for index in range(10)]

    assert advance_geometric_path_index(path, 0, Pose2D(0.64, 0.2, 0.0)) == 6


def test_geometric_progress_skips_yaw_only_waypoints_without_requiring_yaw():
    path = [
        Pose2D(0.0, 0.0, 0.0),
        Pose2D(1.0, 0.0, 0.0),
        Pose2D(1.0, 0.0, math.pi / 2.0),
        Pose2D(2.0, 0.0, math.pi / 2.0),
    ]

    # The yaw-only 1 -> 2 transition is skipped for translational progress
    # despite the robot retaining its original yaw.
    assert advance_geometric_path_index(path, 1, Pose2D(1.0, 0.0, 0.0)) == 2
    assert advance_geometric_path_index(path, 1, Pose2D(1.6, 0.0, 0.0)) == 3


def test_geometric_progress_stays_sequential_at_a_revisited_location():
    path = [
        Pose2D(0.0, 0.0, 0.0),
        Pose2D(1.0, 0.0, 0.0),
        Pose2D(2.0, 0.0, 0.0),
        Pose2D(0.0, 0.0, 0.0),
    ]

    # A later path section occupies the same XY location, but no global
    # nearest-waypoint lookup is allowed to jump to it.
    assert advance_geometric_path_index(path, 0, Pose2D(0.0, 0.0, 0.0)) == 0


def test_geometric_progress_advances_sensibly_through_a_corner():
    path = [
        Pose2D(0.0, 0.0, 0.0),
        Pose2D(1.0, 0.0, 0.0),
        Pose2D(1.0, 1.0, 0.0),
    ]

    before_corner = advance_geometric_path_index(path, 0, Pose2D(0.8, 0.0, 0.0))
    after_corner = advance_geometric_path_index(path, before_corner, Pose2D(1.0, 0.7, 0.0))

    assert before_corner == 1
    assert after_corner == 2


def test_discrete_geometric_progress_avoids_a_stale_yaw_gate_lag_error():
    path = [Pose2D(0.1 * index, 0.0, 0.0) for index in range(9)]
    arc_lengths = cumulative_path_arc_lengths(path)
    progress = advance_geometric_path_index(path, 1, Pose2D(0.79, 0.0, 0.0))

    assert progress == 8
    assert math.isclose(path_arc_length_error(arc_lengths, 8, progress), 0.0)


def test_exported_mur_yaw_only_run_does_not_freeze_progress_near_old_failure():
    """Regression for the yaw-only run that previously left progress near 272.

    The robot's yaw is intentionally irrelevant here.  The actual exported
    MuR-right path contains a zero-XY run from 260 through 271, followed by
    the translating section that reaches the former failure range around 520.
    """
    path = _exported_mur_base_path()
    arc_lengths = cumulative_path_arc_lengths(path)

    # All zero-XY entries are crossed even though this pose retains the yaw at
    # index 260.  The next non-zero segment is not crossed until its XY
    # bisector, preserving the strictly sequential rule.
    assert advance_geometric_path_index(path, 260, path[260]) == 272

    progress = advance_geometric_path_index(path, 272, path[520])
    assert progress == 520
    assert advance_geometric_path_index(path, progress, path[533]) == 533
    assert math.isclose(path_arc_length_error(arc_lengths, 520, progress), 0.0)


def test_discrete_geometric_progress_reports_a_genuine_positive_lag():
    path = [Pose2D(0.1 * index, 0.0, 0.0) for index in range(9)]
    arc_lengths = cumulative_path_arc_lengths(path)
    progress = advance_geometric_path_index(path, 0, Pose2D(0.29, 0.0, 0.0))

    assert progress == 3
    assert math.isclose(path_arc_length_error(arc_lengths, 8, progress), 0.5)


def test_geometric_progress_uses_forward_path_distance_in_all_map_directions():
    cases = [
        (Pose2D(0.0, 0.0, 0.0), Pose2D(1.0, 0.0, 0.0), Pose2D(0.6, 0.0, 0.0)),
        (Pose2D(0.0, 0.0, 0.0), Pose2D(-1.0, 0.0, 0.0), Pose2D(-0.6, 0.0, 0.0)),
        (Pose2D(0.0, 0.0, 0.0), Pose2D(0.0, 1.0, 0.0), Pose2D(0.0, 0.6, 0.0)),
        (Pose2D(0.0, 0.0, 0.0), Pose2D(0.0, -1.0, 0.0), Pose2D(0.0, -0.6, 0.0)),
    ]

    for start, end, robot in cases:
        assert advance_geometric_path_index([start, end], 0, robot) == 1


def test_lateral_offset_does_not_create_a_large_along_path_error():
    path = [Pose2D(0.1 * index, 0.0, 0.0) for index in range(9)]
    arc_lengths = cumulative_path_arc_lengths(path)

    # A large lateral error does not change which dense longitudinal waypoint
    # is nearest in the sequential comparison.
    progress = advance_geometric_path_index(path, 0, Pose2D(0.64, 2.0, 0.0))
    assert progress == 6
    assert math.isclose(path_arc_length_error(arc_lengths, 6, progress), 0.0)


def test_backward_reference_seek_resets_progress_but_forward_reference_does_not():
    assert progress_index_after_reference_seek(8, 6, 3) == 3
    assert progress_index_after_reference_seek(3, 6, 8) == 6


def test_pure_pursuit_progress_correction_is_signed_clamped_and_not_applied_to_rotation():
    path = [
        Pose2D(0.0, 0.0, 0.0),
        Pose2D(1.0, 0.0, 0.0),
        Pose2D(2.0, 0.0, 0.0),
    ]
    common = dict(
        robot_pose=Pose2D(0.0, 0.0, 0.0), path=path, current_index=1, target_index=2,
        timestamps=[0.0, 1.0, 2.0], gains=PurePursuitGains(k_progress=2.0),
        limits=FollowerLimits(max_vx=1.1, max_vy=2.0, max_wz=2.0),
        tolerances=FollowerTolerances(xy_goal_tolerance=0.01, yaw_goal_tolerance=0.01),
        diff_drive_mode=True, max_progress_speed_correction=0.2,
    )
    behind = compute_pure_pursuit_command(progress_error_m=4.0, **common)
    ahead = compute_pure_pursuit_command(progress_error_m=-4.0, **common)
    assert math.isclose(behind.vx, 1.1)  # final velocity is still hard-limited
    assert math.isclose(ahead.vx, 0.8)

    rotation_path = [Pose2D(0.0, 0.0, 0.0), Pose2D(0.0, 0.0, math.pi / 2.0)]
    rotation = compute_pure_pursuit_command(
        Pose2D(0.0, 0.0, 0.0), rotation_path, 1, 1, [0.0, 1.0],
        PurePursuitGains(k_progress=10.0), FollowerLimits(max_vx=2.0, max_vy=2.0, max_wz=2.0),
        FollowerTolerances(xy_goal_tolerance=0.01, yaw_goal_tolerance=0.01),
        diff_drive_mode=True, progress_error_m=10.0, max_progress_speed_correction=1.0,
    )
    assert rotation.vx == 0.0


def test_pure_pursuit_does_not_stop_at_a_revisited_final_pose_before_external_completion():
    # The base path revisits its eventual final pose at the start.  During an
    # externally indexed trajectory this is a normal intermediate pose, not a
    # completion condition.
    path = [
        Pose2D(0.0, 0.0, 0.0),
        Pose2D(1.0, 0.0, 0.0),
        Pose2D(0.0, 0.0, 0.0),
    ]
    common = dict(
        robot_pose=Pose2D(0.0, 0.0, 0.0), path=path, current_index=1, target_index=1,
        timestamps=[0.0, 1.0, 2.0], gains=PurePursuitGains(kv=1.0),
        limits=FollowerLimits(max_vx=2.0, max_vy=2.0, max_wz=2.0),
        tolerances=FollowerTolerances(xy_goal_tolerance=0.05, yaw_goal_tolerance=0.08),
    )

    early = compute_pure_pursuit_command(**common)
    externally_indexed = compute_pure_pursuit_command(check_final_goal=False, **common)

    assert early.reached_goal
    assert early.vx == 0.0
    assert not externally_indexed.reached_goal
    assert externally_indexed.vx > 0.0
