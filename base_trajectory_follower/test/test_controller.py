import math

from base_trajectory_follower.controller import (
    FollowerGains,
    FollowerLimits,
    FollowerTolerances,
    Pose2D,
    PurePursuitGains,
    compute_pure_pursuit_command,
    compute_velocity_command,
    select_lookahead_index,
    wrap_to_pi,
)


def test_wrap_to_pi_bounds_large_angle():
    wrapped = wrap_to_pi(3.0 * math.pi)

    assert math.isclose(abs(wrapped), math.pi)


def test_select_lookahead_stays_forward_from_current_index():
    path = [Pose2D(float(i), 0.0, 0.0) for i in range(5)]
    robot = Pose2D(1.1, 0.0, 0.0)

    assert select_lookahead_index(path, robot, 1.0, current_index=1) == 3


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


def test_compute_velocity_can_use_default_linear_speed():
    robot = Pose2D(0.0, 0.0, 0.0)
    target = Pose2D(3.0, 4.0, 0.0)
    command = compute_velocity_command(
        robot,
        target,
        Pose2D(10.0, 0.0, 0.0),
        FollowerGains(kp_x=10.0, kp_y=10.0, kp_yaw=1.0),
        FollowerLimits(max_vx=2.0, max_vy=2.0, max_wz=2.0),
        FollowerTolerances(xy_goal_tolerance=0.01, yaw_goal_tolerance=0.01),
        default_linear_velocity=0.5,
    )

    assert math.isclose(command.vx, 0.3, abs_tol=1e-6)
    assert math.isclose(command.vy, 0.4, abs_tol=1e-6)


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
