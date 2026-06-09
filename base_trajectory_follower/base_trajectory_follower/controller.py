import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class VelocityCommand:
    vx: float
    vy: float
    wz: float
    reached_goal: bool = False


@dataclass(frozen=True)
class FollowerGains:
    kp_x: float = 0.8
    kp_y: float = 0.8
    kp_yaw: float = 1.2


@dataclass(frozen=True)
class FollowerLimits:
    max_vx: float = 0.4
    max_vy: float = 0.4
    max_wz: float = 0.8


@dataclass(frozen=True)
class FollowerTolerances:
    xy_goal_tolerance: float = 0.05
    yaw_goal_tolerance: float = 0.08


def wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value: float, limit: float) -> float:
    limit = abs(float(limit))
    return max(-limit, min(limit, value))


def distance_xy(a: Pose2D, b: Pose2D) -> float:
    return math.hypot(b.x - a.x, b.y - a.y)


def select_lookahead_index(
    path: Sequence[Pose2D],
    robot_pose: Pose2D,
    lookahead_distance: float,
    current_index: int = 0,
) -> int:
    if not path:
        return 0

    start_index = max(0, min(int(current_index), len(path) - 1))
    nearest_index = min(
        range(start_index, len(path)),
        key=lambda idx: distance_xy(robot_pose, path[idx]),
    )

    lookahead_distance = max(0.0, float(lookahead_distance))
    for idx in range(nearest_index, len(path)):
        if distance_xy(robot_pose, path[idx]) >= lookahead_distance:
            return idx
    return len(path) - 1


def compute_velocity_command(
    robot_pose: Pose2D,
    target_pose: Pose2D,
    final_pose: Pose2D,
    gains: FollowerGains,
    limits: FollowerLimits,
    tolerances: FollowerTolerances,
) -> VelocityCommand:
    goal_distance = distance_xy(robot_pose, final_pose)
    goal_yaw_error = wrap_to_pi(final_pose.yaw - robot_pose.yaw)
    if (
        goal_distance <= tolerances.xy_goal_tolerance
        and abs(goal_yaw_error) <= tolerances.yaw_goal_tolerance
    ):
        return VelocityCommand(0.0, 0.0, 0.0, reached_goal=True)

    dx_world = target_pose.x - robot_pose.x
    dy_world = target_pose.y - robot_pose.y
    cos_yaw = math.cos(robot_pose.yaw)
    sin_yaw = math.sin(robot_pose.yaw)
    dx_robot = cos_yaw * dx_world + sin_yaw * dy_world
    dy_robot = -sin_yaw * dx_world + cos_yaw * dy_world

    yaw_target = final_pose.yaw if target_pose is final_pose else target_pose.yaw
    yaw_error = wrap_to_pi(yaw_target - robot_pose.yaw)

    return VelocityCommand(
        vx=clamp(gains.kp_x * dx_robot, limits.max_vx),
        vy=clamp(gains.kp_y * dy_robot, limits.max_vy),
        wz=clamp(gains.kp_yaw * yaw_error, limits.max_wz),
        reached_goal=False,
    )


def pose_from_xy_yaw(values: Tuple[float, float, float]) -> Pose2D:
    return Pose2D(float(values[0]), float(values[1]), float(values[2]))
