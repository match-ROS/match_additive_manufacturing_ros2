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


@dataclass(frozen=True)
class PurePursuitGains:
    kv: float = 1.0
    kw: float = 1.0
    ky: float = 0.3
    k_distance: float = 0.0
    k_orientation: float = 0.5
    k_index: float = 0.02


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


def select_anchored_lookahead_index(
    path: Sequence[Pose2D],
    anchor_index: int,
    lookahead_distance: float,
) -> int:
    """Return a point a path-distance ahead of a fixed progress anchor.

    Unlike :func:`select_lookahead_index`, this must not search for the
    geometrically nearest future pose.  An external path index is an explicit
    progress contract; choosing a nearest pose from the remaining path can
    skip across adjacent or crossing path sections and command the base toward
    the final pose prematurely.
    """
    if not path:
        return 0

    index = max(0, min(int(anchor_index), len(path) - 1))
    remaining_distance = max(0.0, float(lookahead_distance))
    if remaining_distance <= 0.0:
        return index

    for next_index in range(index + 1, len(path)):
        remaining_distance -= distance_xy(path[next_index - 1], path[next_index])
        if remaining_distance <= 0.0:
            return next_index
    return len(path) - 1


def compute_velocity_command(
    robot_pose: Pose2D,
    target_pose: Pose2D,
    final_pose: Pose2D,
    gains: FollowerGains,
    limits: FollowerLimits,
    tolerances: FollowerTolerances,
    diff_drive_mode: bool = False,
    velocity_override: float = 1.0,
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
    vx = gains.kp_x * dx_robot
    vy = gains.kp_y * dy_robot

    wz = gains.kp_yaw * yaw_error
    if diff_drive_mode:
        heading_error = math.atan2(dy_robot, dx_robot) if math.hypot(dx_robot, dy_robot) > 1e-9 else 0.0
        wz += gains.kp_y * heading_error
        vy = 0.0

    # Keep PID subject to the same live speed override as Pure Pursuit.  The
    # override is intentionally applied before limiting so the configured
    # limits remain hard safety limits even when an override above 100 % is
    # received.
    velocity_scale = max(0.0, float(velocity_override))
    return VelocityCommand(
        vx=clamp(vx * velocity_scale, limits.max_vx),
        vy=clamp(vy * velocity_scale, limits.max_vy),
        wz=clamp(wz * velocity_scale, limits.max_wz),
        reached_goal=False,
    )


def path_segment_velocity(
    path: Sequence[Pose2D],
    index: int,
    timestamps: Optional[Sequence[float]] = None,
    fallback_dt: float = 0.1,
) -> tuple[float, float]:
    if len(path) < 2:
        return 0.0, 0.0
    idx = max(1, min(int(index), len(path) - 1))
    previous = path[idx - 1]
    current = path[idx]
    dt = max(1e-3, float(fallback_dt))
    if timestamps is not None and len(timestamps) > idx:
        stamped_dt = float(timestamps[idx]) - float(timestamps[idx - 1])
        if stamped_dt > 0.0:
            dt = stamped_dt
    return distance_xy(previous, current) / dt, wrap_to_pi(current.yaw - previous.yaw) / dt


def compute_pure_pursuit_command(
    robot_pose: Pose2D,
    path: Sequence[Pose2D],
    current_index: int,
    target_index: int,
    timestamps: Optional[Sequence[float]],
    gains: PurePursuitGains,
    limits: FollowerLimits,
    tolerances: FollowerTolerances,
    velocity_override: float = 1.0,
    fallback_dt: float = 0.1,
    diff_drive_mode: bool = False,
) -> VelocityCommand:
    if not path:
        return VelocityCommand(0.0, 0.0, 0.0)

    final_pose = path[-1]
    goal_distance = distance_xy(robot_pose, final_pose)
    goal_yaw_error = wrap_to_pi(final_pose.yaw - robot_pose.yaw)
    if (
        goal_distance <= tolerances.xy_goal_tolerance
        and abs(goal_yaw_error) <= tolerances.yaw_goal_tolerance
    ):
        return VelocityCommand(0.0, 0.0, 0.0, reached_goal=True)

    path_index = max(0, min(int(current_index), len(path) - 1))
    lookahead_index = max(path_index, min(int(target_index), len(path) - 1))
    target_pose = path[lookahead_index]
    tracking_pose = path[path_index]

    dx_world = target_pose.x - robot_pose.x
    dy_world = target_pose.y - robot_pose.y
    cos_yaw = math.cos(robot_pose.yaw)
    sin_yaw = math.sin(robot_pose.yaw)
    dx_robot = cos_yaw * dx_world + sin_yaw * dy_world
    dy_robot = -sin_yaw * dx_world + cos_yaw * dy_world

    feedforward_v, feedforward_w = path_segment_velocity(
        path,
        path_index,
        timestamps=timestamps,
        fallback_dt=fallback_dt,
    )
    distance_error = distance_xy(robot_pose, tracking_pose)
    orientation_error = wrap_to_pi(tracking_pose.yaw - robot_pose.yaw)
    index_error = int(current_index) - int(path_index)
    velocity_scale = max(0.0, float(velocity_override))

    if diff_drive_mode:
        lateral_error = dy_robot
        target_v = gains.kv * feedforward_v + gains.k_distance * distance_error
        target_v *= 1.0 + gains.k_index * index_error
        target_v = max(0.0, target_v) * velocity_scale
        target_w = (
            gains.kw * feedforward_w
            + gains.k_orientation * math.sin(orientation_error)
            + gains.ky * lateral_error * (1.0 if target_v >= 0.0 else -1.0)
        )
        target_w *= 1.0 + gains.k_index * index_error
        target_w *= velocity_scale
        return VelocityCommand(
            vx=clamp(target_v, limits.max_vx),
            vy=0.0,
            wz=clamp(target_w, limits.max_wz),
            reached_goal=False,
        )

    target_speed = gains.kv * feedforward_v + gains.k_distance * distance_error
    target_speed = max(0.0, target_speed) * velocity_scale
    distance_to_target = math.hypot(dx_robot, dy_robot)
    if distance_to_target > 1e-9:
        vx = dx_robot / distance_to_target * target_speed
        vy = dy_robot / distance_to_target * target_speed
    else:
        vx = 0.0
        vy = 0.0
    wz = (gains.kw * feedforward_w + gains.k_orientation * math.sin(orientation_error)) * velocity_scale

    return VelocityCommand(
        vx=clamp(vx, limits.max_vx),
        vy=clamp(vy, limits.max_vy),
        wz=clamp(wz, limits.max_wz),
        reached_goal=False,
    )


def pose_from_xy_yaw(values: Tuple[float, float, float]) -> Pose2D:
    return Pose2D(float(values[0]), float(values[1]), float(values[2]))
