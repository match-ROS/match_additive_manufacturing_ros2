import math
from dataclasses import dataclass
from typing import Iterable


@dataclass(frozen=True)
class PoseError:
    dx: float
    dy: float
    dz: float
    distance: float
    yaw_error: float


@dataclass(frozen=True)
class PlanarError:
    tangential: float
    cross_track: float


def wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def compute_pose_error(current, reference) -> PoseError:
    dx = reference.position.x - current.position.x
    dy = reference.position.y - current.position.y
    dz = reference.position.z - current.position.z
    distance = math.sqrt(dx * dx + dy * dy + dz * dz)
    current_yaw = yaw_from_quaternion(
        current.orientation.x,
        current.orientation.y,
        current.orientation.z,
        current.orientation.w,
    )
    reference_yaw = yaw_from_quaternion(
        reference.orientation.x,
        reference.orientation.y,
        reference.orientation.z,
        reference.orientation.w,
    )
    return PoseError(dx, dy, dz, distance, wrap_to_pi(reference_yaw - current_yaw))


def compute_planar_error(current, reference, previous=None, following=None) -> PlanarError:
    """Resolve the XY position error into local path tangent/cross-track axes."""
    start = previous.position if previous is not None else reference.position
    end = following.position if following is not None else reference.position
    tx = float(end.x) - float(start.x)
    ty = float(end.y) - float(start.y)
    length = math.hypot(tx, ty)
    if length < 1e-9:
        return PlanarError(0.0, 0.0)
    tx /= length
    ty /= length
    error = compute_pose_error(current, reference)
    return PlanarError(
        tangential=error.dx * tx + error.dy * ty,
        cross_track=error.dx * -ty + error.dy * tx,
    )


def summarize_distances(distances: Iterable[float]) -> dict[str, float]:
    values = sorted(float(value) for value in distances)
    if not values:
        return {}
    count = len(values)
    mean = sum(values) / count
    percentile_index = min(count - 1, math.ceil(0.95 * count) - 1)
    return {
        'count': float(count),
        'mean': mean,
        'rmse': math.sqrt(sum(value * value for value in values) / count),
        'p95': values[percentile_index],
        'max': values[-1],
        'stddev': math.sqrt(sum((value - mean) ** 2 for value in values) / count),
    }
