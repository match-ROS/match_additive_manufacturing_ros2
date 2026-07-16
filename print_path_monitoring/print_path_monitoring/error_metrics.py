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


@dataclass(frozen=True)
class TrackingError:
    """Cartesian error resolved in the same axes as the arm tracker."""

    along_track: float
    lateral: float
    spray_axis: float


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


def tool_z_axis(pose) -> tuple[float, float, float]:
    """Return the reference tool-Z axis in the pose frame."""
    q = pose.orientation
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if norm < 1e-9:
        return (0.0, 0.0, 1.0)
    x, y, z, w = q.x / norm, q.y / norm, q.z / norm, q.w / norm
    return (2.0 * (x * z + w * y), 2.0 * (y * z - w * x), 1.0 - 2.0 * (x * x + y * y))


def compute_tracking_error(current, reference, tangent, spray_axis) -> TrackingError:
    """Resolve position error into along-track, lateral, and spray-axis parts.

    This is deliberately the same orthogonal decomposition used by
    ``cartesian_tracking_command``: the tangent is projected out of the spray
    axis plane, then the residual planar error is lateral.
    """
    error = compute_pose_error(current, reference)
    ex, ey, ez = error.dx, error.dy, error.dz
    sx, sy, sz = (float(component) for component in spray_axis)
    spray_norm = math.sqrt(sx * sx + sy * sy + sz * sz)
    if spray_norm < 1e-9:
        sx, sy, sz = 0.0, 0.0, 1.0
    else:
        sx, sy, sz = sx / spray_norm, sy / spray_norm, sz / spray_norm
    spray = ex * sx + ey * sy + ez * sz
    px, py, pz = ex - spray * sx, ey - spray * sy, ez - spray * sz

    tx, ty, tz = (float(component) for component in tangent)
    tangent_spray = tx * sx + ty * sy + tz * sz
    tx, ty, tz = tx - tangent_spray * sx, ty - tangent_spray * sy, tz - tangent_spray * sz
    tangent_norm = math.sqrt(tx * tx + ty * ty + tz * tz)
    if tangent_norm < 1e-9:
        return TrackingError(0.0, math.sqrt(px * px + py * py + pz * pz), spray)
    tx, ty, tz = tx / tangent_norm, ty / tangent_norm, tz / tangent_norm
    along = px * tx + py * ty + pz * tz
    lx, ly, lz = px - along * tx, py - along * ty, pz - along * tz
    return TrackingError(along, math.sqrt(lx * lx + ly * ly + lz * lz), spray)


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
