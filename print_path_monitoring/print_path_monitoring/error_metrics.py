import math
from dataclasses import dataclass


@dataclass(frozen=True)
class PoseError:
    dx: float
    dy: float
    dz: float
    distance: float
    yaw_error: float


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
