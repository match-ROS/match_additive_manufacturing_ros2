import math
from copy import deepcopy
from typing import Sequence

from geometry_msgs.msg import Quaternion, Vector3
from nav_msgs.msg import Path
from tf_transformations import quaternion_from_euler, quaternion_multiply


def quaternion_from_yaw(yaw: float) -> Quaternion:
    quat = quaternion_from_euler(0.0, 0.0, yaw)
    return Quaternion(x=float(quat[0]), y=float(quat[1]), z=float(quat[2]), w=float(quat[3]))


def rotate_vector_z(x: float, y: float, z: float, yaw: float) -> tuple[float, float, float]:
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (
        cos_yaw * float(x) - sin_yaw * float(y),
        sin_yaw * float(x) + cos_yaw * float(y),
        float(z),
    )


def transform_quaternion_z(orientation: Quaternion, yaw: float) -> Quaternion:
    rotation = quaternion_from_euler(0.0, 0.0, yaw)
    current = [
        float(orientation.x),
        float(orientation.y),
        float(orientation.z),
        float(orientation.w),
    ]
    quat = quaternion_multiply(rotation, current)
    return Quaternion(x=float(quat[0]), y=float(quat[1]), z=float(quat[2]), w=float(quat[3]))


def transform_path(path: Path, translation: Sequence[float], yaw_degrees: float) -> Path:
    transformed = deepcopy(path)
    tx, ty, tz = (list(translation) + [0.0, 0.0, 0.0])[:3]
    yaw = math.radians(float(yaw_degrees))
    for pose_stamped in transformed.poses:
        position = pose_stamped.pose.position
        x, y, z = rotate_vector_z(position.x, position.y, position.z, yaw)
        position.x = x + float(tx)
        position.y = y + float(ty)
        position.z = z + float(tz)
        pose_stamped.pose.orientation = transform_quaternion_z(
            pose_stamped.pose.orientation,
            yaw,
        )
    return transformed


def transform_vector(vector: Vector3, yaw_degrees: float) -> Vector3:
    x, y, z = rotate_vector_z(
        vector.x,
        vector.y,
        vector.z,
        math.radians(float(yaw_degrees)),
    )
    return Vector3(x=x, y=y, z=z)
