from typing import Iterable, List, Tuple

import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion, Vector3
from tf_transformations import quaternion_from_matrix


def as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {'1', 'true', 'yes', 'y', 'on'}


def as_float_list(value, fallback: Iterable[float]) -> List[float]:
    if isinstance(value, str):
        raw_items = value.strip().strip('[]').split(',')
        try:
            parsed = [float(item.strip()) for item in raw_items if item.strip()]
        except ValueError:
            return list(fallback)
        return parsed if parsed else list(fallback)
    try:
        return [float(item) for item in value]
    except TypeError:
        return list(fallback)


def normalize(vec: np.ndarray, fallback: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(vec)
    if norm < 1e-6:
        return fallback
    return vec / norm


def build_orientation(nozzle_axis: np.ndarray, x_axis_hint: np.ndarray) -> Tuple[Quaternion, Vector3]:
    z_axis = normalize(nozzle_axis, np.array([0.0, 0.0, 1.0]))
    ref_axis = normalize(x_axis_hint, np.array([1.0, 0.0, 0.0]))
    if abs(float(np.dot(ref_axis, z_axis))) > 0.95:
        ref_axis = np.array([1.0, 0.0, 0.0]) if abs(z_axis[0]) < 0.95 else np.array([0.0, 1.0, 0.0])

    x_axis = normalize(np.cross(ref_axis, z_axis), np.array([1.0, 0.0, 0.0]))
    y_axis = normalize(np.cross(z_axis, x_axis), np.array([0.0, 1.0, 0.0]))
    x_axis = normalize(np.cross(y_axis, z_axis), np.array([1.0, 0.0, 0.0]))

    rotation = np.eye(4)
    rotation[0:3, 0] = x_axis
    rotation[0:3, 1] = y_axis
    rotation[0:3, 2] = z_axis
    quat = quaternion_from_matrix(rotation)
    return (
        Quaternion(x=float(quat[0]), y=float(quat[1]), z=float(quat[2]), w=float(quat[3])),
        Vector3(x=float(z_axis[0]), y=float(z_axis[1]), z=float(z_axis[2])),
    )


def make_pose(frame_id: str, stamp, position: np.ndarray, orientation: Quaternion) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = stamp
    pose.pose.position.x = float(position[0])
    pose.pose.position.y = float(position[1])
    pose.pose.position.z = float(position[2])
    pose.pose.orientation = orientation
    return pose
