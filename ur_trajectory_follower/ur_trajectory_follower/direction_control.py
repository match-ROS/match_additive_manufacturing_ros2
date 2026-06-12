from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class OrthogonalControlResult:
    command: np.ndarray
    feedforward: np.ndarray
    correction: np.ndarray
    cross_track_error: np.ndarray
    tangent: np.ndarray


def normalize(vector: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if norm < 1e-9:
        return np.zeros(3)
    return vector / norm


def project_onto_plane(vector: np.ndarray, plane_normal: np.ndarray) -> np.ndarray:
    normal = normalize(plane_normal)
    if not np.any(normal):
        return vector.copy()
    return vector - float(np.dot(vector, normal)) * normal


def segment_speed(start: np.ndarray, goal: np.ndarray, duration: float) -> float:
    if duration <= 0.0:
        return 0.0
    return float(np.linalg.norm(goal - start)) / duration


def has_forward_segment(current_index: int, path_size: int) -> bool:
    return 0 <= current_index < path_size - 1


def speed_dependent_orthogonal_command(
    current: np.ndarray,
    segment_start: np.ndarray,
    segment_goal: np.ndarray,
    spray_axis: np.ndarray,
    trajectory_speed: float,
    velocity_override: float,
    orthogonal_kp: float,
    orthogonal_max_velocity: float,
) -> OrthogonalControlResult:
    tangent = normalize(project_onto_plane(segment_goal - segment_start, spray_axis))
    commanded_speed = max(0.0, trajectory_speed) * max(0.0, velocity_override)
    feedforward = tangent * commanded_speed

    goal_error_plane = project_onto_plane(segment_goal - current, spray_axis)
    along_track_error = float(np.dot(goal_error_plane, tangent)) * tangent
    cross_track_error = goal_error_plane - along_track_error
    correction = cross_track_error * max(0.0, orthogonal_kp)

    max_correction = max(0.0, orthogonal_max_velocity)
    correction_norm = float(np.linalg.norm(correction))
    if max_correction > 0.0 and correction_norm > max_correction:
        correction *= max_correction / correction_norm
    elif max_correction == 0.0:
        correction = np.zeros(3)

    return OrthogonalControlResult(
        command=feedforward + correction,
        feedforward=feedforward,
        correction=correction,
        cross_track_error=cross_track_error,
        tangent=tangent,
    )
