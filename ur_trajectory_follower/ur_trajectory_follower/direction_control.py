from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class OrthogonalControlResult:
    command: np.ndarray
    feedforward: np.ndarray
    correction: np.ndarray
    cross_track_error: np.ndarray
    tangent: np.ndarray


@dataclass(frozen=True)
class CartesianTrackingResult:
    command: np.ndarray
    along: np.ndarray
    lateral: np.ndarray
    spray: np.ndarray


def limit_vector(vector: np.ndarray, maximum: float) -> np.ndarray:
    maximum = max(0.0, float(maximum))
    norm = float(np.linalg.norm(vector))
    if maximum <= 0.0:
        return np.zeros(3)
    return vector * min(1.0, maximum / norm) if norm > 1e-12 else vector


def cartesian_tracking_command(
    reference: np.ndarray,
    measured: np.ndarray,
    tangent: np.ndarray,
    spray_axis: np.ndarray,
    feedforward: np.ndarray,
    along_track_kp: float,
    orthogonal_kp: float,
    spray_kp: float,
    max_along: float,
    max_orthogonal: float,
    max_spray: float,
    max_linear: float,
    correction_scale: float = 1.0,
) -> CartesianTrackingResult:
    """Build mutually orthogonal Cartesian tracking corrections."""
    axis = normalize(spray_axis)
    plane_error = project_onto_plane(reference - measured, axis)
    planar_tangent = normalize(project_onto_plane(tangent, axis))
    along_error = float(np.dot(plane_error, planar_tangent)) * planar_tangent if np.any(planar_tangent) else np.zeros(3)
    lateral_error = plane_error - along_error
    spray_error = float(np.dot(reference - measured, axis))
    scale = max(0.0, float(correction_scale))
    along = limit_vector(along_error * max(0.0, along_track_kp), max_along) * scale
    lateral = limit_vector(lateral_error * max(0.0, orthogonal_kp), max_orthogonal) * scale
    spray = limit_vector(axis * spray_error * max(0.0, spray_kp), max_spray) * scale
    return CartesianTrackingResult(
        command=limit_vector(feedforward + along + lateral + spray, max_linear),
        along=along,
        lateral=lateral,
        spray=spray,
    )


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


def path_feedforward(
    segment_delta: np.ndarray,
    trajectory_speed: float,
    velocity_override: float,
) -> np.ndarray:
    """Return the 3D derivative of a linearly interpolated path reference."""
    return (
        normalize(segment_delta)
        * max(0.0, float(trajectory_speed))
        * max(0.0, float(velocity_override))
    )


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
