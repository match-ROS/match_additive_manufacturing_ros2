import math
from typing import Iterable, List

import numpy as np


def generate_line(start: Iterable[float], end: Iterable[float], num_points: int) -> List[np.ndarray]:
    start_arr = _as_point(start)
    end_arr = _as_point(end)
    return [
        start_arr + (end_arr - start_arr) * (i / max(num_points - 1, 1))
        for i in range(max(2, num_points))
    ]


def generate_rectangle(
    center: Iterable[float],
    width: float,
    height: float,
    num_points: int,
    closed: bool = True,
) -> List[np.ndarray]:
    center_arr = _as_point(center)
    half_w = abs(float(width)) * 0.5
    half_h = abs(float(height)) * 0.5
    corners = [
        center_arr + np.array([-half_w, -half_h, 0.0]),
        center_arr + np.array([half_w, -half_h, 0.0]),
        center_arr + np.array([half_w, half_h, 0.0]),
        center_arr + np.array([-half_w, half_h, 0.0]),
    ]
    if closed:
        corners.append(corners[0])
    return _sample_polyline(corners, max(2, num_points))


def generate_circle(
    center: Iterable[float],
    radius: float,
    num_points: int,
    closed: bool = True,
) -> List[np.ndarray]:
    center_arr = _as_point(center)
    count = max(3, num_points)
    radius = abs(float(radius))
    span = 2.0 * math.pi if closed else 2.0 * math.pi * (count - 1) / count
    return [
        center_arr + np.array([
            radius * math.cos(span * i / max(count - 1, 1)),
            radius * math.sin(span * i / max(count - 1, 1)),
            0.0,
        ])
        for i in range(count)
    ]


def generate_waypoints(
    flat_waypoints: Iterable[float],
    num_points: int,
    interpolate: bool = True,
) -> List[np.ndarray]:
    values = [float(value) for value in flat_waypoints]
    points = [_as_point(values[i:i + 3]) for i in range(0, len(values) - 2, 3)]
    if len(points) < 2:
        return generate_line([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], num_points)
    if not interpolate:
        return points
    return _sample_polyline(points, max(2, num_points))


def tangent_yaw(points: List[np.ndarray], index: int, fallback_yaw: float = 0.0) -> float:
    if not points:
        return fallback_yaw
    if index < len(points) - 1:
        delta = points[index + 1] - points[index]
    elif index > 0:
        delta = points[index] - points[index - 1]
    else:
        return fallback_yaw
    if abs(float(delta[0])) < 1e-9 and abs(float(delta[1])) < 1e-9:
        return fallback_yaw
    return math.atan2(float(delta[1]), float(delta[0]))


def _sample_polyline(points: List[np.ndarray], num_points: int) -> List[np.ndarray]:
    if len(points) < 2:
        return points

    lengths = [float(np.linalg.norm(points[i + 1] - points[i])) for i in range(len(points) - 1)]
    total_length = sum(lengths)
    if total_length < 1e-9:
        return [points[0].copy() for _ in range(num_points)]

    samples = []
    for sample_idx in range(num_points):
        target = total_length * sample_idx / max(num_points - 1, 1)
        distance_seen = 0.0
        for segment_idx, segment_length in enumerate(lengths):
            if target <= distance_seen + segment_length or segment_idx == len(lengths) - 1:
                ratio = 0.0 if segment_length < 1e-9 else (target - distance_seen) / segment_length
                samples.append(points[segment_idx] + (points[segment_idx + 1] - points[segment_idx]) * ratio)
                break
            distance_seen += segment_length
    return samples


def _as_point(values: Iterable[float]) -> np.ndarray:
    data = [float(value) for value in values]
    if len(data) < 3:
        data = data + [0.0] * (3 - len(data))
    return np.array(data[:3], dtype=float)
