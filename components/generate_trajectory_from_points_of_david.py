#!/usr/bin/env python3
"""Generate paired arm/base trajectory JSON files from David's exported frames."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any


TARGET_SPEED_MPS = 0.10
START_TIME_SECONDS = 0.0
FRAME_ID = 'robotnik_simple'
NORMAL_AXIS_FIELD = 'z_axis'


def load_frames(path: Path) -> list[dict[str, Any]]:
    with path.open('r', encoding='utf-8') as handle:
        data = json.load(handle)
    if not isinstance(data, list) or not data:
        raise ValueError(f'{path} must contain a non-empty list of frames')
    return data


def save_json(path: Path, data: dict[str, Any]) -> None:
    with path.open('w', encoding='utf-8') as handle:
        json.dump(data, handle, indent=2)
        handle.write('\n')


def save_vector_json(path: Path, vector: tuple[float, float, float]) -> None:
    save_json(path, {'x': vector[0], 'y': vector[1], 'z': vector[2]})


def vector3(frame: dict[str, Any], key: str) -> tuple[float, float, float]:
    value = frame.get(key)
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f"Expected frame field '{key}' to contain three numbers")
    return (float(value[0]), float(value[1]), float(value[2]))


def normalize(vector: tuple[float, float, float]) -> tuple[float, float, float]:
    norm = math.sqrt(sum(component * component for component in vector))
    if norm < 1e-9:
        raise ValueError('Cannot normalize a zero-length axis')
    return tuple(component / norm for component in vector)


def dot(left: tuple[float, float, float], right: tuple[float, float, float]) -> float:
    return sum(a * b for a, b in zip(left, right))


def subtract_projection(
    vector: tuple[float, float, float],
    onto: tuple[float, float, float],
) -> tuple[float, float, float]:
    scale = dot(vector, onto)
    return tuple(component - scale * axis_component for component, axis_component in zip(vector, onto))


def cross(
    left: tuple[float, float, float],
    right: tuple[float, float, float],
) -> tuple[float, float, float]:
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def orthonormal_axes(frame: dict[str, Any]) -> tuple[
    tuple[float, float, float],
    tuple[float, float, float],
    tuple[float, float, float],
]:
    x_axis = normalize(vector3(frame, 'x_axis'))
    y_axis = normalize(subtract_projection(vector3(frame, 'y_axis'), x_axis))
    z_axis = normalize(cross(x_axis, y_axis))

    source_z_axis = normalize(vector3(frame, 'z_axis'))
    if dot(z_axis, source_z_axis) < 0.0:
        y_axis = tuple(-component for component in y_axis)
        z_axis = tuple(-component for component in z_axis)

    return x_axis, y_axis, z_axis


def quaternion_from_axes(
    x_axis: tuple[float, float, float],
    y_axis: tuple[float, float, float],
    z_axis: tuple[float, float, float],
) -> dict[str, float]:
    m00, m01, m02 = x_axis[0], y_axis[0], z_axis[0]
    m10, m11, m12 = x_axis[1], y_axis[1], z_axis[1]
    m20, m21, m22 = x_axis[2], y_axis[2], z_axis[2]
    trace = m00 + m11 + m22

    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (m21 - m12) / scale
        y = (m02 - m20) / scale
        z = (m10 - m01) / scale
    elif m00 > m11 and m00 > m22:
        scale = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        w = (m21 - m12) / scale
        x = 0.25 * scale
        y = (m01 + m10) / scale
        z = (m02 + m20) / scale
    elif m11 > m22:
        scale = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        w = (m02 - m20) / scale
        x = (m01 + m10) / scale
        y = 0.25 * scale
        z = (m12 + m21) / scale
    else:
        scale = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        w = (m10 - m01) / scale
        x = (m02 + m20) / scale
        y = (m12 + m21) / scale
        z = 0.25 * scale

    norm = math.sqrt(x * x + y * y + z * z + w * w)
    return {'x': x / norm, 'y': y / norm, 'z': z / norm, 'w': w / norm}


def seconds_to_stamp(seconds: float) -> dict[str, int]:
    total_nanoseconds = int(round(seconds * 1_000_000_000.0))
    sec, nanosec = divmod(total_nanoseconds, 1_000_000_000)
    return {'sec': int(sec), 'nanosec': int(nanosec)}


def target_timestamps(
    target_frames: list[dict[str, Any]],
    target_speed: float,
) -> list[dict[str, int]]:
    if target_speed <= 0.0:
        raise ValueError('Target speed must be positive')

    timestamps = [seconds_to_stamp(START_TIME_SECONDS)]
    current_time = START_TIME_SECONDS
    for previous_frame, frame in zip(target_frames, target_frames[1:]):
        segment_length = math.dist(vector3(previous_frame, 'origin'), vector3(frame, 'origin'))
        current_time += segment_length / target_speed
        timestamps.append(seconds_to_stamp(current_time))
    return timestamps


def frames_to_path(
    frames: list[dict[str, Any]],
    timestamps: list[dict[str, int]],
    frame_id: str,
) -> dict[str, Any]:
    if len(frames) != len(timestamps):
        raise ValueError('Frame and timestamp counts must match')

    poses = []
    for frame, stamp in zip(frames, timestamps):
        x_axis, y_axis, z_axis = orthonormal_axes(frame)
        origin = vector3(frame, 'origin')
        poses.append(
            {
                'frame_id': frame_id,
                'stamp': stamp,
                'position': {'x': origin[0], 'y': origin[1], 'z': origin[2]},
                'orientation': quaternion_from_axes(x_axis, y_axis, z_axis),
            }
        )

    return {'frame_id': frame_id, 'poses': poses}


def path_stats(frames: list[dict[str, Any]], timestamps: list[dict[str, int]]) -> tuple[float, float, float]:
    distance = sum(
        math.dist(vector3(previous_frame, 'origin'), vector3(frame, 'origin'))
        for previous_frame, frame in zip(frames, frames[1:])
    )
    start_time = timestamps[0]['sec'] + timestamps[0]['nanosec'] / 1_000_000_000.0
    end_time = timestamps[-1]['sec'] + timestamps[-1]['nanosec'] / 1_000_000_000.0
    duration = end_time - start_time
    speed = distance / duration if duration > 0.0 else 0.0
    return distance, duration, speed


def parse_args() -> argparse.Namespace:
    script_dir = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(
        description='Generate arm_path.json, base_path.json, and normal_vector.json from David path frames.'
    )
    parser.add_argument('--targets', type=Path, default=script_dir / '260520_targets.json')
    parser.add_argument('--base-positions', type=Path, default=script_dir / '260520_base_position.json')
    parser.add_argument('--output-dir', type=Path, default=script_dir)
    parser.add_argument('--target-speed', type=float, default=TARGET_SPEED_MPS)
    parser.add_argument('--frame-id', default=FRAME_ID)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    target_frames = load_frames(args.targets)
    base_frames = load_frames(args.base_positions)

    if len(target_frames) != len(base_frames):
        raise ValueError(
            f'Target and base frame counts must match: {len(target_frames)} != {len(base_frames)}'
        )

    timestamps = target_timestamps(target_frames, args.target_speed)
    arm_path = frames_to_path(target_frames, timestamps, args.frame_id)
    base_path = frames_to_path(base_frames, timestamps, args.frame_id)
    normal_vector = normalize(vector3(target_frames[0], NORMAL_AXIS_FIELD))

    args.output_dir.mkdir(parents=True, exist_ok=True)
    save_json(args.output_dir / 'arm_path.json', arm_path)
    save_json(args.output_dir / 'base_path.json', base_path)
    save_vector_json(args.output_dir / 'normal_vector.json', normal_vector)

    distance, duration, speed = path_stats(target_frames, timestamps)
    print(
        f'Generated {len(target_frames)} paired poses at {speed:.4f} m/s '
        f'({distance:.4f} m in {duration:.4f} s).'
    )


if __name__ == '__main__':
    main()
