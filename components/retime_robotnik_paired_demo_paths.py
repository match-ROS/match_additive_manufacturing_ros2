#!/usr/bin/env python3
"""Retiming utility for the exported Robotnik paired demo paths.

The paired demo encodes path speed in waypoint timestamps. This script rewrites
those timestamps so the segment speed matches a requested linear speed while
preserving the waypoint geometry and the initial timestamp.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any


DEFAULT_TARGET_SPEED = 0.1


def load_json(path: Path) -> dict[str, Any]:
    with path.open('r', encoding='utf-8') as handle:
        return json.load(handle)


def save_json(path: Path, data: dict[str, Any]) -> None:
    with path.open('w', encoding='utf-8') as handle:
        json.dump(data, handle, indent=2)
        handle.write('\n')


def stamp_to_seconds(stamp: dict[str, Any]) -> float:
    if not isinstance(stamp, dict):
        raise ValueError("Expected each pose to contain a 'stamp' object")
    if 'sec' not in stamp or 'nanosec' not in stamp:
        raise ValueError("Expected each stamp to contain 'sec' and 'nanosec'")
    return float(stamp['sec']) + float(stamp['nanosec']) / 1_000_000_000.0


def seconds_to_stamp(seconds: float) -> dict[str, int]:
    total_nanoseconds = int(round(float(seconds) * 1_000_000_000.0))
    sec, nanosec = divmod(total_nanoseconds, 1_000_000_000)
    return {'sec': int(sec), 'nanosec': int(nanosec)}


def pose_position(pose: dict[str, Any]) -> tuple[float, float, float]:
    position = pose.get('position')
    if not isinstance(position, dict):
        raise ValueError("Expected each pose to contain a 'position' object")
    try:
        return (
            float(position['x']),
            float(position['y']),
            float(position['z']),
        )
    except KeyError as exc:
        raise ValueError("Expected each pose position to contain x, y, and z") from exc


def retime_path(data: dict[str, Any], target_speed: float) -> dict[str, Any]:
    poses = data.get('poses', [])
    if not isinstance(poses, list):
        raise ValueError("Expected 'poses' to be a list in the path JSON")
    if not poses:
        raise ValueError('Expected at least one pose in the path JSON')
    if target_speed <= 0.0:
        raise ValueError('Target speed must be positive')

    current_time = stamp_to_seconds(poses[0]['stamp'])
    poses[0]['stamp'] = seconds_to_stamp(current_time)

    for previous_pose, pose in zip(poses, poses[1:]):
        previous_position = pose_position(previous_pose)
        position = pose_position(pose)
        segment_length = math.dist(previous_position, position)
        current_time += segment_length / target_speed
        pose['stamp'] = seconds_to_stamp(current_time)

    return data


def path_statistics(data: dict[str, Any]) -> tuple[float, float, float]:
    poses = data.get('poses', [])
    if not isinstance(poses, list) or len(poses) < 2:
        return 0.0, 0.0, 0.0

    distance = 0.0
    for previous_pose, pose in zip(poses, poses[1:]):
        distance += math.dist(pose_position(previous_pose), pose_position(pose))

    start_time = stamp_to_seconds(poses[0]['stamp'])
    end_time = stamp_to_seconds(poses[-1]['stamp'])
    duration = max(0.0, end_time - start_time)
    speed = distance / duration if duration > 0.0 else 0.0
    return distance, duration, speed


def parse_args() -> argparse.Namespace:
    default_directory = Path(__file__).resolve().parent
    default_paths = [
        default_directory / 'base_path.json',
        default_directory / 'arm_path.json',
    ]

    parser = argparse.ArgumentParser(
        description='Retime the exported Robotnik paired demo paths to a target speed.'
    )
    parser.add_argument(
        'paths',
        nargs='*',
        type=Path,
        default=default_paths,
        help=(
            'Path JSON files to retime in place. Defaults to the exported '
            'base_path.json and arm_path.json files in this directory.'
        ),
    )
    parser.add_argument(
        '--speed',
        type=float,
        default=DEFAULT_TARGET_SPEED,
        help=f'Target linear speed in m/s (default: {DEFAULT_TARGET_SPEED}).',
    )
    parser.add_argument(
        '--output-dir',
        type=Path,
        default=None,
        help='Optional directory for rewritten files. Defaults to in-place update.',
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    for input_path in args.paths:
        output_path = (
            args.output_dir / input_path.name if args.output_dir is not None else input_path
        )
        data = load_json(input_path)
        original_distance, original_duration, original_speed = path_statistics(data)
        retimed = retime_path(data, args.speed)
        save_json(output_path, retimed)
        new_distance, new_duration, new_speed = path_statistics(retimed)
        print(
            f'{input_path.name}: '
            f'{original_distance:.4f} m in {original_duration:.4f} s '
            f'({original_speed:.4f} m/s) -> '
            f'{new_distance:.4f} m in {new_duration:.4f} s '
            f'({new_speed:.4f} m/s)'
        )


if __name__ == '__main__':
    main()
