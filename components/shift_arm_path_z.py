#!/usr/bin/env python3
"""Shift the exported Robotnik arm path down in Z by a fixed amount."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


DEFAULT_SHIFT = 0.4


def load_json(path: Path) -> dict[str, Any]:
    with path.open('r', encoding='utf-8') as handle:
        return json.load(handle)


def save_json(path: Path, data: dict[str, Any]) -> None:
    with path.open('w', encoding='utf-8') as handle:
        json.dump(data, handle, indent=2)
        handle.write('\n')


def shift_arm_path_z(data: dict[str, Any], delta_z: float) -> dict[str, Any]:
    poses = data.get('poses', [])
    if not isinstance(poses, list):
        raise ValueError("Expected 'poses' to be a list in the arm path JSON")

    for pose in poses:
        if not isinstance(pose, dict):
            raise ValueError('Expected each pose entry to be a JSON object')
        position = pose.get('position')
        if not isinstance(position, dict):
            raise ValueError("Expected each pose to contain a 'position' object")
        if 'z' not in position:
            raise ValueError("Expected each pose position to contain a 'z' value")
        position['z'] = float(position['z']) - float(delta_z)
    return data


def parse_args() -> argparse.Namespace:
    default_input = Path(__file__).resolve().with_name('arm_path.json')
    parser = argparse.ArgumentParser(
        description='Subtract a constant offset from the Z position of every arm waypoint.'
    )
    parser.add_argument(
        '--input',
        type=Path,
        default=default_input,
        help=f'Input arm path JSON (default: {default_input})',
    )
    parser.add_argument(
        '--output',
        type=Path,
        default=None,
        help='Output file. Defaults to in-place update of the input file.',
    )
    parser.add_argument(
        '--delta-z',
        type=float,
        default=DEFAULT_SHIFT,
        help='Positive amount to subtract from each waypoint z value.',
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    input_path = args.input
    output_path = args.output or input_path

    data = load_json(input_path)
    shifted = shift_arm_path_z(data, args.delta_z)
    save_json(output_path, shifted)


if __name__ == '__main__':
    main()
