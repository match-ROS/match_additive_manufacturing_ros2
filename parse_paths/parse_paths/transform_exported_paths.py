#!/usr/bin/env python3
import argparse
import json
from pathlib import Path

from parse_paths.path_transform import transform_path, transform_vector
from parse_paths.publish_robotnik_base_arm_paths import (
    path_from_dict,
    path_to_dict,
    vector3_from_dict,
    vector3_to_dict,
)


def _parse_xyz(value: str) -> list[float]:
    parts = [part.strip() for part in value.split(',')]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError('expected x,y,z')
    try:
        return [float(part) for part in parts]
    except ValueError as exc:
        raise argparse.ArgumentTypeError('x,y,z must be numeric') from exc


def _read_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding='utf-8'))


def _write_json(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2) + '\n', encoding='utf-8')


def main() -> None:
    parser = argparse.ArgumentParser(
        description='Transform exported base/arm path JSON files by x,y,z and yaw degrees.',
    )
    parser.add_argument('input_dir', type=Path)
    parser.add_argument('output_dir', type=Path)
    parser.add_argument('--xyz', type=_parse_xyz, default=[0.0, 0.0, 0.0])
    parser.add_argument('--yaw-deg', type=float, default=0.0)
    parser.add_argument('--base-file', default='base_path.json')
    parser.add_argument('--arm-file', default='arm_path.json')
    parser.add_argument('--normal-file', default='normal_vector.json')
    args = parser.parse_args()

    base_path = path_from_dict(_read_json(args.input_dir / args.base_file), 'robotnik_simple')
    arm_path = path_from_dict(_read_json(args.input_dir / args.arm_file), base_path.header.frame_id)
    _write_json(
        args.output_dir / args.base_file,
        path_to_dict(transform_path(base_path, args.xyz, args.yaw_deg)),
    )
    _write_json(
        args.output_dir / args.arm_file,
        path_to_dict(transform_path(arm_path, args.xyz, args.yaw_deg)),
    )

    normal_path = args.input_dir / args.normal_file
    if normal_path.exists():
        normal = vector3_from_dict(_read_json(normal_path))
        _write_json(
            args.output_dir / args.normal_file,
            vector3_to_dict(transform_vector(normal, args.yaw_deg)),
        )


if __name__ == '__main__':
    main()
