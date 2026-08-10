#!/usr/bin/env python3
"""Export legacy component print paths in the ROS 2 JSON trajectory format."""

from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path as FilePath
from typing import Any, Iterable

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from tf_transformations import quaternion_from_euler, quaternion_matrix, quaternion_multiply

from parse_paths.path_utils import as_bool, as_float_list


def _load_function(component_dir: FilePath, module_name: str, function_name: str):
    module_path = component_dir / 'print_path' / f'{module_name}.py'
    if not module_path.is_file():
        raise FileNotFoundError(f'Missing component module: {module_path}')
    unique_name = f'am_component_{component_dir.name}_{module_name}'
    spec = importlib.util.spec_from_file_location(unique_name, module_path)
    if spec is None or spec.loader is None:
        raise ImportError(f'Cannot load component module: {module_path}')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    function = getattr(module, function_name, None)
    if function is None or not callable(function):
        raise AttributeError(f'{module_path} does not define callable {function_name}()')
    return function


def _values(component_dir: FilePath, names: Iterable[str], suffix: str = '') -> dict[str, list[float]]:
    loaded: dict[str, list[float]] = {}
    for name in names:
        module_name = f'{name}{suffix}'
        function = _load_function(component_dir, module_name, module_name)
        values = [float(value) for value in function()]
        if not values:
            raise ValueError(f'Component series {module_name}() is empty')
        loaded[name] = values
    return loaded


def _time_message(seconds: float):
    if not math.isfinite(seconds) or seconds < 0.0:
        raise ValueError(f'Invalid trajectory timestamp: {seconds}')
    sec = int(math.floor(seconds))
    nanosec = int(round((seconds - sec) * 1e9))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    stamp = PoseStamped().header.stamp
    stamp.sec = sec
    stamp.nanosec = nanosec
    return stamp


def _orientation_from_yaw(yaw: float):
    q = quaternion_from_euler(0.0, 0.0, yaw)
    pose = PoseStamped()
    pose.pose.orientation.x = float(q[0])
    pose.pose.orientation.y = float(q[1])
    pose.pose.orientation.z = float(q[2])
    pose.pose.orientation.w = float(q[3])
    return pose.pose.orientation


def _component_root() -> FilePath:
    source_root = FilePath(__file__).resolve().parents[2] / 'components'
    if source_root.is_dir():
        return source_root
    return FilePath(get_package_share_directory('parse_paths')) / 'components'


def _transform_path(path: Path, xyz: Iterable[float], rpy: Iterable[float]) -> Path:
    translation = np.asarray(list(xyz), dtype=float)
    rotation = quaternion_from_euler(*list(rpy))
    matrix = quaternion_matrix(rotation)
    transformed = Path()
    transformed.header = path.header
    for original in path.poses:
        pose = PoseStamped()
        pose.header = original.header
        point = np.array([
            original.pose.position.x,
            original.pose.position.y,
            original.pose.position.z,
            1.0,
        ])
        result = matrix @ point
        pose.pose.position.x = float(result[0] + translation[0])
        pose.pose.position.y = float(result[1] + translation[1])
        pose.pose.position.z = float(result[2] + translation[2])
        current = [
            original.pose.orientation.x,
            original.pose.orientation.y,
            original.pose.orientation.z,
            original.pose.orientation.w,
        ]
        orientation = quaternion_multiply(rotation, current)
        pose.pose.orientation.x = float(orientation[0])
        pose.pose.orientation.y = float(orientation[1])
        pose.pose.orientation.z = float(orientation[2])
        pose.pose.orientation.w = float(orientation[3])
        transformed.poses.append(pose)
    transformed.header.frame_id = path.header.frame_id
    return transformed


def _make_path(frame_id: str, positions: list[np.ndarray], orientations: list[Any], timestamps: list[float]) -> Path:
    path = Path()
    path.header.frame_id = frame_id
    for position, orientation, timestamp in zip(positions, orientations, timestamps):
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = _time_message(timestamp)
        pose.pose.position.x = float(position[0])
        pose.pose.position.y = float(position[1])
        pose.pose.position.z = float(position[2])
        pose.pose.orientation = orientation
        path.poses.append(pose)
    return path


def _path_to_dict(path: Path) -> dict:
    return {
        'frame_id': path.header.frame_id,
        'poses': [
            {
                'frame_id': pose.header.frame_id,
                'stamp': {
                    'sec': int(pose.header.stamp.sec),
                    'nanosec': int(pose.header.stamp.nanosec),
                },
                'position': {
                    'x': float(pose.pose.position.x),
                    'y': float(pose.pose.position.y),
                    'z': float(pose.pose.position.z),
                },
                'orientation': {
                    'x': float(pose.pose.orientation.x),
                    'y': float(pose.pose.orientation.y),
                    'z': float(pose.pose.orientation.z),
                    'w': float(pose.pose.orientation.w),
                },
            }
            for pose in path.poses
        ],
    }


def _write_json(path: FilePath, data: dict) -> None:
    path.write_text(json.dumps(data, indent=2) + '\n', encoding='utf-8')


def export_component_paths(
    component_dir: FilePath,
    output_dir: FilePath,
    frame_id: str = 'map',
    start_index: int = 10,
    end_trim: int = 1,
    arm_suffix: str = '',
    base_suffix: str = '',
    base_z: float = 0.0,
    arm_transform_xyz: Iterable[float] = (0.0, 0.0, 0.0),
    arm_transform_rpy: Iterable[float] = (0.0, 0.0, 0.0),
    base_transform_xyz: Iterable[float] = (0.0, 0.0, 0.0),
    base_transform_rpy: Iterable[float] = (0.0, 0.0, 0.0),
    normal_vector: Iterable[float] = (0.0, 1.0, 0.0),
    arm_filename: str = 'arm_path.json',
    base_filename: str = 'base_path.json',
    normal_filename: str = 'normal_vector.json',
) -> tuple[FilePath, FilePath, FilePath]:
    """Convert one legacy component and write ROS 2-compatible JSON files."""

    arm_path, base_path = build_component_paths(
        component_dir,
        frame_id=frame_id,
        start_index=start_index,
        end_trim=end_trim,
        arm_suffix=arm_suffix,
        base_suffix=base_suffix,
        base_z=base_z,
    )
    arm_path = _transform_path(arm_path, arm_transform_xyz, arm_transform_rpy)
    base_path = _transform_path(base_path, base_transform_xyz, base_transform_rpy)

    output_dir.mkdir(parents=True, exist_ok=True)
    arm_file = output_dir / arm_filename
    base_file = output_dir / base_filename
    normal_file = output_dir / normal_filename
    _write_json(arm_file, _path_to_dict(arm_path))
    _write_json(base_file, _path_to_dict(base_path))
    normal = list(normal_vector)
    normal = (normal + [0.0, 0.0, 0.0])[:3]
    _write_json(normal_file, {
        'x': float(normal[0]),
        'y': float(normal[1]),
        'z': float(normal[2]),
    })
    return arm_file, base_file, normal_file


def build_component_paths(
    component_dir: FilePath,
    frame_id: str = 'map',
    start_index: int = 10,
    end_trim: int = 1,
    arm_suffix: str = '',
    base_suffix: str = '',
    base_z: float = 0.0,
) -> tuple[Path, Path]:
    """Convert one legacy component into synchronized arm and base paths."""

    arm = _values(component_dir, ('xTCP', 'yTCP', 'zTCP', 't'), arm_suffix)
    base = _values(
        component_dir,
        ('xMIR', 'yMIR', 'xVecMIRx', 'xVecMIRy', 't'),
        base_suffix,
    )
    lengths = {len(values) for values in (*arm.values(), *base.values())}
    if len(lengths) != 1:
        raise ValueError(f'Component series have different lengths: {sorted(lengths)}')
    total = next(iter(lengths))
    if any(abs(arm['t'][index] - base['t'][index]) > 1e-9 for index in range(total)):
        raise ValueError('Arm and base component timestamps do not match')
    first = max(0, int(start_index))
    last = total - max(0, int(end_trim))
    if last - first < 2:
        raise ValueError(f'Component path is too short after trimming: [{first}, {last})')

    indices = list(range(first, last))
    timestamps = [arm['t'][index] for index in indices]
    if any(current <= previous for previous, current in zip(timestamps, timestamps[1:])):
        raise ValueError('Component timestamps must be strictly increasing')

    arm_positions = [np.array([arm['xTCP'][i], arm['yTCP'][i], arm['zTCP'][i]]) for i in indices]
    base_positions = [np.array([base['xMIR'][i], base['yMIR'][i], float(base_z)]) for i in indices]
    arm_orientations = []
    base_orientations = []
    for index in indices:
        next_index = min(index + 1, total - 1)
        arm_yaw = math.atan2(
            arm['yTCP'][next_index] - arm['yTCP'][index],
            arm['xTCP'][next_index] - arm['xTCP'][index],
        )
        base_x = base['xVecMIRx'][index]
        base_y = base['xVecMIRy'][index]
        if abs(base_x) + abs(base_y) < 1e-9:
            base_yaw = math.atan2(
                base['yMIR'][next_index] - base['yMIR'][index],
                base['xMIR'][next_index] - base['xMIR'][index],
            )
        else:
            base_yaw = math.atan2(base_y, base_x)
        arm_orientations.append(_orientation_from_yaw(arm_yaw))
        base_orientations.append(_orientation_from_yaw(base_yaw))

    return (
        _make_path(frame_id, arm_positions, arm_orientations, timestamps),
        _make_path(frame_id, base_positions, base_orientations, timestamps),
    )


class ComponentPathExporter(Node):
    def __init__(self) -> None:
        super().__init__('component_path_exporter')
        self.declare_parameter('component_name', 'rectangleRoundedCorners')
        self.declare_parameter('component_root', '')
        self.declare_parameter('output_directory', '')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('start_index', 10)
        self.declare_parameter('end_trim', 1)
        self.declare_parameter('arm_suffix', '')
        self.declare_parameter('base_suffix', '')
        self.declare_parameter('base_z', 0.0)
        self.declare_parameter('arm_transform_xyz', [0.0, 0.0, 0.0])
        self.declare_parameter('arm_transform_rpy', [0.0, 0.0, 0.0])
        self.declare_parameter('base_transform_xyz', [0.0, 0.0, 0.0])
        self.declare_parameter('base_transform_rpy', [0.0, 0.0, 0.0])
        self.declare_parameter('normal_vector', [0.0, 1.0, 0.0])
        self.declare_parameter('arm_filename', 'arm_path.json')
        self.declare_parameter('base_filename', 'base_path.json')
        self.declare_parameter('normal_filename', 'normal_vector.json')

        root_value = str(self.get_parameter('component_root').value).strip()
        root = FilePath(root_value).expanduser() if root_value else _component_root()
        component = root / str(self.get_parameter('component_name').value)
        output_value = str(self.get_parameter('output_directory').value).strip()
        output = FilePath(output_value).expanduser() if output_value else component
        files = export_component_paths(
            component,
            output_dir=output,
            frame_id=str(self.get_parameter('frame_id').value),
            start_index=int(self.get_parameter('start_index').value),
            end_trim=int(self.get_parameter('end_trim').value),
            arm_suffix=str(self.get_parameter('arm_suffix').value),
            base_suffix=str(self.get_parameter('base_suffix').value),
            base_z=float(self.get_parameter('base_z').value),
            arm_transform_xyz=as_float_list(
                self.get_parameter('arm_transform_xyz').value, [0.0, 0.0, 0.0]
            ),
            arm_transform_rpy=as_float_list(
                self.get_parameter('arm_transform_rpy').value, [0.0, 0.0, 0.0]
            ),
            base_transform_xyz=as_float_list(
                self.get_parameter('base_transform_xyz').value, [0.0, 0.0, 0.0]
            ),
            base_transform_rpy=as_float_list(
                self.get_parameter('base_transform_rpy').value, [0.0, 0.0, 0.0]
            ),
            normal_vector=as_float_list(
                self.get_parameter('normal_vector').value, [0.0, 1.0, 0.0]
            ),
            arm_filename=str(self.get_parameter('arm_filename').value),
            base_filename=str(self.get_parameter('base_filename').value),
            normal_filename=str(self.get_parameter('normal_filename').value),
        )
        self.get_logger().info(
            f"Exported component '{component.name}' to {output} "
            f"({files[0].name}, {files[1].name}, {files[2].name})."
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ComponentPathExporter()
    try:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
