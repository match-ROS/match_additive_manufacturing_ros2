#!/usr/bin/env python3
"""
Helper for measuring a local coordinate system from Vicon poses.

Parameters:
  input_topic: PoseStamped topic to listen to.
    Default: /vicon/Pointer_passive/Pointer_passive
  record_service: Trigger service used to capture the latest received pose.
    Default: ~/record_pose
  offset_vector: Local-frame xyz offset from the measured origin.
    Default: [-0.5, 0.0, 0.0], which is 0.5 m in the calculated -X direction.
  map_frame: Expected frame_id of the captured poses.
    Default: map

Capture order:
  1. Point on the positive X axis.
  2. Origin, where the X and Y axes intersect.
  3. Point on the negative Y axis.

Example:
  # ROS recording mode, capture order: +X point, origin, -Y point.
  ./determine_base_start_relative.py --ros-args -p offset_vector:="[-0.5, 0.0, 0.0]"
  ros2 service call /determine_base_start_relative/record_pose std_srvs/srv/Trigger {}

  # CLI two-position mode: origin, -Y point.
  ./determine_base_start_relative.py --origin -1.959392 2.613900 -0.538461 \
    --negative-y -1.958683 2.615110 -0.536972

  ./determine_base_start_relative.py -1.959392 2.613900 -0.538461 \
    -1.958683 2.615110 -0.536972

  # CLI three-position mode: +X point, origin, -Y point.
  ./determine_base_start_relative.py --x-point 0.0 0.0 0.0 \
    --origin -1.959392 2.613900 -0.538461 \
    --negative-y -1.958683 2.615110 -0.536972
"""
from __future__ import annotations

import argparse
from copy import deepcopy
import math
import sys
from typing import Optional

from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import Trigger


MAP_UP = np.array([0.0, 0.0, 1.0])


def _normalize(vector: np.ndarray, name: str) -> np.ndarray:
    norm = np.linalg.norm(vector)
    if norm < 1e-9:
        raise ValueError(f"{name} vector is too small; check that the measured poses are distinct")
    return vector / norm


def _normalize_horizontal(vector: np.ndarray, name: str) -> np.ndarray:
    horizontal = np.array([vector[0], vector[1], 0.0], dtype=float)
    return _normalize(horizontal, f"{name} horizontal")


def _yaw_only_frame_from_points(
    origin: np.ndarray,
    negative_y_point: np.ndarray,
    x_point: Optional[np.ndarray] = None,
) -> tuple[np.ndarray, dict[str, float | np.ndarray | None]]:
    measured_y_vector = origin - negative_y_point
    measured_y_horizontal = _normalize_horizontal(measured_y_vector, "y")

    y_yaw = math.atan2(measured_y_horizontal[1], measured_y_horizontal[0]) - (math.pi / 2.0)
    yaw = y_yaw
    x_yaw = None
    x_vector = None
    x_axis_horizontal = None

    if x_point is not None:
        x_vector = x_point - origin
        x_axis_horizontal = _normalize_horizontal(x_vector, "x")
        x_yaw = math.atan2(x_axis_horizontal[1], x_axis_horizontal[0])
        yaw = x_yaw

    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    x_axis = np.array([cos_yaw, sin_yaw, 0.0], dtype=float)
    measured_y = np.array([-sin_yaw, cos_yaw, 0.0], dtype=float)
    z_axis = MAP_UP.copy()
    rotation = np.column_stack((x_axis, measured_y, z_axis))

    raw_z_axis = None
    if x_vector is not None:
        raw_z_axis = _normalize(
            np.cross(_normalize(x_vector, "x"), _normalize(measured_y_vector, "y")),
            "z",
        )

    z_dot_up = float(np.clip(np.dot(z_axis, MAP_UP), -1.0, 1.0))
    z_angle_degrees = math.degrees(math.acos(z_dot_up))
    z_difference = z_axis - MAP_UP
    xy_angle_difference = None
    raw_xy_dot = None
    x_horizontal_distance = None
    x_vertical_delta = None
    if x_axis_horizontal is not None and x_vector is not None:
        raw_xy_dot = float(np.dot(x_axis_horizontal, measured_y_horizontal))
        xy_angle_difference = math.degrees(
            math.acos(float(np.clip(raw_xy_dot, -1.0, 1.0)))
        )
        x_horizontal_distance = float(
            np.linalg.norm(np.array([x_vector[0], x_vector[1]], dtype=float))
        )
        x_vertical_delta = float(x_vector[2])

    y_horizontal_distance = float(
        np.linalg.norm(np.array([measured_y_vector[0], measured_y_vector[1]], dtype=float))
    )
    y_vertical_delta = float(measured_y_vector[2])

    diagnostics = {
        "raw_xy_dot": raw_xy_dot,
        "raw_z_axis": raw_z_axis,
        "z_axis": z_axis,
        "z_difference": z_difference,
        "z_angle_degrees": z_angle_degrees,
        "determinant": float(np.linalg.det(rotation)),
        "yaw_degrees": math.degrees(yaw),
        "x_yaw_degrees": None if x_yaw is None else math.degrees(x_yaw),
        "y_yaw_degrees": math.degrees(y_yaw),
        "xy_angle_difference": xy_angle_difference,
        "x_horizontal_distance": x_horizontal_distance,
        "y_horizontal_distance": y_horizontal_distance,
        "x_vertical_delta": x_vertical_delta,
        "y_vertical_delta": y_vertical_delta,
    }
    return rotation, diagnostics


def _quaternion_from_matrix(rotation: np.ndarray) -> np.ndarray:
    trace = float(np.trace(rotation))

    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (rotation[2, 1] - rotation[1, 2]) / s
        qy = (rotation[0, 2] - rotation[2, 0]) / s
        qz = (rotation[1, 0] - rotation[0, 1]) / s
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        s = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
        qw = (rotation[2, 1] - rotation[1, 2]) / s
        qx = 0.25 * s
        qy = (rotation[0, 1] + rotation[1, 0]) / s
        qz = (rotation[0, 2] + rotation[2, 0]) / s
    elif rotation[1, 1] > rotation[2, 2]:
        s = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
        qw = (rotation[0, 2] - rotation[2, 0]) / s
        qx = (rotation[0, 1] + rotation[1, 0]) / s
        qy = 0.25 * s
        qz = (rotation[1, 2] + rotation[2, 1]) / s
    else:
        s = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
        qw = (rotation[1, 0] - rotation[0, 1]) / s
        qx = (rotation[0, 2] + rotation[2, 0]) / s
        qy = (rotation[1, 2] + rotation[2, 1]) / s
        qz = 0.25 * s

    quaternion = np.array([qx, qy, qz, qw], dtype=float)
    return quaternion / np.linalg.norm(quaternion)


def _format_vector(vector: np.ndarray) -> str:
    return "[" + ", ".join(f"{value:.6f}" for value in vector) + "]"


def _format_optional_float(value: Optional[float], unit: str = "") -> str:
    if value is None:
        return "n/a"
    return f"{value:.6f}{unit}"


def _position_from_pose(pose: PoseStamped) -> np.ndarray:
    position = pose.pose.position
    return np.array([position.x, position.y, position.z], dtype=float)


def _offset_vector_from_parameter(value) -> np.ndarray:
    vector = np.array(value, dtype=float)
    if vector.shape != (3,):
        raise ValueError(f"offset_vector must contain exactly 3 values, got {vector.tolist()}")
    return vector


def _as_vector3(value: list[float], name: str) -> np.ndarray:
    vector = np.array(value, dtype=float)
    if vector.shape != (3,):
        raise ValueError(f"{name} must contain exactly 3 values, got {vector.tolist()}")
    return vector


def _calculate_start_pose(
    origin: np.ndarray,
    negative_y_point: np.ndarray,
    offset_vector: np.ndarray,
    x_point: Optional[np.ndarray] = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, dict[str, float | np.ndarray | None]]:
    rotation, diagnostics = _yaw_only_frame_from_points(
        origin,
        negative_y_point,
        x_point,
    )
    start_position = origin + rotation @ offset_vector
    start_quaternion = _quaternion_from_matrix(rotation)
    return rotation, start_position, start_quaternion, diagnostics


def _calculation_lines(
    origin: np.ndarray,
    negative_y_point: np.ndarray,
    offset_vector: np.ndarray,
    rotation: np.ndarray,
    start_position: np.ndarray,
    start_quaternion: np.ndarray,
    diagnostics: dict[str, float | np.ndarray | None],
    x_point: Optional[np.ndarray] = None,
) -> list[str]:
    lines = ["Input points in map frame:"]
    if x_point is not None:
        lines.append(f"  +X point: {_format_vector(x_point)}")
    lines.extend(
        [
            f"  origin: {_format_vector(origin)}",
            f"  -Y point: {_format_vector(negative_y_point)}",
            "Yaw-only coordinate system in map frame (columns are x, y, z):",
            np.array2string(rotation, precision=6, suppress_small=True),
            "Horizontal measurement diagnostics:",
            "  horizontal distances: "
            f"+X={_format_optional_float(diagnostics['x_horizontal_distance'], ' m')}, "
            f"+Y-from--Y={diagnostics['y_horizontal_distance']:.6f} m",
            "  vertical deltas before projection: "
            f"+X={_format_optional_float(diagnostics['x_vertical_delta'], ' m')}, "
            f"+Y-from--Y={diagnostics['y_vertical_delta']:.6f} m",
            "  yaw estimates: "
            f"from +X={_format_optional_float(diagnostics['x_yaw_degrees'], ' deg')}, "
            f"from -Y={diagnostics['y_yaw_degrees']:.6f} deg, "
            f"used={diagnostics['yaw_degrees']:.6f} deg",
            "  measured horizontal X/Y angle: "
            f"{_format_optional_float(diagnostics['xy_angle_difference'], ' deg')}",
            "Z-axis check against map +Z:",
        ]
    )
    if diagnostics["raw_z_axis"] is not None:
        lines.append(
            "  measured 3D cross-product z-axis: "
            f"{_format_vector(diagnostics['raw_z_axis'])}"
        )
    else:
        lines.append("  measured 3D cross-product z-axis: n/a")
    lines.extend(
        [
            f"  final z-axis: {_format_vector(diagnostics['z_axis'])}",
            f"  final z-axis - map +Z: {_format_vector(diagnostics['z_difference'])}",
            f"  angular difference: {diagnostics['z_angle_degrees']:.6f} deg",
            f"  measured horizontal x dot y: {_format_optional_float(diagnostics['raw_xy_dot'])}",
            f"  rotation determinant: {diagnostics['determinant']:.9f}",
            f"Local offset vector xyz: {_format_vector(offset_vector)}",
            "Pose at origin + rotation * offset_vector:",
            f"  position xyz: {_format_vector(start_position)}",
            f"  orientation xyzw: {_format_vector(start_quaternion)}",
            "Result pose:",
            f"  position xyz: {_format_vector(start_position)}",
            f"  orientation quaternion xyzw: {_format_vector(start_quaternion)}",
        ]
    )
    return lines


class RelativeBaseStartRecorder(Node):

    def __init__(self) -> None:
        super().__init__("determine_base_start_relative")
        self.declare_parameter("input_topic", "/vicon/Pointer_passive/Pointer_passive")
        self.declare_parameter("record_service", "~/record_pose")
        self.declare_parameter("offset_vector", [-0.5, 0.0, 0.0])
        self.declare_parameter("map_frame", "map")

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.map_frame = str(self.get_parameter("map_frame").value).strip().lstrip("/")
        self.latest_pose: Optional[PoseStamped] = None
        self.recorded_poses: list[PoseStamped] = []

        self.create_subscription(PoseStamped, self.input_topic, self._pose_cb, 10)
        self.create_service(
            Trigger,
            str(self.get_parameter("record_service").value),
            self._record_pose_cb,
        )

        self.get_logger().info(
            "Listening for PoseStamped messages on "
            f"{self.input_topic}. Call ~/record_pose three times: "
            "1) +X point, 2) origin, 3) -Y point."
        )

    def _pose_cb(self, msg: PoseStamped) -> None:
        self.latest_pose = msg

    def _record_pose_cb(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        if self.latest_pose is None:
            response.success = False
            response.message = f"No PoseStamped received yet on {self.input_topic}."
            return response

        pose = deepcopy(self.latest_pose)
        self.recorded_poses.append(pose)
        record_count = len(self.recorded_poses)
        self.get_logger().info(
            f"Recorded pose {record_count}/3 at {_format_vector(_position_from_pose(pose))} "
            f"in frame '{pose.header.frame_id}'."
        )

        if record_count < 3:
            response.success = True
            response.message = (
                f"Recorded pose {record_count}/3. "
                "Next order: 1) +X point, 2) origin, 3) -Y point."
            )
            return response

        try:
            result_message = self._calculate_from_recorded_poses()
        except ValueError as exc:
            self.recorded_poses.clear()
            response.success = False
            response.message = f"Failed to calculate coordinate system: {exc}"
            return response

        self.recorded_poses.clear()
        response.success = True
        response.message = result_message
        return response

    def _calculate_from_recorded_poses(self) -> str:
        x_pose, origin_pose, negative_y_pose = self.recorded_poses
        frame_ids = {pose.header.frame_id.strip().lstrip("/") for pose in self.recorded_poses}
        if len(frame_ids) > 1:
            self.get_logger().warn(
                "Recorded poses have different frame_ids. Calculating from raw coordinates anyway: "
                f"{sorted(frame_ids)}"
            )
        elif self.map_frame and self.map_frame not in frame_ids:
            self.get_logger().warn(
                f"Recorded pose frame is {next(iter(frame_ids), '')!r}, expected map frame "
                f"{self.map_frame!r}."
            )

        x_point = _position_from_pose(x_pose)
        origin = _position_from_pose(origin_pose)
        negative_y_point = _position_from_pose(negative_y_pose)
        offset_vector = _offset_vector_from_parameter(
            self.get_parameter("offset_vector").value
        )

        rotation, start_position, start_quaternion, diagnostics = _calculate_start_pose(
            origin,
            negative_y_point,
            offset_vector,
            x_point,
        )

        for line in _calculation_lines(
            origin,
            negative_y_point,
            offset_vector,
            rotation,
            start_position,
            start_quaternion,
            diagnostics,
            x_point,
        ):
            self.get_logger().info(line)

        return (
            "Calculated start pose. "
            f"position xyz={_format_vector(start_position)}, "
            f"orientation xyzw={_format_vector(start_quaternion)}"
        )


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Calculate an upright map-frame start pose from measured positions, or run "
            "as a ROS recorder when no CLI positions are provided."
        )
    )
    parser.add_argument(
        "--origin",
        nargs=3,
        type=float,
        metavar=("X", "Y", "Z"),
        help="Origin position in map frame.",
    )
    parser.add_argument(
        "--negative-y",
        nargs=3,
        type=float,
        metavar=("X", "Y", "Z"),
        help="Position on the negative Y axis in map frame.",
    )
    parser.add_argument(
        "--x-point",
        nargs=3,
        type=float,
        metavar=("X", "Y", "Z"),
        help="Optional position on the positive X axis in map frame.",
    )
    parser.add_argument(
        "--offset-vector",
        nargs=3,
        type=float,
        metavar=("X", "Y", "Z"),
        default=[-0.5, 0.0, 0.0],
        help="Local-frame xyz offset from the measured origin. Default: -0.5 0.0 0.0.",
    )
    parser.add_argument(
        "positions",
        nargs="*",
        type=float,
        help=(
            "Optional compact form: 6 values = origin xyz, -Y xyz; "
            "9 values = +X xyz, origin xyz, -Y xyz."
        ),
    )
    return parser


def _looks_like_float(value: str) -> bool:
    try:
        float(value)
    except ValueError:
        return False
    return True


def _has_cli_positions(args: list[str]) -> bool:
    cli_flags = {"-h", "--help", "--origin", "--negative-y", "--x-point", "--offset-vector"}
    if any(arg in cli_flags for arg in args):
        return True
    return bool(args) and all(_looks_like_float(arg) for arg in args)


def _run_cli(args: list[str]) -> int:
    parser = _build_arg_parser()
    parsed = parser.parse_args(args)

    if parsed.positions:
        if parsed.origin is not None or parsed.negative_y is not None or parsed.x_point is not None:
            parser.error("use either named point options or compact positional values, not both")
        if len(parsed.positions) == 6:
            parsed.origin = parsed.positions[0:3]
            parsed.negative_y = parsed.positions[3:6]
        elif len(parsed.positions) == 9:
            parsed.x_point = parsed.positions[0:3]
            parsed.origin = parsed.positions[3:6]
            parsed.negative_y = parsed.positions[6:9]
        else:
            parser.error("compact positional form expects exactly 6 or 9 numeric values")

    if parsed.origin is None or parsed.negative_y is None:
        parser.error("--origin and --negative-y are required for CLI calculation")

    origin = _as_vector3(parsed.origin, "origin")
    negative_y_point = _as_vector3(parsed.negative_y, "negative_y")
    x_point = None if parsed.x_point is None else _as_vector3(parsed.x_point, "x_point")
    offset_vector = _as_vector3(parsed.offset_vector, "offset_vector")

    try:
        rotation, start_position, start_quaternion, diagnostics = _calculate_start_pose(
            origin,
            negative_y_point,
            offset_vector,
            x_point,
        )
    except ValueError as exc:
        print(f"Failed to calculate coordinate system: {exc}", file=sys.stderr)
        return 1

    for line in _calculation_lines(
        origin,
        negative_y_point,
        offset_vector,
        rotation,
        start_position,
        start_quaternion,
        diagnostics,
        x_point,
    ):
        print(line)
    return 0


def main(args=None) -> None:
    cli_args = list(sys.argv[1:] if args is None else args)
    if _has_cli_positions(cli_args):
        raise SystemExit(_run_cli(cli_args))

    rclpy.init(args=args)
    node = RelativeBaseStartRecorder()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
