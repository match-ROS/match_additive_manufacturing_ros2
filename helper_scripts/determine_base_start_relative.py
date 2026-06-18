#!/usr/bin/env python3
"""
ROS 2 helper for measuring a local coordinate system from Vicon poses.

Parameters:
  input_topic: PoseStamped topic to listen to.
    Default: /vicon/LEDTaster/LEDTaster
  record_service: Trigger service used to capture the latest received pose.
    Default: ~/record_pose
  offset_vector: Local-frame xyz offset from the measured origin.
    Default: [-0.5, 0.0, 0.0], which is 0.5 m in the calculated -X direction.
  map_frame: Expected frame_id of the captured poses.
    Default: map

Capture order:
  1. Point on the positive X axis.
  2. Origin, where the X and Y axes intersect.
  3. Point on the positive Y axis.

Example:
  ./determine_base_start_relative.py --ros-args -p offset_vector:="[-0.5, 0.0, 0.0]"
  ros2 service call /determine_base_start_relative/record_pose std_srvs/srv/Trigger {}
"""
from __future__ import annotations

from copy import deepcopy
import math
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


def _orthonormal_frame_from_points(
    origin: np.ndarray,
    x_point: np.ndarray,
    y_point: np.ndarray,
) -> tuple[np.ndarray, dict[str, float | np.ndarray | bool]]:
    x_axis = _normalize(x_point - origin, "x")
    measured_y = _normalize(y_point - origin, "y")
    measured_z = _normalize(np.cross(x_axis, measured_y), "z")
    flipped_y_for_upward_z = False

    if float(np.dot(measured_z, MAP_UP)) < 0.0:
        measured_y = -measured_y
        measured_z = -measured_z
        flipped_y_for_upward_z = True

    raw_frame = np.column_stack((x_axis, measured_y, measured_z))
    u, _, vt = np.linalg.svd(raw_frame)
    rotation = u @ vt

    if np.linalg.det(rotation) < 0.0:
        u[:, -1] *= -1.0
        rotation = u @ vt

    if float(np.dot(rotation[:, 2], MAP_UP)) < 0.0:
        rotation[:, 1] *= -1.0
        rotation[:, 2] *= -1.0
        flipped_y_for_upward_z = True

    z_axis = rotation[:, 2]
    z_dot_up = float(np.clip(np.dot(z_axis, MAP_UP), -1.0, 1.0))
    z_angle_degrees = math.degrees(math.acos(z_dot_up))
    z_difference = z_axis - MAP_UP

    diagnostics = {
        "flipped_y_for_upward_z": flipped_y_for_upward_z,
        "raw_xy_dot": float(np.dot(x_axis, measured_y)),
        "raw_z_axis": measured_z,
        "z_axis": z_axis,
        "z_difference": z_difference,
        "z_angle_degrees": z_angle_degrees,
        "determinant": float(np.linalg.det(rotation)),
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


def _position_from_pose(pose: PoseStamped) -> np.ndarray:
    position = pose.pose.position
    return np.array([position.x, position.y, position.z], dtype=float)


def _offset_vector_from_parameter(value) -> np.ndarray:
    vector = np.array(value, dtype=float)
    if vector.shape != (3,):
        raise ValueError(f"offset_vector must contain exactly 3 values, got {vector.tolist()}")
    return vector


class RelativeBaseStartRecorder(Node):

    def __init__(self) -> None:
        super().__init__("determine_base_start_relative")
        self.declare_parameter("input_topic", "/vicon/LEDTaster/LEDTaster")
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
            "1) +X point, 2) origin, 3) +Y point."
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
                "Next order: 1) +X point, 2) origin, 3) +Y point."
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
        x_pose, origin_pose, y_pose = self.recorded_poses
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
        y_point = _position_from_pose(y_pose)
        offset_vector = _offset_vector_from_parameter(
            self.get_parameter("offset_vector").value
        )

        rotation, diagnostics = _orthonormal_frame_from_points(origin, x_point, y_point)
        start_position = origin + rotation @ offset_vector
        start_quaternion = _quaternion_from_matrix(rotation)

        self.get_logger().info("Input points in map frame:")
        self.get_logger().info(f"  +X point: {_format_vector(x_point)}")
        self.get_logger().info(f"  origin: {_format_vector(origin)}")
        self.get_logger().info(f"  +Y point: {_format_vector(y_point)}")
        self.get_logger().info(
            "Orthonormal coordinate system in map frame (columns are x, y, z):\n"
            f"{np.array2string(rotation, precision=6, suppress_small=True)}"
        )
        self.get_logger().info("Z-axis check against map +Z:")
        self.get_logger().info(f"  raw z-axis: {_format_vector(diagnostics['raw_z_axis'])}")
        self.get_logger().info(f"  final z-axis: {_format_vector(diagnostics['z_axis'])}")
        self.get_logger().info(
            f"  final z-axis - map +Z: {_format_vector(diagnostics['z_difference'])}"
        )
        self.get_logger().info(f"  angular difference: {diagnostics['z_angle_degrees']:.6f} deg")
        self.get_logger().info(
            f"  y-axis flipped to keep z upward: {diagnostics['flipped_y_for_upward_z']}"
        )
        self.get_logger().info(f"  raw x dot y after upward check: {diagnostics['raw_xy_dot']:.9f}")
        self.get_logger().info(f"  rotation determinant: {diagnostics['determinant']:.9f}")
        self.get_logger().info(f"Local offset vector xyz: {_format_vector(offset_vector)}")
        self.get_logger().info("Pose at origin + rotation * offset_vector:")
        self.get_logger().info(f"  position xyz: {_format_vector(start_position)}")
        self.get_logger().info(f"  orientation xyzw: {_format_vector(start_quaternion)}")

        return (
            "Calculated start pose. "
            f"position xyz={_format_vector(start_position)}, "
            f"orientation xyzw={_format_vector(start_quaternion)}"
        )


def main(args=None) -> None:
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
