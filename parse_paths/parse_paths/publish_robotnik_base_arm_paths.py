#!/usr/bin/env python3
import json
import math
from copy import deepcopy
from pathlib import Path as FilePath
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Vector3
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool
from tf_transformations import euler_from_quaternion, quaternion_from_euler

from parse_paths.path_transform import transform_path, transform_vector
from parse_paths.path_utils import as_bool, as_float_list, build_orientation, make_pose
from parse_paths.test_path_shapes import generate_waypoints


def yaw_from_quaternion(orientation: Quaternion) -> float:
    return float(euler_from_quaternion([
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
    ])[2])


def quaternion_from_yaw(yaw: float) -> Quaternion:
    quat = quaternion_from_euler(0.0, 0.0, yaw)
    return Quaternion(x=float(quat[0]), y=float(quat[1]), z=float(quat[2]), w=float(quat[3]))


def as_vector3(value, fallback) -> np.ndarray:
    values = as_float_list(value, fallback)
    values = (values + list(fallback))[:3]
    return np.array(values, dtype=float)


def rotate_xy(vector: np.ndarray, yaw: float) -> np.ndarray:
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return np.array([
        cos_yaw * vector[0] - sin_yaw * vector[1],
        sin_yaw * vector[0] + cos_yaw * vector[1],
        vector[2],
    ])


def generate_sideways_then_diagonal_points(
    base_start: np.ndarray,
    base_yaw: float,
    sideways_distance: float,
    diagonal_distance: float,
    num_points: int,
) -> List[np.ndarray]:
    diagonal_step = float(diagonal_distance) / math.sqrt(2.0)
    local_points = generate_waypoints(
        [
            0.0, 0.0, 0.0,
            0.0, float(sideways_distance), 0.0,
            diagonal_step, float(sideways_distance) + diagonal_step, 0.0,
        ],
        max(2, int(num_points)),
    )
    return [base_start + rotate_xy(point, base_yaw) for point in local_points]


def generate_arm_points(
    arm_start: np.ndarray,
    base_points: List[np.ndarray],
    base_start: np.ndarray,
    arm_xy_offset: np.ndarray,
    arm_height_delta: float,
    ramp_xy_offset: bool = True,
) -> List[np.ndarray]:
    points = []
    for index, base_point in enumerate(base_points):
        ratio = index / max(len(base_points) - 1, 1)
        displacement = base_point - base_start
        xy_offset = arm_xy_offset * ratio if ramp_xy_offset else arm_xy_offset
        point = arm_start + displacement + xy_offset
        point[2] = arm_start[2] + xy_offset[2] + float(arm_height_delta) * ratio
        points.append(point)
    return points


def base_to_arm_planar_distances(
    base_points: List[np.ndarray],
    arm_points: List[np.ndarray],
) -> List[float]:
    return [
        float(np.linalg.norm((arm_point - base_point)[0:2]))
        for base_point, arm_point in zip(base_points, arm_points)
    ]


def arm_base_points(
    base_points: List[np.ndarray],
    base_yaw: float,
    arm_base_offset: np.ndarray,
) -> List[np.ndarray]:
    """Return planned arm-base positions in the path frame.

    The mobile-base pose is not the UR base pose on the RB-VOGUI.  Keep this
    transformation explicit so reach checks use the same mechanical mounting
    geometry as the controller TF tree.
    """
    offset = rotate_xy(arm_base_offset, base_yaw)
    return [base_point + offset for base_point in base_points]


def arm_base_to_arm_planar_distances(
    arm_base_points_: List[np.ndarray],
    arm_points: List[np.ndarray],
) -> List[float]:
    return [
        float(np.linalg.norm((arm_point - arm_base_point)[0:2]))
        for arm_base_point, arm_point in zip(arm_base_points_, arm_points)
    ]


def pose_stamped_to_dict(pose_stamped: PoseStamped) -> dict:
    pose = pose_stamped.pose
    return {
        'frame_id': pose_stamped.header.frame_id,
        'stamp': {
            'sec': int(pose_stamped.header.stamp.sec),
            'nanosec': int(pose_stamped.header.stamp.nanosec),
        },
        'position': {
            'x': float(pose.position.x),
            'y': float(pose.position.y),
            'z': float(pose.position.z),
        },
        'orientation': {
            'x': float(pose.orientation.x),
            'y': float(pose.orientation.y),
            'z': float(pose.orientation.z),
            'w': float(pose.orientation.w),
        },
    }


def pose_stamped_from_dict(data: dict, fallback_frame_id: str) -> PoseStamped:
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = str(data.get('frame_id') or fallback_frame_id)
    stamp = data.get('stamp', {})
    pose_stamped.header.stamp.sec = int(stamp.get('sec', 0))
    pose_stamped.header.stamp.nanosec = int(stamp.get('nanosec', 0))
    position = data.get('position', {})
    orientation = data.get('orientation', {})
    pose_stamped.pose.position.x = float(position.get('x', 0.0))
    pose_stamped.pose.position.y = float(position.get('y', 0.0))
    pose_stamped.pose.position.z = float(position.get('z', 0.0))
    pose_stamped.pose.orientation.x = float(orientation.get('x', 0.0))
    pose_stamped.pose.orientation.y = float(orientation.get('y', 0.0))
    pose_stamped.pose.orientation.z = float(orientation.get('z', 0.0))
    pose_stamped.pose.orientation.w = float(orientation.get('w', 1.0))
    return pose_stamped


def path_to_dict(path: Path) -> dict:
    return {
        'frame_id': path.header.frame_id,
        'poses': [pose_stamped_to_dict(pose) for pose in path.poses],
    }


def path_from_dict(data: dict, fallback_frame_id: str) -> Path:
    path = Path()
    path.header.frame_id = str(data.get('frame_id') or fallback_frame_id)
    path.poses = [
        pose_stamped_from_dict(pose_data, path.header.frame_id)
        for pose_data in data.get('poses', [])
    ]
    return path


def vector3_to_dict(vector: Vector3) -> dict:
    return {
        'x': float(vector.x),
        'y': float(vector.y),
        'z': float(vector.z),
    }


def vector3_from_dict(data: dict) -> Vector3:
    return Vector3(
        x=float(data.get('x', 0.0)),
        y=float(data.get('y', 0.0)),
        z=float(data.get('z', 0.0)),
    )


class RobotnikBaseArmPathPublisher(Node):
    def __init__(self) -> None:
        super().__init__('robotnik_base_arm_path_publisher')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('base_path_topic', '/base_path')
        self.declare_parameter('base_original_path_topic', '/base_path_original')
        self.declare_parameter('arm_path_topic', '/ur_path_transformed')
        self.declare_parameter('arm_original_path_topic', '/ur_path_original')
        self.declare_parameter('normal_topic', '/normal_vector')
        self.declare_parameter('robot_pose_topic', '/robot_pose')
        self.declare_parameter('current_arm_pose_topic', '/current_tcp_pose')
        self.declare_parameter('use_current_poses', True)
        self.declare_parameter('base_start_xyz', [0.0, 0.0, 0.0])
        self.declare_parameter('base_start_offset', [0.0, 0.0, 0.0])
        self.declare_parameter('base_yaw', 0.0)
        self.declare_parameter('arm_start_xyz', [0.6, 0.0, 0.8])
        self.declare_parameter('sideways_distance', 0.8)
        self.declare_parameter('diagonal_distance', 0.8)
        self.declare_parameter('arm_xy_offset', [0.15, 0.0, 0.0])
        self.declare_parameter('ramp_arm_xy_offset', True)
        self.declare_parameter('arm_height_delta', 0.2)
        self.declare_parameter('arm_base_offset', [0.0, 0.0, 0.0])
        self.declare_parameter('min_reachable_radius', 0.25)
        self.declare_parameter('max_reachable_radius', 0.85)
        self.declare_parameter('nozzle_axis', [0.0, 1.0, 0.0])
        self.declare_parameter('x_axis_hint', [1.0, 0.0, 0.0])
        self.declare_parameter('num_points', 50)
        self.declare_parameter('time_step', 0.1)
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('publish_once', True)
        self.declare_parameter('wait_for_trigger', False)
        self.declare_parameter('trigger_topic', '/start_pose_reached')
        self.declare_parameter('export_trajectories', False)
        self.declare_parameter('load_exported_trajectories', False)
        self.declare_parameter(
            'trajectory_directory',
            'match_additive_manufacturing_ros2/components/robotnik_paired_demo',
        )
        self.declare_parameter('base_trajectory_filename', 'base_path.json')
        self.declare_parameter('arm_trajectory_filename', 'arm_path.json')
        self.declare_parameter('normal_filename', 'normal_vector.json')
        self.declare_parameter('path_transform_xyz', [0.0, 0.0, 0.0])
        self.declare_parameter('path_transform_yaw_deg', 0.0)

        self.frame_id = str(self.get_parameter('frame_id').value)
        self.base_path_topic = str(self.get_parameter('base_path_topic').value)
        self.base_original_path_topic = str(self.get_parameter('base_original_path_topic').value)
        self.arm_path_topic = str(self.get_parameter('arm_path_topic').value)
        self.arm_original_path_topic = str(self.get_parameter('arm_original_path_topic').value)
        self.normal_topic = str(self.get_parameter('normal_topic').value)
        self.robot_pose_topic = str(self.get_parameter('robot_pose_topic').value)
        self.current_arm_pose_topic = str(self.get_parameter('current_arm_pose_topic').value)
        self.use_current_poses = as_bool(self.get_parameter('use_current_poses').value)
        self.num_points = max(2, int(self.get_parameter('num_points').value))
        self.time_step = float(self.get_parameter('time_step').value)
        self.publish_once = as_bool(self.get_parameter('publish_once').value)
        self.wait_for_trigger = as_bool(self.get_parameter('wait_for_trigger').value)
        self.trigger_received = not self.wait_for_trigger
        self.export_trajectories = as_bool(self.get_parameter('export_trajectories').value)
        self.load_exported_trajectories = as_bool(
            self.get_parameter('load_exported_trajectories').value
        )
        self.trajectory_directory = FilePath(
            str(self.get_parameter('trajectory_directory').value)
        ).expanduser()
        self.base_trajectory_filename = str(self.get_parameter('base_trajectory_filename').value)
        self.arm_trajectory_filename = str(self.get_parameter('arm_trajectory_filename').value)
        self.normal_filename = str(self.get_parameter('normal_filename').value)
        self.path_transform_xyz = as_float_list(
            self.get_parameter('path_transform_xyz').value,
            [0.0, 0.0, 0.0],
        )
        self.path_transform_yaw_deg = float(self.get_parameter('path_transform_yaw_deg').value)

        latch_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.base_path_pub = self.create_publisher(Path, self.base_path_topic, latch_qos)
        self.base_original_pub = self.create_publisher(Path, self.base_original_path_topic, latch_qos)
        self.arm_path_pub = self.create_publisher(Path, self.arm_path_topic, latch_qos)
        self.arm_original_pub = self.create_publisher(Path, self.arm_original_path_topic, latch_qos)
        self.normal_pub = self.create_publisher(Vector3, self.normal_topic, latch_qos)

        self.robot_pose: Optional[Pose] = None
        self.current_arm_pose: Optional[PoseStamped] = None
        self.base_original_path_msg: Optional[Path] = None
        self.arm_original_path_msg: Optional[Path] = None
        self.base_path_msg: Optional[Path] = None
        self.arm_path_msg: Optional[Path] = None
        self.original_normal_msg = Vector3()
        self.normal_msg = Vector3()
        self.has_published_once = False

        if self.load_exported_trajectories:
            self._load_paths()
        elif self.use_current_poses:
            self.create_subscription(PoseStamped, self.robot_pose_topic, self._robot_pose_cb, 10)
            self.create_subscription(PoseStamped, self.current_arm_pose_topic, self._arm_pose_cb, 10)
        else:
            self._ensure_paths()
        if self.wait_for_trigger and not self.load_exported_trajectories:
            self.create_subscription(
                Bool,
                str(self.get_parameter('trigger_topic').value),
                self._trigger_cb,
                latch_qos,
            )

        rate = max(0.1, float(self.get_parameter('publish_rate').value))
        self.create_timer(1.0 / rate, self._tick)
        if self.load_exported_trajectories:
            self.get_logger().info(
                f"Robotnik paired base/arm path publisher loaded exported paths from "
                f"{self.trajectory_directory}."
            )
        else:
            wait_msg = " and trigger" if self.wait_for_trigger else ""
            self.get_logger().info(
                f"Robotnik paired base/arm path publisher waiting for robot and TCP poses{wait_msg}."
            )

    def _trigger_cb(self, msg: Bool) -> None:
        if not msg.data:
            return
        if self.trigger_received:
            return
        self.trigger_received = True
        self.base_original_path_msg = None
        self.arm_original_path_msg = None
        self.base_path_msg = None
        self.arm_path_msg = None
        self.has_published_once = False
        self.get_logger().info("Path generation trigger received.")
        self._ensure_paths()

    def _robot_pose_cb(self, msg: PoseStamped) -> None:
        self.robot_pose = msg.pose
        self._ensure_paths()

    def _arm_pose_cb(self, msg: PoseStamped) -> None:
        self.current_arm_pose = msg
        self._ensure_paths()

    def _ensure_paths(self) -> None:
        if self.base_path_msg is not None and self.arm_path_msg is not None:
            return
        if not self.trigger_received:
            return
        if self.use_current_poses and (self.robot_pose is None or self.current_arm_pose is None):
            return

        if self.use_current_poses:
            assert self.robot_pose is not None
            assert self.current_arm_pose is not None
            base_position = self.robot_pose.position
            base_start = np.array([base_position.x, base_position.y, base_position.z], dtype=float)
            base_yaw = yaw_from_quaternion(self.robot_pose.orientation)
            base_start += rotate_xy(
                as_vector3(self.get_parameter('base_start_offset').value, [0.0, 0.0, 0.0]),
                base_yaw,
            )
            arm_position = self.current_arm_pose.pose.position
            arm_start = np.array([arm_position.x, arm_position.y, arm_position.z], dtype=float)
        else:
            base_start = as_vector3(self.get_parameter('base_start_xyz').value, [0.0, 0.0, 0.0])
            base_yaw = float(self.get_parameter('base_yaw').value)
            base_start += rotate_xy(
                as_vector3(self.get_parameter('base_start_offset').value, [0.0, 0.0, 0.0]),
                base_yaw,
            )
            arm_start = as_vector3(self.get_parameter('arm_start_xyz').value, [0.6, 0.0, 0.8])

        base_points = generate_sideways_then_diagonal_points(
            base_start,
            base_yaw,
            float(self.get_parameter('sideways_distance').value),
            float(self.get_parameter('diagonal_distance').value),
            self.num_points,
        )
        arm_points = generate_arm_points(
            arm_start,
            base_points,
            base_start,
            as_vector3(self.get_parameter('arm_xy_offset').value, [0.15, 0.0, 0.0]),
            float(self.get_parameter('arm_height_delta').value),
            as_bool(self.get_parameter('ramp_arm_xy_offset').value),
        )
        arm_bases = arm_base_points(
            base_points,
            base_yaw,
            as_vector3(self.get_parameter('arm_base_offset').value, [0.0, 0.0, 0.0]),
        )
        self._warn_if_unreachable(arm_bases, arm_points)
        self.base_original_path_msg, self.arm_original_path_msg, self.original_normal_msg = self._build_paths(
            base_points,
            arm_points,
            base_yaw,
        )
        self._apply_path_transform()
        if self.export_trajectories:
            self._export_paths()
        self.get_logger().info(
            f"Prepared {len(base_points)} Robotnik paired path poses. "
            f"Base moves sideways then 45 degrees with fixed yaw {base_yaw:.3f} rad."
        )

    def _path_file(self, filename: str) -> FilePath:
        return self.trajectory_directory / filename

    def _export_paths(self) -> None:
        if self.base_original_path_msg is None or self.arm_original_path_msg is None:
            return
        self.trajectory_directory.mkdir(parents=True, exist_ok=True)
        self._path_file(self.base_trajectory_filename).write_text(
            json.dumps(path_to_dict(self.base_original_path_msg), indent=2),
            encoding='utf-8',
        )
        self._path_file(self.arm_trajectory_filename).write_text(
            json.dumps(path_to_dict(self.arm_original_path_msg), indent=2),
            encoding='utf-8',
        )
        self._path_file(self.normal_filename).write_text(
            json.dumps(vector3_to_dict(self.original_normal_msg), indent=2),
            encoding='utf-8',
        )
        self.get_logger().info(
            f"Exported Robotnik paired paths to {self.trajectory_directory}."
        )

    def _load_paths(self) -> None:
        base_file = self._path_file(self.base_trajectory_filename)
        arm_file = self._path_file(self.arm_trajectory_filename)
        normal_file = self._path_file(self.normal_filename)
        if not base_file.exists() or not arm_file.exists():
            raise FileNotFoundError(
                f"Exported trajectory files not found: {base_file} and/or {arm_file}"
            )

        self.base_original_path_msg = path_from_dict(
            json.loads(base_file.read_text(encoding='utf-8')),
            self.frame_id,
        )
        self.arm_original_path_msg = path_from_dict(
            json.loads(arm_file.read_text(encoding='utf-8')),
            self.frame_id,
        )
        if normal_file.exists():
            self.original_normal_msg = vector3_from_dict(
                json.loads(normal_file.read_text(encoding='utf-8'))
            )
        else:
            _, normal = build_orientation(
                np.array(as_float_list(self.get_parameter('nozzle_axis').value, [0.0, 1.0, 0.0]), dtype=float),
                np.array(as_float_list(self.get_parameter('x_axis_hint').value, [1.0, 0.0, 0.0]), dtype=float),
            )
            self.original_normal_msg = normal

        if len(self.base_original_path_msg.poses) != len(self.arm_original_path_msg.poses):
            raise ValueError(
                "Exported base and arm paths must have the same number of poses: "
                f"{len(self.base_original_path_msg.poses)} != {len(self.arm_original_path_msg.poses)}"
            )
        self._apply_path_transform()

    def _apply_path_transform(self) -> None:
        if self.base_original_path_msg is None or self.arm_original_path_msg is None:
            return
        self.base_path_msg = transform_path(
            self.base_original_path_msg,
            self.path_transform_xyz,
            self.path_transform_yaw_deg,
        )
        self.arm_path_msg = transform_path(
            self.arm_original_path_msg,
            self.path_transform_xyz,
            self.path_transform_yaw_deg,
        )
        self.normal_msg = transform_vector(self.original_normal_msg, self.path_transform_yaw_deg)

    def _warn_if_unreachable(
        self,
        arm_base_points_: List[np.ndarray],
        arm_points: List[np.ndarray],
    ) -> None:
        distances = arm_base_to_arm_planar_distances(arm_base_points_, arm_points)
        if not distances:
            return
        min_radius = float(self.get_parameter('min_reachable_radius').value)
        max_radius = float(self.get_parameter('max_reachable_radius').value)
        min_distance = min(distances)
        max_distance = max(distances)
        if min_distance < min_radius or max_distance > max_radius:
            self.get_logger().warn(
                "Robotnik paired arm path may be outside conservative UR reach: "
                f"planar arm-base-to-TCP distance range {min_distance:.3f}..{max_distance:.3f} m "
                f"not within {min_radius:.3f}..{max_radius:.3f} m."
            )

    def _build_paths(
        self,
        base_points: List[np.ndarray],
        arm_points: List[np.ndarray],
        base_yaw: float,
    ) -> Tuple[Path, Path, Vector3]:
        base_orientation = quaternion_from_yaw(base_yaw)
        arm_orientation, normal = build_orientation(
            np.array(as_float_list(self.get_parameter('nozzle_axis').value, [0.0, 1.0, 0.0]), dtype=float),
            np.array(as_float_list(self.get_parameter('x_axis_hint').value, [1.0, 0.0, 0.0]), dtype=float),
        )
        base_path = Path()
        arm_path = Path()
        base_path.header.frame_id = self.frame_id
        arm_path.header.frame_id = self.frame_id
        start_time = self.get_clock().now()

        for index, (base_point, arm_point) in enumerate(zip(base_points, arm_points)):
            stamp = (start_time + Duration(seconds=self.time_step * index)).to_msg()
            base_path.poses.append(make_pose(self.frame_id, stamp, base_point, base_orientation))
            arm_path.poses.append(make_pose(self.frame_id, stamp, arm_point, arm_orientation))

        return base_path, arm_path, normal

    def _tick(self) -> None:
        self._ensure_paths()
        if self.base_path_msg is None or self.arm_path_msg is None:
            return
        if self.publish_once and self.has_published_once:
            return
        stamp = self.get_clock().now().to_msg()
        self.base_path_msg.header.stamp = stamp
        self.arm_path_msg.header.stamp = stamp
        if self.base_original_path_msg is not None:
            self.base_original_path_msg.header.stamp = stamp
        if self.arm_original_path_msg is not None:
            self.arm_original_path_msg.header.stamp = stamp
        self.base_path_pub.publish(self.base_path_msg)
        self.base_original_pub.publish(deepcopy(self.base_original_path_msg or self.base_path_msg))
        self.arm_path_pub.publish(self.arm_path_msg)
        self.arm_original_pub.publish(deepcopy(self.arm_original_path_msg or self.arm_path_msg))
        self.normal_pub.publish(self.normal_msg)
        self.has_published_once = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[RobotnikBaseArmPathPublisher] = None
    try:
        node = RobotnikBaseArmPathPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
