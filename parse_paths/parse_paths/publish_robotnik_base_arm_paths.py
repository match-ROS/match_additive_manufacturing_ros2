#!/usr/bin/env python3
import math
from copy import deepcopy
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Vector3
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from tf_transformations import euler_from_quaternion, quaternion_from_euler

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
) -> List[np.ndarray]:
    points = []
    for index, base_point in enumerate(base_points):
        ratio = index / max(len(base_points) - 1, 1)
        displacement = base_point - base_start
        point = arm_start + displacement + arm_xy_offset
        point[2] = arm_start[2] + arm_xy_offset[2] + float(arm_height_delta) * ratio
        points.append(point)
    return points


class RobotnikBaseArmPathPublisher(Node):
    def __init__(self) -> None:
        super().__init__('robotnik_base_arm_path_publisher')
        self.declare_parameter('frame_id', 'robotnik_simple')
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
        self.declare_parameter('arm_height_delta', 0.2)
        self.declare_parameter('nozzle_axis', [0.0, 1.0, 0.0])
        self.declare_parameter('x_axis_hint', [1.0, 0.0, 0.0])
        self.declare_parameter('num_points', 50)
        self.declare_parameter('time_step', 0.1)
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('publish_once', False)

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
        self.base_path_msg: Optional[Path] = None
        self.arm_path_msg: Optional[Path] = None
        self.normal_msg = Vector3()
        self.has_published_once = False

        if self.use_current_poses:
            self.create_subscription(PoseStamped, self.robot_pose_topic, self._robot_pose_cb, 10)
            self.create_subscription(PoseStamped, self.current_arm_pose_topic, self._arm_pose_cb, 10)
        else:
            self._ensure_paths()

        rate = max(0.1, float(self.get_parameter('publish_rate').value))
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            "Robotnik paired base/arm path publisher waiting for robot and TCP poses."
        )

    def _robot_pose_cb(self, msg: PoseStamped) -> None:
        self.robot_pose = msg.pose
        self._ensure_paths()

    def _arm_pose_cb(self, msg: PoseStamped) -> None:
        self.current_arm_pose = msg
        self._ensure_paths()

    def _ensure_paths(self) -> None:
        if self.base_path_msg is not None and self.arm_path_msg is not None:
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
        )
        self.base_path_msg, self.arm_path_msg, self.normal_msg = self._build_paths(
            base_points,
            arm_points,
            base_yaw,
        )
        self.get_logger().info(
            f"Prepared {len(base_points)} Robotnik paired path poses. "
            f"Base moves sideways then 45 degrees with fixed yaw {base_yaw:.3f} rad."
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
        self.base_path_pub.publish(self.base_path_msg)
        self.base_original_pub.publish(deepcopy(self.base_path_msg))
        self.arm_path_pub.publish(self.arm_path_msg)
        self.arm_original_pub.publish(deepcopy(self.arm_path_msg))
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
