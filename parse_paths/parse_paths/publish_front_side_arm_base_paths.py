#!/usr/bin/env python3
from copy import deepcopy
from typing import Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Vector3
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool

from parse_paths.path_utils import as_bool, as_float_list, build_orientation, make_pose


class FrontSideArmBasePathPublisher(Node):
    def __init__(self) -> None:
        super().__init__('front_side_arm_base_path_publisher')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('arm_path_topic', '/ur_path_transformed')
        self.declare_parameter('arm_original_path_topic', '/ur_path_original')
        self.declare_parameter('base_path_topic', '/mir_path_transformed')
        self.declare_parameter('base_original_path_topic', '/mir_path_original')
        self.declare_parameter('normal_topic', '/normal_vector')
        self.declare_parameter('use_current_arm_pose', True)
        self.declare_parameter('current_arm_pose_topic', '/current_tcp_pose')
        self.declare_parameter('robot_pose_topic', '/robot_pose')
        self.declare_parameter('robot_pose_type', 'pose_stamped')
        self.declare_parameter('wait_for_home_pose', False)
        self.declare_parameter('home_pose_ready_topic', '/home_pose_ready')
        self.declare_parameter('arm_start_offset', [0.0, 0.0, 0.0])
        self.declare_parameter('arm_start_xyz', [0.6, 0.0, 0.45])
        self.declare_parameter('arm_path_delta', [3.0, 3.0, 0.0])
        self.declare_parameter('nozzle_axis', [0.0, 1.0, 0.0])
        self.declare_parameter('x_axis_hint', [1.0, 0.0, 0.0])
        self.declare_parameter('num_points', 50)
        self.declare_parameter('time_step', 0.1)
        self.declare_parameter('publish_rate', 1.0)

        self.frame_id = str(self.get_parameter('frame_id').value)
        self.arm_path_topic = str(self.get_parameter('arm_path_topic').value)
        self.arm_original_path_topic = str(self.get_parameter('arm_original_path_topic').value)
        self.base_path_topic = str(self.get_parameter('base_path_topic').value)
        self.base_original_path_topic = str(self.get_parameter('base_original_path_topic').value)
        self.normal_topic = str(self.get_parameter('normal_topic').value)
        self.use_current_arm_pose = as_bool(self.get_parameter('use_current_arm_pose').value)
        self.wait_for_home_pose = as_bool(self.get_parameter('wait_for_home_pose').value)
        self.current_arm_pose_topic = str(self.get_parameter('current_arm_pose_topic').value)
        self.robot_pose_topic = str(self.get_parameter('robot_pose_topic').value)
        self.home_pose_ready_topic = str(self.get_parameter('home_pose_ready_topic').value)
        self.arm_start_offset = np.array(as_float_list(self.get_parameter('arm_start_offset').value, [0.0, 0.0, 0.0]), dtype=float)
        self.arm_start_xyz = np.array(as_float_list(self.get_parameter('arm_start_xyz').value, [0.6, 0.0, 0.45]), dtype=float)
        self.arm_path_delta = np.array(as_float_list(self.get_parameter('arm_path_delta').value, [3.0, 3.0, 0.0]), dtype=float)
        self.nozzle_axis = np.array(as_float_list(self.get_parameter('nozzle_axis').value, [0.0, 1.0, 0.0]), dtype=float)
        self.x_axis_hint = np.array(as_float_list(self.get_parameter('x_axis_hint').value, [1.0, 0.0, 0.0]), dtype=float)
        self.num_points = max(2, int(self.get_parameter('num_points').value))
        self.time_step = float(self.get_parameter('time_step').value)

        latch_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, reliability=QoSReliabilityPolicy.RELIABLE)
        self.arm_path_pub = self.create_publisher(Path, self.arm_path_topic, latch_qos)
        self.arm_original_pub = self.create_publisher(Path, self.arm_original_path_topic, latch_qos)
        self.base_path_pub = self.create_publisher(Path, self.base_path_topic, latch_qos)
        self.base_original_pub = self.create_publisher(Path, self.base_original_path_topic, latch_qos)
        self.normal_pub = self.create_publisher(Vector3, self.normal_topic, latch_qos)

        self.current_arm_pose: Optional[PoseStamped] = None
        self.robot_pose: Optional[Pose] = None
        self.home_ready = not self.wait_for_home_pose
        self.arm_path_msg: Optional[Path] = None
        self.base_path_msg: Optional[Path] = None
        self.normal_msg = Vector3()

        if self.use_current_arm_pose:
            self.create_subscription(PoseStamped, self.current_arm_pose_topic, self._arm_pose_cb, 10)
        pose_type = str(self.get_parameter('robot_pose_type').value).strip().lower()
        if pose_type in {'pose', 'geometry_msgs/msg/pose'}:
            self.create_subscription(Pose, self.robot_pose_topic, self._robot_pose_cb, 10)
        else:
            self.create_subscription(PoseStamped, self.robot_pose_topic, self._robot_pose_stamped_cb, 10)
        if self.wait_for_home_pose:
            self.create_subscription(Bool, self.home_pose_ready_topic, self._home_pose_cb, 10)

        rate = max(0.1, float(self.get_parameter('publish_rate').value))
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            "Front/side arm+base path publisher waiting for arm pose, robot pose, and optional home signal."
        )

    def _arm_pose_cb(self, msg: PoseStamped) -> None:
        self.current_arm_pose = msg
        self._ensure_paths()

    def _robot_pose_cb(self, msg: Pose) -> None:
        self.robot_pose = msg
        self._ensure_paths()

    def _robot_pose_stamped_cb(self, msg: PoseStamped) -> None:
        self.robot_pose = msg.pose
        self._ensure_paths()

    def _home_pose_cb(self, msg: Bool) -> None:
        if msg.data:
            self.home_ready = True
            self._ensure_paths()

    def _ensure_paths(self) -> None:
        if self.arm_path_msg is not None or self.base_path_msg is not None or not self.home_ready:
            return
        if self.use_current_arm_pose and self.current_arm_pose is None:
            return
        if self.robot_pose is None:
            return

        if self.use_current_arm_pose:
            pos = self.current_arm_pose.pose.position
            arm_start = np.array([pos.x, pos.y, pos.z], dtype=float)
        else:
            arm_start = self.arm_start_xyz
        arm_start = arm_start + self.arm_start_offset

        robot_pos = self.robot_pose.position
        base_start = np.array([robot_pos.x, robot_pos.y, robot_pos.z], dtype=float)
        base_to_arm_start_xy = base_start[0:2] - arm_start[0:2]

        self.arm_path_msg, self.base_path_msg, self.normal_msg = self._build_paths(
            arm_start,
            base_start,
            base_to_arm_start_xy,
            deepcopy(self.robot_pose.orientation),
        )
        self.get_logger().info(
            f"Publishing {self.num_points} paired arm/base poses. "
            f"Base-to-arm startup XY offset is [{base_to_arm_start_xy[0]:.3f}, "
            f"{base_to_arm_start_xy[1]:.3f}]."
        )

    def _build_paths(
        self,
        arm_start: np.ndarray,
        base_start: np.ndarray,
        base_to_arm_start_xy: np.ndarray,
        base_orientation: Quaternion,
    ) -> Tuple[Path, Path, Vector3]:
        arm_orientation, normal = build_orientation(self.nozzle_axis, self.x_axis_hint)
        arm_path = Path()
        base_path = Path()
        arm_path.header.frame_id = self.frame_id
        base_path.header.frame_id = self.frame_id
        start_time = self.get_clock().now()

        for i in range(self.num_points):
            ratio = i / max(self.num_points - 1, 1)
            stamp = (start_time + Duration(seconds=self.time_step * i)).to_msg()
            arm_position = arm_start + self.arm_path_delta * ratio
            base_position = np.array([
                arm_position[0] + base_to_arm_start_xy[0],
                arm_position[1] + base_to_arm_start_xy[1],
                base_start[2],
            ])
            arm_path.poses.append(make_pose(self.frame_id, stamp, arm_position, arm_orientation))
            base_path.poses.append(make_pose(self.frame_id, stamp, base_position, base_orientation))

        return arm_path, base_path, normal

    def _tick(self) -> None:
        self._ensure_paths()
        if self.arm_path_msg is None or self.base_path_msg is None:
            return
        stamp = self.get_clock().now().to_msg()
        self.arm_path_msg.header.stamp = stamp
        self.base_path_msg.header.stamp = stamp
        self.arm_path_pub.publish(self.arm_path_msg)
        self.arm_original_pub.publish(self.arm_path_msg)
        self.base_path_pub.publish(self.base_path_msg)
        self.base_original_pub.publish(self.base_path_msg)
        self.normal_pub.publish(self.normal_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FrontSideArmBasePathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
