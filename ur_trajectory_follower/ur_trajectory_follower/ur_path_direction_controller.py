#!/usr/bin/env python3
from typing import Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, Int32
from tf_transformations import quaternion_matrix


class DirectionController(Node):
    def __init__(self) -> None:
        super().__init__('ur_direction_controller')

        self.declare_parameter('nozzle_height_default', 0.1)
        self.declare_parameter('kp_z', 0.0)
        self.declare_parameter('ki_z', 0.0)
        self.declare_parameter('kd_z', 0.0)
        self.declare_parameter('spray_axis_source', 'tool_z')
        self.declare_parameter('spray_axis_sign', 1.0)
        self.declare_parameter('joint_state_topic', '/mur620c/joint_states')
        self.declare_parameter('lift_joint_name', 'right_lift_joint')
        self.declare_parameter('output_smoothing_coeff', 0.0)
        self.declare_parameter('from_index_offset', -1)
        self.declare_parameter('goal_index_offset', 0)
        self.declare_parameter('start_condition_topic', '/start_condition')
        self.declare_parameter('wait_for_start_condition', True)
        self.declare_parameter('initial_path_index', -1)
        self.declare_parameter('path_index_topic', '/path_index')

        self.nozzle_height_default = float(self.get_parameter('nozzle_height_default').value)
        self.nozzle_height_override = 0.0
        self.kp_z = float(self.get_parameter('kp_z').value)
        self.ki_z = float(self.get_parameter('ki_z').value)
        self.kd_z = float(self.get_parameter('kd_z').value)
        self.get_logger().info(
            f"Spray-axis gains: kp={self.kp_z}, ki={self.ki_z}, kd={self.kd_z}"
        )
        self.spray_axis_source = str(self.get_parameter('spray_axis_source').value)
        self.spray_axis_sign = float(self.get_parameter('spray_axis_sign').value)
        self.joint_state_topic = str(self.get_parameter('joint_state_topic').value)
        self.lift_joint_name = str(self.get_parameter('lift_joint_name').value)
        self.output_smoothing_coeff = float(self.get_parameter('output_smoothing_coeff').value)
        self.output_smoothing_coeff = max(0.0, min(1.0, self.output_smoothing_coeff))
        self.integral_z = 0.0
        self.prev_error_z = 0.0

        self.path: Path = Path()
        self.command_old_twist = Twist()
        self.current_index = 1
        self.trajectory_velocity = 0.0
        self.velocity_override = 1.0
        self.current_lift_height = 0.0
        self.current_pose: Optional[PoseStamped] = None
        self.node_ready = False
        self.path_received = False
        self.joint_state_received = False

        self.from_index_offset = int(self.get_parameter('from_index_offset').value)
        self.goal_index_offset = int(self.get_parameter('goal_index_offset').value)
        self.start_condition_topic = str(self.get_parameter('start_condition_topic').value)
        self.wait_for_start_condition = self._as_bool(
            self.get_parameter('wait_for_start_condition').value
        )
        self.control_enabled = not self.wait_for_start_condition
        self.path_index_topic = str(self.get_parameter('path_index_topic').value)
        self.initial_path_index = self._parse_initial_path_index(
            self.get_parameter('initial_path_index').value
        )
        if self.initial_path_index is not None:
            self.current_index = self.initial_path_index
            self.get_logger().info(
                f"Using initial path index {self.current_index} from parameter."
            )

        self.path_index_received = self.initial_path_index is not None

        self.pub_ur_velocity_world = self.create_publisher(Twist, 'ur_twist_world', 10)

        path_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(Path, 'path', self.path_callback, path_qos)
        self.create_subscription(Int32, self.path_index_topic, self.index_callback, 10)
        self.create_subscription(PoseStamped, 'current_pose', self.ee_pose_callback, 10)
        self.create_subscription(Float32, 'velocity_override', self.velocity_override_callback, 10)
        self.create_subscription(Float32, 'nozzle_height_override', self.nozzle_height_callback, 10)
        self.create_subscription(JointState, self.joint_state_topic, self.joint_state_callback, 10)
        self.create_subscription(Bool, self.start_condition_topic, self.start_condition_callback, 10)

    @staticmethod
    def _as_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in {'1', 'true', 'yes', 'y', 'on'}

    def _parse_initial_path_index(self, raw_value) -> Optional[int]:
        try:
            idx = int(raw_value)
        except (TypeError, ValueError):
            self.get_logger().warn(
                f"Invalid initial_path_index value '{raw_value}', ignoring."
            )
            return None
        return idx if idx >= 0 else None

    def _maybe_set_ready(self) -> None:
        if self.node_ready:
            return
        if not self.path_received or not self.joint_state_received or not self.path_index_received:
            return

        self.node_ready = True
        self.get_logger().info("UR Direction Controller node initialized.")
        self.index_callback(Int32(data=self.current_index))

    def path_callback(self, path_msg: Path) -> None:
        self.path = path_msg
        if not self.path_received:
            self.path_received = True
            self._maybe_set_ready()

    def nozzle_height_callback(self, height_msg: Float32) -> None:
        self.nozzle_height_override = float(height_msg.data)

    def joint_state_callback(self, msg: JointState) -> None:
        if not self.joint_state_received:
            self.joint_state_received = True
            self._maybe_set_ready()
        try:
            idx = msg.name.index(self.lift_joint_name)
            self.current_lift_height = msg.position[idx]
        except ValueError:
            pass

    def index_callback(self, index_msg: Int32) -> None:
        self.current_index = int(index_msg.data)
        if not self.path_index_received:
            self.path_index_received = True
            self._maybe_set_ready()
        if not self.node_ready or not self.control_enabled:
            return

        self.get_traj_velocity(self.from_index_offset, self.goal_index_offset)
        self.calculate_twist(self.from_index_offset, self.goal_index_offset)

    def start_condition_callback(self, msg: Bool) -> None:
        new_state = bool(msg.data) or not self.wait_for_start_condition
        if new_state == self.control_enabled:
            return

        if new_state:
            self.get_logger().info(
                "Start condition fulfilled - enabling UR direction controller output."
            )
        else:
            self.get_logger().info(
                "Start condition reset - holding UR direction controller output."
            )
            self.command_old_twist = Twist()
            self.integral_z = 0.0
            self.prev_error_z = 0.0
            self.pub_ur_velocity_world.publish(Twist())

        self.control_enabled = new_state

    def velocity_override_callback(self, velocity_msg: Float32) -> None:
        self.velocity_override = float(velocity_msg.data)

    def ee_pose_callback(self, pose_msg: PoseStamped) -> None:
        self.current_pose = pose_msg
        if not self.node_ready:
            return
        self.calculate_twist(self.from_index_offset, self.goal_index_offset)

    def _clamp_path_index(self, target_index: int) -> int:
        if not self.path.poses:
            return 0
        return max(0, min(target_index, len(self.path.poses) - 1))

    def get_traj_velocity(self, from_offset: int, goal_offset: int) -> None:
        if not self.path.poses:
            self.get_logger().warn("Received empty path; trajectory velocity set to zero.")
            self.trajectory_velocity = 0.0
            return
        last_idx = self._clamp_path_index(self.current_index + from_offset)
        next_idx = self._clamp_path_index(self.current_index + goal_offset)
        if last_idx == next_idx:
            self.get_logger().warn(
                "Configured waypoint offsets select identical indices; trajectory velocity set to zero."
            )
            self.trajectory_velocity = 0.0
            return

        last_waypoint = self.path.poses[last_idx]
        next_waypoint = self.path.poses[next_idx]

        distance = (
            (next_waypoint.pose.position.x - last_waypoint.pose.position.x) ** 2
            + (next_waypoint.pose.position.y - last_waypoint.pose.position.y) ** 2
        ) ** 0.5
        t_last = Time.from_msg(last_waypoint.header.stamp)
        t_next = Time.from_msg(next_waypoint.header.stamp)
        dt = (t_next - t_last).nanoseconds / 1e9
        if dt > 0.0:
            self.trajectory_velocity = distance / dt
        else:
            self.get_logger().warn(
                "Non-positive time delta encountered in trajectory velocity calculation."
            )
            self.trajectory_velocity = 0.0

    def _get_goal_pose(self, goal_offset: int) -> Optional[PoseStamped]:
        if not self.path.poses:
            return None
        goal_idx = self._clamp_path_index(self.current_index + goal_offset)
        return self.path.poses[goal_idx]

    @staticmethod
    def _normalize_vector(vec: np.ndarray, fallback: np.ndarray) -> np.ndarray:
        norm = np.linalg.norm(vec)
        if norm < 1e-6:
            return fallback
        return vec / norm

    def _get_spray_axis(self, goal_pose: PoseStamped) -> np.ndarray:
        orientation = goal_pose.pose.orientation
        quat = np.array([orientation.x, orientation.y, orientation.z, orientation.w], dtype=float)
        quat_norm = np.linalg.norm(quat)
        if quat_norm < 1e-6:
            return np.array([0.0, 0.0, 1.0])
        quat /= quat_norm
        rotation = quaternion_matrix(quat)
        axes = {
            'tool_x': rotation[0:3, 0],
            'tool_y': rotation[0:3, 1],
            'tool_z': rotation[0:3, 2],
        }
        axis = axes.get(self.spray_axis_source, axes['tool_z'])
        axis = self._normalize_vector(axis, np.array([0.0, 0.0, 1.0]))
        if self.spray_axis_sign < 0.0:
            axis = -axis
        return axis

    def get_direction(
        self, from_offset: int, goal_offset: int
    ) -> Tuple[np.ndarray, float, np.ndarray]:
        if not self.path.poses or self.current_pose is None:
            return np.zeros(3), 0.0, np.array([0.0, 0.0, 1.0])

        goal_pose = self._get_goal_pose(goal_offset)
        if goal_pose is None:
            return np.zeros(3), 0.0, np.array([0.0, 0.0, 1.0])

        direction = np.array(
            [
                goal_pose.pose.position.x - self.current_pose.pose.position.x,
                goal_pose.pose.position.y - self.current_pose.pose.position.y,
                goal_pose.pose.position.z - self.current_pose.pose.position.z,
            ]
        )
        spray_axis = self._get_spray_axis(goal_pose)
        error_spray = float(np.dot(direction, spray_axis))
        direction_plane = direction - error_spray * spray_axis
        direction_plane_norm = self._normalize_vector(direction_plane, np.zeros(3))
        return direction_plane_norm, error_spray, spray_axis

    def smooth_output(self, control_command: Twist) -> Twist:
        smoothed_command = Twist()
        smoothed_command.linear.x = (
            self.output_smoothing_coeff * self.command_old_twist.linear.x
            + (1 - self.output_smoothing_coeff) * control_command.linear.x
        )
        smoothed_command.linear.y = (
            self.output_smoothing_coeff * self.command_old_twist.linear.y
            + (1 - self.output_smoothing_coeff) * control_command.linear.y
        )
        smoothed_command.linear.z = (
            self.output_smoothing_coeff * self.command_old_twist.linear.z
            + (1 - self.output_smoothing_coeff) * control_command.linear.z
        )
        self.command_old_twist = smoothed_command

        return smoothed_command

    def calculate_twist(self, from_offset: int, goal_offset: int) -> None:
        if self.current_pose is None:
            self.get_logger().warn("No current pose received yet.")
            return
        if not self.control_enabled:
            return

        direction_plane_norm, error_spray, spray_axis = self.get_direction(
            from_offset, goal_offset
        )
        v_plane = direction_plane_norm * self.trajectory_velocity * self.velocity_override

        error_spray += self.nozzle_height_default + self.nozzle_height_override
        v_spray = (
            error_spray * self.kp_z
            + self.integral_z * self.ki_z
            + (error_spray - self.prev_error_z) * self.kd_z
        )
        self.integral_z += error_spray
        self.prev_error_z = error_spray

        v_spray_vec = spray_axis * v_spray
        v_cmd = v_plane + v_spray_vec

        control_command = Twist()
        control_command.linear.x = float(v_cmd[0])
        control_command.linear.y = float(v_cmd[1])
        control_command.linear.z = float(v_cmd[2])
        control_command_smoothed = self.smooth_output(control_command)

        self.pub_ur_velocity_world.publish(control_command_smoothed)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DirectionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
