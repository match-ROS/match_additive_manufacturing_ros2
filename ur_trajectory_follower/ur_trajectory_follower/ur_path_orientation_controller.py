#!/usr/bin/env python3
import math
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, Float32, Int32
from tf_transformations import quaternion_inverse, quaternion_multiply


class OrientationController(Node):
    def __init__(self) -> None:
        super().__init__('ur_path_orientation_controller')
        self.declare_parameter('kp_orientation', 1.0)
        self.declare_parameter('ki_orientation', 0.0)
        self.declare_parameter('kd_orientation', 0.0)
        self.declare_parameter('output_smoothing_coeff', 0.9)
        self.declare_parameter('initial_path_index', -1)
        self.declare_parameter('path_topic', '/ur_path_transformed')
        self.declare_parameter('reference_pose_topic', '/arm_trajectory_reference')
        self.declare_parameter('current_pose_topic', '/current_tcp_pose')
        self.declare_parameter('path_index_topic', '/path_index')
        self.declare_parameter('velocity_override_topic', '/velocity_override')
        self.declare_parameter('twist_topic', '/ur_orientation_twist')
        self.declare_parameter('start_condition_topic', '/start_condition')
        self.declare_parameter('wait_for_start_condition', False)
        self.declare_parameter('hold_reference_on_pause', True)
        self.declare_parameter('final_orientation_tolerance', 0.03)
        self.declare_parameter('final_tolerance_cycles', 3)

        self.kp = float(self.get_parameter('kp_orientation').value)
        self.ki = float(self.get_parameter('ki_orientation').value)
        self.kd = float(self.get_parameter('kd_orientation').value)
        self.smoothing = max(0.0, min(1.0, float(self.get_parameter('output_smoothing_coeff').value)))
        self.integral_error = np.zeros(3)
        self.prev_error = np.zeros(3)
        self.old_twist = Twist()
        self.path: Optional[Path] = None
        self.current_pose: Optional[PoseStamped] = None
        self.reference_pose: Optional[PoseStamped] = None
        self.current_index = max(0, int(self.get_parameter('initial_path_index').value))
        self.velocity_override = 1.0
        self.wait_for_start_condition = self._as_bool(
            self.get_parameter('wait_for_start_condition').value
        )
        self.control_enabled = not self.wait_for_start_condition
        self.final_cycles = 0

        path_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(Path, str(self.get_parameter('path_topic').value), self._path_cb, path_qos)
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter('reference_pose_topic').value),
            self._reference_cb,
            path_qos,
        )
        self.create_subscription(Int32, str(self.get_parameter('path_index_topic').value), self._index_cb, 10)
        self.create_subscription(PoseStamped, str(self.get_parameter('current_pose_topic').value), self._pose_cb, 10)
        self.create_subscription(Float32, str(self.get_parameter('velocity_override_topic').value), self._velocity_cb, 10)
        self.create_subscription(
            Bool,
            str(self.get_parameter('start_condition_topic').value),
            self._start_condition_cb,
            10,
        )
        self.pub = self.create_publisher(Twist, str(self.get_parameter('twist_topic').value), 10)

    @staticmethod
    def _as_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in {'1', 'true', 'yes', 'y', 'on'}

    def _path_cb(self, msg: Path) -> None:
        if len(msg.poses) < 2:
            self.get_logger().warn("Received path with less than two waypoints.")
            return
        self.path = msg
        self.current_index = max(1, min(self.current_index, len(msg.poses) - 1))

    def _index_cb(self, msg: Int32) -> None:
        if self.path is None:
            return
        self.current_index = max(1, min(int(msg.data), len(self.path.poses) - 1))
        self.final_cycles = 0
        self._calculate()

    def _reference_cb(self, msg: PoseStamped) -> None:
        self.reference_pose = msg
        self._calculate()

    def _pose_cb(self, msg: PoseStamped) -> None:
        self.current_pose = msg
        self._calculate()

    def _velocity_cb(self, msg: Float32) -> None:
        self.velocity_override = max(0.0, float(msg.data))
        self._calculate()

    def _start_condition_cb(self, msg: Bool) -> None:
        new_state = bool(msg.data) or not self.wait_for_start_condition
        if new_state == self.control_enabled:
            return
        self.control_enabled = new_state
        if self.control_enabled:
            self.get_logger().info("Start condition fulfilled - enabling orientation controller output.")
            self._calculate()
        else:
            self.old_twist = Twist()
            self.integral_error = np.zeros(3)
            self.prev_error = np.zeros(3)
            self.pub.publish(Twist())

    @staticmethod
    def _axis_angle(quat: np.ndarray):
        norm = np.linalg.norm(quat)
        if norm < 1e-6:
            return np.zeros(3), 0.0
        quat = quat / norm
        w = max(-1.0, min(1.0, float(quat[3])))
        angle = 2.0 * math.acos(w)
        if angle > math.pi:
            angle -= 2.0 * math.pi
        sin_half = math.sqrt(max(1.0 - w * w, 0.0))
        axis = np.array([1.0, 0.0, 0.0]) if sin_half < 1e-6 else quat[0:3] / sin_half
        return axis, angle

    def _smooth(self, twist: Twist) -> Twist:
        out = Twist()
        out.angular.x = self.smoothing * self.old_twist.angular.x + (1.0 - self.smoothing) * twist.angular.x
        out.angular.y = self.smoothing * self.old_twist.angular.y + (1.0 - self.smoothing) * twist.angular.y
        out.angular.z = self.smoothing * self.old_twist.angular.z + (1.0 - self.smoothing) * twist.angular.z
        self.old_twist = out
        return out

    def _calculate(self) -> None:
        if self.current_pose is None:
            return
        if not self.control_enabled:
            return
        goal = self.reference_pose
        if goal is None:
            if self.path is None:
                return
            goal = self.path.poses[self.current_index]
        q_des = np.array([goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w], dtype=float)
        q_cur = np.array([
            self.current_pose.pose.orientation.x,
            self.current_pose.pose.orientation.y,
            self.current_pose.pose.orientation.z,
            self.current_pose.pose.orientation.w,
        ], dtype=float)
        if np.linalg.norm(q_des) < 1e-6 or np.linalg.norm(q_cur) < 1e-6:
            self.get_logger().warn("Invalid quaternion for orientation control.")
            return

        q_err = quaternion_multiply(q_des / np.linalg.norm(q_des), quaternion_inverse(q_cur / np.linalg.norm(q_cur)))
        axis, angle = self._axis_angle(np.array(q_err, dtype=float))
        error = axis * angle
        omega = (error * self.kp + self.integral_error * self.ki + (error - self.prev_error) * self.kd)
        paused = self.velocity_override <= 0.0
        if not (paused and self._as_bool(self.get_parameter('hold_reference_on_pause').value)):
            omega *= self.velocity_override
        if not paused:
            self.integral_error += error
        self.prev_error = error

        at_final = self.path is not None and self.current_index >= len(self.path.poses) - 1
        if at_final and float(np.linalg.norm(error)) <= float(self.get_parameter('final_orientation_tolerance').value):
            self.final_cycles += 1
        else:
            self.final_cycles = 0
        if self.final_cycles >= max(1, int(self.get_parameter('final_tolerance_cycles').value)):
            omega = np.zeros(3)

        twist = Twist()
        twist.angular.x = float(omega[0])
        twist.angular.y = float(omega[1])
        twist.angular.z = float(omega[2])
        self.pub.publish(self._smooth(twist))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OrientationController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
