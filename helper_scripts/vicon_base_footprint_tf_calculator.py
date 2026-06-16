#!/usr/bin/env python3

"""
Estimate static transform between the Vicon base marker frame and the robot TF base frame.

Idea:
    Vicon gives:
        T_vicon_world__vicon_base
        T_vicon_world__vicon_tool

    Therefore:
        T_vicon_base__vicon_tool =
            inv(T_vicon_world__vicon_base) * T_vicon_world__vicon_tool

    Robot TF gives:
        T_robot_base__robot_tcp

    If the Vicon tool frame and robot TCP frame are equivalent, then:
        T_vicon_base__robot_base =
            T_vicon_base__vicon_tool * inv(T_robot_base__robot_tcp)

    This is the static transform you usually need to publish between
    the Vicon base marker frame and /robot/base_footprint.
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped, PoseStamped
import tf2_ros


# =========================
# Adjustable configuration
# =========================

# Vicon input topics
VICON_BASE_TOPIC = "/vicon/Base_RB/Base_RB"
VICON_TOOL_TOPIC = "/vicon/Tool_Flange/Tool_Flange"

# Vicon message type:
# Usually Vicon publishes geometry_msgs/TransformStamped.
# Set to "pose" if your Vicon topic publishes geometry_msgs/PoseStamped.
VICON_MSG_TYPE = "transform"   # "transform" or "pose"

# Robot TF frames
ROBOT_BASE_FRAME = "/robot/base_footprint"

# Set this to your robot TCP frame in /tf.
# Examples:
#   "/robot/tool0"
#   "/robot/tcp"
#   "/robot/flange"
ROBOT_TCP_FRAME = "/robot/arm/tcp"

# Optional known offset from robot TCP frame to Vicon tool marker frame.
# Keep identity if Tool_Flange from Vicon corresponds directly to ROBOT_TCP_FRAME.
#
# Meaning:
#   T_robot_tcp__vicon_tool_marker
#
TCP_TO_VICON_TOOL_TRANSLATION = [0.0, 0.0, 0.0]
TCP_TO_VICON_TOOL_QUATERNION_XYZW = [0.0, 0.0, 0.0, 1.0]

# Number of samples used for averaging
NUM_SAMPLES = 200

# Minimum time between accepted samples [s]
SAMPLE_PERIOD = 0.02

# Print every accepted sample?
PRINT_EACH_SAMPLE = False

# If False:
#   output: VICON_BASE_FRAME -> ROBOT_BASE_FRAME
#
# If True:
#   output: ROBOT_BASE_FRAME -> VICON_BASE_FRAME
OUTPUT_INVERT = False

# Parent / child names used in final static publisher command
OUTPUT_PARENT_FRAME = "/vicon/Base_RB/Base_RB"
OUTPUT_CHILD_FRAME = "/robot/base_footprint"

# TF lookup timeout [s]
TF_TIMEOUT = 0.2


# =========================
# Math helpers
# =========================

def normalize_frame_name(name: str) -> str:
    """tf2 frame names should not start with '/'."""
    return name.lstrip("/")


def quaternion_normalize(q):
    q = np.array(q, dtype=float)
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0])
    return q / n


def quaternion_to_matrix(q):
    """Quaternion [x, y, z, w] to 3x3 rotation matrix."""
    x, y, z, w = quaternion_normalize(q)

    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    return np.array([
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz),       2.0 * (xz + wy)],
        [2.0 * (xy + wz),       1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy),       2.0 * (yz + wx),       1.0 - 2.0 * (xx + yy)],
    ])


def matrix_to_quaternion(R):
    """3x3 rotation matrix to quaternion [x, y, z, w]."""
    R = np.array(R, dtype=float)
    tr = np.trace(R)

    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s

    return quaternion_normalize([x, y, z, w])


def transform_matrix(translation, quaternion_xyzw):
    T = np.eye(4)
    T[:3, :3] = quaternion_to_matrix(quaternion_xyzw)
    T[:3, 3] = np.array(translation, dtype=float)
    return T


def invert_transform(T):
    T_inv = np.eye(4)
    R = T[:3, :3]
    p = T[:3, 3]
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ p
    return T_inv


def transform_msg_to_matrix(msg: TransformStamped):
    t = msg.transform.translation
    q = msg.transform.rotation
    return transform_matrix(
        [t.x, t.y, t.z],
        [q.x, q.y, q.z, q.w]
    )


def pose_msg_to_matrix(msg: PoseStamped):
    p = msg.pose.position
    q = msg.pose.orientation
    return transform_matrix(
        [p.x, p.y, p.z],
        [q.x, q.y, q.z, q.w]
    )


def matrix_to_xyz_quat(T):
    xyz = T[:3, 3]
    quat = matrix_to_quaternion(T[:3, :3])
    return xyz, quat


def average_transforms(transform_list):
    """
    Average translation arithmetically and rotation via SVD projection.

    This is simple and works well for small measurement noise.
    For very noisy rotations, use a quaternion averaging method instead.
    """
    translations = np.array([T[:3, 3] for T in transform_list])
    t_avg = np.mean(translations, axis=0)

    R_sum = np.zeros((3, 3))
    for T in transform_list:
        R_sum += T[:3, :3]

    U, _, Vt = np.linalg.svd(R_sum)
    R_avg = U @ Vt

    if np.linalg.det(R_avg) < 0:
        U[:, -1] *= -1
        R_avg = U @ Vt

    T_avg = np.eye(4)
    T_avg[:3, :3] = R_avg
    T_avg[:3, 3] = t_avg
    return T_avg


# =========================
# ROS node
# =========================

class StaticTransformEstimator(Node):
    def __init__(self):
        super().__init__("vicon_static_transform_estimator")

        self.latest_base = None
        self.latest_tool = None
        self.samples = []
        self.last_sample_time = self.get_clock().now()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        if VICON_MSG_TYPE == "transform":
            msg_type = TransformStamped
            self.base_sub = self.create_subscription(
                msg_type, VICON_BASE_TOPIC, self.base_transform_callback, 10
            )
            self.tool_sub = self.create_subscription(
                msg_type, VICON_TOOL_TOPIC, self.tool_transform_callback, 10
            )
        elif VICON_MSG_TYPE == "pose":
            msg_type = PoseStamped
            self.base_sub = self.create_subscription(
                msg_type, VICON_BASE_TOPIC, self.base_pose_callback, 10
            )
            self.tool_sub = self.create_subscription(
                msg_type, VICON_TOOL_TOPIC, self.tool_pose_callback, 10
            )
        else:
            raise ValueError("VICON_MSG_TYPE must be 'transform' or 'pose'.")

        self.timer = self.create_timer(0.01, self.timer_callback)

        self.T_robot_tcp__vicon_tool = transform_matrix(
            TCP_TO_VICON_TOOL_TRANSLATION,
            TCP_TO_VICON_TOOL_QUATERNION_XYZW
        )

        self.get_logger().info("Static transform estimator started.")
        self.get_logger().info(f"Vicon base topic: {VICON_BASE_TOPIC}")
        self.get_logger().info(f"Vicon tool topic: {VICON_TOOL_TOPIC}")
        self.get_logger().info(f"Robot TF: {ROBOT_BASE_FRAME} -> {ROBOT_TCP_FRAME}")
        self.get_logger().info(f"Collecting {NUM_SAMPLES} samples...")

    def base_transform_callback(self, msg):
        self.latest_base = transform_msg_to_matrix(msg)

    def tool_transform_callback(self, msg):
        self.latest_tool = transform_msg_to_matrix(msg)

    def base_pose_callback(self, msg):
        self.latest_base = pose_msg_to_matrix(msg)

    def tool_pose_callback(self, msg):
        self.latest_tool = pose_msg_to_matrix(msg)

    def timer_callback(self):
        if self.latest_base is None or self.latest_tool is None:
            return

        now = self.get_clock().now()
        dt = (now - self.last_sample_time).nanoseconds * 1e-9
        if dt < SAMPLE_PERIOD:
            return

        try:
            tf_msg = self.tf_buffer.lookup_transform(
                normalize_frame_name(ROBOT_BASE_FRAME),
                normalize_frame_name(ROBOT_TCP_FRAME),
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=TF_TIMEOUT)
            )
        except Exception as exc:
            self.get_logger().warn(f"Could not lookup TF yet: {exc}")
            return

        self.last_sample_time = now

        T_vicon_world__vicon_base = self.latest_base
        T_vicon_world__vicon_tool = self.latest_tool
        T_robot_base__robot_tcp = transform_msg_to_matrix(tf_msg)

        # Vicon relative transform:
        # T_vicon_base__vicon_tool
        T_vicon_base__vicon_tool = (
            invert_transform(T_vicon_world__vicon_base)
            @ T_vicon_world__vicon_tool
        )

        # Include optional TCP-to-Vicon-tool offset:
        #
        # T_robot_base__vicon_tool =
        #     T_robot_base__robot_tcp * T_robot_tcp__vicon_tool
        T_robot_base__vicon_tool = (
            T_robot_base__robot_tcp
            @ self.T_robot_tcp__vicon_tool
        )

        # Solve:
        #
        # T_vicon_base__vicon_tool =
        #     T_vicon_base__robot_base * T_robot_base__vicon_tool
        #
        # Therefore:
        #
        # T_vicon_base__robot_base =
        #     T_vicon_base__vicon_tool * inv(T_robot_base__vicon_tool)
        T_vicon_base__robot_base = (
            T_vicon_base__vicon_tool
            @ invert_transform(T_robot_base__vicon_tool)
        )

        if OUTPUT_INVERT:
            T_out = invert_transform(T_vicon_base__robot_base)
        else:
            T_out = T_vicon_base__robot_base

        self.samples.append(T_out)

        if PRINT_EACH_SAMPLE:
            xyz, quat = matrix_to_xyz_quat(T_out)
            self.get_logger().info(
                f"Sample {len(self.samples)}/{NUM_SAMPLES}: "
                f"xyz=({xyz[0]:.6f}, {xyz[1]:.6f}, {xyz[2]:.6f}), "
                f"quat=({quat[0]:.6f}, {quat[1]:.6f}, {quat[2]:.6f}, {quat[3]:.6f})"
            )
        else:
            self.get_logger().info(f"Sample {len(self.samples)}/{NUM_SAMPLES}")

        if len(self.samples) >= NUM_SAMPLES:
            self.print_result()
            rclpy.shutdown()

    def print_result(self):
        T_avg = average_transforms(self.samples)
        xyz, quat = matrix_to_xyz_quat(T_avg)

        parent = OUTPUT_PARENT_FRAME
        child = OUTPUT_CHILD_FRAME

        if OUTPUT_INVERT:
            parent, child = child, parent

        print("\n============================================================")
        print("Estimated static transform")
        print("============================================================")
        print(f"Parent frame: {parent}")
        print(f"Child frame:  {child}")
        print("")
        print("Translation [m]:")
        print(f"  x: {xyz[0]:.9f}")
        print(f"  y: {xyz[1]:.9f}")
        print(f"  z: {xyz[2]:.9f}")
        print("")
        print("Quaternion [x y z w]:")
        print(f"  x: {quat[0]:.9f}")
        print(f"  y: {quat[1]:.9f}")
        print(f"  z: {quat[2]:.9f}")
        print(f"  w: {quat[3]:.9f}")
        print("")
        print("ROS 2 static_transform_publisher command:")
        print(
            "ros2 run tf2_ros static_transform_publisher "
            f"{xyz[0]:.9f} {xyz[1]:.9f} {xyz[2]:.9f} "
            f"{quat[0]:.9f} {quat[1]:.9f} {quat[2]:.9f} {quat[3]:.9f} "
            f"{normalize_frame_name(parent)} {normalize_frame_name(child)}"
        )
        print("============================================================\n")


def main():
    rclpy.init()
    node = StaticTransformEstimator()
    rclpy.spin(node)


if __name__ == "__main__":
    main()