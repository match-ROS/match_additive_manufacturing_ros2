import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from tf2_ros import TransformBroadcaster

from .tool_transforms import DEFAULT_VICON_NOZZLE_TRANSFORM, validated_transform


class ViconToolTransform(Node):
    def __init__(self):
        super().__init__("vicon_tool_transform")

        self.declare_parameter("input_topic", "/vicon/Tool_Flange/Tool_Flange")
        self.declare_parameter("output_topic", "/vicon/tool_transformed")
        self.declare_parameter("marker_frame", "Tool_Flange")
        self.declare_parameter("tcp_frame", "tool_transformed")

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.marker_frame = str(self.get_parameter("marker_frame").value)
        self.tcp_frame = str(self.get_parameter("tcp_frame").value)

        # Nozzle pose relative to the measured marker/EE frame; independent
        # from the robot flange-to-nozzle kinematic calibration.
        self.declare_parameter('marker_to_nozzle_xyz', DEFAULT_VICON_NOZZLE_TRANSFORM['xyz'])
        self.declare_parameter('marker_to_nozzle_quaternion_xyzw',
                               DEFAULT_VICON_NOZZLE_TRANSFORM['quaternion_xyzw'])
        offset = validated_transform({
            'xyz': list(self.get_parameter('marker_to_nozzle_xyz').value),
            'quaternion_xyzw': list(self.get_parameter('marker_to_nozzle_quaternion_xyzw').value),
        })
        self.T_marker_tcp = np.eye(4)
        self.T_marker_tcp[:3, :3] = R.from_quat(offset['quaternion_xyzw']).as_matrix()
        self.T_marker_tcp[:3, 3] = offset['xyz']

        self.pub = self.create_publisher(PoseStamped, self.output_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.sub = self.create_subscription(
            PoseStamped,
            self.input_topic,
            self.callback,
            10,
        )

    def callback(self, msg: PoseStamped):
        t_vicon_marker = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ])

        q = msg.pose.orientation
        quaternion = np.array([q.x, q.y, q.z, q.w])
        norm = math.hypot(*quaternion)
        if (not np.isfinite(t_vicon_marker).all()
                or not np.isfinite(quaternion).all()
                or not math.isfinite(norm) or norm < 1e-9):
            self.get_logger().warning(
                'Skipping invalid Vicon pose: position and orientation must be finite '
                'and the quaternion must have nonzero norm.',
                throttle_duration_sec=2.0,
            )
            return
        quaternion /= norm
        r_vicon_marker = R.from_quat(quaternion)

        T_vicon_marker = np.eye(4)
        T_vicon_marker[:3, :3] = r_vicon_marker.as_matrix()
        T_vicon_marker[:3, 3] = t_vicon_marker

        T_vicon_tcp = T_vicon_marker @ self.T_marker_tcp
        t_vicon_tcp = T_vicon_tcp[:3, 3]
        r_vicon_tcp = R.from_matrix(T_vicon_tcp[:3, :3])

        q_tcp = r_vicon_tcp.as_quat()

        out = PoseStamped()
        out.header = msg.header
        out.pose.position.x = float(t_vicon_tcp[0])
        out.pose.position.y = float(t_vicon_tcp[1])
        out.pose.position.z = float(t_vicon_tcp[2])
        out.pose.orientation.x = float(q_tcp[0])
        out.pose.orientation.y = float(q_tcp[1])
        out.pose.orientation.z = float(q_tcp[2])
        out.pose.orientation.w = float(q_tcp[3])

        self.pub.publish(out)

        tf_marker = TransformStamped()
        tf_marker.header = msg.header
        tf_marker.child_frame_id = self.marker_frame
        tf_marker.transform.translation.x = msg.pose.position.x
        tf_marker.transform.translation.y = msg.pose.position.y
        tf_marker.transform.translation.z = msg.pose.position.z
        (tf_marker.transform.rotation.x, tf_marker.transform.rotation.y,
         tf_marker.transform.rotation.z, tf_marker.transform.rotation.w) = quaternion.tolist()

        tf_tcp = TransformStamped()
        tf_tcp.header = msg.header
        tf_tcp.child_frame_id = self.tcp_frame
        tf_tcp.transform.translation.x = out.pose.position.x
        tf_tcp.transform.translation.y = out.pose.position.y
        tf_tcp.transform.translation.z = out.pose.position.z
        tf_tcp.transform.rotation = out.pose.orientation

        self.tf_broadcaster.sendTransform(tf_marker)
        self.tf_broadcaster.sendTransform(tf_tcp)


def main():
    rclpy.init()
    node = ViconToolTransform()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
