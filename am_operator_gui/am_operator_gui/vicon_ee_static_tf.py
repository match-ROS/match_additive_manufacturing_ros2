import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from tf2_ros import TransformBroadcaster


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

        # Local homogeneous transform: marker cluster -> TCP.
        self.T_marker_tcp = np.array([
            [-0.423156, -0.906056, -0.001210, 0.184687295],
            [0.906036, -0.423136, -0.007349, -0.501541068],
            [0.006147, -0.004206, 0.999972, -0.126693390],
            [0.0, 0.0, 0.0, 1.0],
        ])

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
        r_vicon_marker = R.from_quat([q.x, q.y, q.z, q.w])

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
        tf_marker.transform.rotation = msg.pose.orientation

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
