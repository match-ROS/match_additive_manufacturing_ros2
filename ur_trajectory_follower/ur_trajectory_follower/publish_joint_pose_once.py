#!/usr/bin/env python3
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from ur_trajectory_follower.ros2_utils import as_float_list, as_string_list


class PublishJointPoseOnce(Node):
    def __init__(self) -> None:
        super().__init__('publish_joint_pose_once')
        self.declare_parameter('trajectory_topic', '/ur_joint_trajectory_controller/joint_trajectory')
        self.declare_parameter('joint_names', [])
        self.declare_parameter('positions', [0.0, -1.57, 1.57, -1.57, -1.57, 0.0])
        self.declare_parameter('time_from_start', 4.0)
        self.declare_parameter('publish_delay', 2.0)

        self.pub = self.create_publisher(
            JointTrajectory, str(self.get_parameter('trajectory_topic').value), 10
        )
        delay = max(0.0, float(self.get_parameter('publish_delay').value))
        self.timer = self.create_timer(delay if delay > 0.0 else 0.1, self._publish_once)

    def _publish_once(self) -> None:
        self.timer.cancel()
        names = as_string_list(self.get_parameter('joint_names').value)
        positions = as_float_list(self.get_parameter('positions').value, [])
        if not names or len(names) != len(positions):
            self.get_logger().error(
                f"Cannot publish joint pose: {len(names)} names and {len(positions)} positions."
            )
            return

        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = names
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * len(positions)
        point.time_from_start = Duration(
            seconds=max(0.1, float(self.get_parameter('time_from_start').value))
        ).to_msg()
        msg.points.append(point)
        self.pub.publish(msg)
        self.get_logger().info(f"Published one-shot joint pose to {self.pub.topic_name}.")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PublishJointPoseOnce()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
