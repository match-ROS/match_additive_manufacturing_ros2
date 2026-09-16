"""Manually launched visualization of the selected coupled tracking index."""
from copy import deepcopy

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Int32
from ur_trajectory_follower.increment_path_index import (
    paths_have_same_trajectory, resample_coupled_paths,
)


class IndexPosePreview(Node):
    def __init__(self):
        super().__init__('index_pose_preview')
        self.declare_parameter('arm_path_topic', '/ur_path_transformed')
        self.declare_parameter('base_path_topic', '/base_path')
        self.declare_parameter('initial_path_index', 0)
        self.index = max(0, int(self.get_parameter('initial_path_index').value))
        self.sources = {'arm': None, 'base': None}
        self.paths = None
        self.qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                              reliability=ReliabilityPolicy.RELIABLE)
        self.publishers_by_part = {}
        for part in ('arm', 'base'):
            self.publishers_by_part[part] = self.create_publisher(
                PoseStamped, f'/selected_{part}_index_pose', self.qos)
            self.create_subscription(
                Path, str(self.get_parameter(f'{part}_path_topic').value),
                lambda msg, part=part: self._path_cb(part, msg), self.qos)
        self.create_subscription(Int32, '/path_index_command', self._index_cb, 10)
        self.create_subscription(Int32, '/path_index', self._index_cb, self.qos)
        self.create_timer(0.5, self._publish)

    def _path_cb(self, part, msg):
        if paths_have_same_trajectory(self.sources[part], msg):
            return
        self.sources[part] = msg
        self.paths = None
        arm, base = self.sources['arm'], self.sources['base']
        if arm is None or base is None or not arm.poses or not base.poses:
            return
        try:
            # Use exactly the same index space as the GUI's Path Index node.
            self.paths = resample_coupled_paths(arm, base, 0.005)
        except ValueError as error:
            self.get_logger().warn(f'Cannot preview coupled paths: {error}')
            return
        self._publish()

    def _index_cb(self, msg):
        self.index = max(0, msg.data)
        self._publish()

    def _publish(self):
        if self.paths is None:
            return
        if any(self.index >= len(path.poses) for path in self.paths):
            return
        for part, path in zip(('arm', 'base'), self.paths):
            pose = deepcopy(path.poses[self.index])
            pose.header.frame_id = path.header.frame_id
            pose.header.stamp = self.get_clock().now().to_msg()
            self.publishers_by_part[part].publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = IndexPosePreview()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
