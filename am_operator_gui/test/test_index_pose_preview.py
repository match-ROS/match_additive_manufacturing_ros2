"""Check the preview uses the same coupled index space as Move to Start."""
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Int32

from am_operator_gui.index_pose_preview import IndexPosePreview
from ur_trajectory_follower.increment_path_index import resample_coupled_paths


def test_preview_matches_resampled_index_and_preserves_frames():
    rclpy.init()
    node = IndexPosePreview()
    outputs = {'arm': [], 'base': []}

    class Capture:
        def __init__(self, part):
            self.part = part

        def publish(self, msg):
            outputs[self.part].append(msg)

    node.publishers_by_part = {part: Capture(part) for part in outputs}
    try:
        arm, base = Path(), Path()
        arm.header.frame_id = base.header.frame_id = 'vicon_world'
        for path, length in ((arm, 0.02), (base, 0.04)):
            for index in range(2):
                pose = PoseStamped()
                pose.pose.position.x = index * length
                pose.pose.orientation.w = 1.0
                pose.header.stamp.sec = index
                path.poses.append(pose)
        node._path_cb('arm', arm)
        assert not outputs['arm']
        node._path_cb('base', base)
        node._index_cb(Int32(data=2))
        expected = resample_coupled_paths(arm, base, 0.005)
        for part, path in zip(('arm', 'base'), expected):
            assert outputs[part][-1].pose == path.poses[2].pose
            assert outputs[part][-1].header.frame_id == 'vicon_world'
        count = len(outputs['arm'])
        node._index_cb(Int32(data=10000))
        assert len(outputs['arm']) == count
        node._index_cb(Int32(data=0))
        assert outputs['arm'][-1].pose == arm.poses[0].pose
    finally:
        node.destroy_node()
        rclpy.shutdown()
