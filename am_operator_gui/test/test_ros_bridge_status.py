from geometry_msgs.msg import PoseStamped

from am_operator_gui.ros_bridge import OperatorGuiNode


class _ClockThatMustNotBeRead:
    def now(self):
        raise AssertionError('pose validation must not compare clocks')


def test_map_pose_from_simulation_clock_is_accepted_on_receipt() -> None:
    node = object.__new__(OperatorGuiNode)
    node.get_clock = lambda: _ClockThatMustNotBeRead()
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp.sec = 42
    pose.pose.orientation.w = 1.0

    assert node._is_fresh_map_pose(pose)


def test_pose_validation_rejects_wrong_frame_or_missing_stamp() -> None:
    node = object.__new__(OperatorGuiNode)
    pose = PoseStamped()
    pose.header.frame_id = 'odom'
    pose.header.stamp.sec = 42
    assert not node._is_fresh_map_pose(pose)

    pose.header.frame_id = 'map'
    pose.header.stamp.sec = 0
    assert not node._is_fresh_map_pose(pose)
