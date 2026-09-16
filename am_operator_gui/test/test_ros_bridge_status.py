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

    assert node._is_fresh_control_pose(pose)


def test_pose_validation_rejects_wrong_frame_or_missing_stamp() -> None:
    node = object.__new__(OperatorGuiNode)
    pose = PoseStamped()
    pose.header.frame_id = 'odom'
    pose.header.stamp.sec = 42
    assert not node._is_fresh_control_pose(pose)

    pose.header.frame_id = 'map'
    pose.header.stamp.sec = 0
    assert not node._is_fresh_control_pose(pose)


def test_readiness_uses_configured_vicon_frame_for_pose_and_path() -> None:
    from nav_msgs.msg import Path
    node = object.__new__(OperatorGuiNode)
    node._control_frame = 'vicon_world'
    pose = PoseStamped()
    pose.header.frame_id = 'vicon_world'
    pose.header.stamp.sec = 42
    pose.pose.orientation.w = 1.0
    path = Path()
    path.header = pose.header
    path.poses = [pose]
    assert node._is_fresh_control_pose(pose)
    assert node._is_control_path(path)
    node._control_frame = 'map'
    assert not node._is_fresh_control_pose(pose)
    assert not node._is_control_path(path)


def test_one_hz_paths_stay_ready_between_publications() -> None:
    from types import SimpleNamespace
    from rclpy.time import Time
    node = object.__new__(OperatorGuiNode)
    node._last_base_path_time = Time(seconds=10)
    node._last_arm_path_time = Time(seconds=10)
    node._last_robot_pose_time = None
    node._last_arm_pose_time = None
    node._last_jparse_ready_time = None
    node._last_controller_ready_time = None
    node.get_clock = lambda: SimpleNamespace(now=lambda: Time(seconds=11))
    node._emit_status = lambda: None
    node._freshness_tick()
    assert node._has_path
    node.get_clock = lambda: SimpleNamespace(now=lambda: Time(seconds=13))
    node._freshness_tick()
    assert not node._has_path


def test_move_start_distances_use_tracking_paths_and_fresh_poses():
    import threading
    import pytest
    from copy import deepcopy
    from types import SimpleNamespace
    from nav_msgs.msg import Path
    from rclpy.time import Time

    node = object.__new__(OperatorGuiNode)
    node._control_frame = 'vicon_world'
    node._latest_pose_lock = threading.Lock()
    node.get_clock = lambda: SimpleNamespace(now=lambda: Time(seconds=10))
    pose = PoseStamped()
    pose.header.frame_id = 'vicon_world'
    pose.header.stamp.sec = 1
    target = deepcopy(pose)
    target.pose.position.x = 0.03
    target.pose.position.y = 0.04
    target.pose.position.z = 0.12
    path = Path()
    path.header = pose.header
    path.poses = [pose, target]
    node._latest_tracking_base_path = path
    node._latest_tracking_arm_path = path
    node._latest_robot_pose = pose
    node._latest_arm_pose = pose
    node._last_robot_pose_time = Time(seconds=10)
    node._last_arm_pose_time = Time(seconds=10)
    assert node.move_start_distances_cm(1) == pytest.approx({'base': 5, 'arm': 13})
    assert node.move_start_distances_cm(0) == {'base': 0, 'arm': 0}
    assert node.move_start_distances_cm(99) == {'base': 5, 'arm': None}
    node.get_clock = lambda: SimpleNamespace(now=lambda: Time(seconds=11))
    assert node.move_start_distances_cm(1) == {'base': None, 'arm': None}
    node.get_clock = lambda: SimpleNamespace(now=lambda: Time(seconds=10))
    node._control_frame = 'map'
    assert node.move_start_distances_cm(1) == {'base': None, 'arm': None}
