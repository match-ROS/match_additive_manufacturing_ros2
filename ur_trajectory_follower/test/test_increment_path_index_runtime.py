import os
import time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, Float32, Int32

from ur_trajectory_follower.increment_path_index import IncrementPathIndex


LATCH_QOS = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
)


def _make_path(length: int) -> Path:
    path = Path()
    path.header.frame_id = 'map'
    for index in range(length):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp.sec = index
        pose.pose.position.x = float(index) * 0.01
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)
    return path


def _spin_for(executor: SingleThreadedExecutor, duration: float) -> None:
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)


@pytest.mark.timeout(20)
def test_increment_path_index_advances_monotonically_and_scales_with_velocity_override() -> None:
    os.environ['ROS_DOMAIN_ID'] = '176'
    os.environ['ROS_LOG_DIR'] = '/tmp/ur_trajectory_follower_test_logs'
    rclpy.init(args=[
        '--ros-args',
        '-p', 'path_index_topic:=/test_path_index',
        '-p', 'next_goal_topic:=/test_next_goal',
        '-p', 'normal_topic:=/test_normal',
        '-p', 'initial_path_index:=0',
        '-p', 'path_topic:=/test_path',
        '-p', 'publish_rate:=20.0',
        '-p', 'velocity_override_topic:=/test_velocity_override',
        '-p', 'start_condition_topic:=/test_start_condition',
        '-p', 'wait_for_start_condition:=true',
    ])
    node = Node('increment_path_index_runtime_test', use_global_arguments=False)
    advancer = IncrementPathIndex()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(advancer)

    path_pub = node.create_publisher(Path, '/test_path', LATCH_QOS)
    start_pub = node.create_publisher(Bool, '/test_start_condition', 10)
    velocity_pub = node.create_publisher(Float32, '/test_velocity_override', 10)
    received = []
    node.create_subscription(
        Int32,
        '/test_path_index',
        lambda msg: received.append((time.monotonic(), int(msg.data))),
        LATCH_QOS,
    )

    path = _make_path(200)

    try:
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not received:
            path_pub.publish(path)
            executor.spin_once(timeout_sec=0.05)
        assert received, 'initial path index was never published'

        received.clear()
        start_pub.publish(Bool(data=True))
        _spin_for(executor, 0.6)
        full_speed = [index for _stamp, index in received]
        assert len(full_speed) >= 4
        assert all(curr > prev for prev, curr in zip(full_speed, full_speed[1:]))
        assert all((curr - prev) == 1 for prev, curr in zip(full_speed, full_speed[1:]))
        full_delta = full_speed[-1] - full_speed[0]

        received.clear()
        velocity_pub.publish(Float32(data=0.5))
        _spin_for(executor, 0.1)
        received.clear()
        start_pub.publish(Bool(data=True))
        _spin_for(executor, 0.6)
        half_speed = [index for _stamp, index in received]
        assert len(half_speed) >= 2
        assert all(curr > prev for prev, curr in zip(half_speed, half_speed[1:]))
        assert all((curr - prev) == 1 for prev, curr in zip(half_speed, half_speed[1:]))
        half_delta = half_speed[-1] - half_speed[0]

        assert half_delta < full_delta
        ratio = half_delta / full_delta
        assert 0.3 <= ratio <= 0.7
    finally:
        executor.remove_node(advancer)
        executor.remove_node(node)
        advancer.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.timeout(20)
def test_segment_progress_preserves_phase_when_desired_speed_changes_and_pauses() -> None:
    os.environ['ROS_DOMAIN_ID'] = '177'
    os.environ['ROS_LOG_DIR'] = '/tmp/ur_trajectory_follower_test_logs'
    rclpy.init(args=[
        '--ros-args',
        '-p', 'path_index_topic:=/segment_path_index',
        '-p', 'next_goal_topic:=/segment_next_goal',
        '-p', 'normal_topic:=/segment_normal',
        '-p', 'initial_path_index:=0',
        '-p', 'path_topic:=/segment_path',
        '-p', 'progress_mode:=desired_speed',
        '-p', 'desired_arm_speed:=0.1',
        '-p', 'control_rate:=100.0',
        '-p', 'velocity_override_topic:=/segment_velocity_override',
        '-p', 'start_condition_topic:=/segment_start',
        '-p', 'wait_for_start_condition:=true',
    ])
    node = Node('segment_progress_runtime_test', use_global_arguments=False)
    advancer = IncrementPathIndex()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(advancer)
    path_pub = node.create_publisher(Path, '/segment_path', LATCH_QOS)
    start_pub = node.create_publisher(Bool, '/segment_start', 10)
    override_pub = node.create_publisher(Float32, '/segment_velocity_override', 10)
    speed_pub = node.create_publisher(Float32, '/desired_arm_speed', LATCH_QOS)
    phases = []
    indices = []
    node.create_subscription(Float32, '/trajectory_phase', lambda msg: phases.append(float(msg.data)), LATCH_QOS)
    node.create_subscription(Int32, '/segment_path_index', lambda msg: indices.append(int(msg.data)), LATCH_QOS)

    path = _make_path(4)
    for pose in path.poses:
        pose.pose.position.x *= 2.0  # 2 cm segments: 0.2 s at 0.1 m/s.

    try:
        for _ in range(10):
            path_pub.publish(path)
            executor.spin_once(timeout_sec=0.02)
        start_pub.publish(Bool(data=True))
        _spin_for(executor, 0.12)
        assert phases and max(phases) > 0.2

        # The path publisher periodically refreshes only Path.header.stamp.
        # Receiving that identical trajectory must not reset segment phase.
        progress_before_republish = advancer.path_index + advancer.phase
        path.header.stamp.sec += 1
        path_pub.publish(path)
        _spin_for(executor, 0.03)
        assert advancer.path_index + advancer.phase >= progress_before_republish

        progress_before_change = advancer.path_index + advancer.phase
        speed_pub.publish(Float32(data=0.2))
        _spin_for(executor, 0.03)
        assert advancer.path_index + advancer.phase >= progress_before_change

        override_pub.publish(Float32(data=0.0))
        _spin_for(executor, 0.05)
        paused_progress = advancer.path_index + advancer.phase
        _spin_for(executor, 0.08)
        assert abs(advancer.path_index + advancer.phase - paused_progress) < 0.02
        assert indices and indices[0] == 0
    finally:
        executor.remove_node(advancer)
        executor.remove_node(node)
        advancer.destroy_node()
        node.destroy_node()
        rclpy.shutdown()
