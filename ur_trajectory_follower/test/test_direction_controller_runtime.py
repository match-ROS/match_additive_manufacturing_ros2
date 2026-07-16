import os
import time

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Float32, Int32

from ur_trajectory_follower.ur_path_direction_controller import DirectionController


LATCH_QOS = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
)


def _pose(x: float, stamp: float = 0.0) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp.sec = int(stamp)
    pose.header.stamp.nanosec = int((stamp - int(stamp)) * 1e9)
    pose.pose.position.x = x
    pose.pose.orientation.w = 1.0
    return pose


def _spin_until(executor: SingleThreadedExecutor, predicate, timeout: float = 2.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)
        if predicate():
            return True
    return False


@pytest.mark.timeout(20)
def test_direction_controller_combines_feedforward_and_holds_reference_on_pause() -> None:
    os.environ['ROS_DOMAIN_ID'] = '178'
    os.environ['ROS_LOG_DIR'] = '/tmp/ur_trajectory_follower_test_logs'
    rclpy.init(args=[
        '--ros-args',
        '-p', 'path_topic:=/tracking_path',
        '-p', 'reference_pose_topic:=/tracking_reference',
        '-p', 'current_pose_topic:=/tracking_current',
        '-p', 'path_index_topic:=/tracking_index',
        '-p', 'velocity_override_topic:=/tracking_override',
        '-p', 'desired_speed_topic:=/tracking_speed',
        '-p', 'wait_for_start_condition:=false',
        '-p', 'max_tracking_linear_velocity:=0.12',
        '-p', 'max_along_track_correction:=0.03',
        '-p', 'orthogonal_max_velocity:=0.02',
    ])
    harness = Node('direction_controller_runtime_test', use_global_arguments=False)
    controller = DirectionController()
    executor = SingleThreadedExecutor()
    executor.add_node(harness)
    executor.add_node(controller)
    path_pub = harness.create_publisher(Path, '/tracking_path', LATCH_QOS)
    reference_pub = harness.create_publisher(PoseStamped, '/tracking_reference', LATCH_QOS)
    current_pub = harness.create_publisher(PoseStamped, '/tracking_current', 10)
    index_pub = harness.create_publisher(Int32, '/tracking_index', LATCH_QOS)
    override_pub = harness.create_publisher(Float32, '/tracking_override', 10)
    speed_pub = harness.create_publisher(Float32, '/tracking_speed', LATCH_QOS)
    commands = []
    harness.create_subscription(Twist, '/ur_twist_world', lambda msg: commands.append(msg), 10)

    path = Path()
    path.header.frame_id = 'map'
    path.poses = [_pose(0.0, 0.0), _pose(1.0, 10.0)]
    try:
        path_pub.publish(path)
        index_pub.publish(Int32(data=0))
        speed_pub.publish(Float32(data=0.1))
        override_pub.publish(Float32(data=1.0))
        reference_pub.publish(_pose(0.5))
        current_pub.publish(_pose(0.0))
        assert _spin_until(executor, lambda: bool(commands))
        running = commands[-1]
        assert 0.1 < running.linear.x <= 0.12

        commands.clear()
        override_pub.publish(Float32(data=0.0))
        current_pub.publish(_pose(0.0))
        assert _spin_until(
            executor,
            lambda: any(command.linear.x < 0.05 for command in commands),
        )
        paused = next(command for command in reversed(commands) if command.linear.x < 0.05)
        assert paused.linear.x == pytest.approx(0.03, abs=1e-3)

        commands.clear()
        override_pub.publish(Float32(data=1.0))
        index_pub.publish(Int32(data=1))
        reference_pub.publish(_pose(1.0))
        current_pub.publish(_pose(0.95))
        assert _spin_until(executor, lambda: any(command.linear.x >= 0.029 for command in commands))
        endpoint = next(command for command in reversed(commands) if command.linear.x >= 0.029)
        assert endpoint.linear.x == pytest.approx(0.03, abs=1e-3)
    finally:
        executor.remove_node(controller)
        executor.remove_node(harness)
        controller.destroy_node()
        harness.destroy_node()
        rclpy.shutdown()
