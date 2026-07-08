import os
import time

from am_operator_gui.external_base_reference import ExternalBaseReference
from am_operator_gui.odometry_robot_pose import OdometryRobotPose
from am_operator_gui.pose_stamped_adapter import PoseStampedAdapter
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


@pytest.mark.timeout(20)
def test_pose_is_transformed_and_readiness_is_published() -> None:
    os.environ['ROS_DOMAIN_ID'] = '173'
    os.environ['ROS_LOG_DIR'] = '/tmp/am_operator_gui_pose_test_logs'
    rclpy.init()
    node = Node('pose_adapter_runtime_test')
    adapter = PoseStampedAdapter(parameter_overrides=[
        Parameter('input_topic', value='/test_pose_input'),
        Parameter('output_topic', value='/test_pose_output'),
        Parameter('target_frame', value='target'),
        Parameter('ready_topic', value='/test_pose_ready'),
        Parameter('stale_timeout', value=0.5),
    ])
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(adapter)

    pose_pub = node.create_publisher(PoseStamped, '/test_pose_input', 10)
    outputs = []
    readiness = []
    ready_qos = QoSProfile(
        depth=1,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        reliability=QoSReliabilityPolicy.RELIABLE,
    )
    node.create_subscription(
        PoseStamped,
        '/test_pose_output',
        lambda msg: outputs.append(msg),
        10,
    )
    node.create_subscription(
        Bool,
        '/test_pose_ready',
        lambda msg: readiness.append(msg.data),
        ready_qos,
    )

    transform = TransformStamped()
    transform.header.stamp = node.get_clock().now().to_msg()
    transform.header.frame_id = 'target'
    transform.child_frame_id = 'source'
    transform.transform.translation.x = 1.0
    transform.transform.rotation.w = 1.0
    broadcaster = StaticTransformBroadcaster(node)
    broadcaster.sendTransform(transform)

    try:
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not outputs:
            pose = PoseStamped()
            pose.header.stamp = node.get_clock().now().to_msg()
            pose.header.frame_id = 'source'
            pose.pose.position.x = 2.0
            pose.pose.orientation.w = 1.0
            pose_pub.publish(pose)
            executor.spin_once(timeout_sec=0.05)

        assert outputs
        assert outputs[-1].header.frame_id == 'target'
        assert outputs[-1].pose.position.x == pytest.approx(3.0)
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and True not in readiness:
            executor.spin_once(timeout_sec=0.05)
        assert True in readiness
    finally:
        executor.remove_node(adapter)
        executor.remove_node(node)
        adapter.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.timeout(20)
def test_external_base_reference_converts_marker_pose_to_base_pose() -> None:
    os.environ['ROS_DOMAIN_ID'] = '174'
    os.environ['ROS_LOG_DIR'] = '/tmp/am_operator_gui_pose_test_logs'
    rclpy.init()
    node = Node('external_base_reference_runtime_test')
    adapter = ExternalBaseReference(parameter_overrides=[
        Parameter('input_topic', value='/test_base_marker_input'),
        Parameter('input_pose_frame', value='base_marker'),
        Parameter('output_topic', value='/test_robot_pose_output'),
        Parameter('map_frame', value='map'),
        Parameter('robot_base_frame', value='base'),
        Parameter('robot_tree_root_frame', value=''),
        Parameter('ready_topic', value='/test_base_pose_ready'),
        Parameter('stale_timeout', value=0.5),
    ])
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(adapter)

    pose_pub = node.create_publisher(PoseStamped, '/test_base_marker_input', 10)
    outputs = []
    node.create_subscription(
        PoseStamped,
        '/test_robot_pose_output',
        lambda msg: outputs.append(msg),
        10,
    )

    transform = TransformStamped()
    transform.header.stamp = node.get_clock().now().to_msg()
    transform.header.frame_id = 'base'
    transform.child_frame_id = 'base_marker'
    transform.transform.translation.x = 1.0
    transform.transform.rotation.w = 1.0
    broadcaster = StaticTransformBroadcaster(node)
    broadcaster.sendTransform(transform)

    try:
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not outputs:
            pose = PoseStamped()
            pose.header.stamp = node.get_clock().now().to_msg()
            pose.header.frame_id = 'map'
            pose.pose.position.x = 10.0
            pose.pose.orientation.w = 1.0
            pose_pub.publish(pose)
            executor.spin_once(timeout_sec=0.05)

        assert outputs
        assert outputs[-1].header.frame_id == 'map'
        assert outputs[-1].pose.position.x == pytest.approx(9.0)
    finally:
        executor.remove_node(adapter)
        executor.remove_node(node)
        adapter.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.timeout(20)
def test_odometry_robot_pose_anchors_odom_to_base_path_index() -> None:
    os.environ['ROS_DOMAIN_ID'] = '175'
    os.environ['ROS_LOG_DIR'] = '/tmp/am_operator_gui_pose_test_logs'
    rclpy.init()
    node = Node('odometry_robot_pose_runtime_test')
    adapter = OdometryRobotPose(parameter_overrides=[
        Parameter('odom_topic', value='/test_odom'),
        Parameter('path_topic', value='/test_base_path'),
        Parameter('output_topic', value='/test_robot_pose_from_odom'),
        Parameter('initial_path_index', value=1),
        Parameter('ready_topic', value='/test_odom_pose_ready'),
        Parameter('stale_timeout', value=0.5),
        Parameter('publish_tf', value=False),
    ])
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(adapter)

    path_pub = node.create_publisher(Path, '/test_base_path', 10)
    odom_pub = node.create_publisher(Odometry, '/test_odom', 10)
    outputs = []
    node.create_subscription(
        PoseStamped,
        '/test_robot_pose_from_odom',
        lambda msg: outputs.append(msg),
        10,
    )

    path = Path()
    path.header.frame_id = 'map'
    first = PoseStamped()
    first.header.frame_id = 'map'
    first.pose.orientation.w = 1.0
    second = PoseStamped()
    second.header.frame_id = 'map'
    second.pose.position.x = 10.0
    second.pose.position.y = 20.0
    second.pose.orientation.w = 1.0
    path.poses = [first, second]

    try:
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not outputs:
            path.header.stamp = node.get_clock().now().to_msg()
            path_pub.publish(path)

            odom = Odometry()
            odom.header.stamp = node.get_clock().now().to_msg()
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'
            odom.pose.pose.orientation.w = 1.0
            odom_pub.publish(odom)
            executor.spin_once(timeout_sec=0.05)

        assert outputs

        outputs.clear()
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            odom = Odometry()
            odom.header.stamp = node.get_clock().now().to_msg()
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'
            odom.pose.pose.position.x = 1.5
            odom.pose.pose.position.y = -2.0
            odom.pose.pose.orientation.w = 1.0
            odom_pub.publish(odom)
            executor.spin_once(timeout_sec=0.05)
            if outputs and outputs[-1].pose.position.x > 11.0:
                break

        assert outputs
        assert outputs[-1].header.frame_id == 'map'
        assert outputs[-1].pose.position.x == pytest.approx(11.5)
        assert outputs[-1].pose.position.y == pytest.approx(18.0)
    finally:
        executor.remove_node(adapter)
        executor.remove_node(node)
        adapter.destroy_node()
        node.destroy_node()
        rclpy.shutdown()
