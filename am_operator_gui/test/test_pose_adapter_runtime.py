import os
import time

from am_operator_gui.pose_stamped_adapter import PoseStampedAdapter
from geometry_msgs.msg import PoseStamped, TransformStamped
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
