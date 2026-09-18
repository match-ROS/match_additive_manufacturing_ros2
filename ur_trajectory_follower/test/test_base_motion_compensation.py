"""Numerical and ROS-chain regression tests for moving-base compensation."""
import math
import os
import time

import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool, Float32

from ur_trajectory_follower.base_motion_compensation import BaseMotionCompensation, world_compensation
from ur_trajectory_follower.combine_twists import TwistCombiner
from ur_trajectory_follower.transform_twist_stamped import TransformTwistStamped


def transform(parent, child, xyz=(0., 0., 0.), yaw=0.):
    t = TransformStamped()
    t.header.frame_id, t.child_frame_id = parent, child
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
    t.transform.rotation.z = math.sin(yaw / 2)
    t.transform.rotation.w = math.cos(yaw / 2)
    return t


def linear(t):
    return np.array([t.linear.x, t.linear.y, t.linear.z])


@pytest.fixture
def node(request):
    os.environ['ROS_DOMAIN_ID'] = '181'
    os.environ['ROS_LOG_DIR'] = '/tmp/base_compensation_test_logs'
    rclpy.init(args=['--ros-args',
                     '-p', 'world_frame:=world', '-p', 'tcp_frame:=tip',
                     '-p', 'base_frame:=base', '-p', 'fixed_tool_offset_xyz:=[0.2, 0.0, 0.0]',
                     '-p', 'spray_distance_initial:=0.1',
                     '-p', "twist_topics:='[/tracking, /ur_twist_base_compensation_world]'",
                     '-p', 'combined_twist_topic:=/combined', '-p', 'output_stamped:=true',
                     '-p', 'frame_id:=world', '-p', 'input_topic:=/combined',
                     '-p', 'transform_twist_stamped:output_topic:=/relative', '-p', 'target_frame:=command'])
    n = BaseMotionCompensation(parameter_overrides=[
        Parameter('pose_source', value=getattr(request, 'param', 'tf')),
        Parameter('controller_tcp_frame', value='controller_tcp'),
    ])
    n.tf_buffer.set_transform_static(transform('world', 'base'), 'test')
    n.tf_buffer.set_transform_static(transform('base', 'tip', (1., 0., 0.)), 'test')
    yield n
    n.destroy_node()
    rclpy.shutdown()


def odom(node, vx=0., wz=0., child='base'):
    msg = Odometry()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.child_frame_id = child
    msg.twist.twist.linear.x = vx
    msg.twist.twist.angular.z = wz
    return msg


def test_world_rotation_and_translation():
    r = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
    v, w = world_compensation(np.array([.075, 0., 0.]), np.array([0., 0., .2]),
                              np.array([1., 0., 0.]), r)
    np.testing.assert_allclose(v, [.2, -.075, 0.])
    np.testing.assert_allclose(w, [0., 0., -.2])


@pytest.mark.parametrize('node', ['tcp_pose'], indirect=True)
def test_translation_backup_without_tcp_or_tool_tf(node, monkeypatch):
    node.tf_buffer.clear()
    monkeypatch.setattr(node, '_lookup', lambda *args: pytest.fail('Unexpected TF dependency'))
    node._translation_only_cb(Bool(data=True))
    node._base_pose_cb(pose(node, 'world', yaw=math.pi / 2))
    node._odom_cb(odom(node, vx=.075, wz=.2))
    node.spray_distance = float('nan')
    result = node._compute()
    np.testing.assert_allclose(linear(result), [0., -.075, 0.], atol=1e-12)
    assert result.angular == Twist().angular
    node._odom_cb(odom(node, wz=.2))
    assert node._compute() == Twist()
    node.last_velocity_time -= Duration(seconds=.6)
    assert node._compute() is None
    node._odom_cb(odom(node, vx=.075))
    node.base_pose_received -= 2_000_000_000
    assert node._compute() is None


@pytest.mark.parametrize('node', ['tcp_pose'], indirect=True)
def test_translation_backup_live_toggle_resets_rotation(node):
    setup_topic_geometry(node)
    node._odom_cb(odom(node, wz=.2))
    node._publish()
    assert node.previous_output.angular.z == -.2
    node.smoothing_coeff = .9
    node._translation_only_cb(Bool(data=True))
    assert node.previous_output == Twist()
    node.tcp_pose = node.tcp_pose_received = None
    node._publish()
    assert node.previous_output == Twist()
    assert node.connection_state == 'ready'
    node._translation_only_cb(Bool(data=False))
    assert node.cached_geometry is None
    node._publish()
    assert node.previous_output == Twist()
    assert node.connection_state != 'ready'


def test_translation_backup_tf_needs_no_arm(node):
    node.tf_buffer.clear()
    node.tf_buffer.set_transform_static(transform('world', 'base', yaw=math.pi / 2), 'test')
    node._translation_only_cb(Bool(data=True))
    node._odom_cb(odom(node, vx=.075, wz=.2))
    np.testing.assert_allclose(linear(node._compute()), [0., -.075, 0.], atol=1e-12)


def test_translation_backup_switch_over_ros(node):
    publisher_node = Node('backup_gui_test')
    publisher = publisher_node.create_publisher(
        Bool, '/am/base_compensation/translation_only',
        QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL))
    try:
        for enabled in (True, False):
            publisher.publish(Bool(data=enabled))
            deadline = time.monotonic() + 5.
            while node.translation_only != enabled and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.05)
            assert node.translation_only == enabled
    finally:
        publisher_node.destroy_node()


def test_deposition_lever_arm_and_velocity_origin(node):
    msg = odom(node, wz=.2)
    node._odom_cb(msg)
    result = node._compute()
    np.testing.assert_allclose(linear(result), [0., -.24, 0.], atol=1e-12)
    assert result.angular.z == -.2
    # Same rigid-body velocity measured at another origin: v_child = omega x offset.
    node.tf_buffer.set_transform_static(transform('base', 'sensor', (.5, 0., 0.)), 'test')
    msg.child_frame_id = 'sensor'
    msg.twist.twist.linear.y = .1
    node._odom_cb(msg)
    np.testing.assert_allclose(linear(node._compute()), linear(result), atol=1e-12)
    # Roll changes the spray lever arm in the angular-x case.
    node.tool_rotation = np.array([[1., 0., 0.], [0., 0., -1.], [0., 1., 0.]])
    msg = odom(node)
    msg.twist.twist.angular.x = .2
    node._odom_cb(msg)
    node._distance_cb(Float32(data=.3))
    np.testing.assert_allclose(linear(node._compute()), [0., 0., .06], atol=1e-12)


def test_invalid_stale_missing_inputs_and_smoothing_reset(node):
    assert node._compute() is None
    msg = odom(node, vx=.075)
    node._odom_cb(msg)
    assert node._compute() is not None
    node.last_velocity_time -= Duration(seconds=1.)
    assert node._compute() is None
    msg.header.stamp = (node.get_clock().now() - Duration(seconds=1.)).to_msg()
    node._odom_cb(msg)
    assert node._compute() is None
    msg = odom(node, vx=float('nan'))
    node._odom_cb(msg)
    assert node._compute() is None
    msg = odom(node, child='missing')
    node._odom_cb(msg)
    assert node._compute() is None
    node.previous_output.linear.x = 1.
    node._publish()
    assert node.previous_output == Twist()
    node._odom_cb(odom(node))
    np.testing.assert_allclose(linear(node._compute()), 0.)


def test_ros_chain_and_expired_publisher(node):
    """Best-effort odometry -> compensation -> combiner -> rotated arm command."""
    combiner, rotation = TwistCombiner(), TransformTwistStamped()
    rotation.buffer.set_transform_static(transform('world', 'command', yaw=math.pi / 2), 'test')
    harness = Node('compensation_harness', use_global_arguments=False)
    executor = SingleThreadedExecutor()
    for n in (node, combiner, rotation, harness):
        executor.add_node(n)
    base_pub = harness.create_publisher(Odometry, '/odom', qos_profile_sensor_data)
    tracking_pub = harness.create_publisher(Twist, '/tracking', 10)
    commands = []
    harness.create_subscription(TwistStamped, '/relative', lambda m: commands.append(m.twist), 10)

    def run(vx, wz, tracking_x, expected, timeout=3.):
        commands.clear()
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if vx is not None:
                base_pub.publish(odom(node, vx=vx, wz=wz))
            t = Twist()
            t.linear.x = tracking_x
            tracking_pub.publish(t)
            executor.spin_once(timeout_sec=.01)
            if commands and np.allclose(linear(commands[-1]), expected, atol=1e-6):
                return commands[-1]
        pytest.fail(f'Expected {expected}, got {linear(commands[-1]) if commands else "no output"}')

    try:
        # Allow discovery and receipt of odometry before accepting zero as cancellation.
        run(.075, 0., 0., [0., .075, 0.])
        run(.075, 0., .075, [0., 0., 0.])
        result = run(0., .2, 0., [-.24, 0., 0.])
        assert result.angular.z == pytest.approx(-.2)
        # Simulate a crashed publisher: no final zero message, combiner must expire it.
        for timer in list(node.timers):
            timer.cancel()
        run(None, 0., 0., [0., 0., 0.])
    finally:
        executor.shutdown()
        for n in (harness, rotation, combiner):
            n.destroy_node()


def test_stale_dynamic_tf_and_invalid_distance(node):
    old = transform('world', 'moving_sensor')
    old.header.stamp = (node.get_clock().now() - Duration(seconds=node.tf_timeout + .1)).to_msg()
    node.tf_buffer.set_transform(old, 'test')
    node._odom_cb(odom(node, child='moving_sensor'))
    assert node._compute() is None
    node._odom_cb(odom(node))
    node._distance_cb(Float32(data=float('nan')))
    assert node._compute() is None
    node._distance_cb(Float32(data=.1))
    assert node._compute() is not None


def test_pause_does_not_scale_measured_base_motion(node):
    # Override is intentionally absent from this node's inputs.
    node._odom_cb(odom(node, vx=.075))
    np.testing.assert_allclose(linear(node._compute()), [-.075, 0., 0.])
    assert not node.has_parameter('velocity_override')


def test_stamped_and_unstamped_velocity_sources(node):
    stamped = TwistStamped()
    stamped.header.frame_id = 'base'
    stamped.header.stamp = node.get_clock().now().to_msg()
    stamped.twist.linear.x = .075
    node._stamped_cb(stamped)
    np.testing.assert_allclose(linear(node._compute()), [-.075, 0., 0.])
    node._twist_cb(stamped.twist)
    assert node.velocity_stamp is None
    np.testing.assert_allclose(linear(node._compute()), [-.075, 0., 0.])


def pose(node, frame, xyz=(0., 0., 0.), yaw=0.):
    msg = PoseStamped()
    msg.header.frame_id = frame
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = xyz
    msg.pose.orientation.z = math.sin(yaw / 2)
    msg.pose.orientation.w = math.cos(yaw / 2)
    return msg


def setup_topic_geometry(node):
    node.tf_buffer.set_transform_static(transform('base', 'ur_base', (.3, .1, .5), math.pi), 'test')
    # The controller TCP is already the nozzle: applying the .2m tool offset
    # again would give the wrong yaw-induced compensation.
    node.tf_buffer.set_transform_static(transform('tip', 'controller_tcp', (.2, 0., 0.)), 'test')
    node._tcp_pose_cb(pose(node, 'ur_base', (1., 0., 0.)))
    node._base_pose_cb(pose(node, 'world', yaw=math.pi / 2))


@pytest.mark.parametrize('node', ['tcp_pose'], indirect=True)
def test_topic_geometry_freezes_mount_and_avoids_double_tool_offset(node, monkeypatch):
    setup_topic_geometry(node)
    node._odom_cb(odom(node, wz=.2))
    # TCP at (-.7,.1,.5) in base; world is rotated +90 degrees.
    np.testing.assert_allclose(linear(node._compute()), [-.14, .02, 0.], atol=1e-12)
    monkeypatch.setattr(node, '_lookup', lambda *_: pytest.fail('TF lookup after mounting capture'))
    node.tf_buffer.clear()
    node._tcp_pose_cb(pose(node, 'ur_base', (2., 0., 0.)))
    np.testing.assert_allclose(linear(node._compute()), [-.34, .02, 0.], atol=1e-12)
    node._tcp_pose_cb(pose(node, 'unexpected_frame', (2., 0., 0.)))
    assert node._compute() is None


@pytest.mark.parametrize('node', ['tcp_pose'], indirect=True)
def test_topic_geometry_tool_conversion_and_spray_axis(node):
    setup_topic_geometry(node)
    # Configured nozzle differs from the controller TCP by +.2m along tool x.
    node.tool_xyz = np.array([.4, 0., 0.])
    node.tool_rotation = np.array([[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]])
    node._base_pose_cb(pose(node, 'world'))
    node._odom_cb(odom(node, wz=.2))
    # Base nozzle x=.3-1-.2=-.9; spray adds -.1 on base x.
    np.testing.assert_allclose(linear(node._compute()), [.02, .2, 0.], atol=1e-12)
    node._distance_cb(Float32(data=.3))
    np.testing.assert_allclose(linear(node._compute()), [.02, .24, 0.], atol=1e-9)


@pytest.mark.parametrize('node', ['tcp_pose'], indirect=True)
def test_topic_geometry_uses_velocity_origin(node):
    setup_topic_geometry(node)
    node.tf_buffer.set_transform_static(transform('base', 'sensor', (.5, 0., 0.)), 'test')
    msg = odom(node, wz=.2, child='sensor')
    msg.twist.twist.linear.y = .1
    node._odom_cb(msg)
    np.testing.assert_allclose(linear(node._compute()), [-.14, .02, 0.], atol=1e-12)


@pytest.mark.parametrize('node', ['tcp_pose'], indirect=True)
def test_topic_geometry_rejects_missing_stale_and_wrong_frame_inputs(node):
    node._odom_cb(odom(node, wz=.2))
    assert node._compute() is None
    setup_topic_geometry(node)
    assert node._compute() is not None
    node.tcp_pose_received -= 2_000_000_000
    assert node._compute() is None
    node._tcp_pose_cb(pose(node, 'ur_base'))
    node.base_pose.header.stamp = (node.get_clock().now() - Duration(seconds=node.pose_timeout + .1)).to_msg()
    assert node._compute() is None
    node._base_pose_cb(pose(node, 'wrong_world'))
    assert node._compute() is None
    node._base_pose_cb(pose(node, 'world'))
    node.tcp_pose.pose.orientation.w = 0.
    assert node._compute() is None


@pytest.mark.parametrize('node', ['tcp_pose'], indirect=True)
def test_topic_pose_ros_subscriptions_without_tf_after_capture(node):
    setup_topic_geometry(node)
    node._odom_cb(odom(node))
    assert node._compute() is not None
    node.tf_buffer.clear()
    harness = Node('tcp_pose_compensation_harness', use_global_arguments=False)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(harness)
    tcp_pub = harness.create_publisher(PoseStamped, node.tcp_pose_topic, qos_profile_sensor_data)
    base_pose_pub = harness.create_publisher(PoseStamped, node.base_pose_topic, qos_profile_sensor_data)
    odom_pub = harness.create_publisher(Odometry, '/odom', qos_profile_sensor_data)
    commands = []
    harness.create_subscription(Twist, '/ur_twist_base_compensation_world', commands.append, 10)
    try:
        deadline = time.monotonic() + 3.
        while time.monotonic() < deadline:
            tcp_pub.publish(pose(node, 'ur_base', (2., 0., 0.)))
            base_pose_pub.publish(pose(node, 'world', yaw=math.pi / 2))
            odom_pub.publish(odom(node, wz=.2))
            executor.spin_once(timeout_sec=.01)
            if commands and np.allclose(linear(commands[-1]), [-.34, .02, 0.], atol=1e-6):
                break
        else:
            pytest.fail('No compensation from live pose topics')
    finally:
        executor.shutdown()
        harness.destroy_node()


@pytest.mark.parametrize('node', ['tcp_pose'], indirect=True)
def test_delayed_discovery_recovers_after_startup_grace(node):
    node.started_at -= 20_000_000_000
    node._publish()
    assert node.connection_state == 'connecting'
    assert node.previous_output == Twist()
    node.started_at -= 26_000_000_000
    node._publish()
    assert node.connection_state == 'unavailable'
    setup_topic_geometry(node)
    node._odom_cb(odom(node, wz=.2))
    node._publish()
    assert node.connection_state == 'ready'
    np.testing.assert_allclose(linear(node.previous_output), [-.14, .02, 0.], atol=1e-12)


@pytest.mark.parametrize('node', ['tcp_pose'], indirect=True)
def test_bounded_pose_hold_uses_fresh_velocity_and_recovers(node):
    setup_topic_geometry(node)
    node._odom_cb(odom(node, wz=.2))
    node._publish()
    # Reproduce the measured ~0.92s TCP receive gap. Geometry is retained,
    # but the command is recomputed from the current measured velocity.
    for attribute in ('tcp_pose', 'base_pose'):
        msg = getattr(node, attribute)
        msg.header.stamp = (node.get_clock().now() - Duration(seconds=.95)).to_msg()
        setattr(node, attribute + '_received', node.get_clock().now().nanoseconds - 950_000_000)
    node._odom_cb(odom(node, wz=.1))
    node._publish()
    np.testing.assert_allclose(linear(node.previous_output), [-.07, .01, 0.], atol=1e-12)
    assert node.connection_state == 'ready'
    node._odom_cb(odom(node))
    node._publish()
    assert node.previous_output == Twist()
    # A longer pose outage expires; no indefinite stale output, even with smoothing.
    node.smoothing_coeff = .9
    node.previous_output.linear.x = 1.
    node.tcp_pose.header.stamp = (node.get_clock().now() - Duration(seconds=1.6)).to_msg()
    node._publish()
    assert node.connection_state == 'unavailable'
    assert node.previous_output == Twist()
    node._tcp_pose_cb(pose(node, 'ur_base', (1., 0., 0.)))
    node._base_pose_cb(pose(node, 'world', yaw=math.pi / 2))
    node._odom_cb(odom(node, wz=.2))
    node._publish()
    assert node.connection_state == 'ready'
    # Velocity freshness remains strict even though pose age may reach 1.5s.
    node.last_velocity_time -= Duration(seconds=.6)
    node._publish()
    assert node.connection_state == 'unavailable'
    assert node.previous_output == Twist()


@pytest.mark.parametrize('node', ['tcp_pose'], indirect=True)
def test_late_duplicate_and_invalid_poses_do_not_replace_last_valid(node):
    setup_topic_geometry(node)
    latest = pose(node, 'ur_base', (2., 0., 0.))
    latest.header.stamp = (node.get_clock().now() - Duration(seconds=.1)).to_msg()
    # Start a fresh history, then accept one valid sample.
    node.tcp_pose = node.tcp_pose_received = None
    node._tcp_pose_cb(latest)
    received = node.tcp_pose_received
    late = pose(node, 'ur_base', (100., 0., 0.))
    late.header.stamp = (node.get_clock().now() - Duration(seconds=.9)).to_msg()
    node._tcp_pose_cb(late)
    assert node.tcp_pose is latest
    node._tcp_pose_cb(latest)
    assert node.tcp_pose_received == received
    invalid = pose(node, 'ur_base')
    invalid.pose.orientation.w = 0.
    node._tcp_pose_cb(invalid)
    assert node.tcp_pose is latest
    assert node.tcp_pose_received == received
