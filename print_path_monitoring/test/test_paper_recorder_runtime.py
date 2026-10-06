"""Exercise a complete session, including rosbag finalization on GUI SIGTERM."""

import json
import os
import signal
import subprocess
import sys
import time

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String

from ur_trajectory_follower.increment_path_index import IncrementPathIndex
from print_path_monitoring.paper_accuracy_report import build_report

TEST_DOMAIN_ID = int(os.environ.get('AM_PAPER_TEST_DOMAIN_ID', '211'))


@pytest.mark.timeout(30)
def test_session_records_atomic_references_and_finalizes_raw_bag(tmp_path, monkeypatch):
    # Fast DDS caches discovery configuration across rclpy.init/shutdown.
    # Run this cross-process test in a fresh interpreter, even in a larger
    # suite that has already initialized middleware on another domain.
    if os.environ.get('AM_PAPER_RUNTIME_TEST_CHILD') != '1':
        profile = tmp_path / 'loopback_dds.xml'
        profile.write_text('''<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors><transport_descriptor>
    <transport_id>test_loopback_udp</transport_id><type>UDPv4</type>
    <interfaceWhiteList><address>127.0.0.1</address></interfaceWhiteList>
  </transport_descriptor></transport_descriptors>
  <participant profile_name="test_loopback" is_default_profile="true"><rtps>
    <useBuiltinTransports>false</useBuiltinTransports>
    <userTransports><transport_id>test_loopback_udp</transport_id></userTransports>
    <builtin><initialPeersList><locator><udpv4><address>127.0.0.1</address></udpv4></locator></initialPeersList></builtin>
  </rtps></participant>
</profiles>
''')
        environment = os.environ.copy()
        environment.update(AM_PAPER_RUNTIME_TEST_CHILD='1', ROS_DOMAIN_ID=str(TEST_DOMAIN_ID),
                           ROS_AUTOMATIC_DISCOVERY_RANGE='SUBNET', ROS_STATIC_PEERS='127.0.0.1',
                           FASTDDS_BUILTIN_TRANSPORTS='UDPv4',
                           FASTRTPS_DEFAULT_PROFILES_FILE=str(profile),
                           FASTDDS_DEFAULT_PROFILES_FILE=str(profile))
        result = subprocess.run([sys.executable, '-m', 'pytest', '-q',
                                 f'{__file__}::test_session_records_atomic_references_and_finalizes_raw_bag'],
                                env=environment, capture_output=True, text=True, timeout=25)
        assert result.returncode == 0, result.stdout + result.stderr
        return
    monkeypatch.setenv('ROS_DOMAIN_ID', str(TEST_DOMAIN_ID))
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'logs'))
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'SUBNET')
    monkeypatch.setenv('ROS_STATIC_PEERS', '127.0.0.1')
    monkeypatch.setenv('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4')
    directory = tmp_path / 'trial'
    directory.mkdir()
    (directory / 'manifest.json').write_text(json.dumps({
        'schema_version': 1, 'session_id': 'runtime_test', 'status': 'prepared', 'additional_topics': [],
        'settings': {}, 'platform': 'robotnik', 'simulation': False,
        'control_frame': 'map', 'condition': 'test_baseline', 'initial_path_index': 0,
    }))
    rclpy.init(args=[], domain_id=TEST_DOMAIN_ID)
    publisher = Node('paper_runtime_publisher', use_global_arguments=False)
    # Use the real state serialization with a lightweight publisher context.
    source = object.__new__(IncrementPathIndex)
    source._trajectory_valid = True
    source.path_index = 0
    source.phase = 0.0
    source.completed = False
    source.start_enabled = False
    source.velocity_override = 1.0
    source.desired_arm_speed = 0.04
    source.progress_mode = 'desired_speed'
    source.get_clock = publisher.get_clock
    source.state_pub = publisher.create_publisher(String, '/trajectory_state', 50)
    latch = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=QoSReliabilityPolicy.RELIABLE)
    path = Path()
    path.header.frame_id = 'map'
    for i in range(3):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp.sec = i + 1
        pose.pose.position.x = i * 0.01
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)
    source.path = source.base_path = path
    path_publishers = [publisher.create_publisher(Path, topic, latch)
                       for topic in ('/base_path_tracking', '/ur_path_tracking')]
    poses = [publisher.create_publisher(PoseStamped, topic, 10)
             for topic in ('/robot_pose', '/measured_deposition_pose')]
    command = [sys.executable, '-m', 'print_path_monitoring.paper_accuracy_recorder', '--ros-args',
               '-p', f'session_directory:={directory}']
    with (directory / 'process.log').open('w') as logfile:
        process = subprocess.Popen(command, stdout=logfile, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic() + 12
            while time.monotonic() < deadline:
                assert process.poll() is None, (directory / 'process.log').read_text()
                for pub in path_publishers:
                    pub.publish(path)
                source._publish_measurement_state()
                rclpy.spin_once(publisher, timeout_sec=0.02)
                if source.state_pub.get_subscription_count() >= 3 and all(pub.get_subscription_count() >= 2 for pub in poses):
                    break
            else:
                pytest.fail(f'recorder subscriptions not ready: domain={publisher.context.get_domain_id()}, '
                            f'state={source.state_pub.get_subscription_count()}, '
                            f'poses={[p.get_subscription_count() for p in poses]}, '
                            f'nodes={publisher.get_node_names_and_namespaces()}; '
                            + (directory / 'process.log').read_text())
            source.start_enabled = True
            for step in range(35):
                source.path_index = min(2, step // 10)
                source.phase = 0.0 if source.path_index == 2 else (step % 10) / 10
                source.completed = source.path_index == 2
                source._publish_measurement_state()
                measured = PoseStamped()
                measured.header.frame_id = 'map'
                measured.header.stamp = publisher.get_clock().now().to_msg()
                measured.pose.position.x = (source.path_index + source.phase) * 0.01 - 0.001
                measured.pose.orientation.w = 1.0
                for pub in poses:
                    pub.publish(measured)
                rclpy.spin_once(publisher, timeout_sec=0.02)
            source._publish_measurement_state()
            rclpy.spin_once(publisher, timeout_sec=0.1)
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=8)
            assert process.returncode == 0, (directory / 'process.log').read_text()
            manifest = json.loads((directory / 'manifest.json').read_text())
            assert manifest['status'] == 'finished'
            assert manifest['bag_metadata_available']
            assert manifest['bag_required_topics_present']
            assert manifest['bag_topic_message_counts']['/trajectory_state'] >= 10
            for mode in ('base', 'tcp'):
                summary = json.loads((directory / f'{mode}.json').read_text())
                assert summary['windows']['active_tracking']['samples'] >= 5
                assert summary['windows']['settling']['samples'] >= 5
                assert summary['windows']['active_tracking']['absolute_error']['rmse'] == pytest.approx(0.001, abs=0.001)
                assert summary['reached_path_end']
                assert (directory / f'{mode}.csv').is_file()
            report = build_report(tmp_path)
            assert not report['excluded']
            assert len(report['groups']) == 2
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            publisher.destroy_node()
            rclpy.shutdown()
