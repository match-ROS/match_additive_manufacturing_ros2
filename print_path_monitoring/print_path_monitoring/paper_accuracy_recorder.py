"""One process supervises the raw bag and both accuracy monitors for one trial."""

from __future__ import annotations

import json
import os
import signal
import subprocess
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
from pathlib import Path
from threading import Thread

import rclpy
import yaml
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions

from .trajectory_accuracy_monitor import TrajectoryAccuracyMonitor


RAW_TOPICS = (
    '/trajectory_state', '/robot_pose', '/current_deposition_pose', '/measured_deposition_pose',
    '/current_nozzle_tip_pose', '/current_tcp_pose', '/vicon/tool_transformed',
    '/base_path', '/ur_path_transformed', '/base_path_tracking', '/ur_path_tracking',
    '/arm_trajectory_reference', '/base_trajectory_reference', '/normal_vector',
    '/path_index', '/path_index_command', '/trajectory_phase', '/start_condition',
    '/start_pose_reached', '/trajectory_complete', '/desired_arm_speed', '/velocity_override',
    '/tf', '/tf_static', '/clock', '/robot/robot_description', '/robot/joint_states',
    '/robot/arm/tcp_pose_broadcaster/pose', '/ur_twist_world',
    '/ur_twist_base_compensation_world', '/ur_twist_combined_world',
    '/jparse_velocity_controller_ur/twist_cmd_world', '/jparse_velocity_controller_ur/twist_cmd',
    '/robot/arm/forward_velocity_controller/commands',
    '/robot/arm_forward_velocity_controller/commands', '/am/base_compensation/translation_only',
    '/spray_distance', '/spray_distance_smoothed',
    '/am/tcp_pose/source', '/am/base_pose_ready', '/am/arm_pose_ready',
    '/parameter_events',
)

REQUIRED_BAG_TOPICS = ('/trajectory_state', '/robot_pose', '/measured_deposition_pose',
                       '/base_path_tracking', '/ur_path_tracking')


def inspect_bag(directory):
    try:
        metadata = yaml.safe_load((directory / 'raw' / 'metadata.yaml').read_text())
        topics = {entry['topic_metadata']['name']: int(entry['message_count'])
                  for entry in metadata['rosbag2_bagfile_information']['topics_with_message_count']}
        missing = [name for name in REQUIRED_BAG_TOPICS if topics.get(name, 0) == 0]
        return {'bag_metadata_available': True, 'bag_topic_message_counts': topics,
                'bag_required_topics_present': not missing, 'bag_missing_required_topics': missing}
    except (OSError, ValueError, TypeError, KeyError, yaml.YAMLError) as exc:
        return {'bag_metadata_available': False, 'bag_required_topics_present': False,
                'bag_metadata_error': str(exc)}


def bag_command(directory, topics, use_sim_time):
    command = ['ros2', 'bag', 'record', '--output', str(directory / 'raw'),
               '--disable-keyboard-controls',
               '--qos-profile-overrides-path', str(directory / 'bag_qos.yaml')]
    if use_sim_time:
        command.append('--use-sim-time')
    return command + ['--topics', *sorted(set(topics))]


def capture_runtime_parameters(directory):
    """Capture the values on live nodes, including defaults omitted by the GUI."""
    destination = directory / 'runtime_parameters'
    destination.mkdir(exist_ok=True)
    try:
        discovery = subprocess.run(['ros2', 'node', 'list', '--no-daemon', '--spin-time', '1'],
                                   capture_output=True, text=True, timeout=5)
        names = sorted(set(line.strip() for line in discovery.stdout.splitlines() if line.startswith('/')))
    except (OSError, subprocess.TimeoutExpired) as exc:
        (destination / 'errors.json').write_text(json.dumps({'discovery': str(exc)}) + '\n')
        return

    def dump(name):
        try:
            result = subprocess.run(['ros2', 'param', 'dump', '--no-daemon', '--spin-time', '0.5', name],
                                    capture_output=True, text=True, timeout=5)
            key = name.strip('/').replace('/', '__')
            if result.returncode:
                return name, result.stderr or result.stdout
            (destination / f'{key}.yaml').write_text(result.stdout, encoding='utf-8')
            return name, None
        except (OSError, subprocess.TimeoutExpired) as exc:
            return name, str(exc)

    with ThreadPoolExecutor(max_workers=8) as pool:
        errors = {name: error for name, error in pool.map(dump, names) if error}
    (destination / 'errors.json').write_text(json.dumps(errors, indent=2) + '\n')


def main(args=None):
    # Keep the ROS context alive until the bag and summaries have been flushed.
    # The default rclpy signal handler shuts it down asynchronously, which can
    # race executor wait-set creation and turn a normal stop into an RCLError.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stop_requested = False

    def request_stop(signum, frame):
        nonlocal stop_requested
        stop_requested = True

    previous_handlers = {signum: signal.signal(signum, request_stop)
                         for signum in (signal.SIGINT, signal.SIGTERM)}
    supervisor = Node('paper_accuracy_recorder')
    supervisor.declare_parameter('session_directory', '')
    supervisor.declare_parameter('required_frame', 'map')
    supervisor.declare_parameter('phase', 'baseline')
    supervisor.declare_parameter('max_reference_age', 0.1)
    executor = SingleThreadedExecutor()
    monitors = []
    bag = None
    manifest = None
    manifest_path = None
    parameter_thread = None
    try:
        directory = Path(str(supervisor.get_parameter('session_directory').value)).expanduser()
        manifest_path = directory / 'manifest.json'
        manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
        if manifest.get('status') != 'prepared':
            raise ValueError('a fresh prepared session is required')
        topics = list(RAW_TOPICS) + manifest['additional_topics']
        # Sensor subscriptions must work with reliable and best-effort sources;
        # paths/static TF must be retained when recording begins after launch.
        latched = ('/tf_static', '/robot/robot_description', '/base_path', '/ur_path_transformed',
                   '/base_path_tracking', '/ur_path_tracking', '/arm_trajectory_reference',
                   '/base_trajectory_reference', '/path_index', '/desired_arm_speed', '/trajectory_complete')
        qos = '\n'.join(f'{topic}:\n  reliability: reliable\n  durability: transient_local\n  history: keep_last\n  depth: 1'
                        for topic in latched)
        qos += '\n' + '\n'.join(f'{topic}:\n  reliability: best_effort\n  durability: volatile\n  history: keep_last\n  depth: 100'
                               for topic in sorted(set(topics) - set(latched))) + '\n'
        (directory / 'bag_qos.yaml').write_text(qos, encoding='utf-8')
        use_sim_time = bool(supervisor.get_parameter('use_sim_time').value)
        command = bag_command(directory, topics, use_sim_time)
        manifest['bag_command'] = command
        # A separate group lets the supervisor flush monitors and terminate the
        # bag with SIGINT even when the GUI sends SIGTERM to its process group.
        bag = subprocess.Popen(command, start_new_session=True)
        for mode in ('base', 'tcp'):
            parameters = {
                'mode': mode, 'phase': str(supervisor.get_parameter('phase').value),
                'required_frame': str(supervisor.get_parameter('required_frame').value),
                'use_sim_time': use_sim_time,
                'output_directory': str(directory), 'run_name': mode,
                'session_id': manifest['session_id'],
                'actual_pose_topic': '/robot_pose' if mode == 'base' else '/measured_deposition_pose',
                'reference_path_topic': '/base_path_tracking' if mode == 'base' else '/ur_path_tracking',
                'base_reference_path_topic': '/base_path_tracking',
                'trajectory_state_topic': '/trajectory_state', 'start_condition_topic': '/start_condition',
                'max_reference_age': float(supervisor.get_parameter('max_reference_age').value),
                'evaluate_reachability': False,
                'command_twist_topic': '' if mode == 'base' else '/ur_twist_world',
                'joint_states_topic': '/robot/joint_states' if mode == 'tcp' else '',
            }
            monitor = TrajectoryAccuracyMonitor(
                node_name=f'paper_accuracy_{mode}', use_global_arguments=False,
                parameter_overrides=[Parameter(name, value=value) for name, value in parameters.items()])
            monitors.append(monitor)
            executor.add_node(monitor)
        manifest.update(status='recording', started_utc=datetime.now(timezone.utc).isoformat())
        manifest_path.write_text(json.dumps(manifest, indent=2) + '\n', encoding='utf-8')
        parameter_thread = Thread(target=capture_runtime_parameters, args=(directory,), daemon=True)
        parameter_thread.start()
        supervisor.get_logger().info(f'Paper dataset: {directory}; waiting for trajectory state and sensor poses')

        def check_bag():
            if bag.poll() is not None:
                raise RuntimeError(f'raw bag recorder exited unexpectedly with code {bag.returncode}')

        supervisor.create_timer(0.5, check_bag)
        executor.add_node(supervisor)
        while rclpy.ok() and not stop_requested:
            executor.spin_once(timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        if manifest is not None:
            manifest['error'] = str(exc)
        raise
    finally:
        if bag is not None and bag.poll() is None:
            os.killpg(bag.pid, signal.SIGINT)
            try:
                bag.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(bag.pid, signal.SIGTERM)
                try:
                    bag.wait(timeout=1)
                except subprocess.TimeoutExpired:
                    os.killpg(bag.pid, signal.SIGKILL)
                    bag.wait()
        results = {}
        for monitor in monitors:
            try:
                monitor.write_summary()
                results[monitor.mode] = json.loads(monitor.summary_path.read_text())
            finally:
                monitor.csv_file.close()
                monitor.destroy_node()
        if manifest is not None and manifest_path is not None:
            manifest.update(inspect_bag(manifest_path.parent))
            if not manifest['bag_required_topics_present'] and not manifest.get('error'):
                manifest['error'] = 'Raw bag is missing required measurement topics; inspect bag metadata.'
            manifest.update(status='failed' if manifest.get('error') else 'finished',
                            stopped_utc=datetime.now(timezone.utc).isoformat(),
                            bag_return_code=bag.returncode if bag else None,
                            runtime_parameters_complete=parameter_thread is not None and not parameter_thread.is_alive(),
                            measurements={mode: {'samples': result['samples'],
                                                 'path_hashes': result['path_hashes'],
                                                 'reached_path_end': result['reached_path_end']}
                                          for mode, result in results.items()})
            manifest_path.write_text(json.dumps(manifest, indent=2) + '\n', encoding='utf-8')
        executor.shutdown()
        supervisor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)


if __name__ == '__main__':
    main()
