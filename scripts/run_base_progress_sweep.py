#!/usr/bin/env python3
"""Run the isolated coupled base-progress simulation matrix and summarize it.

The script intentionally leaves the arm/coordinator launch untouched.  It only
starts an isolated simulator, publishes the requested velocity override, and
records separate base and deposition/TCP monitor summaries for each case.
"""
from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Any


SPEEDS = (0.05, 0.10, 0.15)
OVERRIDES = (0.20, 0.50, 1.00, 1.50, 2.00, 2.50)
START_X = 50.868916
START_Y = 43.615944


def shell_environment(underlay: Path, local_prefixes: list[Path]) -> dict[str, str]:
    """Source the simulator underlay and put this workspace's packages first."""
    command = (
        'source /opt/ros/jazzy/setup.bash; '
        f'source {underlay / "setup.bash"}; '
        'env -0'
    )
    raw = subprocess.check_output(['bash', '-lc', command])
    environment = {
        item.split('=', 1)[0]: item.split('=', 1)[1]
        for item in raw.decode().split('\0') if '=' in item
    }
    prefix_text = ':'.join(str(prefix) for prefix in local_prefixes)
    environment['AMENT_PREFIX_PATH'] = prefix_text + ':' + environment.get('AMENT_PREFIX_PATH', '')
    python_paths: list[Path] = []
    for prefix in local_prefixes:
        # A supplied overlay can be a standalone temporary install prefix, so
        # use its installed Python package directly instead of assuming the
        # workspace's build-directory layout.
        python_paths.extend(prefix.glob('lib/python*/site-packages'))
        build_path = prefix.parents[1] / 'build' / prefix.name
        if build_path.exists():
            python_paths.append(build_path)
    environment['PYTHONPATH'] = ':'.join(str(path) for path in python_paths) + ':' + environment.get('PYTHONPATH', '')
    return environment


def start(command: list[str], environment: dict[str, str], log_path: Path) -> subprocess.Popen:
    log = log_path.open('w', encoding='utf-8')
    return subprocess.Popen(
        command,
        env=environment,
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )


def stop(process: subprocess.Popen | None) -> None:
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=20)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        try:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=10)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            os.killpg(process.pid, signal.SIGKILL)


def read_json(path: Path) -> dict[str, Any] | None:
    try:
        return json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError):
        return None


def case_name(speed: float, override: float) -> str:
    return f'speed_{speed:.2f}_override_{int(round(override * 100)):03d}'


def summarize_case(
    name: str, directory: Path, timeout: bool, infrastructure_failure: str | None = None,
) -> dict[str, Any]:
    base = read_json(directory / f'{name}_base.json')
    tcp = read_json(directory / f'{name}_tcp.json')
    tcp_max = None if tcp is None else tcp.get('absolute_error', {}).get('max')
    saturation = None if base is None else base.get('command_twist', {}).get('saturation_time_fraction')
    lag = None if base is None else base.get('base_progress', {}).get('arc_length_error_m', {}).get('maximum_signed')
    result: dict[str, Any] = {
        'case': name,
        'base_summary': str(directory / f'{name}_base.json') if base else None,
        'tcp_summary': str(directory / f'{name}_tcp.json') if tcp else None,
        'tcp_max_error_m': tcp_max,
        'progress_lag_m': lag,
        'base_saturation_time_fraction': saturation,
        'timed_out': timeout,
        'pass': bool(tcp_max is not None and float(tcp_max) <= 0.025),
    }
    if infrastructure_failure:
        result['failure'] = infrastructure_failure
        result['infrastructure_failure'] = True
    elif tcp_max is None:
        result['failure'] = 'missing TCP monitor summary'
    elif float(tcp_max) > 0.025:
        result['failure'] = 'TCP/deposition maximum exceeds 25 mm'
        result['feasibility_limit'] = bool(saturation is not None and float(saturation) > 0.0)
    return result


def launch_failure(path: Path) -> str | None:
    """Detect startup faults which make an empty monitor result meaningless."""
    try:
        tail = path.read_bytes()[-131072:].decode(errors='replace')
    except OSError:
        return None
    markers = {
        'Failed to activate controller': 'controller activation failed',
        "Waiting for data on 'robot_description' topic to finish initialization": (
            'Gazebo ros2_control did not initialize from robot_description'
        ),
        'process has died': 'a required launch process exited',
    }
    for marker, description in markers.items():
        if marker in tail:
            return description
    return None


def run_case(args: argparse.Namespace, environment: dict[str, str], speed: float, override: float) -> dict[str, Any]:
    name = case_name(speed, override)
    directory = args.output_directory / name
    directory.mkdir(parents=True, exist_ok=True)
    case_environment = environment.copy()
    case_environment['ROS_DOMAIN_ID'] = str(args.ros_domain_id)
    case_environment['GZ_PARTITION'] = f'base_progress_{name}'
    case_environment['ROS_LOG_DIR'] = str(directory / 'ros_logs')

    base_monitor = [
        'ros2', 'run', 'print_path_monitoring', 'trajectory_accuracy_monitor', '--ros-args',
        '-p', 'mode:=base', '-p', 'actual_pose_topic:=/robot_pose',
        '-p', 'reference_path_topic:=/base_path_tracking',
        '-p', 'reference_pose_topic:=/base_trajectory_reference',
        '-p', 'command_twist_topic:=/robot/robotnik_base_control/cmd_vel_unstamped',
        '-p', 'max_tracking_linear_velocity:=1.0', '-p', 'start_condition_topic:=/start_condition',
        '-p', f'output_directory:={directory}', '-p', f'run_name:={name}_base',
        '-p', 'shutdown_after_completion:=true',
    ]
    tcp_monitor = [
        'ros2', 'run', 'print_path_monitoring', 'trajectory_accuracy_monitor', '--ros-args',
        '-p', 'mode:=tcp', '-p', 'actual_pose_topic:=/current_deposition_pose',
        '-p', 'reference_path_topic:=/ur_path_tracking',
        '-p', 'reference_pose_topic:=/arm_trajectory_reference',
        '-p', 'base_reference_path_topic:=/base_path_tracking',
        '-p', 'arm_base_offset:=[0.26,0.0,1.046]', '-p', 'start_condition_topic:=/start_condition',
        '-p', f'output_directory:={directory}', '-p', f'run_name:={name}_tcp',
        '-p', 'shutdown_after_completion:=true',
    ]
    override_publisher = [
        'ros2', 'topic', 'pub', '-r', '20', '/velocity_override', 'std_msgs/msg/Float32',
        f'{{data: {override:.6f}}}',
    ]
    launch = [
        'ros2', 'launch', str(args.launch_file), 'launch_sim:=true', 'gui:=false',
        'use_exported_trajectories:=true', 'generate_test_paths:=false',
        f'trajectory_directory:={args.trajectory_directory}',
        f'sim_initial_x:={START_X}', f'sim_initial_y:={START_Y}', 'sim_initial_z:=0.1',
        f'desired_arm_speed:={speed:.6f}', f'max_vx:={args.max_vx:.6f}',
        f'pure_pursuit_k_progress:={args.progress_gain:.6f}',
        f'max_progress_speed_correction:={args.max_progress_correction:.6f}',
        'external_path_index_stride:=1',
    ]
    if args.dry_run:
        return {'case': name, 'commands': {'launch': launch, 'base_monitor': base_monitor, 'tcp_monitor': tcp_monitor}}

    processes: list[subprocess.Popen] = []
    timed_out = False
    infrastructure_failure = None
    try:
        processes.append(start(base_monitor, case_environment, directory / 'base_monitor.log'))
        processes.append(start(tcp_monitor, case_environment, directory / 'tcp_monitor.log'))
        processes.append(start(override_publisher, case_environment, directory / 'override.log'))
        processes.append(start(launch, case_environment, directory / 'launch.log'))
        deadline = time.monotonic() + args.case_timeout
        while time.monotonic() < deadline:
            if (directory / f'{name}_base.json').exists() and (directory / f'{name}_tcp.json').exists():
                break
            infrastructure_failure = launch_failure(directory / 'launch.log')
            if infrastructure_failure:
                break
            if processes[-1].poll() is not None:
                infrastructure_failure = 'paired launch exited before monitor completion'
                break
            time.sleep(1.0)
        else:
            timed_out = True
    finally:
        for process in reversed(processes):
            stop(process)
    return summarize_case(name, directory, timed_out, infrastructure_failure)


def main() -> int:
    repository = Path(__file__).resolve().parents[1]
    workspace = repository.parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=Path, default=workspace)
    parser.add_argument('--underlay', type=Path, default=Path('/home/rosmatch/workspaces/wattle_daub_ros2_ws/install'))
    parser.add_argument(
        '--overlay-prefix', type=Path, action='append', default=[],
        help='Installed package prefix to place before the workspace packages (repeatable).',
    )
    parser.add_argument('--trajectory-directory', type=Path, default=repository / 'components' / 'doubleCurvedTElement')
    parser.add_argument('--output-directory', type=Path, default=Path('/tmp/base_progress_sweep'))
    parser.add_argument('--ros-domain-id', type=int, default=223)
    parser.add_argument('--case-timeout', type=float, default=3600.0)
    parser.add_argument('--progress-gain', type=float, default=1.0)
    parser.add_argument('--max-progress-correction', type=float, default=0.5)
    parser.add_argument('--max-vx', type=float, default=1.0)
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('--speed', type=float, action='append', choices=SPEEDS)
    parser.add_argument('--override', type=float, action='append', choices=OVERRIDES)
    args = parser.parse_args()
    args.workspace = args.workspace.resolve()
    args.launch_file = args.workspace / 'src' / 'match_additive_manufacturing_ros2' / 'am_bringup' / 'launch' / 'rbvogui_paired_base_arm_demo.launch.py'
    args.output_directory = args.output_directory.resolve()
    local_packages = ('am_bringup', 'base_trajectory_follower', 'print_path_monitoring', 'ur_trajectory_follower', 'move_to_path_idx', 'parse_paths')
    local_prefixes = [prefix.resolve() for prefix in args.overlay_prefix]
    local_prefixes.extend(args.workspace / 'install' / package for package in local_packages)
    environment = shell_environment(args.underlay.resolve(), local_prefixes)
    results = []
    for speed in args.speed or SPEEDS:
        for override in args.override or OVERRIDES:
            result = run_case(args, environment, speed, override)
            results.append(result)
            args.output_directory.mkdir(parents=True, exist_ok=True)
            (args.output_directory / 'campaign_results.json').write_text(
                json.dumps(results, indent=2, sort_keys=True) + '\n', encoding='utf-8')
            print(json.dumps(result, sort_keys=True), flush=True)
    if args.dry_run:
        return 0
    return 0 if all(result.get('pass', False) for result in results) else 1


if __name__ == '__main__':
    sys.exit(main())
