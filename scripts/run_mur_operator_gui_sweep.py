#!/usr/bin/env python3
"""Run isolated MuR right-arm cases through the supported Operator GUI API."""

from __future__ import annotations

import argparse
import csv
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Any
from urllib.error import URLError
from urllib.request import Request, urlopen


SPEEDS = (0.05, 0.10, 0.15)
OVERRIDES = (0.20, 0.50, 1.00, 1.50, 2.00, 2.50)


def case_name(speed: float, override: float) -> str:
    return f'speed_{speed:.2f}_override_{int(round(override * 100)):03d}'


def request_json(url: str, path: str, method: str = 'GET', body: dict[str, Any] | None = None) -> dict[str, Any]:
    payload = None if body is None else json.dumps(body).encode('utf-8')
    request = Request(url + path, data=payload, method=method)
    if payload is not None:
        request.add_header('content-type', 'application/json')
    with urlopen(request, timeout=10) as response:  # nosec B310: loopback GUI only
        return json.loads(response.read().decode('utf-8'))


def start_server(environment: dict[str, str], port: int, log_path: Path) -> subprocess.Popen:
    env = environment.copy()
    env['AM_OPERATOR_WEB_NO_BROWSER'] = '1'
    env['AM_OPERATOR_WEB_PORT'] = str(port)
    log = log_path.open('w', encoding='utf-8')
    return subprocess.Popen(
        ['ros2', 'run', 'am_operator_gui', 'am_operator_web'],
        env=env,
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )


def stop_process(process: subprocess.Popen | None) -> None:
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=15)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        os.killpg(process.pid, signal.SIGTERM)


def wait_for_api(base_url: str, server: subprocess.Popen, timeout: float = 30.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if server.poll() is not None:
            raise RuntimeError('Operator GUI exited before its API became ready')
        try:
            request_json(base_url, '/api/state')
            return
        except (URLError, TimeoutError, json.JSONDecodeError):
            time.sleep(0.25)
    raise RuntimeError('Timed out waiting for Operator GUI API')


def latest_csv_row(path: Path) -> dict[str, str] | None:
    if not path.exists() or path.stat().st_size == 0:
        return None
    with path.open(newline='', encoding='utf-8') as stream:
        rows = csv.DictReader(stream)
        last = None
        for row in rows:
            last = row
        return last


def number(row: dict[str, str] | None, key: str) -> float | None:
    if row is None:
        return None
    value = row.get(key, '').strip()
    try:
        return float(value) if value else None
    except ValueError:
        return None


def wait_for_start_moves(base_url: str, timeout: float) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        state = request_json(base_url, '/api/state')
        processes = state.get('processes', {})
        base = processes.get('move_base', {})
        arm = processes.get('move_arm', {})
        if not base.get('running', False) and not arm.get('running', False):
            if base.get('return_code') == 0 and arm.get('return_code') == 0:
                return
            raise RuntimeError(f'move-to-start failed: base={base}, arm={arm}')
        time.sleep(1.0)
    raise RuntimeError('timed out waiting for MuR base/arm move-to-start')


def load_summary(path: Path) -> dict[str, Any] | None:
    try:
        return json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError):
        return None


def run_case(args: argparse.Namespace, base_url: str, speed: float, override: float) -> dict[str, Any]:
    name = case_name(speed, override)
    directory = args.output_directory / name
    directory.mkdir(parents=True, exist_ok=True)
    base_csv = directory / f'{name}_base.csv'
    base_summary = directory / f'{name}_base.json'
    tcp_summary = directory / f'{name}_tcp.json'
    request_json(base_url, '/api/actions/stop_all', 'POST')
    settings = {
        'platform': 'mur620_sim',
        'simulation': True,
        'simulation_gui': False,
        'mur_arm': 'r',
        'trajectory_directory': str(args.trajectory_directory),
        'path_index': args.path_index,
        'original_arm_index': args.path_index,
        'default_velocity_enabled': True,
        'default_velocity': speed,
        'velocity_override': override * 100.0,
        'monitor_output_directory': str(directory),
        'monitor_run_name': name,
        'pid_gains': {
            'base_follower.max_vx': args.max_vx,
            'base_follower.pure_pursuit_k_progress': args.progress_gain,
            'base_follower.max_progress_speed_correction': args.max_progress_correction,
            'base_follower.base_progress_xy_tolerance': args.progress_xy_tolerance,
            'base_follower.base_progress_yaw_tolerance': args.progress_yaw_tolerance,
        },
    }
    request_json(base_url, '/api/settings', 'PUT', {'values': settings})
    request_json(base_url, '/api/actions/launch_all', 'POST')
    early_failure = False
    try:
        wait_for_start_moves(base_url, args.start_timeout)
        request_json(base_url, '/api/actions/base_accuracy', 'POST')
        request_json(base_url, '/api/actions/tcp_accuracy', 'POST')
        # The GUI opens its start gate as a short, repeated pulse.  Give both
        # freshly launched monitor processes time to create their subscriptions
        # before asking it to start the coupled trajectory.
        time.sleep(args.monitor_ready_delay)
        request_json(base_url, '/api/actions/start_following', 'POST')
        deadline = time.monotonic() + args.case_timeout
        early_failure = False
        while time.monotonic() < deadline:
            state = request_json(base_url, '/api/state')
            processes = state.get('processes', {})
            failed = {
                key: value for key, value in processes.items()
                if key in {'simulation', 'base_follower', 'arm_follower', 'path_index', 'controllers'}
                and not value.get('running', False) and value.get('return_code') not in (None, 0)
            }
            if failed:
                raise RuntimeError(f'launch process failed: {failed}')
            row = latest_csv_row(base_csv)
            reference_index = number(row, 'path_index')
            lag = number(row, 'base_progress_error_m')
            if (reference_index is not None and lag is not None
                    and reference_index >= args.early_failure_min_index
                    and lag >= args.early_failure_lag_m):
                early_failure = True
                break
            if reference_index is not None and reference_index >= args.end_index:
                time.sleep(args.completion_grace)
                break
            time.sleep(1.0)
        else:
            early_failure = True
    finally:
        request_json(base_url, '/api/actions/stop_following', 'POST')
        request_json(base_url, '/api/actions/stop_all', 'POST')
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and (not base_summary.exists() or not tcp_summary.exists()):
            time.sleep(0.25)

    base = load_summary(base_summary)
    tcp = load_summary(tcp_summary)
    base_max = None if base is None else base.get('absolute_error', {}).get('max')
    tcp_max = None if tcp is None else tcp.get('absolute_error', {}).get('max')
    progress = None if base is None else base.get('base_progress', {}).get('arc_length_error_m', {}).get('maximum_signed')
    commands = None if base is None else base.get('command_twist', {})
    linear_sat = None if commands is None else commands.get('saturation_time_fraction')
    angular_sat = None if commands is None else commands.get('angular_saturation_time_fraction')
    result = {
        'case': name,
        'desired_speed_mps': speed,
        'velocity_override': override,
        'base_summary': str(base_summary) if base else None,
        'tcp_summary': str(tcp_summary) if tcp else None,
        'base_max_tracking_error_m': base_max,
        'tcp_max_error_m': tcp_max,
        'progress_lag_m': progress,
        'base_linear_saturation_time_fraction': linear_sat,
        'base_angular_saturation_time_fraction': angular_sat,
        'early_failure': early_failure,
        'pass': bool(tcp_max is not None and float(tcp_max) <= 0.025 and not early_failure),
    }
    if not result['pass']:
        result['failure'] = 'TCP/deposition maximum exceeds 25 mm or early feasibility stop'
        result['feasibility_limit'] = bool((linear_sat or 0.0) > 0.0 or (angular_sat or 0.0) > 0.0)
    return result


def main() -> int:
    repository = Path(__file__).resolve().parents[1]
    workspace = repository.parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=Path, default=workspace)
    parser.add_argument('--trajectory-directory', type=Path, default=repository / 'components' / 'doubleCurvedTElement')
    parser.add_argument('--output-directory', type=Path, default=Path('/tmp/mur_operator_gui_sweep'))
    parser.add_argument('--ros-domain-id', type=int, default=226)
    parser.add_argument('--web-port', type=int, default=8012)
    parser.add_argument('--path-index', type=int, default=100)
    parser.add_argument('--end-index', type=int, default=4986)
    parser.add_argument('--start-timeout', type=float, default=180.0)
    parser.add_argument('--case-timeout', type=float, default=3600.0)
    parser.add_argument('--completion-grace', type=float, default=5.0)
    parser.add_argument('--monitor-ready-delay', type=float, default=2.0)
    parser.add_argument('--early-failure-min-index', type=int, default=500)
    parser.add_argument('--early-failure-lag-m', type=float, default=2.0)
    parser.add_argument('--progress-gain', type=float, default=1.0)
    parser.add_argument('--max-progress-correction', type=float, default=0.5)
    parser.add_argument('--max-vx', type=float, default=1.0)
    parser.add_argument('--progress-xy-tolerance', type=float, default=0.05)
    parser.add_argument('--progress-yaw-tolerance', type=float, default=0.08)
    parser.add_argument('--speed', type=float, action='append', choices=SPEEDS)
    parser.add_argument('--override', type=float, action='append', choices=OVERRIDES)
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()
    args.workspace = args.workspace.resolve()
    args.trajectory_directory = args.trajectory_directory.resolve()
    args.output_directory.mkdir(parents=True, exist_ok=True)
    speeds = args.speed or SPEEDS
    overrides = args.override or OVERRIDES
    cases = [case_name(speed, override) for speed in speeds for override in overrides]
    if args.dry_run:
        print(json.dumps({'cases': cases, 'path_index': args.path_index, 'end_index': args.end_index}, indent=2))
        return 0

    environment = os.environ.copy()
    environment['ROS_DOMAIN_ID'] = str(args.ros_domain_id)
    # ROS domains do not isolate Gazebo Transport.  Without a dedicated
    # partition, an operator sweep can attach to a user's already-running
    # Gazebo world through shared /clock and /stats topics.
    environment['GZ_PARTITION'] = f'mur_operator_gui_sweep_{args.ros_domain_id}'
    base_url = f'http://127.0.0.1:{args.web_port}'
    server = start_server(environment, args.web_port, args.output_directory / 'operator_gui.log')
    results: list[dict[str, Any]] = []
    try:
        wait_for_api(base_url, server)
        for speed in speeds:
            for override in overrides:
                result = run_case(args, base_url, speed, override)
                results.append(result)
                print(json.dumps(result, sort_keys=True), flush=True)
    finally:
        try:
            request_json(base_url, '/api/actions/stop_all', 'POST')
        except (URLError, TimeoutError, json.JSONDecodeError):
            pass
        stop_process(server)
    campaign = {
        'trajectory_directory': str(args.trajectory_directory),
        'path_index': args.path_index,
        'settings': {
            'max_vx': args.max_vx,
            'progress_gain': args.progress_gain,
            'max_progress_correction': args.max_progress_correction,
        },
        'results': results,
    }
    (args.output_directory / 'campaign_results.json').write_text(
        json.dumps(campaign, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    return 0 if all(result['pass'] for result in results) else 1


if __name__ == '__main__':
    sys.exit(main())
