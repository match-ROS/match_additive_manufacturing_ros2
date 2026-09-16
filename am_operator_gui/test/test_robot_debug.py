import json
import subprocess
from unittest.mock import patch

import pytest

from am_operator_gui.robot_debug import probe_robot, format_debug_info, RobotDebugInfo


def test_offset_excludes_remote_processing_and_reports_uncertainty():
    response = subprocess.CompletedProcess([], 0, json.dumps({
        'remote_start': 198.1, 'remote_end': 200.1,
        'configured_servers': '192.168.0.222', 'active_server': '192.168.0.222',
        'synchronized': False,
    }), '')
    with patch('am_operator_gui.robot_debug.subprocess.run', return_value=response), \
         patch('am_operator_gui.robot_debug.time.time', side_effect=[100, 102.2, 102.2]), \
         patch('am_operator_gui.robot_debug.time.monotonic', side_effect=[10, 12.2]):
        info = probe_robot()
    assert info['ssh_reachable'] is True
    assert info['offset_seconds'] == pytest.approx(98)
    assert info['uncertainty_seconds'] == pytest.approx(0.1)
    assert '192.168.0.222' in format_debug_info(info)
    assert 'synchronisiert: nein' in format_debug_info(info)


@pytest.mark.parametrize('failure', [
    subprocess.CompletedProcess([], 255, '', 'Permission denied'),
    subprocess.TimeoutExpired('ssh', 8),
])
def test_ssh_failure_never_keeps_a_previous_offset(failure):
    kwargs = {'side_effect': failure} if isinstance(failure, Exception) else {'return_value': failure}
    with patch('am_operator_gui.robot_debug.subprocess.run', **kwargs):
        info = probe_robot()
    assert info['ssh_reachable'] is False
    assert info['offset_seconds'] is None
    assert info['error']
    assert info['checked_at']


def test_remote_clock_jump_rejects_time_measurement():
    response = subprocess.CompletedProcess([], 0, '{"remote_start": 200, "remote_end": 100}', '')
    with patch('am_operator_gui.robot_debug.subprocess.run', return_value=response):
        info = probe_robot()
    assert info['ssh_reachable'] is True
    assert info['offset_seconds'] is None
    assert 'Clock changed' in info['error']


def test_remote_command_failure_still_reports_successful_ssh_login():
    response = subprocess.CompletedProcess([], 127, '', 'python3: not found')
    with patch('am_operator_gui.robot_debug.subprocess.run', return_value=response):
        info = probe_robot()
    assert info['ssh_reachable'] is True
    assert info['offset_seconds'] is None
    assert info['error'] == 'python3: not found'


def test_snapshot_is_lazy_and_starts_only_one_background_probe():
    with patch('am_operator_gui.robot_debug.Thread') as thread:
        debug = RobotDebugInfo()
        thread.assert_not_called()
        first = debug.snapshot()
        debug.snapshot()
        assert first['checking'] is True
        assert first['ssh_reachable'] is None
        assert thread.call_count == 1


def test_cached_result_does_not_start_new_probe():
    with patch('am_operator_gui.robot_debug.probe_robot', return_value={
        'target': 'robot@192.168.0.200', 'ssh_reachable': False,
    }), patch('am_operator_gui.robot_debug.Thread') as thread:
        debug = RobotDebugInfo()
        debug._refresh()
        assert debug.snapshot()['ssh_reachable'] is False
        thread.assert_not_called()
