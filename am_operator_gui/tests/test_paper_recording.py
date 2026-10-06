import json
from types import SimpleNamespace

import pytest

from am_operator_gui.operator_service import OperatorService


def test_paper_action_prepares_persistent_manifest_and_toggles_one_session(tmp_path, monkeypatch):
    config_path = tmp_path / 'config.json'
    service = OperatorService(config_path=config_path)
    service.update_config({'paper_output_directory': str(tmp_path / 'datasets'),
                           'paper_condition': 'compensation_on', 'paper_notes': 'calibration 2026-10-06',
                           'control_frame': 'vicon_world', 'platform': 'bunker'})
    calls = []
    running = {}

    def start(name, command, **kwargs):
        calls.append((name, command))
        running[name] = SimpleNamespace(command=command, is_running=lambda: True)

    monkeypatch.setattr(service.processes, 'start', start)
    monkeypatch.setattr(service.processes, 'get', running.get)
    monkeypatch.setattr(service.processes, 'stop', lambda name: running.pop(name))
    service.action('paper_accuracy')
    assert len(calls) == 1
    manifest_path = next((tmp_path / 'datasets').glob('*/manifest.json'))
    manifest = json.loads(manifest_path.read_text())
    assert manifest['status'] == 'prepared'
    assert manifest['condition'] == 'compensation_on'
    assert '/odom' in manifest['additional_topics']
    assert '/diff_drive_controller/cmd_vel' in manifest['additional_topics']
    assert 'required_frame:=vicon_world' in calls[0][1]
    assert 'paper_accuracy_recorder' in calls[0][1]
    service.action('paper_accuracy')
    assert not running
    assert len(calls) == 1
    assert 'paper_accuracy_report' in service.command_for('paper_report')


def test_tuning_uses_tracking_path_and_configured_frame(tmp_path):
    service = OperatorService(config_path=tmp_path / 'config.json')
    service.update_config({'control_frame': 'vicon_world'})
    command = service.command_for('base_accuracy')
    assert 'reference_path_topic:=/base_path_tracking' in command
    assert 'required_frame:=vicon_world' in command
    assert 'start_condition_topic:=/start_condition' in command


def test_empty_output_directory_rejected_before_mutating_settings(tmp_path):
    service = OperatorService(config_path=tmp_path / 'config.json')
    with pytest.raises(ValueError, match='output directory'):
        service.update_config({'paper_output_directory': ''})
    assert 'paper_output_directory' not in service.config


def test_measured_pose_adapter_preserves_timestamp_and_input_header():
    from geometry_msgs.msg import PoseStamped
    from am_operator_gui.pose_stamped_adapter import PoseStampedAdapter

    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp.sec = 12
    pose.pose.orientation.w = 1.0
    received = []
    adapter = SimpleNamespace(source='measured', target_frame='map', _publish=received.append)
    PoseStampedAdapter._pose_cb(adapter, pose)
    assert received[0].header.stamp.sec == 12
    received[0].header.stamp.sec = 99
    assert pose.header.stamp.sec == 12


def test_desktop_buttons_dispatch_actions_and_persist_dataset_settings(tmp_path, monkeypatch):
    monkeypatch.setenv('QT_QPA_PLATFORM', 'offscreen')
    pytest.importorskip('PyQt5')
    from PyQt5.QtWidgets import QApplication
    from am_operator_gui import gui

    app = QApplication.instance() or QApplication([])
    service = OperatorService(config_path=tmp_path / 'config.json')
    service.ros_bridge = SimpleNamespace(
        publish_velocity_override=lambda value: None, publish_spray_distance=lambda value: None,
        publish_start_condition=lambda value: None, publish_stop_commands=lambda frame: None,
        move_start_distances_cm=lambda index: {'base': None, 'arm': None}, stop=lambda: None)
    monkeypatch.setattr(service, 'ensure_ros', lambda: True)
    actions = []
    monkeypatch.setattr(service, 'action', actions.append)
    monkeypatch.setattr(gui, 'OperatorService', lambda **kwargs: service)
    window = gui.OperatorWindow()
    try:
        window.paper_output_directory.setText(str(tmp_path / 'campaign'))
        window.paper_condition.setText('tuned_speed_004')
        window.paper_notes.setText('calibration A')
        window.paper_accuracy_button.click()
        window.paper_report_button.click()
        assert actions == ['paper_accuracy', 'paper_report']
        assert service.config['paper_condition'] == 'tuned_speed_004'
        assert service.config['paper_notes'] == 'calibration A'
        assert service.config['paper_output_directory'] == str(tmp_path / 'campaign')
    finally:
        window.status_timer.stop()
        window.close()
        app.processEvents()


def test_web_dataset_actions_and_controls(tmp_path, monkeypatch):
    import asyncio
    pytest.importorskip('fastapi')
    from am_operator_gui.web_app import app

    service = OperatorService(config_path=tmp_path / 'config.json')
    actions = []
    monkeypatch.setattr(service, 'action', actions.append)
    monkeypatch.setattr(app.state, 'operator', service, raising=False)

    async def request(method, path):
        messages = []

        async def receive():
            return {'type': 'http.request', 'body': b'', 'more_body': False}

        async def send(message):
            messages.append(message)

        await app({'type': 'http', 'asgi': {'version': '3.0'}, 'http_version': '1.1',
                   'method': method, 'path': path, 'raw_path': path.encode(),
                   'query_string': b'', 'headers': [], 'scheme': 'http', 'root_path': '',
                   'server': ('test', 80), 'client': ('127.0.0.1', 1234)}, receive, send)
        status = next(m['status'] for m in messages if m['type'] == 'http.response.start')
        body = b''.join(m.get('body', b'') for m in messages if m['type'] == 'http.response.body')
        return status, body.decode()

    status, body = asyncio.run(request('GET', '/'))
    assert status == 200
    assert 'data-action="paper_accuracy"' in body
    assert 'data-action="paper_report"' in body
    assert asyncio.run(request('POST', '/api/actions/paper_accuracy'))[0] == 200
    assert asyncio.run(request('POST', '/api/actions/paper_report'))[0] == 200
    assert actions == ['paper_accuracy', 'paper_report']
