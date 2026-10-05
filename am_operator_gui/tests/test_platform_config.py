"""Persisted and submitted profiles must work across status and command paths."""

import json

import pytest

from am_operator_gui.operator_service import OperatorService


def test_stale_simulation_profile_recovers_to_robotnik_hardware(tmp_path):
    path = tmp_path / 'config.json'
    path.write_text(json.dumps({'platform': 'mur620_sim', 'simulation': True,
                                'control_frame': 'vicon_world'}))

    service = OperatorService(config_path=path)
    state = service.snapshot()

    assert state['config']['platform'] == 'robotnik'
    assert state['config']['simulation'] is False
    assert state['config']['battery_topic'] == '/robot/battery_estimator/data'
    assert service._use_sim_time() == 'false'
    saved = json.loads(path.read_text())
    assert saved['platform'] == 'robotnik'
    assert saved['simulation'] is False
    assert saved['control_frame'] == 'vicon_world'


@pytest.mark.parametrize('values', [
    {'platform': 'mur620_sim'},
    {'platform': 'robotnik_hw'},
    {'simulation': 'false'},
])
def test_invalid_settings_do_not_change_hardware_selection(tmp_path, values):
    path = tmp_path / 'config.json'
    path.write_text(json.dumps({'platform': 'robotnik', 'simulation': False}))
    service = OperatorService(config_path=path)
    before = path.read_text()

    with pytest.raises(ValueError, match='Invalid setting'):
        service.update_config(values)

    assert path.read_text() == before
    assert service.snapshot()['config']['platform'] == 'robotnik'
    assert service.snapshot()['config']['simulation'] is False


@pytest.mark.parametrize('platform', ['robotnik', 'bunker'])
def test_valid_profile_preserves_explicit_simulation_mode(tmp_path, platform):
    path = tmp_path / 'config.json'
    service = OperatorService(config_path=path)
    service.update_config({'platform': f' {platform.upper()} ', 'simulation': True})

    reloaded = OperatorService(config_path=path)
    assert reloaded.snapshot()['config']['platform'] == platform
    assert reloaded.snapshot()['config']['simulation'] is True
