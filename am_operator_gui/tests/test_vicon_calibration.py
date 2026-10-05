"""Measured nozzle feedback and base reconstruction share one calibration."""
import json
from unittest.mock import Mock

import pytest

from am_operator_gui.operator_service import OperatorService


VISIBLE = {'xyz': [0.32, 0.088, 0.0027], 'quaternion_xyzw': [0.5, 0.5, 0.5, 0.5]}
LEGACY = {'xyz': [0.03, 0.02, -0.16], 'quaternion_xyzw': [0.0, 0.0, 0.0, 1.0]}


@pytest.mark.parametrize('has_visible', [True, False])
def test_old_calibration_is_migrated_and_persisted(tmp_path, has_visible):
    path = tmp_path / 'config.json'
    config = {'vicon_fallback_nozzle_transform': LEGACY, 'control_frame': 'vicon_world'}
    if has_visible:
        config['vicon_nozzle_transform'] = VISIBLE
    path.write_text(json.dumps(config))

    service = OperatorService(config_path=path)

    saved = json.loads(path.read_text())
    assert saved['vicon_nozzle_transform'] == (VISIBLE if has_visible else LEGACY)
    assert 'vicon_fallback_nozzle_transform' not in saved
    assert 'vicon_fallback_nozzle_transform' not in service.config
    assert saved['control_frame'] == 'vicon_world'


def test_save_reload_and_launch_use_shared_calibration(tmp_path):
    path = tmp_path / 'config.json'
    service = OperatorService(config_path=path)
    service.update_config({'vicon_nozzle_transform': VISIBLE,
                           'vicon_fallback_nozzle_transform': LEGACY})
    reloaded = OperatorService(config_path=path)
    reloaded.processes = Mock()

    reloaded.start_pose_adapters()

    commands = {call.args[0]: call.args[1] for call in reloaded.processes.start.call_args_list}
    for name in ('vicon_ee_static_tf', 'vicon_fallback_nozzle_tf'):
        command = commands[name]
        assert f"marker_to_nozzle_xyz:={VISIBLE['xyz']}" in command
        assert f"marker_to_nozzle_quaternion_xyzw:={VISIBLE['quaternion_xyzw']}" in command
    assert 'output_topic:=/vicon/tool_transformed' in commands['vicon_ee_static_tf']
    assert 'output_topic:=/vicon/nozzle_fallback' in commands['vicon_fallback_nozzle_tf']
    assert 'tcp_topic:=/vicon/nozzle_fallback' in commands['base_pose_adapter']
    assert 'vicon_fallback_nozzle_transform' not in json.loads(path.read_text())
