"""The Vicon action uses the GUI workspace instead of a separate overlay."""
from pathlib import Path

import pytest

from am_operator_gui import operator_service


@pytest.mark.parametrize('package_path', [
    'src/match_additive_manufacturing_ros2/am_operator_gui',
    'install/am_operator_gui/lib/python3.12/site-packages/am_operator_gui',
])
def test_vicon_sources_own_workspace(tmp_path, monkeypatch, package_path):
    workspace = tmp_path / 'workspace with spaces'
    (workspace / 'src').mkdir(parents=True)
    setup = workspace / 'install' / 'setup.bash'
    setup.parent.mkdir()
    setup.touch()
    # A stale build inside src must not be mistaken for the workspace.
    nested_setup = workspace / 'src' / 'install' / 'setup.bash'
    nested_setup.parent.mkdir()
    nested_setup.touch()
    old_setup = tmp_path / 'vicon_receiver_ws' / 'install' / 'setup.bash'
    old_setup.parent.mkdir(parents=True)
    old_setup.touch()
    monkeypatch.setattr(Path, 'home', lambda: tmp_path)
    monkeypatch.setattr(operator_service, 'PACKAGE_ROOT', workspace / package_path)
    service = operator_service.OperatorService(config_path=tmp_path / 'config.json')
    service.config['control_frame'] = 'vicon_world'

    command = service.command_for('vicon')

    assert command[:5] == [
        'bash', '-c', 'source "$1" && shift && exec "$@"', 'vicon', str(setup)]
    assert command[5:] == [
        'ros2', 'launch', 'vicon_receiver', 'client.launch.py',
        'hostname:=192.168.0.30:8802', 'topic_namespace:=vicon',
        'world_frame:=vicon_world', 'vicon_frame:=vicon']
