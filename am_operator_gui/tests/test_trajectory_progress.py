"""Reference progress stays coherent across segments, pauses and lost telemetry."""

import json
from types import SimpleNamespace

import pytest

from am_operator_gui.trajectory_progress import TrajectoryProgress


def _state(**changes):
    state = {'path_index': 20, 'path_points': 101, 'segment_phase': 0.5,
             'velocity_override': 1.0, 'start_enabled': True}
    state.update(changes)
    return json.dumps(state)


def test_segment_phase_and_nonzero_start_use_entire_path():
    progress = TrajectoryProgress()
    assert progress.snapshot()['percent'] is None
    progress.update(_state())
    snapshot = progress.snapshot()
    assert snapshot['percent'] == pytest.approx(20.5)
    assert snapshot['status'] == 'running'
    assert 'Index 20 / 100' in snapshot['text']


@pytest.mark.parametrize('changes, status, percent', [
    ({'start_enabled': False}, 'stopped', 20.5),
    ({'velocity_override': 0.0}, 'paused', 20.5),
    ({'path_index': 100, 'segment_phase': 0.0}, 'complete', 100.0),
    ({'path_index': 0, 'path_points': 1, 'segment_phase': 0.0}, 'complete', 100.0),
])
def test_stopped_paused_and_endpoint(changes, status, percent):
    progress = TrajectoryProgress()
    progress.update(_state(**changes))
    assert progress.snapshot()['status'] == status
    assert progress.snapshot()['percent'] == pytest.approx(percent)


def test_stale_data_keeps_value_and_recovers(monkeypatch):
    now = [100.0]
    monkeypatch.setattr('am_operator_gui.trajectory_progress.time.monotonic', lambda: now[0])
    progress = TrajectoryProgress()
    progress.update(_state())
    now[0] += 3.0
    snapshot = progress.snapshot()
    assert snapshot['status'] == 'stale'
    assert snapshot['percent'] == pytest.approx(20.5)
    progress.update(_state(path_index=21))
    assert progress.snapshot()['status'] == 'running'


@pytest.mark.parametrize('payload', [
    'broken JSON', 'null', '[]', '{}',
    _state(path_index=-1), _state(path_index=101), _state(path_points=0),
    _state(segment_phase=float('nan')), _state(segment_phase=1.1),
    _state(path_index=True), _state(velocity_override=float('inf')),
])
def test_invalid_data_neither_replaces_nor_refreshes_progress(payload, monkeypatch):
    now = [100.0]
    monkeypatch.setattr('am_operator_gui.trajectory_progress.time.monotonic', lambda: now[0])
    progress = TrajectoryProgress()
    progress.update(_state())
    now[0] += 3.0
    progress.update(payload)
    assert progress.snapshot()['status'] == 'stale'
    assert progress.snapshot()['percent'] == pytest.approx(20.5)


def test_service_exposes_ros_progress_and_offline_default(tmp_path):
    from am_operator_gui.operator_service import OperatorService

    service = OperatorService(config_path=tmp_path / 'config.json')
    assert service.snapshot()['trajectory_progress']['status'] == 'waiting'
    progress = TrajectoryProgress()
    progress.update(_state())
    service.ros_bridge = SimpleNamespace(
        trajectory_progress=progress.snapshot,
        move_start_distances_cm=lambda _index: {'base': None, 'arm': None})
    assert service.snapshot()['trajectory_progress']['percent'] == pytest.approx(20.5)


def test_desktop_renders_fractional_progress_and_stale_state(monkeypatch):
    monkeypatch.setenv('QT_QPA_PLATFORM', 'offscreen')
    pytest.importorskip('PyQt5')
    from PyQt5.QtWidgets import QApplication, QProgressBar
    from am_operator_gui.gui import OperatorWindow

    app = QApplication.instance() or QApplication([])
    progress = TrajectoryProgress()
    progress.update(_state())
    bar = QProgressBar()
    bar.setRange(0, 1000)
    # Use the actual refresh method with the unrelated controls disabled.
    class RefreshOnly:
        def __getattr__(self, name):
            if name.startswith('_set_'):
                return lambda *args: None
            return SimpleNamespace(setText=lambda _text: None)

    window = RefreshOnly()
    window.service = SimpleNamespace(
        trajectory_progress=progress.snapshot,
        move_start_distances_cm=lambda: {'base': None, 'arm': None})
    window.trajectory_progress_bar = bar
    OperatorWindow._refresh_process_states(window)
    assert bar.value() == 205
    assert 'Index 20 / 100' in bar.format()
    monkeypatch.setattr('am_operator_gui.trajectory_progress.time.monotonic',
                        lambda: progress._received_at + 3.0)
    OperatorWindow._refresh_process_states(window)
    assert not bar.isEnabled()
    assert 'veraltet' in bar.format()
    app.processEvents()
