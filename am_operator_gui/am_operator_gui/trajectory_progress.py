"""Display progress from the atomic /trajectory_state reference snapshot."""

import json
import math
import threading
import time


class TrajectoryProgress:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._state = None
        self._received_at = None

    def update(self, payload: str) -> None:
        try:
            state = json.loads(payload)
            index, points = state['path_index'], state['path_points']
            phase, override = state['segment_phase'], state['velocity_override']
            if (type(index) is not int or type(points) is not int
                    or points < 1 or not 0 <= index < points
                    or type(phase) not in (int, float) or not math.isfinite(phase)
                    or not 0.0 <= phase <= 1.0
                    or type(override) not in (int, float) or not math.isfinite(override)
                    or override < 0.0 or type(state['start_enabled']) is not bool):
                return
        except (ValueError, KeyError, TypeError):
            return
        with self._lock:
            self._state = state
            self._received_at = time.monotonic()

    def snapshot(self) -> dict:
        with self._lock:
            state, received_at = self._state, self._received_at
        if state is None:
            return {'percent': None, 'path_index': None, 'path_points': None,
                    'status': 'waiting', 'text': 'Wartet auf Trackingdaten'}
        index, points = state['path_index'], state['path_points']
        percent = 100.0 if points == 1 else min(
            100.0, 100.0 * (index + state['segment_phase']) / (points - 1))
        if time.monotonic() - received_at > 2.5:
            status, label = 'stale', 'Trackingdaten veraltet'
        elif index == points - 1:
            status, label = 'complete', 'Pfadende erreicht'
        elif not state['start_enabled']:
            status, label = 'stopped', 'Angehalten'
        elif state['velocity_override'] == 0.0:
            status, label = 'paused', 'Pausiert'
        else:
            status, label = 'running', 'Tracking aktiv'
        return {'percent': percent, 'path_index': index, 'path_points': points,
                'status': status,
                'text': f'{percent:.1f} % · Index {index} / {points - 1} · {label}'}
