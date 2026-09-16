"""Bounded, read-only SSH diagnostics shared by the desktop and web GUI."""

import json
import math
import shlex
import subprocess
import time
from threading import Lock, Thread

ROBOT_SSH_TARGET = 'robot@192.168.0.200'
REMOTE_PROBE = r'''
import json, subprocess, time
start = time.time()
def read(args):
    try:
        result = subprocess.run(args, capture_output=True, text=True, timeout=2)
        return result.stdout.strip() if result.returncode == 0 else ''
    except (OSError, subprocess.TimeoutExpired):
        return ''
properties = dict(line.split('=', 1) for line in read(
    ['timedatectl', 'show-timesync', '--all']).splitlines() if '=' in line)
synchronized = read(['timedatectl', 'show', '-p', 'NTPSynchronized', '--value'])
print(json.dumps({
    'remote_start': start, 'remote_end': time.time(),
    'configured_servers': properties.get('SystemNTPServers', ''),
    'link_servers': properties.get('LinkNTPServers', ''),
    'fallback_servers': properties.get('FallbackNTPServers', ''),
    'active_server': properties.get('ServerName', ''),
    'synchronized': {'yes': True, 'no': False}.get(synchronized),
}))
'''


def probe_robot():
    """Estimate robot minus PC time, excluding remote processing duration."""
    result = {'target': ROBOT_SSH_TARGET, 'ssh_reachable': False,
              'offset_seconds': None, 'uncertainty_seconds': None,
              'configured_servers': '', 'link_servers': '', 'fallback_servers': '',
              'active_server': '', 'synchronized': None, 'error': None}
    started = time.time()
    monotonic_start = time.monotonic()
    try:
        completed = subprocess.run(
            ['ssh', '-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=yes',
             '-o', 'ConnectTimeout=3', '-o', 'ConnectionAttempts=1',
             ROBOT_SSH_TARGET, 'python3 -c ' + shlex.quote(REMOTE_PROBE)],
            capture_output=True, text=True, timeout=8,
        )
        elapsed = time.monotonic() - monotonic_start
        ended = time.time()
        # SSH uses 255 for transport/authentication failures. Other nonzero
        # statuses belong to the remote command, after a successful login.
        result['ssh_reachable'] = completed.returncode != 255
        if completed.returncode != 0:
            result['error'] = completed.stderr.strip()[-600:] or 'Remote diagnostics failed'
            return result
        data = json.loads(completed.stdout)
        remote_start, remote_end = float(data['remote_start']), float(data['remote_end'])
        if not all(math.isfinite(value) for value in (remote_start, remote_end)):
            raise ValueError('Invalid remote timestamps')
        if abs((ended - started) - elapsed) > 0.1 or remote_end < remote_start:
            raise ValueError('Clock changed during measurement; retry')
        result.update({key: data.get(key) for key in (
            'configured_servers', 'link_servers', 'fallback_servers',
            'active_server', 'synchronized')})
        result['offset_seconds'] = (remote_start + remote_end - started - ended) / 2
        result['uncertainty_seconds'] = max(0, elapsed - (remote_end - remote_start)) / 2
    except (OSError, subprocess.TimeoutExpired, ValueError, KeyError, TypeError) as exc:
        result['error'] = str(exc)
    finally:
        result['checked_at'] = time.time()
    return result


class RobotDebugInfo:
    """Refresh on demand at most every ten seconds, without blocking the UI."""

    def __init__(self):
        self._lock = Lock()
        self._running = False
        self._last_check = None
        self._result = {'target': ROBOT_SSH_TARGET, 'ssh_reachable': None,
                        'checked_at': None}

    def snapshot(self):
        with self._lock:
            if not self._running and (
                self._last_check is None or time.monotonic() - self._last_check >= 10
            ):
                self._running = True
                Thread(target=self._refresh, daemon=True).start()
            return dict(self._result, checking=self._running)

    def _refresh(self):
        try:
            result = probe_robot()
        except Exception as exc:
            result = {'target': ROBOT_SSH_TARGET, 'ssh_reachable': None,
                      'checked_at': time.time(), 'error': str(exc)}
        with self._lock:
            self._result = result
            self._last_check = time.monotonic()
            self._running = False


def format_debug_info(info):
    reachable = info.get('ssh_reachable')
    ssh = 'erreichbar' if reachable is True else 'nicht erreichbar' if reachable is False else 'noch nicht geprüft'
    offset = info.get('offset_seconds')
    difference = 'unbekannt' if offset is None else (
        f"{offset:+.3f} s (± {info['uncertainty_seconds']:.3f} s)"
    )
    synced = info.get('synchronized')
    synchronization = 'ja' if synced is True else 'nein' if synced is False else 'unbekannt'
    checked = info.get('checked_at')
    return '\n'.join([
        f"SSH {info['target']}: {ssh}",
        f"Zeitversatz Roboter − PC: {difference} · positiv = Roboter geht vor",
        f"Eingestellte Zeitserver: {info.get('configured_servers') or 'unbekannt'}",
        f"Zeitserver vom Netzwerk: {info.get('link_servers') or 'keine'}",
        f"Fallback-Zeitserver: {info.get('fallback_servers') or 'keine'}",
        f"Aktuell gewählter Zeitserver: {info.get('active_server') or 'unbekannt'}",
        f"Roboter-Uhr synchronisiert: {synchronization}",
        'Letzte Prüfung: ' + (time.strftime('%H:%M:%S', time.localtime(checked)) if checked else '—')
        + (' · Prüfung läuft …' if info.get('checking') else ''),
        *([f"Fehler: {info['error']}"] if info.get('error') else []),
    ])
