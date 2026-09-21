#!/usr/bin/env python3
"""Foreground process supervisor used through one persistent SSH connection."""

from __future__ import annotations

import json
import os
import re
import signal
import subprocess
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import IO, Optional


PROTOCOL_VERSION = 1
HEARTBEAT_TIMEOUT_S = 3.0
NAME_PATTERN = re.compile(r'^[A-Za-z0-9_.:-]+$')


@dataclass
class Child:
    name: str
    command: list[str]
    process: subprocess.Popen
    expected_stop: bool = False


class RemoteSupervisor:
    def __init__(self, stdin: IO[str] = sys.stdin, stdout: IO[str] = sys.stdout) -> None:
        self.stdin = stdin
        self.stdout = stdout
        self.children: dict[str, Child] = {}
        self.lock = threading.RLock()
        self.write_lock = threading.Lock()
        self.last_heartbeat = time.monotonic()
        self.heartbeat_lost = False
        self.stopping = False
        self.logs: dict[str, deque[str]] = {}

    def emit(self, payload: dict) -> None:
        with self.write_lock:
            self.stdout.write(json.dumps(payload, separators=(',', ':')) + '\n')
            self.stdout.flush()

    def serve(self) -> int:
        self.emit({'event': 'hello', 'version': PROTOCOL_VERSION, 'pid': os.getpid()})
        watchdog = threading.Thread(target=self._watchdog, daemon=True)
        watchdog.start()
        try:
            for raw in self.stdin:
                request = None
                try:
                    request = json.loads(raw)
                    self.last_heartbeat = time.monotonic()
                    self.heartbeat_lost = False
                    self._handle(request)
                except Exception as exc:  # noqa: BLE001 - protocol boundary
                    request_id = request.get('id') if isinstance(request, dict) else None
                    self.emit({'event': 'error', 'id': request_id, 'message': str(exc)})
        finally:
            self.stopping = True
            self.stop_all('SSH channel closed')
        return 0

    def _handle(self, request: dict) -> None:
        operation = str(request.get('op', ''))
        request_id = request.get('id')
        if operation == 'ping':
            return
        if operation == 'start':
            name = self._valid_name(request.get('name'))
            command = request.get('command')
            if not isinstance(command, list) or not command or not all(isinstance(item, str) for item in command):
                raise ValueError('command must be a non-empty string array')
            self.start(name, command)
            self.emit({'event': 'ack', 'id': request_id, 'ok': True})
            return
        if operation == 'stop':
            self.stop(self._valid_name(request.get('name')), 'operator request')
            self.emit({'event': 'ack', 'id': request_id, 'ok': True})
            return
        if operation == 'stop_all':
            self.stop_all('operator request')
            self.emit({'event': 'ack', 'id': request_id, 'ok': True})
            return
        if operation == 'status':
            self.emit({'event': 'status', 'id': request_id, 'processes': self.snapshot()})
            return
        if operation == 'logs':
            name = self._valid_name(request.get('name'))
            limit = max(1, min(500, int(request.get('limit', 100))))
            with self.lock:
                messages = list(self.logs.get(name, ())) [-limit:]
            self.emit({'event': 'logs', 'id': request_id, 'name': name, 'messages': messages})
            return
        raise ValueError(f'unknown operation: {operation}')

    @staticmethod
    def _valid_name(value) -> str:
        name = str(value or '')
        if not NAME_PATTERN.fullmatch(name):
            raise ValueError(f'invalid process name: {name!r}')
        return name

    def start(self, name: str, command: list[str]) -> None:
        with self.lock:
            current = self.children.get(name)
            if current is not None and current.process.poll() is None:
                raise RuntimeError(f'{name} is already running')
            process = subprocess.Popen(
                command,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                start_new_session=True,
            )
            child = Child(name=name, command=list(command), process=process)
            self.children[name] = child
            self.logs[name] = deque(maxlen=500)
        self.emit({'event': 'started', 'name': name, 'pid': process.pid})
        threading.Thread(target=self._read_output, args=(child,), daemon=True).start()
        threading.Thread(target=self._wait_child, args=(child,), daemon=True).start()

    def _read_output(self, child: Child) -> None:
        if child.process.stdout is None:
            return
        for line in child.process.stdout:
            message = line.rstrip()
            chunks = [message[offset:offset + 8000] for offset in range(0, len(message), 8000)] or ['']
            for index, chunk in enumerate(chunks):
                rendered = chunk if index == 0 else f'[continued] {chunk}'
                with self.lock:
                    self.logs.setdefault(child.name, deque(maxlen=500)).append(rendered)
                self.emit({'event': 'log', 'name': child.name, 'message': rendered})

    def _wait_child(self, child: Child) -> None:
        code = child.process.wait()
        with self.lock:
            current = self.children.get(child.name)
            if current is child:
                self.children.pop(child.name, None)
        self.emit({
            'event': 'exited', 'name': child.name, 'return_code': code,
            'expected': child.expected_stop,
        })

    def stop(self, name: str, reason: str) -> None:
        with self.lock:
            child = self.children.get(name)
        if child is None or child.process.poll() is not None:
            return
        child.expected_stop = True
        self.emit({'event': 'stopping', 'name': name, 'reason': reason})
        self._terminate_group(child.process)

    @staticmethod
    def _terminate_group(process: subprocess.Popen) -> None:
        for sig, timeout in ((signal.SIGINT, 2.0), (signal.SIGTERM, 2.0)):
            if process.poll() is not None:
                return
            try:
                os.killpg(process.pid, sig)
                process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                continue
            except ProcessLookupError:
                return
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=1.0)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                pass

    def stop_all(self, reason: str) -> None:
        # Stop motion producers first. J-PARSE remains alive briefly so its
        # command timeout can publish zero joint velocities before shutdown.
        order = ('move_base', 'base_follower', 'move_arm', 'arm_follower',
                 'switch_arm_velocity', 'path_index')
        with self.lock:
            names = list(self.children)
        for name in order:
            if name in names:
                self.stop(name, reason)
        time.sleep(0.25)
        with self.lock:
            remaining = list(self.children)
        for name in remaining:
            self.stop(name, reason)

    def snapshot(self) -> dict:
        with self.lock:
            return {
                name: {'running': child.process.poll() is None,
                       'return_code': child.process.poll(), 'command': child.command}
                for name, child in self.children.items()
            }

    def _watchdog(self) -> None:
        while not self.stopping:
            time.sleep(0.25)
            if (not self.heartbeat_lost and
                    time.monotonic() - self.last_heartbeat > HEARTBEAT_TIMEOUT_S):
                self.heartbeat_lost = True
                self.emit({'event': 'watchdog', 'message': 'GUI heartbeat timed out; stopping remote processes'})
                self.stop_all('GUI heartbeat timeout')


def main() -> int:
    supervisor = RemoteSupervisor()
    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda *_args: sys.stdin.close())
    return supervisor.serve()


if __name__ == '__main__':
    raise SystemExit(main())
