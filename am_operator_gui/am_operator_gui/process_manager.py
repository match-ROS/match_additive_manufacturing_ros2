import os
import signal
import subprocess
from collections import deque
from dataclasses import dataclass, field
from threading import Lock, Thread
from typing import Callable, Deque, List, Optional


OutputCallback = Callable[[str, str], None]


@dataclass
class ManagedProcess:
    name: str
    command: List[str]
    output_callback: Optional[OutputCallback] = None
    process: Optional[subprocess.Popen] = None
    output: Deque[str] = field(default_factory=lambda: deque(maxlen=500))
    return_code: Optional[int] = None
    _lock: Lock = field(default_factory=Lock)

    def start(self) -> None:
        with self._lock:
            if self.is_running():
                return
            self.return_code = None
            self.output.clear()
            self.process = subprocess.Popen(
                self.command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                start_new_session=True,
            )
            Thread(target=self._read_output, daemon=True).start()
            Thread(target=self._wait, daemon=True).start()

    def is_running(self) -> bool:
        return self.process is not None and self.process.poll() is None

    def poll(self) -> Optional[int]:
        if self.process is None:
            return self.return_code
        code = self.process.poll()
        if code is not None:
            self.return_code = code
        return code

    def stop(self, timeout: float = 5.0) -> None:
        with self._lock:
            process = self.process
            if process is None:
                return
            if process.poll() is not None:
                self.return_code = process.returncode
                return
            try:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
            except ProcessLookupError:
                pass
            self.return_code = process.returncode

    def _read_output(self) -> None:
        process = self.process
        if process is None or process.stdout is None:
            return
        for line in process.stdout:
            text = line.rstrip()
            self.output.append(text)
            if self.output_callback is not None:
                self.output_callback(self.name, text)

    def _wait(self) -> None:
        process = self.process
        if process is None:
            return
        self.return_code = process.wait()
        if self.output_callback is not None:
            self.output_callback(self.name, f"exited with code {self.return_code}")


class ProcessRegistry:
    def __init__(self, output_callback: Optional[OutputCallback] = None) -> None:
        self._processes: dict[str, ManagedProcess] = {}
        self._output_callback = output_callback

    def start(self, name: str, command: List[str], replace: bool = True) -> ManagedProcess:
        existing = self._processes.get(name)
        if existing is not None and existing.is_running():
            if not replace:
                return existing
            existing.stop()
        managed = ManagedProcess(name=name, command=command, output_callback=self._output_callback)
        self._processes[name] = managed
        managed.start()
        return managed

    def get(self, name: str) -> Optional[ManagedProcess]:
        return self._processes.get(name)

    def stop(self, name: str) -> None:
        process = self._processes.get(name)
        if process is not None:
            process.stop()

    def stop_all(self) -> None:
        for process in list(self._processes.values()):
            process.stop()
