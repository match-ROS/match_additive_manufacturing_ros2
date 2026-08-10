"""Hardware-neutral watchdog between the process policy and valve motor topics."""

from dataclasses import dataclass
import math

from .flow_protocol import ValveMapper


@dataclass
class ValveCommandGate:
    left_mapper: ValveMapper
    right_mapper: ValveMapper
    timeout: float = 0.25
    enabled: bool = False
    target: float = 0.0
    last_target_time: float | None = None

    def observe_target(self, target: float, now: float) -> None:
        if not math.isfinite(target):
            raise ValueError('valve target must be finite')
        self.target = min(1.0, max(0.0, target))
        self.last_target_time = now

    def closed_positions(self) -> tuple[int, int]:
        return (self.left_mapper.position_for_target(0.0), self.right_mapper.position_for_target(0.0))

    def command(self, now: float) -> tuple[int, int]:
        fresh = self.last_target_time is not None and now - self.last_target_time <= self.timeout
        if not self.enabled or not fresh:
            return self.closed_positions()
        return (self.left_mapper.position_for_target(self.target), self.right_mapper.position_for_target(self.target))
