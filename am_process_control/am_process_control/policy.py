"""Pure process-output policy; deliberately independent of a hardware driver."""

from dataclasses import dataclass


@dataclass
class ProcessPolicy:
    max_target: float = 1.0
    max_rate_per_second: float = 0.25
    feedback_timeout: float = 0.5
    target: float = 0.0
    armed: bool = False
    acknowledged: bool = False
    print_enabled: bool = False
    last_feedback_time: float | None = None
    output: float = 0.0

    def set_target(self, target: float) -> None:
        if target < 0.0 or target > self.max_target:
            raise ValueError(f'target must be in [0, {self.max_target}]')
        self.target = target

    def observe_feedback(self, now: float) -> None:
        self.last_feedback_time = now

    def revoke_acknowledgement(self) -> None:
        self.acknowledged = False

    def safe_target(self, now: float) -> float:
        feedback_fresh = (self.last_feedback_time is not None
                          and now - self.last_feedback_time <= self.feedback_timeout)
        if not (self.armed and self.acknowledged and self.print_enabled and feedback_fresh):
            return 0.0
        return self.target

    def step(self, now: float, elapsed: float) -> float:
        """Return a bounded command; every disarm/fault immediately commands zero."""
        desired = self.safe_target(now)
        if desired == 0.0:
            self.output = 0.0
            return self.output
        maximum_delta = self.max_rate_per_second * max(0.0, elapsed)
        if desired > self.output:
            self.output = min(desired, self.output + maximum_delta)
        else:
            self.output = max(desired, self.output - maximum_delta)
        return self.output
