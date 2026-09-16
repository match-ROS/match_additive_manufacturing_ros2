"""Acceleration-limited velocity followed by a finite moving-average filter."""

from collections import deque
import math


class VelocitySmoother:
    """Bound vector acceleration and jerk, preserving the velocity norm limit.

    The ramp has acceleration norm <= A. Averaging it over T = 2*A/J
    bounds jerk by J, including reversals, and reaches constant targets in
    finite time. Samples describe piecewise-linear velocities.
    """

    def __init__(self, acceleration: float, jerk: float) -> None:
        if not all(math.isfinite(v) and v > 0 for v in (acceleration, jerk)):
            raise ValueError('Acceleration and jerk limits must be finite and positive')
        self.acceleration = acceleration
        self.window = 2.0 * acceleration / jerk
        self.time = 0.0
        self.ramp = (0.0, 0.0, 0.0)
        self.samples = deque([(-self.window, self.ramp), (0.0, self.ramp)])

    @property
    def stopped(self) -> bool:
        return all(all(v == 0.0 for v in sample) for _, sample in self.samples)

    def step(self, target: tuple[float, float, float], dt: float) -> tuple[float, ...]:
        if not math.isfinite(dt) or dt <= 0 or not all(map(math.isfinite, target)):
            raise ValueError('Finite target and positive timestep required')
        delta = tuple(t - v for t, v in zip(target, self.ramp))
        norm = math.sqrt(sum(v * v for v in delta))
        scale = min(1.0, self.acceleration * dt / norm) if norm else 1.0
        self.ramp = target if scale == 1.0 else tuple(
            v + scale * d for v, d in zip(self.ramp, delta))
        self.time += dt
        self.samples.append((self.time, self.ramp))
        cutoff = self.time - self.window
        while len(self.samples) > 2 and self.samples[1][0] <= cutoff:
            self.samples.popleft()
        t0, v0 = self.samples[0]
        t1, v1 = self.samples[1]
        fraction = (cutoff - t0) / (t1 - t0)
        self.samples[0] = (cutoff, tuple(
            a + fraction * (b - a) for a, b in zip(v0, v1)))
        # Preserve exact constant targets (especially zero) after settling.
        if all(v == self.ramp for _, v in self.samples):
            return self.ramp
        integral = [0.0, 0.0, 0.0]
        previous_time, previous = self.samples[0]
        for timestamp, value in list(self.samples)[1:]:
            for i in range(3):
                integral[i] += (previous[i] + value[i]) * 0.5 * (timestamp - previous_time)
            previous_time, previous = timestamp, value
        return tuple(v / self.window for v in integral)
