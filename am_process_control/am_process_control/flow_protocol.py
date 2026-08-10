"""ROS 1-compatible foam flow-sensor parsing and hardware-neutral valve mapping."""

from dataclasses import dataclass
import csv
import math


@dataclass(frozen=True)
class FlowSample:
    device_time_ms: int
    channel: int
    raw_adc: float
    voltage_v: float
    current_ma: float
    percent: float
    engineering_value: float = math.nan


def parse_flow_line(line: str, prefix: str = '') -> FlowSample:
    """Parse the deployed Arduino CSV format; reject headers and malformed data."""
    line = line.strip()
    if prefix:
        if not line.startswith(prefix):
            raise ValueError('flow line does not have expected prefix')
        line = line[len(prefix):]
    fields = next(csv.reader([line]))
    if not fields or fields[0].strip() == 'time_ms':
        raise ValueError('flow line is a header')
    if len(fields) < 6:
        raise ValueError('flow line needs at least six CSV fields')
    try:
        values = FlowSample(
            device_time_ms=int(float(fields[0])), channel=int(float(fields[1])),
            raw_adc=float(fields[2]), voltage_v=float(fields[3]),
            current_ma=float(fields[4]), percent=float(fields[5]),
            engineering_value=math.nan if len(fields) < 7 or not fields[6].strip() else float(fields[6]),
        )
    except (TypeError, ValueError) as error:
        raise ValueError('flow line has non-numeric fields') from error
    if values.device_time_ms < 0 or values.channel < 0 or not all(math.isfinite(value) for value in (
            values.raw_adc, values.voltage_v, values.current_ma, values.percent)):
        raise ValueError('flow line has invalid numeric range')
    return values


@dataclass
class ExponentialFlowFilter:
    alpha: float = 0.25
    value: float | None = None

    def update(self, measurement: float) -> float:
        if not math.isfinite(measurement):
            raise ValueError('flow measurement must be finite')
        if not 0.0 < self.alpha <= 1.0:
            raise ValueError('filter alpha must be in (0, 1]')
        self.value = measurement if self.value is None else self.alpha * measurement + (1.0 - self.alpha) * self.value
        return self.value


@dataclass(frozen=True)
class ValveMapper:
    minimum_position: int = 0
    maximum_position: int = 1023

    def position_for_target(self, target: float) -> int:
        if not math.isfinite(target):
            raise ValueError('valve target must be finite')
        target = min(1.0, max(0.0, target))
        return round(self.minimum_position + target * (self.maximum_position - self.minimum_position))


class RecordingDynamixelAdapter:
    """Mockable adapter used in offline tests; a vendor service adapter replaces it later."""
    def __init__(self) -> None:
        self.commands: list[tuple[str, int]] = []

    def set_goal_position(self, motor: str, position: int) -> None:
        self.commands.append((motor, position))
