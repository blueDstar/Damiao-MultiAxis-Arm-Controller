"""Position references sampled by elapsed wall-clock time, in SI units."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class PositionTrajectory:
    start_rad: float
    target_rad: float
    requested_speed_rad_s: float

    def __post_init__(self) -> None:
        if not all(math.isfinite(v) for v in (self.start_rad, self.target_rad, self.requested_speed_rad_s)):
            raise ValueError("Position trajectory values must be finite.")
        if self.requested_speed_rad_s == 0:
            raise ValueError("Zero speed requests a stop, not a position trajectory.")

    @property
    def duration_s(self) -> float:
        return abs(self.target_rad - self.start_rad) / abs(self.requested_speed_rad_s)

    @property
    def velocity_rad_s(self) -> float:
        if self.start_rad == self.target_rad:
            return 0.0
        return math.copysign(abs(self.requested_speed_rad_s), self.target_rad - self.start_rad)

    def sample(self, elapsed_s: float) -> tuple[float, float, bool]:
        if not math.isfinite(elapsed_s):
            raise ValueError("Trajectory elapsed time must be finite.")
        elapsed_s = max(0.0, elapsed_s)
        if elapsed_s >= self.duration_s:
            return self.target_rad, 0.0, True
        return self.start_rad + self.velocity_rad_s * elapsed_s, self.velocity_rad_s, False
