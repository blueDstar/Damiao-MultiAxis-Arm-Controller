"""Accumulate measured rotation across single-turn and CAN-range rollovers."""

from __future__ import annotations

import math


class PositionTracker:
    def __init__(self) -> None:
        self.position_rad: float | None = None
        self.raw_rad: float | None = None
        self.velocity = 0.0
        self.timestamp = 0.0
        self.detected_period: float | None = None

    @staticmethod
    def equivalent(reference: float, raw: float, position_range: float, tolerance=0.05) -> bool:
        difference = reference - raw
        return any(abs(difference - round(difference / p) * p) <= tolerance
                   for p in (2 * math.pi, 2 * position_range))

    def seed(self, position: float, raw: float, velocity: float, timestamp: float) -> None:
        self.position_rad, self.raw_rad = position, raw
        self.velocity, self.timestamp = velocity, timestamp
        self.detected_period = None

    def update(self, raw: float, velocity: float, timestamp: float, position_range: float) -> float:
        if self.raw_rad is None or self.position_rad is None:
            self.seed(raw, raw, velocity, timestamp)
            return raw
        delta = raw - self.raw_rad
        elapsed = max(0.0, timestamp - self.timestamp)
        predicted = (self.velocity + velocity) * 0.5 * elapsed
        candidates = [(abs(delta - predicted), delta, None)]
        periods = (self.detected_period,) if self.detected_period else (2 * math.pi, 2 * position_range)
        for period in periods:
            # Only unwrap an observed discontinuity. Elapsed time/velocity must
            # never invent revolutions from an ordinary small position change.
            if abs(delta) < period / 2:
                continue
            if abs(raw) > period / 2 + 0.01 or abs(self.raw_rad) > period / 2 + 0.01:
                continue
            turns = round((predicted - delta) / period)
            if turns:
                corrected = delta + turns * period
                candidates.append((abs(corrected - predicted), corrected, period))
        _, measured_delta, period = min(candidates, key=lambda item: item[0])
        if period is not None:
            self.detected_period = period
        self.position_rad += measured_delta
        self.raw_rad, self.velocity, self.timestamp = raw, velocity, timestamp
        return self.position_rad
