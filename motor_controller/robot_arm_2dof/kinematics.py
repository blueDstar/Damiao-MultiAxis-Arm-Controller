"""Planar shoulder/elbow geometry and explicit encoder-to-joint mapping (SI units)."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class ArmGeometry:
    upper_length: float = 0.30
    forearm_length: float = 0.25
    shoulder_height: float = 0.17

    def __post_init__(self):
        if not all(math.isfinite(v) and v > 0 for v in
                   (self.upper_length, self.forearm_length, self.shoulder_height)):
            raise ValueError("Kích thước cánh tay phải là số dương hữu hạn (m).")

    def points(self, shoulder: float, elbow: float):
        """Both axes point along Y; positive angles lift the link toward +Z."""
        if not all(math.isfinite(v) for v in (shoulder, elbow)):
            raise ValueError("Góc khớp phải là số hữu hạn.")
        base = (0.0, 0.0, self.shoulder_height)
        middle = (self.upper_length * math.cos(shoulder), 0.0,
                  base[2] + self.upper_length * math.sin(shoulder))
        tip = (middle[0] + self.forearm_length * math.cos(shoulder + elbow), 0.0,
               middle[2] + self.forearm_length * math.sin(shoulder + elbow))
        return base, middle, tip


@dataclass(frozen=True)
class JointMapping:
    mounting_rad: float
    direction: int = 1

    def __post_init__(self):
        if self.direction not in (-1, 1) or not math.isfinite(self.mounting_rad):
            raise ValueError("Chiều khớp phải là +1 hoặc -1; góc lắp phải hữu hạn.")

    def angle(self, motor_position: float, software_zero: float) -> float:
        return self.mounting_rad + self.direction * (motor_position - software_zero)


ANGLE_FACTORS = {"deg": math.pi / 180, "rad": 1.0}
SPEED_FACTORS = {"rad/s": 1.0, "rpm": math.tau / 60}


def parse_target(angle_text, angle_unit, speed_text, speed_unit, vmax=30.0):
    angle = float(angle_text) * ANGLE_FACTORS[angle_unit]
    speed = float(speed_text) * SPEED_FACTORS[speed_unit]
    if not math.isfinite(angle) or not math.isfinite(speed):
        raise ValueError("Góc và tốc độ phải là số hữu hạn.")
    limit = min(30.0, vmax)
    if abs(speed) > limit and not math.isclose(abs(speed), limit, rel_tol=1e-12):
        raise ValueError(f"Tốc độ motor phải nằm trong [-{limit:g}, +{limit:g}] rad/s.")
    return angle, math.copysign(min(abs(speed), limit), speed)


def validate_ids(ids):
    if any(len(pair) != 2 for pair in ids) or any(
        not isinstance(v, int) or isinstance(v, bool) for pair in ids for v in pair
    ):
        raise ValueError("CAN ID và Master ID phải là số nguyên.")
    if len(ids) != 2 or len({c for c, _ in ids}) != 2 or len({m for _, m in ids}) != 2:
        raise ValueError("Cần hai CAN ID và hai Master ID riêng biệt.")
    command_ids = {v for c, _ in ids for v in (c, c + 0x100)}
    if any(not 1 <= c <= 15 or not 1 <= m < 0x7FF or m in command_ids for c, m in ids):
        raise ValueError("CAN ID: 0x01..0x0F. Master ID: 0x001..0x7FE, không trùng ID lệnh.")
    return ids
