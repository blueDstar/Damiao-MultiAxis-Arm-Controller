"""Offline CAN transport, not a hardware driver or a calibrated dynamics model.

The real codec/router/workers still perform verification, enable and position
control. Only the transport and motor response are simulated here.
"""

from __future__ import annotations

import math
import queue
import struct
import threading
import time
from dataclasses import dataclass

import can

from single_motor.damiao_v12 import INTEGER_REGISTERS
from robot_arm_2dof.kinematics import validate_ids


@dataclass
class DemoAxis:
    master_id: int
    position: float = 0.0
    velocity: float = 0.0
    requested_velocity: float = 0.0
    enabled: bool = False
    active: bool = False
    updated: float = 0.0


class DemoBus:
    # Unloaded demonstration response, deliberately not a hardware rating.
    RESPONSE_TIME_S = 0.01
    MAX_ACCELERATION_RAD_S2 = 1000.0

    def __init__(self, motor_ids=((1, 0x11), (2, 0x12))):
        validate_ids(motor_ids)
        now = time.monotonic()
        self.axes = {c: DemoAxis(m, updated=now) for c, m in motor_ids}
        self._rx = queue.Queue()
        self._lock = threading.Lock()
        self.shutdown_called = False

    def _advance(self, axis, now):
        dt = max(0.0, now - axis.updated)
        axis.updated = now
        if not axis.enabled:
            axis.velocity = 0.0
            return
        target = axis.requested_velocity if axis.active else 0.0
        # A simple first-order response with finite acceleration. No gravity,
        # collision, gearbox or load model; display is explicitly labelled demo.
        old = axis.velocity
        change = (target - old) * (1.0 - math.exp(-dt / self.RESPONSE_TIME_S))
        maximum_change = self.MAX_ACCELERATION_RAD_S2 * dt
        axis.velocity = old + max(-maximum_change, min(maximum_change, change))
        axis.position += 0.5 * (old + axis.velocity) * dt

    def _feedback(self, can_id, axis):
        wrapped = (axis.position + 12.5) % 25 - 12.5
        p = round((wrapped + 12.5) * 65535 / 25)
        v = round((axis.velocity + 30) * 4095 / 60)
        v = max(0, min(4095, v))
        t = 2048
        payload = bytes((can_id | int(axis.enabled) << 4, p >> 8, p & 255,
                         v >> 4, (v & 15) << 4 | t >> 8, t & 255, 25, 28))
        return can.Message(arbitration_id=axis.master_id, is_extended_id=False, data=payload)

    def send(self, message, timeout=None):
        with self._lock:
            if self.shutdown_called:
                raise RuntimeError("CAN mô phỏng đã đóng.")
            data = bytes(message.data)
            if len(data) != 8:
                return
            can_id = data[0] if message.arbitration_id == 0x7FF else message.arbitration_id
            axis = self.axes.get(can_id)
            if axis is None:
                return
            self._advance(axis, time.monotonic())
            if message.arbitration_id == 0x7FF:
                if data[2] == 0xCC:
                    self._rx.put(self._feedback(can_id, axis))
                elif data[2] == 0x33:
                    register = data[3]
                    values = {7: axis.master_id, 8: can_id, 10: 1, 14: 5017, 0x24: 5,
                              0x15: 12.5, 0x16: 30.0, 0x17: 10.0, 0x51: axis.position}
                    value = struct.pack("<I" if register in INTEGER_REGISTERS else "<f",
                                        values.get(register, 1))
                    self._rx.put(can.Message(arbitration_id=axis.master_id, is_extended_id=False,
                                             data=data[:4] + value))
                return
            if data == b"\xff" * 7 + b"\xfc":
                axis.enabled = True
            elif data == b"\xff" * 7 + b"\xfd":
                axis.enabled = False
                axis.velocity = axis.requested_velocity = 0.0
                axis.active = False
            elif data[:7] != b"\xff" * 7:
                kp = ((data[3] & 15) << 8) | data[4]
                kd = (data[5] << 4) | (data[6] >> 4)
                raw_velocity = (data[2] << 4) | (data[3] >> 4)
                velocity = raw_velocity * 60 / 4095 - 30
                axis.requested_velocity = 0.0 if abs(velocity) <= 60 / 4095 else velocity
                axis.active = bool(kp or kd)
            self._rx.put(self._feedback(can_id, axis))

    def recv(self, timeout=None):
        try:
            return self._rx.get(timeout=timeout)
        except queue.Empty:
            return None

    def shutdown(self):
        with self._lock:
            for axis in self.axes.values():
                axis.enabled = False
                axis.velocity = 0.0
            self.shutdown_called = True
