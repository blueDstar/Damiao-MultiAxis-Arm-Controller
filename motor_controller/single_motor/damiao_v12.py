"""CAN helpers for the Damiao DM4310 V3 / firmware V17 protocol."""

from __future__ import annotations

import math
import struct
import threading
import time
from dataclasses import dataclass
from typing import Callable

import can


REGISTER_MASTER_ID = 0x07
REGISTER_CAN_ID = 0x08
REGISTER_CONTROL_MODE = 0x0A
REGISTER_POSITION_RANGE = 0x15
REGISTER_VELOCITY_RANGE = 0x16
REGISTER_TORQUE_RANGE = 0x17
REGISTER_FIRMWARE_VERSION = 0x0E
REGISTER_TIMEOUT = 0x09
REGISTER_POLE_PAIRS = 0x10
REGISTER_CAN_BAUD = 0x23
REGISTER_SUB_VERSION = 0x24
CONTROL_MODE_MIT = 1
CONTROL_MODE_POSITION_VELOCITY = 2
REGISTER_READ_COMMAND = 0x33
CAN_REGISTER_REQUEST_ID = 0x7FF
INTEGER_REGISTERS = {
    REGISTER_MASTER_ID,
    REGISTER_CAN_ID,
    REGISTER_TIMEOUT,
    REGISTER_CONTROL_MODE,
    REGISTER_FIRMWARE_VERSION,
    REGISTER_POLE_PAIRS,
    REGISTER_CAN_BAUD,
    REGISTER_SUB_VERSION,
}
PRESENT_PARAMETER_REGISTERS = (
    (0x00, "Undervoltage threshold", "V"),
    (0x01, "Torque constant", "Nm/A"),
    (0x02, "Motor overtemperature limit", "degC"),
    (0x03, "Overcurrent limit", "per-unit"),
    (0x04, "Acceleration", "krad/s^2"),
    (0x05, "Deceleration", "krad/s^2"),
    (0x06, "Maximum speed", "rad/s"),
    (REGISTER_MASTER_ID, "Master ID", "hex"),
    (REGISTER_CAN_ID, "CAN ID", "hex"),
    (REGISTER_TIMEOUT, "CAN timeout", "50 us/count"),
    (REGISTER_CONTROL_MODE, "Control mode", "code"),
    (0x0B, "Viscous damping", ""),
    (0x0C, "Rotor inertia", "kg*m^2"),
    (REGISTER_FIRMWARE_VERSION, "Firmware version", "code"),
    (REGISTER_POLE_PAIRS, "Pole pairs", ""),
    (0x11, "Phase resistance", "ohm"),
    (0x12, "Phase inductance", "H"),
    (0x13, "Flux linkage", "Wb"),
    (0x14, "Gear ratio", ""),
    (0x15, "Position range (PMAX)", "rad"),
    (0x16, "Velocity range (VMAX)", "rad/s"),
    (0x17, "Torque range (TMAX)", "Nm"),
    (0x18, "Current-loop bandwidth", "Hz"),
    (0x19, "Velocity-loop Kp", ""),
    (0x1A, "Velocity-loop Ki", ""),
    (0x1B, "Position-loop Kp", ""),
    (0x1C, "Position-loop Ki", ""),
    (0x1D, "Overvoltage threshold", "V"),
    (0x1E, "Gear torque efficiency", ""),
    (0x1F, "Velocity damping coefficient", ""),
    (0x20, "Velocity filter bandwidth", "Hz"),
    (0x21, "Current-loop gain factor", ""),
    (0x22, "Velocity-loop gain factor", ""),
    (REGISTER_CAN_BAUD, "CAN bitrate code", "code"),
    (REGISTER_SUB_VERSION, "Sub-version", ""),
    (0x32, "U-phase offset", ""),
    (0x33, "V-phase offset", ""),
    (0x34, "Calibration factor K1", ""),
    (0x35, "Calibration factor K2", ""),
    (0x36, "Mechanical offset", "rad"),
    (0x37, "Motor direction", ""),
    (0x50, "Motor-side position", "rad"),
    (0x51, "Output-shaft position", "rad"),
)


@dataclass(frozen=True)
class MotorSettings:
    can_id: int
    master_id: int
    control_mode: int
    position_range_rad: float
    velocity_range_rad_s: float
    torque_range_nm: float
    firmware_version: int
    sub_version: int


@dataclass(frozen=True)
class MotorFeedback:
    status: int
    position_rad: float
    velocity_rad_s: float
    torque_nm: float
    driver_temperature_c: int
    motor_temperature_c: int


@dataclass(frozen=True)
class PresentParameter:
    address: int
    name: str
    value: int | float
    unit: str


class MoveCancelledError(RuntimeError):
    """Raised when a user stops an active motor move."""


class DamiaoMotor:
    def __init__(self, bus: can.BusABC, can_id: int, master_id: int) -> None:
        if not 0 <= can_id < 16:
            raise ValueError("CAN ID must be between 0x00 and 0x0F for status feedback.")
        if not 0 <= master_id <= 0x7FF:
            raise ValueError("Master ID must be a standard 11-bit CAN ID.")

        self.bus = bus
        self.can_id = can_id
        self.master_id = master_id
        self.position_range_rad = 0.0
        self.velocity_range_rad_s = 0.0
        self.torque_range_nm = 0.0
        self.control_mode = 0
        self.control_frame_id = can_id
        self.enabled = False
        self.last_feedback: MotorFeedback | None = None

    def _send(self, arbitration_id: int, payload: bytes) -> None:
        message = can.Message(
            arbitration_id=arbitration_id,
            is_extended_id=False,
            data=payload,
        )
        self.bus.send(message)

    def read_register(self, register: int, timeout_s: float = 1.0) -> int | float:
        request = bytes(
            (
                self.can_id & 0xFF,
                (self.can_id >> 8) & 0xFF,
                REGISTER_READ_COMMAND,
                register,
                0,
                0,
                0,
                0,
            )
        )
        self._send(CAN_REGISTER_REQUEST_ID, request)

        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            message = self.bus.recv(timeout=max(0.0, deadline - time.monotonic()))
            if message is None or message.arbitration_id != self.master_id:
                continue
            if len(message.data) != 8:
                continue
            if message.data[:4] != request[:4]:
                continue

            value_bytes = bytes(message.data[4:8])
            if register in INTEGER_REGISTERS:
                return int.from_bytes(value_bytes, byteorder="little", signed=False)
            return struct.unpack("<f", value_bytes)[0]

        raise TimeoutError(
            f"No register 0x{register:02X} response from Master ID 0x{self.master_id:02X}. "
            "Check the USB-CAN adapter, CAN wiring, bitrate, and configured IDs."
        )

    def read_present_parameters(self) -> list[PresentParameter]:
        return [
            PresentParameter(
                address=address,
                name=name,
                value=self.read_register(address),
                unit=unit,
            )
            for address, name, unit in PRESENT_PARAMETER_REGISTERS
        ]

    def verify_configuration(self) -> MotorSettings:
        actual_can_id = int(self.read_register(REGISTER_CAN_ID))
        actual_master_id = int(self.read_register(REGISTER_MASTER_ID))
        actual_mode = int(self.read_register(REGISTER_CONTROL_MODE))
        position_range = float(self.read_register(REGISTER_POSITION_RANGE))
        velocity_range = float(self.read_register(REGISTER_VELOCITY_RANGE))
        torque_range = float(self.read_register(REGISTER_TORQUE_RANGE))
        firmware_version = int(self.read_register(REGISTER_FIRMWARE_VERSION))
        sub_version = int(self.read_register(REGISTER_SUB_VERSION))

        if actual_can_id != self.can_id:
            raise RuntimeError(
                f"Driver CAN ID is 0x{actual_can_id:02X}, expected 0x{self.can_id:02X}."
            )
        if actual_master_id != self.master_id:
            raise RuntimeError(
                f"Driver Master ID is 0x{actual_master_id:02X}, "
                f"expected 0x{self.master_id:02X}."
            )
        if actual_mode not in (CONTROL_MODE_MIT, CONTROL_MODE_POSITION_VELOCITY):
            raise RuntimeError(
                f"Driver mode is {actual_mode}; this interface supports MIT (1) "
                "and Position-Velocity (2)."
            )
        if not math.isfinite(position_range) or position_range <= 0:
            raise RuntimeError("Driver returned an invalid PMAX position range.")
        if not math.isfinite(velocity_range) or velocity_range <= 0:
            raise RuntimeError("Driver returned an invalid VMAX velocity range.")
        if not math.isfinite(torque_range) or torque_range <= 0:
            raise RuntimeError("Driver returned an invalid TMAX torque range.")

        self.position_range_rad = position_range
        self.velocity_range_rad_s = velocity_range
        self.torque_range_nm = torque_range
        self.control_mode = actual_mode
        self.control_frame_id = (
            self.can_id
            if actual_mode == CONTROL_MODE_MIT
            else 0x100 + self.can_id
        )
        return MotorSettings(
            can_id=actual_can_id,
            master_id=actual_master_id,
            control_mode=actual_mode,
            position_range_rad=position_range,
            velocity_range_rad_s=velocity_range,
            torque_range_nm=torque_range,
            firmware_version=firmware_version,
            sub_version=sub_version,
        )

    @staticmethod
    def _float_to_uint(value: float, minimum: float, maximum: float, bits: int) -> int:
        if not minimum <= value <= maximum:
            raise ValueError(f"Value {value:g} is outside [{minimum:g}, {maximum:g}].")
        return int((value - minimum) * ((1 << bits) - 1) / (maximum - minimum))

    def _pack_mit_command(
        self,
        position_rad: float,
        velocity_rad_s: float,
        kp: float,
        kd: float,
    ) -> bytes:
        position = self._float_to_uint(
            position_rad, -self.position_range_rad, self.position_range_rad, 16
        )
        velocity = self._float_to_uint(
            velocity_rad_s, -self.velocity_range_rad_s, self.velocity_range_rad_s, 12
        )
        kp_value = self._float_to_uint(kp, 0.0, 500.0, 12)
        kd_value = self._float_to_uint(kd, 0.0, 5.0, 12)
        torque = self._float_to_uint(0.0, -self.torque_range_nm, self.torque_range_nm, 12)

        return bytes(
            (
                (position >> 8) & 0xFF,
                position & 0xFF,
                (velocity >> 4) & 0xFF,
                ((velocity & 0x0F) << 4) | ((kp_value >> 8) & 0x0F),
                kp_value & 0xFF,
                (kd_value >> 4) & 0xFF,
                ((kd_value & 0x0F) << 4) | ((torque >> 8) & 0x0F),
                torque & 0xFF,
            )
        )

    def encode_position_command(
        self,
        position_rad: float,
        velocity_rad_s: float,
        kp: float,
        kd: float,
    ) -> tuple[int, bytes]:
        """Return the CAN arbitration ID and eight-byte position command."""
        if not all(math.isfinite(value) for value in (position_rad, velocity_rad_s, kp, kd)):
            raise ValueError("Position, velocity, Kp, and Kd must be finite numbers.")
        if abs(position_rad) > self.position_range_rad:
            raise ValueError(
                f"Target angle exceeds the driver's PMAX range ±{self.position_range_rad:g} rad."
            )
        if abs(velocity_rad_s) > self.velocity_range_rad_s:
            raise ValueError(
                f"Velocity exceeds the driver's VMAX range ±{self.velocity_range_rad_s:g} rad/s."
            )

        if self.control_mode == CONTROL_MODE_MIT:
            return self.can_id, self._pack_mit_command(
                position_rad, velocity_rad_s, kp, kd
            )
        if self.control_mode == CONTROL_MODE_POSITION_VELOCITY:
            return 0x100 + self.can_id, struct.pack("<ff", position_rad, velocity_rad_s)
        raise RuntimeError(f"Control mode {self.control_mode} does not support position commands.")

    def _read_feedback(self, timeout_s: float) -> MotorFeedback | None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            message = self.bus.recv(timeout=max(0.0, deadline - time.monotonic()))
            if message is None or message.arbitration_id != self.master_id:
                continue
            if len(message.data) != 8:
                continue

            payload = bytes(message.data)
            feedback_id = payload[0] & 0x0F
            if feedback_id != self.can_id:
                continue

            position_raw = int.from_bytes(payload[1:3], byteorder="big", signed=True)
            velocity_raw = (payload[3] << 4) | (payload[4] >> 4)
            if velocity_raw & 0x800:
                velocity_raw -= 0x1000
            torque_raw = ((payload[4] & 0x0F) << 8) | payload[5]
            if torque_raw & 0x800:
                torque_raw -= 0x1000

            feedback = MotorFeedback(
                status=payload[0] >> 4,
                position_rad=position_raw * self.position_range_rad / 32768.0,
                velocity_rad_s=velocity_raw * self.velocity_range_rad_s / 2048.0,
                torque_nm=torque_raw * self.torque_range_nm / 2048.0,
                driver_temperature_c=payload[6],
                motor_temperature_c=payload[7],
            )
            self.last_feedback = feedback
            return feedback
        return None

    def _send_enable(self) -> MotorFeedback:
        self.enabled = False
        self._send(self.control_frame_id, b"\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFC")
        feedback = self._read_feedback(timeout_s=1.0)
        if feedback is None:
            self.last_feedback = None
            raise TimeoutError("No motor feedback after enable command; motion was not started.")
        if feedback.status != 1:
            self.last_feedback = feedback
            raise RuntimeError(
                f"Motor did not enter Enable Mode (status {feedback.status}). "
                "Check the fault code in Damiao Debugging Tool."
            )
        self.enabled = True
        self.last_feedback = feedback
        return feedback

    def enable(self) -> MotorFeedback:
        """Enable motor torque and require positive enabled-state feedback."""
        return self._send_enable()

    def disable(self) -> None:
        self._send(self.control_frame_id, b"\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFD")
        self.enabled = False

    def move_to_position(
        self,
        position_rad: float,
        max_velocity_rad_s: float,
        position_tolerance_rad: float,
        velocity_tolerance_rad_s: float,
        timeout_s: float,
        command_rate_hz: float,
        kp: float = 2.0,
        kd: float = 1.0,
        cancel_event: threading.Event | None = None,
        progress_callback: Callable[[MotorFeedback], None] | None = None,
        enable_first: bool = True,
    ) -> MotorFeedback:
        values = (
            position_rad,
            max_velocity_rad_s,
            position_tolerance_rad,
            velocity_tolerance_rad_s,
            timeout_s,
            command_rate_hz,
            kp,
            kd,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("Position, speed, tolerances, timeout, and command rate must be finite.")
        if not 0 < max_velocity_rad_s <= self.velocity_range_rad_s:
            raise ValueError(
                f"Speed must be in (0, {self.velocity_range_rad_s:g}] rad/s."
            )
        if position_tolerance_rad <= 0 or velocity_tolerance_rad_s < 0:
            raise ValueError("Position tolerance must be positive and velocity tolerance non-negative.")
        if timeout_s <= 0 or command_rate_hz <= 0:
            raise ValueError("Move timeout and command rate must be greater than zero.")
        if self.control_mode == CONTROL_MODE_MIT and (not 0 <= kp <= 500 or not 0 <= kd <= 5):
            raise ValueError("MIT gains must be in the ranges Kp [0, 500] and Kd [0, 5].")

        arbitration_id, command = self.encode_position_command(
            position_rad, max_velocity_rad_s, kp, kd
        )
        if enable_first:
            feedback = self.enable()
        else:
            if not self.enabled:
                raise RuntimeError("Motor is not enabled. Send the Enter Motor command first.")
            feedback = self.last_feedback
            if feedback is None or feedback.status != 1:
                self.enabled = False
                raise RuntimeError("Motor is not reporting Enable Mode; send Enter Motor again.")
        print(
            f"Enabled at {feedback.position_rad:.3f} rad; "
            f"moving to {position_rad:.3f} rad."
        )

        interval_s = 1.0 / command_rate_hz
        deadline = time.monotonic() + timeout_s
        stable_samples = 0

        while time.monotonic() < deadline:
            if cancel_event is not None and cancel_event.is_set():
                self.disable()
                raise MoveCancelledError("Move stopped; motor disabled and active torque removed.")

            cycle_started = time.monotonic()
            self._send(arbitration_id, command)
            feedback = self._read_feedback(timeout_s=interval_s)
            if feedback is not None:
                if feedback.status != 1:
                    self.enabled = False
                    raise RuntimeError(
                        f"Motor left Enable Mode or reported fault status {feedback.status}."
                    )

                position_error = abs(position_rad - feedback.position_rad)
                if (
                    position_error <= position_tolerance_rad
                    and abs(feedback.velocity_rad_s) <= velocity_tolerance_rad_s
                ):
                    stable_samples += 1
                    if stable_samples >= 3:
                        return feedback
                else:
                    stable_samples = 0

                if progress_callback is not None:
                    progress_callback(feedback)
                else:
                    print(
                        f"Position {feedback.position_rad:.3f} rad, "
                        f"speed {feedback.velocity_rad_s:.3f} rad/s, "
                        f"error {position_error:.3f} rad",
                        end="\r",
                        flush=True,
                    )

            remaining_s = interval_s - (time.monotonic() - cycle_started)
            if remaining_s > 0:
                time.sleep(remaining_s)

        raise TimeoutError(
            f"Move did not settle within {timeout_s:g} seconds. "
            "The drive remains enabled with its last target; check the mechanism and stop it safely."
        )


DamiaoV12 = DamiaoMotor