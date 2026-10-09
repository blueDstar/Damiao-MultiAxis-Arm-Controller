"""One command worker per axis; USB-CAN receive is owned by CANRouter."""

from __future__ import annotations

import math
import queue
import threading
import time
from concurrent.futures import Future
from dataclasses import dataclass, replace
from typing import Callable

from single_motor.damiao_v12 import (
    CONTROL_MODE_MIT, ConnectionCancelledError, MotorFeedback, PRESENT_PARAMETER_REGISTERS,
    PresentParameter,
)
from multi_motor.can_router import CANRouter, RoutedMotor
from multi_motor.trajectory import PositionTrajectory
from multi_motor.position_tracker import PositionTracker
from multi_motor.velocity_loop import VelocityLoop


JOG_MAX_SPEED_RAD_S = 30.0


@dataclass(frozen=True)
class Motion:
    kind: str
    position: float
    speed: float
    kp: float
    kd: float


class MotorSession:
    def __init__(
        self, motor: RoutedMotor, notify: Callable[[int, str, object], None],
        speed_cap: float = 30.0, rate_hz: float = 100.0, feedback_timeout: float = 1.0,
        move_timeout: float = 30.0,
    ) -> None:
        if not all(math.isfinite(v) and v > 0 for v in (speed_cap, rate_hz, feedback_timeout, move_timeout)):
            raise ValueError("Speed cap, command rate and timeouts must be positive finite numbers.")
        self.motor = motor
        self.notify = notify
        self.speed_cap = speed_cap
        self.interval = 1.0 / rate_hz
        self.feedback_timeout = feedback_timeout
        self.move_timeout = move_timeout
        self.settings = None
        self.zero_offset = 0.0
        self._zero_initialized = False
        self.trajectory: PositionTrajectory | None = None
        self.move_deadline = 0.0
        self.motion: Motion | None = None
        self.command_position = 0.0
        self.last_feedback_time = 0.0
        self.move_started = 0.0
        self.stable_samples = 0
        self.panel_mode = "position"
        self.pending_mode: str | None = None
        self.command_velocity = 0.0
        self.position_tracker = PositionTracker()
        self._last_motion_tick = 0.0
        self._last_poll = 0.0
        self._overspeed_samples = 0
        self.velocity_loop = VelocityLoop()
        self._position_stopping = False
        self._position_direction = 0.0
        self._jobs: queue.Queue = queue.Queue()
        self._cancel = threading.Event()
        self._closed = threading.Event()
        self._generation = 0
        self._submit_lock = threading.Lock()
        self._thread = threading.Thread(target=self._run, name=f"motor-{motor.can_id:02x}", daemon=True)
        self._thread.start()

    def _emit(self, event: str, value: object) -> None:
        self.notify(self.motor.can_id, event, value)

    @property
    def position_rad(self) -> float:
        if self.position_tracker.position_rad is not None:
            return self.position_tracker.position_rad
        return self.motor.last_feedback.position_rad if self.motor.last_feedback else 0.0

    def _accept_feedback(self, feedback: MotorFeedback) -> None:
        self.last_feedback_time = time.monotonic()
        total = self.position_tracker.update(feedback.position_rad, feedback.velocity_rad_s,
                                             feedback.received_at, self.motor.position_range_rad)
        self._emit("feedback", replace(feedback, position_rad=total, raw_position_rad=feedback.position_rad))

    def submit(self, action: str, *args) -> Future:
        future: Future = Future()
        with self._submit_lock:
            if self._closed.is_set():
                future.set_exception(RuntimeError("Motor session is closed."))
                return future
            if action == "disable":
                self._generation += 1
                self._cancel.set()
            elif action == "mode":
                # Invalidate pending targets/jogs from the previous control panel.
                self._generation += 1
            self._jobs.put((self._generation, future, action, args))
        return future

    def _validate_motion(self, position: float, speed: float, kp: float, kd: float, *, jog: bool = False) -> float:
        if not all(math.isfinite(v) for v in (position, speed, kp, kd)):
            raise ValueError("Angle, speed and gains must be finite.")
        limit = min(JOG_MAX_SPEED_RAD_S, self.motor.velocity_range_rad_s)
        if abs(speed) > limit and not math.isclose(abs(speed), limit, rel_tol=1e-12, abs_tol=1e-12):
            raise ValueError(f"Speed must be within [-{limit:g}, +{limit:g}] rad/s.")
        speed = math.copysign(min(abs(speed), limit), speed)
        if not 0 <= kp <= 500 or not 0 < kd <= 5:
            raise ValueError("Use Kp in [0, 500] and Kd in (0, 5].")
        # A multi-turn target is a host coordinate, never a MIT uint16 position.
        frame_position = 0.0 if self.motor.control_mode == CONTROL_MODE_MIT else position
        self.motor.encode_position_command(frame_position, speed, 0.0 if self.motor.control_mode == CONTROL_MODE_MIT else kp, kd)
        return speed

    def _execute(self, action: str, args: tuple):
        motor = self.motor
        if action == "disable":
            self.velocity_loop.reset()
            self.motion = None
            self.trajectory = None
            self.pending_mode = None
            self.command_velocity = 0.0
            # A failed verification must never send a guessed command ID.
            try:
                if self.settings is not None:
                    motor.disable()
            finally:
                motor.enabled = False
                self._cancel.clear()
            self._emit("state", "Disabled")
            self._emit("stale", None)
            return None
        if self._cancel.is_set():
            raise ConnectionCancelledError("Operation cancelled by Disable.")
        if action == "verify":
            if motor.enabled:
                raise RuntimeError("Disable this motor before reading its configuration.")
            self.settings = None
            had_position = self.position_tracker.position_rad is not None
            settings = motor.verify_configuration(cancel_event=self._cancel)
            reference_position = float(motor.read_register(0x51, cancel_event=self._cancel))
            self._poll_status(timeout_s=0.5)
            feedback = motor.last_feedback
            if feedback is None:
                raise TimeoutError("No status feedback to validate position decoding; Enable is unavailable.")
            if not math.isfinite(reference_position) or not PositionTracker.equivalent(reference_position, feedback.position_rad, motor.position_range_rad):
                raise RuntimeError(
                    f"Position decoding mismatch: XOUT={reference_position:.4f} rad, "
                    f"feedback={feedback.position_rad:.4f} rad; "
                    f"RX [{feedback.raw_payload.hex(' ').upper()}]. Enable is unavailable."
                )
            if not had_position:
                self.position_tracker.seed(reference_position, feedback.position_rad, feedback.velocity_rad_s, feedback.received_at)
                self._emit("feedback", replace(feedback, position_rad=reference_position))
            self.settings = settings
            self._emit("settings", self.settings)
            self._emit("state", "Verified / disabled")
            return self.settings
        if self.settings is None:
            raise RuntimeError("Verify this motor's CAN configuration first.")
        if action == "parameters":
            if motor.enabled:
                raise RuntimeError("Disable this motor before reading its parameter table.")
            result = [PresentParameter(address, name, motor.read_register(address, cancel_event=self._cancel), unit)
                      for address, name, unit in PRESENT_PARAMETER_REGISTERS]
            self._emit("parameters", result)
            return result
        if action == "mode":
            mode, = args
            if mode not in ("position", "manual"):
                raise ValueError("Choose Position or Manual Jog.")
            if motor.last_feedback is None or time.monotonic() - motor.last_feedback.received_at > self.feedback_timeout:
                self._poll_status()
            if motor.last_feedback is None:
                raise RuntimeError("Wait for fresh feedback before changing control panels.")
            if not motor.enabled:
                self._finish_mode(mode, motor.last_feedback)
            else:
                self.pending_mode = mode
                self.stable_samples = 0
                self.motion = Motion("switch", self.position_rad, 0.0, 0.0, 1.0)
                self._emit("state", "Stopping before mode switch")
            return None
        if action == "enable":
            if motor.enabled:
                return motor.last_feedback
            if motor.control_mode == CONTROL_MODE_MIT:
                # Replace any target retained by the driver before re-enabling.
                neutral_id, neutral = motor.encode_position_command(0.0, 0.0, 0.0, 0.0)
                motor._send(neutral_id, neutral)
                previous = motor._read_feedback(timeout_s=0.1)
                if previous is not None and previous.has_fault:
                    raise RuntimeError(f"Driver fault before Enable: {previous.status_text}.")
            feedback = motor.enable()
            if self._cancel.is_set():
                motor.disable()
                raise ConnectionCancelledError("Enable cancelled by Disable.")
            self._accept_feedback(feedback)
            self.command_position = self.position_rad
            self.last_feedback_time = time.monotonic()
            self.command_velocity = 0.0
            self.motion = Motion("idle", self.position_rad, 0.0, 0.0, 0.0)
            if not self._zero_initialized:
                self.zero_offset = self.position_rad
                self._zero_initialized = True
                self._emit("zero", self.zero_offset)
            self._emit("feedback", replace(feedback, position_rad=self.position_rad))
            self._emit("state", "Enabled / idle — no position hold")
            return feedback
        if not motor.enabled or motor.last_feedback is None:
            raise RuntimeError("Enable this motor before sending motion commands.")
        if action in ("target", "jog") and time.monotonic() - motor.last_feedback.received_at > self.feedback_timeout:
            raise RuntimeError("Motion requires fresh motor feedback.")
        if self.pending_mode is not None and action != "hold":
            raise RuntimeError("Wait until the motor stops and the mode switch completes.")
        if action == "zero":
            self.zero_offset = self.position_rad
            self._zero_initialized = True
            self._emit("zero", self.zero_offset)
            return self.zero_offset
        if action == "target":
            if self.panel_mode != "position":
                raise RuntimeError("Position commands are unavailable in Manual Jog.")
            relative_position, speed, kp, kd = args[:4]
            rotate_from_current = bool(args[4]) if len(args) > 4 else False
            position = relative_position + (self.position_rad if rotate_from_current else self.zero_offset)
            speed = self._validate_motion(position, speed, kp, kd)
            if speed == 0.0:
                return self._execute("hold", ())
            self.command_position = self.position_rad
            self.command_velocity = 0.0
            self.motion = Motion("target", position, abs(speed), kp, kd)
            self.trajectory = PositionTrajectory(self.command_position, position, speed)
            self.move_started = time.monotonic()
            self._position_stopping = False
            self._position_direction = math.copysign(1.0, position - self.position_rad)
            self.velocity_loop.reset()
            self.move_deadline = self.move_started + max(self.move_timeout, self.trajectory.duration_s * 2 + 5.0)
            self.stable_samples = 0
            self._overspeed_samples = 0
            self._emit("trajectory", self.trajectory)
            self._emit("state", "Moving to target")
        elif action == "jog":
            if self.panel_mode != "manual":
                raise RuntimeError("Switch to Manual Jog before jogging.")
            speed, kp, kd = args
            speed = self._validate_motion(motor.last_feedback.position_rad, speed, kp, kd, jog=True)
            if speed == 0.0:
                return self._execute("hold", ())
            self.command_position = self.position_rad
            self.command_velocity = speed
            self.velocity_loop.reset()
            self._last_motion_tick = time.monotonic()
            self._overspeed_samples = 0
            self.motion = Motion("jog", self.command_position, speed, kp, kd)
            self._emit("state", "Jog +" if speed > 0 else "Jog −")
        elif action == "hold":
            self.velocity_loop.reset()
            if self.motion is not None:
                m = self.motion
                self.stable_samples = 0
                self.command_velocity = 0.0
                self.motion = Motion("switch" if self.pending_mode else "brake", self.position_rad, 0.0, 0.0, max(m.kd, 1.0))
                self._emit("state", "Stop requested / zero velocity immediately")
        else:
            raise ValueError(f"Unknown motor action: {action}")

    def _finish_mode(self, mode: str, feedback: MotorFeedback) -> None:
        self.panel_mode = mode
        self.pending_mode = None
        self.command_position = self.position_rad
        self.command_velocity = 0.0
        # Each control-mode entry starts from the stopped physical position.
        # Notebook view changes never call this method.
        self.zero_offset = self.position_rad
        self._zero_initialized = True
        self.velocity_loop.reset()
        self.trajectory = None
        self.motion = Motion("idle", self.position_rad, 0.0, 0.0, 0.0) if self.motor.enabled else None
        self._emit("mode", (mode, self.zero_offset))
        self._emit("state", "Manual Jog ready / zero at current position" if mode == "manual" else "Position ready / idle")

    def _poll_status(self, timeout_s: float | None = None) -> None:
        motor = self.motor
        motor._send(0x7FF, bytes((motor.can_id, 0, 0xCC, 0, 0, 0, 0, 0)))
        feedback = motor._read_feedback(timeout_s=self.interval if timeout_s is None else timeout_s)
        if feedback is not None:
            self._accept_feedback(feedback)

    def _tick(self) -> None:
        m = self.motion
        motor = self.motor
        if self._cancel.is_set():
            return
        if m is None:
            if self.settings is not None and time.monotonic() - self._last_poll >= 0.25:
                self._last_poll = time.monotonic()
                self._poll_status()
            return
        if not motor.enabled:
            return
        position, velocity, kp = m.position, m.speed, m.kp
        if m.kind == "idle":
            # Kp=Kd=0 means neither the position nor velocity fields apply torque.
            position, velocity, kp = 0.0, 0.0, 0.0
            if motor.control_mode != CONTROL_MODE_MIT:
                self._poll_status()
                if time.monotonic() - self.last_feedback_time > self.feedback_timeout:
                    raise TimeoutError("Motor status timed out.")
                return
        elif m.kind in ("target", "hold"):
            if self.trajectory is None:
                raise RuntimeError("Missing position trajectory.")
            error = m.position - self.position_rad
            # Leave one position-encoding step for measurement rounding.
            tolerance = max(0.001, 0.005 - 2 * motor.position_range_rad / 65535)
            self.command_position = m.position
            if motor.control_mode == CONTROL_MODE_MIT:
                # Position is controlled by measured remaining rotation, not by
                # chasing an elapsed-time reference that can outrun the motor.
                position, kp = 0.0, 0.0
                if abs(error) <= tolerance or self._position_direction * error <= 0:
                    self._position_stopping = True
                # Never re-enable position Kp with a wrapped/raw CAN angle: the
                # driver's internal coordinate may be many revolutions away.
                # Once the endpoint is reached/crossed, brake and latch Stop.
                velocity = 0.0 if self._position_stopping or m.kind == "hold" else math.copysign(min(m.speed, abs(error) / 0.1), error)
            else:
                # Native Pos-Vel uses float32 positions, rather than MIT uint16.
                position, velocity = m.position, m.speed
            self.command_velocity = velocity if motor.control_mode == CONTROL_MODE_MIT else math.copysign(m.speed, error)
        elif m.kind in ("jog", "brake", "switch", "manual_stop"):
            velocity = m.speed if m.kind == "jog" else 0.0
            self.command_velocity = velocity
            if motor.control_mode == CONTROL_MODE_MIT:
                position, kp = 0.0, 0.0
            else:
                if m.kind == "jog":
                    tick_time = time.monotonic()
                    self.command_position += velocity * max(0.0, tick_time - self._last_motion_tick)
                    self._last_motion_tick = tick_time
                else:
                    self.command_position = self.position_rad
                position, velocity = self.command_position, max(abs(velocity), min(0.2, motor.velocity_range_rad_s))
        reference_velocity = velocity if motor.control_mode == CONTROL_MODE_MIT else self.command_velocity
        if motor.control_mode == CONTROL_MODE_MIT:
            velocity = self.velocity_loop.command(reference_velocity, motor.last_feedback.velocity_rad_s,
                                                   motor.last_feedback.received_at, motor.velocity_range_rad_s)
        self._emit("reference", (m.position if m.kind in ("target", "hold") else None, reference_velocity, velocity))
        arbitration_id, payload = motor.encode_position_command(position, velocity, kp, m.kd)
        if m.kind == "target":
            if time.monotonic() - self.last_feedback_time > self.feedback_timeout:
                raise TimeoutError("Position motion requires fresh feedback; no further setpoint sent.")
        motor._send(arbitration_id, payload)
        self._emit("tx", (arbitration_id, payload))
        feedback = motor._read_feedback(timeout_s=self.interval)
        now = time.monotonic()
        if feedback is not None:
            self._accept_feedback(feedback)
            if not feedback.is_enabled:
                raise RuntimeError(f"Driver reported {feedback.status_text} (status 0x{feedback.status_code:X}); command stream stopped.")
            if m.kind in ("target", "jog"):
                excessive = abs(feedback.velocity_rad_s) > max(abs(m.speed) * 2, abs(m.speed) + 0.15)
                self._overspeed_samples = self._overspeed_samples + 1 if excessive else 0
                if self._overspeed_samples >= 3:
                    raise RuntimeError("Measured speed exceeds requested motion speed; motion stopped.")
            if m.kind in ("brake", "switch"):
                stopped = (motor.control_mode != CONTROL_MODE_MIT or self.command_velocity == 0.0) and abs(feedback.velocity_rad_s) <= max(0.03, 4 * motor.velocity_range_rad_s / 4095)
                self.stable_samples = self.stable_samples + 1 if stopped else 0
                if self.stable_samples >= 3:
                    if self.pending_mode is not None:
                        self._finish_mode(self.pending_mode, feedback)
                    else:
                        self.motion = Motion("manual_stop", self.position_rad, 0.0, 0.0, max(m.kd, 1.0))
                        self._emit("state", "Stopped / zero velocity / no position recoil")
            if m.kind == "target":
                if motor.control_mode == CONTROL_MODE_MIT:
                    settled = self._position_stopping and abs(feedback.velocity_rad_s) <= max(0.03, 4 * motor.velocity_range_rad_s / 4095)
                else:
                    settled = abs(m.position - self.position_rad) <= 0.005 and abs(feedback.velocity_rad_s) <= max(0.03, 4 * motor.velocity_range_rad_s / 4095)
                self.stable_samples = self.stable_samples + 1 if settled else 0
                if self.stable_samples >= 3:
                    self.motion = Motion("hold", m.position, m.speed, m.kp, m.kd)
                    self._emit("state", f"Position stopped; measured angle error {self.position_rad - m.position:+.4f} rad")
        if now - self.last_feedback_time > self.feedback_timeout:
            raise TimeoutError("Motor feedback timed out; Disable requested.")
        if m.kind == "target" and self.motion.kind == "target" and now > self.move_deadline:
            raise TimeoutError("Target did not settle before the move timeout; Disable requested.")

    def _fail(self, exc: Exception) -> None:
        self.motion = None
        self.trajectory = None
        self.pending_mode = None
        self.command_velocity = 0.0
        self._overspeed_samples = 0
        try:
            if self.settings is not None:
                self.motor.disable()
        except Exception as disable_error:
            self._emit("error", f"{exc} Disable could not be delivered: {disable_error}")
        else:
            self._emit("error", str(exc))
        finally:
            self.motor.enabled = False
            self._emit("state", "Stopped after error; check driver")

    def _run(self) -> None:
        next_tick = time.monotonic()
        while not self._closed.is_set():
            wait = max(0.0, next_tick - time.monotonic()) if self.motion is not None else 0.05
            try:
                generation, future, action, args = self._jobs.get(timeout=wait)
            except queue.Empty:
                pass
            else:
                if not future.set_running_or_notify_cancel():
                    continue
                if generation != self._generation:
                    future.set_exception(ConnectionCancelledError("Operation superseded by Disable."))
                    continue
                try:
                    result = self._execute(action, args)
                except Exception as exc:
                    if action == "enable":
                        self._fail(exc)
                    future.set_exception(exc)
                else:
                    future.set_result(result)
            if time.monotonic() >= next_tick:
                started = time.monotonic()
                try:
                    self._tick()
                except Exception as exc:
                    self._fail(exc)
                next_tick = started + self.interval
        while True:
            try:
                _, future, _, _ = self._jobs.get_nowait()
            except queue.Empty:
                break
            if not future.done():
                future.set_exception(RuntimeError("Motor session closed."))

    def close(self) -> None:
        if self._closed.is_set():
            return
        try:
            self.submit("disable").result(timeout=2.5)
        finally:
            self._closed.set()
            self._cancel.set()
            self._thread.join(timeout=2.0)
            if self._thread.is_alive():
                raise RuntimeError("Motor worker did not stop.")


class MultiMotorController:
    def __init__(self, bus, motor_ids, notify=lambda *_: None, **session_options) -> None:
        self.router = CANRouter(bus)
        self.sessions: dict[int, MotorSession] = {}
        try:
            # Validate every ID before starting motor workers.
            motors = [self.router.add_motor(can_id, master_id) for can_id, master_id in motor_ids]
            for motor in motors:
                self.sessions[motor.can_id] = MotorSession(motor, notify, **session_options)
        except Exception:
            self.close()
            raise

    def disable_all(self) -> list[Future]:
        return [session.submit("disable") for session in self.sessions.values()]

    def close(self) -> None:
        errors = []
        # Request every stop first so one slow axis cannot keep another moving.
        self.disable_all()
        for session in self.sessions.values():
            try:
                session.close()
            except Exception as exc:
                errors.append(str(exc))
        self.router.close()
        if errors:
            raise RuntimeError("; ".join(errors))
