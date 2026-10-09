"""Desktop controls for independent motors sharing one Damiao USB2CAN adapter."""

from __future__ import annotations

import math
import os
import queue
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk

from dotenv import load_dotenv
from serial.tools import list_ports

from multi_motor.controller import JOG_MAX_SPEED_RAD_S, MultiMotorController
from multi_motor.trajectory import PositionTrajectory
from multi_motor.plots import MotorPlots
from multi_motor.theme import BACKGROUND, INPUT, NEON, WHITE, MUTED, BORDER, apply_theme
from single_motor.move_to_angle import validate_usb2can_port
from single_motor.usb2can_serial import DamiaoUSB2CANBus, USB2CAN_CAN_BAUDS_KBPS


load_dotenv(Path(__file__).resolve().parents[1] / ".env")


def display_number(value: float, quantum: float) -> str:
    """Suppress only one quantization step and negative rounded zero in the UI."""
    return f"{0.0 if abs(value) <= max(quantum, 0.0005) else value:.3f}"


class MotorPanel:
    def __init__(self, app: MultiMotorApp, parent, number: int) -> None:
        self.app = app
        self.can_id = number
        self.session = None
        self.settings = None
        self.enabled = False
        self.pending = 0
        self.zero = 0.0
        self.feedback = None
        self.saved_target = None
        self.jogging = False
        self.last_error = None
        self.control_mode = "position"
        self.mode_pending = False
        self.frame = ttk.LabelFrame(parent, text=f"MOTOR {number:02d}", padding=12, style="Motor.TLabelframe")
        self.frame.grid(row=(number - 1) // 2, column=(number - 1) % 2, sticky="new", padx=10, pady=10)
        self.id_var = tk.StringVar(value=f"0x{number:02X}")
        self.master_var = tk.StringVar(value=f"0x{number + 0x10:02X}")
        self.state = tk.StringVar(value="Chưa xác minh")
        self.info = tk.StringVar(value="Firmware / PMAX / VMAX / TMAX: chưa đọc")
        self.live = tk.StringVar(value="Vị trí: —   Tốc độ: —   Mô-men: —")
        self.zero_text = tk.StringVar(value="Zero phần mềm: 0.000 rad")
        self.hex_text = tk.StringVar(value="Chưa gửi lệnh")
        self.rx_text = tk.StringVar(value="RX: chờ phản hồi CAN")
        self.mode_text = tk.StringVar(value="POSITION · Góc đích")
        self.jog_speed = tk.StringVar(value="0.2")
        self.jog_unit = tk.StringVar(value="rad/s")
        self.jog_kd = tk.StringVar(value="1")
        self.jog_limit_text = tk.StringVar(value="JOG −30 … +30 rad/s · 0 = dừng")
        self.metric_position = tk.StringVar(value="—")
        self.metric_velocity = tk.StringVar(value="—")
        self.metric_torque = tk.StringVar(value="—")
        self.thermal = tk.StringVar(value="DRIVER  — °C     MOTOR  — °C     STATUS  —")
        self.badge = tk.StringVar(value="● CHƯA XÁC MINH")
        self.angle = tk.StringVar(value="0")
        self.angle_unit = tk.StringVar(value="deg")
        self.target_kind = tk.StringVar(value="relative")
        self.saved_target_kind = None
        self.speed = tk.StringVar(value="0.2")
        self.speed_unit = tk.StringVar(value="rad/s")
        self.kp = tk.StringVar(value="0")
        self.kd = tk.StringVar(value="1")
        self.trajectory_info = tk.StringVar(value="Góc tổng nhiều vòng · ±30 rad/s · 0 = dừng")
        self._previous_units = {"angle": "deg", "speed": "rad/s", "jog": "rad/s"}
        self._converting_units = False
        self.buttons = {}
        self.inputs = []

        ids = ttk.Frame(self.frame)
        ids.pack(fill="x")
        ttk.Label(ids, text="CAN ID").pack(side="left")
        self.id_entry = ttk.Entry(ids, textvariable=self.id_var, width=8)
        self.id_entry.pack(side="left", padx=(5, 12))
        ttk.Label(ids, text="Master ID").pack(side="left")
        self.master_entry = ttk.Entry(ids, textvariable=self.master_var, width=8)
        self.master_entry.pack(side="left", padx=5)
        self.state_label = ttk.Label(ids, textvariable=self.badge, style="Badge.TLabel")
        self.state_label.pack(side="right")
        ttk.Label(self.frame, textvariable=self.info, style="Muted.TLabel", wraplength=570).pack(anchor="w", pady=(6, 8))

        metrics = ttk.Frame(self.frame)
        metrics.pack(fill="x")
        for column, (title, variable) in enumerate((("VỊ TRÍ / rad", self.metric_position), ("TỐC ĐỘ / rad/s", self.metric_velocity), ("MÔ-MEN / Nm", self.metric_torque))):
            metrics.columnconfigure(column, weight=1, uniform="metrics")
            tile = ttk.Frame(metrics, style="Metric.TFrame", padding=(12, 10))
            tile.grid(row=0, column=column, sticky="ew", padx=(0 if column == 0 else 5, 0))
            ttk.Label(tile, text=title, style="Metric.TLabel").pack(anchor="w")
            ttk.Label(tile, textvariable=variable, style="MetricValue.TLabel").pack(anchor="w", pady=(5, 0))
        ttk.Label(self.frame, textvariable=self.thermal, style="Muted.TLabel", wraplength=530).pack(anchor="w", pady=(8, 2))
        ttk.Label(self.frame, textvariable=self.zero_text, style="Muted.TLabel").pack(anchor="w", pady=(0, 8))

        actions = ttk.Frame(self.frame)
        actions.pack(fill="x", pady=(0, 6))
        for column in range(3):
            actions.columnconfigure(column, weight=1, uniform="actions")
        for index, (name, title, command) in enumerate((
            ("verify", "Xác minh motor", lambda: self.dispatch("verify")),
            ("enable", "Enable", lambda: self.dispatch("enable")),
            ("zero", "Lưu zero", self.save_zero),
            ("parameters", "Đọc tham số", lambda: self.dispatch("parameters")),
            ("disable", "Disable motor", lambda: self.dispatch("disable")),
        )):
            button = ttk.Button(actions, text=title, command=command, style="Primary.TButton" if name == "enable" else "TButton")
            button.grid(row=index // 3, column=index % 3, sticky="ew", padx=(0, 5), pady=(0, 5))
            self.buttons[name] = button

        modes = ttk.Frame(self.frame)
        modes.pack(fill="x", pady=(0, 7))
        ttk.Label(modes, textvariable=self.mode_text, style="Section.TLabel").pack(side="left")
        self.buttons["mode"] = ttk.Button(modes, text="SANG MANUAL JOG", command=self.switch_mode)
        self.buttons["mode"].pack(side="right")
        self.position_controls = ttk.Frame(self.frame)
        self.position_controls.pack(fill="x")
        self.manual_controls = ttk.Frame(self.frame)
        target_mode = ttk.Frame(self.position_controls)
        target_mode.pack(fill="x", pady=(0, 7))
        ttk.Radiobutton(target_mode, text="Quay thêm từ hiện tại", variable=self.target_kind, value="relative").pack(side="left", padx=(0, 14))
        ttk.Radiobutton(target_mode, text="Tới góc từ zero", variable=self.target_kind, value="absolute").pack(side="left")
        values = ttk.Frame(self.position_controls)
        values.pack(fill="x", pady=(0, 6))
        for column in range(4):
            values.columnconfigure(column, weight=2 if column < 2 else 1)
        for index, (label, variable, width) in enumerate((("GÓC (XEM CHẾ ĐỘ)", self.angle, 5), ("TỐC ĐỘ", self.speed, 5), ("Kp MIT = 0", self.kp, 4), ("Kd", self.kd, 4))):
            field = ttk.Frame(values)
            field.grid(row=0, column=index, sticky="ew", padx=(0, 6), pady=(0, 5))
            ttk.Label(field, text=label, style="Muted.TLabel").pack(anchor="w", pady=(0, 4))
            line = ttk.Frame(field)
            line.pack(fill="x")
            entry = ttk.Entry(line, textvariable=variable, width=width)
            entry.pack(side="left", fill="x", expand=True, padx=(0, 5))
            self.inputs.append(entry)
            if variable is self.kp:
                entry.configure(state="readonly")
            if variable is self.angle:
                unit = ttk.Combobox(line, textvariable=self.angle_unit, values=("deg", "rad"), width=4, state="readonly")
                unit.pack(side="left")
            elif variable is self.speed:
                unit = ttk.Combobox(line, textvariable=self.speed_unit, values=("rad/s", "rpm"), width=5, state="readonly")
                unit.pack(side="left")

        ttk.Label(self.position_controls, textvariable=self.trajectory_info, style="Muted.TLabel", wraplength=550).pack(anchor="w", pady=(0, 7))
        motion = ttk.Frame(self.position_controls)
        motion.pack(fill="x", pady=(0, 8))
        for column in range(6):
            motion.columnconfigure(column, weight=1, uniform="motion")
        for index, (name, title, command) in enumerate((
            ("save", "Lưu đích / xem CAN", self.save_target),
            ("target", "Gửi đích", self.send_target),
            ("hold", "DỪNG CHUYỂN ĐỘNG", lambda: self.dispatch("hold")),
        )):
            button = ttk.Button(motion, text=title, command=command, style="Primary.TButton" if name == "target" else "TButton")
            button.grid(row=0 if index < 2 else 1, column=index * 3 if index < 2 else 0, columnspan=3 if index < 2 else 6, sticky="ew", padx=(0, 5), pady=(0, 5))
            self.buttons[name] = button
        manual_values = ttk.Frame(self.manual_controls)
        manual_values.pack(fill="x", pady=(0, 8))
        ttk.Label(manual_values, text="TỐC ĐỘ JOG").pack(side="left", padx=(0, 6))
        ttk.Entry(manual_values, textvariable=self.jog_speed, width=7).pack(side="left", padx=(0, 5))
        ttk.Combobox(manual_values, textvariable=self.jog_unit, values=("rad/s", "rpm"), state="readonly", width=5).pack(side="left", padx=(0, 12))
        ttk.Label(manual_values, text="Kd").pack(side="left", padx=(0, 5))
        ttk.Entry(manual_values, textvariable=self.jog_kd, width=5).pack(side="left")
        ttk.Label(self.manual_controls, textvariable=self.jog_limit_text, style="Muted.TLabel").pack(anchor="w", pady=(0, 3))
        ttk.Label(self.manual_controls, text="Bấm một lần để chạy liên tục · Bấm DỪNG JOG để dừng", style="Muted.TLabel").pack(anchor="w", pady=(0, 7))
        jog_buttons = ttk.Frame(self.manual_controls)
        jog_buttons.pack(fill="x", pady=(0, 8))
        jog_buttons.columnconfigure(0, weight=1, uniform="jog")
        jog_buttons.columnconfigure(1, weight=1, uniform="jog")
        for column, (name, title, direction) in enumerate((("jog_minus", "◀  CHẠY JOG −", -1), ("jog_plus", "CHẠY JOG +  ▶", 1))):
            button = ttk.Button(jog_buttons, text=title, command=lambda d=direction: self.start_jog(d))
            button.grid(row=0, column=column, sticky="ew", padx=(0, 5), pady=(0, 5))
            self.buttons[name] = button
        self.buttons["jog_stop"] = ttk.Button(jog_buttons, text="DỪNG JOG", command=self.stop_motion)
        self.buttons["jog_stop"].grid(row=1, column=0, sticky="ew", padx=(0, 5))
        self.buttons["jog_signed"] = ttk.Button(jog_buttons, text="CHẠY TỐC ĐỘ ĐÃ NHẬP", style="Primary.TButton", command=lambda: self.start_jog(None))
        self.buttons["jog_signed"].grid(row=1, column=1, sticky="ew", padx=(0, 5))
        ttk.Label(self.frame, textvariable=self.hex_text, style="Hex.TLabel", wraplength=505).pack(fill="x", pady=(0, 6))
        rx_label = ttk.Label(self.frame, textvariable=self.rx_text, style="Hex.TLabel", wraplength=505)
        rx_label.pack(fill="x", pady=(0, 6))
        rx_label.bind("<Double-Button-1>", lambda _: self.copy_rx())
        ttk.Label(self.frame, textvariable=self.state, style="Muted.TLabel", wraplength=530).pack(anchor="w")
        for variable in (self.angle, self.angle_unit, self.speed, self.speed_unit, self.kp, self.kd, self.target_kind):
            variable.trace_add("write", lambda *_: self.update_buttons())
        self.angle_unit.trace_add("write", lambda *_: self.convert_units("angle"))
        self.speed_unit.trace_add("write", lambda *_: self.convert_units("speed"))
        self.jog_unit.trace_add("write", lambda *_: self.convert_units("jog"))
        self.update_buttons()

    def convert_units(self, kind: str) -> None:
        value_var, unit_var = {"angle": (self.angle, self.angle_unit), "speed": (self.speed, self.speed_unit), "jog": (self.jog_speed, self.jog_unit)}[kind]
        previous, selected = self._previous_units[kind], unit_var.get()
        self._previous_units[kind] = selected
        if previous == selected or self._converting_units:
            return
        try:
            value = float(value_var.get())
            if not math.isfinite(value):
                return
            if kind == "angle":
                value = math.radians(value) if selected == "rad" else math.degrees(value)
            else:
                value = value * 60 / (2 * math.pi) if selected == "rpm" else value * 2 * math.pi / 60
            self._converting_units = True
            value_var.set(f"{value:.15g}")
        except ValueError:
            pass
        finally:
            self._converting_units = False
        self.update_buttons()

    def target_matches_saved(self) -> bool:
        if self.saved_target is None:
            return False
        if self.saved_target_kind != self.target_kind.get():
            return False
        return all(math.isclose(a, b, rel_tol=1e-12, abs_tol=1e-12) for a, b in zip(self.values(), self.saved_target))

    def show_trajectory(self, trajectory: PositionTrajectory) -> None:
        start = trajectory.start_rad - self.zero
        target = trajectory.target_rad - self.zero
        self.trajectory_info.set(
            f"{math.degrees(start):.2f}° → {math.degrees(target):.2f}° "
            f"({target:.4f} rad) · v={trajectory.velocity_rad_s:+.3f} rad/s "
            f"({trajectory.velocity_rad_s * 60 / (2 * math.pi):+.3f} rpm) · T={trajectory.duration_s:.3f} s lý tưởng"
        )

    def values(self) -> tuple[float, float, float, float]:
        angle = float(self.angle.get())
        speed = float(self.speed.get())
        if self.angle_unit.get() == "deg":
            angle = math.radians(angle)
        if self.speed_unit.get() == "rpm":
            speed *= 2 * math.pi / 60
        kp, kd = float(self.kp.get()), float(self.kd.get())
        if not all(math.isfinite(v) for v in (angle, speed, kp, kd)):
            raise ValueError("Góc, tốc độ, Kp và Kd phải là số hữu hạn.")
        if self.session is None or self.settings is None:
            raise ValueError("Xác minh motor trước khi chuẩn bị lệnh.")
        origin = self.session.position_rad if self.target_kind.get() == "relative" else self.zero
        speed = self.session._validate_motion(angle + origin, speed, kp, kd)
        return angle, speed, kp, kd

    def dispatch(self, action: str, *args) -> None:
        if self.session is None or self.app.shutting_down:
            return
        if self.pending and action not in ("disable", "hold"):
            return
        if action == "disable":
            self.jogging = False
        self.pending += 1
        self.update_buttons()
        future = self.session.submit(action, *args)
        future.add_done_callback(lambda result: self.app.events.put((self.can_id, "done", (action, result))))

    def save_target(self) -> None:
        if self.control_mode != "position" or self.mode_pending:
            return
        try:
            target = self.values()
            self.saved_target_kind = self.target_kind.get()
            if target[1] == 0.0:
                self.saved_target = target
                self.trajectory_info.set("Tốc độ 0: gửi yêu cầu giảm tốc/dừng, không đi tới góc mới")
                self.hex_text.set("Yêu cầu dừng · TX thực tế giảm tốc từ vận tốc hiện tại")
                self.update_buttons()
                return
            start = self.session.position_rad if self.session.motor.last_feedback is not None else self.zero
            endpoint = target[0] + (start if self.target_kind.get() == "relative" else self.zero)
            trajectory = PositionTrajectory(start, endpoint, target[1])
            arbitration_id, payload = self.session.motor.encode_position_command(
                0.0 if self.settings.control_mode == 1 else endpoint,
                trajectory.velocity_rad_s if self.settings.control_mode == 1 else abs(target[1]),
                0.0 if self.settings.control_mode == 1 else target[2], target[3]
            )
        except (ValueError, RuntimeError) as exc:
            self.app.show_error(str(exc))
            return
        self.saved_target = target
        self.show_trajectory(trajectory)
        self.hex_text.set(f"CAN 0x{arbitration_id:03X} [{payload.hex(' ').upper()}] · góc tổng được đếm từ feedback")
        self.app.log(f"Motor 0x{self.can_id:02X}: lưu đích {target[0]:.4f} rad từ zero.")
        self.update_buttons()

    def send_target(self) -> None:
        if self.control_mode != "position" or self.mode_pending:
            return
        try:
            if not self.target_matches_saved():
                raise ValueError("Lưu lại đích sau khi chỉnh thông số.")
        except (ValueError, RuntimeError) as exc:
            self.app.show_error(str(exc))
            return
        self.dispatch("target", *self.saved_target, self.saved_target_kind == "relative")

    def save_zero(self) -> None:
        self.saved_target = None
        self.dispatch("zero")

    def start_jog(self, direction: int | None) -> None:
        if self.control_mode != "manual" or self.mode_pending or not self.enabled or self.pending or self.app.shutting_down:
            return
        if self.feedback is None or time.monotonic() - self.feedback.received_at > 0.6:
            return
        try:
            speed, kd = float(self.jog_speed.get()), float(self.jog_kd.get())
            if self.jog_unit.get() == "rpm":
                speed *= 2 * math.pi / 60
            limit = min(JOG_MAX_SPEED_RAD_S, self.settings.velocity_range_rad_s)
            if not math.isfinite(speed) or abs(speed) > limit:
                raise ValueError(f"Tốc độ Jog phải trong [{-limit:g}, +{limit:g}] rad/s.")
            if not math.isfinite(kd) or not 0 < kd <= 5:
                raise ValueError("Kd Jog phải trong (0, 5].")
        except (ValueError, RuntimeError) as exc:
            self.app.show_error(str(exc))
            return
        if speed == 0.0:
            self.stop_motion()
            return
        self.jogging = True
        requested_speed = speed if direction is None else direction * abs(speed)
        self.dispatch("jog", requested_speed, 2.0, kd)

    def switch_mode(self) -> None:
        if self.pending or self.mode_pending or self.session is None:
            return
        self.jogging = False
        self.mode_pending = True
        self.saved_target = None
        self.hex_text.set("TX: dừng luồng cũ trước khi chuyển chế độ")
        self.dispatch("mode", "manual" if self.control_mode == "position" else "position")

    def copy_rx(self) -> None:
        self.app.root.clipboard_clear()
        self.app.root.clipboard_append(self.rx_text.get())
        self.app.log(self.rx_text.get())

    def stop_motion(self) -> None:
        self.jogging = False
        self.dispatch("hold")

    def stop_jog(self) -> None:
        if self.jogging:
            self.jogging = False
            self.dispatch("hold")

    def update_buttons(self) -> None:
        if self._converting_units:
            return
        connected = self.session is not None and not self.app.shutting_down
        ready = connected and not self.pending
        position = self.control_mode == "position" and not self.mode_pending
        manual = self.control_mode == "manual" and not self.mode_pending
        fresh = self.feedback is not None and time.monotonic() - self.feedback.received_at <= 0.6
        try:
            saved = self.target_matches_saved()
        except (ValueError, RuntimeError):
            saved = False
        states = {
            "verify": ready and not self.enabled,
            "enable": ready and self.settings is not None and not self.enabled,
            "parameters": ready and self.settings is not None and not self.enabled,
            "disable": connected,
            "zero": ready and self.enabled,
            "save": ready and self.settings is not None and position,
            "target": ready and self.enabled and saved and position and fresh,
            "hold": ready and self.enabled and position,
            "jog_minus": ready and self.enabled and manual and fresh,
            "jog_plus": ready and self.enabled and manual and fresh,
            "jog_stop": connected and self.enabled and manual,
            "jog_signed": ready and self.enabled and manual and fresh,
            "mode": ready and self.settings is not None and not self.mode_pending and fresh,
        }
        for name, button in self.buttons.items():
            button.configure(state="normal" if states[name] else "disabled")
        if self.enabled:
            self.badge.set("● ENABLED")
            self.state_label.configure(style="Enabled.Badge.TLabel")
        elif self.last_error is not None:
            self.badge.set("● LỖI / DỪNG")
            self.state_label.configure(style="Error.Badge.TLabel")
        elif self.settings is not None:
            self.badge.set("● DISABLED")
            self.state_label.configure(style="Badge.TLabel")
        else:
            self.badge.set("● CHƯA XÁC MINH")
            self.state_label.configure(style="Badge.TLabel")
        ids_state = "disabled" if self.app.controller is not None or self.app.connecting else "normal"
        self.id_entry.configure(state=ids_state)
        self.master_entry.configure(state=ids_state)

    def event(self, event: str, value) -> None:
        if event == "settings":
            self.settings = value
            self.last_error = None
            self.saved_target = None
            mode = "MIT" if value.control_mode == 1 else "Position-Velocity"
            self.info.set(f"FW {value.firmware_version}.{value.sub_version:03d} | {mode} | PMAX ±{value.position_range_rad:g} rad | VMAX {value.velocity_range_rad_s:g} rad/s | TMAX {value.torque_range_nm:g} Nm")
            jog_limit = min(JOG_MAX_SPEED_RAD_S, value.velocity_range_rad_s)
            self.jog_limit_text.set(f"JOG −{jog_limit:g} … +{jog_limit:g} rad/s · 0 = dừng")
            self.trajectory_info.set(f"Góc tổng nhiều vòng · ±{jog_limit:g} rad/s · 0 = dừng")
        elif event == "feedback":
            self.feedback = value
            self.enabled = value.is_enabled and self.session.motor.enabled
            if value.has_fault:
                self.last_error = value.status_text
            elif self.enabled:
                self.last_error = None
            self.metric_position.set(display_number(value.position_rad - self.zero, 2 * getattr(self.settings, "position_range_rad", 12.5) / 65535))
            self.metric_velocity.set(display_number(value.velocity_rad_s, 2 * getattr(self.settings, "velocity_range_rad_s", 30.0) / 4095))
            self.metric_torque.set(display_number(value.torque_nm, 2 * getattr(self.settings, "torque_range_nm", 10.0) / 4095))
            if value.raw_payload:
                self.rx_text.set(f"RX 0x{self.session.motor.master_id:03X} [{value.raw_payload.hex(' ').upper()}]")
            self.thermal.set(f"DRIVER {value.driver_temperature_c} °C    MOTOR {value.motor_temperature_c} °C    STATUS {value.status_text} / 0x{value.status_code:X}")
            self.live.set(f"Pos {value.position_rad - self.zero:.4f} rad từ zero | Vel {value.velocity_rad_s:.3f} rad/s | Torque {value.torque_nm:.3f} Nm | Status {value.status_text} (0x{value.status_code:X})")
        elif event == "state":
            self.enabled = self.session.motor.enabled
            self.state.set(str(value))
            self.app.log(f"Motor 0x{self.can_id:02X}: {value}")
        elif event == "zero":
            self.zero = value
            self.saved_target = None
            self.trajectory_info.set("Zero đã cập nhật · Lưu lại đích trước khi gửi")
            self.zero_text.set(f"Zero phần mềm: {value:.4f} rad (riêng motor này)")
            if self.feedback is not None:
                self.event("feedback", self.feedback)
        elif event == "mode":
            self.control_mode, self.zero = value
            self.mode_pending = False
            self.saved_target = None
            self.angle.set("0")
            self.position_controls.pack_forget()
            self.manual_controls.pack_forget()
            # Keep the selected controls above TX/RX and status labels.
            controls = self.manual_controls if self.control_mode == "manual" else self.position_controls
            controls.pack(fill="x", after=self.buttons["mode"].master)
            self.mode_text.set("MANUAL JOG · Dừng / zero tại vị trí hiện tại" if self.control_mode == "manual" else "POSITION · Góc đích")
            self.buttons["mode"].configure(text="SANG POSITION" if self.control_mode == "manual" else "SANG MANUAL JOG")
            self.hex_text.set("TX: chế độ mới đang dừng, chờ thao tác")
            self.trajectory_info.set("Góc tổng nhiều vòng · ±30 rad/s · 0 = dừng")
            self.zero_text.set(f"Zero phần mềm: {self.zero:.4f} rad")
            if self.feedback is not None:
                self.event("feedback", self.feedback)
        elif event == "tx":
            arbitration_id, payload = value
            self.hex_text.set(f"TX CAN 0x{arbitration_id:03X} [{payload.hex(' ').upper()}]")
        elif event == "trajectory":
            self.show_trajectory(value)
        elif event == "parameters":
            self.app.show_parameters(self.can_id, value)
        elif event == "error":
            self.enabled = False
            self.jogging = False
            self.last_error = str(value)
            self.app.log(f"Motor 0x{self.can_id:02X} ERROR: {value}")
            self.state.set(str(value))
            self.mode_pending = False
        elif event == "done":
            action, future = value
            self.pending = max(0, self.pending - 1)
            try:
                future.result()
            except Exception as exc:
                self.last_error = str(exc)
                self.app.log(f"Motor 0x{self.can_id:02X} {action}: {exc}")
                self.state.set(str(exc))
                if action == "verify":
                    self.settings = None
                if action == "jog":
                    self.jogging = False
                if action == "mode":
                    self.mode_pending = False
        elif event == "stale":
            self.feedback = None
            for variable in (self.metric_position, self.metric_velocity, self.metric_torque):
                variable.set("—")
        self.update_buttons()

    def expire_feedback(self) -> None:
        if self.feedback is not None and time.monotonic() - self.feedback.received_at > 0.6:
            for variable in (self.metric_position, self.metric_velocity, self.metric_torque):
                variable.set("—")
            self.thermal.set("Feedback đã cũ · chờ RX mới")
            self.update_buttons()


class MultiMotorApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("DAMIAO | Điều khiển độc lập CAN bus")
        self.root.geometry("1360x1000")
        self.root.minsize(1140, 760)
        self.controller = None
        self.connecting = False
        self.shutting_down = False
        self.closing = False
        self._closed_view = False
        self._poll_timer = None
        self.events: queue.Queue = queue.Queue()
        self.panels: list[MotorPanel] = []
        self.channel = tk.StringVar(value=os.getenv("DAMIAO_CAN_CHANNEL", "COM3"))
        self.bitrate = tk.StringVar(value=os.getenv("DAMIAO_CAN_BITRATE", "1000000"))
        self.baud = tk.StringVar(value=os.getenv("DAMIAO_CAN_SERIAL_BAUDRATE", "921600"))
        self.status = tk.StringVar(value="Chưa mở USB2CAN")
        self.style = apply_theme(root)
        header = ttk.Frame(root, padding=(22, 16, 22, 10), style="Page.TFrame")
        header.pack(fill="x")
        ttk.Label(header, text="DAMIAO  /  CAN CONTROL", style="Brand.TLabel").pack(side="left")
        ttk.Label(header, text="ĐIỀU KHIỂN ĐỘC LẬP TỪNG MOTOR", style="PageMuted.TLabel").pack(side="right")

        top = ttk.LabelFrame(root, text="KẾT NỐI USB2CAN", padding=(14, 10))
        top.pack(fill="x", padx=22, pady=(0, 8))
        self.connection_inputs = []
        ttk.Label(top, text="USB2CAN COM").grid(row=0, column=0, sticky="w")
        self.port_combo = ttk.Combobox(top, textvariable=self.channel, width=10)
        self.port_combo.grid(row=1, column=0, padx=(0, 8))
        self.connection_inputs.append(self.port_combo)
        for column, (label, variable, width) in enumerate((("CAN bit/s", self.bitrate, 12), ("Serial baud", self.baud, 12)), start=1):
            ttk.Label(top, text=label).grid(row=0, column=column, sticky="w")
            entry = ttk.Entry(top, textvariable=variable, width=width)
            entry.grid(row=1, column=column, padx=(0, 8))
            self.connection_inputs.append(entry)
        ttk.Label(top, text="POSITION / JOG").grid(row=0, column=3, sticky="w")
        ttk.Label(top, text="−30 … +30 rad/s", padding=(8, 7)).grid(row=1, column=3, padx=(0, 8))
        self.connect_button = ttk.Button(top, text="MỞ BUS CAN", command=self.connect, style="Primary.TButton")
        self.connect_button.grid(row=1, column=4, padx=4)
        self.disconnect_button = ttk.Button(top, text="Ngắt kết nối", command=self.disconnect, state="disabled")
        self.disconnect_button.grid(row=1, column=5, padx=4)
        self.all_stop = ttk.Button(top, text="■ DISABLE TẤT CẢ", command=self.disable_all, state="disabled", style="Stop.TButton")
        self.all_stop.grid(row=1, column=6, padx=4)
        ttk.Label(root, textvariable=self.status, padding=(22, 4), style="Page.TLabel").pack(anchor="w")
        ttk.Label(root, text="Một bus CAN · Trạng thái, zero và lệnh riêng từng motor · Disable bỏ mô-men giữ", padding=(22, 4), style="PageMuted.TLabel").pack(anchor="w")

        self.notebook = ttk.Notebook(root)
        self.notebook.pack(fill="both", expand=True)
        self.control_tab = ttk.Frame(self.notebook, style="Page.TFrame")
        self.graph_tab = ttk.Frame(self.notebook, style="Page.TFrame")
        self.notebook.add(self.control_tab, text="ĐIỀU KHIỂN")
        self.notebook.add(self.graph_tab, text="ĐỒ THỊ POS / VEL / TOR")
        self.graphs = MotorPlots(self.graph_tab, lambda: self.notebook.select() == str(self.graph_tab))
        self.graphs.pack(fill="both", expand=True)
        self.notebook.bind("<<NotebookTabChanged>>", lambda _: self.graphs.redraw() if self.notebook.select() == str(self.graph_tab) else None)
        container = ttk.Frame(self.control_tab, style="Page.TFrame")
        container.pack(fill="both", expand=True, padx=12, pady=4)
        canvas = tk.Canvas(container, highlightthickness=0, background=BACKGROUND)
        self.motor_canvas = canvas
        scrollbar = ttk.Scrollbar(container, orient="vertical", command=canvas.yview)
        scrollbar.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)
        canvas.configure(yscrollcommand=scrollbar.set)
        self.motor_area = ttk.Frame(canvas, style="Page.TFrame")
        self.motor_area.columnconfigure(0, weight=1, uniform="motor")
        self.motor_area.columnconfigure(1, weight=1, uniform="motor")
        window = canvas.create_window((0, 0), window=self.motor_area, anchor="nw")
        self.motor_area.bind("<Configure>", lambda _: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.bind("<Configure>", lambda event: canvas.itemconfigure(window, width=event.width))
        root.bind_all("<MouseWheel>", self.scroll_view, add="+")
        footer = ttk.Frame(self.control_tab, style="Page.TFrame")
        footer.pack(fill="x", padx=22, pady=(4, 8))
        self.add_button = ttk.Button(footer, text="＋ THÊM MOTOR", command=self.add_motor)
        self.add_button.pack(side="left")
        ttk.Label(footer, text="NHẬT KÝ HOẠT ĐỘNG", style="PageMuted.TLabel").pack(side="right")
        self.log_widget = tk.Text(self.control_tab, height=3, state="disabled", font=("Consolas", 10), background=INPUT, foreground=WHITE, insertbackground=WHITE, selectbackground="#164c32", selectforeground=WHITE, relief="flat", highlightthickness=1, highlightbackground=BORDER, highlightcolor=NEON, padx=12, pady=8)
        self.log_widget.pack(fill="x", padx=22, pady=(0, 16))
        self.add_motor()
        self.add_motor()
        self.refresh_ports()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.bind("<Destroy>", self._view_destroyed, add="+")
        self.root.update_idletasks()
        self._poll_timer = self.root.after(80, self.poll)

    def _view_destroyed(self, event) -> None:
        if event.widget is not self.root:
            return
        self._closed_view = True
        if self._poll_timer is not None:
            try:
                self.root.after_cancel(self._poll_timer)
            except tk.TclError:
                pass
            self._poll_timer = None

    def add_motor(self) -> None:
        if self.controller is not None or self.connecting or len(self.panels) >= 15:
            return
        self.panels.append(MotorPanel(self, self.motor_area, len(self.panels) + 1))
        self.graphs.configure_motors([p.can_id for p in self.panels])

    def scroll_view(self, event) -> None:
        canvas = self.graphs.canvas if self.notebook.select() == str(self.graph_tab) else self.motor_canvas
        canvas.yview_scroll(-int(event.delta / 120), "units")

    def refresh_ports(self) -> None:
        ports = list_ports.comports()
        usb = [p.device for p in ports if p.vid is not None and p.pid is not None]
        self.port_combo.configure(values=sorted(set(usb or [p.device for p in ports])))

    def show_error(self, text: str) -> None:
        messagebox.showerror("Điều khiển motor", text, parent=self.root)

    def log(self, text: str) -> None:
        self.log_widget.configure(state="normal")
        self.log_widget.insert("end", text + "\n")
        if int(self.log_widget.index("end-1c").split(".")[0]) > 600:
            self.log_widget.delete("1.0", "100.0")
        self.log_widget.see("end")
        self.log_widget.configure(state="disabled")

    def connection_buttons(self) -> None:
        busy = self.connecting or self.shutting_down
        connected = self.controller is not None
        self.connect_button.configure(state="normal" if not busy and not connected else "disabled")
        self.disconnect_button.configure(state="normal" if not busy and connected else "disabled")
        self.all_stop.configure(state="normal" if not busy and connected else "disabled")
        self.add_button.configure(state="normal" if not busy and not connected else "disabled")
        for widget in self.connection_inputs:
            widget.configure(state="disabled" if busy or connected else "normal")
        for panel in self.panels:
            panel.update_buttons()

    def connect(self) -> None:
        if self.controller is not None or self.connecting or self.shutting_down:
            return
        try:
            channel = self.channel.get().strip()
            validate_usb2can_port(channel)
            bitrate, baud = int(self.bitrate.get()), int(self.baud.get())
            if bitrate % 1000 or bitrate // 1000 not in USB2CAN_CAN_BAUDS_KBPS or baud <= 0:
                raise ValueError("CAN bitrate hoặc serial baud không hợp lệ.")
            code = USB2CAN_CAN_BAUDS_KBPS.index(bitrate // 1000)
            rate = float(os.getenv("DAMIAO_MULTI_COMMAND_RATE_HZ", "100"))
            if not math.isfinite(rate) or rate <= 0:
                raise ValueError("Tần số gửi CAN phải dương và hữu hạn.")
            ids = [(int(p.id_var.get(), 0), int(p.master_var.get(), 0)) for p in self.panels]
            # Validate IDs before opening a physical adapter.
            command_ids = {v for can_id, _ in ids for v in (can_id, can_id + 0x100)}
            if len({c for c, _ in ids}) != len(ids) or len({m for _, m in ids}) != len(ids):
                raise ValueError("CAN ID và Master ID phải riêng biệt cho mỗi motor.")
            if any(not 1 <= c <= 15 or not 1 <= m < 0x7FF or m in command_ids for c, m in ids):
                raise ValueError("CAN ID: 0x01..0x0F. Master ID: 0x001..0x7FE, không trùng ID lệnh.")
        except (ValueError, RuntimeError) as exc:
            self.show_error(str(exc))
            return
        self.connecting = True
        self.status.set("Đang mở USB2CAN…")
        self.connection_buttons()

        def open_bus():
            try:
                bus = DamiaoUSB2CANBus(channel, baudrate=baud, can_bitrate_code=code)
                controller = MultiMotorController(bus, ids, notify=lambda *event: self.events.put(event), rate_hz=rate)
            except Exception as exc:
                self.events.put((None, "open_error", str(exc)))
            else:
                self.events.put((None, "opened", controller))
        threading.Thread(target=open_bus, name="multi-open", daemon=True).start()

    def disable_all(self) -> None:
        for panel in self.panels:
            panel.dispatch("disable")

    def release_jogs(self) -> None:
        """Mouse release is a view interaction, not a latched Jog stop."""

    def focus_lost(self, _event) -> None:
        """Focus/tab changes must not alter ongoing motor commands."""

    def disconnect(self) -> None:
        if self.controller is None or self.shutting_down:
            return
        self.shutting_down = True
        self.status.set("Đang Disable tất cả và đóng USB2CAN…")
        self.connection_buttons()
        controller = self.controller

        def close_bus():
            error = None
            try:
                controller.close()
            except Exception as exc:
                error = str(exc)
            self.events.put((None, "closed", error))
        threading.Thread(target=close_bus, name="multi-close", daemon=True).start()

    def show_parameters(self, can_id: int, parameters) -> None:
        window = tk.Toplevel(self.root)
        window.configure(background=BACKGROUND)
        window.title(f"Motor CAN 0x{can_id:02X} — tham số hiện tại")
        window.geometry("750x550")
        tree = ttk.Treeview(window, columns=("address", "name", "value", "unit"), show="headings")
        for name, title, width in (("address", "Register", 70), ("name", "Parameter", 300), ("value", "Value", 140), ("unit", "Unit", 120)):
            tree.heading(name, text=title)
            tree.column(name, width=width)
        scroll = ttk.Scrollbar(window, orient="vertical", command=tree.yview)
        tree.configure(yscrollcommand=scroll.set)
        scroll.pack(side="right", fill="y")
        tree.pack(fill="both", expand=True)
        for p in parameters:
            value = f"0x{int(p.value):03X}" if p.unit == "hex" else f"{p.value:.7g}"
            tree.insert("", "end", values=(f"0x{p.address:02X}", p.name, value, p.unit))

    def poll(self) -> None:
        if self._closed_view:
            return
        if self._poll_timer is not None:
            try:
                self.root.after_cancel(self._poll_timer)
            except tk.TclError:
                pass
            self._poll_timer = None
        while True:
            try:
                can_id, event, value = self.events.get_nowait()
            except queue.Empty:
                break
            if event == "opened":
                self.controller = value
                self.connecting = False
                self.graphs.history.clear()
                self.graphs.configure_motors(list(value.sessions))
                for panel, session in zip(self.panels, value.sessions.values()):
                    panel.can_id = session.motor.can_id
                    panel.session = session
                    panel.dispatch("verify")
                self.status.set("USB2CAN đã mở — kiểm tra phản hồi từng motor riêng")
                self.connection_buttons()
                if self.closing:
                    self.disconnect()
            elif event == "open_error":
                self.connecting = False
                self.status.set(str(value))
                self.log(f"USB2CAN ERROR: {value}")
                self.connection_buttons()
                if self.closing:
                    self.root.destroy()
                    return
            elif event == "closed":
                self.controller = None
                self.shutting_down = False
                for panel in self.panels:
                    panel.session = None
                    panel.settings = None
                    panel.enabled = False
                    panel.saved_target = None
                    panel.pending = 0
                    panel.zero = 0.0
                    panel.feedback = None
                    panel.jogging = False
                    panel.last_error = None
                    panel.state.set("Đã ngắt kết nối")
                    panel.info.set("Firmware / PMAX / VMAX / TMAX: chưa đọc")
                    panel.zero_text.set("Zero phần mềm: 0.000 rad")
                    panel.live.set("Vị trí: —   Tốc độ: —   Mô-men: —")
                    panel.hex_text.set("Chưa gửi lệnh")
                    for variable in (panel.metric_position, panel.metric_velocity, panel.metric_torque):
                        variable.set("—")
                    panel.thermal.set("DRIVER  — °C     MOTOR  — °C     STATUS  —")
                    panel.rx_text.set("RX: chờ phản hồi CAN")
                    panel.event("mode", ("position", 0.0))
                self.status.set("USB2CAN đã đóng" if value is None else f"Đóng bus: {value}")
                if value is not None:
                    self.log(str(value))
                self.connection_buttons()
                if self.closing:
                    self.root.destroy()
                    return
            else:
                panel = next((p for p in self.panels if p.can_id == can_id and p.session is not None), None)
                if panel is not None:
                    panel.event(event, value)
                    if event == "feedback":
                        self.graphs.history.add(can_id, value)
                    if event == "reference":
                        self.graphs.history.references[can_id] = value
                    if event in ("zero", "mode", "feedback"):
                        self.graphs.history.zero_offsets[can_id] = panel.zero
        for panel in self.panels:
            panel.expire_feedback()
        self._poll_timer = self.root.after(80, self.poll)

    def on_close(self) -> None:
        self.closing = True
        if self.controller is not None:
            self.disconnect()
        elif not self.connecting:
            self.root.destroy()


def main() -> None:
    root = tk.Tk()
    MultiMotorApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
