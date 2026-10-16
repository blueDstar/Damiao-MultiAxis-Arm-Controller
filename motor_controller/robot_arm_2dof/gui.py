"""3D arm at left, compact independent Position controls at right."""

from __future__ import annotations

import argparse
import json
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

from multi_motor.controller import MultiMotorController
from multi_motor.theme import BACKGROUND, BORDER, ERROR, INPUT, MUTED, NEON, WHITE, apply_theme
from robot_arm_2dof.demo_bus import DemoBus
from robot_arm_2dof.kinematics import (
    ANGLE_FACTORS, SPEED_FACTORS, ArmGeometry, JointMapping, parse_target, validate_ids,
)
from robot_arm_2dof.scene import ArmScene
from single_motor.move_to_angle import validate_usb2can_port
from single_motor.usb2can_serial import DamiaoUSB2CANBus, USB2CAN_CAN_BAUDS_KBPS


PROJECT_DIR = Path(__file__).resolve().parents[1]
CONFIG_PATH = Path(__file__).resolve().parent / "local_config.json"
load_dotenv(PROJECT_DIR / ".env")
SOURCE_DEMO = "MÔ PHỎNG"
SOURCE_CAN = "CAN THẬT"


def metric(value, quantum=0):
    return f"{0 if abs(value) <= quantum else value:+.3f}"


class JointPanel:
    def __init__(self, app, parent, index):
        self.app, self.index = app, index
        self.session = None
        self.settings = None
        self.feedback = None
        self.zero = 0.0
        self.pending = 0
        self.reference = None
        self.id_var = tk.StringVar(value=f"0x{index + 1:02X}")
        self.master_var = tk.StringVar(value=f"0x{index + 0x11:02X}")
        self.angle = tk.StringVar(value="30" if index == 0 else "-30")
        self.angle_unit = tk.StringVar(value="deg")
        self.speed = tk.StringVar(value="1")
        self.speed_unit = tk.StringVar(value="rad/s")
        self.target_kind = tk.StringVar(value="Quay thêm từ hiện tại")
        self.status = tk.StringVar(value="Chưa kết nối")
        self.position_text = tk.StringVar(value="—")
        self.velocity_text = tk.StringVar(value="—")
        self.torque_text = tk.StringVar(value="—")
        self._previous_units = {"angle": "deg", "speed": "rad/s"}
        self.frame = ttk.LabelFrame(parent, text="J1 · VAI" if index == 0 else "J2 · KHUỶU",
                                    style="Arm.TLabelframe", padding=(10, 4))
        self.frame.pack(fill="x", pady=(0, 8))
        ids = ttk.Frame(self.frame)
        ids.pack(fill="x", pady=(0, 3))
        ttk.Label(ids, text="CAN", style="Muted.TLabel").pack(side="left")
        self.id_entry = ttk.Entry(ids, textvariable=self.id_var, width=6, style="Arm.TEntry", font=("Segoe UI", 9))
        self.id_entry.pack(side="left", padx=(5, 10))
        ttk.Label(ids, text="Master", style="Muted.TLabel").pack(side="left")
        self.master_entry = ttk.Entry(ids, textvariable=self.master_var, width=6, style="Arm.TEntry", font=("Segoe UI", 9))
        self.master_entry.pack(side="left", padx=5)
        self.buttons = {}
        self.buttons["verify"] = ttk.Button(ids, text="Kiểm tra", style="Compact.TButton",
                                             command=lambda: self.dispatch("verify"))
        self.buttons["verify"].pack(side="right")
        self.status_label = ttk.Label(self.frame, textvariable=self.status, foreground=MUTED,
                                      font=("Segoe UI", 9), wraplength=355)
        self.status_label.pack(anchor="w", pady=(0, 4))
        metrics = ttk.Frame(self.frame, style="Metric.TFrame", padding=(6, 4))
        metrics.pack(fill="x")
        for i, (label, value) in enumerate((("P/°", self.position_text),
                                          ("V/rad/s", self.velocity_text),
                                          ("T/Nm", self.torque_text))):
            metrics.columnconfigure(i, weight=1)
            cell = ttk.Frame(metrics, style="Metric.TFrame")
            cell.grid(row=0, column=i, sticky="ew")
            ttk.Label(cell, text=label, style="Metric.TLabel", font=("Segoe UI", 8)).pack(side="left", padx=(0, 4))
            ttk.Label(cell, textvariable=value, style="ArmMetric.TLabel").pack(side="left")
        form = ttk.Frame(self.frame)
        form.pack(fill="x", pady=(5, 2))
        form.columnconfigure(0, weight=1)
        form.columnconfigure(2, weight=1)
        ttk.Label(form, text="Góc motor", style="Muted.TLabel", font=("Segoe UI", 9)).grid(row=0, column=0, columnspan=2, sticky="w")
        ttk.Label(form, text="Tốc độ motor", style="Muted.TLabel", font=("Segoe UI", 9)).grid(row=0, column=2, columnspan=2, sticky="w", padx=(12, 0))
        ttk.Entry(form, textvariable=self.angle, width=8, style="Arm.TEntry", font=("Segoe UI", 9)).grid(row=1, column=0, sticky="ew", pady=2)
        angle_unit = ttk.Combobox(form, textvariable=self.angle_unit, values=tuple(ANGLE_FACTORS),
                                 width=4, state="readonly", style="Arm.TCombobox", font=("Segoe UI", 9))
        angle_unit.grid(row=1, column=1, padx=(4, 0))
        ttk.Entry(form, textvariable=self.speed, width=7, style="Arm.TEntry", font=("Segoe UI", 9)).grid(row=1, column=2, sticky="ew", padx=(12, 0), pady=2)
        speed_unit = ttk.Combobox(form, textvariable=self.speed_unit, values=tuple(SPEED_FACTORS),
                                 width=5, state="readonly", style="Arm.TCombobox", font=("Segoe UI", 9))
        speed_unit.grid(row=1, column=3, padx=(4, 0))
        angle_unit.bind("<<ComboboxSelected>>", lambda _: self.convert_unit("angle"))
        speed_unit.bind("<<ComboboxSelected>>", lambda _: self.convert_unit("speed"))
        mode_row = ttk.Frame(self.frame)
        mode_row.pack(fill="x", pady=(0, 4))
        ttk.Combobox(mode_row, textvariable=self.target_kind,
                     values=("Quay thêm từ hiện tại", "Tới góc motor từ zero"),
                     state="readonly", width=24, style="Arm.TCombobox", font=("Segoe UI", 9)).pack(side="left", fill="x", expand=True)
        self.buttons["zero"] = ttk.Button(mode_row, text="Zero", style="Compact.TButton",
                                           command=lambda: self.dispatch("zero"))
        self.buttons["zero"].pack(side="right", padx=(8, 0))
        commands = ttk.Frame(self.frame)
        commands.pack(fill="x")
        for column, (action, label, command) in enumerate((
            ("enable", "Enable", lambda: self.dispatch("enable")),
            ("target", "Gửi góc", self.send_target),
            ("hold", "Dừng", lambda: self.dispatch("hold")),
            ("disable", "Disable", lambda: self.dispatch("disable")),
        )):
            commands.columnconfigure(column, weight=1)
            button = ttk.Button(commands, text=label, command=command,
                                 style="ArmPrimary.TButton" if action == "target" else "Compact.TButton")
            button.grid(row=0, column=column, sticky="ew", padx=(0 if column == 0 else 4, 0))
            self.buttons[action] = button
        self.update_buttons()

    @property
    def can_id(self):
        return self.session.motor.can_id if self.session is not None else int(self.id_var.get(), 0)

    def convert_unit(self, field):
        variable = self.angle if field == "angle" else self.speed
        unit = self.angle_unit.get() if field == "angle" else self.speed_unit.get()
        factors = ANGLE_FACTORS if field == "angle" else SPEED_FACTORS
        try:
            value = float(variable.get()) * factors[self._previous_units[field]] / factors[unit]
            if math.isfinite(value):
                variable.set(f"{value:.15g}")
        except ValueError:
            pass
        self._previous_units[field] = unit

    def target_args(self):
        if self.session is None or self.settings is None or not self.session.motor.enabled:
            raise ValueError("Kiểm tra và Enable motor trước khi gửi góc.")
        if self.feedback is None or time.monotonic() - self.feedback.received_at > 1.0:
            raise ValueError("Cần feedback mới từ motor trước khi gửi góc.")
        if self.pending or self.app.shutting_down:
            raise ValueError("Đợi lệnh hiện tại hoàn tất trước khi gửi góc.")
        angle, speed = parse_target(self.angle.get(), self.angle_unit.get(), self.speed.get(),
                                    self.speed_unit.get(), self.settings.velocity_range_rad_s)
        # Preserve the existing measured-position MIT controller and its damping.
        return angle, speed, 0.0, 1.0, self.target_kind.get() == "Quay thêm từ hiện tại"

    def send_target(self):
        try:
            args = self.target_args()
        except ValueError as exc:
            self.app.show_error(str(exc))
            return
        self.dispatch("target", *args)

    def dispatch(self, action, *args):
        if self.session is None or self.app.shutting_down:
            return
        self.pending += 1
        if action == "verify":
            self.settings = None
        self.update_buttons()
        can_id = self.session.motor.can_id
        future = self.session.submit(action, *args)
        future.add_done_callback(lambda f: self.app.events.put((can_id, "done", (action, f))))

    def event(self, event, value):
        if event == "settings":
            self.settings = value
        elif event == "feedback":
            self.feedback = value
            self.status_label.configure(foreground=NEON if value.is_enabled else MUTED)
        elif event == "zero":
            self.zero = float(value)
        elif event == "trajectory":
            self.reference = value
        elif event == "state":
            self.status.set(self.translate_state(value))
            self.app.log(f"J{self.index + 1} · {self.status.get()}")
        elif event == "error":
            self.status.set(str(value))
            self.status_label.configure(foreground=ERROR)
            self.app.log(f"J{self.index + 1} · LỖI: {value}")
        elif event == "stale":
            self.feedback = None
        elif event == "done":
            action, future = value
            self.pending = max(0, self.pending - 1)
            try:
                future.result()
            except Exception as exc:
                self.status.set(str(exc))
                self.status_label.configure(foreground=ERROR)
                self.app.log(f"J{self.index + 1} · {action}: {exc}")
        self.update_buttons()

    @staticmethod
    def translate_state(value):
        states = {"Verified / disabled": "CAN đã xác minh · Đang Disable",
                  "Enabled / idle — no position hold": "ENABLED · Sẵn sàng · Không giữ vị trí",
                  "Moving to target": "Đang quay tới góc đã gửi",
                  "Disabled": "DISABLED · Đã bỏ mô-men",
                  "Stop requested / zero velocity immediately": "Đang dừng · Tốc độ lệnh = 0"}
        if value.startswith("Target reached"):
            return "Đã tới góc đích · Tốc độ lệnh = 0"
        if value.startswith("Position stopped; measured angle error"):
            error = value.split("error", 1)[-1].strip()
            return f"Đã dừng · Sai số đo {error}"
        return states.get(value, value)

    def update_buttons(self):
        connected = self.session is not None and not self.app.shutting_down
        enabled = connected and self.session.motor.enabled
        fresh = self.feedback is not None and time.monotonic() - self.feedback.received_at <= 1.0
        verified = connected and self.settings is not None
        ready = enabled and fresh and self.pending == 0
        stopped = ready and abs(self.feedback.velocity_rad_s) < 0.05
        actions = {"verify": connected and not enabled and self.pending == 0,
                   "enable": verified and not enabled and self.pending == 0,
                   "target": ready, "zero": stopped,
                   "hold": bool(enabled), "disable": bool(verified)}
        for action, allowed in actions.items():
            self.buttons[action].configure(state="normal" if allowed else "disabled")
        state = "disabled" if connected or self.app.connecting or self.app.shutting_down else "normal"
        self.id_entry.configure(state=state)
        self.master_entry.configure(state=state)
        if fresh:
            self.position_text.set(f"{math.degrees(self.feedback.position_rad - self.zero):+.2f}")
            self.velocity_text.set(metric(self.feedback.velocity_rad_s, 60 / 4095))
            self.torque_text.set(metric(self.feedback.torque_nm, 20 / 4095))
        else:
            self.position_text.set("—")
            self.velocity_text.set("—")
            self.torque_text.set("—")

    def reset(self):
        self.session = self.settings = self.feedback = self.reference = None
        self.pending = 0
        self.status.set("Chưa kết nối")
        self.status_label.configure(foreground=MUTED)
        self.update_buttons()


class RobotArmApp:
    def __init__(self, root):
        self.root = root
        self.style = apply_theme(root)
        self.style.configure("Compact.TButton", padding=(6, 3), font=("Segoe UI", 9, "bold"))
        self.style.configure("ArmPrimary.TButton", background="#125c37", padding=(6, 3), font=("Segoe UI", 9, "bold"))
        self.style.configure("ArmMetric.TLabel", background=INPUT, foreground=WHITE, font=("Consolas", 12, "bold"))
        self.style.configure("Arm.TEntry", padding=2)
        self.style.configure("Arm.TCombobox", padding=2)
        self.style.configure("Arm.TLabelframe", bordercolor=NEON, lightcolor=NEON, darkcolor=NEON, borderwidth=1)
        self.style.configure("Arm.TLabelframe.Label", foreground=NEON, font=("Segoe UI", 11, "bold"))
        root.title("DAMIAO | Robot Arm 2 DOF · CAN Control")
        root.geometry("1440x900")
        root.minsize(1160, 780)
        self.events = queue.Queue()
        self.controller = None
        self.connecting = False
        self.shutting_down = False
        self._closing_view = False
        self._closed_view = False
        self._poll_timer = None
        self.geometry = ArmGeometry()
        self.mappings = [JointMapping(math.radians(55)), JointMapping(math.radians(-80))]
        self.measured_positions = [None, None]
        self.view_zeros = [None, None]
        self.view_angles = [m.mounting_rad for m in self.mappings]
        self.source = tk.StringVar(value=SOURCE_DEMO)
        self.active_source = None
        ports = [p.device for p in list_ports.comports()]
        self.channel = tk.StringVar(value=os.getenv("DAMIAO_CAN_CHANNEL", ports[0] if ports else "COM3"))
        self.bitrate = tk.StringVar(value=os.getenv("DAMIAO_CAN_BITRATE", "1000000"))
        self.baud = tk.StringVar(value=os.getenv("DAMIAO_CAN_SERIAL_BAUDRATE", "921600"))
        self.connection_status = tk.StringVar(value="Chọn MÔ PHỎNG để thử giao diện hoặc CAN THẬT để nối motor.")
        self.endpoint = tk.StringVar()
        self.joint_readout = tk.StringVar()
        self._build(ports)
        self._load_configuration()
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.bind("<Destroy>", self._on_destroy, add="+")
        self.poll()

    def _build(self, ports):
        outer = ttk.Frame(self.root, style="Page.TFrame", padding=12)
        outer.pack(fill="both", expand=True)
        header = ttk.Frame(outer, style="Page.TFrame")
        header.pack(fill="x", pady=(0, 12))
        ttk.Label(header, text="DAMIAO / ROBOT ARM", style="Brand.TLabel").pack(side="left")
        ttk.Label(header, text="2 DOF   ·   DIGITAL VIEW / CAN CONTROL", style="PageMuted.TLabel",
                  font=("Consolas", 11)).pack(side="right")
        connection = ttk.Frame(outer, padding=(12, 8))
        connection.pack(fill="x", pady=(0, 12))
        controls = []
        for column, (label, variable, values, width) in enumerate((
            ("NGUỒN DỮ LIỆU", self.source, (SOURCE_DEMO, SOURCE_CAN), 13),
            ("USB2CAN COM", self.channel, ports, 8),
            ("CAN bit/s", self.bitrate, None, 10),
            ("Serial baud", self.baud, None, 10),
        )):
            ttk.Label(connection, text=label, style="Muted.TLabel", font=("Segoe UI", 9)).grid(row=0, column=column, sticky="w")
            field = ttk.Entry(connection, textvariable=variable, width=width) if values is None else \
                ttk.Combobox(connection, textvariable=variable, values=values, width=width,
                             state="readonly" if column == 0 else "normal")
            field.grid(row=1, column=column, sticky="w", padx=(0, 10), pady=(3, 0))
            controls.append(field)
        self.connection_fields = controls
        self.connect_button = ttk.Button(connection, text="MỞ PHIÊN", style="Primary.TButton", command=self.connect)
        self.connect_button.grid(row=1, column=4, padx=(6, 8))
        self.disconnect_button = ttk.Button(connection, text="Ngắt kết nối", command=self.disconnect)
        self.disconnect_button.grid(row=1, column=5, padx=4)
        self.disable_button = ttk.Button(connection, text="■ DISABLE CẢ HAI", command=self.disable_all)
        self.disable_button.grid(row=1, column=6, padx=(10, 0))
        connection.columnconfigure(7, weight=1)
        self.source.trace_add("write", lambda *_: self.connection_buttons())
        body = ttk.Frame(outer, style="Page.TFrame")
        body.pack(fill="both", expand=True)
        body.columnconfigure(0, weight=1)
        body.columnconfigure(1, weight=0, minsize=420)
        body.rowconfigure(0, weight=1)
        left = tk.Frame(body, background=INPUT, highlightthickness=1, highlightbackground=BORDER)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 16))
        toolbar = ttk.Frame(left, padding=(12, 8))
        toolbar.pack(fill="x")
        ttk.Label(toolbar, text="3D / FEEDBACK VIEW", style="Section.TLabel").pack(side="left")
        ttk.Button(toolbar, text="Góc nhìn gốc", style="Compact.TButton",
                   command=lambda: self.scene.home()).pack(side="right")
        ttk.Button(toolbar, text="Cấu hình mô hình", style="Compact.TButton",
                   command=self.configure_model).pack(side="right", padx=8)
        self.scene = ArmScene(left, self.geometry)
        self.scene.pack(fill="both", expand=True)
        footer = ttk.Frame(left, padding=(12, 10))
        footer.pack(fill="x")
        ttk.Label(footer, textvariable=self.joint_readout, foreground=WHITE,
                  font=("Consolas", 12, "bold")).pack(anchor="w")
        ttk.Label(footer, textvariable=self.endpoint, style="Muted.TLabel").pack(anchor="w", pady=(4, 0))
        right_host = ttk.Frame(body, style="Page.TFrame")
        right_host.grid(row=0, column=1, sticky="nsew")
        self.control_canvas = tk.Canvas(right_host, width=405, highlightthickness=0, background=BACKGROUND)
        self.control_canvas.pack(side="left", fill="both", expand=True)
        scroll = ttk.Scrollbar(right_host, orient="vertical", command=self.control_canvas.yview)
        scroll.pack(side="right", fill="y")
        self.control_canvas.configure(yscrollcommand=scroll.set)
        right = ttk.Frame(self.control_canvas, style="Page.TFrame")
        control_window = self.control_canvas.create_window((0, 0), window=right, anchor="nw")
        right.bind("<Configure>", lambda _: self.control_canvas.configure(scrollregion=self.control_canvas.bbox("all")))
        self.control_canvas.bind("<Configure>", lambda event: self.control_canvas.itemconfigure(control_window, width=event.width))
        ttk.Label(right, text="POSITION / HAI KHỚP", style="Page.TLabel",
                  font=("Segoe UI", 14, "bold")).pack(anchor="w", pady=(0, 3))
        ttk.Label(right, text="Lệnh riêng từng motor · Tốc độ −30 … +30 rad/s",
                  style="PageMuted.TLabel", font=("Segoe UI", 9)).pack(anchor="w", pady=(0, 8))
        self.panels = [JointPanel(self, right, i) for i in range(2)]
        pair = ttk.Frame(right, style="Page.TFrame")
        pair.pack(fill="x")
        self.pair_button = ttk.Button(pair, text="GỬI GÓC CẢ HAI", style="ArmPrimary.TButton", command=self.send_pair)
        self.pair_button.pack(side="left", fill="x", expand=True)
        self.stop_button = ttk.Button(pair, text="■ DỪNG CẢ HAI", style="Compact.TButton", command=self.stop_all)
        self.stop_button.pack(side="left", padx=(8, 0), fill="x", expand=True)
        ttk.Label(right, text="Zero đổi mốc lệnh; mô hình vẫn giữ tư thế đo.\nMô hình vẽ theo feedback, không theo góc vừa nhập.",
                  style="PageMuted.TLabel", font=("Segoe UI", 9), wraplength=395).pack(anchor="w", pady=(8, 6))
        self.log_widget = tk.Text(right, height=2, width=40, wrap="word", background=INPUT, foreground=MUTED,
                                  font=("Consolas", 9), relief="flat", highlightthickness=1,
                                  highlightbackground=BORDER, state="disabled")
        self.log_widget.pack(fill="both", expand=True)
        def bind_scroll(widget):
            widget.bind("<MouseWheel>", self._scroll_controls)
            for child in widget.winfo_children():
                bind_scroll(child)
        bind_scroll(right)
        self.control_canvas.bind("<MouseWheel>", self._scroll_controls)
        ttk.Label(outer, textvariable=self.connection_status, style="PageMuted.TLabel",
                  font=("Segoe UI", 10)).pack(fill="x", pady=(10, 0))
        self.connection_buttons()

    def _scroll_controls(self, event):
        bounds = self.control_canvas.bbox("all")
        if bounds and event.delta and bounds[3] > self.control_canvas.winfo_height():
            self.control_canvas.yview_scroll(-1 if event.delta > 0 else 1, "units")
        return "break"

    def connection_buttons(self):
        busy = self.connecting or self.shutting_down
        opened = self.controller is not None
        self.connect_button.configure(state="disabled" if busy or opened else "normal")
        self.disconnect_button.configure(state="normal" if opened and not busy else "disabled")
        self.disable_button.configure(state="normal" if opened and not busy else "disabled")
        for i, field in enumerate(self.connection_fields):
            field.configure(state="disabled" if busy or opened or (i > 0 and self.source.get() == SOURCE_DEMO)
                            else "readonly" if i == 0 else "normal")
        for panel in getattr(self, "panels", []):
            panel.update_buttons()

    def connect(self):
        if self.controller is not None or self.connecting or self.shutting_down or self._closing_view:
            return
        try:
            ids = validate_ids([(int(p.id_var.get(), 0), int(p.master_var.get(), 0)) for p in self.panels])
            source = self.source.get()
            if source not in (SOURCE_DEMO, SOURCE_CAN):
                raise ValueError("Chọn nguồn dữ liệu hợp lệ.")
            channel = self.channel.get().strip()
            rate = float(os.getenv("DAMIAO_MULTI_COMMAND_RATE_HZ", "100"))
            if not math.isfinite(rate) or rate <= 0:
                raise ValueError("Tần số gửi CAN phải là số dương hữu hạn.")
            code, baud = 0, 921600
            if source == SOURCE_CAN:
                validate_usb2can_port(channel)
                bitrate, baud = int(self.bitrate.get()), int(self.baud.get())
                if bitrate % 1000 or bitrate // 1000 not in USB2CAN_CAN_BAUDS_KBPS or baud <= 0:
                    raise ValueError("CAN bitrate hoặc serial baud không hợp lệ.")
                code = USB2CAN_CAN_BAUDS_KBPS.index(bitrate // 1000)
        except (ValueError, RuntimeError) as exc:
            self.show_error(str(exc))
            return
        self.connecting = True
        self.active_source = source
        self.connection_status.set("Đang mở phiên và đọc cấu hình hai motor…")
        self.connection_buttons()
        self.measured_positions = [None, None]
        self.view_zeros = [None, None]
        for panel in self.panels:
            panel.zero = 0.0
            panel.reset()

        def open_bus():
            bus = None
            try:
                bus = DemoBus(ids) if source == SOURCE_DEMO else \
                    DamiaoUSB2CANBus(channel, baudrate=baud, can_bitrate_code=code)
                controller = MultiMotorController(bus, ids, notify=lambda *event: self.events.put(event), rate_hz=rate)
            except Exception as exc:
                if bus is not None:
                    try:
                        bus.shutdown()
                    except Exception:
                        pass
                self.events.put((None, "open_error", str(exc)))
            else:
                self.events.put((None, "opened", controller))
        threading.Thread(target=open_bus, name="arm-open", daemon=True).start()

    def send_pair(self):
        try:
            # Validate both first so a bad second field cannot start the first.
            args = [p.target_args() for p in self.panels]
        except ValueError as exc:
            self.show_error(str(exc))
            return
        for panel, target in zip(self.panels, args):
            panel.dispatch("target", *target)

    def stop_all(self):
        for panel in self.panels:
            if panel.session is not None and panel.session.motor.enabled:
                panel.dispatch("hold")

    def disable_all(self):
        for panel in self.panels:
            panel.dispatch("disable")

    def disconnect(self):
        if self.controller is None or self.shutting_down:
            return
        self.shutting_down = True
        self.connection_status.set("Đang Disable hai motor và đóng bus…")
        self.connection_buttons()
        controller = self.controller

        def close_bus():
            error = None
            try:
                controller.close()
            except Exception as exc:
                error = str(exc)
            self.events.put((None, "closed", error))
        threading.Thread(target=close_bus, name="arm-close", daemon=True).start()

    def close(self):
        self._closing_view = True
        if self.controller is not None:
            self.disconnect()
        elif not self.connecting:
            self.root.destroy()

    def _on_destroy(self, event):
        if event.widget is self.root:
            self._closed_view = True
            if self._poll_timer is not None:
                self.root.after_cancel(self._poll_timer)
                self._poll_timer = None

    def _refresh_scene(self):
        stale = []
        for i, panel in enumerate(self.panels):
            fresh = panel.settings is not None and panel.feedback is not None and time.monotonic() - panel.feedback.received_at <= 1
            if fresh:
                self.measured_positions[i] = panel.feedback.position_rad
                if self.view_zeros[i] is None:
                    self.view_zeros[i] = panel.feedback.position_rad
            position = self.measured_positions[i]
            # The model origin is the first verified feedback, independent of
            # the Position command's software zero. Zero must not move a model
            # joint when the corresponding physical motor has not moved.
            zero = self.view_zeros[i]
            self.view_angles[i] = self.mappings[i].angle(position, zero) if position is not None else self.mappings[i].mounting_rad
            stale.append(not fresh)
        self.scene.set_pose(self.view_angles)
        if tuple(stale) != self.scene.stale:
            self.scene.stale = tuple(stale)
            self.scene.invalidate()
        if self.controller is None:
            source = "ĐÃ NGẮT · GÓC FEEDBACK CUỐI" if any(v is not None for v in self.measured_positions) else "CHƯA KẾT NỐI · TƯ THẾ LẮP"
        else:
            source = "MÔ PHỎNG CAN · FEEDBACK ẢO" if self.active_source == SOURCE_DEMO else "CAN THẬT · GÓC ĐO TỪ MOTOR"
            if any(stale):
                source += " · CHỜ FEEDBACK"
        if source != self.scene.source:
            self.scene.source = source
            self.scene.invalidate()
        self.scene.render()
        self.joint_readout.set(f"J1  {math.degrees(self.view_angles[0]):+7.2f}°     J2  {math.degrees(self.view_angles[1]):+7.2f}°")
        tip = self.geometry.points(*self.view_angles)[2]
        suffix = " · góc cuối / chưa có feedback" if any(stale) else ""
        self.endpoint.set(f"Đầu tay  X {tip[0]:+.3f} m   Y {tip[1]:+.3f} m   Z {tip[2]:+.3f} m{suffix}")

    def poll(self):
        if self._closed_view:
            return
        started = time.monotonic()
        if self._poll_timer is not None:
            self.root.after_cancel(self._poll_timer)
            self._poll_timer = None
        for _ in range(1500):
            try:
                can_id, event, value = self.events.get_nowait()
            except queue.Empty:
                break
            if event == "opened":
                self.controller = value
                self.connecting = False
                self.scene.motor_ids = tuple(value.sessions)
                self.scene.invalidate()
                for panel, session in zip(self.panels, value.sessions.values()):
                    panel.session = session
                    panel.dispatch("verify")
                self.connection_status.set(f"{self.active_source} · Bus đã mở. Kiểm tra từng motor trước khi Enable.")
                self.connection_buttons()
                if self._closing_view:
                    self.disconnect()
            elif event == "open_error":
                self.connecting = False
                self.active_source = None
                self.connection_status.set(f"Không mở được phiên: {value}")
                self.log(str(value))
                self.connection_buttons()
                if self._closing_view:
                    self.root.destroy()
                    return
            elif event == "closed":
                self.controller = None
                self.shutting_down = False
                self.active_source = None
                self.connection_status.set("Đã Disable và ngắt kết nối." if value is None else f"Đóng bus: {value}")
                for panel in self.panels:
                    panel.reset()
                self.connection_buttons()
                if self._closing_view:
                    self.root.destroy()
                    return
            else:
                panel = next((p for p in self.panels if p.session is not None and p.session.motor.can_id == can_id), None)
                if panel is not None:
                    panel.event(event, value)
        for panel in self.panels:
            panel.update_buttons()
        self.pair_button.configure(state="normal" if all(str(p.buttons["target"]["state"]) == "normal" for p in self.panels) else "disabled")
        self.stop_button.configure(state="normal" if any(p.session is not None and p.session.motor.enabled for p in self.panels) and not self.shutting_down else "disabled")
        self._refresh_scene()
        delay_ms = max(5, 33 - int((time.monotonic() - started) * 1000))
        self._poll_timer = self.root.after(delay_ms, self.poll)

    def log(self, text):
        self.log_widget.configure(state="normal")
        self.log_widget.insert("end", f"{time.strftime('%H:%M:%S')} {text}\n")
        if int(self.log_widget.index("end-1c").split(".")[0]) > 120:
            self.log_widget.delete("1.0", "40.0")
        self.log_widget.see("end")
        self.log_widget.configure(state="disabled")

    def show_error(self, text):
        self.log(text)
        messagebox.showerror("Robot Arm 2 DOF", text, parent=self.root)

    def _load_configuration(self):
        if not CONFIG_PATH.exists():
            return
        try:
            data = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
            geometry = ArmGeometry(**data["geometry"])
            mappings = [JointMapping(**v) for v in data["mappings"]]
            ids = validate_ids([tuple(pair) for pair in data["motor_ids"]])
            if len(mappings) != 2:
                raise ValueError("Cấu hình cần hai khớp.")
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self.log(f"Không đọc được cấu hình mô hình: {exc}")
            return
        self.geometry, self.mappings = geometry, mappings
        self.scene.set_geometry(geometry)
        for panel, (can_id, master_id) in zip(self.panels, ids):
            panel.id_var.set(f"0x{can_id:02X}")
            panel.master_var.set(f"0x{master_id:02X}")

    def configure_model(self):
        window = tk.Toplevel(self.root)
        window.title("Kích thước và cách lắp hai khớp")
        window.configure(background=BACKGROUND)
        window.resizable(False, False)
        window.transient(self.root)
        frame = ttk.Frame(window, padding=20)
        frame.pack(fill="both", expand=True)
        ttk.Label(frame, text="MÔ HÌNH VAI / KHUỶU", style="Section.TLabel").grid(row=0, column=0, columnspan=3, sticky="w", pady=(0, 10))
        ttk.Label(frame, text="Góc khớp = góc lắp + chiều × góc quay motor từ lúc nối\nCác giá trị này chỉ đổi mô hình, không gửi lệnh quay.",
                  style="Muted.TLabel").grid(row=1, column=0, columnspan=3, sticky="w", pady=(0, 12))
        dimensions = [tk.StringVar(value=f"{v:g}") for v in
                      (self.geometry.upper_length, self.geometry.forearm_length, self.geometry.shoulder_height)]
        for row, (label, variable) in enumerate(zip(("Tay trên (m)", "Cẳng tay (m)", "Cao trục vai (m)"), dimensions), 2):
            ttk.Label(frame, text=label).grid(row=row, column=0, sticky="w", pady=4)
            ttk.Entry(frame, textvariable=variable, width=12).grid(row=row, column=1, pady=4, padx=12)
        mounting = [tk.StringVar(value=f"{math.degrees(m.mounting_rad):g}") for m in self.mappings]
        signs = [tk.StringVar(value=f"{m.direction:+d}") for m in self.mappings]
        for i in range(2):
            ttk.Label(frame, text=f"J{i + 1} · Góc lắp lúc bắt đầu nối (°)").grid(row=5 + i, column=0, sticky="w", pady=4)
            ttk.Entry(frame, textvariable=mounting[i], width=12).grid(row=5 + i, column=1, pady=4, padx=12)
            ttk.Combobox(frame, textvariable=signs[i], values=("+1", "-1"), state="readonly", width=4).grid(row=5 + i, column=2)
        ttk.Label(frame, text="Zero chỉ đổi mốc lệnh Position, không đổi tư thế mô hình.\nGóc lắp và chiều dài cần đặt theo cơ cấu thực.",
                  style="Muted.TLabel").grid(row=7, column=0, columnspan=3, sticky="w", pady=12)

        def apply():
            try:
                geometry = ArmGeometry(*(float(v.get()) for v in dimensions))
                mappings = [JointMapping(math.radians(float(v.get())), int(s.get())) for v, s in zip(mounting, signs)]
                ids = validate_ids([(int(p.id_var.get(), 0), int(p.master_var.get(), 0)) for p in self.panels])
                data = {"geometry": {"upper_length": geometry.upper_length, "forearm_length": geometry.forearm_length,
                                     "shoulder_height": geometry.shoulder_height},
                        "mappings": [{"mounting_rad": m.mounting_rad, "direction": m.direction} for m in mappings],
                        "motor_ids": ids}
                CONFIG_PATH.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
            except (ValueError, OSError) as exc:
                messagebox.showerror("Cấu hình mô hình", str(exc), parent=window)
                return
            self.geometry, self.mappings = geometry, mappings
            self.scene.set_geometry(geometry)
            self.log("Đã lưu kích thước và cách lắp mô hình.")
            window.destroy()
        ttk.Button(frame, text="LƯU CẤU HÌNH", style="Primary.TButton", command=apply).grid(row=8, column=0, columnspan=3, sticky="ew")


def main():
    parser = argparse.ArgumentParser(description="DAMIAO 3D robot arm, two independent CAN motors")
    parser.add_argument("--demo", action="store_true", help="Open the simulated CAN bus; motors remain disabled")
    args = parser.parse_args()
    root = tk.Tk()
    app = RobotArmApp(root)
    if args.demo:
        root.after(100, app.connect)
    root.mainloop()


if __name__ == "__main__":
    main()
