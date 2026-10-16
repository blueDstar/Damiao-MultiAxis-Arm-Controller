"""Tkinter interface for a single Damiao DM4310 V3 motor."""

from __future__ import annotations

import math
import os
import queue
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from pathlib import Path
from tkinter import messagebox, ttk

import can
from dotenv import load_dotenv
from serial.tools import list_ports

from single_motor.damiao_v12 import (
    ConnectionCancelledError,
    DamiaoMotor,
    MoveCancelledError,
    MotorFeedback,
    MotorSettings,
    PresentParameter,
)
from single_motor.move_to_angle import validate_usb2can_port
from single_motor.usb2can_serial import DamiaoUSB2CANBus, USB2CAN_CAN_BAUDS_KBPS


PROJECT_DIR = Path(__file__).resolve().parents[1]
load_dotenv(PROJECT_DIR / ".env")


def env_int(name: str, default: int) -> int:
    value = os.getenv(name)
    return int(value, 0) if value else default


def env_float(name: str, default: float) -> float:
    value = os.getenv(name)
    return float(value) if value else default


@dataclass(frozen=True)
class SavedTarget:
    relative_angle_rad: float
    position_rad: float
    speed_rad_s: float
    kp: float
    kd: float
    arbitration_id: int
    payload: bytes


class MotorControlApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("DAMIAO | Motor Control")
        self.root.geometry("860x850")
        self.root.minsize(720, 700)

        self.events: queue.Queue[tuple[str, object]] = queue.Queue()
        self.cancel_event = threading.Event()
        self.connection_cancel_event = threading.Event()
        self.worker: threading.Thread | None = None
        self.bus: can.BusABC | None = None
        self.motor: DamiaoMotor | None = None
        self.settings: MotorSettings | None = None
        self.busy = False
        self.current_task: str | None = None
        self.motor_enabled = False
        self.disable_after_cancel = False
        self.closing = False
        self.jog_thread: threading.Thread | None = None
        self.jog_stop_event: threading.Event | None = None
        self.jog_direction = 0
        self.jog_target_position = 0.0
        self.jog_reference_position = 0.0
        self.jog_speed_limit = 0.0
        self.zero_offset_rad = 0.0
        self.active_control_panel = "target"
        self.saved_target: SavedTarget | None = None
        self.target_state_lock = threading.Lock()
        self.target_generation = 0
        self.target_move_cancel_event: threading.Event | None = None
        self.port_descriptions: dict[str, str] = {}

        self.channel_var = tk.StringVar(value=os.getenv("DAMIAO_CAN_CHANNEL", "COM3"))
        self.can_bitrate_var = tk.StringVar(value=str(env_int("DAMIAO_CAN_BITRATE", 1_000_000)))
        self.serial_baud_var = tk.StringVar(
            value=str(env_int("DAMIAO_CAN_SERIAL_BAUDRATE", 921_600))
        )
        self.can_id_var = tk.StringVar(value=f"0x{env_int('DAMIAO_CAN_ID', 0x02):02X}")
        self.master_id_var = tk.StringVar(value=f"0x{env_int('DAMIAO_MASTER_ID', 0x12):02X}")
        self.angle_var = tk.StringVar(value="0")
        self.control_panel_var = tk.StringVar(value="target")
        self.angle_unit_var = tk.StringVar(value="rad")
        self.speed_var = tk.StringVar(value="0.2")
        self.speed_unit_var = tk.StringVar(value="rad/s")
        self.jog_speed_var = tk.StringVar(value=str(env_float("DAMIAO_JOG_SPEED_RAD_S", 15.0)))
        self.kp_var = tk.StringVar(value="2.0")
        self.kd_var = tk.StringVar(value="1.0")
        self.status_var = tk.StringVar(value="Chưa kết nối")
        self.connection_state_var = tk.StringVar(value="CAN DISCONNECTED")
        self.driver_info_var = tk.StringVar(value="Chưa đọc thông tin driver")
        self.feedback_var = tk.StringVar(value="Vị trí: -- rad    Tốc độ: -- rad/s")
        self.motor_state_var = tk.StringVar(value="MOTOR DISABLED")
        self.hex_preview_var = tk.StringVar(value="Read driver mode to preview the CAN frame.")
        self.zero_status_var = tk.StringVar(value="Zero offset: 0.000 rad (not saved)")

        self._configure_style()
        self._build_widgets()
        self.refresh_ports()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.after(100, self._poll_events)

    def _configure_style(self) -> None:
        style = ttk.Style(self.root)
        if "clam" in style.theme_names():
            style.theme_use("clam")

        bg = "#020817"
        bg_panel = "#0b1327"
        bg_card = "#101a2d"
        accent = "#38bdf8"
        accent2 = "#7dd3fc"
        text = "#e2e8f0"
        muted = "#93c5fd"
        green = "#22c55e"
        red = "#ef4444"
        amber = "#f59e0b"

        self.root.configure(bg=bg)
        style.configure("TFrame", background=bg)
        style.configure("TLabel", background=bg, foreground=text)
        style.configure("Title.TLabel", font=("Segoe UI", 22, "bold"), foreground=accent2)
        style.configure("Hint.TLabel", foreground=muted)
        style.configure("TLabelFrame", background=bg, foreground=accent2)
        style.configure("TLabelFrame.Label", font=("Segoe UI", 10, "bold"), foreground=accent2)
        style.configure("TEntry", fieldbackground=bg_card, foreground=text)
        style.configure("TCombobox", fieldbackground=bg_card, foreground=text)
        style.configure("Primary.TButton", font=("Segoe UI", 10, "bold"), padding=(12, 8), background=accent, foreground="#03131d")
        style.map(
            "Primary.TButton",
            background=[("active", accent2), ("pressed", "#0ea5e9")],
            foreground=[("active", "#03131d"), ("pressed", "#03131d")],
        )
        style.configure("Danger.TButton", font=("Segoe UI", 10, "bold"), padding=(12, 8), background=red, foreground="#ffffff")
        style.map("Danger.TButton", background=[("active", "#f87171"), ("pressed", "#dc2626")])
        style.configure("MotorOff.TLabel", background="#4b5563", foreground="#ffffff", padding=(8, 4))
        style.configure("MotorOn.TLabel", background=green, foreground="#04110b", padding=(8, 4))
        style.configure("Connecting.TLabel", background=amber, foreground="#0b1014", padding=(8, 4))
        style.configure("Jog.TButton", font=("Segoe UI", 10, "bold"), padding=(12, 10), background=bg_panel, foreground=accent2)
        style.map("Jog.TButton", background=[("active", "#0f172a"), ("pressed", "#0b1327")], foreground=[("active", accent2), ("pressed", accent2)])

    def _build_widgets(self) -> None:
        container = ttk.Frame(self.root, padding=20)
        container.pack(fill="both", expand=True)
        container.columnconfigure(0, weight=1)

        ttk.Label(container, text="DAMIAO MOTOR CONTROL", style="Title.TLabel").grid(
            row=0, column=0, sticky="w"
        )
        ttk.Label(
            container,
            text="Single motor  /  DM4310 V3  /  CAN",
            style="Hint.TLabel",
        ).grid(row=1, column=0, sticky="w", pady=(2, 14))

        connection = ttk.LabelFrame(container, text="CAN connection", padding=12)
        connection.grid(row=2, column=0, sticky="ew", pady=(0, 10))
        connection.columnconfigure(1, weight=1)
        connection.columnconfigure(3, weight=1)

        ttk.Label(connection, text="COM port").grid(row=0, column=0, sticky="w", padx=(0, 8), pady=4)
        self.port_combo = ttk.Combobox(
            connection, textvariable=self.channel_var, values=(), width=14
        )
        self.port_combo.grid(row=0, column=1, sticky="ew", pady=4)
        ttk.Button(connection, text="Refresh", command=self.refresh_ports).grid(
            row=0, column=2, padx=8, pady=4
        )
        ttk.Label(connection, text="CAN bitrate").grid(
            row=0, column=3, sticky="e", padx=(8, 8), pady=4
        )
        ttk.Entry(connection, textvariable=self.can_bitrate_var, width=12).grid(
            row=0, column=4, sticky="e", pady=4
        )

        ttk.Label(connection, text="Adapter baud").grid(
            row=1, column=0, sticky="w", padx=(0, 8), pady=4
        )
        ttk.Entry(connection, textvariable=self.serial_baud_var, width=14).grid(
            row=1, column=1, sticky="w", pady=4
        )
        ttk.Label(connection, text="Vendor USB2CAN serial adapter", style="Hint.TLabel").grid(
            row=1, column=2, columnspan=3, sticky="e", pady=4
        )

        ids = ttk.LabelFrame(container, text="Motor IDs", padding=12)
        ids.grid(row=3, column=0, sticky="ew", pady=(0, 10))
        ids.columnconfigure(1, weight=1)
        ids.columnconfigure(3, weight=1)
        ttk.Label(ids, text="CAN ID").grid(row=0, column=0, sticky="w", padx=(0, 8))
        ttk.Entry(ids, textvariable=self.can_id_var, width=14).grid(
            row=0, column=1, sticky="ew", padx=(0, 18)
        )
        ttk.Label(ids, text="Master ID").grid(row=0, column=2, sticky="w", padx=(0, 8))
        ttk.Entry(ids, textvariable=self.master_id_var, width=14).grid(
            row=0, column=3, sticky="ew"
        )

        connect_actions = ttk.Frame(ids)
        connect_actions.grid(row=1, column=0, columnspan=4, sticky="ew", pady=(12, 4))
        connect_actions.columnconfigure(0, weight=1)
        connect_actions.columnconfigure(1, weight=1)
        self.apply_button = ttk.Button(
            connect_actions,
            text="Connect",
            style="Primary.TButton",
            command=self.apply_connection,
        )
        self.apply_button.grid(row=0, column=0, sticky="ew", padx=(0, 5))
        self.cancel_connection_button = ttk.Button(
            connect_actions,
            text="Cancel",
            command=self.cancel_connection,
            state="disabled",
        )
        self.cancel_connection_button.grid(row=0, column=1, sticky="ew", padx=(5, 0))
        ttk.Label(ids, textvariable=self.driver_info_var, style="Hint.TLabel").grid(
            row=2, column=0, columnspan=4, sticky="w", pady=(4, 0)
        )
        self.connection_state_label = ttk.Label(
            ids, textvariable=self.connection_state_var, style="MotorOff.TLabel"
        )
        self.connection_state_label.grid(row=3, column=0, columnspan=4, sticky="w", pady=(8, 0))

        modes = ttk.Frame(container)
        modes.grid(row=4, column=0, sticky="w", pady=(0, 8))
        ttk.Radiobutton(
            modes,
            text="Position target",
            value="target",
            variable=self.control_panel_var,
            command=self._change_control_panel,
        ).pack(side="left", padx=(0, 12))
        ttk.Radiobutton(
            modes,
            text="Manual jog",
            value="manual",
            variable=self.control_panel_var,
            command=self._change_control_panel,
        ).pack(side="left")

        target = ttk.LabelFrame(container, text="Position target", padding=12)
        self.target_panel = target
        target.grid(row=5, column=0, sticky="ew", pady=(0, 10))
        target.columnconfigure(1, weight=1)
        target.columnconfigure(4, weight=1)

        ttk.Label(target, text="Angle").grid(row=0, column=0, sticky="w", padx=(0, 8), pady=5)
        ttk.Entry(target, textvariable=self.angle_var, width=16).grid(
            row=0, column=1, sticky="ew", pady=5
        )
        ttk.Combobox(
            target,
            textvariable=self.angle_unit_var,
            values=("rad", "deg"),
            state="readonly",
            width=8,
        ).grid(row=0, column=2, padx=(8, 18), pady=5)

        ttk.Label(target, text="Speed").grid(row=0, column=3, sticky="w", padx=(0, 8), pady=5)
        ttk.Entry(target, textvariable=self.speed_var, width=16).grid(
            row=0, column=4, sticky="ew", pady=5
        )
        ttk.Combobox(
            target,
            textvariable=self.speed_unit_var,
            values=("rad/s", "rpm"),
            state="readonly",
            width=8,
        ).grid(row=0, column=5, padx=(8, 0), pady=5)

        ttk.Label(target, text="MIT Kp").grid(row=1, column=0, sticky="w", padx=(0, 8), pady=5)
        ttk.Entry(target, textvariable=self.kp_var, width=12).grid(
            row=1, column=1, sticky="w", pady=5
        )
        ttk.Label(target, text="MIT Kd").grid(row=1, column=2, sticky="e", padx=(8, 8), pady=5)
        ttk.Entry(target, textvariable=self.kd_var, width=12).grid(
            row=1, column=3, sticky="w", pady=5
        )
        ttk.Label(
            target,
            text="Position-PV uses speed as a maximum; MIT uses it as velocity feed-forward.",
            style="Hint.TLabel",
        ).grid(row=2, column=0, columnspan=6, sticky="w", pady=(7, 0))
        ttk.Label(
            target,
            textvariable=self.hex_preview_var,
            style="Hint.TLabel",
            wraplength=760,
        ).grid(row=3, column=0, columnspan=6, sticky="w", pady=(8, 0))
        self.save_zero_button = ttk.Button(
            target,
            text="Save current position as zero",
            command=self.save_zero,
            state="disabled",
        )
        self.save_zero_button.grid(row=4, column=0, columnspan=2, sticky="w", pady=(8, 0))
        ttk.Label(target, textvariable=self.zero_status_var, style="Hint.TLabel").grid(
            row=4, column=2, columnspan=4, sticky="w", padx=(10, 0), pady=(8, 0)
        )

        actions = ttk.Frame(container)
        actions.grid(row=6, column=0, sticky="ew", pady=(0, 10))
        for column in range(4):
            actions.columnconfigure(column, weight=1)
        self.enter_button = ttk.Button(
            actions,
            text="Enter Motor",
            style="Primary.TButton",
            command=self.enter_motor,
            state="disabled",
        )
        self.enter_button.grid(row=0, column=0, sticky="ew", padx=(0, 6))
        self.send_button = ttk.Button(
            actions,
            text="Save target",
            style="Primary.TButton",
            command=self.save_target,
            state="disabled",
        )
        self.send_button.grid(row=0, column=1, sticky="ew", padx=6)
        self.hold_send_button = ttk.Button(
            actions,
            text="Send",
            style="Jog.TButton",
            command=self.send_target,
            state="disabled",
        )
        self.hold_send_button.grid(row=0, column=2, sticky="ew", padx=6)
        self.stop_button = ttk.Button(
            actions,
            text="Stop / Disable",
            style="Danger.TButton",
            command=self.stop_motor,
            state="disabled",
        )
        self.stop_button.grid(row=0, column=3, sticky="ew", padx=(6, 0))

        jog = ttk.LabelFrame(container, text="Manual jog", padding=12)
        self.jog_panel = jog
        jog.grid(row=7, column=0, sticky="ew", pady=(0, 10))
        jog.columnconfigure(0, weight=1)
        jog.columnconfigure(1, weight=1)
        ttk.Label(jog, text="Jog speed (rad/s)").grid(row=0, column=0, sticky="w", padx=(0, 8), pady=(0, 5))
        ttk.Entry(jog, textvariable=self.jog_speed_var, width=12).grid(row=0, column=1, sticky="w", pady=(0, 5))

        self.jog_left_button = ttk.Button(
            jog,
            text="◀ Jog left",
            style="Jog.TButton",
            state="disabled",
        )
        self.jog_left_button.grid(row=1, column=0, sticky="ew", padx=(0, 6), pady=(6, 0))
        self.jog_left_button.bind("<ButtonPress-1>", lambda _event: self.start_jog(-1))
        self.jog_left_button.bind("<ButtonRelease-1>", lambda _event: self.stop_jog())

        self.jog_right_button = ttk.Button(
            jog,
            text="Jog right ▶",
            style="Jog.TButton",
            state="disabled",
        )
        self.jog_right_button.grid(row=1, column=1, sticky="ew", padx=(6, 0), pady=(6, 0))
        self.jog_right_button.bind("<ButtonPress-1>", lambda _event: self.start_jog(1))
        self.jog_right_button.bind("<ButtonRelease-1>", lambda _event: self.stop_jog())
        jog.grid_remove()

        self.motor_state_label = ttk.Label(
            container, textvariable=self.motor_state_var, style="MotorOff.TLabel"
        )
        self.motor_state_label.grid(row=8, column=0, sticky="w", pady=(0, 8))

        parameters = ttk.LabelFrame(container, text="Present parameters", padding=10)
        parameters.grid(row=9, column=0, sticky="nsew", pady=(0, 10))
        parameters.columnconfigure(0, weight=1)
        parameters.rowconfigure(1, weight=1)
        self.parameter_button = ttk.Button(
            parameters,
            text="Read present parameters",
            command=self.read_present_parameters,
            state="disabled",
        )
        self.parameter_button.grid(row=0, column=0, sticky="w", pady=(0, 8))
        table_frame = ttk.Frame(parameters)
        table_frame.grid(row=1, column=0, sticky="nsew")
        table_frame.columnconfigure(0, weight=1)
        table_frame.rowconfigure(0, weight=1)
        self.parameter_tree = ttk.Treeview(
            table_frame,
            columns=("address", "name", "value", "unit"),
            show="headings",
            height=8,
        )
        for column, title, width in (
            ("address", "Register", 75),
            ("name", "Parameter", 300),
            ("value", "Current value", 160),
            ("unit", "Unit", 100),
        ):
            self.parameter_tree.heading(column, text=title)
            self.parameter_tree.column(column, width=width, anchor="w")
        scrollbar = ttk.Scrollbar(
            table_frame, orient="vertical", command=self.parameter_tree.yview
        )
        self.parameter_tree.configure(yscrollcommand=scrollbar.set)
        self.parameter_tree.grid(row=0, column=0, sticky="nsew")
        scrollbar.grid(row=0, column=1, sticky="ns")

        ttk.Label(container, textvariable=self.feedback_var, font=("Segoe UI", 11, "bold")).grid(
            row=10, column=0, sticky="w", pady=(4, 8)
        )
        ttk.Label(container, textvariable=self.status_var, style="Hint.TLabel").grid(
            row=11, column=0, sticky="w", pady=(0, 8)
        )

        self.log = tk.Text(
            container,
            height=8,
            wrap="word",
            bg="#202923",
            fg="#e6eee9",
            insertbackground="#e6eee9",
            relief="flat",
            padx=10,
            pady=8,
            font=("Consolas", 9),
            state="disabled",
        )
        self.log.grid(row=12, column=0, sticky="nsew")
        container.rowconfigure(9, weight=1)
        container.rowconfigure(12, weight=1)
        self._log("Select COM and IDs, then Connect; use Cancel to abort a pending connection.")
        self._log("This interface does not write IDs or motor parameters to flash.")
        for variable in (
            self.angle_var,
            self.angle_unit_var,
            self.speed_var,
            self.speed_unit_var,
            self.kp_var,
            self.kd_var,
        ):
            variable.trace_add("write", self._update_hex_preview)
            variable.trace_add("write", self._update_target_buttons)

    def refresh_ports(self) -> None:
        ports = list(list_ports.comports())
        selected_ports = {}
        for port in ports:
            key = port.device.upper()
            current = selected_ports.get(key)
            is_usb = port.vid is not None and port.pid is not None
            current_is_usb = current is not None and current.vid is not None and current.pid is not None
            if current is None or (is_usb and not current_is_usb):
                selected_ports[key] = port
        self.port_descriptions = {
            port.device.upper(): port.description or "Serial device"
            for port in selected_ports.values()
        }
        devices = [port.device for port in selected_ports.values()]
        self.port_combo.configure(values=devices)
        selected = self.channel_var.get().upper()
        description = self.port_descriptions.get(selected)
        if description:
            self.status_var.set(f"{selected}: {description}")
        elif not devices:
            self.status_var.set("No serial ports detected")
        self._log("Detected ports: " + (", ".join(devices) if devices else "none"))

    def _parse_ids(self) -> tuple[int, int]:
        can_id = int(self.can_id_var.get().strip(), 0)
        master_id = int(self.master_id_var.get().strip(), 0)
        if not 0 <= can_id < 16:
            raise ValueError("CAN ID must be between 0x00 and 0x0F for this feedback format.")
        if not 0 <= master_id <= 0x7FF:
            raise ValueError("Master ID must be between 0x000 and 0x7FF.")
        return can_id, master_id

    def _start_worker(self, task_name: str, operation: object) -> None:
        if self.busy:
            return
        self.busy = True
        self.current_task = task_name
        self.apply_button.configure(state="disabled")
        self.cancel_connection_button.configure(
            state="normal" if task_name == "Connecting" else "disabled"
        )
        self.enter_button.configure(state="disabled")
        self.send_button.configure(state="disabled")
        self.parameter_button.configure(state="disabled")
        self.stop_button.configure(
            state="normal" if task_name in ("Moving motor", "Sending target") else "disabled"
        )
        self._update_target_buttons()
        self.status_var.set(task_name)

        def run() -> None:
            try:
                result = operation()  # type: ignore[operator]
            except ConnectionCancelledError as exc:
                self.events.put(("connection_cancelled", str(exc)))
            except MoveCancelledError as exc:
                force_disable = self.disable_after_cancel
                if force_disable and self.motor is not None:
                    self.motor.disable()
                self.disable_after_cancel = False
                message = (
                    "Target stopped; motor disabled."
                    if force_disable
                    else str(exc)
                )
                self.events.put(("cancelled", message))
            except Exception as exc:
                self.events.put(("error", str(exc)))
            else:
                self.events.put((task_name, result))

        self.worker = threading.Thread(target=run, name="damiao-gui-worker", daemon=True)
        self.worker.start()

    def apply_connection(self) -> None:
        try:
            can_id, master_id = self._parse_ids()
            channel = self.channel_var.get().strip()
            validate_usb2can_port(channel)
            bitrate = int(self.can_bitrate_var.get().strip(), 0)
            serial_baudrate = int(self.serial_baud_var.get().strip(), 0)
            if bitrate <= 0 or serial_baudrate <= 0:
                raise ValueError("CAN bitrate and adapter baud must be positive.")
            try:
                can_bitrate_code = USB2CAN_CAN_BAUDS_KBPS.index(bitrate // 1000)
            except ValueError as exc:
                raise ValueError(f"Unsupported Damiao USB2CAN CAN bitrate: {bitrate} bit/s.") from exc
        except (ValueError, RuntimeError) as exc:
            messagebox.showerror("Connection settings", str(exc), parent=self.root)
            return

        if self.motor is not None:
            if not messagebox.askyesno(
                "Reapply connection",
                "Disable the currently connected motor and reopen the CAN connection?\n\n"
                "Disabling removes active motor torque.",
                parent=self.root,
            ):
                return
            try:
                self.motor.disable()
                if self.bus is not None:
                    self.bus.shutdown()
            except can.CanError as exc:
                messagebox.showerror("CAN connection", str(exc), parent=self.root)
                return
            self.bus = None
            self.motor = None
            self.settings = None
            self.send_button.configure(state="disabled")
            self.stop_button.configure(state="disabled")

        self.connection_cancel_event.clear()
        self.connection_state_var.set("CAN CONNECTING")
        self.connection_state_label.configure(style="Connecting.TLabel")
        self._log(f"Connecting to {channel}; checking CAN response...")

        def connect() -> tuple[can.BusABC, DamiaoMotor, MotorSettings]:
            if self.bus is not None:
                self.bus.shutdown()
            bus = DamiaoUSB2CANBus(
                channel=channel,
                baudrate=serial_baudrate,
                can_bitrate_code=can_bitrate_code,
            )
            try:
                motor = DamiaoMotor(bus, can_id=can_id, master_id=master_id)
                settings = motor.verify_configuration(cancel_event=self.connection_cancel_event)
            except TimeoutError as exc:
                bus.shutdown()
                raise RuntimeError(
                    "COM port opened, but the motor did not answer the read-only CAN query. "
                    "Check that the USB2CAN adapter is connected to COM3, the UART is 921600, "
                    "the CAN bitrate matches the device configuration, and the motor IDs match. "
                    "No enable or movement command was sent."
                ) from exc
            except Exception:
                bus.shutdown()
                raise
            return bus, motor, settings

        self._start_worker("Connecting", connect)

    def cancel_connection(self) -> None:
        if self.busy and self.current_task == "Connecting":
            self.connection_cancel_event.set()
            self.cancel_connection_button.configure(state="disabled")
            self.status_var.set("Cancelling connection...")
            self._log("Connection cancelled by user; no motor enable/move command sent.")

    def _target_values(self) -> tuple[float, float, float, float]:
        angle_value = float(self.angle_var.get().strip())
        speed_value = float(self.speed_var.get().strip())
        kp = float(self.kp_var.get().strip())
        kd = float(self.kd_var.get().strip())
        if not all(math.isfinite(value) for value in (angle_value, speed_value, kp, kd)):
            raise ValueError("Angle, speed, Kp, and Kd must be finite numbers.")

        angle_rad = math.radians(angle_value) if self.angle_unit_var.get() == "deg" else angle_value
        speed_rad_s = (
            speed_value * math.tau / 60.0
            if self.speed_unit_var.get() == "rpm"
            else speed_value
        )
        return angle_rad, speed_rad_s, kp, kd

    def _motor_target_position(self, relative_angle_rad: float) -> float:
        return self.zero_offset_rad + relative_angle_rad

    def save_zero(self) -> None:
        sending_target = self.busy and self.current_task == "Sending target"
        if self.busy and not sending_target:
            self.status_var.set("Wait for the target operation to stop before saving zero")
            return
        feedback = self.motor.last_feedback if self.motor is not None else None
        if not self.motor_enabled or feedback is None:
            self.status_var.set("Enable the motor and wait for feedback before saving zero")
            return
        self.zero_offset_rad = feedback.position_rad
        self.zero_status_var.set(f"Zero offset: {self.zero_offset_rad:.3f} rad")
        with self.target_state_lock:
            previous_target = self.saved_target
        if previous_target is not None:
            try:
                rebased_target = self._build_saved_target(
                    previous_target.relative_angle_rad,
                    previous_target.speed_rad_s,
                    previous_target.kp,
                    previous_target.kd,
                )
            except ValueError as exc:
                rebased_target = None
                self.status_var.set(f"Zero saved; prior target cleared: {exc}")
            else:
                status = (
                    "Zero saved; stream restarting from current position"
                    if sending_target
                    else "Zero saved; saved target rebased from the current position"
                )
                self.status_var.set(status)
            with self.target_state_lock:
                self.saved_target = rebased_target
                self.target_generation += 1
                move_cancel_event = self.target_move_cancel_event if sending_target else None
            if move_cancel_event is not None:
                move_cancel_event.set()
        else:
            self.status_var.set("Current feedback position saved as software zero")
        self._log(f"Saved software zero at motor position {self.zero_offset_rad:.3f} rad.")
        self._update_target_buttons()
        self._update_hex_preview()

    def _change_control_panel(self) -> None:
        requested = self.control_panel_var.get()
        jog_active = self.jog_thread is not None and self.jog_thread.is_alive()
        if self.busy or jog_active:
            self.control_panel_var.set(self.active_control_panel)
            self.status_var.set("Stop the active operation before switching control modes")
            return
        if requested not in ("target", "manual"):
            self.control_panel_var.set(self.active_control_panel)
            return

        self.active_control_panel = requested
        if requested == "manual":
            self.angle_var.set("0")
            self.saved_target = None
            self.target_panel.grid_remove()
            self.jog_panel.grid()
            self.send_button.grid_remove()
            self.hold_send_button.grid_remove()
            self.stop_button.grid(row=0, column=1, sticky="ew", padx=6)
        else:
            self.jog_panel.grid_remove()
            self.target_panel.grid()
            self.send_button.grid(row=0, column=1, sticky="ew", padx=6)
            self.hold_send_button.grid(row=0, column=2, sticky="ew", padx=6)
            self.stop_button.grid(row=0, column=3, sticky="ew", padx=(6, 0))
        self._set_jog_buttons(self.motor_enabled)
        self._update_target_buttons()
        self._update_hex_preview()

    def _target_inputs_match_saved(self) -> bool:
        if self.saved_target is None or self.settings is None:
            return False
        try:
            angle_rad, speed_rad_s, kp, kd = self._target_values()
        except ValueError:
            return False
        return all(
            math.isclose(actual, expected, rel_tol=0.0, abs_tol=1e-9)
            for actual, expected in (
                (angle_rad, self.saved_target.relative_angle_rad),
                (self._motor_target_position(angle_rad), self.saved_target.position_rad),
                (speed_rad_s, self.saved_target.speed_rad_s),
                (kp, self.saved_target.kp),
                (kd, self.saved_target.kd),
            )
        )

    def _update_target_buttons(self, *_args: str) -> None:
        target_mode = self.active_control_panel == "target"
        can_save = target_mode and self.settings is not None and not self.busy
        can_send = (
            can_save
            and self.motor_enabled
            and self._target_inputs_match_saved()
        )
        self.send_button.configure(state="normal" if can_save else "disabled")
        self.hold_send_button.configure(state="normal" if can_send else "disabled")

    def _build_saved_target(
        self,
        relative_angle_rad: float,
        speed_rad_s: float,
        kp: float,
        kd: float,
    ) -> SavedTarget:
        if self.motor is None or self.settings is None:
            raise ValueError("Connect to the motor before saving a target.")
        if not all(math.isfinite(value) for value in (relative_angle_rad, speed_rad_s, kp, kd)):
            raise ValueError("Target angle, speed, Kp, and Kd must be finite numbers.")
        if speed_rad_s <= 0:
            raise ValueError("Speed must be greater than zero.")
        safe_speed = env_float("DAMIAO_SAFE_MAX_SPEED_RAD_S", 30.0)
        if speed_rad_s > min(safe_speed, self.settings.velocity_range_rad_s):
            raise ValueError(
                f"Speed must not exceed {min(safe_speed, self.settings.velocity_range_rad_s):g} rad/s."
            )
        position_rad = self._motor_target_position(relative_angle_rad)
        if abs(position_rad) > self.settings.position_range_rad:
            raise ValueError(
                f"Target maps to motor position {position_rad:.3f} rad, outside the "
                f"driver's PMAX range ±{self.settings.position_range_rad:g} rad."
            )
        if self.settings.control_mode == 1:
            feedback = getattr(self.motor, "last_feedback", None)
            current_position = (
                feedback.position_rad if feedback is not None else self.zero_offset_rad
            )
            remaining = position_rad - current_position
            frame_velocity = math.copysign(speed_rad_s, remaining) if remaining else 0.0
        else:
            frame_velocity = speed_rad_s
        arbitration_id, payload = self.motor.encode_position_command(
            position_rad, frame_velocity, kp, kd
        )
        return SavedTarget(
            relative_angle_rad=relative_angle_rad,
            position_rad=position_rad,
            speed_rad_s=speed_rad_s,
            kp=kp,
            kd=kd,
            arbitration_id=arbitration_id,
            payload=payload,
        )

    def _update_hex_preview(self, *_args: str) -> None:
        if self.motor is None or self.settings is None:
            self.hex_preview_var.set("Apply & read driver to preview its active CAN mode.")
            return
        try:
            angle_rad, speed_rad_s, kp, kd = self._target_values()
            preview = self._build_saved_target(angle_rad, speed_rad_s, kp, kd)
            saved = self.saved_target
            is_saved = saved is not None and all(
                math.isclose(actual, expected, abs_tol=1e-9)
                for actual, expected in (
                    (saved.relative_angle_rad, preview.relative_angle_rad),
                    (saved.position_rad, preview.position_rad),
                    (saved.speed_rad_s, preview.speed_rad_s),
                    (saved.kp, preview.kp),
                    (saved.kd, preview.kd),
                )
            )
            displayed = saved if is_saved else preview
            label = "SAVED TARGET" if is_saved else "UNSAVED PREVIEW"
            self.hex_preview_var.set(
                f"{label} CAN 0x{displayed.arbitration_id:03X}  "
                f"DATA {displayed.payload.hex(' ').upper()}  |  "
                f"Relative {displayed.relative_angle_rad:.3f} rad; "
                f"motor {displayed.position_rad:.3f} rad; stream profile changes frames"
            )
        except (ValueError, RuntimeError) as exc:
            self.hex_preview_var.set(f"Frame preview unavailable: {exc}")

    def _set_jog_buttons(self, enabled: bool) -> None:
        state = (
            "normal"
            if enabled and self.active_control_panel == "manual" and not self.busy
            else "disabled"
        )
        if hasattr(self, "jog_left_button"):
            self.jog_left_button.configure(state=state)
        if hasattr(self, "jog_right_button"):
            self.jog_right_button.configure(state=state)

    def _jog_speed_value(self) -> float:
        speed = float(self.jog_speed_var.get().strip())
        if not math.isfinite(speed) or speed <= 0:
            raise ValueError("Jog speed must be a positive value.")
        max_speed = min(30.0, self.settings.velocity_range_rad_s if self.settings is not None else 30.0)
        if speed > max_speed:
            raise ValueError(f"Jog speed must not exceed {max_speed:g} rad/s.")
        return speed

    def _send_jog_velocity(self, velocity_rad_s: float, delta_s: float) -> bool:
        if self.motor is None:
            return False
        try:
            kp_var = getattr(self, "kp_var", None)
            kd_var = getattr(self, "kd_var", None)
            kp = float((kp_var.get() if kp_var is not None else "2.0").strip())
            kd = float((kd_var.get() if kd_var is not None else "1.0").strip())
            limit_reached = False
            if self.settings.control_mode == 1:
                position_rad = self.jog_reference_position
                command_velocity = velocity_rad_s
                command_kp = 0.0
            elif self.settings.control_mode == 2:
                requested_position = self.jog_target_position + velocity_rad_s * delta_s
                position_limit = self.settings.position_range_rad
                position_rad = max(
                    -position_limit,
                    min(position_limit, requested_position),
                )
                limit_reached = position_rad != requested_position
                command_velocity = self.jog_speed_limit
                command_kp = kp
            else:
                raise RuntimeError(
                    f"Jog is not supported in control mode {self.settings.control_mode}."
                )
            arbitration_id, payload = self.motor.encode_position_command(
                position_rad,
                command_velocity,
                command_kp,
                kd,
            )
            self.motor._send(arbitration_id, payload)
            if self.settings.control_mode == 2:
                self.jog_target_position = position_rad
            return limit_reached
        except (ValueError, RuntimeError) as exc:
            self.events.put(("jog_error", str(exc)))
            raise

    def start_jog(self, direction: int) -> None:
        motor_enabled = getattr(self, "motor_enabled", getattr(self.motor, "enabled", False))
        if (
            self.active_control_panel != "manual"
            or self.motor is None
            or not motor_enabled
            or self.settings is None
        ):
            return
        if direction not in (-1, 1):
            return
        try:
            speed = self._jog_speed_value()
        except ValueError as exc:
            messagebox.showerror("Jog speed", str(exc), parent=self.root)
            return

        if self.jog_thread is not None and self.jog_thread.is_alive():
            return

        self.jog_direction = direction
        feedback = self.motor.last_feedback
        self.jog_reference_position = feedback.position_rad if feedback is not None else 0.0
        self.jog_target_position = self.jog_reference_position
        self.jog_speed_limit = speed
        self.jog_stop_event = threading.Event()
        initial_velocity = -direction * min(speed, 3.0 * 0.01)
        try:
            limit_reached = self._send_jog_velocity(initial_velocity, 0.01)
        except Exception:
            self.jog_stop_event = None
            return
        if limit_reached:
            self.jog_stop_event.set()

        self.jog_thread = threading.Thread(
            target=self._jog_loop,
            args=(direction, speed, self.jog_stop_event),
            name="damiao-jog-loop",
            daemon=True,
        )
        self.jog_thread.start()
        self.status_var.set(f"Jog {('left' if direction < 0 else 'right')} @ {abs(speed):.2f} rad/s")
        self._log(f"Jog {('left' if direction < 0 else 'right')} at {abs(speed):.2f} rad/s.")

    def _jog_loop(
        self, direction: int, speed: float, stop_event: threading.Event
    ) -> None:
        acceleration_rad_s2 = 3.0
        velocity = -direction * min(speed, acceleration_rad_s2 * 0.01)
        last_update = time.monotonic()
        try:
            while self.motor is not None and getattr(
                self, "motor_enabled", getattr(self.motor, "enabled", False)
            ):
                now = time.monotonic()
                delta_s = min(0.05, max(0.001, now - last_update))
                last_update = now
                stopping = stop_event.is_set()
                target_velocity = 0.0 if stopping else -direction * speed
                max_change = acceleration_rad_s2 * delta_s
                velocity += max(
                    -max_change,
                    min(max_change, target_velocity - velocity),
                )
                limit_reached = self._send_jog_velocity(velocity, delta_s)
                if limit_reached and not stopping:
                    self.events.put(("jog_limit", "Position target reached the driver's PMAX limit."))
                    stop_event.set()
                    stopping = True

                read_feedback = getattr(self.motor, "_read_feedback", None)
                feedback = read_feedback(timeout_s=0.001) if read_feedback else None
                if feedback is not None:
                    self.events.put(("feedback", feedback))

                if stopping and abs(velocity) <= 0.01:
                    break
                stop_event.wait(0.01)

            if self.motor is not None and getattr(self.motor, "enabled", False):
                hold_position = (
                    self.jog_reference_position
                    if self.settings.control_mode == 1
                    else self.jog_target_position
                )
                kp_var = getattr(self, "kp_var", None)
                kd_var = getattr(self, "kd_var", None)
                kp = float((kp_var.get() if kp_var is not None else "2.0").strip())
                kd = float((kd_var.get() if kd_var is not None else "1.0").strip())
                hold_velocity = 0.0 if self.settings.control_mode == 1 else self.jog_speed_limit
                arbitration_id, payload = self.motor.encode_position_command(
                    hold_position,
                    hold_velocity,
                    0.0 if self.settings.control_mode == 1 else kp,
                    kd,
                )
                self.motor._send(arbitration_id, payload)
                self.jog_target_position = hold_position
            self.events.put(("jog_stopped", None))
        except (ValueError, RuntimeError, can.CanError, OSError) as exc:
            stop_event.set()
            self.events.put(("jog_error", str(exc)))

    def stop_jog(self) -> None:
        if self.jog_stop_event is None:
            return
        self.jog_stop_event.set()
        self.status_var.set("Jog decelerating; motor remains enabled")
        self._log("Jog released; decelerating to a hold without disabling the motor.")

    def enter_motor(self) -> None:
        if self.motor is None:
            return
        if self.motor_enabled:
            return
        motor = self.motor

        def enable() -> MotorFeedback:
            return motor.enable()

        self._log(
            f"Enter Motor: TX CAN ID 0x{motor.control_frame_id:03X} "
            "DATA FF FF FF FF FF FF FF FC"
        )
        self._start_worker("Entering Motor", enable)

    def read_present_parameters(self) -> None:
        if self.motor is None:
            return
        self._log("Reading present motor parameters over CAN...")
        self._start_worker("Reading parameters", self.motor.read_present_parameters)

    def send_target(self) -> None:
        if self.active_control_panel != "target":
            return
        if not self.motor_enabled or self.motor is None or self.settings is None:
            messagebox.showwarning(
                "Motor disabled",
                "Connect and Enter Motor before sending a saved target.",
                parent=self.root,
            )
            return
        if self.saved_target is None:
            messagebox.showwarning(
                "No saved target",
                "Save a target before holding Send.",
                parent=self.root,
            )
            return
        if not self._target_inputs_match_saved():
            messagebox.showwarning(
                "Unsaved target changes",
                "Save the edited target before sending it.",
                parent=self.root,
            )
            return
        if self.busy:
            return

        target = self.saved_target
        motor = self.motor
        self.cancel_event.clear()
        self.disable_after_cancel = False

        def move() -> MotorFeedback:
            interval_s = 1.0 / env_float("DAMIAO_COMMAND_RATE_HZ", 20.0)
            while not self.cancel_event.is_set():
                with self.target_state_lock:
                    target = self.saved_target
                    generation = self.target_generation
                    move_cancel_event = threading.Event()
                    self.target_move_cancel_event = move_cancel_event
                if target is None:
                    raise RuntimeError("No valid saved target remains to send.")

                try:
                    feedback = motor.move_to_position(
                        position_rad=target.position_rad,
                        max_velocity_rad_s=target.speed_rad_s,
                        position_tolerance_rad=env_float("DAMIAO_POSITION_TOLERANCE_RAD", 0.05),
                        velocity_tolerance_rad_s=env_float("DAMIAO_VELOCITY_TOLERANCE_RAD_S", 0.1),
                        timeout_s=env_float("DAMIAO_MOVE_TIMEOUT_S", 30.0),
                        command_rate_hz=env_float("DAMIAO_COMMAND_RATE_HZ", 20.0),
                        kp=target.kp,
                        kd=target.kd,
                        cancel_event=move_cancel_event,
                        progress_callback=lambda value: self.events.put(("feedback", value)),
                        enable_first=False,
                        disable_on_cancel=False,
                    )
                except MoveCancelledError:
                    with self.target_state_lock:
                        target_changed = generation != self.target_generation
                        if self.target_move_cancel_event is move_cancel_event:
                            self.target_move_cancel_event = None
                    if self.cancel_event.is_set():
                        raise MoveCancelledError("Send stopped; motor disable requested.")
                    if target_changed:
                        continue
                    raise
                finally:
                    with self.target_state_lock:
                        if self.target_move_cancel_event is move_cancel_event:
                            self.target_move_cancel_event = None

                with self.target_state_lock:
                    target_changed = generation != self.target_generation
                if target_changed:
                    continue

                hold_velocity = (
                    0.0 if self.settings.control_mode == 1 else target.speed_rad_s
                )
                while not self.cancel_event.is_set():
                    with self.target_state_lock:
                        if generation != self.target_generation:
                            break
                    if not motor.enabled:
                        raise RuntimeError("Motor is no longer enabled while holding the target.")
                    arbitration_id, payload = motor.encode_position_command(
                        target.position_rad,
                        hold_velocity,
                        target.kp,
                        target.kd,
                    )
                    motor._send(arbitration_id, payload)
                    feedback = motor._read_feedback(timeout_s=interval_s)
                    if feedback is not None:
                        if not feedback.is_enabled:
                            motor.enabled = False
                            raise RuntimeError(
                                f"Drive reported fault code {feedback.error_code}; "
                                "motor commands stopped for safety."
                            )
                        self.events.put(("feedback", feedback))
                    self.cancel_event.wait(interval_s)

            raise MoveCancelledError("Send stopped; motor disable requested.")

        self._log(
            "Send stream started; target setpoints will update from the saved profile."
        )
        self._start_worker("Sending target", move)

    def save_target(self) -> None:
        if self.active_control_panel != "target" or self.busy:
            return
        try:
            relative_angle_rad, speed_rad_s, kp, kd = self._target_values()
            target = self._build_saved_target(relative_angle_rad, speed_rad_s, kp, kd)
        except ValueError as exc:
            messagebox.showerror("Target values", str(exc), parent=self.root)
            return

        with self.target_state_lock:
            self.saved_target = target
            self.target_generation += 1
        self._update_hex_preview()
        self.status_var.set("Target saved; click Send to start")
        self._log(
            f"Saved target CAN 0x{target.arbitration_id:03X} "
            f"[{target.payload.hex(' ').upper()}]."
        )
        self._update_target_buttons()

    def stop_motor(self) -> None:
        if self.busy:
            self.disable_after_cancel = self.current_task == "Sending target"
            self.cancel_event.set()
            with self.target_state_lock:
                move_cancel_event = self.target_move_cancel_event
            if move_cancel_event is not None:
                move_cancel_event.set()
            self.status_var.set("Stopping: sending disable command...")
            self._log("Stop requested; cancellation sends Disable and removes motor torque.")
            return
        if self.motor is None:
            return

        motor = self.motor

        def disable() -> str:
            motor.disable()
            return "Motor disabled; active torque removed."

        self._start_worker("Disabling motor", disable)

    def _log(self, text: str) -> None:
        self.log.configure(state="normal")
        self.log.insert("end", text + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    def _poll_events(self) -> None:
        while True:
            try:
                event, payload = self.events.get_nowait()
            except queue.Empty:
                break

            if event == "feedback":
                feedback = payload
                if isinstance(feedback, MotorFeedback):
                    self._show_feedback(feedback)
                continue
            if event == "jog_error":
                self.jog_thread = None
                self.jog_stop_event = None
                self.status_var.set("Jog command failed")
                self._log(f"JOG ERROR: {payload}")
                messagebox.showerror("Motor jog", str(payload), parent=self.root)
                continue
            if event == "jog_limit":
                self.status_var.set("Jog stopped at the driver's position limit")
                self._log(str(payload))
                continue
            if event == "jog_stopped":
                self.jog_thread = None
                self.jog_stop_event = None
                self.status_var.set("Jog stopped; motor remains enabled")
                self._log("Jog stopped at hold position; motor remains enabled.")
                continue

            finished_task = self.current_task
            self.current_task = None
            self.busy = False
            self.worker = None
            self.cancel_event.clear()
            self.cancel_connection_button.configure(state="disabled")
            if finished_task == "Connecting":
                self.connection_cancel_event.clear()
            self.apply_button.configure(state="normal")
            self.motor_enabled = bool(self.motor is not None and self.motor.enabled)
            self.enter_button.configure(
                state="normal" if self.motor is not None and not self.motor_enabled else "disabled"
            )
            self._update_target_buttons()
            self.parameter_button.configure(state="normal" if self.motor is not None else "disabled")
            self.stop_button.configure(state="normal" if self.motor_enabled else "disabled")
            self._set_jog_buttons(self.motor_enabled)
            self.save_zero_button.configure(
                state="normal" if self.motor_enabled and self.motor.last_feedback is not None else "disabled"
            )
            if event == "error":
                if finished_task == "Connecting":
                    self.bus = None
                    self.motor = None
                    self.settings = None
                    self.connection_state_var.set("CAN DISCONNECTED")
                    self.connection_state_label.configure(style="MotorOff.TLabel")
                    self.enter_button.configure(state="disabled")
                    self.send_button.configure(state="disabled")
                    self.parameter_button.configure(state="disabled")
                self.status_var.set("Operation failed")
                self._log(f"ERROR: {payload}")
                messagebox.showerror("Motor operation", str(payload), parent=self.root)
            elif event == "connection_cancelled":
                self.connection_state_var.set("CAN DISCONNECTED")
                self.connection_state_label.configure(style="MotorOff.TLabel")
                self.status_var.set("Connection cancelled")
                self._log(str(payload))
            elif event == "cancelled":
                self._set_motor_state(self.motor_enabled)
                self.status_var.set(str(payload))
                self._log(str(payload))
            elif event == "Connecting":
                bus, motor, settings = payload
                self.bus = bus
                self.motor = motor
                self.settings = settings
                self.zero_offset_rad = 0.0
                self.saved_target = None
                self.zero_status_var.set("Zero offset: 0.000 rad (not saved)")
                self.motor_enabled = False
                self.connection_state_var.set("CAN CONNECTED")
                self.connection_state_label.configure(style="MotorOn.TLabel")
                self._set_motor_state(False)
                self.enter_button.configure(state="normal")
                self.send_button.configure(state="disabled")
                self.parameter_button.configure(state="normal")
                mode_name = "MIT" if settings.control_mode == 1 else "Position-Velocity"
                self.driver_info_var.set(
                    f"Firmware {settings.firmware_version}.{settings.sub_version:03d}  |  "
                    f"Mode {mode_name}  |  CAN 0x{settings.can_id:02X}  |  "
                    f"Master 0x{settings.master_id:03X}  |  "
                    f"PMAX ±{settings.position_range_rad:g} rad  |  "
                    f"VMAX {settings.velocity_range_rad_s:g} rad/s"
                )
                self.status_var.set("Connected and verified")
                self._log(self.driver_info_var.get())
                self._update_hex_preview()
                self._update_target_buttons()
            elif event == "Entering Motor":
                feedback = payload
                if isinstance(feedback, MotorFeedback):
                    self._show_feedback(feedback)
                self.motor_enabled = True
                self._set_motor_state(True)
                self.enter_button.configure(state="disabled")
                self.stop_button.configure(state="normal")
                self._update_target_buttons()
                self.status_var.set("Motor enabled; ready to send target")
                self._log("Enable acknowledged: motor status is green.")
            elif event == "Moving motor":
                feedback = payload
                if isinstance(feedback, MotorFeedback):
                    self._show_feedback(feedback)
                self.motor_enabled = bool(self.motor is not None and self.motor.enabled)
                self._set_motor_state(self.motor_enabled)
                self.status_var.set("Target reached; motor remains enabled")
                self._log("Target reached. Motor is still enabled; use Stop / disable torque when safe.")
            elif event == "Reading parameters":
                parameters = payload
                if isinstance(parameters, list):
                    self._display_parameters(parameters)
                    self.status_var.set(f"Read {len(parameters)} parameters from motor")
                    self._log(f"Read {len(parameters)} present parameters.")
            elif event == "Disabling motor":
                self.motor_enabled = False
                self._set_motor_state(False)
                self.enter_button.configure(state="normal" if self.motor is not None else "disabled")
                self.send_button.configure(state="disabled")
                self.hold_send_button.configure(state="disabled")
                self.stop_button.configure(state="disabled")
                self._set_jog_buttons(False)
                self.status_var.set(str(payload))
                self._log(str(payload))

        if self.closing and not self.busy:
            self._finish_close()
        else:
            self.root.after(100, self._poll_events)

    def _set_motor_state(self, enabled: bool) -> None:
        self.motor_state_var.set("MOTOR ENABLED" if enabled else "MOTOR DISABLED")
        self.motor_state_label.configure(style="MotorOn.TLabel" if enabled else "MotorOff.TLabel")

    def _show_feedback(self, feedback: MotorFeedback) -> None:
        self.motor_enabled = bool(
            self.motor is not None and self.motor.enabled and feedback.is_enabled
        )
        self._set_motor_state(self.motor_enabled)
        if hasattr(self, "save_zero_button"):
            self.save_zero_button.configure(state="normal" if self.motor_enabled else "disabled")
        relative_position = feedback.position_rad - self.zero_offset_rad
        self.feedback_var.set(
            f"Pos {relative_position:.3f} rad from zero  |  "
            f"Vel {feedback.velocity_rad_s:.3f} rad/s  |  "
            f"Torque {feedback.torque_nm:.3f} Nm  |  "
            f"Driver {feedback.driver_temperature_c} C  |  "
            f"Motor {feedback.motor_temperature_c} C  |  "
            f"Status {feedback.status_text} (0x{feedback.status_code:X})"
        )

    def _display_parameters(self, parameters: list[PresentParameter]) -> None:
        for item in self.parameter_tree.get_children():
            self.parameter_tree.delete(item)
        mode_names = {1: "MIT", 2: "Position-Velocity", 3: "Velocity", 4: "Hybrid"}
        baud_rates = {0: "125 kbit/s", 1: "200 kbit/s", 2: "250 kbit/s", 3: "500 kbit/s", 4: "1 Mbit/s"}
        for parameter in parameters:
            value = parameter.value
            if parameter.unit == "hex":
                value_text = f"0x{int(value):03X}"
            elif parameter.address == 0x0A:
                value_text = mode_names.get(int(value), str(value))
            elif parameter.address == 0x23:
                value_text = baud_rates.get(int(value), f"code {value}")
            elif isinstance(value, int):
                value_text = str(value)
            else:
                value_text = f"{value:.7g}"
            self.parameter_tree.insert(
                "",
                "end",
                values=(f"0x{parameter.address:02X}", parameter.name, value_text, parameter.unit),
            )

    def _on_close(self) -> None:
        if self.busy:
            if messagebox.askyesno(
                "Stop motor and close?",
                "A motor operation is active. Cancel it, send Disable, and close?",
                parent=self.root,
            ):
                self.closing = True
                if self.current_task == "Connecting":
                    self.connection_cancel_event.set()
                    self.status_var.set("Cancelling connection before closing...")
                else:
                    self.disable_after_cancel = self.current_task == "Sending target"
                    self.cancel_event.set()
                    with self.target_state_lock:
                        move_cancel_event = self.target_move_cancel_event
                    if move_cancel_event is not None:
                        move_cancel_event.set()
                    self.status_var.set("Stopping motor before closing...")
            return

        disable = False
        if self.motor is not None:
            if not messagebox.askyesno(
                "Close controller",
                "Disable the motor and close? Disabling removes active torque.",
                parent=self.root,
            ):
                return
            disable = True

        self.closing = True
        self._finish_close(disable=disable)

    def _finish_close(self, disable: bool = False) -> None:
        if self.bus is not None:
            try:
                if disable and self.motor is not None:
                    self.motor.disable()
                self.bus.shutdown()
            except can.CanError:
                pass
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    MotorControlApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
