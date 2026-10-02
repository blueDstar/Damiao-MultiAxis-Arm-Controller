"""Tkinter interface for a single Damiao DM4310 V3 motor."""

from __future__ import annotations

import math
import os
import queue
import threading
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk

import can
from dotenv import load_dotenv
from serial.tools import list_ports

from single_motor.damiao_v12 import (
    DamiaoMotor,
    MoveCancelledError,
    MotorFeedback,
    MotorSettings,
    PresentParameter,
)
from single_motor.move_to_angle import validate_slcan_port


PROJECT_DIR = Path(__file__).resolve().parents[1]
load_dotenv(PROJECT_DIR / ".env")


def env_int(name: str, default: int) -> int:
    value = os.getenv(name)
    return int(value, 0) if value else default


def env_float(name: str, default: float) -> float:
    value = os.getenv(name)
    return float(value) if value else default


class MotorControlApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("DAMIAO | Motor Control")
        self.root.geometry("860x850")
        self.root.minsize(720, 700)

        self.events: queue.Queue[tuple[str, object]] = queue.Queue()
        self.cancel_event = threading.Event()
        self.worker: threading.Thread | None = None
        self.bus: can.BusABC | None = None
        self.motor: DamiaoMotor | None = None
        self.settings: MotorSettings | None = None
        self.busy = False
        self.motor_enabled = False
        self.closing = False
        self.port_descriptions: dict[str, str] = {}

        self.channel_var = tk.StringVar(value=os.getenv("DAMIAO_CAN_CHANNEL", "COM3"))
        self.can_bitrate_var = tk.StringVar(value=str(env_int("DAMIAO_CAN_BITRATE", 1_000_000)))
        self.serial_baud_var = tk.StringVar(
            value=str(env_int("DAMIAO_CAN_SERIAL_BAUDRATE", 115_200))
        )
        self.can_id_var = tk.StringVar(value=f"0x{env_int('DAMIAO_CAN_ID', 0x01):02X}")
        self.master_id_var = tk.StringVar(value=f"0x{env_int('DAMIAO_MASTER_ID', 0x11):02X}")
        self.angle_var = tk.StringVar(value="0")
        self.angle_unit_var = tk.StringVar(value="rad")
        self.speed_var = tk.StringVar(value="0.2")
        self.speed_unit_var = tk.StringVar(value="rad/s")
        self.kp_var = tk.StringVar(value="2.0")
        self.kd_var = tk.StringVar(value="1.0")
        self.status_var = tk.StringVar(value="Chưa kết nối")
        self.driver_info_var = tk.StringVar(value="Chưa đọc thông tin driver")
        self.feedback_var = tk.StringVar(value="Vị trí: -- rad    Tốc độ: -- rad/s")
        self.motor_state_var = tk.StringVar(value="MOTOR DISABLED")
        self.hex_preview_var = tk.StringVar(value="Read driver mode to preview the CAN frame.")

        self._configure_style()
        self._build_widgets()
        self.refresh_ports()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.after(100, self._poll_events)

    def _configure_style(self) -> None:
        style = ttk.Style(self.root)
        if "clam" in style.theme_names():
            style.theme_use("clam")
        self.root.configure(bg="#f2f4f1")
        style.configure("TFrame", background="#f2f4f1")
        style.configure("TLabel", background="#f2f4f1", foreground="#202923")
        style.configure("Title.TLabel", font=("Segoe UI", 18, "bold"), foreground="#183b34")
        style.configure("Hint.TLabel", foreground="#58655e")
        style.configure("TLabelFrame", background="#f2f4f1", foreground="#183b34")
        style.configure("TLabelFrame.Label", font=("Segoe UI", 10, "bold"))
        style.configure("Primary.TButton", font=("Segoe UI", 10, "bold"), padding=(12, 8))
        style.configure("Danger.TButton", foreground="#8f2d25", padding=(12, 8))
        style.configure("MotorOff.TLabel", background="#9d3328", foreground="#ffffff", padding=(8, 4))
        style.configure("MotorOn.TLabel", background="#187448", foreground="#ffffff", padding=(8, 4))

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
        ttk.Label(connection, text="SLCAN-compatible USB2CAN", style="Hint.TLabel").grid(
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

        self.apply_button = ttk.Button(
            ids,
            text="Apply & read driver",
            style="Primary.TButton",
            command=self.apply_connection,
        )
        self.apply_button.grid(row=1, column=0, columnspan=4, sticky="ew", pady=(12, 4))
        ttk.Label(ids, textvariable=self.driver_info_var, style="Hint.TLabel").grid(
            row=2, column=0, columnspan=4, sticky="w", pady=(4, 0)
        )

        target = ttk.LabelFrame(container, text="Position target", padding=12)
        target.grid(row=4, column=0, sticky="ew", pady=(0, 10))
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

        actions = ttk.Frame(container)
        actions.grid(row=5, column=0, sticky="ew", pady=(0, 10))
        actions.columnconfigure(0, weight=1)
        actions.columnconfigure(1, weight=1)
        actions.columnconfigure(2, weight=1)
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
            text="Send target",
            style="Primary.TButton",
            command=self.send_target,
            state="disabled",
        )
        self.send_button.grid(row=0, column=1, sticky="ew", padx=6)
        self.stop_button = ttk.Button(
            actions,
            text="Stop / Disable",
            style="Danger.TButton",
            command=self.stop_motor,
            state="disabled",
        )
        self.stop_button.grid(row=0, column=2, sticky="ew", padx=(6, 0))

        self.motor_state_label = ttk.Label(
            container, textvariable=self.motor_state_var, style="MotorOff.TLabel"
        )
        self.motor_state_label.grid(row=6, column=0, sticky="w", pady=(0, 8))

        parameters = ttk.LabelFrame(container, text="Present parameters", padding=10)
        parameters.grid(row=7, column=0, sticky="nsew", pady=(0, 10))
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
            row=8, column=0, sticky="w", pady=(4, 8)
        )
        ttk.Label(container, textvariable=self.status_var, style="Hint.TLabel").grid(
            row=9, column=0, sticky="w", pady=(0, 8)
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
        self.log.grid(row=10, column=0, sticky="nsew")
        container.rowconfigure(7, weight=1)
        container.rowconfigure(10, weight=1)
        self._log("Set the IDs used by the driver, then Apply & read driver.")
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
        self.apply_button.configure(state="disabled")
        self.enter_button.configure(state="disabled")
        self.send_button.configure(state="disabled")
        self.parameter_button.configure(state="disabled")
        self.stop_button.configure(state="normal" if task_name == "Moving motor" else "disabled")
        self.status_var.set(task_name)

        def run() -> None:
            try:
                result = operation()  # type: ignore[operator]
            except MoveCancelledError as exc:
                self.events.put(("cancelled", str(exc)))
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
            validate_slcan_port(channel)
            bitrate = int(self.can_bitrate_var.get().strip(), 0)
            serial_baudrate = int(self.serial_baud_var.get().strip(), 0)
            if bitrate <= 0 or serial_baudrate <= 0:
                raise ValueError("CAN bitrate and adapter baud must be positive.")
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

        self._log(f"Connecting to {channel}; reading driver registers...")

        def connect() -> tuple[can.BusABC, DamiaoMotor, MotorSettings]:
            if self.bus is not None:
                self.bus.shutdown()
            bus = can.Bus(
                interface="slcan",
                channel=channel,
                bitrate=bitrate,
                tty_baudrate=serial_baudrate,
            )
            try:
                motor = DamiaoMotor(bus, can_id=can_id, master_id=master_id)
                settings = motor.verify_configuration()
            except TimeoutError as exc:
                bus.shutdown()
                raise RuntimeError(
                    "COM port opened, but the motor did not answer the read-only CAN query. "
                    "Check that this USB2CAN supports SLCAN, CAN bitrate is 1 Mbps, and the "
                    "motor IDs match. No enable or movement command was sent. If Damiao "
                    "Debugging Tool reads this same adapter, it may use a vendor USB2CAN "
                    "driver/DLL instead of SLCAN."
                ) from exc
            except Exception:
                bus.shutdown()
                raise
            return bus, motor, settings

        self._start_worker("Applying connection", connect)

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

    def _update_hex_preview(self, *_args: str) -> None:
        if self.motor is None or self.settings is None:
            self.hex_preview_var.set("Apply & read driver to preview its active CAN mode.")
            return
        try:
            angle_rad, speed_rad_s, kp, kd = self._target_values()
            arbitration_id, payload = self.motor.encode_position_command(
                angle_rad, speed_rad_s, kp, kd
            )
            self.hex_preview_var.set(
                f"TX CAN ID 0x{arbitration_id:03X}  DATA {payload.hex(' ').upper()}  |  "
                f"Feedback Master 0x{self.settings.master_id:03X}"
            )
        except (ValueError, RuntimeError) as exc:
            self.hex_preview_var.set(f"Frame preview unavailable: {exc}")

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
        if self.motor is None or self.settings is None:
            messagebox.showwarning("Motor not applied", "Apply the CAN settings and read the driver first.")
            return
        if not self.motor_enabled:
            messagebox.showwarning(
                "Motor disabled",
                "Enter Motor first and wait for the status indicator to turn green.",
                parent=self.root,
            )
            return

        try:
            angle_rad, speed_rad_s, kp, kd = self._target_values()
            safe_speed = env_float("DAMIAO_SAFE_MAX_SPEED_RAD_S", 0.5)
            if speed_rad_s <= 0:
                raise ValueError("Speed must be greater than zero.")
            if speed_rad_s > safe_speed:
                raise ValueError(
                    f"Speed exceeds the initial software cap of {safe_speed:g} rad/s in .env."
                )
            if abs(angle_rad) > self.settings.position_range_rad:
                raise ValueError(
                    f"Angle is outside the driver's PMAX range ±{self.settings.position_range_rad:g} rad."
                )
            if speed_rad_s > self.settings.velocity_range_rad_s:
                raise ValueError(
                    f"Speed exceeds the driver's VMAX {self.settings.velocity_range_rad_s:g} rad/s."
                )
        except ValueError as exc:
            messagebox.showerror("Target values", str(exc), parent=self.root)
            return

        if not messagebox.askyesno(
            "Confirm motor movement",
            f"Move to {angle_rad:.3f} rad at {speed_rad_s:.3f} rad/s?\n\n"
            "Make sure the motor is securely mounted and the path is clear.",
            parent=self.root,
        ):
            return

        self.cancel_event.clear()
        motor = self.motor

        def move() -> MotorFeedback:
            return motor.move_to_position(
                position_rad=angle_rad,
                max_velocity_rad_s=speed_rad_s,
                position_tolerance_rad=env_float("DAMIAO_POSITION_TOLERANCE_RAD", 0.05),
                velocity_tolerance_rad_s=env_float("DAMIAO_VELOCITY_TOLERANCE_RAD_S", 0.1),
                timeout_s=env_float("DAMIAO_MOVE_TIMEOUT_S", 30.0),
                command_rate_hz=env_float("DAMIAO_COMMAND_RATE_HZ", 20.0),
                kp=kp,
                kd=kd,
                cancel_event=self.cancel_event,
                progress_callback=lambda feedback: self.events.put(("feedback", feedback)),
                enable_first=False,
            )

        self._log(
            f"Send target: {angle_rad:.3f} rad, {speed_rad_s:.3f} rad/s "
            f"using mode {self.settings.control_mode}."
        )
        self._start_worker("Moving motor", move)

    def stop_motor(self) -> None:
        if self.busy:
            self.cancel_event.set()
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

            self.busy = False
            self.worker = None
            self.cancel_event.clear()
            self.apply_button.configure(state="normal")
            self.motor_enabled = bool(self.motor is not None and self.motor.enabled)
            self.enter_button.configure(
                state="normal" if self.motor is not None and not self.motor_enabled else "disabled"
            )
            self.send_button.configure(state="normal" if self.motor_enabled else "disabled")
            self.parameter_button.configure(state="normal" if self.motor is not None else "disabled")
            self.stop_button.configure(state="normal" if self.motor_enabled else "disabled")
            self._set_motor_state(self.motor_enabled)

            if event == "error":
                self.status_var.set("Operation failed")
                self._log(f"ERROR: {payload}")
                messagebox.showerror("Motor operation", str(payload), parent=self.root)
            elif event == "cancelled":
                self.status_var.set("Move stopped; motor disabled")
                self._log(str(payload))
            elif event == "Applying connection":
                bus, motor, settings = payload
                self.bus = bus
                self.motor = motor
                self.settings = settings
                self.motor_enabled = False
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
            elif event == "Entering Motor":
                feedback = payload
                if isinstance(feedback, MotorFeedback):
                    self._show_feedback(feedback)
                self.motor_enabled = True
                self._set_motor_state(True)
                self.enter_button.configure(state="disabled")
                self.send_button.configure(state="normal")
                self.stop_button.configure(state="normal")
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
                self.stop_button.configure(state="disabled")
                self.status_var.set(str(payload))
                self._log(str(payload))

    def _set_motor_state(self, enabled: bool) -> None:
        self.motor_state_var.set("MOTOR ENABLED" if enabled else "MOTOR DISABLED")
        self.motor_state_label.configure(style="MotorOn.TLabel" if enabled else "MotorOff.TLabel")

    def _show_feedback(self, feedback: MotorFeedback) -> None:
        self.motor_enabled = feedback.status == 1
        self._set_motor_state(self.motor_enabled)
        self.feedback_var.set(
            f"Pos {feedback.position_rad:.3f} rad  |  "
            f"Vel {feedback.velocity_rad_s:.3f} rad/s  |  "
            f"Torque {feedback.torque_nm:.3f} Nm  |  "
            f"Driver {feedback.driver_temperature_c} C  |  "
            f"Motor {feedback.motor_temperature_c} C  |  "
            f"Status {feedback.status}"
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

        if self.closing and not self.busy:
            self._finish_close()
        else:
            self.root.after(100, self._poll_events)

    def _on_close(self) -> None:
        if self.busy:
            if messagebox.askyesno(
                "Stop motor and close?",
                "A motor operation is active. Cancel it, send Disable, and close?",
                parent=self.root,
            ):
                self.closing = True
                self.cancel_event.set()
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