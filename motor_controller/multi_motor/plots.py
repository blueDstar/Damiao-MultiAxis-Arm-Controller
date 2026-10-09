"""Telemetry-only Tk charts; no CAN commands or control-state mutations."""

from __future__ import annotations

import math
import time
import tkinter as tk
import threading
from collections import deque
from tkinter import ttk

from multi_motor.theme import BACKGROUND, BORDER, INPUT, MUTED, NEON, WHITE


class TelemetryHistory:
    def __init__(self, capacity: int = 10000) -> None:
        self.capacity = capacity
        self.samples = {}
        self.zero_offsets = {}
        self.references = {}

    def add(self, motor_id, feedback) -> None:
        series = self.samples.setdefault(motor_id, deque(maxlen=self.capacity))
        goal, reference, wire = self.references.get(motor_id, (None, None, None))
        sample = (feedback.received_at, feedback.position_rad, feedback.velocity_rad_s, feedback.torque_nm,
                  reference, wire, goal, feedback.status_code, feedback.raw_payload.hex(' ').upper(),
                  time.time() + feedback.received_at - time.monotonic(), feedback.raw_position_rad)
        if series and sample[0] == series[-1][0]:
            series[-1] = sample
        elif not series or sample[0] > series[-1][0]:
            series.append(sample)

    def window(self, motor_id, now, seconds):
        return [s for s in self.samples.get(motor_id, ()) if now - seconds <= s[0] <= now]

    def clear(self) -> None:
        self.samples.clear()


class TraceChart(tk.Canvas):
    def __init__(self, parent, label, color):
        super().__init__(parent, background=INPUT, highlightthickness=1,
                         highlightbackground=BORDER, height=115)
        self.label, self.color = label, color

    def draw(self, samples, column, now, seconds, offset=0.0):
        self.delete("all")
        width, height = self.winfo_width(), self.winfo_height()
        if width < 120 or height < 60:
            return
        left, top, right, bottom = 70, 26 if height < 105 else 30, width - 16, height - (24 if height < 105 else 30)
        self.create_text(12, 13, text=self.label, fill=WHITE, anchor="w", font=("Segoe UI", 10, "bold"))
        values = [s[column] - offset for s in samples if math.isfinite(s[column])]
        reference_values = [s[4] for s in samples if len(s) > 4 and s[4] is not None] if column == 2 else []
        if not values:
            self.create_text(width / 2, height / 2, text="Chờ feedback…", fill=MUTED)
            return
        low, high = min(values + reference_values), max(values + reference_values)
        margin = max((high - low) * 0.12, 0.01)
        low, high = low - margin, high + margin
        ticks = 3 if height < 130 else 5
        for step in range(ticks):
            y = top + (bottom - top) * step / (ticks - 1)
            value = high - (high - low) * step / (ticks - 1)
            self.create_line(left, y, right, y, fill="#20342b")
            self.create_text(left - 7, y, text=f"{value:.3g}", fill=MUTED, anchor="e", font=("Consolas", 9))
        for step in range(5):
            x = left + (right - left) * step / 4
            self.create_line(x, top, x, bottom, fill="#20342b")
            self.create_text(x, bottom + 14, text=f"−{seconds * (1 - step / 4):g}s" if step < 4 else "0s", fill=MUTED, font=("Consolas", 9))
        self.create_text(right, 13, text=f"{values[-1]:.4f}", fill=self.color, anchor="e", font=("Consolas", 11, "bold"))
        if reference_values:
            self.create_text(right - 110, 13, text=f"Đặt {reference_values[-1]:.3f}", fill="#ffd078", anchor="e", font=("Consolas", 10))
        # Bound rendering work independently of sample/command frequency.
        stride = max(1, (len(samples) + 499) // 500)
        chosen = samples[::stride]
        if chosen[-1] is not samples[-1]:
            chosen.append(samples[-1])
        points = []
        previous_time = None
        for sample in chosen:
            x = left + (sample[0] - (now - seconds)) / seconds * (right - left)
            y = bottom - (sample[column] - offset - low) / (high - low) * (bottom - top)
            if previous_time is not None and sample[0] - previous_time > 0.6:
                if len(points) >= 4:
                    self.create_line(*points, fill=self.color, width=2)
                points = []
            points.extend((x, y))
            previous_time = sample[0]
        if len(points) >= 4:
            self.create_line(*points, fill=self.color, width=2)
        elif points:
            x, y = points
            self.create_oval(x - 2, y - 2, x + 2, y + 2, fill=self.color, outline=self.color)
        if reference_values:
            ref_points = []
            for sample in chosen:
                if sample[4] is None:
                    continue
                x = left + (sample[0] - (now - seconds)) / seconds * (right - left)
                y = bottom - (sample[4] - low) / (high - low) * (bottom - top)
                ref_points.extend((x, y))
            if len(ref_points) >= 4:
                self.create_line(*ref_points, fill="#ffd078", width=1, dash=(5, 3))


class MotorPlots(ttk.Frame):
    def __init__(self, parent, is_visible):
        super().__init__(parent, style="Page.TFrame")
        self.is_visible = is_visible
        self.history = TelemetryHistory()
        self.cards = {}
        self.window_seconds = tk.StringVar(value="30")
        self.frozen = False
        self.export_motor = tk.StringVar(value="0x01")
        self.export_status = tk.StringVar(value="Xuất toàn bộ dữ liệu đang lưu của motor được chọn")
        self.exporting = False
        toolbar = ttk.Frame(self, padding=10)
        toolbar.pack(fill="x")
        ttk.Label(toolbar, text="FEEDBACK · POS / VEL / TOR", style="Section.TLabel").pack(side="left", padx=(0, 20))
        ttk.Label(toolbar, text="Cửa sổ (s)").pack(side="left", padx=(0, 6))
        ttk.Combobox(toolbar, textvariable=self.window_seconds, values=("10", "30", "60", "120"), state="readonly", width=5).pack(side="left", padx=(0, 12))
        self.freeze_button = ttk.Button(toolbar, text="TẠM DỪNG HIỂN THỊ", command=self.toggle_freeze)
        self.freeze_button.pack(side="left", padx=(0, 8))
        ttk.Button(toolbar, text="XÓA ĐỒ THỊ", command=self.clear).pack(side="left")
        export_bar = ttk.Frame(self, padding=(10, 4))
        export_bar.pack(fill="x")
        ttk.Label(export_bar, text="LƯU MOTOR").pack(side="left", padx=(0, 6))
        self.export_selector = ttk.Combobox(export_bar, textvariable=self.export_motor, values=("0x01", "0x02"), state="readonly", width=7)
        self.export_selector.pack(side="left", padx=(0, 10))
        for label, formats in (("LƯU ẢNH PNG", ("png",)), ("LƯU EXCEL", ("xlsx",)), ("LƯU CẢ HAI", ("png", "xlsx"))):
            ttk.Button(export_bar, text=label, command=lambda f=formats: self.export(f)).pack(side="left", padx=(0, 6))
        ttk.Label(self, textvariable=self.export_status, style="PageMuted.TLabel", padding=(12, 3), wraplength=1120).pack(fill="x")
        ttk.Label(self, text="Pos theo zero hiện tại · Vel xanh: đo thật, nét vàng: tốc độ đặt · Chuyển tab không dừng motor", style="PageMuted.TLabel", padding=(12, 5)).pack(anchor="w")
        container = ttk.Frame(self, style="Page.TFrame")
        container.pack(fill="both", expand=True)
        self.canvas = tk.Canvas(container, background=BACKGROUND, highlightthickness=0)
        scroll = ttk.Scrollbar(container, orient="vertical", command=self.canvas.yview)
        scroll.pack(side="right", fill="y")
        self.canvas.pack(side="left", fill="both", expand=True)
        self.canvas.configure(yscrollcommand=scroll.set)
        self.area = ttk.Frame(self.canvas, style="Page.TFrame")
        window = self.canvas.create_window((0, 0), window=self.area, anchor="nw")
        self.area.columnconfigure(0, weight=1, uniform="plots")
        self.area.columnconfigure(1, weight=1, uniform="plots")
        self.area.bind("<Configure>", lambda _: self.canvas.configure(scrollregion=self.canvas.bbox("all")))
        self.canvas.bind("<Configure>", lambda e: self.resize(e, window))
        self._timer = self.after(200, self.refresh)

    def resize(self, event, window):
        self.canvas.itemconfigure(window, width=event.width)
        height = max(90, min(180, (event.height - 100) // 3))
        for _, charts in self.cards.values():
            for chart in charts:
                chart.configure(height=height)

    def configure_motors(self, ids) -> None:
        values = [f"0x{i:02X}" for i in ids]
        self.export_selector.configure(values=values)
        if self.export_motor.get() not in values and values:
            self.export_motor.set(values[0])
        if tuple(self.cards) == tuple(ids):
            return
        for card, _ in self.cards.values():
            card.destroy()
        self.cards.clear()
        for index, motor_id in enumerate(ids):
            card = ttk.LabelFrame(self.area, text=f"MOTOR CAN 0x{motor_id:02X}", padding=10, style="Motor.TLabelframe")
            card.grid(row=index // 2, column=index % 2, sticky="new", padx=8, pady=8)
            charts = []
            for label, color in (("POS / rad", NEON), ("VEL / rad/s", "#61dbff"), ("TOR / Nm", "#ffd078")):
                chart = TraceChart(card, label, color)
                chart.pack(fill="x", pady=5)
                charts.append(chart)
            self.cards[motor_id] = card, charts

    def export(self, formats):
        if self.exporting:
            return
        motor_id = int(self.export_motor.get(), 0)
        samples = list(self.history.samples.get(motor_id, ()))
        if not samples:
            self.export_status.set("Motor này chưa có feedback để lưu")
            return
        from multi_motor.export_feedback import export_feedback
        self.exporting = True
        self.export_status.set("Đang lưu ảnh/Excel; các motor vẫn tiếp tục chạy")
        zero = self.history.zero_offsets.get(motor_id, 0.0)

        def save():
            try:
                result = export_feedback(motor_id, samples, zero, formats=formats)
                message = "Đã lưu: " + ", ".join(str(p) for p in result)
            except Exception as exc:
                message = f"Lưu thất bại: {exc}"
            self._export_result = message
        self._export_result = None
        threading.Thread(target=save, name="feedback-export", daemon=True).start()

    def toggle_freeze(self):
        self.frozen = not self.frozen
        self.freeze_button.configure(text="TIẾP TỤC HIỂN THỊ" if self.frozen else "TẠM DỪNG HIỂN THỊ")

    def clear(self):
        self.history.clear()
        self.redraw()

    def redraw(self):
        if self.frozen:
            return
        now, seconds = time.monotonic(), float(self.window_seconds.get())
        for motor_id, (_, charts) in self.cards.items():
            samples = self.history.window(motor_id, now, seconds)
            for column, chart in enumerate(charts, 1):
                chart.draw(samples, column, now, seconds, self.history.zero_offsets.get(motor_id, 0.0) if column == 1 else 0.0)

    def refresh(self):
        if self.exporting and self._export_result is not None:
            self.export_status.set(self._export_result)
            self.exporting = False
        if self.is_visible():
            self.redraw()
        self._timer = self.after(200, self.refresh)

    def destroy(self):
        if self._timer:
            self.after_cancel(self._timer)
            self._timer = None
        super().destroy()
