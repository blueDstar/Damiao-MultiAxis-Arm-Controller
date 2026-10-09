"""Measured turn counts, immediate persistent Jog and telemetry-only tabs."""

from __future__ import annotations

import math
import time
import tkinter as tk
import unittest
from unittest.mock import patch

from multi_motor.controller import MultiMotorController
from multi_motor.gui import MultiMotorApp
from multi_motor.position_tracker import PositionTracker
from multi_motor.plots import TelemetryHistory
from single_motor.damiao_v12 import MotorFeedback
from test_multi_motor import SharedFakeBus, wait_for


class PositionTrackerTests(unittest.TestCase):
    def test_measured_rotations_accumulate_both_supported_wrap_periods(self):
        for period in (2 * math.pi, 25.0):
            for direction in (-1, 1):
                with self.subTest(period=period, direction=direction):
                    tracker = PositionTracker()
                    tracker.seed(0.4, 0.4, direction * 5, 0)
                    for index in range(1, 1001):
                        actual = 0.4 + direction * 5 * index * 0.01
                        raw = (actual + period / 2) % period - period / 2
                        total = tracker.update(raw, direction * 5, index * 0.01, 12.5)
                        self.assertAlmostEqual(total, actual, places=8)
                    self.assertAlmostEqual(tracker.detected_period, period)

    def test_a_stopped_motor_does_not_accumulate_turns_from_noisy_midpoints(self):
        tracker = PositionTracker()
        tracker.seed(0, 0, 0, 0)
        for index in range(1, 101):
            tracker.update(0.0002 if index % 2 else -0.0002, 0.007, index * 0.05, 12.5)
        self.assertLess(abs(tracker.position_rad), 0.001)


class MultiturnTests(unittest.TestCase):
    def setUp(self):
        self.bus = SharedFakeBus()
        self.controller = MultiMotorController(self.bus, [(1, 0x11)], move_timeout=10)
        self.addCleanup(self.controller.close)
        self.session = self.controller.sessions[1]
        self.session.submit("verify").result(2)
        self.session.submit("enable").result(2)

    def test_360_then_720_targets_reach_measured_one_and_two_turns(self):
        origin = self.session.zero_offset
        for angle in (360, 720):
            self.session.submit("target", math.radians(angle), 10, 2, 1).result(2)
            wait_for(lambda: self.session.motion.kind == "hold", timeout=5)
            self.assertTrue(self.session.motor.enabled)
            self.assertAlmostEqual(self.session.position_rad - origin, math.radians(angle), delta=0.005)
            self.assertAlmostEqual(self.bus.positions[1] - origin, math.radians(angle), delta=0.005)
        for message in self.bus.commands(1):
            if bytes(message.data) in (b"\xff" * 7 + b"\xfc", b"\xff" * 7 + b"\xfd"):
                continue
            raw_position = int.from_bytes(message.data[:2], "big")
            self.assertLessEqual(raw_position, 65535)

    def test_single_turn_feedback_wraps_still_reach_720_degrees(self):
        self.bus.feedback_period[1] = 2 * math.pi
        origin = self.session.zero_offset
        self.session.submit("target", 4 * math.pi, 10, 2, 1).result(2)
        wait_for(lambda: self.session.motion.kind == "hold", timeout=5)
        self.assertAlmostEqual(self.session.position_rad - origin, 4 * math.pi, delta=0.005)
        self.assertAlmostEqual(self.session.position_tracker.detected_period, 2 * math.pi)

    def test_high_speed_command_survives_large_feedback_lag_without_tracking_disable(self):
        # A load responds at 25% of the commanded velocity: its measured distance
        # must still drive completion rather than an elapsed-time reference.
        original_send = self.bus.send

        def slow_motor(message):
            if message.arbitration_id == 1 and bytes(message.data) not in (b"\xff" * 7 + b"\xfc", b"\xff" * 7 + b"\xfd"):
                d = bytearray(message.data)
                raw = (d[2] << 4) | (d[3] >> 4)
                requested = (raw * 60 / 4095 - 30) * 0.25
                raw = round((requested + 30) * 4095 / 60)
                d[2] = raw >> 4
                d[3] = ((raw & 15) << 4) | (d[3] & 15)
                import can
                message = can.Message(arbitration_id=1, is_extended_id=False, data=d)
            original_send(message)

        with patch.object(self.bus, "send", side_effect=slow_motor):
            self.session.submit("target", math.pi / 2, 5, 2, 1).result(2)
            wait_for(lambda: self.session.motion.kind == "hold", timeout=5)
        self.assertTrue(self.session.motor.enabled)
        self.assertAlmostEqual(self.session.position_rad - self.session.zero_offset, math.pi / 2, delta=0.005)

    def test_jog_sets_requested_speed_immediately_and_runs_until_stop(self):
        self.session.submit("mode", "manual").result(2)
        wait_for(lambda: self.session.pending_mode is None)
        start = len(self.bus.commands(1))
        self.session.submit("jog", 5, 2, 1).result(2)
        wait_for(lambda: len(self.bus.commands(1)) > start)
        data = self.bus.commands(1)[start].data
        velocity = ((data[2] << 4) | (data[3] >> 4)) * 60 / 4095 - 30
        self.assertAlmostEqual(velocity, 5, delta=60 / 4095)
        wait_for(lambda: len(self.bus.commands(1)) >= start + 10)
        self.assertEqual(self.session.motion.kind, "jog")
        self.session.submit("hold").result(2)
        wait_for(lambda: self.session.motion.kind == "manual_stop")
        self.assertTrue(self.session.motor.enabled)


class PlotAndViewTests(unittest.TestCase):
    def test_history_is_bounded_and_clear_affects_only_telemetry(self):
        history = TelemetryHistory(capacity=3)
        for index in range(5):
            history.add(1, MotorFeedback(1, index, 2, 3, 25, 30, received_at=index))
        self.assertEqual(len(history.samples[1]), 3)
        self.assertEqual([p[1] for p in history.window(1, 4, 1)], [3, 4])
        history.clear()
        self.assertFalse(history.samples)

    def test_tab_changes_mouse_release_freeze_and_clear_preserve_controls_and_running_jog(self):
        root = tk.Tk()
        root.withdraw()
        self.addCleanup(root.destroy)
        with patch("multi_motor.gui.list_ports.comports", return_value=[]):
            app = MultiMotorApp(root)
        controller = MultiMotorController(SharedFakeBus(), [(1, 0x11)], notify=lambda *event: app.events.put(event))
        self.addCleanup(controller.close)
        session, panel = controller.sessions[1], app.panels[0]
        app.controller = controller
        panel.session = session
        panel.settings = session.submit("verify").result(2)
        session.submit("enable").result(2)
        session.submit("mode", "manual").result(2)
        wait_for(lambda: session.pending_mode is None)
        app.poll()
        panel.jog_speed.set("5")
        panel.angle.set("720")
        controls = (panel.angle.get(), panel.jog_speed.get(), panel.control_mode, id(panel.frame))
        panel.buttons["jog_signed"].invoke()
        wait_for(lambda: session.motion.kind == "jog")
        app.poll()
        start = len(controller.router.bus.commands(1))
        app.notebook.select(app.graph_tab)
        app.release_jogs()
        app.focus_lost(None)
        root.update()
        app.graphs.toggle_freeze()
        app.graphs.clear()
        wait_for(lambda: len(controller.router.bus.commands(1)) > start + 3)
        app.poll()
        self.assertEqual(session.motion.kind, "jog")
        self.assertTrue(app.graphs.history.samples.get(1))
        app.notebook.select(app.control_tab)
        root.update()
        self.assertEqual((panel.angle.get(), panel.jog_speed.get(), panel.control_mode, id(panel.frame)), controls)
        self.assertTrue(panel.jogging)
        panel.buttons["jog_stop"].invoke()
        wait_for(lambda: session.motion.kind == "manual_stop")


if __name__ == "__main__":
    unittest.main()
