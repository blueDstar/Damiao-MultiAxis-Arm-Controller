"""Interleaved CAN feedback and independently running axis regression tests."""

from __future__ import annotations

import math
import queue
import struct
import threading
import time
import tkinter as tk
import unittest
from unittest.mock import patch

import can

from multi_motor.can_router import CANRouter
from multi_motor.controller import MultiMotorController
from multi_motor.gui import MultiMotorApp
from multi_motor.gui import display_number
from single_motor.damiao_v12 import INTEGER_REGISTERS, MotorFeedback
from multi_motor.theme import BACKGROUND, NEON, WHITE


def wait_for(predicate, timeout=2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.005)
    raise AssertionError("Expected state did not arrive before timeout.")


class SharedFakeBus:
    def __init__(self):
        self.rx = queue.Queue()
        self.sent = []
        self.positions = {1: 0.4, 2: -0.8}
        self.velocities = {1: 0.0, 2: 0.0}
        self.online = {1, 2}
        self.faults = {1: 0, 2: 0}
        self.enabled = {1: False, 2: False}
        self.shutdown_called = False
        self.fail_rx = False
        self.lock = threading.Lock()
        self.last_command_time = {}

    def feedback(self, can_id):
        raw = round((self.positions[can_id] + 12.5) * 65535 / 25.0)
        velocity_raw = round((self.velocities[can_id] + 30) * 4095 / 60)
        status = self.faults[can_id] or int(self.enabled[can_id])
        payload = bytes((can_id | status << 4,)) + raw.to_bytes(2, "big", signed=False) + bytes((velocity_raw >> 4, ((velocity_raw & 15) << 4) | 8, 0, 25, 30))
        return can.Message(arbitration_id=can_id + 0x10, is_extended_id=False, data=payload)

    def send(self, message):
        with self.lock:
            self.sent.append(message)
            if message.arbitration_id == 0x7FF:
                can_id, _, _, register = message.data[:4]
                if can_id not in self.online:
                    return
                if message.data[2] == 0xCC:
                    self.rx.put(self.feedback(can_id))
                    return
                values = {7: can_id + 0x10, 8: can_id, 10: 1, 14: 5017, 0x24: 5,
                          0x15: 12.5, 0x16: 30.0, 0x17: 10.0}
                values[0x51] = self.positions[can_id]
                value = struct.pack("<I" if register in INTEGER_REGISTERS else "<f", values.get(register, 1))
                self.rx.put(can.Message(arbitration_id=can_id + 0x10, is_extended_id=False, data=bytes(message.data[:4]) + value))
            else:
                can_id = message.arbitration_id & 0xFF
                if can_id not in self.online:
                    return
                if message.data == b"\xff" * 7 + b"\xfc":
                    self.enabled[can_id] = True
                elif message.data == b"\xff" * 7 + b"\xfd":
                    self.enabled[can_id] = False
                    self.velocities[can_id] = 0.0
                if message.data != b"\xff" * 7 + b"\xfc" and message.data != b"\xff" * 7 + b"\xfd":
                    now = time.monotonic()
                    delta = min(0.1, now - self.last_command_time.get(can_id, now))
                    self.last_command_time[can_id] = now
                    raw = int.from_bytes(message.data[:2], "big")
                    kp = ((message.data[3] & 0x0F) << 8) | message.data[4]
                    kd = (message.data[5] << 4) | (message.data[6] >> 4)
                    velocity_raw = (message.data[2] << 4) | (message.data[3] >> 4)
                    velocity = velocity_raw * 60 / 4095 - 30
                    if self.enabled[can_id] and (kp or kd):
                        self.velocities[can_id] = velocity if abs(velocity) > 60 / 4095 else 0.0
                    if self.enabled[can_id] and kp:
                        self.positions[can_id] = raw * 25 / 65535 - 12.5
                    elif self.enabled[can_id] and kd:
                        self.positions[can_id] += self.velocities[can_id] * delta
                self.rx.put(self.feedback(can_id))

    def recv(self, timeout=None):
        if self.fail_rx:
            raise OSError("adapter unplugged")
        try:
            return self.rx.get(timeout=timeout)
        except queue.Empty:
            return None

    def shutdown(self):
        self.shutdown_called = True

    def commands(self, can_id):
        with self.lock:
            return [m for m in self.sent if m.arbitration_id == can_id]


class RouterTests(unittest.TestCase):
    def setUp(self):
        self.bus = SharedFakeBus()
        self.router = CANRouter(self.bus)
        self.addCleanup(self.router.close)
        self.motor1 = self.router.add_motor(1, 0x11)
        self.motor2 = self.router.add_motor(2, 0x12)

    def test_interleaved_feedback_is_delivered_to_its_motor(self):
        self.motor1.verify_configuration()
        self.motor2.verify_configuration()
        self.bus.rx.put(self.bus.feedback(2))
        self.bus.rx.put(self.bus.feedback(1))
        self.assertAlmostEqual(self.motor1._read_feedback(0.2).position_rad, 0.4, places=3)
        self.assertAlmostEqual(self.motor2._read_feedback(0.2).position_rad, -0.8, places=3)

    def test_literal_midpoint_frames_decode_near_zero_without_sign_jump(self):
        self.motor1.verify_configuration()
        values = []
        for payload in ("11 7F FF 7F F7 FF 19 1E", "11 80 00 80 08 00 19 1E"):
            self.bus.rx.put(can.Message(arbitration_id=0x11, is_extended_id=False, data=bytes.fromhex(payload)))
            feedback = self.motor1._read_feedback(0.2)
            values.append(feedback)
            self.assertLess(abs(feedback.position_rad), 0.001)
            self.assertLess(abs(feedback.velocity_rad_s), 0.01)
            self.assertLess(abs(feedback.torque_nm), 0.003)
            self.assertEqual(display_number(feedback.velocity_rad_s, 60 / 4095), "0.000")
        self.assertLess(abs(values[0].position_rad - values[1].position_rad), 0.001)

    def test_literal_range_endpoints_follow_unsigned_linear_mapping(self):
        self.motor1.verify_configuration()
        for payload, expected in (("11 00 00 00 00 00 19 1E", (-12.5, -30, -10)), ("11 FF FF FF FF FF 19 1E", (12.5, 30, 10))):
            self.bus.rx.put(can.Message(arbitration_id=0x11, is_extended_id=False, data=bytes.fromhex(payload)))
            feedback = self.motor1._read_feedback(0.2)
            self.assertEqual((feedback.position_rad, feedback.velocity_rad_s, feedback.torque_nm), expected)

    def test_register_reply_is_never_consumed_as_status(self):
        self.motor1.verify_configuration()
        response = bytes((1, 0, 0x33, 0x15)) + struct.pack("<f", 12.5)
        self.bus.rx.put(can.Message(arbitration_id=0x11, is_extended_id=False, data=response))
        self.bus.rx.put(self.bus.feedback(1))
        self.assertAlmostEqual(self.motor1._read_feedback(0.2).position_rad, 0.4, places=3)

    def test_fault_is_not_overwritten_by_new_status_or_command(self):
        self.motor1.verify_configuration()
        self.bus.faults[1] = 8
        channel = self.motor1.bus
        channel.deliver(self.bus.feedback(1))
        self.bus.faults[1] = 0
        channel.deliver(self.bus.feedback(1))
        self.motor1._send(1, self.motor1._pack_mit_command(0.4, 0.0, 2, 1))
        self.assertEqual(self.motor1._read_feedback(0.2).error_code, 8)

    def test_duplicate_ids_and_command_feedback_collisions_rejected(self):
        for can_id, master_id in ((1, 0x13), (3, 0x12), (3, 1), (3, 0x103), (0, 0x13), (3, 0x7FF)):
            with self.subTest(can_id=can_id, master_id=master_id), self.assertRaises(ValueError):
                self.router.add_motor(can_id, master_id)

    def test_concurrent_configuration_reads_use_separate_inboxes(self):
        results = []
        threads = [threading.Thread(target=lambda m=m: results.append(m.verify_configuration())) for m in (self.motor1, self.motor2)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join(2)
        self.assertEqual({s.can_id for s in results}, {1, 2})


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.bus = SharedFakeBus()
        self.events = queue.Queue()
        self.controller = MultiMotorController(self.bus, [(1, 0x11), (2, 0x12)],
                                               notify=lambda *event: self.events.put(event),
                                               feedback_timeout=0.15, move_timeout=2)
        self.addCleanup(self.controller.close)
        self.one = self.controller.sessions[1]
        self.two = self.controller.sessions[2]
        for session in (self.one, self.two):
            session.submit("verify").result(2)

    def enable_both(self):
        futures = [s.submit("enable") for s in (self.one, self.two)]
        for future in futures:
            future.result(2)

    def switch_manual(self, session):
        session.submit("mode", "manual").result(2)
        wait_for(lambda: session.panel_mode == "manual" and session.pending_mode is None)

    def test_verification_is_read_only_and_motion_requires_enable(self):
        self.assertFalse(self.bus.commands(1))
        self.assertFalse(self.bus.commands(2))
        with self.assertRaises(RuntimeError):
            self.one.submit("target", 0.5, 0.2, 2.0, 1.0).result(2)
        self.assertFalse(self.bus.commands(1))

    def test_enabled_status_one_keeps_both_motor_streams_running(self):
        self.enable_both()
        count1, count2 = len(self.bus.commands(1)), len(self.bus.commands(2))
        wait_for(lambda: len(self.bus.commands(1)) >= count1 + 8 and len(self.bus.commands(2)) >= count2 + 8)
        for session in (self.one, self.two):
            self.assertTrue(session.motor.enabled)
            self.assertTrue(session.motor.last_feedback.is_enabled)
            self.assertFalse(session.motor.last_feedback.has_fault)
            self.assertEqual(session.motion.kind, "idle")
            self.assertFalse(any(bytes(m.data) == b"\xff" * 7 + b"\xfd" for m in self.bus.commands(session.motor.can_id)))

    def test_enable_neutralizes_driver_without_position_or_velocity_torque(self):
        self.one.submit("enable").result(2)
        wait_for(lambda: len(self.bus.commands(1)) >= 6)
        commands = self.bus.commands(1)
        self.assertNotEqual(bytes(commands[0].data), b"\xff" * 7 + b"\xfc")
        for message in commands:
            data = message.data
            if bytes(data) == b"\xff" * 7 + b"\xfc":
                continue
            self.assertEqual(((data[3] & 15) << 8) | data[4], 0)
            self.assertEqual((data[5] << 4) | (data[6] >> 4), 0)
            self.assertIn((data[2] << 4) | (data[3] >> 4), (2047, 2048))
        self.assertAlmostEqual(self.bus.positions[1], 0.4, places=4)

    def test_control_modes_block_wrong_commands_and_manual_switch_sets_local_zero(self):
        self.enable_both()
        with self.assertRaisesRegex(RuntimeError, "Manual Jog"):
            self.one.submit("jog", 0.2, 2, 1).result(2)
        self.switch_manual(self.one)
        self.assertAlmostEqual(self.one.zero_offset, 0.4, places=3)
        self.assertAlmostEqual(self.bus.positions[1], 0.4, places=4)
        self.assertEqual(self.one.motion.kind, "idle")
        with self.assertRaisesRegex(RuntimeError, "Manual Jog"):
            self.one.submit("target", 0.2, 0.2, 2, 1).result(2)
        self.assertEqual(self.two.panel_mode, "position")

    def test_wrong_position_decoding_blocks_enable_before_motion(self):
        self.one.motor.position_range_rad = 12.5
        original = self.one.motor.read_register

        def wrong_reference(register, *args, **kwargs):
            return 8.0 if register == 0x51 else original(register, *args, **kwargs)

        with patch.object(self.one.motor, "read_register", side_effect=wrong_reference):
            with self.assertRaisesRegex(RuntimeError, "decoding mismatch"):
                self.one.submit("verify").result(2)
        self.assertIsNone(self.one.settings)
        with self.assertRaises(RuntimeError):
            self.one.submit("enable").result(2)
        self.assertFalse(self.bus.commands(1))

    def test_jog_stream_keeps_zero_kp_and_requested_direction_after_release(self):
        self.enable_both()
        self.switch_manual(self.one)
        start = len(self.bus.commands(1))
        self.one.submit("jog", -0.2, 2, 1).result(2)
        wait_for(lambda: len(self.bus.commands(1)) >= start + 5)
        self.one.submit("hold").result(2)
        wait_for(lambda: self.one.motion.kind == "manual_stop")
        velocities = []
        for message in self.bus.commands(1)[start:]:
            d = message.data
            self.assertEqual(((d[3] & 15) << 8) | d[4], 0)
            raw = (d[2] << 4) | (d[3] >> 4)
            velocities.append(raw * 60 / 4095 - 30)
        self.assertTrue(all(-0.22 <= v <= 0.01 for v in velocities))
        self.assertLess(abs(velocities[-1]), 0.01)
        self.assertLess(self.bus.positions[1], self.one.zero_offset)
        self.assertEqual(self.bus.velocities[1], 0.0)

    def test_jog_accepts_full_signed_range_without_using_position_cap(self):
        self.one.speed_cap = 0.2
        self.one.submit("enable").result(2)
        self.switch_manual(self.one)
        for speed in (-30.0, -5.0, 5.0, 30.0):
            self.one.submit("jog", speed, 2, 1).result(2)
            self.assertEqual(self.one.motion.kind, "jog")
            self.assertEqual(self.one.motion.speed, speed)
        for speed, raw_expected in ((-30.0, 0), (30.0, 4095)):
            _, payload = self.one.motor.encode_position_command(0, speed, 0, 1)
            self.assertEqual((payload[2] << 4) | (payload[3] >> 4), raw_expected)
        self.one.submit("jog", 0.0, 2, 1).result(2)
        wait_for(lambda: self.one.motion.kind == "manual_stop")
        self.assertTrue(self.one.motor.enabled)

    def test_jog_keeps_driver_vmax_and_position_limit_separate(self):
        for speed in (-31, 31, math.inf, math.nan):
            with self.subTest(speed=speed), self.assertRaises(ValueError):
                self.one._validate_motion(0.4, speed, 2, 1, jog=True)
        self.one.motor.velocity_range_rad_s = 8.0
        for speed in (-8, 8):
            self.one._validate_motion(0.4, speed, 2, 1, jog=True)
        for speed in (-8.1, 8.1):
            with self.subTest(speed=speed), self.assertRaises(ValueError):
                self.one._validate_motion(0.4, speed, 2, 1, jog=True)
        self.assertEqual(self.one._validate_motion(0.4, 1, 2, 1), 1)

    def test_mit_jog_does_not_stop_at_feedback_position_range_boundary(self):
        self.one.submit("enable").result(2)
        self.switch_manual(self.one)
        self.bus.positions[1] = 12.49
        wait_for(lambda: self.one.motor.last_feedback.position_rad > 12.48)
        start = len(self.bus.commands(1))
        self.one.submit("jog", 0.2, 2, 1).result(2)
        wait_for(lambda: len(self.bus.commands(1)) >= start + 3)
        self.assertEqual(self.one.motion.kind, "jog")
        self.assertGreater(self.one.command_velocity, 0)

    def test_disabled_feedback_stops_axis_without_treating_it_as_enabled(self):
        self.enable_both()
        self.bus.enabled[1] = False
        wait_for(lambda: self.one.motion is None)
        self.assertFalse(self.one.motor.enabled)
        self.assertTrue(self.two.motor.enabled)

    def test_independent_targets_and_zero_offsets(self):
        self.enable_both()
        zero = self.one.submit("zero").result(2)
        self.assertAlmostEqual(zero, 0.4, places=3)
        self.assertAlmostEqual(self.two.zero_offset, -0.8, places=3)
        self.one.submit("target", 0.1, 0.5, 2, 1).result(2)
        self.two.submit("target", -0.9, 0.5, 2, 1).result(2)
        wait_for(lambda: self.one.motion.kind == "hold" and self.two.motion.kind == "hold")
        self.assertAlmostEqual(self.bus.positions[1], zero + 0.1, delta=0.05)
        self.assertAlmostEqual(self.bus.positions[2], self.two.zero_offset - 0.9, delta=0.005)

    def test_disabling_one_does_not_interrupt_other_motor(self):
        self.enable_both()
        self.switch_manual(self.two)
        self.two.submit("jog", -0.2, 2, 1).result(2)
        self.one.submit("disable").result(2)
        count1, count2 = len(self.bus.commands(1)), len(self.bus.commands(2))
        wait_for(lambda: len(self.bus.commands(2)) >= count2 + 3)
        self.assertEqual(len(self.bus.commands(1)), count1)
        self.assertFalse(self.one.motor.enabled)
        self.assertTrue(self.two.motor.enabled)
        self.assertEqual(self.two.motion.kind, "jog")

    def test_missing_motor_does_not_block_other_axis(self):
        self.bus.online.remove(2)
        missing = self.two.submit("verify")
        self.one.submit("enable").result(0.5)
        self.one.submit("target", 0.5, 0.5, 2, 1).result(0.5)
        with self.assertRaises(TimeoutError):
            missing.result(2)
        self.assertTrue(self.one.motor.enabled)
        self.assertFalse(self.bus.commands(2))

    def test_invalid_target_does_not_replace_other_commands(self):
        self.enable_both()
        original = self.one.motion
        for target in ((13, 0.2, 2, 1), (0, 31, 2, 1), (0, 0.2, 0, 1), (math.nan, 0.2, 2, 1)):
            with self.subTest(target=target), self.assertRaises(ValueError):
                self.one.submit("target", *target).result(2)
        self.assertIs(self.one.motion, original)
        self.assertTrue(self.two.motor.enabled)

    def test_jog_release_brakes_without_position_recoil_or_disabling(self):
        self.enable_both()
        self.switch_manual(self.one)
        self.one.submit("jog", -0.2, 2, 1).result(2)
        self.one.submit("hold").result(2)
        wait_for(lambda: self.one.motion.kind == "manual_stop")
        self.assertTrue(self.one.motor.enabled)
        self.assertTrue(self.two.motor.enabled)

    def test_fault_stops_only_affected_axis(self):
        self.enable_both()
        self.bus.faults[1] = 8
        wait_for(lambda: self.one.motion is None)
        self.assertFalse(self.one.motor.enabled)
        self.assertEqual(bytes(self.bus.commands(1)[-1].data), b"\xff" * 7 + b"\xfd")
        self.assertTrue(self.two.motor.enabled)

    def test_missing_feedback_disables_only_affected_axis(self):
        self.enable_both()
        self.bus.online.remove(1)
        wait_for(lambda: self.one.motion is None)
        self.assertFalse(self.one.motor.enabled)
        self.assertTrue(self.two.motor.enabled)
        self.assertEqual(bytes(self.bus.commands(1)[-1].data), b"\xff" * 7 + b"\xfd")

    def test_parameters_require_disabled_axis_other_motor_keeps_running(self):
        self.enable_both()
        with self.assertRaises(RuntimeError):
            self.one.submit("parameters").result(2)
        self.one.submit("disable").result(2)
        parameters = self.one.submit("parameters").result(2)
        self.assertGreater(len(parameters), 30)
        self.assertTrue(self.two.motor.enabled)

    def test_disable_all_and_close_stop_workers_before_transport(self):
        self.enable_both()
        for future in self.controller.disable_all():
            future.result(2)
        self.controller.close()
        self.assertTrue(self.bus.shutdown_called)
        self.assertFalse(self.one._thread.is_alive())
        self.assertFalse(self.two._thread.is_alive())
        self.assertFalse(self.controller.router._reader.is_alive())

    def test_receiver_failure_reaches_both_axes(self):
        self.enable_both()
        self.bus.fail_rx = True
        wait_for(lambda: self.one.motion is None and self.two.motion is None)
        self.assertFalse(self.one.motor.enabled)
        self.assertFalse(self.two.motor.enabled)
        with self.assertRaisesRegex(RuntimeError, "adapter unplugged"):
            self.controller.close()

    def test_disable_cancels_pending_enable_without_restarting_stream(self):
        entered, release = threading.Event(), threading.Event()
        enable = self.one.motor.enable

        def delayed_enable():
            entered.set()
            release.wait(2)
            return enable()

        with patch.object(self.one.motor, "enable", side_effect=delayed_enable):
            pending = self.one.submit("enable")
            self.assertTrue(entered.wait(1))
            stop = self.one.submit("disable")
            release.set()
            with self.assertRaises(RuntimeError):
                pending.result(2)
            stop.result(2)
        self.assertFalse(self.one.motor.enabled)
        self.assertIsNone(self.one.motion)
        self.assertEqual(bytes(self.bus.commands(1)[-1].data), b"\xff" * 7 + b"\xfd")


class GUITests(unittest.TestCase):
    def setUp(self):
        self.root = tk.Tk()
        self.root.withdraw()
        self.addCleanup(self.root.destroy)
        with patch("multi_motor.gui.list_ports.comports", return_value=[]):
            self.app = MultiMotorApp(self.root)

    def test_default_motor_ids_and_separate_controls(self):
        one, two = self.app.panels
        self.assertEqual((one.id_var.get(), one.master_var.get()), ("0x01", "0x11"))
        self.assertEqual((two.id_var.get(), two.master_var.get()), ("0x02", "0x12"))
        one.angle.set("90")
        self.assertEqual(two.angle.get(), "0")
        self.assertEqual(str(one.buttons["target"]["state"]), "disabled")

    def test_black_neon_theme_and_side_by_side_motor_cards(self):
        self.assertEqual(self.root.cget("background"), BACKGROUND)
        self.assertEqual(self.app.log_widget.cget("foreground"), WHITE)
        self.assertEqual(self.app.style.lookup("TButton", "bordercolor"), NEON)
        self.assertEqual(self.app.style.lookup("TEntry", "foreground"), WHITE)
        self.assertEqual(self.app.panels[0].frame.grid_info()["column"], 0)
        self.assertEqual(self.app.panels[1].frame.grid_info()["column"], 1)

    def test_status_one_shows_enabled_badge_and_enables_target_controls(self):
        panel = self.app.panels[0]
        from types import SimpleNamespace
        panel.session = SimpleNamespace(motor=SimpleNamespace(enabled=True))
        panel.settings = SimpleNamespace(control_mode=1)
        panel.event("feedback", MotorFeedback(1, 0.4, 0, 0, 25, 30))
        self.assertTrue(panel.enabled)
        self.assertEqual(panel.badge.get(), "● ENABLED")
        self.assertEqual(str(panel.buttons["jog_plus"]["state"]), "disabled")
        self.assertEqual(str(panel.buttons["enable"]["state"]), "disabled")

    def test_stale_feedback_never_displays_old_motion_as_current(self):
        from types import SimpleNamespace
        panel = self.app.panels[0]
        panel.session = SimpleNamespace(motor=SimpleNamespace(enabled=True))
        panel.settings = SimpleNamespace(control_mode=1)
        panel.event("feedback", MotorFeedback(1, 0.4, 1, 2, 25, 30, received_at=time.monotonic() - 1))
        panel.expire_feedback()
        self.assertEqual(panel.metric_velocity.get(), "—")
        self.assertEqual(panel.metric_torque.get(), "—")
        self.assertEqual(str(panel.buttons["mode"]["state"]), "disabled")

    def test_jog_fields_are_independent_of_invalid_position_target(self):
        from types import SimpleNamespace
        panel = self.app.panels[0]
        commands = []
        panel.session = SimpleNamespace(motor=SimpleNamespace(enabled=True), speed_cap=0.5)
        panel.settings = SimpleNamespace(velocity_range_rad_s=30)
        panel.enabled = True
        panel.control_mode = "manual"
        panel.feedback = MotorFeedback(1, 0.4, 0, 0, 25, 30)
        panel.angle.set("invalid position input")
        panel.dispatch = lambda *args: commands.append(args)
        panel.start_jog(-1)
        self.assertEqual(commands, [("jog", -0.2, 2.0, 1.0)])

    def test_signed_jog_input_reaches_30_rad_s_with_direction_and_zero_controls(self):
        from types import SimpleNamespace
        panel = self.app.panels[0]
        commands = []
        panel.session = SimpleNamespace(motor=SimpleNamespace(enabled=True), speed_cap=0.2)
        panel.settings = SimpleNamespace(velocity_range_rad_s=30)
        panel.enabled = True
        panel.control_mode = "manual"
        panel.feedback = MotorFeedback(1, 0.4, 0, 0, 25, 30)
        panel.dispatch = lambda *args: commands.append(args)
        panel.jog_speed.set("-30")
        panel.start_jog(None)
        panel.start_jog(1)
        panel.start_jog(-1)
        panel.jog_speed.set("30")
        panel.start_jog(None)
        panel.jog_speed.set("0")
        panel.start_jog(None)
        self.assertEqual(commands, [("jog", -30, 2, 1), ("jog", 30, 2, 1), ("jog", -30, 2, 1), ("jog", 30, 2, 1), ("hold",)])

    def test_signed_jog_rpm_conversion_obeys_actual_vmax(self):
        from types import SimpleNamespace
        panel = self.app.panels[0]
        commands, errors = [], []
        panel.session = SimpleNamespace(motor=SimpleNamespace(enabled=True), speed_cap=0.2)
        panel.settings = SimpleNamespace(velocity_range_rad_s=30)
        panel.enabled = True
        panel.control_mode = "manual"
        panel.feedback = MotorFeedback(1, 0.4, 0, 0, 25, 30)
        panel.dispatch = lambda *args: commands.append(args)
        self.app.show_error = errors.append
        panel.jog_unit.set("rpm")
        panel.jog_speed.set("-180")
        panel.start_jog(None)
        self.assertAlmostEqual(commands[-1][1], -6 * math.pi)
        panel.settings.velocity_range_rad_s = 8
        panel.start_jog(None)
        self.assertEqual(len(commands), 1)
        self.assertEqual(len(errors), 1)

    def test_gui_routes_each_panel_command_and_release_independently(self):
        bus = SharedFakeBus()
        controller = MultiMotorController(bus, [(1, 0x11), (2, 0x12)], notify=lambda *event: self.app.events.put(event))
        self.addCleanup(controller.close)
        self.app.events.put((None, "opened", controller))
        self.app.poll()
        wait_for(lambda: all(s.settings is not None for s in controller.sessions.values()))
        self.app.poll()
        one, two = self.app.panels
        one.dispatch("enable")
        two.dispatch("enable")
        wait_for(lambda: all(s.motor.enabled for s in controller.sessions.values()))
        wait_for(lambda: one.session.motion is not None and two.session.motion is not None)
        self.app.poll()
        one.angle.set("5")
        one.save_target()
        one.send_target()
        wait_for(lambda: one.session.motion.kind == "target")
        self.assertEqual(two.session.motion.kind, "idle")
        self.app.poll()
        two.switch_mode()
        wait_for(lambda: two.session.panel_mode == "manual" and two.session.pending_mode is None)
        self.app.poll()
        two.start_jog(-1)
        wait_for(lambda: two.session.motion.kind == "jog")
        self.app.release_jogs()
        wait_for(lambda: two.session.motion.kind == "manual_stop")
        one.dispatch("disable")
        wait_for(lambda: not one.session.motor.enabled)
        self.assertTrue(two.session.motor.enabled)


if __name__ == "__main__":
    unittest.main()
