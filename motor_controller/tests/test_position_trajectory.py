"""Wall-clock position motion and unit invariance regressions."""

from __future__ import annotations

import math
import time
import tkinter as tk
import unittest
from unittest.mock import patch

from multi_motor.controller import MultiMotorController
from multi_motor.gui import MultiMotorApp
from multi_motor.trajectory import PositionTrajectory
from test_multi_motor import SharedFakeBus, wait_for


class PositionTrajectoryTests(unittest.TestCase):
    def test_ninety_degrees_at_one_rad_s_has_exact_expected_reference_duration(self):
        trajectory = PositionTrajectory(0, math.radians(90), 1)
        self.assertAlmostEqual(trajectory.duration_s, math.pi / 2)
        # Uneven frame intervals must not change angular speed or distance.
        for elapsed in (0, 0.017, 0.12, 0.57, 1.0, 1.4):
            position, velocity, finished = trajectory.sample(elapsed)
            self.assertAlmostEqual(position, elapsed)
            self.assertEqual(velocity, 1)
            self.assertFalse(finished)
        self.assertEqual(trajectory.sample(math.pi / 2), (math.pi / 2, 0, True))
        self.assertEqual(trajectory.sample(4), (math.pi / 2, 0, True))

    def test_negative_input_uses_angle_direction_and_speed_magnitude(self):
        positive = PositionTrajectory(0, math.pi / 2, -1)
        negative = PositionTrajectory(0, -math.pi / 2, 1)
        self.assertEqual(positive.sample(1), (1, 1, False))
        self.assertEqual(negative.sample(1), (-1, -1, False))
        self.assertAlmostEqual(positive.duration_s, negative.duration_s)

    def test_start_offset_and_fast_short_move_stop_exactly_at_destination(self):
        trajectory = PositionTrajectory(2.08, 2.08 + math.pi / 2, 30)
        self.assertAlmostEqual(trajectory.duration_s, math.pi / 60)
        p, v, finished = trajectory.sample(0.01)
        self.assertAlmostEqual(p, 2.38)
        self.assertEqual(v, 30)
        self.assertFalse(finished)
        self.assertEqual(trajectory.sample(0.1), (trajectory.target_rad, 0, True))

    def test_same_position_has_zero_duration_without_rotation(self):
        trajectory = PositionTrajectory(2.08, 2.08, -1)
        self.assertEqual(trajectory.sample(0), (2.08, 0, True))


class PositionControllerTests(unittest.TestCase):
    def setUp(self):
        self.bus = SharedFakeBus()
        self.controller = MultiMotorController(self.bus, [(1, 0x11)], move_timeout=2)
        self.addCleanup(self.controller.close)
        self.session = self.controller.sessions[1]
        self.session.submit("verify").result(2)
        self.session.submit("enable").result(2)

    def test_first_enable_sets_software_origin_and_ninety_degree_move_reaches_motor_target(self):
        origin = self.session.zero_offset
        self.assertAlmostEqual(origin, 0.4, delta=0.001)
        started = time.monotonic()
        self.session.submit("target", math.pi / 2, 1, 2, 1).result(2)
        self.assertAlmostEqual(self.session.trajectory.duration_s, math.pi / 2, delta=0.001)
        wait_for(lambda: self.session.motion.kind == "hold", timeout=3)
        self.assertGreaterEqual(time.monotonic() - started, math.pi / 2 - 0.02)
        self.assertAlmostEqual(self.bus.positions[1] - origin, math.pi / 2, delta=0.005)
        self.assertEqual(self.session.command_velocity, 0)

    def test_position_allows_signed_30_and_zero_requests_stop(self):
        for speed in (-30, 30):
            self.session.submit("target", math.pi / 2, speed, 2, 1).result(2)
            self.assertEqual(self.session.motion.speed, 30)
            self.assertEqual(self.session.trajectory.velocity_rad_s, 30)
        # Keep this range/stop API check short even if a 30 rad/s sample was sent.
        self.session.acceleration = 3000
        self.session.submit("target", math.pi / 2, 0, 2, 1).result(2)
        wait_for(lambda: self.session.motion.kind == "manual_stop")
        self.assertTrue(self.session.motor.enabled)

    def test_reenable_preserves_user_saved_software_zero(self):
        self.bus.positions[1] = 0.6
        wait_for(lambda: self.session.motor.last_feedback.position_rad > 0.59)
        origin = self.session.submit("zero").result(2)
        self.session.submit("disable").result(2)
        self.bus.positions[1] = 0.8
        self.session.submit("enable").result(2)
        self.assertEqual(self.session.zero_offset, origin)

    def test_slow_reference_does_not_timeout_before_calculated_duration(self):
        self.session.submit("target", math.pi / 2, 0.01, 2, 1).result(2)
        self.assertGreater(self.session.move_deadline - self.session.move_started, 157)


class PositionUnitsTests(unittest.TestCase):
    def setUp(self):
        self.root = tk.Tk()
        self.root.withdraw()
        self.addCleanup(self.root.destroy)
        with patch("multi_motor.gui.list_ports.comports", return_value=[]):
            self.app = MultiMotorApp(self.root)
        self.controller = MultiMotorController(SharedFakeBus(), [(1, 0x11)])
        self.addCleanup(self.controller.close)
        self.session = self.controller.sessions[1]
        self.panel = self.app.panels[0]
        self.panel.session = self.session
        self.panel.settings = self.session.submit("verify").result(2)
        feedback = self.session.submit("enable").result(2)
        self.panel.event("zero", self.session.zero_offset)
        self.panel.event("feedback", feedback)

    def test_switching_deg_rad_and_rad_s_rpm_preserves_saved_physical_request(self):
        self.panel.angle.set("90")
        self.panel.speed.set("1")
        self.panel.save_target()
        original = self.panel.saved_target
        self.panel.angle_unit.set("rad")
        self.panel.speed_unit.set("rpm")
        self.assertAlmostEqual(float(self.panel.angle.get()), math.pi / 2)
        self.assertAlmostEqual(float(self.panel.speed.get()), 60 / (2 * math.pi))
        self.assertTrue(self.panel.target_matches_saved())
        self.assertEqual(self.panel.saved_target, original)
        self.assertIn("T=1.571 s", self.panel.trajectory_info.get())
        self.panel.send_target()
        wait_for(lambda: self.session.motion.kind == "target")
        self.assertAlmostEqual(self.session.trajectory.duration_s, math.pi / 2, delta=0.001)
        self.panel.angle_unit.set("deg")
        self.panel.speed_unit.set("rad/s")
        self.assertAlmostEqual(float(self.panel.angle.get()), 90)
        self.assertAlmostEqual(float(self.panel.speed.get()), 1)

    def test_signed_rpm_input_and_rad_input_produce_same_trajectory(self):
        self.panel.angle_unit.set("rad")
        self.panel.angle.set(str(-math.pi / 2))
        self.panel.speed_unit.set("rpm")
        self.panel.speed.set(str(-60 / (2 * math.pi)))
        angle, speed, *_ = self.panel.values()
        trajectory = PositionTrajectory(self.session.zero_offset, self.session.zero_offset + angle, speed)
        self.assertAlmostEqual(trajectory.duration_s, math.pi / 2)
        self.assertAlmostEqual(trajectory.velocity_rad_s, -1)


if __name__ == "__main__":
    unittest.main()
