from __future__ import annotations

import math
import time
import unittest

from multi_motor.controller import MultiMotorController
from multi_motor.position_tracker import PositionTracker
from multi_motor.velocity_loop import VelocityLoop
from test_multi_motor import SharedFakeBus, wait_for


class VelocityLoopTests(unittest.TestCase):
    def test_constant_drag_is_corrected_without_altering_measured_feedback(self):
        for reference, drag in ((10, 0.366), (15, 0.15), (-10, 0.366)):
            loop = VelocityLoop()
            measured = 0.0
            for index in range(600):
                command = loop.command(reference, measured, index * 0.01, 30)
                measured = math.copysign(max(0.0, abs(command) - drag), command)
            self.assertAlmostEqual(measured, reference, delta=0.005)
            self.assertLessEqual(abs(loop.trim), 2)
            self.assertEqual(loop.command(0, measured, 7, 30), 0)

    def test_direction_change_and_saturated_command_do_not_wind_up(self):
        loop = VelocityLoop()
        for index in range(100):
            self.assertLessEqual(loop.command(30, 29, index * 0.1, 30), 30)
        self.assertEqual(loop.trim, 0)
        self.assertEqual(loop.command(-15, 15, 11, 30), -15)


class PositionRegressionTests(unittest.TestCase):
    def test_relative_ninety_degrees_after_many_turns_for_each_motor_never_enables_position_kp(self):
        bus = SharedFakeBus()
        bus.positions = {1: 50.0, 2: -80.0}
        controller = MultiMotorController(bus, [(1, 0x11), (2, 0x12)])
        self.addCleanup(controller.close)
        origins = {}
        for ident, session in controller.sessions.items():
            session.submit("verify").result(2)
            session.submit("enable").result(2)
            origins[ident] = session.position_rad
            # Old Jog origin remains far away, as in the reported failure.
            session.zero_offset = 0.4
            session.submit("target", math.pi / 2, 1, 2, 1, True).result(2)
        wait_for(lambda: all(s.motion.kind == "hold" for s in controller.sessions.values()), timeout=5)
        for ident, session in controller.sessions.items():
            self.assertAlmostEqual(session.position_rad - origins[ident], math.pi / 2, delta=0.005)
            self.assertTrue(session.motor.enabled)
            before = bus.positions[ident]
            time.sleep(0.08)
            self.assertAlmostEqual(bus.positions[ident], before, delta=0.001)
            for message in bus.commands(ident):
                data = message.data
                if bytes(data) in (b"\xff" * 7 + b"\xfc", b"\xff" * 7 + b"\xfd"):
                    continue
                self.assertEqual(((data[3] & 15) << 8) | data[4], 0)

    def test_overshoot_latches_stop_instead_of_repeated_direction_reversals(self):
        bus = SharedFakeBus()
        events = []
        controller = MultiMotorController(bus, [(1, 0x11)], notify=lambda *e: events.append(e))
        self.addCleanup(controller.close)
        session = controller.sessions[1]
        session.submit("verify").result(2)
        session.submit("enable").result(2)
        session.submit("target", 0.5, 1, 0, 1, True).result(2)
        bus.positions[1] = session.motion.position + 0.02
        wait_for(lambda: session.motion.kind == "hold")
        mark = len(events)
        time.sleep(0.05)
        references = [e[2][1] for e in events[mark:] if e[1] == "reference"]
        self.assertTrue(references)
        self.assertTrue(all(r == 0 for r in references))

    def test_velocity_prediction_does_not_invent_a_revolution_without_a_position_wrap(self):
        tracker = PositionTracker()
        tracker.seed(0.4, 0.4, 30, 0)
        self.assertAlmostEqual(tracker.update(0.5, 0, 0.5, 12.5), 0.5)
        self.assertIsNone(tracker.detected_period)


if __name__ == "__main__":
    unittest.main()
