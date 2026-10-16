"""Arm geometry, feedback mapping and the GUI's shared-controller workflow."""

import math
import os
import time
import tkinter as tk
import unittest
from dataclasses import replace
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch

from multi_motor.controller import MultiMotorController
from robot_arm_2dof.demo_bus import DemoBus
from robot_arm_2dof.gui import RobotArmApp, SOURCE_CAN, SOURCE_DEMO
from robot_arm_2dof.kinematics import ArmGeometry, JointMapping, parse_target, validate_ids
from robot_arm_2dof.scene import Camera, arm_mesh


def wait_for(predicate, timeout=4):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.005)
    raise AssertionError("Expected state did not arrive")


class KinematicsTests(unittest.TestCase):
    def test_straight_and_folded_arm_endpoints(self):
        arm = ArmGeometry(0.3, 0.2, 0.17)
        self.assertEqual(arm.points(0, 0)[2], (0.5, 0, 0.17))
        base, elbow, tip = arm.points(math.pi / 2, -math.pi / 2)
        self.assertAlmostEqual(elbow[0], 0)
        self.assertAlmostEqual(elbow[2], 0.47)
        self.assertAlmostEqual(tip[0], 0.2)
        self.assertAlmostEqual(tip[2], 0.47)
        tip = arm.points(0, math.pi)[2]
        self.assertAlmostEqual(tip[0], 0.1)

    def test_link_lengths_are_invariant_through_multiple_turns(self):
        arm = ArmGeometry()
        for shoulder, elbow in ((0.1, -0.3), (math.tau * 3, -math.tau * 5), (2.2, 1.1)):
            a, b, c = arm.points(shoulder, elbow)
            self.assertAlmostEqual(math.dist(a, b), arm.upper_length)
            self.assertAlmostEqual(math.dist(b, c), arm.forearm_length)
            self.assertEqual(c[1], 0)

    def test_feedback_zero_sign_and_mounting_are_independent(self):
        mapping = JointMapping(math.radians(55), -1)
        self.assertAlmostEqual(mapping.angle(12.0, 12.0), math.radians(55))
        self.assertAlmostEqual(mapping.angle(12 + math.pi / 2, 12), math.radians(-35))
        self.assertAlmostEqual(mapping.angle(12 + math.tau * 2, 12), math.radians(55) - math.tau * 2)

    def test_units_signed_speed_and_unbounded_rotation(self):
        angle, speed = parse_target("720", "deg", "-60", "rpm")
        self.assertAlmostEqual(angle, 4 * math.pi)
        self.assertAlmostEqual(speed, -math.tau)
        self.assertEqual(parse_target("0", "rad", "30", "rad/s"), (0, 30))
        for target in (("nan", "deg", "1", "rad/s"), ("90", "deg", "31", "rad/s"),
                       ("90", "deg", "inf", "rpm")):
            with self.assertRaises(ValueError):
                parse_target(*target)
        with self.assertRaises(ValueError):
            parse_target("90", "deg", "9", "rad/s", vmax=8)

    def test_invalid_dimensions_and_aliasing_ids_are_rejected(self):
        for values in ((0, 0.2, 0.1), (0.3, math.nan, 0.1), (0.3, 0.2, -1)):
            with self.assertRaises(ValueError):
                ArmGeometry(*values)
        for ids in (((1, 17), (1, 18)), ((1, 17), (2, 17)), ((1, 2), (2, 18)), ((16, 17), (2, 18))):
            with self.assertRaises(ValueError):
                validate_ids(ids)

    def test_camera_is_independent_and_mesh_vertices_are_finite(self):
        arm = ArmGeometry()
        before = arm.points(1, -0.5)
        camera = Camera()
        camera.orbit(200, 50)
        camera.zoom_by(10)
        self.assertEqual(arm.points(1, -0.5), before)
        self.assertEqual(camera.zoom, 2.2)
        for face in arm_mesh(arm, 1, -0.5):
            self.assertTrue(all(math.isfinite(v) for point in face.vertices for v in point))


class DemoControllerTests(unittest.TestCase):
    def setUp(self):
        self.bus = DemoBus(((3, 0x21), (5, 0x22)))
        self.controller = MultiMotorController(self.bus, [(3, 0x21), (5, 0x22)])
        self.addCleanup(self.controller.close)
        for session in self.controller.sessions.values():
            session.submit("verify").result(3)

    def test_custom_ids_verify_and_enable_remains_stationary(self):
        one, two = self.controller.sessions.values()
        self.assertEqual(one.settings.master_id, 0x21)
        self.assertEqual(two.settings.master_id, 0x22)
        one.submit("enable").result(2)
        time.sleep(0.1)
        self.assertTrue(one.motor.enabled)
        self.assertFalse(two.motor.enabled)
        self.assertAlmostEqual(self.bus.axes[3].position, 0, places=3)

    def test_both_axes_finish_relative_goals_and_one_stop_is_independent(self):
        one, two = self.controller.sessions.values()
        for session in (one, two):
            session.submit("enable").result(2)
        one.submit("target", math.pi / 2, 2, 0, 1, True).result(2)
        two.submit("target", -math.pi / 4, 1, 0, 1, True).result(2)
        wait_for(lambda: one.motion.kind == "hold" and two.motion.kind == "hold")
        self.assertAlmostEqual(self.bus.axes[3].position, math.pi / 2, delta=0.02)
        self.assertAlmostEqual(self.bus.axes[5].position, -math.pi / 4, delta=0.02)
        one.submit("disable").result(2)
        self.assertTrue(two.motor.enabled)
        second_start = self.bus.axes[5].position
        two.submit("target", -0.2, 1, 0, 1, True).result(2)
        wait_for(lambda: two.motion.kind == "hold")
        self.assertAlmostEqual(self.bus.axes[5].position, second_start - 0.2, delta=0.02)

    def test_shutdown_disables_simulated_axes(self):
        for session in self.controller.sessions.values():
            session.submit("enable").result(2)
        self.controller.close()
        self.assertTrue(self.bus.shutdown_called)
        self.assertTrue(all(not axis.enabled for axis in self.bus.axes.values()))

    def test_multi_turn_target_crosses_wire_wrap_and_stops(self):
        one = self.controller.sessions[3]
        one.submit("enable").result(2)
        one.submit("target", math.tau * 2, 15, 0, 1, True).result(2)
        wait_for(lambda: one.motion.kind == "hold")
        self.assertTrue(one.motor.enabled)
        self.assertGreater(one.position_rad, 12.5)
        self.assertAlmostEqual(self.bus.axes[3].position, math.tau * 2, delta=0.025)
        self.assertLess(abs(self.bus.axes[3].velocity), 0.04)


class ArmGUITests(unittest.TestCase):
    def setUp(self):
        self.root = tk.Tk()
        self.root.withdraw()
        self.directory = TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.config_patch = patch("robot_arm_2dof.gui.CONFIG_PATH", Path(self.directory.name) / "config.json")
        self.config_patch.start()
        self.addCleanup(self.config_patch.stop)
        self.addCleanup(self.root.destroy)
        with patch("robot_arm_2dof.gui.list_ports.comports", return_value=[]):
            self.app = RobotArmApp(self.root)
        self.app.show_error = lambda error: self.errors.append(error)
        self.errors = []
        self.controller = MultiMotorController(DemoBus(), [(1, 17), (2, 18)],
                                                notify=lambda *event: self.app.events.put(event))
        self.addCleanup(self.controller.close)
        self.app.active_source = SOURCE_DEMO
        self.app.events.put((None, "opened", self.controller))
        self.app.poll()
        wait_for(lambda: all(s.settings is not None for s in self.controller.sessions.values()))
        self.app.poll()

    def enable_both(self):
        for panel in self.app.panels:
            panel.dispatch("enable")
        wait_for(lambda: all(s.motor.enabled and s.motion is not None for s in self.controller.sessions.values()))
        self.app.poll()

    def test_control_defaults_and_camera_do_not_enable_motors(self):
        self.assertEqual(self.app.source.get(), SOURCE_DEMO)
        self.assertEqual(self.app.panels[0].target_kind.get(), "Quay thêm từ hiện tại")
        self.app.scene.camera.orbit(100, 30)
        self.app.scene.zoom(1.2)
        self.app.scene.home()
        self.app.poll()
        self.assertTrue(all(not s.motor.enabled and s.motion is None for s in self.controller.sessions.values()))

    def test_relative_command_routes_only_to_selected_can_axis(self):
        self.enable_both()
        one, two = self.app.panels
        one.angle.set("90")
        one.send_target()
        wait_for(lambda: one.session.motion.kind == "target")
        self.assertEqual(two.session.motion.kind, "idle")
        self.assertAlmostEqual(one.session.motion.position - one.session.trajectory.start_rad, math.pi / 2)

    def test_pair_validation_prevents_partial_send(self):
        self.enable_both()
        self.app.panels[1].angle.set("bad angle")
        self.app.send_pair()
        self.assertEqual(len(self.errors), 1)
        self.assertTrue(all(s.motion.kind == "idle" for s in self.controller.sessions.values()))

    def test_unit_change_preserves_real_target_speed(self):
        self.enable_both()
        panel = self.app.panels[0]
        panel.angle.set("90")
        before = panel.target_args()
        panel.angle_unit.set("rad")
        panel.convert_unit("angle")
        panel.speed_unit.set("rpm")
        panel.convert_unit("speed")
        after = panel.target_args()
        self.assertAlmostEqual(before[0], after[0])
        self.assertAlmostEqual(before[1], after[1])

    def test_scene_uses_measured_feedback_instead_of_entered_target(self):
        self.enable_both()
        panel = self.app.panels[0]
        panel.angle.set("720")
        # A literal measured increment is enough to prove target text is ignored.
        panel.feedback = replace(panel.feedback, position_rad=panel.zero + 0.4)
        self.app._refresh_scene()
        self.assertAlmostEqual(self.app.view_angles[0], math.radians(55) + 0.4)
        self.assertAlmostEqual(self.app.view_angles[1], math.radians(-80), delta=0.001)
        panel.feedback = replace(panel.feedback, position_rad=panel.zero + 1, received_at=time.monotonic() - 2)
        self.app._refresh_scene()
        self.assertTrue(self.app.scene.stale[0])
        self.assertAlmostEqual(self.app.view_angles[0], math.radians(55) + 0.4)

    def test_last_measured_pose_is_retained_after_disconnect(self):
        self.enable_both()
        panel = self.app.panels[0]
        panel.feedback = replace(panel.feedback, position_rad=panel.zero + 0.4)
        self.app._refresh_scene()
        before = self.app.view_angles[:]
        self.app.controller = None
        for panel in self.app.panels:
            panel.reset()
        self.app._refresh_scene()
        self.assertEqual(self.app.view_angles, before)
        self.assertTrue(all(self.app.scene.stale))

    def test_software_zero_does_not_move_the_visual_joint(self):
        self.enable_both()
        panel = self.app.panels[0]
        measured = panel.zero + 0.4
        panel.feedback = replace(panel.feedback, position_rad=measured)
        self.app._refresh_scene()
        before = self.app.view_angles[:]
        panel.event("zero", measured)
        self.app._refresh_scene()
        self.assertEqual(self.app.view_angles, before)
        self.assertEqual(panel.position_text.get(), "+0.00")

    def test_simulation_open_never_constructs_physical_serial_bus(self):
        self.controller.close()
        self.app.controller = None
        for panel in self.app.panels:
            panel.reset()
        with patch("robot_arm_2dof.gui.DamiaoUSB2CANBus") as physical_bus:
            self.app.connect()
            def opened():
                self.app.poll()
                return self.app.controller is not None
            wait_for(opened)
            wait_for(lambda: all(s.settings is not None for s in self.app.controller.sessions.values()))
            self.app.poll()
            self.assertFalse(physical_bus.called)
            self.assertTrue(all(not s.motor.enabled for s in self.app.controller.sessions.values()))
            self.addCleanup(self.app.controller.close)

    def test_invalid_second_id_does_not_open_hardware(self):
        self.app.controller = None
        for panel in self.app.panels:
            panel.reset()
        self.app.source.set(SOURCE_CAN)
        self.app.panels[1].id_var.set("0x01")
        with patch("robot_arm_2dof.gui.DamiaoUSB2CANBus") as physical_bus:
            self.app.connect()
            self.assertFalse(physical_bus.called)
            self.assertFalse(self.app.connecting)
            self.assertEqual(len(self.errors), 1)

    def test_real_connection_uses_shared_usb2can_arguments_and_no_auto_enable(self):
        self.controller.close()
        self.app.controller = None
        for panel in self.app.panels:
            panel.reset()
        self.app.source.set(SOURCE_CAN)
        self.app.channel.set("COM_TEST")
        self.app.bitrate.set("1000000")
        self.app.baud.set("921600")
        bus = DemoBus()
        with patch("robot_arm_2dof.gui.validate_usb2can_port"), \
             patch("robot_arm_2dof.gui.DamiaoUSB2CANBus", return_value=bus) as physical_bus, \
             patch.dict(os.environ, {"DAMIAO_MULTI_COMMAND_RATE_HZ": "100"}):
            self.app.connect()
            def opened():
                self.app.poll()
                return self.app.controller is not None
            wait_for(opened)
            self.addCleanup(self.app.controller.close)
            wait_for(lambda: all(s.settings is not None for s in self.app.controller.sessions.values()))
            self.app.poll()
            physical_bus.assert_called_once_with("COM_TEST", baudrate=921600, can_bitrate_code=0)
            self.assertTrue(all(not s.motor.enabled for s in self.app.controller.sessions.values()))
            self.assertIn("CAN THẬT", self.app.scene.source)


if __name__ == "__main__":
    unittest.main()
