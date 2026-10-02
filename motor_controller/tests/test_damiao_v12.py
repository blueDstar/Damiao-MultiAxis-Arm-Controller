import struct
import threading
import unittest
from collections import deque
from types import SimpleNamespace
from unittest.mock import patch

import can

from single_motor.damiao_v12 import (
    CAN_REGISTER_REQUEST_ID,
    REGISTER_CAN_ID,
    REGISTER_CONTROL_MODE,
    REGISTER_FIRMWARE_VERSION,
    REGISTER_MASTER_ID,
    REGISTER_POSITION_RANGE,
    REGISTER_SUB_VERSION,
    REGISTER_TORQUE_RANGE,
    REGISTER_VELOCITY_RANGE,
    INTEGER_REGISTERS,
    DamiaoV12,
    MoveCancelledError,
)
from single_motor.move_to_angle import validate_slcan_port


class FakeBus:
    def __init__(self, control_mode: int = 2) -> None:
        self.sent: list[can.Message] = []
        self.received: deque[can.Message] = deque()
        self.register_values = {
            REGISTER_CAN_ID: struct.pack("<I", 0x01),
            REGISTER_MASTER_ID: struct.pack("<I", 0x11),
            REGISTER_CONTROL_MODE: struct.pack("<I", control_mode),
            REGISTER_POSITION_RANGE: struct.pack("<f", 12.5),
            REGISTER_VELOCITY_RANGE: struct.pack("<f", 30.0),
            REGISTER_TORQUE_RANGE: struct.pack("<f", 10.0),
            REGISTER_FIRMWARE_VERSION: struct.pack("<I", 5017),
            REGISTER_SUB_VERSION: struct.pack("<I", 5),
        }

    def send(self, message: can.Message) -> None:
        self.sent.append(message)
        if message.arbitration_id == CAN_REGISTER_REQUEST_ID:
            register = message.data[3]
            value = self.register_values.get(register)
            if value is None:
                value = (
                    struct.pack("<I", 1)
                    if register in INTEGER_REGISTERS
                    else struct.pack("<f", 1.0)
                )
            response = bytes(message.data[:4]) + value
            self.received.append(
                can.Message(arbitration_id=0x11, is_extended_id=False, data=response)
            )
            return

        if message.arbitration_id in (0x01, 0x101):
            if message.data[-1] == 0xFC:
                self.received.append(self.feedback(0.0))
            elif message.arbitration_id == 0x01:
                self.received.append(self.feedback(0.0))
            else:
                position_rad, _ = struct.unpack("<ff", message.data)
                self.received.append(self.feedback(position_rad))

    def recv(self, timeout: float | None = None) -> can.Message | None:
        if self.received:
            return self.received.popleft()
        return None

    @staticmethod
    def feedback(position_rad: float) -> can.Message:
        position_raw = round(position_rad * 32768 / 12.5)
        position_bytes = position_raw.to_bytes(2, byteorder="big", signed=True)
        payload = bytes((0x11,)) + position_bytes + bytes((0, 0, 0, 25, 30))
        return can.Message(arbitration_id=0x11, is_extended_id=False, data=payload)


class DamiaoV12Tests(unittest.TestCase):
    def test_reads_driver_configuration(self) -> None:
        bus = FakeBus()
        motor = DamiaoV12(bus, can_id=0x01, master_id=0x11)

        settings = motor.verify_configuration()

        self.assertEqual(settings.control_mode, 2)
        self.assertEqual(settings.position_range_rad, 12.5)
        self.assertEqual(settings.velocity_range_rad_s, 30.0)
        self.assertEqual(settings.firmware_version, 5017)
        self.assertEqual(settings.sub_version, 5)
        self.assertEqual(len(bus.sent), 8)
        self.assertTrue(all(frame.arbitration_id == CAN_REGISTER_REQUEST_ID for frame in bus.sent))

    def test_reads_present_parameter_registers(self) -> None:
        bus = FakeBus()
        motor = DamiaoV12(bus, can_id=0x01, master_id=0x11)

        parameters = motor.read_present_parameters()

        by_address = {parameter.address: parameter for parameter in parameters}
        self.assertEqual(by_address[REGISTER_CAN_ID].value, 1)
        self.assertEqual(by_address[REGISTER_FIRMWARE_VERSION].value, 5017)
        self.assertEqual(by_address[0x10].value, 1)
        self.assertEqual(by_address[0x51].name, "Output-shaft position")
        self.assertEqual(len(bus.sent), len(parameters))

    def test_rejects_wrong_mode_before_enable(self) -> None:
        bus = FakeBus(control_mode=3)
        motor = DamiaoV12(bus, can_id=0x01, master_id=0x11)

        with self.assertRaisesRegex(RuntimeError, "supports MIT"):
            motor.verify_configuration()

        self.assertTrue(all(frame.arbitration_id == CAN_REGISTER_REQUEST_ID for frame in bus.sent))

    def test_sends_little_endian_position_and_speed_commands(self) -> None:
        bus = FakeBus()
        motor = DamiaoV12(bus, can_id=0x01, master_id=0x11)
        motor.verify_configuration()

        result = motor.move_to_position(
            position_rad=1.0,
            max_velocity_rad_s=0.2,
            position_tolerance_rad=0.05,
            velocity_tolerance_rad_s=0.1,
            timeout_s=1.0,
            command_rate_hz=20.0,
        )

        move_frames = [frame for frame in bus.sent if frame.arbitration_id == 0x101]
        self.assertGreaterEqual(len(move_frames), 4)
        self.assertEqual(move_frames[0].data[-1], 0xFC)
        self.assertEqual(move_frames[1].data, struct.pack("<ff", 1.0, 0.2))
        self.assertAlmostEqual(result.position_rad, 1.0, delta=0.001)
        self.assertEqual(result.torque_nm, 0.0)
        self.assertEqual(result.driver_temperature_c, 25)
        self.assertEqual(result.motor_temperature_c, 30)

    def test_sends_packed_mit_command_for_mit_mode(self) -> None:
        bus = FakeBus(control_mode=1)
        motor = DamiaoV12(bus, can_id=0x01, master_id=0x11)
        settings = motor.verify_configuration()

        motor.move_to_position(
            position_rad=0.0,
            max_velocity_rad_s=0.2,
            position_tolerance_rad=0.05,
            velocity_tolerance_rad_s=0.1,
            timeout_s=1.0,
            command_rate_hz=20.0,
            kp=2.0,
            kd=1.0,
        )

        mit_frames = [frame for frame in bus.sent if frame.arbitration_id == 0x01]
        self.assertEqual(settings.control_mode, 1)
        self.assertGreaterEqual(len(mit_frames), 4)
        self.assertEqual(mit_frames[0].data, b"\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFC")
        self.assertEqual(len(mit_frames[1].data), 8)
        self.assertEqual(mit_frames[1].data[:2], b"\x7F\xFF")
        self.assertEqual(mit_frames[1].data[-2:], b"\x37\xFF")

    def test_encodes_user_mit_hex_example(self) -> None:
        bus = FakeBus(control_mode=1)
        motor = DamiaoV12(bus, can_id=0x01, master_id=0x11)
        motor.verify_configuration()

        arbitration_id, payload = motor.encode_position_command(
            position_rad=0.0,
            velocity_rad_s=5.0,
            kp=0.0,
            kd=1.0,
        )

        self.assertEqual(arbitration_id, 0x01)
        self.assertEqual(payload.hex(" ").upper(), "7F FF 95 40 00 33 37 FF")

    def test_send_after_explicit_enable_does_not_enable_twice(self) -> None:
        bus = FakeBus(control_mode=1)
        motor = DamiaoV12(bus, can_id=0x01, master_id=0x11)
        motor.verify_configuration()
        motor.enable()

        motor.move_to_position(
            position_rad=0.0,
            max_velocity_rad_s=0.2,
            position_tolerance_rad=0.05,
            velocity_tolerance_rad_s=0.1,
            timeout_s=1.0,
            command_rate_hz=20.0,
            enable_first=False,
        )

        enable_frames = [
            frame
            for frame in bus.sent
            if frame.arbitration_id == 0x01 and frame.data[-1] == 0xFC
        ]
        self.assertEqual(len(enable_frames), 1)

    def test_cancelling_move_sends_disable(self) -> None:
        bus = FakeBus(control_mode=1)
        motor = DamiaoV12(bus, can_id=0x01, master_id=0x11)
        motor.verify_configuration()
        cancel_event = threading.Event()
        cancel_event.set()

        with self.assertRaises(MoveCancelledError):
            motor.move_to_position(
                position_rad=0.0,
                max_velocity_rad_s=0.2,
                position_tolerance_rad=0.05,
                velocity_tolerance_rad_s=0.1,
                timeout_s=1.0,
                command_rate_hz=20.0,
                cancel_event=cancel_event,
            )

        disable_frames = [
            frame for frame in bus.sent if frame.arbitration_id == 0x01 and frame.data[-1] == 0xFD
        ]
        self.assertEqual(len(disable_frames), 1)

    def test_rejects_target_outside_driver_range_before_enable(self) -> None:
        bus = FakeBus()
        motor = DamiaoV12(bus, can_id=0x01, master_id=0x11)
        motor.verify_configuration()

        with self.assertRaisesRegex(ValueError, "PMAX"):
            motor.move_to_position(
                position_rad=13.0,
                max_velocity_rad_s=0.2,
                position_tolerance_rad=0.05,
                velocity_tolerance_rad_s=0.1,
                timeout_s=1.0,
                command_rate_hz=20.0,
            )

        self.assertTrue(all(frame.arbitration_id == CAN_REGISTER_REQUEST_ID for frame in bus.sent))


class SerialPortSelectionTests(unittest.TestCase):
    @patch("single_motor.move_to_angle.list_ports.comports")
    def test_prefers_usb_device_when_bluetooth_has_same_com_number(self, comports) -> None:
        comports.return_value = [
            SimpleNamespace(
                device="COM3", description="Standard Serial over Bluetooth link", vid=None, pid=None
            ),
            SimpleNamespace(
                device="COM3", description="USB Serial Device", vid=0x2E88, pid=0x4603
            ),
        ]

        validate_slcan_port("COM3")

    @patch("single_motor.move_to_angle.list_ports.comports")
    def test_rejects_bluetooth_only_port(self, comports) -> None:
        comports.return_value = [
            SimpleNamespace(
                device="COM3", description="Standard Serial over Bluetooth link", vid=None, pid=None
            )
        ]

        with self.assertRaisesRegex(RuntimeError, "not a USB serial adapter"):
            validate_slcan_port("COM3")


if __name__ == "__main__":
    unittest.main()