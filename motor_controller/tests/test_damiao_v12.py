from __future__ import annotations

import math
import struct
import threading
import unittest
from collections import deque
from queue import Empty, Queue
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
    ConnectionCancelledError,
    DamiaoV12,
    MoveCancelledError,
    MotorFeedback,
)
from single_motor.gui import MotorControlApp, SavedTarget
from single_motor.move_to_angle import validate_slcan_port
from single_motor.usb2can_serial import (
    DamiaoUSB2CANBus,
    USB2CAN_CAN_BAUD_COMMAND,
    USB2CAN_HEARTBEAT_COMMAND,
    USB2CAN_STOP_COMMAND,
    encode_usb2can_frame,
)


class FakeBus:
    def __init__(self, control_mode: int = 2) -> None:
        self.control_mode = control_mode
        self.feedback_error_code = 1
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
            if message.data[-1] in (0xFC, 0xFD):
                self.received.append(self.feedback(0.0, self.feedback_error_code))
            elif message.arbitration_id == 0x01:
                position_raw = int.from_bytes(message.data[:2], byteorder="big")
                position = position_raw * 25.0 / 65535.0 - 12.5
                self.received.append(self.feedback(position, self.feedback_error_code))
            else:
                position_rad, _ = struct.unpack("<ff", message.data)
                self.received.append(self.feedback(position_rad, self.feedback_error_code))
    def recv(self, timeout: float | None = None) -> can.Message | None:
        if self.received:
            return self.received.popleft()
        return None

    @staticmethod
    def feedback(position_rad: float, error_code: int = 1) -> can.Message:
        position_raw = round((position_rad + 12.5) * 65535 / 25.0)
        position_bytes = position_raw.to_bytes(2, byteorder="big", signed=False)
        payload = bytes((error_code << 4 | 0x01,)) + position_bytes + bytes((0x80, 0x08, 0x00, 25, 30))
        return can.Message(arbitration_id=0x11, is_extended_id=False, data=payload)


class CancellingBus(FakeBus):
    def __init__(self, cancel_event: threading.Event) -> None:
        super().__init__()
        self.cancel_event = cancel_event

    def send(self, message: can.Message) -> None:
        self.sent.append(message)
        if message.arbitration_id == CAN_REGISTER_REQUEST_ID:
            self.cancel_event.set()


class FakeSerialPort:
    def __init__(self) -> None:
        self.is_open = True
        self.rts = False
        self.writes: list[bytes] = []
        self.incoming: Queue[bytes] = Queue()
        self.remainder = bytearray()

    @property
    def in_waiting(self) -> int:
        return len(self.remainder)

    def read(self, size: int = 1) -> bytes:
        if not self.remainder:
            try:
                self.remainder.extend(self.incoming.get(timeout=0.01))
            except Empty:
                return b""
        result = bytes(self.remainder[:size])
        del self.remainder[:size]
        return result

    def write(self, data: bytes) -> int:
        self.writes.append(bytes(data))
        return len(data)

    def close(self) -> None:
        self.is_open = False


class USB2CANSerialTests(unittest.TestCase):
    def test_encodes_vendor_frame_for_damiao_hex_payload(self) -> None:
        message = can.Message(
            arbitration_id=0x02,
            is_extended_id=False,
            data=bytes.fromhex("7F FF 95 40 00 33 37 FF"),
        )

        packet = encode_usb2can_frame(message)

        self.assertEqual(len(packet), 30)
        self.assertEqual(packet[:4], bytes.fromhex("55 AA 1E 01"))
        self.assertEqual(packet[4:21], bytes.fromhex("01 00 00 00 0A 00 00 00 00 02 00 00 00 00 08 00 00"))
        self.assertEqual(packet[21:29], bytes.fromhex("7F FF 95 40 00 33 37 FF"))
        from single_motor.usb2can_serial import crc8_dallas_maxim

        self.assertEqual(packet[-1], crc8_dallas_maxim(packet[:-1]))

    def test_parses_receive_frame_and_sends_control_packets(self) -> None:
        port = FakeSerialPort()
        bus = DamiaoUSB2CANBus("COM3", serial_instance=port)
        try:
            bus.send_heartbeat()
            bus.stop_periodic_send()
            bus.set_can_bitrate(4)
            self.assertEqual(port.writes, [
                USB2CAN_HEARTBEAT_COMMAND,
                USB2CAN_STOP_COMMAND,
                bytes.fromhex("55 05 04 AA 55"),
            ])

            port.incoming.put(bytes.fromhex("00 AA 01 08 02 00 00 00 11 22 33 44 55 66 77 88 55"))
            message = bus.recv(timeout=0.5)
            self.assertIsNotNone(message)
            self.assertEqual(message.arbitration_id, 0x02)
            self.assertEqual(message.data, bytes.fromhex("11 22 33 44 55 66 77 88"))
            self.assertFalse(message.is_extended_id)
        finally:
            bus.shutdown()


class DamiaoV12Tests(unittest.TestCase):
    def test_cancelled_register_read_aborts_without_waiting_for_timeout(self) -> None:
        cancel_event = threading.Event()
        bus = CancellingBus(cancel_event)
        motor = DamiaoV12(bus, can_id=0x01, master_id=0x11)

        with self.assertRaises(ConnectionCancelledError):
            motor.read_register(REGISTER_CAN_ID, timeout_s=5.0, cancel_event=cancel_event)

        self.assertEqual(len(bus.sent), 1)

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

    def test_enable_rejects_fault_feedback_code(self) -> None:
        bus = FakeBus(control_mode=1)
        bus.feedback_error_code = 2
        motor = DamiaoV12(bus, can_id=0x01, master_id=0x11)
        motor.verify_configuration()

        with self.assertRaisesRegex(RuntimeError, "fault code 2"):
            motor.enable()

        self.assertFalse(motor.enabled)
        self.assertEqual(motor.last_feedback.error_code, 2)

    def test_enable_requires_enabled_status_one_not_disabled_zero(self) -> None:
        bus = FakeBus(control_mode=1)
        bus.feedback_error_code = 0
        motor = DamiaoV12(bus, can_id=0x01, master_id=0x11)
        motor.verify_configuration()
        with self.assertRaisesRegex(RuntimeError, "Disabled"):
            motor.enable()
        self.assertFalse(motor.enabled)
        self.assertFalse(motor.last_feedback.has_fault)

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
        self.assertAlmostEqual(result.torque_nm, 0.0, delta=10.0 / 4095)
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

    def test_releasing_target_send_holds_position_without_disabling(self) -> None:
        bus = FakeBus(control_mode=1)
        motor = DamiaoV12(bus, can_id=0x01, master_id=0x11)
        motor.verify_configuration()
        motor.enabled = True
        motor.last_feedback = motor._read_feedback(timeout_s=0.0)
        cancel_event = threading.Event()
        cancel_event.set()

        with self.assertRaisesRegex(MoveCancelledError, "remains enabled"):
            motor.move_to_position(
                position_rad=0.5,
                max_velocity_rad_s=0.2,
                position_tolerance_rad=0.05,
                velocity_tolerance_rad_s=0.1,
                timeout_s=1.0,
                command_rate_hz=20.0,
                cancel_event=cancel_event,
                disable_on_cancel=False,
            )

        self.assertTrue(motor.enabled)
        self.assertFalse(
            any(frame.data[-1] == 0xFD for frame in bus.sent if frame.arbitration_id == 0x01)
        )

    def test_mit_target_90_degrees_reaches_pi_over_two(self) -> None:
        bus = FakeBus(control_mode=1)
        motor = DamiaoV12(bus, can_id=0x01, master_id=0x11)
        motor.verify_configuration()
        target = math.pi / 2

        result = motor.move_to_position(
            position_rad=target,
            max_velocity_rad_s=0.2,
            position_tolerance_rad=0.01,
            velocity_tolerance_rad_s=0.01,
            timeout_s=10.0,
            command_rate_hz=20.0,
            kp=2.0,
            kd=1.0,
        )

        self.assertAlmostEqual(result.position_rad, target, delta=0.01)
        command_frames = [
            frame.data
            for frame in bus.sent
            if frame.arbitration_id == 0x01 and frame.data[-1] not in (0xFC, 0xFD)
        ]
        self.assertGreater(len(command_frames), 10)
        first_velocity_raw = (command_frames[0][2] << 4) | (command_frames[0][3] >> 4)
        zero_velocity_raw = motor._float_to_uint(0.0, -30.0, 30.0, 12)
        self.assertGreater(first_velocity_raw, zero_velocity_raw)
        self.assertEqual(
            (command_frames[-1][2] << 4) | (command_frames[-1][3] >> 4),
            zero_velocity_raw,
        )

    def test_mit_position_move_uses_target_direction_and_zeroes_feedforward(self) -> None:
        for target in (0.05, -0.05):
            with self.subTest(target=target):
                bus = FakeBus(control_mode=1)
                motor = DamiaoV12(bus, can_id=0x01, master_id=0x11)
                motor.verify_configuration()

                motor.move_to_position(
                    position_rad=target,
                    max_velocity_rad_s=0.2,
                    position_tolerance_rad=0.005,
                    velocity_tolerance_rad_s=0.01,
                    timeout_s=1.0,
                    command_rate_hz=20.0,
                    kp=2.0,
                    kd=1.0,
                )

                commands = [
                    frame.data
                    for frame in bus.sent
                    if frame.arbitration_id == 0x01 and frame.data[-1] != 0xFC
                ]
                velocity_raw_values = [
                    (payload[2] << 4) | (payload[3] >> 4) for payload in commands
                ]
                zero_velocity_raw = motor._float_to_uint(0.0, -30.0, 30.0, 12)
                self.assertGreaterEqual(len(commands), 3)
                self.assertGreater(velocity_raw_values[0], zero_velocity_raw) if target > 0 else self.assertLess(
                    velocity_raw_values[0], zero_velocity_raw
                )
                self.assertTrue(
                    all(
                        value >= zero_velocity_raw if target > 0 else value <= zero_velocity_raw
                        for value in velocity_raw_values
                    )
                )
                self.assertEqual(velocity_raw_values[-1], zero_velocity_raw)

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

    @patch("single_motor.gui.DamiaoUSB2CANBus")
    @patch("single_motor.gui.can.Bus")
    @patch("single_motor.gui.validate_usb2can_port")
    def test_gui_uses_vendor_usb2can_bus_for_connection(
        self, validate_port, can_bus, usb2can_bus
    ) -> None:
        app = object.__new__(MotorControlApp)
        app.root = SimpleNamespace()
        app.channel_var = SimpleNamespace(get=lambda: "COM3")
        app.can_bitrate_var = SimpleNamespace(get=lambda: "1000000")
        app.serial_baud_var = SimpleNamespace(get=lambda: "921600")
        app.can_id_var = SimpleNamespace(get=lambda: "0x02")
        app.master_id_var = SimpleNamespace(get=lambda: "0x12")
        app.motor = None
        app.bus = None
        app.settings = None
        app.busy = False
        app.current_task = None
        app.connection_cancel_event = threading.Event()
        app.connection_state_var = SimpleNamespace(set=lambda value: None)
        app.connection_state_label = SimpleNamespace(configure=lambda **kwargs: None)
        app.status_var = SimpleNamespace(set=lambda value: None)
        app._log = lambda *args, **kwargs: None
        app._start_worker = lambda task_name, operation: operation()

        fake_bus = SimpleNamespace(shutdown=lambda: None)
        fake_motor = SimpleNamespace(verify_configuration=lambda cancel_event=None: SimpleNamespace(
            control_mode=2,
            master_id=0x12,
            position_range_rad=12.5,
            velocity_range_rad_s=30.0,
            firmware_version=5017,
            sub_version=5,
            can_id=0x02,
        ))
        usb2can_bus.return_value.__enter__ = lambda self: fake_bus
        usb2can_bus.return_value.__exit__ = lambda self, exc_type, exc, tb: None
        usb2can_bus.return_value = fake_bus
        with patch("single_motor.gui.DamiaoMotor", return_value=fake_motor):
            app.apply_connection()

        validate_port.assert_called_once_with("COM3")
        can_bus.assert_not_called()
        usb2can_bus.assert_called_once_with(
            channel="COM3",
            baudrate=921600,
            can_bitrate_code=0,
        )

    def test_gui_jog_press_sends_signed_velocity_and_release_stops(self) -> None:
        app = object.__new__(MotorControlApp)
        app.active_control_panel = "manual"
        sent: list[tuple[int, bytes]] = []
        disable_calls: list[bool] = []

        app.motor = SimpleNamespace(
            enabled=True,
            last_feedback=SimpleNamespace(position_rad=0.0),
            control_mode=2,
            encode_position_command=lambda position_rad, velocity_rad_s, kp, kd: (
                0x01,
                struct.pack("<ff", position_rad, velocity_rad_s),
            ),
            _send=lambda arbitration_id, payload: sent.append((arbitration_id, payload)),
            disable=lambda: disable_calls.append(True),
        )
        app.settings = SimpleNamespace(
            control_mode=2,
            velocity_range_rad_s=30.0,
            position_range_rad=12.5,
        )
        app.jog_speed_var = SimpleNamespace(get=lambda: "12")
        app.jog_stop_event = threading.Event()
        app.jog_thread = None
        app.events = Queue()
        app._log = lambda *args, **kwargs: None
        app.status_var = SimpleNamespace(set=lambda *args, **kwargs: None)
        app.root = SimpleNamespace()
        app._set_motor_state = lambda *args, **kwargs: None

        app.start_jog(-1)
        self.assertIsNotNone(app.jog_thread)
        app.stop_jog()
        app.jog_thread.join(timeout=1.0)
        self.assertTrue(sent)
        first_position, first_speed = struct.unpack("<ff", sent[0][1])
        self.assertGreater(first_position, 0.0)
        self.assertGreater(first_speed, 0.0)
        self.assertEqual(first_speed, 12.0)
        stop_position, stop_speed = struct.unpack("<ff", sent[-1][1])
        self.assertAlmostEqual(stop_position, app.jog_target_position, places=5)
        self.assertEqual(stop_speed, 12.0)
        self.assertGreaterEqual(stop_position, first_position)
        self.assertEqual(disable_calls, [])
        self.assertTrue(app.motor.enabled)

    def test_mit_jog_uses_fixed_position_and_continuous_velocity(self) -> None:
        app = object.__new__(MotorControlApp)
        sent: list[tuple[int, bytes]] = []
        app.motor = SimpleNamespace(
            _send=lambda arbitration_id, payload: sent.append((arbitration_id, payload)),
            encode_position_command=lambda position_rad, velocity_rad_s, kp, kd: (
                0x01,
                struct.pack("<ffff", position_rad, velocity_rad_s, kp, kd),
            ),
        )
        app.settings = SimpleNamespace(control_mode=1, position_range_rad=12.5)
        app.jog_reference_position = 12.4
        app.jog_target_position = 12.4
        app.jog_speed_limit = 5.0
        app.kp_var = SimpleNamespace(get=lambda: "2.0")
        app.kd_var = SimpleNamespace(get=lambda: "1.0")
        app.events = Queue()

        for _ in range(100):
            self.assertFalse(app._send_jog_velocity(5.0, 0.01))
        app._send_jog_velocity(0.0, 0.01)

        commands = [struct.unpack("<ffff", payload) for _, payload in sent]
        self.assertTrue(all(abs(command[0] - 12.4) < 0.00001 for command in commands))
        self.assertTrue(all(command[1] == 5.0 for command in commands[:-1]))
        self.assertEqual(commands[-1][1], 0.0)
        self.assertTrue(all(command[2] == 0.0 for command in commands))

    def test_mit_jog_release_decelerates_without_reversing_or_recoiling(self) -> None:
        app = object.__new__(MotorControlApp)
        app.active_control_panel = "manual"
        sent: list[tuple[int, bytes]] = []
        enough_commands = threading.Event()

        def record_command(arbitration_id: int, payload: bytes) -> None:
            sent.append((arbitration_id, payload))
            if len(sent) >= 4:
                enough_commands.set()

        app.motor = SimpleNamespace(
            enabled=True,
            last_feedback=SimpleNamespace(position_rad=1.25),
            _read_feedback=lambda timeout_s: None,
            encode_position_command=lambda position_rad, velocity_rad_s, kp, kd: (
                0x01,
                struct.pack("<ffff", position_rad, velocity_rad_s, kp, kd),
            ),
            _send=record_command,
        )
        app.motor_enabled = True
        app.settings = SimpleNamespace(
            control_mode=1,
            velocity_range_rad_s=30.0,
            position_range_rad=12.5,
        )
        app.jog_speed_var = SimpleNamespace(get=lambda: "5")
        app.kp_var = SimpleNamespace(get=lambda: "2.0")
        app.kd_var = SimpleNamespace(get=lambda: "1.0")
        app.jog_stop_event = None
        app.jog_thread = None
        app.events = Queue()
        app._log = lambda *args, **kwargs: None
        app.status_var = SimpleNamespace(set=lambda *args, **kwargs: None)

        app.start_jog(-1)
        self.assertTrue(enough_commands.wait(timeout=1.0))
        app.stop_jog()
        app.jog_thread.join(timeout=1.0)

        commands = [struct.unpack("<ffff", payload) for _, payload in sent]
        self.assertEqual(commands[-1][1], 0.0)
        self.assertTrue(all(command[1] >= 0.0 for command in commands))
        self.assertTrue(all(abs(command[0] - 1.25) < 0.00001 for command in commands))

    def test_target_angle_is_converted_from_saved_software_zero(self) -> None:
        app = object.__new__(MotorControlApp)
        app.zero_offset_rad = 1.25

        self.assertAlmostEqual(app._motor_target_position(0.5), 1.75)
        self.assertAlmostEqual(app._motor_target_position(-0.5), 0.75)

    def test_save_zero_uses_current_motor_feedback(self) -> None:
        app = object.__new__(MotorControlApp)
        app.motor = SimpleNamespace(
            last_feedback=SimpleNamespace(position_rad=-0.75)
        )
        app.motor_enabled = True
        app.busy = False
        app.saved_target = None
        app.target_state_lock = threading.Lock()
        app.target_generation = 0
        app.target_move_cancel_event = None
        status: list[str] = []
        zero_status: list[str] = []
        app.status_var = SimpleNamespace(set=status.append)
        app.zero_status_var = SimpleNamespace(set=zero_status.append)
        app._log = lambda *args, **kwargs: None
        app._update_target_buttons = lambda: None
        app._update_hex_preview = lambda *args, **kwargs: None

        app.save_zero()

        self.assertAlmostEqual(app.zero_offset_rad, -0.75)
        self.assertEqual(zero_status, ["Zero offset: -0.750 rad"])
        self.assertEqual(status, ["Current feedback position saved as software zero"])

    def test_save_zero_rebases_target_and_restarts_active_send(self) -> None:
        app = object.__new__(MotorControlApp)
        app.motor = SimpleNamespace(
            last_feedback=SimpleNamespace(position_rad=3.0),
            encode_position_command=lambda position, speed, kp, kd: (0x102, b"12345678"),
        )
        app.settings = SimpleNamespace(
            control_mode=2,
            position_range_rad=12.5,
            velocity_range_rad_s=30.0,
        )
        app.motor_enabled = True
        app.busy = True
        app.current_task = "Sending target"
        app.target_state_lock = threading.Lock()
        app.target_generation = 1
        app.target_move_cancel_event = threading.Event()
        app.zero_offset_rad = 0.0
        app.saved_target = SavedTarget(0.5, 0.5, 2.0, 2.0, 1.0, 0x102, b"oldframe")
        app.status_var = SimpleNamespace(set=lambda value: None)
        app.zero_status_var = SimpleNamespace(set=lambda value: None)
        app._log = lambda *args, **kwargs: None
        app._update_target_buttons = lambda: None
        app._update_hex_preview = lambda: None

        app.save_zero()

        self.assertAlmostEqual(app.zero_offset_rad, 3.0)
        self.assertAlmostEqual(app.saved_target.position_rad, 3.5)
        self.assertAlmostEqual(app.saved_target.relative_angle_rad, 0.5)
        self.assertTrue(app.target_move_cancel_event.is_set())
        self.assertEqual(app.target_generation, 2)

    def test_gui_keeps_enable_state_on_enabled_feedback_status(self) -> None:
        app = object.__new__(MotorControlApp)
        app.motor = SimpleNamespace(enabled=True)
        app.zero_offset_rad = 0.0
        app.motor_enabled = True
        app._set_motor_state = lambda enabled: setattr(app, "displayed_enabled", enabled)
        app.feedback_var = SimpleNamespace(set=lambda value: setattr(app, "displayed_feedback", value))

        app._show_feedback(
            MotorFeedback(
                error_code=1,
                position_rad=1.0,
                velocity_rad_s=0.2,
                torque_nm=0.0,
                driver_temperature_c=25,
                motor_temperature_c=30,
            )
        )

        self.assertTrue(app.motor_enabled)
        self.assertTrue(app.displayed_enabled)
        self.assertIn("Status Enabled (0x1)", app.displayed_feedback)

    def test_save_zero_restarts_running_sender_with_rebased_target(self) -> None:
        app = object.__new__(MotorControlApp)
        app.active_control_panel = "target"
        app.busy = False
        app.current_task = None
        app.motor_enabled = True
        app.settings = SimpleNamespace(
            control_mode=2,
            position_range_rad=12.5,
            velocity_range_rad_s=30.0,
        )
        app.zero_offset_rad = 0.0
        app.saved_target = SavedTarget(0.5, 0.5, 2.0, 2.0, 1.0, 0x102, b"oldframe")
        app.target_state_lock = threading.Lock()
        app.target_generation = 1
        app.target_move_cancel_event = None
        app.cancel_event = threading.Event()
        app.disable_after_cancel = False
        app.events = Queue()
        app.status_var = SimpleNamespace(set=lambda value: None)
        app.zero_status_var = SimpleNamespace(set=lambda value: None)
        app.hex_preview_var = SimpleNamespace(set=lambda value: None)
        app._update_target_buttons = lambda: None
        app._update_hex_preview = lambda: None
        app._log = lambda *args, **kwargs: None
        app.root = SimpleNamespace()
        app.angle_var = SimpleNamespace(get=lambda: "0.5")
        app.angle_unit_var = SimpleNamespace(get=lambda: "rad")
        app.speed_var = SimpleNamespace(get=lambda: "2")
        app.speed_unit_var = SimpleNamespace(get=lambda: "rad/s")
        app.kp_var = SimpleNamespace(get=lambda: "2")
        app.kd_var = SimpleNamespace(get=lambda: "1")

        first_move_started = threading.Event()
        second_move_started = threading.Event()
        commanded_positions: list[float] = []
        worker_operation: list[object] = []

        def move_to_position(**kwargs):
            commanded_positions.append(kwargs["position_rad"])
            if len(commanded_positions) == 1:
                first_move_started.set()
                if not kwargs["cancel_event"].wait(timeout=2.0):
                    raise AssertionError("Save Zero did not interrupt the original move")
                raise MoveCancelledError("Target superseded")
            second_move_started.set()
            return SimpleNamespace(
                position_rad=kwargs["position_rad"],
                velocity_rad_s=0.0,
                error_code=1,
                is_enabled=True,
            )

        app.motor = SimpleNamespace(
            enabled=True,
            last_feedback=SimpleNamespace(position_rad=3.0),
            move_to_position=move_to_position,
            encode_position_command=lambda position, velocity, kp, kd: (0x102, b"12345678"),
            _send=lambda arbitration_id, payload: None,
            _read_feedback=lambda timeout_s: SimpleNamespace(
                position_rad=3.5,
                velocity_rad_s=0.0,
                error_code=1,
                is_enabled=True,
            ),
        )
        app._start_worker = lambda task, operation: (
            setattr(app, "busy", True),
            setattr(app, "current_task", task),
            worker_operation.append(operation),
        )

        app.send_target()
        errors: list[BaseException] = []

        def run_worker() -> None:
            try:
                worker_operation[0]()
            except BaseException as exc:
                errors.append(exc)

        worker = threading.Thread(target=run_worker)
        worker.start()
        self.assertTrue(first_move_started.wait(timeout=1.0))
        app.save_zero()
        self.assertTrue(second_move_started.wait(timeout=1.0))
        app.cancel_event.set()
        worker.join(timeout=1.0)

        self.assertFalse(worker.is_alive())
        self.assertEqual(commanded_positions, [0.5, 3.5])
        self.assertTrue(errors)
        self.assertIsInstance(errors[0], MoveCancelledError)

    def test_inactive_control_mode_rejects_its_commands(self) -> None:
        app = object.__new__(MotorControlApp)
        app.active_control_panel = "manual"
        app.motor = SimpleNamespace(enabled=True)
        app.motor_enabled = True

        app.send_target()
        app.active_control_panel = "target"
        app.start_jog(1)

    def test_save_target_adds_saved_zero_before_encoding(self) -> None:
        app = object.__new__(MotorControlApp)
        encoded: list[tuple[float, float, float, float]] = []
        app.active_control_panel = "target"
        app.zero_offset_rad = -1.0
        app.busy = False
        app.motor_enabled = False
        app.settings = SimpleNamespace(
            position_range_rad=12.5,
            velocity_range_rad_s=30.0,
            control_mode=2,
            master_id=0x12,
        )
        app.motor = SimpleNamespace(
            enabled=True,
            encode_position_command=lambda position, speed, kp, kd: (
                encoded.append((position, speed, kp, kd)) or (0x102, b"12345678")
            ),
        )
        app.angle_var = SimpleNamespace(get=lambda: "0.5")
        app.angle_unit_var = SimpleNamespace(get=lambda: "rad")
        app.speed_var = SimpleNamespace(get=lambda: "2")
        app.speed_unit_var = SimpleNamespace(get=lambda: "rad/s")
        app.kp_var = SimpleNamespace(get=lambda: "2")
        app.kd_var = SimpleNamespace(get=lambda: "1")
        app.root = SimpleNamespace()
        app.saved_target = None
        app.target_state_lock = threading.Lock()
        app.target_generation = 0
        app.hex_preview_var = SimpleNamespace(set=lambda value: None)
        app.status_var = SimpleNamespace(set=lambda value: None)
        app._log = lambda *args, **kwargs: None
        button = SimpleNamespace(configure=lambda **kwargs: None)
        app.send_button = button
        app.hold_send_button = button

        app.save_target()

        self.assertEqual(encoded[0][0], -0.5)
        self.assertEqual(app.saved_target.position_rad, -0.5)
        self.assertEqual(app.saved_target.relative_angle_rad, 0.5)

    def test_send_click_starts_persistent_target_worker(self) -> None:
        app = object.__new__(MotorControlApp)
        started: list[tuple[str, object]] = []
        app.active_control_panel = "target"
        app.busy = False
        app.motor_enabled = True
        app.motor = SimpleNamespace(enabled=True)
        app.settings = SimpleNamespace(control_mode=1)
        app.saved_target = SavedTarget(0.25, 0.25, 0.2, 2.0, 1.0, 0x01, b"12345678")
        app.zero_offset_rad = 0.0
        app.angle_var = SimpleNamespace(get=lambda: "0.25")
        app.angle_unit_var = SimpleNamespace(get=lambda: "rad")
        app.speed_var = SimpleNamespace(get=lambda: "0.2")
        app.speed_unit_var = SimpleNamespace(get=lambda: "rad/s")
        app.kp_var = SimpleNamespace(get=lambda: "2.0")
        app.kd_var = SimpleNamespace(get=lambda: "1.0")
        app.cancel_event = threading.Event()
        app.events = Queue()
        app._log = lambda *args, **kwargs: None
        app._start_worker = lambda task, operation: started.append((task, operation))

        app.send_target()

        self.assertEqual(len(started), 1)
        self.assertEqual(started[0][0], "Sending target")
        self.assertFalse(app.cancel_event.is_set())

    @patch("single_motor.gui.messagebox.showwarning")
    def test_unsaved_target_edits_cannot_be_sent(self, showwarning) -> None:
        app = object.__new__(MotorControlApp)
        app.active_control_panel = "target"
        app.busy = False
        app.motor_enabled = True
        app.zero_offset_rad = 0.0
        app.settings = SimpleNamespace(position_range_rad=12.5, velocity_range_rad_s=30.0)
        app.saved_target = SavedTarget(0.25, 0.25, 0.2, 2.0, 1.0, 0x01, b"12345678")
        app.motor = SimpleNamespace(enabled=True)
        app.angle_var = SimpleNamespace(get=lambda: "0.5")
        app.angle_unit_var = SimpleNamespace(get=lambda: "rad")
        app.speed_var = SimpleNamespace(get=lambda: "0.2")
        app.speed_unit_var = SimpleNamespace(get=lambda: "rad/s")
        app.kp_var = SimpleNamespace(get=lambda: "2.0")
        app.kd_var = SimpleNamespace(get=lambda: "1.0")
        app.root = SimpleNamespace()
        started: list[str] = []
        app._start_worker = lambda task, operation: started.append(task)

        states: dict[str, str] = {}
        app.send_button = SimpleNamespace(configure=lambda **kwargs: states.update(save=kwargs["state"]))
        app.hold_send_button = SimpleNamespace(configure=lambda **kwargs: states.update(send=kwargs["state"]))
        app._update_target_buttons()
        app.send_target()

        self.assertEqual(states, {"save": "normal", "send": "disabled"})
        self.assertEqual(started, [])
        showwarning.assert_called_once()

    def test_target_values_converts_90_degrees_to_radians(self) -> None:
        app = object.__new__(MotorControlApp)
        app.angle_var = SimpleNamespace(get=lambda: "90")
        app.angle_unit_var = SimpleNamespace(get=lambda: "deg")
        app.speed_var = SimpleNamespace(get=lambda: "0.2")
        app.speed_unit_var = SimpleNamespace(get=lambda: "rad/s")
        app.kp_var = SimpleNamespace(get=lambda: "2")
        app.kd_var = SimpleNamespace(get=lambda: "1")

        angle_rad, speed_rad_s, _, _ = app._target_values()

        self.assertAlmostEqual(angle_rad, math.pi / 2)
        self.assertEqual(speed_rad_s, 0.2)

    def test_saved_mit_target_hex_uses_signed_requested_speed(self) -> None:
        motor = DamiaoV12(FakeBus(control_mode=1), can_id=0x01, master_id=0x11)
        settings = motor.verify_configuration()
        motor.last_feedback = SimpleNamespace(position_rad=0.0)
        app = object.__new__(MotorControlApp)
        app.motor = motor
        app.settings = settings
        app.zero_offset_rad = 0.0

        positive = app._build_saved_target(math.pi / 2, 0.2, 2.0, 1.0)
        negative = app._build_saved_target(-math.pi / 2, 0.2, 2.0, 1.0)

        def decode_velocity(payload: bytes) -> float:
            raw = (payload[2] << 4) | (payload[3] >> 4)
            return raw * 60.0 / 4095.0 - 30.0

        self.assertAlmostEqual(decode_velocity(positive.payload), 0.2, delta=0.02)
        self.assertAlmostEqual(decode_velocity(negative.payload), -0.2, delta=0.02)


if __name__ == "__main__":
    unittest.main()
