"""Serial framing used by Damiao USB2CAN Tools 2.0.0.3."""

from __future__ import annotations

import queue
import struct
import threading
import time
from typing import Protocol

import can
import serial


class SerialPort(Protocol):
    is_open: bool

    def read(self, size: int = 1) -> bytes: ...

    def write(self, data: bytes) -> int: ...

    def close(self) -> None: ...


USB2CAN_HEADER = b"\x55\xAA\x1E\x01"
USB2CAN_CAN_COMMAND = 0x01
USB2CAN_STOP_COMMAND = b"\x55\x03\xAA\x55"
USB2CAN_HEARTBEAT_COMMAND = b"\x55\x04\xAA\x55"
USB2CAN_CONTROL_BLOCK_SIZE = 17
USB2CAN_CAN_DATA_SIZE = 8
USB2CAN_FRAME_SIZE = 16
USB2CAN_CAN_BAUD_COMMAND = bytes((0x55, 0x05, 0x00, 0xAA, 0x55))
USB2CAN_CAN_BAUDS_KBPS = (1000, 800, 666, 500, 400, 250, 200, 125, 100, 80, 50, 40, 20, 10, 5)


def crc8_dallas_maxim(data: bytes) -> int:
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ (0x8C if crc & 1 else 0)
    return crc


def encode_usb2can_frame(message: can.Message) -> bytes:
    if message.is_error_frame:
        raise ValueError("The Damiao USB2CAN serial protocol does not support error frames.")
    if message.dlc > USB2CAN_CAN_DATA_SIZE:
        raise ValueError("CAN classic frames may contain at most 8 data bytes.")

    maximum_id = 0x1FFFFFFF if message.is_extended_id else 0x7FF
    if not 0 <= message.arbitration_id <= maximum_id:
        raise ValueError("CAN arbitration ID is outside the selected frame format.")

    send_count = 1
    send_interval_tenths_ms = 10
    can_id_type = int(message.is_extended_id)
    can_frame_type = int(message.is_remote_frame)
    payload = bytes(message.data).ljust(USB2CAN_CAN_DATA_SIZE, b"\x00")
    control_block = struct.pack(
        "<IIBIBBBB",
        send_count,
        send_interval_tenths_ms,
        can_id_type,
        message.arbitration_id,
        can_frame_type,
        message.dlc,
        0,
        0,
    )
    packet = bytearray(USB2CAN_HEADER + control_block + payload + b"\x00")
    packet[-1] = crc8_dallas_maxim(packet[:-1])
    return bytes(packet)


class DamiaoUSB2CANBus(can.BusABC):
    """A python-can-compatible facade over the vendor USB2CAN UART packet format."""

    def __init__(
        self,
        channel: str,
        baudrate: int = 921_600,
        can_bitrate_code: int | None = None,
        timeout: float = 0.05,
        serial_instance: SerialPort | None = None,
    ) -> None:
        if serial_instance is None:
            self._serial: SerialPort = serial.Serial(
                port=channel,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=timeout,
                write_timeout=0.5,
            )
        else:
            self._serial = serial_instance

        try:
            self._serial.rts = True  # type: ignore[attr-defined]
        except (AttributeError, OSError, serial.SerialException):
            pass

        self.channel_info = f"Damiao USB2CAN serial interface: {channel}"
        self._messages: queue.Queue[can.Message] = queue.Queue()
        self._errors: queue.Queue[BaseException] = queue.Queue()
        self._rx_buffer = bytearray()
        self._write_lock = threading.Lock()
        self._closed = threading.Event()
        self._reader = threading.Thread(
            target=self._read_loop,
            name="damiao-usb2can-reader",
            daemon=True,
        )
        self._reader.start()
        self._can_protocol = can.CanProtocol.CAN_20
        super().__init__(channel=channel)

        if can_bitrate_code is not None:
            self.set_can_bitrate(can_bitrate_code)

    @property
    def is_open(self) -> bool:
        return self._serial.is_open and not self._closed.is_set()

    def set_can_bitrate(self, code: int) -> None:
        if not 0 <= code < len(USB2CAN_CAN_BAUDS_KBPS):
            raise ValueError("Unsupported Damiao USB2CAN CAN bitrate selection.")
        command = bytearray(USB2CAN_CAN_BAUD_COMMAND)
        command[2] = code
        self._write(bytes(command))

    def _write(self, payload: bytes) -> None:
        with self._write_lock:
            self._serial.write(payload)

    def send(self, message: can.Message, timeout: float | None = None) -> None:
        if not self.is_open:
            raise can.CanOperationError("USB2CAN serial port is closed.")
        self._write(encode_usb2can_frame(message))

    def recv(self, timeout: float | None = None) -> can.Message | None:
        if not self.is_open and self._messages.empty():
            raise can.CanOperationError("USB2CAN serial port is closed.")
        try:
            return self._messages.get(timeout=timeout)
        except queue.Empty:
            try:
                error = self._errors.get_nowait()
            except queue.Empty:
                return None
            raise can.CanOperationError(f"USB2CAN serial receive failed: {error}") from error

    def send_heartbeat(self) -> None:
        self._write(USB2CAN_HEARTBEAT_COMMAND)

    def stop_periodic_send(self) -> None:
        self._write(USB2CAN_STOP_COMMAND)

    def shutdown(self) -> None:
        if self._closed.is_set():
            return
        self._closed.set()
        if self._reader.is_alive():
            self._reader.join(timeout=0.5)
        if self._serial.is_open:
            try:
                self._serial.write(USB2CAN_STOP_COMMAND)
            except (OSError, serial.SerialException):
                pass
            self._serial.close()
        super().shutdown()

    def _read_loop(self) -> None:
        while not self._closed.is_set():
            try:
                chunk = self._serial.read(256)
            except (OSError, serial.SerialException) as exc:
                if not self._closed.is_set():
                    self._errors.put(exc)
                return
            if chunk:
                self._rx_buffer.extend(chunk)
                self._extract_frames()

    def _extract_frames(self) -> None:
        while len(self._rx_buffer) >= USB2CAN_FRAME_SIZE:
            start = self._rx_buffer.find(b"\xAA")
            if start < 0:
                self._rx_buffer.clear()
                return
            if start:
                del self._rx_buffer[:start]
            if len(self._rx_buffer) < USB2CAN_FRAME_SIZE:
                return

            frame = bytes(self._rx_buffer[:USB2CAN_FRAME_SIZE])
            if frame[-1] != 0x55:
                del self._rx_buffer[0]
                continue
            del self._rx_buffer[:USB2CAN_FRAME_SIZE]

            length_flags = frame[2]
            dlc = length_flags & 0x0F
            if dlc > USB2CAN_CAN_DATA_SIZE:
                continue
            arbitration_id = int.from_bytes(frame[3:7], byteorder="little", signed=False)
            is_extended_id = bool(length_flags & 0x40)
            is_remote_frame = bool(length_flags & 0x80)
            self._messages.put(
                can.Message(
                    timestamp=time.time(),
                    arbitration_id=arbitration_id,
                    is_extended_id=is_extended_id,
                    is_remote_frame=is_remote_frame,
                    dlc=dlc,
                    data=frame[7 : 7 + dlc],
                )
            )