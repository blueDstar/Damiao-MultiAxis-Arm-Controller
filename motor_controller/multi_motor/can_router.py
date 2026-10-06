"""One CAN receiver, separate status/register inboxes for each motor."""

from __future__ import annotations

import queue
import threading

import can

from single_motor.damiao_v12 import CAN_REGISTER_REQUEST_ID, REGISTER_READ_COMMAND, DamiaoMotor


class MotorChannel:
    def __init__(self, router: CANRouter, can_id: int, master_id: int) -> None:
        self.router = router
        self.can_id = can_id
        self.master_id = master_id
        self.status: queue.Queue[can.Message] = queue.Queue(maxsize=1)
        self.registers: queue.Queue[can.Message] = queue.Queue(maxsize=64)
        self.reading_register = False
        self._inbox_lock = threading.Lock()

    @staticmethod
    def clear(inbox: queue.Queue) -> None:
        while True:
            try:
                inbox.get_nowait()
            except queue.Empty:
                return

    def send(self, message: can.Message) -> None:
        if message.arbitration_id == CAN_REGISTER_REQUEST_ID and message.data[2] == REGISTER_READ_COMMAND:
            self.clear(self.registers)
        else:
            # A response must follow this command, rather than an earlier enable/target.
            # Preserve faults already received until the motor worker sees them.
            with self._inbox_lock:
                fault = None
                while True:
                    try:
                        previous = self.status.get_nowait()
                        if previous.data[0] >> 4 not in (0, 1):
                            fault = previous
                    except queue.Empty:
                        break
                if fault is not None and bytes(message.data) != b"\xff" * 7 + b"\xfc":
                    self.status.put_nowait(fault)
        self.router.send(message)

    def recv(self, timeout: float | None = None) -> can.Message | None:
        self.router.check_alive()
        inbox = self.registers if self.reading_register else self.status
        try:
            result = inbox.get(timeout=timeout)
        except queue.Empty:
            self.router.check_alive()
            return None
        self.router.check_alive()
        return result

    def deliver(self, message: can.Message) -> None:
        data = message.data
        is_register = (
            data[0] == self.can_id and data[1] == 0 and data[2] == REGISTER_READ_COMMAND
        )
        if not is_register and data[0] & 0x0F != self.can_id:
            return
        inbox = self.registers if is_register else self.status
        with self._inbox_lock:
            try:
                inbox.put_nowait(message)
            except queue.Full:
                try:
                    previous = inbox.get_nowait()
                except queue.Empty:
                    previous = None
                if not is_register and previous is not None and previous.data[0] >> 4 not in (0, 1):
                    inbox.put_nowait(previous)
                else:
                    inbox.put_nowait(message)


class RoutedMotor(DamiaoMotor):
    """Reuse the single-motor codec while keeping register replies out of feedback."""

    def read_register(self, register, timeout_s=1.0, cancel_event=None):
        self.bus.reading_register = True
        try:
            return super().read_register(register, timeout_s, cancel_event)
        finally:
            self.bus.reading_register = False


class CANRouter:
    def __init__(self, bus: can.BusABC) -> None:
        self.bus = bus
        self.channels: dict[int, MotorChannel] = {}
        self._lock = threading.Lock()
        self._write_lock = threading.Lock()
        self._closed = threading.Event()
        self.error: Exception | None = None
        self._reader = threading.Thread(target=self._receive, name="multi-can-rx", daemon=True)
        self._reader.start()

    def add_motor(self, can_id: int, master_id: int) -> RoutedMotor:
        if not 1 <= can_id <= 0x0F:
            raise ValueError("CAN ID must be 0x01..0x0F for this feedback format.")
        if not 1 <= master_id < CAN_REGISTER_REQUEST_ID:
            raise ValueError("Master ID must be 0x001..0x7FE.")
        with self._lock:
            if master_id in self.channels or any(c.can_id == can_id for c in self.channels.values()):
                raise ValueError("Each motor must have a unique CAN ID and Master ID.")
            command_ids = {c.can_id for c in self.channels.values()} | {
                0x100 + c.can_id for c in self.channels.values()
            }
            if master_id in command_ids | {can_id, 0x100 + can_id} or (
                can_id in self.channels or 0x100 + can_id in self.channels
            ):
                raise ValueError("Master IDs must not overlap motor command IDs.")
            channel = MotorChannel(self, can_id, master_id)
            motor = RoutedMotor(channel, can_id, master_id)
            self.channels[master_id] = channel
            return motor

    def check_alive(self) -> None:
        if self.error is not None:
            raise can.CanOperationError(f"Shared CAN receive failed: {self.error}") from self.error
        if self._closed.is_set():
            raise can.CanOperationError("Shared CAN connection is closed.")

    def send(self, message: can.Message) -> None:
        with self._write_lock:
            self.check_alive()
            self.bus.send(message)

    def _receive(self) -> None:
        try:
            while not self._closed.is_set():
                message = self.bus.recv(timeout=0.05)
                if message is None or message.is_extended_id or message.is_remote_frame or message.is_error_frame:
                    continue
                if len(message.data) != 8:
                    continue
                with self._lock:
                    channel = self.channels.get(message.arbitration_id)
                if channel is not None:
                    channel.deliver(message)
        except Exception as exc:
            if not self._closed.is_set():
                self.error = exc

    def close(self) -> None:
        self._closed.set()
        self._reader.join(timeout=1.0)
        self.bus.shutdown()
