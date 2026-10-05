"""Move one DM4310 V3 motor to an absolute output-shaft angle."""

from __future__ import annotations

import argparse
import math
import os
import re
import sys
from pathlib import Path

import can
from dotenv import load_dotenv
from serial.tools import list_ports

from single_motor.damiao_v12 import DamiaoV12
from single_motor.usb2can_serial import DamiaoUSB2CANBus, USB2CAN_CAN_BAUDS_KBPS


PROJECT_DIR = Path(__file__).resolve().parents[1]
load_dotenv(PROJECT_DIR / ".env")


def env_int(name: str, default: int) -> int:
    value = os.getenv(name)
    return int(value, 0) if value else default


def env_float(name: str, default: float) -> float:
    value = os.getenv(name)
    return float(value) if value else default


def validate_usb2can_port(channel: str) -> None:
    if not re.fullmatch(r"COM\d+", channel, flags=re.IGNORECASE):
        raise ValueError("USB2CAN channel must be a Windows COM port, for example COM5.")

    matches = [
        item for item in list_ports.comports() if item.device.upper() == channel.upper()
    ]
    port = next((item for item in matches if item.vid is not None and item.pid is not None), None)
    if port is None:
        port = next(
            (item for item in matches if "bluetooth" not in (item.description or "").lower()),
            None,
        )
    if port is None:
        available = ", ".join(item.device for item in list_ports.comports()) or "none"
        raise RuntimeError(f"{channel} is not a USB serial adapter. Detected ports: {available}.")


validate_slcan_port = validate_usb2can_port


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Move a DM-J4310-2EC V1.2 to an absolute output-shaft angle."
    )
    parser.add_argument("--angle", type=float, required=True, help="Target angle in radians")
    parser.add_argument(
        "--speed", type=float, required=True, help="Maximum speed in radians per second"
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Actually connect, enable the motor, and execute the move",
    )
    parser.add_argument(
        "--disable-after",
        action="store_true",
        help="Disable torque after reaching the target; only use if the mechanism can relax safely",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    can_channel = os.getenv("DAMIAO_CAN_CHANNEL", "COM3")
    bitrate = env_int("DAMIAO_CAN_BITRATE", 1_000_000)
    serial_baudrate = env_int("DAMIAO_CAN_SERIAL_BAUDRATE", 921_600)
    can_id = env_int("DAMIAO_CAN_ID", 0x02)
    master_id = env_int("DAMIAO_MASTER_ID", 0x12)
    safe_speed = env_float("DAMIAO_SAFE_MAX_SPEED_RAD_S", 30.0)
    position_tolerance = env_float("DAMIAO_POSITION_TOLERANCE_RAD", 0.05)
    velocity_tolerance = env_float("DAMIAO_VELOCITY_TOLERANCE_RAD_S", 0.1)
    move_timeout = env_float("DAMIAO_MOVE_TIMEOUT_S", 30.0)
    command_rate = env_float("DAMIAO_COMMAND_RATE_HZ", 20.0)

    if not math.isfinite(args.angle) or not math.isfinite(args.speed):
        raise ValueError("Target angle and speed must be finite numbers.")
    if args.speed > safe_speed:
        raise ValueError(
            f"Requested speed {args.speed:g} rad/s exceeds the configured initial cap "
            f"of {safe_speed:g} rad/s in .env."
        )
    if args.speed <= 0:
        raise ValueError("Speed must be greater than zero.")
    if command_rate <= 0 or move_timeout <= 0:
        raise ValueError("Command rate and move timeout must be greater than zero.")

    try:
        can_bitrate_code = USB2CAN_CAN_BAUDS_KBPS.index(bitrate // 1000)
    except ValueError as exc:
        raise ValueError(f"Unsupported Damiao USB2CAN CAN bitrate: {bitrate} bit/s.") from exc

    print(
        f"Target: {args.angle:g} rad; max speed: {args.speed:g} rad/s; "
        f"CAN ID: 0x{can_id:02X}; Master ID: 0x{master_id:02X}; "
        f"{can_channel} at {bitrate} bit/s."
    )
    if not args.execute:
        print("Preview only. Add --execute to connect and move the motor.")
        return 0

    validate_usb2can_port(can_channel)

    with DamiaoUSB2CANBus(
        channel=can_channel,
        baudrate=serial_baudrate,
        can_bitrate_code=can_bitrate_code,
    ) as bus:
        motor = DamiaoV12(bus, can_id=can_id, master_id=master_id)
        settings = motor.verify_configuration()
        print(
            f"Verified Position-Velocity Mode; PMAX ±{settings.position_range_rad:g} rad, "
            f"VMAX ±{settings.velocity_range_rad_s:g} rad/s."
        )
        result = motor.move_to_position(
            position_rad=args.angle,
            max_velocity_rad_s=args.speed,
            position_tolerance_rad=position_tolerance,
            velocity_tolerance_rad_s=velocity_tolerance,
            timeout_s=move_timeout,
            command_rate_hz=command_rate,
        )
        print(
            f"Reached {result.position_rad:.3f} rad "
            f"(speed {result.velocity_rad_s:.3f} rad/s)."
        )
        if args.disable_after:
            motor.disable()
            print("Motor disabled. Active torque has been removed.")
        else:
            print("Motor remains enabled at the target position.")

    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (can.CanError, OSError, RuntimeError, TimeoutError, ValueError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc