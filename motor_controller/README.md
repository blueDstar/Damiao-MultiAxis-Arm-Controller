# Damiao Motor Controller

Python controller for one DM4310 V3 motor running firmware 5017. The GUI supports the motor's current MIT mode and Position-Velocity mode; `multi_motor/` is reserved for future coordinated control.

## Setup

Use Python 3.12 and the local virtual environment:

```powershell
cd F:\Damiao_Control\motor_controller
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
```
cd f:\Damiao_Control\motor_controller
.\.venv\Scripts\python.exe -m single_motor.gui 

Local connection settings are in `.env`. The starter configuration uses CAN ID `0x01`, Master ID `0x11`, CAN bitrate `1 Mbps`, and a conservative software speed cap of `0.5 rad/s`.

Start the desktop GUI with:

```powershell
python -m single_motor.gui
```

The GUI accepts angle in radians or degrees and speed in rad/s or rpm. **Connect** opens the selected adapter and verifies the configured CAN IDs and driver registers; **Cancel** aborts a pending register read. A separate CAN indicator reports verified communication. **Enter Motor** sends the enable frame; the motor indicator turns green only after the driver reports enabled status. These indicators describe the CAN link and motor driver state, not a controllable USB2CAN LED. The live preview and send log show the arbitration ID and eight payload bytes in hex. **Send target** is enabled only while the motor is green. **Stop / Disable** sends the disable frame and turns the motor indicator red. **Read present parameters** reads V3/V17 CAN registers into a table. MIT and Position-Velocity modes are supported; configure the mode in Damiao Debugging Tool if needed.

## Connection requirement

Windows currently exposes a USB device on `COM3` (`VID:PID 2E88:4603`) alongside a stale Bluetooth record with the same COM number; the GUI now prefers the USB device. Its current transport uses SLCAN. A read-only query opened COM3 but received no motor response, so SLCAN compatibility or the adapter's vendor runtime is not yet confirmed. The GUI does not send Enable or movement commands when this read fails. If Damiao Debugging Tool communicates with this adapter, use the matching USB2CAN protocol/runtime rather than changing motor IDs or CAN bitrate blindly. Adapter serial baud and CAN bitrate are separate settings.

## Move to an angle from the command line

Angles are absolute output-shaft positions in radians; speed is a positive maximum in rad/s. First preview without touching the motor:

```powershell
python -m single_motor.move_to_angle --angle 1.0 --speed 0.2
```

After confirming the motor is securely mounted, the CAN wiring and adapter are correct, and the target is within the mechanism's travel, execute:

```powershell
python -m single_motor.move_to_angle --angle 1.0 --speed 0.2 --execute
```

The program only sends an enable command after the driver replies with the expected IDs and a supported mode. It waits for position and velocity feedback, stops on a fault or timeout, and leaves the drive enabled at the target unless `--disable-after` is specified. Disabling removes active motor torque; use that option only if the mechanism can safely relax.

The current speed cap, timeout, command rate, and tolerances can be adjusted in `.env`. Keep the speed low during initial tests. The software cannot know the arm's mechanical limits or load; verify travel and emergency-stop behavior before motion.

## Tests

Run the simulated CAN protocol tests without connecting a motor:

```powershell
python -m unittest discover -s tests -v
```

## Protocol scope

This implementation targets the DM4310 V3 / firmware 5017 register and CAN protocol. It reads the firmware version and sub-version from the drive before showing the active configuration. V1.1, V1.2, and V3 motors have different ratings; this program does not update firmware or CAN bitrate.