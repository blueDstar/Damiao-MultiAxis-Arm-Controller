# Damiao Motor Controller

Python controller for DM4310 V3 motors running firmware 5017. The single-motor GUI supports MIT and Position-Velocity modes. `multi_motor/` provides independent controls for multiple motors sharing one USB2CAN adapter.

For the two-motor GUI (CAN IDs `0x01`/`0x02`, Master IDs `0x11`/`0x12`), run from this directory:

```powershell
python run_multi_motor.py
```

See [multi_motor/README.md](multi_motor/README.md) for independent Enable, target, jog, software zero, feedback routing and Disable controls.

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

The connection code prefers USB serial devices over Bluetooth records with the same COM number. Both GUIs use the Damiao USB2CAN vendor serial framing in `single_motor/usb2can_serial.py`, with a default serial baud of 921600. They verify motor registers before allowing Enable or motion. Adapter serial baud and CAN bitrate are separate settings. Select the actual adapter COM port and close Damiao Debugging Tool or other programs holding that port before connecting.

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
