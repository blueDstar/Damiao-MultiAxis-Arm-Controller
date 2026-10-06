# Independent motors on one CAN bus

Run from the controller directory on Windows:

```powershell
cd F:\Damiao_Control\motor_controller
python run_multi_motor.py
```

The launcher uses the installed Python and, if CAN dependencies are missing there, reuses `.venv/Lib/site-packages`. A working virtual environment can also run `python -m multi_motor.gui`. The existing environment may need rebuilding if its original base Python has been removed.

The interface uses black surfaces, white text, neon green outlined buttons and side-by-side motor cards. Each card has a persistent Enable badge and large position, speed and torque readouts.

The GUI opens one Damiao USB2CAN serial adapter for all motors. Initial panels use:

| Motor | CAN ID (commands) | Master ID (feedback) | Reported firmware | Reported mode |
| --- | --- | --- | --- | --- |
| 1 | `0x01` | `0x11` | `5017.005` | MIT |
| 2 | `0x02` | `0x12` | `5017.005` | MIT |

IDs identify motors; they do not change the IDs stored on a drive. Edit panel IDs before opening the bus. CAN IDs and Master IDs must be unique. This implementation accepts CAN IDs `0x01` through `0x0F`, matching the four-bit motor identity in the existing feedback decoder. Master IDs must be standard CAN IDs, nonzero, below `0x7FF`, and must not overlap command IDs. Add more panels before connecting when needed.

Connect every drive to the same CAN-H/CAN-L bus and configure all devices for the same CAN bitrate. The defaults are CAN **1 Mbps** and USB serial **921600 baud**. These are different settings. Opening the bus configures the adapter's CAN bitrate; it does not rewrite a motor's firmware, IDs, mode, calibration or bitrate. Close other programs using the COM port first.

1. Choose the adapter COM port and open the CAN bus. Each panel independently reads IDs, mode, firmware, PMAX, VMAX and TMAX, then compares decoded feedback position against the floating-point XOUT register (`0x51`). A mismatch or missing status feedback blocks Enable. Verification sends only register/status queries. A missing motor leaves its panel unavailable while other panels can operate. Use **Xác minh motor** to retry when that motor is stationary and connected.
2. Click **Enable**. In MIT mode the controller first sends a neutral command to replace a retained target, then enables the driver and streams **Kp=0, Kd=0, velocity=0, feedforward torque≈0**. Enable does not engage position holding or start a target/jog motion. Torque zero has a small encoding quantization error. The driver must report **1 (Enabled)**; **0 means Disabled**. These are normal states, not faults. Position-Velocity mode uses status queries while idle rather than sending a position target.
3. The first **Enable** in a connection automatically records the current feedback position as that motor's software zero, without moving the shaft. **Lưu zero** records a new software origin when needed. Position targets use `absolute target = saved zero + entered angle`. Neither action writes persistent drive calibration. Re-enabling preserves the saved origin; disconnect resets it. Saving zero invalidates saved targets and affects future moves, without redirecting an existing trajectory.
4. In **POSITION**, enter angle (degrees or radians), signed speed **−30 to +30 rad/s** (or equivalent rpm), Kp and Kd. The target angle determines rotation direction; Position uses the speed's magnitude. Negative speed does not negate the target angle. Zero speed requests deceleration/stop rather than moving to a new angle. Click **Lưu đích / xem CAN**, then **Gửi đích**. The preview shows start/target angles, signed trajectory velocity and expected duration. Both driver modes use position references sampled from elapsed monotonic time: `p(t) = p_start + sign(p_target - p_start) * abs(speed) * t`, clamped at the target. Duration is `abs(p_target - p_start) / abs(speed)`. MIT velocity feedforward follows that signed speed during the reference trajectory and becomes zero at its endpoint; native Position-Velocity frames use the positive speed magnitude as their speed cap. Completion requires the trajectory to finish and three feedback samples within **0.005 rad** of the target with near-zero velocity. The motor then holds the target. **DỪNG CHUYỂN ĐỘNG** requests deceleration instead. Position does not use the Manual Jog acceleration ramp; that would change the requested constant-speed reference duration.

   Example from software zero: **90° = π/2 rad** at **1 rad/s = 9.5492966 rpm** gives a **1.5707963 second reference trajectory**. A move from 60° to 90° at the same speed has only 30° left and takes 0.5235988 seconds. Software timing describes the reference, not a guarantee of measured shaft velocity or settling time; load, MIT gains, CAN quantization and driver response still affect actual feedback. Position tracking and overspeed checks stop an axis that deviates excessively. Physical motor verification remains outstanding.

   Switching the angle/speed unit selectors converts the existing number rather than reinterpreting it. This preserves the physical request and a saved target across deg↔rad and rad/s↔rpm changes. The hexadecimal preview is the endpoint frame; TX shows actual intermediate frames. Editing the physical target or gains requires saving again.
5. Click **SANG MANUAL JOG** to change panels. The old stream is cancelled; an enabled motor decelerates and must report stopped feedback before the switch completes. The stopped position becomes software zero, so the manual readout starts at zero without physically returning to absolute zero. Saved position targets are cleared. No motor mode register or permanent zero is written. In **MANUAL JOG**, enter a signed Jog speed from **−30 to +30 rad/s**, or the equivalent rpm, subject to the individual driver's VMAX. **GIỮ TỐC ĐỘ ĐÃ NHẬP** uses that signed speed directly. **Jog +** uses the positive magnitude and **Jog −** the negative magnitude, regardless of the entered sign. Entering zero and pressing a Jog control requests deceleration/stop. Jog's initial field value `0.2` is only a starting value, not a speed limit. MIT jog keeps position **Kp=0**, ramps signed velocity with Kd damping, and decelerates to zero at 1 rad/s² on mouse release, focus loss or **DỪNG JOG**. The ramp remains active: reaching 30 rad/s from rest takes approximately 30 seconds of continuous holding. MIT velocity jog does not stop at the feedback's PMAX boundary; its position field applies no holding torque. Native Position-Velocity commands retain their PMAX travel check. It does not re-engage position Kp on release. The inactive panel's commands are blocked in both GUI and controller. Switching back to Position also cancels the manual stream and waits for the motor to stop.
6. **Disable motor** cancels queued commands and stops only that motor. **DISABLE TẤT CẢ**, disconnect and window close request Disable for every verified motor. These actions remove active holding torque. A motor with failed verification receives no guessed Enable/Disable/movement frame.

Each panel displays its feedback position relative to its own zero, velocity, torque, temperatures, driver fault code, firmware and active ranges. **Đọc tham số** opens that motor's register table while it is disabled. Register reads are blocked on an enabled axis so long parameter reads cannot interrupt its command stream; other axes continue running.

The three numeric displays suppress only values within one CAN quantization step of zero. This display adjustment is not applied to control feedback; a stopped motor can still produce measurable torque. Status queries (`0x7FF`, payload `CAN_ID 00 CC 00 00 00 00 00`) refresh disabled axes at 4 Hz. Readings older than 0.6 seconds become **—**, rather than retaining stale speed/torque. RX displays the actual feedback bytes; double-click RX to copy the frame to the clipboard and log.

The shared connection settings default from `.env` (`DAMIAO_CAN_CHANNEL`, `DAMIAO_CAN_BITRATE`, `DAMIAO_CAN_SERIAL_BAUDRATE`). **Position and Manual Jog both accept ±min(30, driver VMAX) rad/s**. The old `DAMIAO_MULTI_SPEED_CAP_RAD_S` setting no longer reduces Position speed. The legacy controller `speed_cap` argument remains accepted for compatibility but does not cap requested Position/Jog speeds. MIT mode does not provide a guaranteed measured-speed limiter. Position targets must remain within PMAX. The software has not been tested on the physical two-motor assembly.

Active axes default to 100 command frames/s; set `DAMIAO_MULTI_COMMAND_RATE_HZ` before launching to change this. An axis requests Disable on a fault, 1 second of lost feedback, tracking error above `max(0.15 rad, requested speed * max(0.05 s, 2 command intervals))`, or three samples of speed above `max(2 * requested speed, requested speed + 0.15 rad/s)`. Target timeout is `max(configured timeout, reference duration + 2 seconds)`, so a valid slow trajectory is not cut off at 30 seconds. These checks do not constitute a hardware speed limiter. If a shared adapter connection is lost, Disable delivery cannot be confirmed. Error details appear in the panel and log.

The controller reuses `single_motor.damiao_v12.DamiaoMotor`. Feedback decoding has been corrected to **unsigned offset-binary mapping**: `value = raw * (2 * range) / (2**bits - 1) - range`, with 16 bits for position and 12 for velocity/torque. This matches the [Damiao Python library's feedback decoder](https://github.com/cmjang/DM_Control_Python/blob/main/DM_CAN.py). Reading those fields as two's-complement signed integers turns the midpoint near zero into alternating near-maximum positive/negative readings and corrupts position references. Literal midpoint/end-point frame tests cover this regression. The controller supports MIT (driver mode 1) and Position-Velocity (driver mode 2); the Position/Manual Jog buttons select application control panels and do not rewrite the driver's mode. Speed-only and hybrid driver modes are rejected during verification.

Files:

- `can_router.py`: one adapter receiver, routes by Master ID and checks payload motor identity; separate register and feedback inboxes preserve received faults.
- `controller.py`: one serialized command worker and periodic stream per motor, with independent configuration, zero, motion and timeout state.
- `trajectory.py`: constant-speed Position references using radians, rad/s and elapsed time, independent of CAN frame count.
- `gui.py`: shared connection controls and independent motor panels. Tk widgets are updated on the main thread through queued events.

Run the simulated shared-bus and GUI regression checks, followed by existing single-motor tests:

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
```

If the original virtual environment's Python is missing, run the tests with the launcher package setup:

```powershell
python -c "import unittest; from run_multi_motor import prepare_local_packages; prepare_local_packages(); unittest.main(module=None, argv=['unittest', 'discover', '-s', 'tests', '-v'])"
```

For programmatic use, create `MultiMotorController(bus, [(0x01, 0x11), (0x02, 0x12)])`, then call a session's `submit(action, *args)`. It returns a `Future`. Actions are `verify`, `enable`, `zero`, `mode('position' or 'manual')`, `target(relative_rad, signed_speed_rad_s, kp, kd)`, `jog(signed_speed_rad_s, kp, kd)`, `hold` (decelerate/stop), `parameters` and `disable`. Target zero speed requests a stop. Mode switches complete when a `mode` event is emitted and `pending_mode` becomes `None`; submission futures acknowledge requests, not completion of motion. Always call `controller.close()` to stop workers and close the shared bus.
