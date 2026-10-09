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
3. The first **Enable** records a software zero without moving the shaft. **Lưu zero** records another origin when needed. Entry into either Position or Manual Jog after a control-mode switch uses the stopped position as the new software origin. This does not write drive calibration; switching view tabs does not change the origin.
4. In **POSITION**, the default **Quay thêm từ hiện tại** means 90° rotates another quarter turn from the position measured when Send is pressed, regardless of earlier Jog rotations. 360° rotates another complete turn, and 720° another two turns. **Tới góc từ zero** instead selects a coordinate relative to the saved origin. The saved profile includes this choice; changing it requires saving again. Speed accepts ±min(30, VMAX) rad/s (or equivalent rpm); its magnitude controls Position speed and angle sign determines the rotation direction. In MIT mode every move/end/stop frame keeps **Kp=0**, including at the endpoint. No wrapped/raw feedback coordinate is sent as an active position-PD target, which previously could pull the drive back to zero after multiple turns. Near the endpoint, velocity decreases using measured remaining angle / 0.1 seconds. Reaching the tolerance or crossing the endpoint latches a zero-velocity Stop; overshoot does not initiate repeated direction reversals. Completion waits for three near-zero measured velocity samples and logs the measured final angle error. The end state retains enabled velocity damping, not firmware position holding. Native Pos-Vel mode uses its float32 position target. There is no elapsed-reference tracking-error Disable. Zero speed and Stop buttons request zero velocity on the next cycle.

   The preview duration is **ideal**: `abs(target - current measured angle) / abs(speed)`. For example, 90° at 1 rad/s is ideally 1.5707963 seconds; 720° at 5 rad/s is ideally 2.5132741 seconds. Actual completion follows feedback and can take longer due to load, endpoint braking, gains and CAN quantization. Physical verification remains outstanding.

   Switching unit selectors converts the existing number and preserves the physical request. The MIT preview shows an initial velocity command; TX shows actual control frames. Editing the physical target or gains requires saving again.
5. Click **SANG MANUAL JOG** to change control modes. The previous motion receives a stop request; an enabled motor must report stopped feedback before the mode switch completes. The stopped position becomes software zero. Saved position targets are cleared; no persistent calibration or motor mode register is written. In **MANUAL JOG**, enter a signed speed within ±min(30, VMAX) rad/s, or equivalent rpm. Click **CHẠY TỐC ĐỘ ĐÃ NHẬP** once to use its sign directly, or click **CHẠY JOG +/−** to select the direction using the entered magnitude. The motor continues running until **DỪNG JOG**, a zero-speed run request, a control-mode switch, Disable, disconnect or application close. Mouse release, focus changes and changing view tabs do not stop Jog. The requested speed is sent immediately on the next CAN cycle; the former 1 rad/s² software acceleration/deceleration ramp is removed. Stop likewise sends velocity zero immediately, with Kd damping and no position Kp recoil. Physical acceleration/braking still depend on the drive and load.
6. **Disable motor** cancels queued commands and stops only that motor. **DISABLE TẤT CẢ**, disconnect and window close request Disable for every verified motor. These actions remove active holding torque. A motor with failed verification receives no guessed Enable/Disable/movement frame.

Each panel displays its feedback position relative to its own zero, velocity, torque, temperatures, driver fault code, firmware and active ranges. **Đọc tham số** opens that motor's register table while it is disabled. Register reads are blocked on an enabled axis so long parameter reads cannot interrupt its command stream; other axes continue running.

The three numeric displays suppress only values within one CAN quantization step of zero. This display adjustment is not applied to control feedback; a stopped motor can still produce measurable torque. Status queries (`0x7FF`, payload `CAN_ID 00 CC 00 00 00 00 00`) refresh disabled axes at 4 Hz. Readings older than 0.6 seconds become **—**, rather than retaining stale speed/torque. RX displays the actual feedback bytes; double-click RX to copy the frame to the clipboard and log.

The shared connection settings default from `.env` (`DAMIAO_CAN_CHANNEL`, `DAMIAO_CAN_BITRATE`, `DAMIAO_CAN_SERIAL_BAUDRATE`). **Position and Manual Jog accept ±min(30, driver VMAX) rad/s**. The old `DAMIAO_MULTI_SPEED_CAP_RAD_S` setting and legacy `speed_cap` argument do not reduce these requested speeds. PMAX remains the MIT **wire encoding scale**, and is not enlarged or rewritten; software multi-turn targets do not have this limit. MIT mode does not guarantee measured velocity under every load. Physical motor verification remains outstanding.

Active axes default to 100 command frames/s; set `DAMIAO_MULTI_COMMAND_RATE_HZ` before launch to change this. Driver faults, 1 second of lost feedback and three measured overspeed samples above `max(2 * requested speed, requested speed + 0.15 rad/s)` still stop an axis. There is no elapsed-reference tracking-error cutoff. Target timeout is `max(configured timeout, 2 * ideal duration + 5 seconds)`. If the connection is lost, Disable delivery cannot be confirmed. Error details appear in the panel and log.

Multi-turn position tracking accumulates measured position differences. Rollover candidates are one shaft turn (`2π`) and the CAN position interval (`2 * PMAX`); the tracker uses feedback velocity/timestamps to identify the observed rollover period. Verification accepts XOUT/feedback positions equivalent modulo those periods. Counting requires fresh continuous feedback; rotation while powered off, lost data spanning several turns, or reconnecting cannot be assumed to preserve the previous software turn count.

The tracker now unwraps only an observed position discontinuity within a supported interval. Velocity prediction alone cannot invent an extra turn after delayed feedback. Each motor has its own velocity trim integrator: while measured speed is near the reference, it integrates the measured speed error (gain 1/s), caps correction at ±2 rad/s and respects VMAX. Direction changes and Stop reset the correction; startup/limit conditions do not wind up the integrator. This reduces steady drag-related speed offset, but instantaneous sensor readings can still fluctuate and CAN encoding has finite resolution. Plots/export preserve those measured values.

## View tabs and live plots

**ĐIỀU KHIỂN** retains all existing controls. **ĐỒ THỊ POS / VEL / TOR** has three independent live traces for each configured CAN ID: accumulated position relative to the current software zero (rad), velocity (rad/s), and torque (Nm). A selectable 10/30/60/120 second window, display pause/resume and clear-history controls affect plots only. Feedback collection continues in either tab and while plotting is paused. Records are bounded to 10,000 samples per motor; plots redraw at 5 Hz using reduced points, separately from the CAN workers. Feedback gaps are shown as breaks rather than connecting stale measurements.

Changing tabs does not recreate motor panels, reset input values/units/zeros, switch Position/Jog modes, Enable/Disable motors or change queued commands. Running Jog/Position continues. Stop-all and connection controls remain available above both tabs.

## Save feedback by motor

On the Plots tab, choose a CAN ID beside **LƯU MOTOR** and click **LƯU ẢNH PNG**, **LƯU EXCEL**, or **LƯU CẢ HAI**. A snapshot of all retained records for that motor is saved under `motor_controller/motor_feedback/`, rather than only the visible time window. Names use `motor_feedback_motor_0x01_YYYY-MM-DD_HH-MM-SS_microseconds.png/.xlsx` in **Asia/Saigon (UTC+07)**. Repeated saves do not overwrite previous files.

The image has three time plots. The workbook has editable native charts and a flat Measurements table with local timestamp, elapsed seconds, pos rad/deg, measured/reference/CAN velocity, velocity error, torque, raw position, driver status, RX hex and target angle. Degree conversion and velocity error remain Excel formulas. Raw readings are preserved; the dashed velocity line represents the reference. Export runs from a copied snapshot in a background process and does not pause CAN workers or reset controls/history.

The exporter uses the bundled `@oai/artifact-tool` Node runtime available with this Codex installation, automatically found at `~/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/`. For another installation, set `DAMIAO_EXPORT_NODE` to its Node executable and `DAMIAO_EXPORT_MODULES` to that runtime's `node_modules` directory. Export failure is shown on the tab and does not stop motors.

The controller reuses `single_motor.damiao_v12.DamiaoMotor`. Feedback decoding has been corrected to **unsigned offset-binary mapping**: `value = raw * (2 * range) / (2**bits - 1) - range`, with 16 bits for position and 12 for velocity/torque. This matches the [Damiao Python library's feedback decoder](https://github.com/cmjang/DM_Control_Python/blob/main/DM_CAN.py). Reading those fields as two's-complement signed integers turns the midpoint near zero into alternating near-maximum positive/negative readings and corrupts position references. Literal midpoint/end-point frame tests cover this regression. The controller supports MIT (driver mode 1) and Position-Velocity (driver mode 2); the Position/Manual Jog buttons select application control panels and do not rewrite the driver's mode. Speed-only and hybrid driver modes are rejected during verification.

Files:

- `can_router.py`: one adapter receiver, routes by Master ID and checks payload motor identity; separate register and feedback inboxes preserve received faults.
- `controller.py`: one serialized command worker and periodic stream per motor, with independent configuration, zero, motion and timeout state.
- `trajectory.py`: nominal duration/angle calculations for the preview; completion uses measured position.
- `position_tracker.py`: accumulated feedback rotation across supported rollover intervals.
- `velocity_loop.py`: independent measured-speed correction with anti-windup and Stop reset.
- `export_feedback.py` / `.mjs`: timestamped PNG/XLSX snapshot exports.
- `plots.py`: bounded telemetry histories and native Tk plots; no CAN writes or controller commands.
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
