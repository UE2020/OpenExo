# Ankle calibration freeze and PJMC jitter investigation

Investigated September 11, 2026 against the local OpenExo source, both SD-card copies, and saved GUI logs/trial records. The user confirmed the affected button is **Recalibrate Torque Sensor** on the Python GUI trial page, and that walking jitter disappears with **Use PID = 0**.

The user subsequently confirmed that **SDCard_asfound_20260825 is the SD-card copy in use**. The calibration defect is reproduced from the production C++ functions. The large PJMC command discontinuities reproduced with the other SD-card copy do not explain the reported jitter with the confirmed settings: derivative feedback and gain scheduling are disabled. The walking jitter's root cause remains unresolved. No firmware, GUI implementation, or SD-card parameters were changed. Nothing was flashed or operated on hardware.

**Later hardware-test update:** the user did not reproduce the calibration freeze on September 11. Today's saved recording confirms live data continued after the recalibration request. The isolated source test establishes a missing status-restoration path, not an unconditional freeze on the running device. Ongoing FSR calibration/refinement can overwrite the global status and mask that path (`Side.cpp:192` onward); the user requested FSR calibration at 15:18:51.482, then torque calibration at 15:18:56.613. Whether that mechanism explains this particular non-reproduction is not proven because the recording omits the relevant status/flag history. The user reports both boards were flashed from this local source at the end of August; do not assume an older incompatible firmware as the cause of current behavior.

The follow-up investigation of **Zero Torque, motor IDs, and missing update ACKs** is in `zero_torque_and_ack_findings.md`. Unlike the walking jitter, the Zero Torque flat measurement has a direct source explanation corroborated by today's unilateral and bilateral recordings.

## 1. Recalibration leaves telemetry disabled

Both GUI buttons call `QtExoDeviceManager.calibrateTorque()` and asynchronously queue BLE command `H`. The trial-page handler does not wait for the BLE operation. The starting page additionally starts a three-second UI timer, then permits Start Trial. That timer is not a device calibration-completion acknowledgement.

The firmware sequence is:

1. `ble_commands.h:236` forwards the torque calibration request through UART.
2. `uart_commands.h:326` calls `ExoData::start_pretrial_cal()`, setting each used joint's calibration flag.
3. `_Joint::check_calibration()` in `Joint.cpp:99` sets the global status to `torque_calibration` while a sensor is calibrating.
4. `TorqueSensor::calibrate()` in `TorqueSensor.cpp:43` completes its approximately one-second acquisition and clears the sensor flag. No matching operation restores the global trial status.
5. `Exo::run()` in `Exo.cpp:108` sends live telemetry only for `trial_on`, `fsr_calibration`, `fsr_refinement`, or `error`. The device remains in `torque_calibration`, so live plots and CSV recording stop receiving samples.

On the starting page this is masked: the subsequent Start Trial command changes the status, allowing telemetry again. During a trial there is no corresponding start command after recalibration.

**Saved-session corroboration:**

| Session | Calibration command sent | Last CSV sample (same local clock, UTC−4) | Later GUI activity |
|---|---|---|---|
| July 27, 2026 | 13:20:24.294 | 13:20:24.352407 | Commands submitted at 13:20:38 and trial shutdown at 13:20:58 |
| July 28, 2026 | 16:10:23.808 | 16:10:23.867805 | Shutdown writes completed at 16:10:41 |

Sources: `Python_GUI/Saved_Data/logs/device_manager_20260727_131703.log`, `device_manager_20260728_160746.log`, and matching `trial_20260727_131733.csv`, `trial_20260728_160813.csv`. These historical logs ran an earlier GUI revision, so they corroborate the symptom rather than establish the exact firmware version then installed. Continued command handling supports a frozen telemetry display rather than a fully deadlocked Qt event loop.

**Recommended correction:** give torque calibration an explicit lifecycle, managed across both sides and all used joints. Capture the prior operating state once, wait for all requested sensors to finish, and leave calibration status through a defined completion path. A stop, disconnect/reset, or error arriving during calibration must take precedence over restoring an old state. Report completion/failure to the GUI rather than relying on a fixed timer.

Calibration also needs an explicit motor-off condition and unloaded sensors. The current `H` handler does not disable motors, and the CAN send path gates on `motor.enabled`, not calibration status. Entering calibration therefore does not by itself guarantee torque output is off. A correction should preserve this distinction and require an explicit resume after motors have been disabled; merely adding `torque_calibration` to the telemetry allowlist would leave the state and calibration-load problems unresolved.

## 2. PJMC feedback can produce abrupt commands

There are two materially different parameter files at the repository root. The user confirmed the dated `SDCard_asfound_20260825` copy is in use. Treat its values as the baseline for this investigation; any subsequent GUI parameter changes would be overrides of that baseline.

| Parameter | `SDCard/ankleControllers/PJMC.csv`, row 6 | `SDCard_asfound_20260825/ankleControllers/PJMC.csv`, row 6 |
|---|---:|---:|
| Maximum stance torque (Nm) | 0 | 10 |
| Swing setpoint (Nm) | 0 | 0 |
| Assistance | 1 | 1 |
| Use PID | 1 | 1 |
| P | 6 | 0.5 |
| I | 0 | 0 |
| D | 0.03 | 0 |
| Torque alpha | 1 | 0.5 |
| Gain scheduling | 1 | 0 |
| P near zero | 3 | 0 (unused) |
| I near zero | 0 | 0 (unused) |
| D near zero | 0.001 | 0 (unused) |

The code's parameter indices match these twelve columns (`ControllerData.h:34`). This is not an evident CSV column-order mismatch. With the confirmed dated copy, Use PID=1 enables proportional feedback only: `command = target + 0.5 * (target - filtered_measured_torque)`. Torque alpha=0.5 smooths the measured signal. With Use PID=0, the FSR-driven feedforward target remains, including the 10 Nm stance scaling. The user's observation that jitter disappears therefore implicates the added torque feedback or its interaction with the hardware, but does not establish a specific fault in that path.

The following A–C reproductions describe the **other, inactive SDCard profile**, and are retained as separate source findings rather than explanations for this device's reported jitter.

**A. Abrupt gain switching, when gain scheduling is enabled.** In `Controller.cpp:671`, reduced gains apply only when absolute setpoint is at most 0.5 Nm AND absolute error is at most 3.5 Nm. There is no hysteresis or blending. With the main SD file, at zero setpoint, a settled measurement changing from 3.49 to 3.51 Nm changes the command from −10.47 to −21.06 Nm. A 0.02 Nm measurement change produces a **10.59 Nm controller-command step**, before gearing and current limits. This reproduction isolates the proportional gain switch by keeping derivative contribution zero. Repeated crossings can create chatter. A similar switch is possible at the 0.5 Nm setpoint boundary.

**B. Unfiltered torque plus derivative feedback, when D is nonzero.** `Controller.cpp:609` uses the configured alpha, and `Utilities.cpp:312` implements `old + alpha*(new-old)`. Alpha=1 therefore means no torque smoothing. The derivative in `_pid()` is measurement change divided by elapsed seconds. At the nominal 500 Hz loop rate, a 0.1 Nm sample change gives a 1.5 Nm derivative contribution with D=0.03. The setpoint and final-command EWMAs also use alpha=1 (`Controller.cpp:646`, `714`), so those do not smooth command changes either. This arithmetic illustrates sensitivity, not a measurement of actual sensor noise.

**C. Derivative dropout on slightly late cycles.** `_pid()` in `Controller.cpp:214` accepts derivative timing only up to nominal interval × 1.1: 2,200 microseconds with this configuration. Beyond that, it sets derivative contribution to zero while still updating previous measurement. For the same 0.1 Nm measurement increment, the reproduced command is −25.5 Nm at 2,000 microseconds and −24 Nm at 2,210 microseconds. Timing variation could therefore modulate damping abruptly. Actual device loop intervals have not been measured.

The confirmed dated copy has D=0 and gain scheduling=0, so A and C cannot account for its normal PJMC operation; its alpha=0.5 also excludes B's unfiltered derivative mechanism. The same settled 3.49→3.51 Nm input yields −1.745→−1.755 Nm with that file at zero target, as expected from P=0.5. Investigate feedback sign, calibration offset, sensor noise/filter delay, torque tracking, transmission dynamics, and saturation rather than attributing the symptom to disabled features. I=0 also excludes accumulated integral feedback as an explanation with these settings.

The latest trial CSV headers contain desired/measured torque and gait signals, but not a complete active parameter snapshot, actual PID terms, or per-control-cycle timing. Their GUI receipt times are visibly batched. They cannot establish high-frequency loop timing or uniquely identify the jitter source. August 31 logs show Use PID updates for PJMC on ankle IDs 65/33, but do not establish all active gains.

**Recommended verification for the confirmed configuration:** use P=0.5, I=0, D=0, torque alpha=0.5, and gain scheduling=0 as the baseline. Check for GUI overrides when collecting new diagnostic records. Test feedback direction and zero offsets with the device unloaded and supported, then compare proportional feedback enabled/disabled while preserving the same target. Record raw/filtered torque, target, proportional correction, final command, loop interval, and saturation at the control rate. Examine target transitions as well: setpoint and final-command filtering both use alpha=1, so proportional feedback can amplify an abrupt target change. This is a candidate mechanism, not a confirmed explanation of the walking jitter. Do not treat arbitrary gain reductions as validated walking settings. Fixing gain scheduling or derivative dropout alone would not address the confirmed profile.

## Reproduction and limits

`reproduce.py` extracts the actual production calibration, status, EWMA, PID, and PJMC function bodies and compiles them with clang++ using fake sensor/time/data objects. It also extracts the telemetry allowlist and parameter definitions. It verifies calibration finishes while telemetry stays disabled, verifies a new trial status restores telemetry, and quantifies the PJMC command transitions for both SD-card copies. Output is saved in `results.txt`.

All source-behavior assertions passed. `inspect_records.py` reads existing records without changing them. These tests do not emulate BLE, the complete firmware scheduler, motor electronics, or the mechanical plant, and are not a firmware build or hardware validation. The numerical commands above are controller outputs in joint torque units before downstream gearing/current saturation, not measured delivered torque.
