# Motor IDs, Zero Torque flat plots, and update acknowledgements

September 11, 2026. This follow-up uses the confirmed `SDCard_asfound_20260825` configuration, local firmware/GUI code, and today's recorded trials. The user reports both boards were flashed from this local source at the end of August. No production files, device settings, or firmware were changed during this investigation.

## Motor identity and plot mapping

`ExoCode/src/ParseIni.h:122` defines the motor/joint IDs. A left-side bit (`0x40`) or right-side bit (`0x20`) is combined with the joint-type bit. These identifiers are passed into `JointData` and `MotorData`; switching controllers does not change them.

| Joint | Left motor ID | Right motor ID |
|---|---:|---:|
| Ankle | 65 (`0x41`) | 33 (`0x21`) |
| Knee | 66 (`0x42`) | 34 (`0x22`) |
| Hip | 68 (`0x44`) | 36 (`0x24`) |

The active SD configuration specifies bilateral ankles and AK60v3 motors. For this configuration, firmware telemetry channels 0/1 are left desired/measured torque and channels 2/3 are right desired/measured torque (`uart_commands.h:410`). The Python GUI places channels 0/1 on top and 2/3 on the bottom (`ActiveTrialPage.py:516`). Thus, on the torque view, **top = left ankle = 65**, **bottom = right ankle = 33**. The alternate plot view shows FSR/stance channels rather than torque.

The AK60v3 CAN command-frame identifier includes the command mode in the upper bits (`Motor.cpp:269`): `(8 << 8) | motor_id`. Consequently, the full extended command-frame IDs are `0x841` for left ankle and `0x821` for right ankle. The node IDs to distinguish the motors remain 65 and 33. Reply matching uses the low byte. These are code/config identities; this investigation did not query the physical motors' programmed IDs.

Controller IDs are a separate numbering system: ankle controller **1 = Disabled**, **2 = Zero Torque**, **3 = PJMC** (`ParseIni.h:179`). The GUI logs confirm the new tests sent controller 2, not Disabled.

## Zero Torque has a stale measurement field

`ZeroTorque::calc_motor_cmd()` (`Controller.cpp:312`) sets desired torque to zero and returns a zero motor command when PID is off. It never refreshes `controller.filtered_torque_reading`. But that is precisely the measurement field sent to the ankle torque plots. Sensor acquisition still updates `joint.torque_reading`; Zero Torque's optional PID even reads that raw field directly. The GUI receives a fresh frame containing an old measurement value on every cycle.

This explains the side-specific symptom: switching one side leaves one measurement frozen; bilateral updates leave both frozen. It is a firmware telemetry-field maintenance bug, not evidence that the motor node disappeared from CAN or that BLE stopped receiving data.

The active `ankleControllers/zeroTorque.csv` row is `0,0,0,0` (Use PID, P, I, D). Today's commands explicitly set controller 2, parameter 0, value 0. Therefore **no assistive motor torque is expected in that mode**. Zero commanded torque does not cancel transmission friction or actively compensate external load. Switching to Zero Torque also does not itself clear `motor.enabled`; controller 1, Disabled, is the branch that does that (`Joint.cpp:1182`). These observations explain flat plots plus loss of assistance. They do not independently prove the physical motor stayed healthy if there was an additional electrical fault.

### Evidence in today's records

The earlier log `device_manager_20260911_150421.log` shows unilateral controller 2 updates to **joint 65** at 15:05:55.887 and 15:07:26.874.

| Recorded window (Eastern) | Samples | Left measured torque | Right measured torque |
|---|---:|---|---|
| 15:05:58–15:06:15 | 1,595 | Exactly −0.10 Nm throughout | 96 distinct values |
| 15:07:29–15:07:55 | 2,445 | Exactly −0.05 Nm throughout | 58 distinct values |

The later log `device_manager_20260911_151751.log` shows bilateral controller 2 updates at 15:19:43.499 and 15:21:47.569, then bilateral PJMC updates at 15:20:10.099 and 15:22:05.276.

| Recorded window (Eastern) | Samples | Left measured torque | Right measured torque |
|---|---:|---|---|
| Zero Torque, 15:19:45–15:20:09 | 2,266 | Exactly −4.90 Nm | Exactly −6.15 Nm |
| PJMC restored, 15:20:12–15:20:32 | 1,932 | 702 distinct values | 737 distinct values |
| Zero Torque, 15:21:49–15:22:04 | 1,403 | Exactly +1.19 Nm | Exactly −3.39 Nm |
| PJMC restored, 15:22:07–15:22:27 | 1,940 | 733 distinct values | 507 distinct values |

Foot-sensor values and device time continue changing during both Zero Torque windows. Desired torque is zero on both sides. This is continued telemetry with stale torque fields, not a full telemetry freeze. The two earlier CSVs are `trial_20260911_150523.csv` and `trial_20260911_150644.csv`; the bilateral windows are in `trial_20260911_151815.csv`.

The C++ host reproduction executes the actual Zero Torque function with raw torque changing from +2 to −8 Nm while the transmitted measurement field stays at −4.9 Nm. Its command remains zero. This matches the recorded pattern without a simulated BLE or motor failure.

### Correction or workaround

The direct correction is to refresh the plotted measurement in Zero Torque, with an explicit choice of raw versus filtered signal, or maintain telemetry measurements independently of controller selection. That requires a firmware code change and flashing to affect the device; a GUI cannot reconstruct a live torque value that the device never transmits.

For a no-flash workaround, **remain in PJMC and set maximum stance torque=0, swing setpoint=0, Use PID=0 on both ankles**. The actual PJMC function then returns zero command while continuing to update its measured-torque field in both stance and swing. This was verified in the host reproduction. Use PID=0 alone is insufficient when stance torque remains 10 Nm: feedforward assistance still exists. Apply/verify settings with motors off because parameter changes are separate transactions. This is zero-output mode, not a tuned friction-compensating zero-torque controller, and has not been tested on the hardware by the assistant.

## Missing update ACKs are a separate problem

Today's logs show no-ACK warnings for Zero Torque and PJMC changes, including changes that visibly took effect in recorded telemetry. The first bilateral Zero Torque update was queued at 15:19:43.499; the GUI reported both missing ACKs at 15:19:53.530–.532, while the intervening CSV already shows the Zero Torque behavior. Thus an ACK timeout does **not** establish that the controller update failed.

The current intended protocol is:

1. GUI sends the parameter request.
2. Teensy applies/rejects it and sends UART `update_controller_param_ack` (`uart_commands.h:587`, `609`).
3. Nano forwards it as BLE command `a` (`ComsMCU.cpp:135`, `342`).
4. `RtBridge.feed_bytes()` parses it and emits a Qt acknowledgement signal.
5. `MainWindow` clears the corresponding pending update, or warns after ten seconds.

Current source includes ACK support on both boards; the user's end-of-August flash history gives no basis for asserting that support is absent. The available logs do not capture raw incoming BLE ACK frames, so they cannot distinguish a lost Teensy-to-Nano reply, failed Nano BLE notification, or GUI parse loss.

**Confirmed GUI defects/limitations:**

- `MainWindow.py:498` renders a timeout as **"Controller update failed: no device acknowledgement"**. The accurate status is **"Controller update unconfirmed: no device acknowledgement"** because a timeout does not prove rejection or non-application. Neither automatic success nor automatic resend is justified by a timeout.
- `RtBridge.py:317` treats each callback as if it contained one whole protocol frame. Tests of the actual parser accept complete ACKs for both sides, but lose or misparse the ACK in 19 of 20 possible two-chunk split positions. Two ACK frames supplied in one chunk yield only the first. These are reproducible parser robustness defects; they do **not** prove that today's callback boundaries had those forms. Qt signal/regex/timer objects were stubbed for these tests; actual BLE delivery was not emulated.

**Next diagnostic step:** capture incoming BLE notification bytes and callback boundaries around a controller change, together with its outgoing request and timeout. If complete `Sa5c...` frames arrive, check GUI parsing and signal handling. If only fragments arrive, repair stream reassembly. If no ACK bytes arrive, investigate the board-to-board return path and Nano notification send result. This capture and the GUI timeout wording/parser fixes can be done without flashing; restoring a firmware reply that never reaches the host would require a different remedy.

## Calibration non-reproduction and remaining jitter

Today's data continued after torque recalibration. The earlier isolated source reproduction omitted concurrent FSR status writes, which can take the system back out of torque-calibration status. The later test requested FSR calibration about five seconds before torque recalibration; FSR refinement waits for steps. This is a plausible masking condition, not a confirmed status trace. The firmware state-lifecycle concern remains, but an unconditional freeze should not be claimed for the installed build.

The walking jitter's mechanical/control root cause remains unresolved. With the confirmed dated profile, gain scheduling, derivative feedback, and integral feedback are disabled. Today's logs additionally show GUI P-gain updates to 0.3, 0.4, and 0.35 later in the session; those requests must be distinguished from the baseline P=0.5 and their application is not established by ACKs. No tuning changes were made during this investigation.

Reproduction files: `reproduce.py`, `reproduce_ack_parser.py`, and `inspect_zero_torque.py`. They create local diagnostic outputs only. No production firmware or GUI edits were made.
