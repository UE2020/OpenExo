# Release Notes

## Controller Editor and Recorded Controller Settings

### Changed
- Python GUI controller editing shows all properties together, with current device
  values and one Apply for multiple changes. Saved attempted values no longer
  override device truth. Bilateral editing displays differing values on both sides.
- Apply sends acknowledged, sequential single-property updates using the existing
  firmware protocol. Rejection or timeout stops unsent properties and reports
  partial completion; the update is not atomic.
- Trial CSVs include per-joint active controller IDs, confirmation status, and all
  catalog controller properties, including PID gains. Values follow confirmed
  changes during recording; inactive or unknown values remain blank.

### Firmware
- New live-controller readback (`Q`/`p`, UART `0x1C`/`0x1D`) returns authoritative
  active controller settings with float32 precision and lossless BLE chunking.
  SD-default handshake values are no longer treated as live settings.
- Flash both Teensy and Nano for live values. Existing firmware can still receive
  property updates, but CSV settings stay unknown without readback support.

## Arm Configuration and arm_1/arm_2 Joints

### Added
- New arm exoskeleton configuration `bilateralArm` that supports bilateral `arm_1` and `arm_2` joints.
- New joint types `arm_1` and `arm_2` with full data/model integration across parsing, configuration, and runtime.
- New controller mappings for arm joints with `constantTorque` and `spline` (and `disabled`).
- New arm controller parameter CSVs under `SDCard/arm1Controllers` and `SDCard/arm2Controllers`.

### Configuration
- `SDCard/config.ini` now supports arm joints and settings:
  - `arm_1`, `arm_2`
  - `arm_1GearRatio`, `arm_2GearRatio`
  - `arm_1DefaultController`, `arm_2DefaultController`
  - `arm_1UseTorqueSensor`, `arm_2UseTorqueSensor`
  - `arm_1FlipMotorDir`, `arm_2FlipMotorDir`
  - `arm_1FlipTorqueDir`, `arm_2FlipTorqueDir`
  - `arm_1FlipAngleDir`, `arm_2FlipAngleDir`
  - `leftArm1RoM`, `rightArm1RoM`, `leftArm2RoM`, `rightArm2RoM`
  - `leftArm1TorqueOffset`, `rightArm1TorqueOffset`, `leftArm2TorqueOffset`, `rightArm2TorqueOffset`

### CAN IDs
- `left_arm_1` = 80
- `right_arm_1` = 48
- `left_arm_2` = 192
- `right_arm_2` = 160

### Notes
- Arm joints use the same motor types and CAN motor selection as existing joints.
- If your hardware limits the number of motors per side, ensure board pin availability and update any board limits as needed.
