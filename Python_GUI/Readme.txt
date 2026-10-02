-To run the GUI, first ensure that Python is installed on your computer. You can find more information here: https://www.python.org/downloads/

-Once Python is installed, open a terminal in the Python_GUI folder and run python GUI_install_dependencies.py. Wait until all dependencies are installed.

-Finally, to start the GUI, run python GUI.py. Saved exoskeleton datastreams will be located in the Saved_Data folder.

Controller settings
-------------------
The Update Controller page shows every property for the selected joint/controller,
live device values, editable proposed values, and each proposal's source. The
in-use controller is shown above the table. Proposals use current live settings,
then the last fully confirmed settings for that mode, then its SD-file defaults.
Switching zeroTorque -> PJMC -> zeroTorque restores the original confirmed preset;
unused modes start with their SD preset instead of invented zeros. Presets last
for the current device connection and are cleared on disconnect/device change.
Reopening requests fresh live values without erasing those presets. Saved attempted
edits never become device truth; only selection and bilateral preference persist.

Edit multiple properties and press Apply once. The GUI sends the existing
single-property firmware command sequentially, waiting for each acknowledgement.
This is not an atomic transaction: a rejection or timeout stops remaining changes,
but already accepted changes stay applied. The status reports partial completion.
Pause the exo before changing settings if intermediate configurations are unsafe.
Bilateral mode requires matching controller metadata and displays both sides.
Explicit edits affect both sides; untouched fields preserve each side's own values.
Switching modes sends the complete preset, even if no field was manually edited;
an already in-use mode receives only explicit edits. Missing preset fields must
be entered before switching. Unknown fields are blank, never fabricated zeros.
After Apply, the GUI returns to the trial plots without stopping recording or
clearing the graphs. Readback waits for both connection and catalog readiness.
A timeout marks live settings unconfirmed; it does not diagnose missing firmware
or disable plotting/recording. Complete presets remain usable after a timeout.
Without a controller catalog, the existing raw-ID Basic page remains available.

Recording CSV
-------------
Telemetry retains the epoch, mark, and first ten data-channel columns. Additional
columns identify the active controller and its settings for each joint:
  controller_<joint_id>_id
  controller_<joint_id>_status
  controller_<joint_id>_<controller_id>_<zero_based_index>_<parameter_name>
For example, controller_65_2_1_p_gain records joint 65's zero-torque P gain.
All catalog properties are included, including PID enable/gains, torque values,
filters, and controller-specific settings. Inactive controller columns are blank.
The schema is fixed when recording starts; wait for the catalog before recording.
If no catalog is available, controller_settings_status is explicitly unknown.

Each sample records the latest device-confirmed configuration, so changes made
during a recording appear after acknowledgement and full readback. Status is
confirmed, updating, or unknown; uncertain values are blank, never invented zeros
or SD-file defaults. Readback occurs after handshake, trial startup, opening the
editor, starting recording, and finishing an Apply. Trial startup resets firmware
defaults, so its readback waits for the startup command sequence to complete.
These are host-observed configuration snapshots, not settings synchronized to the
firmware's exact telemetry sampling instant; changes outside this GUI require a
fresh readback (open Update Controller).

Firmware requirement
--------------------
Live readback initially requires both Teensy and Nano firmware from this branch.
If both were flashed from the first controller-editor PR revision, only the Nano
needs reflashing for the communication fixes. Build with the repository's bundled
ArduinoBLE library, not an unmodified separately installed copy.
Bulk editing uses the existing f command and a acknowledgement. Readback adds Q
(joint query), p (unscaled float32 snapshot), and UART commands 0x1C/0x1D. Handshake
v rows are SD defaults, not live state. Small PID gains and large values retain
float32 precision. Live replies, ACKs and telemetry use an ordered FIFO and
negotiated-MTU chunks without the former blocking per-chunk delays. Exhausted BLE
credits defer sends while MCU polling continues; full queues report an error
instead of overwriting queued samples. A failed/congested link can still prevent
delivery. Older firmware can accept edits, but readback may time out and CSV
settings remain unknown. See Documentation/BUILD_AND_FLASH.md.

Hardware-free regression commands (run from the repository root):
  python tests/test_controller_settings.py
  python tests/test_ack_parser.py
With g++ installed, the native queue regression can also be run from Windows cmd:
  g++ -std=c++11 tests/test_ble_tx_queue.cpp -o "%TEMP%/openexo-ble-tx-smoke.exe" && "%TEMP%/openexo-ble-tx-smoke.exe"
These checks use no physical exo; validate continuous torque plots and recording
through repeated mode switches on hardware before using this revision in a study.