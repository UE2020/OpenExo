-To run the GUI, first ensure that Python is installed on your computer. You can find more information here: https://www.python.org/downloads/

-Once Python is installed, open a terminal in the Python_GUI folder and run python GUI_install_dependencies.py. Wait until all dependencies are installed.

-Finally, to start the GUI, run python GUI.py. Saved exoskeleton datastreams will be located in the Saved_Data folder.

Controller settings
-------------------
The Update Controller page shows every property for the selected joint/controller,
the device's current value, and an editable new value. The in-use controller is
shown above the table. Reopening the page requests fresh settings; saved attempted
values are never restored as device values. Only the joint/controller selection
and bilateral checkbox are remembered.

Edit multiple properties and press Apply once. The GUI sends the existing
single-property firmware command sequentially, waiting for each acknowledgement.
This is not an atomic transaction: a rejection or timeout stops remaining changes,
but already accepted changes stay applied. The status reports partial completion.
Pause the exo before changing settings if intermediate configurations are unsafe.
Bilateral mode shows both sides' values and requires matching controller metadata;
it applies the selected side's new values to both sides, including existing differences.
Unknown fields are not sent until explicitly edited. Controllers not currently in
use have unknown properties until selected/applied and read back from the device.
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
Flash both the Teensy and Nano firmware from this revision for live settings.
Bulk editing itself uses the existing f command and its a acknowledgement. Accurate
readback adds Q (joint query), p (unscaled float32 snapshot), and UART commands
0x1C/0x1D. The existing handshake v rows are SD defaults, not live controller state.
The new UART reply preserves small PID gains and large parameter values instead
of using the telemetry fixed-point encoding. Long replies use 19-byte BLE chunks.
Older firmware can still accept parameter changes, but readback will time out and
CSV settings remain unknown. See Documentation/BUILD_AND_FLASH.md.

Hardware-free regression commands (run from the repository root):
  python tests/test_controller_settings.py
  python tests/test_ack_parser.py