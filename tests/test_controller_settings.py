"""Hardware-free regressions using real Qt widgets and the production parser.

Run separately from the legacy stub-based parser tests:
    python tests/test_controller_settings.py

Preferences, device files, and recordings are isolated in a temporary directory.
"""
import csv
import io
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "Python_GUI"))
from PySide6 import QtCore, QtWidgets
import MainWindow as window_module
from pages import ActiveTrialSettingsPage
from services import QtExoDeviceManager, RtBridge
from utils import SettingsManager


MATRIX = [
    ["Ankle(L)", "65", "PID", "2", "kp", "ki", "kd"],
    ["Ankle(L)", "65", "Constant", "3", "torque"],
    ["Ankle(R)", "33", "PID", "2", "kp", "ki", "kd"],
    ["Ankle(R)", "33", "Constant", "3", "torque"],
]


def snapshot(joint, controller, values):
    tokens = [joint, controller, *values]
    return ("Sp%dc" % len(tokens) + "".join("%sn" % value for value in tokens)).encode("ascii")


def streaming_bridge():
    bridge = RtBridge()
    bridge._handshake = True
    bridge._collecting_names = False
    bridge._controllers_done = True
    bridge._collecting_handshake_payload = False
    return bridge


class Transport(QtCore.QObject):
    """Only replace the hardware boundary, not MainWindow orchestration."""
    connected = QtCore.Signal(str, str)
    disconnected = QtCore.Signal()
    error = QtCore.Signal(str)
    log = QtCore.Signal(str)
    dataReceived = QtCore.Signal(bytes)
    scanResults = QtCore.Signal(list)
    scanProgress = QtCore.Signal(int)
    connectScanProgress = QtCore.Signal(int)
    connectionProgress = QtCore.Signal(int)
    trialStarted = QtCore.Signal()
    build_parameter_updates = staticmethod(QtExoDeviceManager.build_parameter_updates)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.updates = []
        self.queries = []
        self.writes = []

    def updateTorqueValues(self, payload):
        self.updates.append(list(payload))
        return True

    def requestControllerValues(self, joint):
        self.queries.append(joint)
        return True

    def write(self, data):
        self.writes.append(data)
        return True

    def get_log_file_path(self):
        return None

    def disconnect(self):
        pass
    def beginTrial(self):
        # Completion is controlled by the test, like an asynchronous BLE write.
        return True



class IsolatedQtTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.base = Path(self.directory.name)
        self.settings = self.base / "gui_settings.txt"
        self.settings_patch = patch.object(
            SettingsManager, "get_settings_path",
            side_effect=lambda filename=None: str(self.base / (filename or "gui_settings.txt")),
        )
        self.settings_patch.start()
        self.addCleanup(self.settings_patch.stop)
        # Timeouts are exercised explicitly; no wall-clock sleeps or callbacks
        # retaining deleted windows after a test has completed.
        self.timer_patch = patch.object(QtCore.QTimer, "singleShot", side_effect=lambda *args: None)
        self.timer_patch.start()
        self.addCleanup(self.timer_patch.stop)
        self.widgets = []
        self.addCleanup(self.destroy_widgets)

    def destroy_widgets(self):
        for widget in self.widgets:
            widget.close()
            widget.deleteLater()
        QtCore.QCoreApplication.sendPostedEvents(None, QtCore.QEvent.DeferredDelete)

    def page(self):
        page = ActiveTrialSettingsPage()
        self.widgets.append(page)
        page.set_controller_matrix(MATRIX)
        page.set_active_controllers({"65": "2", "33": "2"})
        return page

    def window(self):
        with patch.object(window_module, "QtExoDeviceManager", Transport), patch.object(
            window_module.ScanWindowQt, "_load_saved_device", return_value=None
        ):
            window = window_module.MainWindow()
        window.scan_page.SETTINGS_FILE = str(self.base / "saved_device.txt")
        self.widgets.append(window)
        self.addCleanup(window._stop_recording)
        window._on_controller_matrix(MATRIX)
        window.qt_dev.connected.emit("Test device", "test")
        bridge = window.rt_bridge
        bridge._handshake = True
        bridge._collecting_names = False
        bridge._controllers_done = True
        bridge._collecting_handshake_payload = False
        return window

    def readback(self, window, left=(2, [1, .01, .5]), right=(2, [2, .02, .6])):
        window.rt_bridge.feed_bytes(snapshot(65, *left))
        window.rt_bridge.feed_bytes(snapshot(33, *right))

    def acknowledged_window(self):
        window = self.window()
        window._on_controller_values([["65", "2", "999", "999", "999"]])
        self.readback(window)
        return window

    def ack(self, window, joint=65, index=0, accepted=True, reason=0, controller=2):
        window.rt_bridge.feed_bytes(
            (f"Sa5c{joint * 100}n{controller * 100}n{index * 100}n{100 if accepted else 0}n{reason * 100}n").encode()
        )


class EditorTests(IsolatedQtTest):
    def test_all_properties_edit_in_one_apply_and_reopen_uses_device_cache(self):
        SettingsManager.update_settings({"last_joint": "65", "last_controller": "2", "last_value": "999"})
        page = self.page()
        page.set_controller_values({("65", "2"): [1, .01, .5]})
        emitted = []
        page.applyRequested.connect(emitted.append)
        for editor, value in zip(page._param_editors, [3, .02, .7]):
            editor.setValue(value)
        page.btn_apply.click()
        self.assertEqual(emitted, [[[False, 65, 2, 0, 3.0], [False, 65, 2, 1, .02], [False, 65, 2, 2, .7]]])
        # An attempted Apply is not truth; reopening restores only device values.
        page.set_controller_matrix(MATRIX)
        self.assertEqual([editor.value() for editor in page._param_editors], [1, .01, .5])
        self.assertNotIn("last_value", SettingsManager.load_settings())

    def test_bilateral_untouched_differences_remain_and_explicit_edits_affect_both(self):
        page = self.page()
        page.set_controller_values({("65", "2"): [1, .01, .5], ("33", "2"): [2, .01, .8]})
        emitted = []
        page.applyRequested.connect(emitted.append)
        page.chk_bilateral.setChecked(True)
        self.assertIn("2", page.table.item(0, 1).text())
        self.assertIn("1", page.table.item(0, 1).text())
        page.btn_apply.click()
        self.assertEqual(emitted, [])
        page._param_editors[0].setValue(3)
        page.btn_apply.click()
        self.assertEqual(emitted, [[[True, 65, 2, 0, 3.0]]])
        page._param_editors[2].lineEdit().textEdited.emit("0.5")
        page.btn_apply.click()
        self.assertEqual(emitted[-1], [[True, 65, 2, 0, 3.0], [False, 33, 2, 2, .5]])

    def test_unknown_properties_require_explicit_edit_including_zero(self):
        page = self.page()
        page.set_controller_values({("65", "2"): [1, None, None]})
        emitted = []
        page.applyRequested.connect(emitted.append)
        page._param_editors[0].setValue(4)
        page.btn_apply.click()
        self.assertEqual(emitted[-1], [[False, 65, 2, 0, 4.0]])
        page._param_editors[1].lineEdit().setText("0")
        page._param_editors[1].lineEdit().textEdited.emit("0")
        page.btn_apply.click()
        self.assertEqual(emitted[-1], [[False, 65, 2, 0, 4.0], [False, 65, 2, 1, 0.0]])


class ModePresetEditorTests(IsolatedQtTest):
    ZERO_FIELDS = ["use_pid", "p_gain", "i_gain", "d_gain"]
    PJMC_FIELDS = ["stance_max", "swing_max", "is_assistance", "use_pid", "p_gain",
                   "i_gain", "d_gain", "torque_alpha", "GS_Flag", "kp_zero",
                   "ki_zero", "kd_zero"]
    ZERO = [1, 6.5, .1, .4]
    PJMC = [25, 3, 1, 1, 8, .01, .3, .2, 0, 2, .02, .1]

    def mode_page(self):
        page = ActiveTrialSettingsPage()
        self.widgets.append(page)
        matrix = [
            [name, joint, controller, cid, *fields]
            for name, joint in (("Ankle(L)", "65"), ("Ankle(R)", "33"))
            for controller, cid, fields in (("zeroTorqu", "2", self.ZERO_FIELDS),
                                           ("PJMC", "3", self.PJMC_FIELDS))
        ]
        page._last_selection.update(joint="65", controller="2")
        page.set_controller_matrix(matrix)
        page.set_active_controllers({"65": "2", "33": "2"})
        page.set_controller_values({("65", "2"): self.ZERO, ("33", "2"): self.ZERO})
        defaults = {(joint, "3"): self.PJMC for joint in ("65", "33")}
        page.set_controller_presets({("65", "2"): self.ZERO, ("33", "2"): self.ZERO}, defaults)
        emitted = []
        page.applyRequested.connect(emitted.append)
        return page, emitted, matrix, defaults

    def select_mode(self, page, name):
        page.combo_controller.setCurrentIndex(page.combo_controller.findText(name))

    def values(self, page):
        return [editor.value() for editor in page._param_editors]

    def test_initial_pjmc_uses_complete_sd_preset_without_arbitrary_edit(self):
        page, emitted, _, _ = self.mode_page()
        self.select_mode(page, "PJMC")
        self.assertEqual(self.values(page), self.PJMC)
        page._param_editors[1].setValue(4)
        page.btn_apply.click()
        expected = list(self.PJMC)
        expected[1] = 4
        self.assertEqual(emitted, [[[False, 65, 3, i, value]
                                    for i, value in enumerate(expected)]])
        page.set_controller_matrix(page._controller_matrix)
        page.btn_apply.click()
        self.assertEqual(emitted[-1], [[False, 65, 3, i, value]
                                      for i, value in enumerate(self.PJMC)])

    def test_mode_roundtrip_uses_last_confirmed_not_sd_or_inactive_live_cache(self):
        page, emitted, _, defaults = self.mode_page()
        self.select_mode(page, "PJMC")
        used = list(self.PJMC)
        used[0], used[4] = 31, 9.25
        presets = {("65", "2"): self.ZERO, ("65", "3"): used}
        page.set_controller_presets(presets, defaults)
        page.set_active_controllers({"65": "3", "33": "2"})
        page.set_controller_values({("65", "3"): used})
        self.select_mode(page, "zeroTorqu")
        self.assertEqual(self.values(page), self.ZERO)
        page.btn_apply.click()
        self.assertEqual(emitted[-1], [[False, 65, 2, i, value]
                                      for i, value in enumerate(self.ZERO)])
        page.set_active_controllers({"65": "2", "33": "2"})
        page.set_controller_values({("65", "2"): self.ZERO, ("65", "3"): [999] * 12})
        self.select_mode(page, "PJMC")
        self.assertEqual(self.values(page), used)
        page.btn_apply.click()
        self.assertEqual(emitted[-1], [[False, 65, 3, i, value] for i, value in enumerate(used)])
        self.select_mode(page, "zeroTorqu")
        self.assertEqual(self.values(page), self.ZERO)

    def test_live_refresh_preserves_dirty_fields_and_saved_preset_is_not_live(self):
        page, emitted, _, defaults = self.mode_page()
        page._param_editors[1].setValue(7)
        fresh = [1, 6.75, .15, .45]
        page.set_controller_values({("65", "2"): fresh})
        page.set_controller_presets({("65", "2"): fresh}, defaults)
        self.assertEqual(self.values(page), [1, 7, .15, .45])
        self.assertEqual(page.table.item(1, 1).text(), "6.75")
        page.btn_apply.click()
        self.assertEqual(emitted[-1], [[False, 65, 2, 1, 7.0]])
        page.set_controller_values({})
        self.assertEqual(self.values(page), [1, 7, .15, .45])

    def test_cancelled_or_failed_apply_and_reopening_do_not_replace_confirmed_preset(self):
        page, emitted, matrix, defaults = self.mode_page()
        presets = {("65", "2"): self.ZERO, ("65", "3"): self.PJMC}
        page.set_controller_presets(presets, defaults)
        self.select_mode(page, "PJMC")
        page._param_editors[0].setValue(40)
        page.btn_apply.click()
        page.set_update_pending(True)
        page.set_param_update_status("Device rejected update.")
        page.set_update_pending(False)
        page.btn_cancel.click()
        page.set_controller_matrix(matrix)
        self.assertEqual(self.values(page), self.PJMC)
        page.btn_apply.click()
        self.assertEqual(emitted[-1], [[False, 65, 3, i, value]
                                      for i, value in enumerate(self.PJMC)])

    def test_unknown_switch_value_blocks_entire_batch_until_explicitly_entered(self):
        page, emitted, _, defaults = self.mode_page()
        defaults[("65", "3")] = [*self.PJMC[:-1], None]
        page.set_controller_presets({}, defaults)
        self.select_mode(page, "PJMC")
        self.assertEqual(page._param_editors[-1].lineEdit().text(), "")
        page.btn_apply.click()
        self.assertEqual(emitted, [])
        editor = page._param_editors[-1]
        editor.lineEdit().setText("0")
        editor.lineEdit().textEdited.emit("0")
        page.btn_apply.click()
        expected = [*self.PJMC[:-1], 0]
        self.assertEqual(emitted[-1], [[False, 65, 3, i, value]
                                      for i, value in enumerate(expected)])

    def test_bilateral_switch_only_inactive_side_preserves_other_side_live_values(self):
        page, emitted, _, _ = self.mode_page()
        opposite = list(self.PJMC)
        opposite[0] = 41
        page.set_active_controllers({"65": "2", "33": "3"})
        page.set_controller_values({("65", "2"): self.ZERO, ("33", "3"): opposite})
        self.select_mode(page, "PJMC")
        page.chk_bilateral.setChecked(True)
        page.btn_apply.click()
        self.assertEqual(emitted[-1], [[False, 65, 3, i, value]
                                      for i, value in enumerate(self.PJMC)])
        page._param_editors[0].setValue(30)
        page.btn_apply.click()
        self.assertEqual(emitted[-1], [[True, 65, 3, 0, 30.0],
                                      *[[False, 65, 3, i, self.PJMC[i]]
                                        for i in range(1, len(self.PJMC))]])

    def test_bilateral_switch_preserves_distinct_side_presets_and_rejects_unknown_side(self):
        page, emitted, _, defaults = self.mode_page()
        right = list(self.PJMC)
        right[0] = 40
        defaults[("33", "3")] = right
        page.set_controller_presets({}, defaults)
        self.select_mode(page, "PJMC")
        page.chk_bilateral.setChecked(True)
        page.btn_apply.click()
        self.assertEqual(emitted[-1], [[False, 65, 3, 0, 25], [False, 33, 3, 0, 40],
                                      *[[True, 65, 3, i, self.PJMC[i]]
                                        for i in range(1, len(self.PJMC))]])
        defaults[("33", "3")] = [*right[:-1], None]
        page.set_controller_presets({}, defaults)
        page.btn_apply.click()
        self.assertEqual(len(emitted), 1)

    def test_bilateral_schema_mismatch_is_not_a_matching_joint(self):
        page, emitted, matrix, _ = self.mode_page()
        matrix[-1] = [*matrix[-1][:-1], "other_property"]
        page.set_controller_matrix(matrix)
        self.select_mode(page, "PJMC")
        self.assertFalse(page.chk_bilateral.isEnabled())
        page.chk_bilateral.setChecked(True)
        page.btn_apply.click()
        self.assertEqual(emitted, [])

    def test_unknown_active_mode_requires_full_explicit_preset_not_sparse_guess(self):
        page, emitted, _, _ = self.mode_page()
        page.set_active_controllers({})
        self.select_mode(page, "PJMC")
        page._param_editors[0].setValue(30)
        page.btn_apply.click()
        expected = [30, *self.PJMC[1:]]
        self.assertEqual(emitted, [[[False, 65, 3, i, value]
                                    for i, value in enumerate(expected)]])

    def test_untouched_switch_values_keep_confirmed_precision_not_spinbox_rounding(self):
        page, emitted, _, defaults = self.mode_page()
        confirmed = [*self.PJMC]
        confirmed[4] = "9.250000001"
        page.set_controller_presets({("65", "3"): confirmed}, defaults)
        self.select_mode(page, "PJMC")
        page.btn_apply.click()
        self.assertEqual(emitted[-1][4], [False, 65, 3, 4, 9.250000001])


class BatchTests(IsolatedQtTest):
    def test_one_field_at_a_time_matching_ack_advances_bilateral_batch(self):
        window = self.acknowledged_window()
        window._on_apply_settings([[True, 65, 2, 0, 3], [False, 65, 2, 1, .02]])
        self.assertEqual(window.qt_dev.updates, [[False, 65, 2, 0, 3.0]])
        self.ack(window, joint=33)  # A stray/mismatched ACK cannot advance.
        self.assertEqual(len(window.qt_dev.updates), 1)
        self.ack(window)
        self.assertEqual(window.qt_dev.updates[-1], [False, 33, 2, 0, 3.0])
        self.ack(window, joint=33)
        self.assertEqual(window.qt_dev.updates[-1][:4], [False, 65, 2, 1])
        self.ack(window, index=1)
        self.assertEqual(len(window.qt_dev.updates), 3)
        self.assertEqual(window.qt_dev.queries[-1], 65)
        self.readback(window, left=(2, [3, .02, .5]), right=(2, [3, .02, .6]))
        self.assertFalse(window._pending_param_updates)
        self.assertFalse(window.settings_page._update_pending)

    def test_rejection_and_timeout_stop_remaining_properties(self):
        for failure in ("reject", "timeout"):
            with self.subTest(failure=failure):
                window = self.acknowledged_window()
                window._on_apply_settings([[False, 65, 2, i, value] for i, value in enumerate([3, .02, .7])])
                self.ack(window)
                self.assertEqual(len(window.qt_dev.updates), 2)
                if failure == "reject":
                    self.ack(window, index=1, accepted=False, reason=5)
                else:
                    key, records = next(iter(window._pending_param_updates.items()))
                    window._on_param_update_timeout(key, records[0]["token"])
                self.assertEqual(len(window.qt_dev.updates), 2)
                self.assertFalse(window._controller_batch)
                self.assertFalse(window._pending_param_updates)
                # Late ACK cannot send the abandoned third property.
                self.ack(window, index=1)
                self.assertEqual(len(window.qt_dev.updates), 2)
                self.assertNotIn("65", window._confirmed_controller_joints)

    def test_disconnect_clears_batch_and_old_ack_cannot_resume_it(self):
        window = self.acknowledged_window()
        window._on_apply_settings([[False, 65, 2, 0, 3], [False, 65, 2, 1, .02]])
        window._on_dev_disconnected()
        self.ack(window)
        self.assertEqual(len(window.qt_dev.updates), 1)
        self.assertFalse(window._controller_batch)
        self.assertFalse(window._pending_param_updates)
        self.assertFalse(window._active_controllers)


class CsvTests(IsolatedQtTest):
    def test_recording_without_catalog_explicitly_marks_settings_unknown(self):
        window = self.window()
        window._on_controller_matrix([])
        with patch.object(window_module, "__file__", str(self.base / "MainWindow.py")):
            window._start_csv_auto("no_catalog")
        path = Path(window._csv_path_last)
        window._on_rt_update([1, 2, 3, 4])
        window._stop_recording()
        with path.open(newline="") as stream:
            row = next(csv.DictReader(stream))
        self.assertEqual(row["controller_settings_status"], "unknown")

    def test_recording_tracks_full_live_cache_with_fixed_rectangular_schema(self):
        window = self.acknowledged_window()
        # Exercise the real recording file creation, but never Saved_Data in repo.
        with patch.object(window_module, "__file__", str(self.base / "MainWindow.py")):
            window._start_csv_auto("regression")
        path = Path(window._csv_path_last)
        self.assertTrue(path.is_relative_to(self.base))
        window._on_rt_update([1, 2, 3, 4])  # Start-recording readback is not yet complete.
        self.readback(window)
        window._on_rt_update([1, 2, 3, 4])
        window._on_apply_settings([[False, 65, 2, 1, .03]])
        window._on_rt_update([1, 2])  # Pending update must not look confirmed.
        self.ack(window, index=1)
        window._on_rt_update([1, 2, 3, 4, 5, 6])  # ACK alone is not full readback.
        self.readback(window, left=(2, [1, .03, .5]), right=(3, [9]))
        window._on_rt_update([1, 2, 3, 4, 5, 6])
        window._stop_recording()
        with path.open(newline="") as stream:
            rows = list(csv.reader(stream))
        self.assertEqual({len(row) for row in rows}, {len(rows[0])})
        data = [dict(zip(rows[0], row)) for row in rows[1:]]
        for name in ("controller_65_2_0_kp", "controller_65_2_1_ki", "controller_65_2_2_kd", "controller_33_3_0_torque"):
            self.assertIn(name, rows[0])
        self.assertEqual(data[0]["controller_65_status"], "unknown")
        self.assertEqual(data[0]["controller_65_2_1_ki"], "")
        self.assertEqual(data[1]["controller_65_status"], "confirmed")
        self.assertEqual(float(data[1]["controller_65_2_1_ki"]), .01)
        self.assertEqual(data[1]["controller_65_3_0_torque"], "")
        self.assertEqual(data[2]["controller_65_status"], "updating")
        self.assertEqual(data[2]["controller_65_2_1_ki"], "")
        self.assertEqual(data[3]["controller_65_status"], "unknown")
        self.assertEqual(data[4]["controller_65_status"], "confirmed")
        self.assertEqual(float(data[4]["controller_65_2_1_ki"]), .03)
        self.assertEqual(data[4]["controller_33_id"], "3")
        self.assertEqual(data[4]["controller_33_2_0_kp"], "")
        self.assertEqual(float(data[4]["controller_33_3_0_torque"]), 9)
        self.assertEqual(float(data[1]["controller_65_2_0_kp"]), 1)
        self.assertEqual(float(data[1]["controller_65_2_2_kd"]), .5)
        self.assertEqual(data[2]["data2"], "")

    def test_legacy_defaults_and_incomplete_readback_cannot_confirm_csv_settings(self):
        window = self.window()
        window._on_controller_values([["65", "2", "999", "888", "777"]])
        window._csv_controller_columns = window._build_controller_csv_columns()
        output = io.StringIO()
        window._csv_writer = csv.writer(output)
        window.rt_bridge.feed_bytes(snapshot(33, 2, [2, .02, .6]))  # Wrong joint while Q65 is pending.
        window.rt_bridge.feed_bytes(snapshot(65, 2, [1]))  # Wrong property count.
        window._on_rt_update([1, 2, 3, 4])
        token = window._controller_readback_token
        window._on_controller_readback_timeout("65", token)
        window.rt_bridge.feed_bytes(snapshot(33, 2, [2, .02, .6]))
        window._on_rt_update([1, 2, 3, 4])
        rows = list(csv.reader(io.StringIO(output.getvalue())))
        first, second = [dict(zip(rows[0], row)) for row in rows[1:]]
        self.assertEqual(first["controller_65_status"], "unknown")
        self.assertEqual(first["controller_33_status"], "unknown")
        self.assertEqual(second["controller_65_status"], "unknown")
        self.assertEqual(second["controller_65_id"], "")
        self.assertEqual(second["controller_65_2_0_kp"], "")
        self.assertEqual(second["controller_33_status"], "confirmed")
        self.assertEqual(float(second["controller_33_2_1_ki"]), .02)

    def test_trial_start_waits_for_reset_completion_before_confirming_settings(self):
        window = self.acknowledged_window()
        previous_queries = list(window.qt_dev.queries)
        window._go_trial()
        with patch.object(window_module, "__file__", str(self.base / "MainWindow.py")):
            window._start_csv_auto("trial_reset")
        path = Path(window._csv_path_last)
        # An outstanding pre-reset snapshot cannot establish trial settings.
        self.readback(window, left=(2, [99, 88, 77]), right=(2, [66, 55, 44]))
        window._on_rt_update([1, 2, 3, 4])
        self.assertEqual(window.qt_dev.queries, previous_queries)
        window.qt_dev.trialStarted.emit()
        self.assertEqual(window.qt_dev.queries, previous_queries + [65])
        self.readback(window, left=(2, [3, .04, .8]), right=(2, [4, .05, .9]))
        window._on_rt_update([1, 2, 3, 4])
        window._stop_recording()
        with path.open(newline="") as stream:
            rows = list(csv.reader(stream))
        before, after = [dict(zip(rows[0], row)) for row in rows[1:]]
        self.assertEqual(before["controller_65_status"], "unknown")
        self.assertEqual(before["controller_65_2_0_kp"], "")
        self.assertEqual(after["controller_65_status"], "confirmed")
        self.assertEqual(float(after["controller_65_2_0_kp"]), 3)
        self.assertEqual(float(after["controller_65_2_1_ki"]), .04)
        self.assertEqual(float(after["controller_33_2_2_kd"]), .9)


class SnapshotParserTests(IsolatedQtTest):
    def feed(self, chunks, flush=False):
        bridge = streaming_bridge()
        live, rt, acks = [], [], []
        bridge.liveControllerValuesReceived.connect(live.append)
        bridge.rtDataUpdated.connect(rt.append)
        bridge.paramUpdateAckReceived.connect(acks.append)
        for chunk in chunks:
            bridge.feed_bytes(chunk)
        if flush:
            bridge._flush_partial_frame()
        return live, rt, acks

    def test_every_split_and_byte_fragment_preserve_unscaled_precision(self):
        frame = snapshot(65, 2, ["1.23456789e-8", "12345678", "-0.125"])
        expected = [{"joint_id": 65, "controller_id": 2, "values": [1.23456789e-8, 12345678.0, -.125]}]
        for split in range(1, len(frame)):
            with self.subTest(split=split):
                self.assertEqual(self.feed([frame[:split], frame[split:]]), (expected, [], []))
        self.assertEqual(self.feed([bytes([byte]) for byte in frame]), (expected, [], []))

    def test_coalesced_snapshot_telemetry_and_ack_are_distinct(self):
        live, rt, acks = self.feed([snapshot(65, 2, [.001, 10000]) + b"Sd3c100n200n300nSa5c6500n200n0n100n0n"])
        self.assertEqual(live, [{"joint_id": 65, "controller_id": 2, "values": [.001, 10000.0]}])
        self.assertEqual(len(rt), 1)
        self.assertEqual(rt[0][:3], [1, 2, 3])
        self.assertEqual(acks, [{"joint_id": 65, "controller_id": 2, "param_index": 0, "accepted": True, "reason": 0}])
        self.assertEqual(self.feed([snapshot(33, 0, [])])[0], [{"joint_id": 33, "controller_id": 0, "values": []}])

    def test_truncated_snapshot_never_tail_flushes_and_next_frame_recovers(self):
        valid = snapshot(65, 2, [.001, 10000])
        for truncated in (valid[:-1], b"Sp4c65n2n0.001n"):
            with self.subTest(truncated=truncated):
                self.assertEqual(self.feed([truncated], flush=True), ([], [], []))
                self.assertEqual(self.feed([truncated, valid], flush=True)[0], [{"joint_id": 65, "controller_id": 2, "values": [.001, 10000.0]}])

    def test_malformed_snapshots_never_become_telemetry(self):
        for frame in (b"Sp1c65n", b"Sp3c65.5n2n1n", b"Sp3c65n2.5n1n", b"Sp3c65n2nNaNn", b"Sp3c65n2nInfn", b"Sp3c65n2ninvalidn"):
            with self.subTest(frame=frame):
                self.assertEqual(self.feed([frame], flush=True), ([], [], []))


class ReadbackLifecycleTests(IsolatedQtTest):
    def test_catalog_and_connection_can_arrive_in_either_order(self):
        for metadata_first in (True, False):
            with self.subTest(metadata_first=metadata_first):
                window = self.window()
                window._destroy_controller_db()
                window._on_controller_matrix(MATRIX)
                defaults = [["65", "2", "9", "8", "7"], ["65", "3", "6"]]
                if metadata_first:
                    window._on_controller_values(defaults)
                    self.assertEqual(window.qt_dev.queries, [])
                    window.qt_dev.connected.emit("Test device", "test")
                else:
                    window.qt_dev.connected.emit("Test device", "test")
                    self.assertEqual(window.qt_dev.queries, [])
                    window._on_controller_values(defaults)
                self.assertEqual(window.qt_dev.queries, [65])
                self.readback(window)
                self.assertEqual(window._confirmed_controller_joints, {"65", "33"})
                self.assertEqual([spin.value() for spin in window.settings_page._param_editors],
                                 [1, .01, .5])

    def test_new_device_cannot_inherit_previous_device_mode_presets(self):
        window = self.acknowledged_window()
        window._request_controller_values()
        self.readback(window, left=(3, [7]))
        window._on_dev_disconnected()
        window._on_controller_matrix(MATRIX)
        window._on_controller_values([["65", "2", "9", "8", "7"], ["65", "3", "11"]])
        window.qt_dev.connected.emit("Different exo", "different")
        self.readback(window, left=(2, [9, 8, 7]), right=(2, [4, 5, 6]))
        window._on_update_controller()
        self.readback(window, left=(2, [9, 8, 7]), right=(2, [4, 5, 6]))
        page = window.settings_page
        self.assertEqual([spin.value() for spin in page._param_editors], [9, 8, 7])
        page.combo_controller.setCurrentIndex(page.combo_controller.findText("Constant"))
        self.assertEqual(page._param_editors[0].value(), 11)
        with self.assertRaises(ValueError):
            float(page.table.item(0, 1).text())

    def test_refresh_and_mode_roundtrip_preserve_presets_not_csv_truth(self):
        window = self.acknowledged_window()
        window._controller_defaults[("65", "3")] = ["7"]
        window._on_update_controller()
        self.readback(window)
        page = window.settings_page
        page.combo_controller.setCurrentIndex(page.combo_controller.findText("Constant"))
        self.assertEqual(page._param_editors[0].value(), 7)
        page.btn_apply.click()
        self.assertIs(window.stack.currentWidget(), window.trial_page)
        self.ack(window, controller=3)
        self.readback(window, left=(3, [7]))
        window._on_update_controller()
        self.readback(window, left=(3, [7]))
        page.combo_controller.setCurrentIndex(page.combo_controller.findText("PID"))
        self.assertEqual([spin.value() for spin in page._param_editors], [1, .01, .5])
        self.assertNotIn("999", [spin.lineEdit().text() for spin in page._param_editors])
        page.btn_apply.click()
        for index in range(3):
            self.ack(window, index=index)
        self.readback(window)
        self.assertEqual(window._controller_presets[("65", "3")], ["7.0"])
        self.assertEqual([float(value) for value in window._controller_presets[("65", "2")]],
                         [1, .01, .5])
        window._csv_controller_columns = window._build_controller_csv_columns()
        csv_values = dict(zip([column[0] for column in window._csv_controller_columns],
                              window._controller_csv_values()))
        self.assertEqual(csv_values["controller_65_3_0_torque"], "")
        self.assertEqual(float(csv_values["controller_65_2_1_ki"]), .01)

    def test_query_timeout_preserves_graph_recording_and_complete_mode_presets(self):
        window = self.acknowledged_window()
        window._controller_defaults[("65", "3")] = ["7"]
        with patch.object(window_module, "__file__", str(self.base / "MainWindow.py")):
            window._start_csv_auto("timeout_continuity")
        path = Path(window._csv_path_last)
        for joint in ("65", "33"):
            window._on_controller_readback_timeout(joint, window._controller_readback_token)
        page = window.settings_page
        page.combo_controller.setCurrentIndex(page.combo_controller.findText("Constant"))
        self.assertEqual(page._param_editors[0].value(), 7)
        page.btn_apply.click()
        self.assertIs(window.stack.currentWidget(), window.trial_page)
        frames = [b"Sd4c100n1100n2100n3100n",
                  b"Sd4c200n1200n2200n3200n",
                  b"Sd4c300n1300n2300n3300n"]
        window.rt_bridge.feed_bytes(frames[0])
        self.ack(window, controller=3)
        window.rt_bridge.feed_bytes(frames[1])
        self.readback(window, left=(3, [7]))
        window.rt_bridge.feed_bytes(frames[2])
        self.assertEqual(list(window.trial_page.top_meas_vals), [11, 12, 13])
        self.assertEqual(list(window.trial_page.curve_top_meas.getData()[1]), [11, 12, 13])
        window._stop_recording()
        with path.open(newline="") as stream:
            records = list(csv.DictReader(stream))
        self.assertEqual([float(record["data0"]) for record in records], [1, 2, 3])
        self.assertEqual([record["controller_65_status"] for record in records],
                         ["updating", "unknown", "confirmed"])
        self.assertEqual(float(records[-1]["controller_65_3_0_torque"]), 7)


if __name__ == "__main__":
    unittest.main(verbosity=2)
