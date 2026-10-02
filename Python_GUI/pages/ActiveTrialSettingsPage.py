import logging
import math
import struct

try:
    from PySide6 import QtCore, QtWidgets
except ImportError as e:
    raise SystemExit("PySide6 is required. Install with: pip install PySide6") from e

from utils import UIConfig, SettingsManager, style_button, style_combo_box, style_spinbox

_logger = logging.getLogger(__name__)


class ActiveTrialSettingsPage(QtWidgets.QWidget):
    """Edit live controller values or explicitly sourced per-mode presets."""

    # Each entry is [isBilateral, joint_id, controller_id, parameter_index, value].
    applyRequested = QtCore.Signal(list)
    cancelRequested = QtCore.Signal()

    _SIDE_LEFT = 0x40
    _SIDE_RIGHT = 0x20
    _SIDE_MASK = _SIDE_LEFT | _SIDE_RIGHT

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("ActiveTrialSettingsPage")
        self._controller_matrix = []
        self._controller_values = {}
        self._controller_presets = {}
        self._controller_defaults = {}
        self._active_controllers = {}
        self._joint_controllers = {}
        self._param_editors = []
        self._edited_parameters = set()
        self._update_pending = False
        self._restoring_selection = False
        self._last_selection = {"bilateral": False, "joint": None, "controller": None}
        self._load_settings()
        self._build_ui()
        self._update_enabled_state()

    def _build_ui(self):
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(UIConfig.MARGIN_PAGE, UIConfig.MARGIN_PAGE,
                                  UIConfig.MARGIN_PAGE, UIConfig.MARGIN_PAGE)
        layout.setSpacing(UIConfig.SPACING_MEDIUM)

        title = QtWidgets.QLabel("Update Controller Settings")
        title.setAlignment(QtCore.Qt.AlignCenter)
        font = title.font()
        font.setPointSize(UIConfig.FONT_SUBTITLE)
        title.setFont(font)
        layout.addWidget(title)

        selectors = QtWidgets.QGridLayout()
        self.combo_joint = QtWidgets.QComboBox()
        self.combo_controller = QtWidgets.QComboBox()
        for combo in (self.combo_joint, self.combo_controller):
            style_combo_box(combo, height=UIConfig.BTN_HEIGHT_SMALL,
                            font_size=UIConfig.FONT_SMALL)
            combo.setSizeAdjustPolicy(QtWidgets.QComboBox.AdjustToMinimumContentsLengthWithIcon)
            combo.setMinimumContentsLength(8)
            combo.setMinimumWidth(0)
        selectors.addWidget(QtWidgets.QLabel("Joint"), 0, 0)
        selectors.addWidget(self.combo_joint, 0, 1)
        selectors.addWidget(QtWidgets.QLabel("Controller"), 0, 2)
        selectors.addWidget(self.combo_controller, 0, 3)
        selectors.setColumnStretch(1, 1)
        selectors.setColumnStretch(3, 1)
        layout.addLayout(selectors)

        self.chk_bilateral = QtWidgets.QCheckBox("Apply to both matching joints")
        self.chk_bilateral.setChecked(self._last_selection["bilateral"])
        layout.addWidget(self.chk_bilateral)
        self.lbl_active_controller = QtWidgets.QLabel("In-use controller: unknown")
        self.lbl_active_controller.setWordWrap(True)
        layout.addWidget(self.lbl_active_controller)

        self.table = QtWidgets.QTableWidget(0, 4)
        self.table.setHorizontalHeaderLabels(
            ["Parameter", "Live device", "Proposed value", "Preset source"])
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        self.table.verticalHeader().hide()
        self.table.verticalHeader().setDefaultSectionSize(UIConfig.TABLE_ROW_HEIGHT)
        header = self.table.horizontalHeader()
        header.setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        header.setMinimumSectionSize(90)
        self.table.setMinimumHeight(80)
        self.table.setWordWrap(True)
        layout.addWidget(self.table, 1)

        self.lbl_param_update_status = QtWidgets.QLabel("")
        self.lbl_param_update_status.setWordWrap(True)
        self.set_param_update_status("")
        layout.addWidget(self.lbl_param_update_status)

        buttons = QtWidgets.QHBoxLayout()
        self.btn_apply = QtWidgets.QPushButton("Apply")
        self.btn_cancel = QtWidgets.QPushButton("Cancel")
        for button in (self.btn_apply, self.btn_cancel):
            style_button(button, height=UIConfig.BTN_HEIGHT_SMALL,
                         width=UIConfig.BTN_WIDTH_SMALL, font_size=UIConfig.FONT_SMALL,
                         padding="4px 12px")
        buttons.addStretch(1)
        buttons.addWidget(self.btn_cancel)
        buttons.addWidget(self.btn_apply)
        layout.addLayout(buttons)

        self.combo_joint.currentIndexChanged.connect(self._on_joint_changed)
        self.combo_controller.currentIndexChanged.connect(self._on_controller_changed)
        self.chk_bilateral.toggled.connect(self._on_bilateral_changed)
        self.btn_apply.clicked.connect(self._on_apply)
        self.btn_cancel.clicked.connect(self.cancelRequested.emit)
        self.btn_apply.setToolTip(
            "Switching modes sends the full preset. In-use controllers receive only "
            "edited changes. Bilateral edits affect both sides; untouched side values "
            "are preserved. Updates are acknowledged in order and are not atomic."
        )

    @staticmethod
    def _metadata_id(raw):
        """Require an explicit integer ID; never infer an ID from a row position."""
        if isinstance(raw, bool) or not isinstance(raw, (str, int)):
            raise ValueError("Expected an integer controller/joint ID")
        value = int(raw)
        if not 0 <= value <= 255:
            raise ValueError("Controller/joint ID is outside the protocol range")
        return value

    def set_controller_matrix(self, matrix: list):
        """Set [joint display, joint ID, controller name, controller ID, parameters...].

        Opening the page discards attempted edits, not confirmed per-mode presets.
        Presets and SD defaults initialize proposed values, never device truth.
        """
        rows = []
        invalid = 0
        for raw in matrix or []:
            try:
                if not isinstance(raw, (list, tuple)) or len(raw) < 5:
                    raise ValueError("Missing controller property metadata")
                joint_id = self._metadata_id(raw[1])
                controller_id = self._metadata_id(raw[3])
                if joint_id == 0 or any(not isinstance(name, str) or not name.strip()
                                        for name in [raw[0], raw[2], *raw[4:]]):
                    raise ValueError("Missing joint/controller/property name")
                rows.append([raw[0], str(joint_id), raw[2], str(controller_id), *raw[4:]])
            except (TypeError, ValueError):
                invalid += 1

        # Duplicate identities cannot safely identify a property schema.
        identities = {}
        for row in rows:
            key = (row[1], row[3])
            identities[key] = identities.get(key, 0) + 1
        self._controller_matrix = [row for row in rows if identities[(row[1], row[3])] == 1]
        invalid += len(rows) - len(self._controller_matrix)
        self._joint_controllers = {}
        for index, row in enumerate(self._controller_matrix):
            self._joint_controllers.setdefault(row[1], []).append(index)

        self._restoring_selection = True
        self.combo_joint.blockSignals(True)
        self.combo_joint.clear()
        for joint_id, indices in self._joint_controllers.items():
            self.combo_joint.addItem(self._controller_matrix[indices[0]][0], joint_id)
        preferred = self._last_selection["joint"]
        index = self.combo_joint.findData(preferred)
        if index < 0 and preferred is not None:
            index = self.combo_joint.findText(str(preferred))  # Old name-only preference.
        if index >= 0:
            self.combo_joint.setCurrentIndex(index)
        self.combo_joint.blockSignals(False)
        self._on_joint_changed(self.combo_joint.currentIndex())
        self._restoring_selection = False
        self._remember_selection()
        if invalid:
            self.set_param_update_status(
                f"Ignored {invalid} invalid or ambiguous controller metadata row(s).")
        else:
            self.set_param_update_status("")

    def set_controller_values(self, values_db: dict):
        """Refresh device truth without replacing unacknowledged user edits."""
        self._controller_values = self._copy_values(values_db)
        self._refresh_values()

    def set_controller_presets(self, presets: dict, defaults: dict):
        """Set confirmed history and SD defaults without treating either as live."""
        self._controller_presets = self._copy_values(presets)
        self._controller_defaults = self._copy_values(defaults)
        self._refresh_values()

    @staticmethod
    def _copy_values(values_db):
        return {
            (str(key[0]), str(key[1])): list(values)
            for key, values in (values_db or {}).items()
            if isinstance(key, tuple) and len(key) == 2 and isinstance(values, (list, tuple))
        }

    def set_active_controllers(self, controllers: dict):
        self._active_controllers = {str(joint): str(controller)
                                    for joint, controller in (controllers or {}).items()}
        self._refresh_active_controller()
        self._refresh_values()

    def _refresh_active_controller(self):
        selected = self._current_row()
        mirror = self._mirror_row() if self.chk_bilateral.isChecked() else None
        descriptions = []
        for side in (selected, mirror):
            if side is None:
                continue
            controller_id = self._active_controllers.get(side[1])
            active = next((row for row in self._controller_matrix
                           if row[1] == side[1] and row[3] == controller_id), None)
            mode = f"{active[2]} ({controller_id})" if active else controller_id or "unknown"
            descriptions.append(f"{side[0]}: {mode}")
        self.lbl_active_controller.setText("In-use controller: " + (
            " | ".join(descriptions) if descriptions else "unknown"))


    def set_update_pending(self, pending: bool):
        self._update_pending = bool(pending)
        self._update_enabled_state()

    def set_param_update_status(self, message: str, warning: bool = True):
        text = message or ""
        color = UIConfig.COLOR_PARAM_REJECT if warning else UIConfig.COLOR_LABEL
        self.lbl_param_update_status.setStyleSheet(
            f"font-size: {UIConfig.FONT_TINY}pt; color: {color}; font-weight: bold;"
        )
        self.lbl_param_update_status.setText(text)
        if text and warning:
            QtCore.QTimer.singleShot(8000, lambda expected=text: self._clear_param_update_status(expected))

    def _clear_param_update_status(self, expected: str):
        if self.lbl_param_update_status.text() == expected:
            self.lbl_param_update_status.setText("")

    def clear_device_session_preferences(self):
        """Forget selections, cached truth and edits when the BLE device changes."""
        self._last_selection = {"bilateral": False, "joint": None, "controller": None}
        self._controller_values = {}
        self._controller_presets.clear()
        self._controller_defaults.clear()
        self._active_controllers.clear()
        self.chk_bilateral.blockSignals(True)
        self.chk_bilateral.setChecked(False)
        self.chk_bilateral.blockSignals(False)
        self._update_pending = False
        self.set_controller_matrix([])
        SettingsManager.purge_keys({"bilateral", "last_joint", "last_controller",
                                    "last_parameter", "last_value"})

    def _load_settings(self):
        try:
            self._last_selection["bilateral"] = SettingsManager.get_bool("bilateral", False)
            for selection, key in (("joint", "last_joint"), ("controller", "last_controller")):
                value = SettingsManager.get_setting(key)
                if value and value != "None":
                    self._last_selection[selection] = value
            # Legacy attempted parameter values must never be restored as truth.
            SettingsManager.purge_keys({"last_parameter", "last_value"})
        except Exception as e:
            _logger.warning("Error loading controller selections: %s", e)

    def _remember_selection(self):
        if self._restoring_selection:
            return
        row = self._current_row()
        if row is not None:
            self._last_selection["joint"] = row[1]
            self._last_selection["controller"] = row[3]
        self._last_selection["bilateral"] = self.chk_bilateral.isChecked()
        try:
            updates = {"bilateral": str(self._last_selection["bilateral"])}
            for selection, key in (("joint", "last_joint"), ("controller", "last_controller")):
                if self._last_selection[selection] is not None:
                    updates[key] = self._last_selection[selection]
            SettingsManager.update_settings(updates)
        except Exception as e:
            _logger.warning("Error saving controller selections: %s", e)

    def _current_row(self):
        index = self.combo_controller.currentData()
        if isinstance(index, int) and 0 <= index < len(self._controller_matrix):
            return self._controller_matrix[index]
        return None

    def _mirror_row(self):
        row = self._current_row()
        if row is None:
            return None
        joint_id = int(row[1])
        side = joint_id & self._SIDE_MASK
        if side not in (self._SIDE_LEFT, self._SIDE_RIGHT):
            return None
        mirror_id = str(joint_id ^ self._SIDE_MASK)
        for candidate in self._controller_matrix:
            if (candidate[1] == mirror_id and candidate[3] == row[3]
                    and candidate[3:] == row[3:]):
                return candidate
        return None

    def _update_enabled_state(self):
        row = self._current_row()
        enabled = not self._update_pending
        self.combo_joint.setEnabled(enabled and self.combo_joint.count() > 0)
        self.combo_controller.setEnabled(enabled and row is not None)
        self.table.setEnabled(enabled and row is not None)
        self.btn_apply.setEnabled(enabled and row is not None and bool(self._param_editors))
        has_mirror = self._mirror_row() is not None
        self.chk_bilateral.setEnabled(enabled and has_mirror)
        self.chk_bilateral.setToolTip(
            "Both sides have matching controller IDs and property schemas."
            if has_mirror else "Unavailable: this controller has no matching opposite-side schema."
        )

    @QtCore.Slot(int)
    def _on_joint_changed(self, _index: int):
        preferred = self._last_selection["controller"]
        self.combo_controller.blockSignals(True)
        self.combo_controller.clear()
        for index in self._joint_controllers.get(self.combo_joint.currentData(), []):
            row = self._controller_matrix[index]
            self.combo_controller.addItem(row[2], index)
        for index in range(self.combo_controller.count()):
            row = self._controller_matrix[self.combo_controller.itemData(index)]
            if preferred in (row[2], row[3]):
                self.combo_controller.setCurrentIndex(index)
                break
        self.combo_controller.blockSignals(False)
        self._on_controller_changed(self.combo_controller.currentIndex())

    @QtCore.Slot(int)
    def _on_controller_changed(self, _index: int):
        self.table.setRowCount(0)
        self._param_editors = []
        self._edited_parameters.clear()
        row = self._current_row()
        if self._mirror_row() is None:
            checked = False
        else:
            checked = self._last_selection["bilateral"]
        self.chk_bilateral.blockSignals(True)
        self.chk_bilateral.setChecked(checked)
        self.chk_bilateral.blockSignals(False)
        if row is not None:
            self.table.setRowCount(len(row) - 4)
            for index, name in enumerate(row[4:]):
                self.table.setItem(index, 0, QtWidgets.QTableWidgetItem(name))
                self.table.setItem(index, 1, QtWidgets.QTableWidgetItem("Unknown"))
                spin = QtWidgets.QDoubleSpinBox()
                spin.setDecimals(8)
                spin.setRange(-3.4028234663852886e38, 3.4028234663852886e38)
                spin.setSingleStep(0.1)
                style_spinbox(spin, height=UIConfig.BTN_HEIGHT_SMALL,
                              font_size=UIConfig.FONT_SMALL)
                spin.setMinimumWidth(0)
                spin.valueChanged.connect(lambda _value, i=index: self._mark_edited(i))
                # An explicit zero in a blank unknown field is an edit too.
                spin.lineEdit().textEdited.connect(lambda _text, i=index: self._mark_edited(i))
                self._param_editors.append(spin)
                self.table.setCellWidget(index, 2, spin)
                self.table.setItem(index, 3, QtWidgets.QTableWidgetItem("Unknown"))
        self._refresh_values()
        self._update_enabled_state()
        self._remember_selection()
        self._refresh_active_controller()

    @QtCore.Slot(bool)
    def _on_bilateral_changed(self, _checked: bool):
        self._refresh_values()
        self._refresh_active_controller()
        self._remember_selection()

    def _mark_edited(self, index):
        self._edited_parameters.add(index)
        item = self.table.item(index, 3)
        if item is not None:
            item.setText("User edit (both sides)" if self.chk_bilateral.isChecked() else "User edit")

    @staticmethod
    def _value_from(values_db, row, index):
        if row is None:
            return None, None
        values = values_db.get((row[1], row[3]), [])
        if index >= len(values) or values[index] is None:
            return None, None
        raw = values[index]
        try:
            number = float(raw)
            if math.isfinite(number):
                return str(raw), number
        except (TypeError, ValueError, OverflowError):
            pass
        return None, None

    def _cached_value(self, row, index):
        if row is None or self._active_controllers.get(row[1]) != row[3]:
            return None, None
        return self._value_from(self._controller_values, row, index)

    def _proposed_value(self, row, index):
        raw, number = self._cached_value(row, index)
        if number is not None:
            return raw, number, "Live device"
        for values_db, source in ((self._controller_presets, "Last confirmed settings"),
                                  (self._controller_defaults, "SD default")):
            raw, number = self._value_from(values_db, row, index)
            if number is not None:
                return raw, number, source
        return None, None, "Unknown"

    @staticmethod
    def _same_device_value(first, second):
        if first is None or second is None:
            return False
        if first == second:
            return True
        try:
            # BLE controller parameters are float32; decimal ACK formatting may differ.
            return struct.pack("<f", first) == struct.pack("<f", second)
        except (OverflowError, struct.error):
            return first == second

    def _refresh_values(self):
        row = self._current_row()
        mirror = self._mirror_row() if self.chk_bilateral.isChecked() else None
        for index, spin in enumerate(self._param_editors):
            raw, _ = self._cached_value(row, index)
            active = row is not None and self._active_controllers.get(row[1]) == row[3]
            text = raw if raw is not None else ("Unknown" if active else "Not in use")
            _, number, source = self._proposed_value(row, index)
            if mirror is not None:
                mirror_raw, _ = self._cached_value(mirror, index)
                mirror_active = self._active_controllers.get(mirror[1]) == mirror[3]
                mirror_live = mirror_raw if mirror_raw is not None else (
                    "Unknown" if mirror_active else "Not in use")
                text = f"Selected: {text}\nOpposite: {mirror_live}"
                mirror_proposed, _, mirror_source = self._proposed_value(mirror, index)
                source = (f"Selected: {source}\nOpposite: {mirror_source}"
                          f" ({mirror_proposed if mirror_proposed is not None else 'unknown'})")
            self.table.item(index, 1).setText(text)
            self.table.item(index, 1).setToolTip(text)
            if index in self._edited_parameters:
                source = "User edit (both sides)" if mirror is not None else "User edit"
            else:
                spin.blockSignals(True)
                if number is not None:
                    spin.setValue(number)
                else:
                    spin.setValue(0)
                    spin.lineEdit().clear()
                    spin.lineEdit().setPlaceholderText("Enter value")
                spin.blockSignals(False)
            self.table.item(index, 3).setText(source)
            self.table.item(index, 3).setToolTip(source)
            spin.setToolTip(
                "Explicit edits apply to both sides; untouched opposite-side values are preserved."
                if mirror is not None else
                "Switching sends the complete proposed preset; in-use modes send only edited changes."
            )
        self.table.resizeRowsToContents()

    @QtCore.Slot()
    def _on_apply(self):
        if self._update_pending:
            return
        row = self._current_row()
        if row is None:
            self.set_param_update_status("No valid controller metadata selected.")
            return
        bilateral = self.chk_bilateral.isChecked()
        mirror = self._mirror_row() if bilateral else None
        if bilateral and mirror is None:
            self.set_param_update_status("Bilateral update requires a matching opposite-side controller schema.")
            return
        sides = [row, mirror] if mirror is not None else [row]
        # A timeout does not prevent applying an explicit complete preset.
        # If the actual mode is unknown, send every field rather than assuming
        # that a sparse update can reuse unspecified device values.
        switching = {side[1]: self._active_controllers.get(side[1]) != side[3] for side in sides}
        edits = {}
        for index in sorted(self._edited_parameters):
            spin = self._param_editors[index]
            if not spin.hasAcceptableInput():
                self.set_param_update_status(f"Enter a valid value for {row[index + 4]}.")
                return
            spin.interpretText()
            edits[index] = float(spin.value())
        payloads = []
        for index in range(len(self._param_editors)):
            updates = []
            for side in sides:
                if not switching[side[1]] and index not in edits:
                    continue
                _, proposed, _ = self._proposed_value(side, index)
                target = edits.get(index, proposed)
                if target is None:
                    self.set_param_update_status(
                        f"Cannot switch {side[0]}: {side[index + 4]} is unknown. "
                        "Read its settings or enter an explicit value; no zero default is assumed.")
                    return
                _, current = self._cached_value(side, index)
                if switching[side[1]] or not self._same_device_value(current, target):
                    updates.append((side, target))
            if len(updates) == 2 and self._same_device_value(updates[0][1], updates[1][1]):
                payloads.append([True, int(row[1]), int(row[3]), index, updates[0][1]])
            else:
                payloads.extend([False, int(side[1]), int(side[3]), index, target]
                                for side, target in updates)
        if not payloads:
            self.set_param_update_status("No changed properties to apply.", warning=False)
            return
        self._remember_selection()
        self.applyRequested.emit(payloads)
