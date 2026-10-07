"""Regression coverage for GUI CSV recording while motor output is paused.

This loads the real MainWindow methods with minimal Qt/import stubs so the
recording path can be tested without PySide6 or connected hardware.
Run with: python tests/test_gui_paused_recording.py
"""
import csv
import importlib.util
import io
import logging
from pathlib import Path
import sys
import types


class QMainWindow:
    pass


def _slot(*args):
    return lambda function: function


def _load_main_window():
    root = Path(__file__).resolve().parents[1]
    qtcore = types.SimpleNamespace(Slot=_slot)
    qtwidgets = types.SimpleNamespace(QMainWindow=QMainWindow)
    pyside = types.ModuleType("PySide6")
    pyside.QtCore = qtcore
    pyside.QtWidgets = qtwidgets
    pyside.QtGui = types.SimpleNamespace()

    pages = types.ModuleType("pages")
    for name in (
        "ScanWindowQt",
        "ActiveTrialPage",
        "ActiveTrialSettingsPage",
        "ActiveTrialBasicSettingsPage",
        "BioFeedbackPage",
    ):
        setattr(pages, name, object)

    services = types.ModuleType("services")
    services.QtExoDeviceManager = object
    services.RtBridge = object
    utils = types.ModuleType("utils")
    utils.SettingsManager = object

    replacements = {
        "PySide6": pyside,
        "pages": pages,
        "services": services,
        "utils": utils,
    }
    previous = {name: sys.modules.get(name) for name in replacements}
    sys.modules.update(replacements)
    try:
        spec = importlib.util.spec_from_file_location(
            "tested_main_window", root / "Python_GUI/MainWindow.py"
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module.MainWindow
    finally:
        for name, original in previous.items():
            if original is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = original


def test_rt_updates_are_written_to_csv_while_device_is_paused():
    main_window = _load_main_window()
    output = io.StringIO()
    trial_page = types.SimpleNamespace(apply_values=lambda values: None)
    bio_feedback_page = types.SimpleNamespace(apply_values=lambda values: None)
    window = types.SimpleNamespace(
        trial_page=trial_page,
        bio_feedback_page=bio_feedback_page,
        stack=types.SimpleNamespace(currentWidget=lambda: None),
        _csv_writer=csv.writer(output),
        _csv_header_written=False,
        _param_names=["ankle_angle"],
        _t0=None,
        _mark_counter=0,
        _device_paused=True,
        logger=logging.getLogger("test_gui_paused_recording"),
    )

    main_window._on_rt_update(window, [12.5])

    rows = list(csv.reader(io.StringIO(output.getvalue())))
    assert rows[0] == ["epoch", "mark", "ankle_angle"]
    assert len(rows) == 2
    assert rows[1][1:] == ["0", "12.500000"]


def test_pause_status_keeps_recording_active():
    main_window = _load_main_window()
    recording_states = []
    window = types.SimpleNamespace(
        logger=logging.getLogger("test_gui_paused_recording"),
        _device_paused=False,
        _csv_file=object(),
        qt_dev=types.SimpleNamespace(motorOff=lambda: None),
        trial_page=types.SimpleNamespace(
            set_recording_state=lambda active, status: recording_states.append((active, status))
        ),
    )

    main_window._on_device_stop_motors(window)

    assert window._device_paused is True
    assert recording_states == [(True, "Recording (exo paused)")]


def main():
    tests = [
        test_rt_updates_are_written_to_csv_while_device_is_paused,
        test_pause_status_keeps_recording_active,
    ]
    for test in tests:
        test()
        print(f"ok   {test.__name__}")
    print(f"\n{len(tests)}/{len(tests)} tests passed")


if __name__ == "__main__":
    main()
