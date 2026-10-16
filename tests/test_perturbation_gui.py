"""Real Qt parser/UI tests; no BLE adapter or physical exo required."""
import os
import sys
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "Python_GUI"))
from PySide6 import QtWidgets
from services.RtBridge import RtBridge


def app():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


def test_fragmented_perturbation_status_does_not_enter_telemetry():
    application = app()
    bridge = RtBridge()
    bridge._collecting_names = False
    bridge._controllers_done = True
    events, telemetry = [], []
    bridge.perturbationStatusReceived.connect(events.append)
    bridge.rtDataUpdated.connect(telemetry.append)
    frame = b"Sp5c700n100n100n-250n10000n"
    for byte in frame:
        bridge.feed_bytes(bytes([byte]))
    assert events == [dict(request_id=7, side=1, state=1, magnitude=-2.5, duration_ms=100)]
    assert telemetry == []
    bridge._data_rate_timer.stop()


class Device:
    def __init__(self):
        self.commands = []
    def queryPerturbation(self):
        self.commands.append("query")
        return True
    def sendPerturbation(self, side, magnitude, duration, request_id):
        self.commands.append((side, magnitude, duration, request_id))
        return True


def status(request_id=0, side=0, state=0, magnitude=0, duration_ms=0):
    return dict(request_id=request_id, side=side, state=state,
                magnitude=magnitude, duration_ms=duration_ms)


def test_researcher_cannot_arm_until_firmware_ack_then_one_shot():
    application = app()
    from services.Perturbation import PerturbationSession
    device = Device()
    session = PerturbationSession(device)
    session.set_running(True)
    assert not session.arm(1, 2.5, 100)
    session.check_firmware()
    assert device.commands == ["query"]
    session.receive(status())
    assert session.ready
    assert session.arm(1, 2.5, 100)
    request_id = device.commands[-1][-1]
    assert not session.arm(2, 3, 100)
    session.receive(status(request_id, 1, 1, 2.5, 100))
    assert session.busy
    session.receive(status(request_id, 1, 2, 2.5, 100))
    session.receive(status(request_id, 1, 3, 2.5, 100))
    assert not session.busy and session.ready
    assert len(device.commands) == 2


def test_device_uses_safe_query_and_atomic_double_payload():
    from services.QtExoDeviceManager import QtExoDeviceManager
    import struct
    captured = []
    class Stub:
        def _ensure_connected(self):
            return True
        def _queue_write_steps(self, steps, message):
            captured.append(steps)
    stub = Stub()
    assert QtExoDeviceManager.queryPerturbation(stub)
    assert captured == [[(b"J", True, 0.0)]]
    assert QtExoDeviceManager.sendPerturbation(stub, 2, -3.5, 80, 4)
    assert [step[0] for step in captured[-1]] == [b"j"] + [
        struct.pack("<d", v) for v in (2, -3.5, 80, 4)]


def test_real_trial_ui_has_independent_arm_buttons_and_adjustable_pulse():
    application = app()
    from pages.ActiveTrialPage import ActiveTrialPage
    page = ActiveTrialPage()
    emitted = []
    page.perturbationArmRequested.connect(lambda *args: emitted.append(args))
    assert not page.btn_arm_left.isEnabled()
    page.set_perturbation_state(True, False, "Ready")
    page.spin_impulse_magnitude.setValue(-2.5)
    page.spin_impulse_duration.setValue(80)
    page.btn_arm_left.click()
    page.btn_arm_right.click()
    assert emitted == [(1, -2.5, 80), (2, -2.5, 80)]
    page.set_perturbation_state(False, True, "Left: Armed")
    assert not page.btn_arm_left.isEnabled() and not page.btn_arm_right.isEnabled()
    assert page.btn_cancel_impulse.isEnabled()
    page.close()


def test_main_window_routes_buttons_parser_and_pause(monkeypatch):
    application = app()
    import logging
    from services.QtExoDeviceManager import QtExoDeviceManager
    from MainWindow import MainWindow
    monkeypatch.setattr(QtExoDeviceManager, "_setup_logging",
                        lambda self: setattr(self, "logger", logging.getLogger("mock-exo")))
    monkeypatch.setattr(QtExoDeviceManager, "_ensure_connected", lambda self: True)
    writes = []
    monkeypatch.setattr(QtExoDeviceManager, "_queue_write_steps",
                        lambda self, steps, message: writes.append(steps))
    window = MainWindow()
    window.perturbation.set_running(True)
    window.trial_page.btn_check_impulse.click()
    window.rt_bridge._collecting_names = False
    window.rt_bridge._controllers_done = True
    window.qt_dev.dataReceived.emit(b"Sp5c0n0n0n0n0n")
    assert window.trial_page.btn_arm_left.isEnabled()
    window.trial_page.spin_impulse_magnitude.setValue(2)
    window.trial_page.btn_arm_left.click()
    window.qt_dev.dataReceived.emit(b"Sp5c100n100n100n200n10000n")
    assert "Armed" in window.trial_page.lbl_impulse_status.text()
    window._on_device_stop_motors()
    assert not window.trial_page.btn_arm_left.isEnabled()
    assert any(steps[0][0] == b"w" for steps in writes)
    window.perturbation.reset()
    window.rt_bridge._data_rate_timer.stop()
    window.close()


def test_firmware_events_are_saved_next_to_recording_with_receipt_time(tmp_path):
    from MainWindow import MainWindow
    import json
    import logging
    from types import SimpleNamespace
    recording = tmp_path / "trial.csv"
    event = status(1, 1, 3, 2, 100)
    window = SimpleNamespace(logger=logging.getLogger("test"),
                             _csv_file=SimpleNamespace(name=str(recording)))
    MainWindow._on_perturbation_event(window, event)
    log = Path(str(recording) + ".perturbations.jsonl")
    row = json.loads(log.read_text().strip())
    assert row["event"] == event
    assert row["timestamp_kind"] == "gui_receipt"
    assert row["epoch"] > 0


def ready_session():
    from services.Perturbation import PerturbationSession
    device = Device()
    session = PerturbationSession(device)
    session.set_running(True)
    session.check_firmware()
    session.receive(status())
    return session, device


def test_timeout_never_claims_disarmed_or_retries_arm():
    application = app()
    session, device = ready_session()
    session.arm(2, -2, 50)
    session._timeout()
    assert session.busy and not session.ready
    assert "unknown" in session.message
    assert device.commands[-1] == (0, 0, 1, 1)
    assert not session.check_firmware()
    assert not session.arm(1, 2, 100)
    session.reset()
    assert not session.ready and not session.supported


def test_stale_or_wrong_side_status_cannot_disarm_pending():
    application = app()
    session, device = ready_session()
    session.arm(1, 2, 100)
    session.receive(status(2, 1, 3, 2, 100))
    session.receive(status(1, 2, 3, 2, 100))
    assert session.busy
    session.receive(status(1, 1, 4, 2, 100))
    assert not session.busy
    session.reset()


def test_invalid_and_zero_pulses_never_transmitted():
    application = app()
    session, device = ready_session()
    for side, magnitude, duration in ((0, 2, 100), (3, 2, 100),
            (1, 0, 100), (1, float("nan"), 100), (1, 11, 100),
            (1, 2, 9), (1, 2, 201), (1, 2, 10.5)):
        assert not session.arm(side, magnitude, duration)
    assert device.commands == ["query"]
    session.reset()


def test_legacy_firmware_no_ack_leaves_arming_disabled():
    application = app()
    from services.Perturbation import PerturbationSession
    session = PerturbationSession(Device())
    session.set_running(True)
    session.check_firmware()
    session._timeout()
    assert not session.ready and not session.busy
    assert not session.arm(1, 2, 100)


def test_bad_status_frames_never_become_telemetry():
    application = app()
    bridge = RtBridge()
    bridge._collecting_names = False
    events, telemetry = [], []
    bridge.perturbationStatusReceived.connect(events.append)
    bridge.rtDataUpdated.connect(telemetry.append)
    for frame in (b"Sp4c100n100n100n100n", b"Sp5cnann100n100n100n100n",
                  b"Sp5c100n300n100n100n100n", b"Sp5c100n100n700n100n100n",
                  b"Sp5c150n100n100n100n100n"):
        bridge.feed_bytes(frame)
    assert events == telemetry == []
    bridge._data_rate_timer.stop()


def test_mock_exo_waits_for_new_ipsilateral_stance_then_completes_once():
    application = app()
    from mock_perturbation import MockExo
    device = MockExo()
    events = []
    bridge = RtBridge()
    bridge._collecting_names = False
    bridge.perturbationStatusReceived.connect(events.append)
    device.dataReceived.connect(bridge.feed_bytes)
    device.queryPerturbation()
    device.advance([0, 0, 0, 0, 0, 1, 0, 0], 0)
    device.sendPerturbation(1, 2, 50, 1)
    device.advance([0, 0, 0, 0, 0, 1, 0, 1], 20)
    assert events[-1]["state"] == 1
    device.advance([0, 0, 0, 0, 0, 0, 0, 0], 30)
    device.advance([0, 0, 0, 0, 0, 1, 0, 0], 40)
    assert events[-1]["state"] == 2
    device.advance([0, 0, 0, 0, 0, 1, 0, 0], 90)
    assert events[-1]["state"] == 3
    device.advance([0, 0, 0, 0, 0, 0, 0, 0], 100)
    device.advance([0, 0, 0, 0, 0, 1, 0, 0], 110)
    assert [e["state"] for e in events] == [0, 1, 2, 3]
    bridge._data_rate_timer.stop()
