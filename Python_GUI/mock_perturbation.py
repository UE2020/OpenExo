"""Hardware-free GUI demonstration using recorded data and mock status frames.

Plots replay actual saved samples, NOT a prediction of the perturbation response.
The mock uses recorded stance rising edges; production pulses use forefoot loading.
"""
import argparse
import csv
from pathlib import Path
import sys
import time
from PySide6 import QtCore, QtGui, QtWidgets
from pages.ActiveTrialPage import ActiveTrialPage
from services.Perturbation import PerturbationSession
from services.FsrThresholds import FsrThresholdSession, valid_center
from services.RtBridge import RtBridge
from styles import DARK_STYLESHEET


class MockExo(QtCore.QObject):
    dataReceived = QtCore.Signal(bytes)
    fsrThresholdStatusReceived = QtCore.Signal(dict)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.pending = None
        self.stance = [False, False]
        self.now = 0
        self.last_id = 0
        self.paused = False
        self.fsr_centers = (.25, .25)
        self._fsr_cached = None

    def set_paused(self, paused):
        self.paused = bool(paused)
        if paused:
            self.sendPerturbation(0, 0, 1, self.last_id or 1)

    def queryFsrThresholds(self):
        self._fsr_cached = dict(request_id=0, state=0, reason=0, editable=False,
            left_lower=0, left_upper=0, right_lower=0, right_upper=0)
        self.fsrThresholdStatusReceived.emit(dict(self._fsr_cached))
        return True

    def sendFsrThresholds(self, operation, left, right, request_id):
        if operation not in (0, 1) or not 1 <= request_id <= 9999:
            return False
        if operation and not (valid_center(left) and valid_center(right)):
            return False
        reason = (1 if not self.paused else 2 if self.pending else 0) if operation else 0
        if operation and not reason:
            self.fsr_centers = (left, right)
        actual_left, actual_right = self.fsr_centers
        self._fsr_cached = dict(request_id=request_id, state=3 if reason else 2 if operation else 1,
            reason=reason, editable=self.paused and self.pending is None,
            left_lower=round(actual_left-.05, 2), left_upper=round(actual_left+.05, 2),
            right_lower=round(actual_right-.05, 2), right_upper=round(actual_right+.05, 2))
        self.fsrThresholdStatusReceived.emit(dict(self._fsr_cached))
        return True

    def readFsrThresholdStatus(self):
        if self._fsr_cached is None:
            return False
        self.fsrThresholdStatusReceived.emit(dict(self._fsr_cached))
        return True

    def _reply(self, request_id, side, state, magnitude, duration):
        values = (request_id, side, state, magnitude, duration)
        frame = ("Sp5c" + "".join(f"{v * 100:.0f}n" for v in values)).encode("ascii")
        for offset in range(0, len(frame), 17):
            self.dataReceived.emit(frame[offset:offset + 17])

    def queryPerturbation(self):
        self._reply(0, 0, 0, 0, 0)
        return True

    def sendPerturbation(self, side, magnitude, duration, request_id):
        if side == 0:
            if self.pending:
                pulse = self.pending
                self.pending = None
                self._reply(pulse[0], pulse[1], 4, pulse[2], pulse[3])
            return True
        if self.paused or self.pending or request_id <= self.last_id:
            self._reply(request_id, side, 5, magnitude, duration)
            return True
        self.last_id = request_id
        self.pending = [request_id, side, magnitude, duration, self.now, None]
        self._reply(request_id, side, 1, magnitude, duration)
        return True

    def advance(self, values, now_ms):
        self.now = now_ms
        stance = [values[5] > 0.5, values[7] > 0.5]
        pulse = self.pending
        if pulse:
            request_id, side, magnitude, duration, armed_at, started_at = pulse
            if started_at is None and now_ms - armed_at >= 10000:
                self.pending = None
                self._reply(request_id, side, 6, magnitude, duration)
            elif started_at is None and stance[side - 1] and not self.stance[side - 1]:
                pulse[5] = now_ms
                self._reply(request_id, side, 2, magnitude, duration)
            elif started_at is not None and now_ms - started_at >= duration:
                self.pending = None
                self._reply(request_id, side, 3, magnitude, duration)
        self.stance = stance


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", type=Path, default=Path(__file__).parent /
        "Saved_Data/EXOpower_perturb__20261002_170455_236654.csv")
    parser.add_argument("--screenshot", type=Path, help="Save real Qt widget capture and exit")
    parser.add_argument("--settings-screenshot", action="store_true", help="Capture paused threshold editor")
    args = parser.parse_args()
    with args.csv.open(newline="") as source:
        reader = csv.reader(source)
        header = next(reader)[2:]
        rows = list(reader)
        epochs = [float(row[0]) for row in rows]
        samples = [[float(value) for value in row[2:]] for row in rows]
    if not samples:
        parser.error("CSV contains no samples")
    app = QtWidgets.QApplication(sys.argv[:1])
    # Windows offscreen Qt has no system font database; load a real local font.
    if not QtGui.QFontDatabase.families() and sys.platform == "win32":
        font_file = Path("C:/Windows/Fonts/segoeui.ttf")
        if font_file.exists():
            font_id = QtGui.QFontDatabase.addApplicationFont(str(font_file))
            families = QtGui.QFontDatabase.applicationFontFamilies(font_id)
            if families:
                app.setFont(QtGui.QFont(families[0], 10))
    app.setStyleSheet(DARK_STYLESHEET)
    window = QtWidgets.QWidget()
    window.setWindowTitle("OpenExo — MOCK EXO / Recorded Trial Replay")
    layout = QtWidgets.QVBoxLayout(window)
    label = QtWidgets.QLabel(
        "MOCK EXO • No hardware connected • Plots replay recorded data, not predicted pulse response")
    label.setWordWrap(True)
    label.setStyleSheet("color: #f4b942; padding: 6px; font-weight: bold;")
    layout.addWidget(label)
    page = ActiveTrialPage()
    layout.addWidget(page)
    page.set_channel_labels(header)
    page.update_battery_level(12.3)
    page.btn_end_trial.clicked.connect(window.close)
    for button in (page.btn_start_recording, page.btn_stop_recording,
                   page.btn_update_controller, page.btn_bio_feedback, page.btn_ml,
                   page.btn_recal_fsr, page.btn_send_preset_fsr, page.btn_recal_torque):
        button.setEnabled(False)
    device = MockExo(window)
    bridge = RtBridge(window)
    bridge._collecting_names = False
    bridge._controllers_done = True
    session = PerturbationSession(device, window)
    device.dataReceived.connect(bridge.feed_bytes)
    bridge.perturbationStatusReceived.connect(session.receive)
    session.changed.connect(lambda: page.set_perturbation_state(
        session.ready, session.busy, session.message))
    page.perturbationCheckRequested.connect(session.check_firmware)
    page.perturbationArmRequested.connect(session.arm)
    page.perturbationCancelRequested.connect(session.cancel)
    settings = FsrThresholdSession(device, session, window)
    device.fsrThresholdStatusReceived.connect(settings.receive)
    page.fsrReadRequested.connect(settings.check)
    page.fsrApplyRequested.connect(settings.apply)
    def update_settings():
        page.set_fsr_threshold_state(settings.ready, settings.pending, settings.message)
        page.set_perturbation_state(session.ready and not (settings.pending or settings.uncertain),
                                   session.busy, session.message)
        page.btn_pause_play.setEnabled(not device.paused or not (settings.pending or settings.uncertain))
    settings.changed.connect(update_settings)
    def pause():
        device.set_paused(True)
        session.set_running(False)
        settings.set_paused(True)
        settings.check()
    def resume():
        if not settings.pending and not settings.uncertain:
            device.set_paused(False)
            settings.set_paused(False)
            session.set_running(True)
    page.deviceStopRequested.connect(pause)
    page.deviceStartRequested.connect(resume)
    session.set_running(True)
    window.resize(1200, 900)
    window.show()
    index = 0
    start = time.monotonic()
    timer = QtCore.QTimer(window)

    def tick():
        nonlocal index
        values = samples[index % len(samples)]
        index += 1
        page.apply_values(values)
        device.advance(values, int((time.monotonic() - start) * 1000))

    timer.timeout.connect(tick)
    timer.start(10)
    if args.screenshot:
        def capture():
            timer.stop()
            # A real captured Qt screen, using existing trial samples for plots.
            selected = samples[1000:1500] or samples[:500]
            selected_epochs = epochs[1000:1500] or epochs[:500]
            x = [epoch - selected_epochs[0] for epoch in selected_epochs]
            for channel, curve in enumerate((page.curve_top_cmd, page.curve_top_meas,
                                             page.curve_bot_a, page.curve_bot_b)):
                curve.setData(x, [values[channel] for values in selected])
            for plot in (page.plot_top, page.plot_bottom):
                plot.setLabel("bottom", "Recorded GUI receipt time (s)")
                plot.setLabel("left", "Torque (Nm)")
            page.spin_impulse_magnitude.setValue(2.0)
            if args.settings_screenshot:
                page.btn_pause_play.click()
                page.spin_fsr_left.setValue(.35)
                page.spin_fsr_right.setValue(.45)
                page.btn_apply_fsr.click()
                page._controls_scroll.ensureWidgetVisible(page.lbl_fsr_status)
            else:
                page.btn_check_impulse.click()
                page.btn_arm_left.click()
            app.processEvents()
            args.screenshot.parent.mkdir(parents=True, exist_ok=True)
            if not window.grab().save(str(args.screenshot)):
                raise RuntimeError("Qt screenshot save failed")
            print(f"SCREENSHOT={args.screenshot.resolve()}")
            print(f"STATUS={page.lbl_impulse_status.text()}")
            print(f"FSR_STATUS={page.lbl_fsr_status.text()}")
            print(f"REPLAY={args.csv.name}; rows={len(samples)}")
            app.quit()
        QtCore.QTimer.singleShot(250, capture)
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
