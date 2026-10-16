"""Researcher-side interlocks; firmware alone schedules the physical pulse."""
import math
import struct
from PySide6 import QtCore


REJECTION_REASONS = {
    1: "trial is not running", 2: "e-stop is active", 3: "exo is paused",
    4: "device error is active", 5: "calibration is still in progress",
    6: "select PJMC or PJMC_PLUS on this ankle", 7: "ankle is not configured for use",
    8: "ankle motor is disabled", 9: "ankle motor communications timed out",
    10: "torque sensor is not calibrated", 11: "sensor reading is invalid",
    12: "firmware session is not established", 13: "Nano heartbeat is stale",
    14: "another perturbation is pending", 15: "request ID was already used",
    16: "request is malformed", 17: "firmware safety interlock is not satisfied",
}


def decode_readback(data):
    """Decode a complete 12-byte, versioned, read-only GATT status value."""
    if len(data) != 12:
        raise ValueError("Incomplete perturbation status readback")
    magic, version, request_id, side, state, reason, magnitude, duration = struct.unpack(
        "<2sBHBBBhH", data)
    if (magic != b"AP" or version != 1 or request_id > 9999 or side > 2
            or state > 6 or reason > 17 or abs(magnitude) > 1000 or duration > 200):
        raise ValueError("Invalid perturbation status readback")
    reply = dict(request_id=request_id, side=side, state=state,
                 magnitude=magnitude / 100.0, duration_ms=duration)
    if reason:
        reply["reason"] = reason
    return reply


class PerturbationSession(QtCore.QObject):
    changed = QtCore.Signal()
    event = QtCore.Signal(dict)

    def __init__(self, device, parent=None):
        super().__init__(parent)
        self.device = device
        self.supported = False
        self.running = False
        self.busy = False
        self.message = "Check firmware before arming"
        self._checking = False
        self._request_id = 0
        self._pending = None
        self._last_terminal = None
        self._last_state = None
        self._uncertain = False
        self._poll_timer = QtCore.QTimer(self)
        self._poll_timer.timeout.connect(self.poll_status)
        self._timer = QtCore.QTimer(self)
        self._timer.setSingleShot(True)
        self._timer.timeout.connect(self._timeout)

    @property
    def ready(self):
        return self.supported and self.running and not self.busy and not self._checking

    def check_firmware(self):
        if self.busy or self._checking:
            return False
        self.supported = False
        self._checking = True
        self.message = "Checking firmware…"
        self._timer.start(3000)
        if not self.device.queryPerturbation():
            self._timeout()
            return False
        self.changed.emit()
        return True

    def set_running(self, running):
        self.running = bool(running)
        self.changed.emit()

    def arm(self, side, magnitude, duration_ms):
        if not self.ready:
            return False
        if (side not in (1, 2) or not math.isfinite(magnitude)
                or not 0 < abs(magnitude) <= 10 or not 10 <= duration_ms <= 200
                or duration_ms != int(duration_ms)):
            self.message = "Invalid pulse: nonzero ±10 Nm, 10–200 ms"
            self.changed.emit()
            return False
        # Do not wrap within one connection: old request IDs must never refire.
        if self._request_id >= 9999:
            self.message = "Request IDs exhausted; reconnect before further pulses"
            self.changed.emit()
            return False
        self._request_id += 1
        self._pending = (self._request_id, side, magnitude, duration_ms)
        self._last_terminal = None
        self._last_state = None
        self._uncertain = False
        self._poll_timer.start(250)
        self.busy = True
        self.message = "Sending arm request (not yet confirmed)…"
        self._timer.start(3000)
        if not self.device.sendPerturbation(side, magnitude, duration_ms, self._request_id):
            self._timeout()
            return False
        self.changed.emit()
        return True

    def poll_status(self):
        read = getattr(self.device, "readPerturbationStatus", None)
        if read and (self._pending or self._last_terminal):
            read()

    def receive(self, reply):
        pending = self._pending or (self._last_terminal if reply.get("reason") else None)
        if self._checking and reply == dict(request_id=0, side=0, state=0,
                                           magnitude=0, duration_ms=0):
            self._checking = False
            self.supported = True
            self._timer.stop()
            self.message = "Firmware supported — runtime interlocks still apply"
        elif pending and reply["request_id"] == pending[0]:
            request_id, side, magnitude, duration = pending
            if reply["side"] != side or reply["magnitude"] != magnitude or reply["duration_ms"] != duration:
                return
            state = reply["state"]
            labels = {1: "Armed — waiting for next step onset", 2: "Impulse active",
                      3: "Delivered — disarmed", 4: "Cancelled — disarmed",
                      5: "Firmware rejected request", 6: "No step within 10 s — expired"}
            if state not in labels:
                return
            if state in (1, 2):
                if self._uncertain or (self._last_state is not None and state <= self._last_state):
                    return
            self._last_state = state
            detail = labels[state]
            if state == 5:
                detail += ": " + REJECTION_REASONS.get(reply.get("reason"),
                    "check controller, calibration and motor state")
            self.message = f"{'Left' if side == 1 else 'Right'}: {detail}"
            self.event.emit(dict(reply))
            if state in (1, 2):
                self._timer.start(12000 if state == 1 else 3000)
            else:
                self.busy = False
                self._last_terminal = pending if state == 5 else None
                self._pending = None
                self._timer.stop()
                self._poll_timer.stop()
                if state == 5 and not reply.get("reason"):
                    QtCore.QTimer.singleShot(0, self.poll_status)
        else:
            return
        self.changed.emit()

    def cancel(self):
        if not self.supported:
            return False
        self.message = "Cancel requested — awaiting firmware confirmation"
        self.changed.emit()
        return self.device.sendPerturbation(0, 0, 1, self._request_id or 1)

    def reset(self):
        self._timer.stop()
        self._poll_timer.stop()
        self._last_terminal = None
        self.supported = False
        self.running = False
        self.busy = False
        self._checking = False
        self._pending = None
        self._request_id = 0
        self.message = "Disconnected — firmware must be checked again"
        self.changed.emit()

    def _timeout(self):
        self._uncertain = True
        checking = self._checking
        self._checking = False
        if not checking and self.supported:
            self.device.sendPerturbation(0, 0, 1, self._request_id or 1)
        self.supported = False
        # An unacknowledged command is NOT evidence of an unarmed exo.
        self.message = ("Unsupported firmware / no response — arming disabled" if checking
                        else "Status unknown — cancel sent; PAUSE exo before continuing")
        self.changed.emit()
