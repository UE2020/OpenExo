"""Confirmed, paused-only configuration of the shared forefoot contact thresholds."""
import math
import struct
from PySide6 import QtCore


MIN_CENTER = .06
MAX_CENTER = .94
HYSTERESIS = .05
REASONS = {1: "Pause exo / disable motors first", 2: "Perturbation is pending",
           3: "Invalid threshold configuration", 4: "Calibration is in progress"}


def valid_center(value):
    return (math.isfinite(value) and MIN_CENTER <= value <= MAX_CENTER
            and abs(value * 100 - round(value * 100)) < 1e-6)


def decode_status(data):
    if len(data) != 16:
        raise ValueError("Incomplete FSR settings status")
    magic, version, request_id, state, reason, ll, lu, rl, ru, editable = struct.unpack(
        '<2sBHBBhhhhB', data)
    if (magic != b'FT' or version != 1 or request_id > 9999 or state > 3
            or reason > 4 or editable > 1 or not 0 <= ll <= lu <= 100
            or not 0 <= rl <= ru <= 100):
        raise ValueError("Invalid FSR settings status")
    return dict(request_id=request_id, state=state, reason=reason,
                left_lower=ll/100, left_upper=lu/100,
                right_lower=rl/100, right_upper=ru/100, editable=bool(editable))


class FsrThresholdSession(QtCore.QObject):
    changed = QtCore.Signal()
    event = QtCore.Signal(dict)

    def __init__(self, device, perturbation, parent=None):
        super().__init__(parent)
        self.device = device
        self.perturbation = perturbation
        self.paused = False
        self.supported = False
        self.editable = False
        self.pending = False
        self.uncertain = False
        self.confirmed = None
        self.message = 'Read firmware settings; pause and disarm before applying'
        self._checking = False
        self._request_id = 0
        self._request = None
        self._timer = QtCore.QTimer(self)
        self._timer.setSingleShot(True)
        self._timer.timeout.connect(self._timeout)
        self._poll = QtCore.QTimer(self)
        self._poll.timeout.connect(self.poll_status)
        perturbation.changed.connect(self.changed.emit)

    @property
    def ready(self):
        return (self.supported and self.paused and self.editable and not self.pending
                and not self.uncertain and not self.perturbation.busy
                and not self.perturbation.running)

    def set_paused(self, paused):
        self.paused = bool(paused)
        if not paused:
            self.editable = False
        self.changed.emit()

    def check(self):
        if self.pending or self.perturbation.busy:
            return False
        self.supported = False
        self._checking = True
        self.pending = True
        self.message = 'Reading settings from control MCU…'
        self._timer.start(3000)
        self._poll.start(250)
        if not self.device.queryFsrThresholds():
            self._timeout()
            return False
        self.changed.emit()
        return True

    def _send(self, operation, left=0, right=0):
        if self._request_id >= 9999:
            self._timeout()
            return False
        self._request_id += 1
        self._request = (self._request_id, operation, left, right)
        self.pending = True
        self.message = 'Awaiting control-MCU confirmation…'
        self._timer.start(3000)
        self._poll.start(250)
        if not self.device.sendFsrThresholds(operation, left, right, self._request_id):
            self._timeout()
            return False
        self.changed.emit()
        return True

    def apply(self, left, right):
        if not self.ready or not valid_center(left) or not valid_center(right):
            return False
        return self._send(1, left, right)

    def poll_status(self):
        if self.pending:
            self.device.readFsrThresholdStatus()

    def receive(self, reply):
        if self._checking and reply['request_id'] == 0 and reply['state'] == 0:
            self._checking = False
            self.supported = True
            self._send(0)  # Read actual settings with a correlated request ID.
            return
        if not self.pending or self._request is None:
            return
        request_id, operation, left, right = self._request
        if reply['request_id'] != request_id:
            return
        state = reply['state']
        if state not in (1, 2, 3) or (state != 3 and state != (2 if operation else 1)):
            return
        if state == 2:
            expected = [round(v*100) for v in (left-HYSTERESIS, left+HYSTERESIS,
                                               right-HYSTERESIS, right+HYSTERESIS)]
            actual = [round(reply[k]*100) for k in ('left_lower', 'left_upper',
                                                    'right_lower', 'right_upper')]
            if expected != actual:
                return
        if state != 3:
            self.confirmed = (round((reply['left_lower']+reply['left_upper'])/2, 2),
                              round((reply['right_lower']+reply['right_upper'])/2, 2))
            self.uncertain = False
        # A rejected read may contain placeholder values for invalid settings.
        # Only an actual successful read/apply can resolve an ambiguous write.
        self.editable = reply['editable'] and state != 3
        self.pending = False
        self._request = None
        self._timer.stop()
        self._poll.stop()
        self.message = ('Rejected: ' + REASONS.get(reply['reason'], 'unknown reason')
                        if state == 3 else
                        f"Confirmed: L ON {reply['left_upper']:.2f} / OFF {reply['left_lower']:.2f}; "
                        f"R ON {reply['right_upper']:.2f} / OFF {reply['right_lower']:.2f}")
        self.event.emit(dict(reply))
        self.changed.emit()

    def _timeout(self):
        self._timer.stop()
        self._poll.stop()
        self._checking = False
        self.pending = False
        self.uncertain = self.uncertain or (self._request is not None and self._request[1] == 1)
        self.supported = False
        self.editable = False
        self.message = ('Settings unconfirmed — remain paused and read again before resuming'
                        if self.uncertain else
                        'FSR editor unavailable / no response — ordinary assistance remains available')
        self.changed.emit()

    def reset(self):
        self._timer.stop()
        self._poll.stop()
        self._checking = False
        self._request = None
        self._request_id = 0
        self.paused = self.supported = self.editable = self.pending = self.uncertain = False
        self.confirmed = None
        self.message = 'Disconnected — settings not confirmed'
        self.changed.emit()
