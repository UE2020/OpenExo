"""Exercise the actual GUI parser with Qt signal/regex/timer stubs, no BLE.

Checks complete frames and transport fragmentation/coalescing. Qt UI dispatch
and hardware notification delivery are intentionally outside this reproduction.
"""
import importlib.util
import logging
from pathlib import Path
import re
import sys
import types


class Signal:
    def __init__(self, *args):
        self.events = []

    def connect(self, *args):
        pass

    def emit(self, *args):
        self.events.append(args)


class QObject:
    def __init__(self, *args):
        pass


class Timer:
    timeout = Signal()

    def start(self, *args):
        pass


class Regex:
    def __init__(self, pattern):
        self.pattern = pattern

    def match(self, text):
        match = re.search(self.pattern, text)
        return types.SimpleNamespace(hasMatch=lambda: match is not None,
                                     captured=lambda index: match.group(index))


qt = types.SimpleNamespace(QObject=QObject, Signal=Signal, QTimer=Timer,
                           QRegularExpression=Regex, Slot=lambda *args: lambda fn: fn)
sys.modules["PySide6"] = types.SimpleNamespace(QtCore=qt)
ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("tested_rt_bridge", ROOT / "Python_GUI/services/RtBridge.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
logging.disable(logging.CRITICAL)


def run(chunks):
    bridge = module.RtBridge()
    # Replace class-level test signals with independent collectors.
    bridge.paramUpdateAckReceived = Signal()
    bridge.rtDataUpdated = Signal()
    bridge._handshake = True
    bridge._collecting_names = False
    for chunk in chunks:
        bridge.feed_bytes(chunk)
    return [args[0] for args in bridge.paramUpdateAckReceived.events]


# Firmware package_raw_data encodes S + command + count + c + integer*100 + n.
left = b"Sa5c6500n200n0n100n0n"
right = b"Sa5c3300n200n0n100n0n"
expected = dict(joint_id=65, controller_id=2, param_index=0, accepted=True, reason=0)
assert run([left]) == [expected]
assert len(run([left, right])) == 2
print("Complete ACK frames: both sides parsed correctly.")
split_failures = []
for split in range(1, len(left)):
    if run([left[:split], left[split:]]) != [expected]:
        split_failures.append(split)
print(f"Fragmented ACK: {len(split_failures)} of {len(left)-1} split positions lose or misparse ACK.")
print("ACKs from two frames in one chunk:", len(run([left + right])))
print("These are parser robustness findings, not proof that today's BLE transport fragmented ACKs.")
