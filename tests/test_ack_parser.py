"""Regression tests for RtBridge BLE frame reassembly.

These exercise the real ``RtBridge`` parser with Qt signal/regex/timer stubs so
they run without PySide6 or hardware. They cover the failure mode where a
parameter-update ACK is split across, or coalesced within, BLE notifications and
the UI therefore never clears its pending acknowledgement.

Run with: python tests/test_ack_parser.py
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

    def __init__(self, *args):
        pass

    def setSingleShot(self, *args):
        pass

    def start(self, *args):
        pass

    def stop(self, *args):
        pass


class Regex:
    def __init__(self, pattern):
        self.pattern = pattern

    def match(self, text):
        match = re.search(self.pattern, text)
        return types.SimpleNamespace(
            hasMatch=lambda: match is not None,
            captured=lambda index: match.group(index),
        )


qt = types.SimpleNamespace(
    QObject=QObject,
    Signal=Signal,
    QTimer=Timer,
    QRegularExpression=Regex,
    Slot=lambda *args: lambda fn: fn,
)
sys.modules.setdefault("PySide6", types.SimpleNamespace(QtCore=qt))

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    "tested_rt_bridge", ROOT / "Python_GUI/services/RtBridge.py"
)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
logging.disable(logging.CRITICAL)


# Firmware package_raw_data encodes S + command + count + c + integer*100 + n.
LEFT = b"Sa5c6500n200n0n100n0n"
RIGHT = b"Sa5c3300n200n0n100n0n"
REJECTED = b"Sa5c6500n200n0n0n700n"
RT = b"Sd3c100n200n300n"
LEFT_ACK = dict(joint_id=65, controller_id=2, param_index=0, accepted=True, reason=0)
RIGHT_ACK = dict(joint_id=33, controller_id=2, param_index=0, accepted=True, reason=0)


def make_bridge():
    bridge = module.RtBridge()
    bridge.paramUpdateAckReceived = Signal()
    bridge.rtDataUpdated = Signal()
    # Simulate a completed handshake so live frames take the stream path.
    bridge._handshake = True
    bridge._collecting_names = False
    bridge._controllers_done = True
    bridge._collecting_handshake_payload = False
    return bridge


def feed(chunks):
    bridge = make_bridge()
    for chunk in chunks:
        bridge.feed_bytes(chunk)
    return bridge


def acks(bridge):
    return [args[0] for args in bridge.paramUpdateAckReceived.events]


def rt_frames(bridge):
    return [args[0] for args in bridge.rtDataUpdated.events]


def test_complete_ack():
    assert acks(feed([LEFT])) == [LEFT_ACK]


def test_stream_mode_engages_after_handshake():
    bridge = module.RtBridge()
    bridge.paramUpdateAckReceived = Signal()
    bridge.rtDataUpdated = Signal()
    # Fresh-session state, as MainWindow sees it before READY.
    bridge._handshake = False
    bridge._collecting_names = True
    bridge._controllers_done = False
    bridge.feed_bytes(b"READY")
    bridge.feed_bytes(b"Ankle(L),65,zeroTorqu,2,use_pid|")
    bridge.feed_bytes(b"\n")
    assert bridge._is_metadata_phase() is False
    bridge.feed_bytes(LEFT[:6])
    bridge.feed_bytes(LEFT[6:])
    assert acks(bridge) == [LEFT_ACK]


def test_every_split_position():
    failures = []
    for split in range(1, len(LEFT)):
        if acks(feed([LEFT[:split], LEFT[split:]])) != [LEFT_ACK]:
            failures.append(split)
    assert not failures, f"ACK lost at split positions: {failures}"


def test_byte_by_byte():
    assert acks(feed([LEFT[i:i + 1] for i in range(len(LEFT))])) == [LEFT_ACK]


def test_coalesced_acks():
    bridge = feed([LEFT + RIGHT])
    assert acks(bridge) == [LEFT_ACK, RIGHT_ACK]


def test_ack_split_across_coalesced_then_next_callback():
    bridge = feed([LEFT[:7], LEFT[7:] + RIGHT])
    assert acks(bridge) == [LEFT_ACK, RIGHT_ACK]


def test_ack_with_rt_data_in_same_callback():
    bridge = feed([RT + LEFT])
    assert acks(bridge) == [LEFT_ACK]
    assert len(rt_frames(bridge)) == 1


def test_split_within_final_token_completes_without_flush():
    assert acks(feed([LEFT[:-1], b"n"])) == [LEFT_ACK]


def test_truncated_final_delimiter_recovers_on_flush():
    # A 20-byte notification drops the trailing 'n'; the idle flush must still
    # recover the ACK rather than leave the pending update to time out.
    bridge = feed([LEFT[:-1]])
    assert acks(bridge) == []
    bridge._flush_partial_frame()
    assert acks(bridge) == [LEFT_ACK]


def test_flush_recovers_truncated_rt_frame():
    bridge = feed([RT[:-1]])
    assert rt_frames(bridge) == []
    bridge._flush_partial_frame()
    frames = rt_frames(bridge)
    assert len(frames) == 1
    assert frames[0][:3] == [1.0, 2.0, 3.0]


def test_flush_discards_unrecoverable_partial():
    bridge = feed([b"Sa5c6500n200"])
    bridge._flush_partial_frame()
    assert acks(bridge) == []


def test_rejected_ack():
    bridge = feed([REJECTED])
    assert acks(bridge) == [
        dict(joint_id=65, controller_id=2, param_index=0, accepted=False, reason=7)
    ]


def test_fragmented_rt_frame_emits_once():
    for split in (1, 5, 9, len(RT) - 1):
        bridge = feed([RT[:split], RT[split:]])
        frames = rt_frames(bridge)
        assert len(frames) == 1, f"split {split} emitted {len(frames)} frames"
        assert frames[0][:3] == [1.0, 2.0, 3.0]
        assert len(frames[0]) == 16


def test_resync_after_garbage_prefix():
    bridge = feed([b"Xjunk" + LEFT])
    assert acks(bridge) == [LEFT_ACK]


def test_duplicate_acks_are_all_reported():
    assert acks(feed([LEFT + LEFT])) == [LEFT_ACK, LEFT_ACK]


def test_bilateral_out_of_order():
    bridge = feed([RIGHT + LEFT])
    assert acks(bridge) == [RIGHT_ACK, LEFT_ACK]


TESTS = [value for name, value in sorted(globals().items()) if name.startswith("test_")]


def main():
    failures = 0
    for test in TESTS:
        try:
            test()
        except AssertionError as error:
            failures += 1
            print(f"FAIL {test.__name__}: {error}")
        else:
            print(f"ok   {test.__name__}")
    print(f"\n{len(TESTS) - failures}/{len(TESTS)} tests passed")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
