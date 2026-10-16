"""Regressions for the lab's lost status-prefix notification and opaque rejection."""
import asyncio
import logging
import struct

from test_perturbation_gui import app, Device, status
from test_perturbation_firmware import compile_run, controller_context


def packet(request_id=3, side=1, state=5, reason=6, magnitude=500, duration=200):
    return struct.pack("<2sBHBBBhH", b"AP", 1, request_id, side, state, reason, magnitude, duration)


def test_lost_notification_prefix_is_recovered_without_resending_arm():
    application = app()
    from services.Perturbation import PerturbationSession, decode_readback
    from services.RtBridge import RtBridge
    class PollingDevice(Device):
        reads = 0
        def readPerturbationStatus(self):
            self.reads += 1
            return True
    device = PollingDevice()
    session = PerturbationSession(device)
    session.set_running(True)
    session.check_firmware()
    session.receive(status())
    session._request_id = 2
    session.arm(1, 5, 200)
    bridge = RtBridge()
    bridge._collecting_names = False
    bridge.perturbationStatusReceived.connect(session.receive)
    # Exact third-attempt notification payload in the hardware log: the 20-byte
    # prefix never reached the RX callback, only its 6-byte trailing duration.
    bridge.feed_bytes(b"20000n")
    assert session.busy
    session.poll_status()
    session.receive(decode_readback(packet()))
    assert not session.busy
    assert "PJMC" in session.message
    assert device.reads == 1
    assert device.commands == ["query", (1, 5, 200, 3)]
    bridge._data_rate_timer.stop()
    session.reset()


def test_real_manager_reads_fresh_complete_status_and_ignores_old_sessions(monkeypatch):
    application = app()
    from services.QtExoDeviceManager import QtExoDeviceManager, PERTURBATION_STATUS_UUID
    monkeypatch.setattr(QtExoDeviceManager, "_setup_logging",
                        lambda self: setattr(self, "logger", logging.getLogger("status-test")))
    manager = QtExoDeviceManager()
    received = []
    manager.perturbationStatusReceived.connect(received.append)
    class Client:
        class services:
            @staticmethod
            def get_characteristic(uuid):
                assert uuid == PERTURBATION_STATUS_UUID
                return object()
        async def read_gatt_char(self, uuid, *, use_cached):
            assert uuid == PERTURBATION_STATUS_UUID and use_cached is False
            return packet()
    client = Client()
    monkeypatch.setattr(manager, "_capture_session", lambda: (client, 1))
    monkeypatch.setattr(manager, "_is_current_session", lambda *args: True)
    def submit(coro):
        asyncio.run(coro)
        return True
    monkeypatch.setattr(manager, "_submit", submit)
    assert manager.readPerturbationStatus()
    assert received[-1]["reason"] == 6
    monkeypatch.setattr(manager, "_is_current_session", lambda *args: False)
    manager.readPerturbationStatus()
    assert len(received) == 1
    current = iter((True, False))
    monkeypatch.setattr(manager, "_is_current_session", lambda *args: next(current))
    manager.readPerturbationStatus()  # disconnected/reconnected during the read
    assert len(received) == 1


def test_readback_rejection_details_can_follow_a_delivered_legacy_notification():
    application = app()
    from services.Perturbation import PerturbationSession, decode_readback
    session = PerturbationSession(Device())
    session.set_running(True)
    session.check_firmware()
    session.receive(status())
    session._request_id = 2
    session.arm(1, 5, 200)
    session.receive(status(3, 1, 5, 5, 200))
    assert not session.busy
    session.receive(decode_readback(packet()))
    assert "PJMC" in session.message and not session.busy
    # A delayed generic notification must not erase the more precise diagnosis.
    session.receive(status(3, 1, 5, 5, 200))
    assert "PJMC" in session.message
    session.reset()


def test_readback_codec_rejects_short_invalid_and_future_version_values():
    import pytest
    from services.Perturbation import decode_readback
    for data in (b"20000n", b"", packet()[:-1], packet()+b"x",
                 b"XX"+packet()[2:], packet()[:2]+b"\x02"+packet()[3:]):
        with pytest.raises(ValueError):
            decode_readback(data)


def test_new_connection_discovers_added_readback_characteristic_uncached(monkeypatch):
    import importlib
    module = importlib.import_module("services.QtExoDeviceManager")
    captured = []
    def factory(target, **kwargs):
        captured.append((target, kwargs))
        return "client"
    monkeypatch.setattr(module, "BleakClient", factory)
    callback = lambda client: None
    assert module._make_ble_client("exo", callback) == "client"
    assert captured == [("exo", dict(disconnected_callback=callback,
                                    winrt={"use_cached_services": False}))]


def test_duplicate_readbacks_do_not_refresh_deadlines_or_clear_unknown_status():
    application = app()
    from services.Perturbation import PerturbationSession
    from types import SimpleNamespace
    session = PerturbationSession(Device())
    session.set_running(True)
    session.check_firmware()
    session.receive(status())
    starts = []
    session._timer = SimpleNamespace(start=starts.append, stop=lambda: None)
    session.arm(1, 5, 200)
    session.receive(status(1, 1, 1, 5, 200))
    session.receive(status(1, 1, 1, 5, 200))
    assert starts == [3000, 12000]
    session._timeout()
    session.receive(status(1, 1, 1, 5, 200))
    assert "unknown" in session.message and session.busy and not session.ready
    session.receive(status(1, 1, 4, 5, 200))
    assert not session.busy
    session.reset()


def test_production_firmware_keeps_complete_readback_and_rejection_reason(tmp_path):
    compile_run(tmp_path, controller_context() + r'''
#define UARTMSG_H
struct UART_msg_t {uint8_t command=0,joint_id=0; float data[128]={}; uint8_t len=0;};
#include "AnklePerturbationProtocol.h"
int main(){
 using namespace ankle_perturbation;
 ExoData data;data.left_side.ankle.controller.controller=2;
 assert(rejection_reason(&data,1)==unsupported_controller);
 data.perturbation.begin_session(0);
 data.perturbation.request(1,5,200,3,0,false,rejection_reason(&data,1));
 Status s;assert(data.perturbation.pop(s) && s.state==rejected && s.reason==unsupported_controller);
 uint8_t bytes[20]={};assert(wire::pack_readback(s,bytes,sizeof(bytes))==12);
 assert(bytes[0]=='A' && bytes[1]=='P' && bytes[2]==1 && bytes[3]==3);
 assert(bytes[5]==1 && bytes[6]==5 && bytes[7]==unsupported_controller);
 Status decoded;assert(wire::unpack_status(wire::pack_status(s),decoded));
 assert(decoded.reason==unsupported_controller);
 data.left_side.ankle.controller.controller=3;
 data.right_side.do_calibration_toe_fsr=true;
 assert(rejection_reason(&data,1)==calibration_in_progress);
 data.right_side.do_calibration_toe_fsr=false;
 data.left_side.ankle.motor.enabled=false;
 assert(rejection_reason(&data,1)==motor_disabled);
}
''')
