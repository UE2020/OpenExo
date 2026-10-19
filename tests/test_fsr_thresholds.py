"""Runtime FSR configuration: actual Qt/session, wire and control-MCU tests."""
import struct
from test_perturbation_gui import app, Device


class ThresholdDevice(Device):
    def queryFsrThresholds(self):
        self.commands.append('probe')
        return True

    def sendFsrThresholds(self, operation, left, right, request_id):
        self.commands.append((operation, left, right, request_id))
        return True

    def readFsrThresholdStatus(self):
        return True


def reply(request_id=0, state=0, reason=0, editable=False,
          left_lower=.20, left_upper=.30, right_lower=.20, right_upper=.30):
    return dict(request_id=request_id, state=state, reason=reason,
                editable=editable, left_lower=left_lower, left_upper=left_upper,
                right_lower=right_lower, right_upper=right_upper)


def test_apply_requires_pause_disarm_and_matching_firmware_confirmation():
    application = app()
    from services.Perturbation import PerturbationSession
    from services.FsrThresholds import FsrThresholdSession
    device = ThresholdDevice()
    pulse = PerturbationSession(device)
    settings = FsrThresholdSession(device, pulse)
    events = []
    settings.event.connect(events.append)
    assert not settings.apply(.35, .45)
    settings.set_paused(True)
    assert settings.check()
    settings.receive(reply())  # capability is not a settings acknowledgement
    assert device.commands[-1] == (0, 0, 0, 1)
    settings.receive(reply(1, 1, editable=True))
    assert settings.ready
    assert settings.apply(.35, .45)
    assert device.commands[-1] == (1, .35, .45, 2)
    assert settings.confirmed == (.25, .25)
    assert not settings.ready
    settings.receive(reply(1, 2, editable=True))  # stale ID
    assert settings.pending
    settings.receive(reply(2, 2, editable=True, left_lower=.30,
                           left_upper=.40, right_lower=.40, right_upper=.50))
    assert settings.confirmed == (.35, .45) and settings.ready
    assert events[-1]['state'] == 2
    pulse.busy = True
    assert not settings.apply(.40, .40)
    pulse.busy = False
    settings.set_paused(False)
    assert not settings.apply(.40, .40)
    settings.reset()


def test_editor_routes_paused_apply_and_logs_only_confirmed_settings(monkeypatch, tmp_path):
    application = app()
    import logging
    from MainWindow import MainWindow
    from services.QtExoDeviceManager import QtExoDeviceManager
    monkeypatch.setattr(QtExoDeviceManager, '_setup_logging',
                        lambda self: setattr(self, 'logger', logging.getLogger('fsr-test')))
    monkeypatch.setattr(QtExoDeviceManager, '_ensure_connected', lambda self: True)
    writes = []
    monkeypatch.setattr(QtExoDeviceManager, '_queue_write_steps',
                        lambda self, steps, message: writes.append(steps))
    window = MainWindow()
    page = window.trial_page
    assert not page.btn_apply_fsr.isEnabled()
    window._on_device_stop_motors()
    window.qt_dev.fsrThresholdStatusReceived.emit(reply())
    window.qt_dev.fsrThresholdStatusReceived.emit(reply(1, 1, editable=True))
    assert page.btn_apply_fsr.isEnabled()
    page.spin_fsr_left.setValue(.35)
    page.spin_fsr_right.setValue(.45)
    assert '0.40' in page.lbl_fsr_preview.text()
    assert '0.50' in page.lbl_fsr_preview.text()
    page.btn_apply_fsr.click()
    assert writes[-1][0][0] == b'k'
    assert not page.btn_apply_fsr.isEnabled()
    assert not page.btn_pause_play.isEnabled()
    window._on_device_start()  # programmatic resume cannot bypass pending apply
    assert window._device_paused
    import json
    from types import SimpleNamespace
    recording = tmp_path/'recording.csv'
    window._csv_file = SimpleNamespace(name=str(recording))
    window.qt_dev.fsrThresholdStatusReceived.emit(reply(2, 2, editable=True,
        left_lower=.30, left_upper=.40, right_lower=.40, right_upper=.50))
    assert window.qt_dev._curr_left_fsr_value == .35
    assert 'Confirmed' in page.lbl_fsr_status.text()
    assert page.btn_pause_play.isEnabled()
    rows = (tmp_path/'recording.csv.fsr_thresholds.jsonl').read_text().splitlines()
    assert json.loads(rows[0])['event']['state'] == 2
    window._csv_file = None
    window.fsr_thresholds.reset()
    window.perturbation.reset()
    window.rt_bridge._data_rate_timer.stop()
    window.close()


def test_control_mcu_checks_pause_and_readbacks_actual_thresholds(tmp_path):
    from test_perturbation_firmware import compile_run, controller_context
    context = controller_context().replace(
        'toe_fsr_upper_threshold=.25f;',
        'toe_fsr_upper_threshold=.30f,toe_fsr_lower_threshold=.20f;')
    context = context.replace('ankle_perturbation::Engine perturbation;', '''
 ankle_perturbation::Engine perturbation;
 template<class F> void for_each_joint(F f,float* a=nullptr){
  for(auto s:{&left_side,&right_side}) for(auto j:{&s->ankle,&s->hip,&s->knee,&s->elbow,&s->arm_1,&s->arm_2}) f(j,a);
 }
''')
    compile_run(tmp_path, context + r'''
#define UARTMSG_H
struct UART_msg_t {uint8_t command=0,joint_id=0;float data[128]={};uint8_t len=0;};
#include "FsrThresholdUart.h"
struct Handler {UART_msg_t last; void UART_msg(const UART_msg_t& m){last=m;}};
int main(){
 using namespace fsr_thresholds;
 ExoData data;Handler h; UART_msg_t request;
 float values[4]={1,.35f,.45f,2}; assert(from_ble(values,4,request));
 assert(handle_uart(&h,&data,request)); Snapshot s;
 assert(unpack(h.last,s) && s.state==3 && s.reason==1);
 assert(data.left_side.toe_fsr_upper_threshold==.30f);
 data.user_paused=true;
 data.for_each_joint([](JointData* j,float*){j->motor.enabled=false;});
 assert(handle_uart(&h,&data,request));
 assert(unpack(h.last,s) && s.id==2 && s.state==2 && s.editable);
 assert(s.ll==30 && s.lu==40 && s.rl==40 && s.ru==50);
 uint8_t bytes[20]={}; assert(pack_readback(s,bytes,sizeof(bytes))==16);
 assert(bytes[0]=='F' && bytes[1]=='T' && bytes[3]==2 && bytes[5]==2);
 data.perturbation.begin_session(0);
 data.perturbation.request(1,2,100,1,0,true);
 assert(!data.perturbation.contact_ready(1,true));
 data.perturbation.tick(1,1,data.perturbation.contact_ready(1,true),true);
 assert(data.perturbation.offset(1)==0);
 assert(data.perturbation.contact_ready(1,false));
 data.perturbation.tick(2,1,data.perturbation.contact_ready(1,true),true);
 assert(data.perturbation.offset(1)==2);
 assert(handle_uart(&h,&data,request));
 assert(unpack(h.last,s) && s.state==3 && s.reason==2);
 data.perturbation.cancel(); data.right_side.do_calibration_toe_fsr=true;
 assert(handle_uart(&h,&data,request));
 assert(unpack(h.last,s) && s.state==3 && s.reason==4);
 data.right_side.do_calibration_toe_fsr=false;
 values[1]=NAN; assert(!from_ble(values,4,request));
 values[1]=.01f; assert(!from_ble(values,4,request));
 values[1]=.351f; assert(!from_ble(values,4,request));
 values[1]=.06f; values[2]=.94f; assert(from_ble(values,4,request));
 assert(handle_uart(&h,&data,request));
 assert(unpack(h.last,s) && s.ll==1 && s.lu==11 && s.rl==89 && s.ru==99);
}
''')


def test_readback_sender_keeps_settings_separate_from_legacy_notifications(tmp_path):
    from test_perturbation_firmware import compile_run, source_block
    sender = source_block('ExoBLE.cpp', 'bool ExoBLE::send_fsr_threshold_status(')
    compile_run(tmp_path, r'''
#include <cassert>
#include <vector>
#include "AnklePerturbation.h"
#define UARTMSG_H
struct UART_msg_t {uint8_t command=0,joint_id=0;float data[128]={};uint8_t len=0;};
#include "FsrThresholds.h"
struct Char {std::vector<uint8_t> data;int writeValue(const uint8_t* b,int n){data.assign(b,b+n);return 1;}};
struct ExoBLE {bool _connected=true;struct {Char TXChar,FsrThresholdStatusChar;} _gatt_db;
 bool send_fsr_threshold_status(const fsr_thresholds::Snapshot&);};
''' + sender + r'''
int main(){
 ExoBLE ble;fsr_thresholds::Snapshot s{9999,2,0,30,40,40,50,1};
 assert(ble.send_fsr_threshold_status(s));auto b=ble._gatt_db.FsrThresholdStatusChar.data;
 assert(b.size()==16 && b[0]=='F' && b[1]=='T' && b[3]==15 && b[4]==39);
 assert(b[7]==30 && b[9]==40 && b[11]==40 && b[13]==50 && b[15]==1);
 assert(ble._gatt_db.TXChar.data.empty());
 ble._connected=false;s.id=1;assert(!ble.send_fsr_threshold_status(s));
 assert(ble._gatt_db.FsrThresholdStatusChar.data==b);
}
''')


def test_timeout_never_retries_apply_and_requires_a_new_confirmed_read():
    application = app()
    from services.Perturbation import PerturbationSession
    from services.FsrThresholds import FsrThresholdSession
    device = ThresholdDevice()
    pulse = PerturbationSession(device)
    settings = FsrThresholdSession(device, pulse)
    settings.set_paused(True)
    settings.check()
    settings.receive(reply())
    settings.receive(reply(1, 1, editable=True))
    settings.apply(.35, .45)
    commands = list(device.commands)
    settings.poll_status()
    settings._timeout()
    assert device.commands == commands
    assert settings.uncertain and not settings.ready
    settings.receive(reply(2, 2, editable=True))  # too late to unlock by assumption
    assert settings.uncertain
    settings.check()
    settings.receive(reply())
    settings.receive(reply(3, 1, editable=True, left_lower=.30,
                           left_upper=.40, right_lower=.40, right_upper=.50))
    assert not settings.uncertain and settings.ready
    settings.reset()
    # A missing optional feature on old firmware must not lock ordinary assistance.
    settings.set_paused(True)
    settings.check()
    settings._timeout()
    assert not settings.uncertain
    settings.reset()


def test_binary_status_and_device_transport_reject_stale_or_invalid_data(monkeypatch):
    application = app()
    import asyncio
    import logging
    import pytest
    from services.FsrThresholds import decode_status
    from services.QtExoDeviceManager import QtExoDeviceManager, FSR_THRESHOLD_STATUS_UUID
    data = struct.pack('<2sBHBBhhhhB', b'FT', 1, 9, 2, 0, 30, 40, 40, 50, 1)
    assert decode_status(data)['left_upper'] == .40
    for bad in (data[:-1], b'', b'XX'+data[2:], data[:2]+b'\x02'+data[3:]):
        with pytest.raises(ValueError):
            decode_status(bad)
    monkeypatch.setattr(QtExoDeviceManager, '_setup_logging',
                        lambda self: setattr(self, 'logger', logging.getLogger('read-fsr-test')))
    manager = QtExoDeviceManager()
    received = []
    manager.fsrThresholdStatusReceived.connect(received.append)
    class Client:
        class services:
            @staticmethod
            def get_characteristic(uuid):
                assert uuid == FSR_THRESHOLD_STATUS_UUID
                return object()
        async def read_gatt_char(self, uuid, *, use_cached):
            assert uuid == FSR_THRESHOLD_STATUS_UUID and not use_cached
            return data
    client = Client()
    monkeypatch.setattr(manager, '_capture_session', lambda: (client, 3))
    monkeypatch.setattr(manager, '_is_current_session', lambda *args: True)
    monkeypatch.setattr(manager, '_submit', lambda coro: (asyncio.run(coro), True)[1])
    assert manager.readFsrThresholdStatus()
    assert received[-1]['request_id'] == 9
    states = iter((True, False))
    monkeypatch.setattr(manager, '_is_current_session', lambda *args: next(states))
    assert manager.readFsrThresholdStatus()
    assert len(received) == 1
    writes = []
    monkeypatch.setattr(manager, '_ensure_connected', lambda: True)
    monkeypatch.setattr(manager, '_queue_write_steps', lambda steps, msg: writes.append(steps))
    assert manager.queryFsrThresholds() and writes[-1] == [(b'K', True, 0.0)]
    assert manager.sendFsrThresholds(1, .35, .45, 9999)
    assert [step[0] for step in writes[-1]] == [b'k'] + [
        struct.pack('<d', v) for v in (1, .35, .45, 9999)]
    before = len(writes)
    for left in (float('nan'), float('inf'), 0, .05, .95, .351):
        assert not manager.sendFsrThresholds(1, left, .45, 1)
    assert len(writes) == before


def test_reconnect_read_is_not_blocked_or_cleared_by_old_read(monkeypatch):
    application = app()
    import asyncio
    import logging
    from services.QtExoDeviceManager import QtExoDeviceManager
    monkeypatch.setattr(QtExoDeviceManager, '_setup_logging',
                        lambda self: setattr(self, 'logger', logging.getLogger('reconnect-fsr')))
    manager = QtExoDeviceManager()
    received = []
    manager.fsrThresholdStatusReceived.connect(received.append)
    async def scenario():
        gates = {1: asyncio.Event(), 2: asyncio.Event()}
        class Client:
            class services:
                @staticmethod
                def get_characteristic(uuid):
                    return object()
            def __init__(self, generation):
                self.generation = generation
            async def read_gatt_char(self, uuid, **kwargs):
                await gates[self.generation].wait()
                return struct.pack('<2sBHBBhhhhB', b'FT', 1, self.generation,
                                   1, 0, 20, 30, 20, 30, 1)
        current = [Client(1), 1]
        tasks = []
        monkeypatch.setattr(manager, '_capture_session', lambda: tuple(current))
        monkeypatch.setattr(manager, '_is_current_session',
                            lambda client, generation: client is current[0] and generation == current[1])
        def submit(coro):
            task = asyncio.create_task(coro)
            tasks.append(task)
            return task
        monkeypatch.setattr(manager, '_submit', submit)
        assert manager.readFsrThresholdStatus()
        await asyncio.sleep(0)
        current[:] = [Client(2), 2]
        assert manager.readFsrThresholdStatus()
        await asyncio.sleep(0)
        gates[1].set()
        await tasks[0]
        assert not manager.readFsrThresholdStatus()  # new read still owns its guard
        gates[2].set()
        await tasks[1]
        assert [event['request_id'] for event in received] == [2]
    asyncio.run(scenario())


def test_rejected_read_cannot_clear_an_ambiguous_apply_or_replace_last_confirmation():
    application = app()
    from services.Perturbation import PerturbationSession
    from services.FsrThresholds import FsrThresholdSession
    device = ThresholdDevice()
    settings = FsrThresholdSession(device, PerturbationSession(device))
    settings.set_paused(True)
    settings.check()
    settings.receive(reply())
    settings.receive(reply(1, 1, editable=True))
    settings.apply(.35, .45)
    settings._timeout()
    settings.check()
    settings.receive(reply())
    assert settings.uncertain and settings.pending  # capability alone proves no values
    settings.receive(reply(3, 3, reason=3, left_lower=0, left_upper=0,
                           right_lower=0, right_upper=0))
    assert settings.uncertain and settings.confirmed == (.25, .25)
    assert not settings.ready
    settings.check()
    settings.receive(reply())
    settings.receive(reply(4, 1, editable=True, left_lower=.30,
                           left_upper=.40, right_lower=.40, right_upper=.50))
    assert not settings.uncertain and settings.confirmed == (.35, .45)
    settings.reset()


def test_mock_supports_confirmed_threshold_configuration_without_hardware():
    application = app()
    from mock_perturbation import MockExo
    from services.Perturbation import PerturbationSession
    from services.FsrThresholds import FsrThresholdSession
    device = MockExo()
    pulse = PerturbationSession(device)
    settings = FsrThresholdSession(device, pulse)
    device.fsrThresholdStatusReceived.connect(settings.receive)
    settings.set_paused(True)
    settings.check()
    assert settings.confirmed == (.25, .25) and not settings.ready
    device.set_paused(True)
    settings.check()
    assert settings.ready and settings.apply(.35, .45)
    assert settings.confirmed == (.35, .45)
    device.set_paused(False)
    assert settings.apply(.40, .40)
    assert 'Rejected' in settings.message and settings.confirmed == (.35, .45)
    settings.reset()


def test_production_pulse_trigger_uses_forefoot_even_if_heel_channel_is_active(tmp_path):
    from test_perturbation_firmware import compile_run, controller_context, source_block
    context = controller_context().replace('ground_strike=false;', 'ground_strike=false,toe_strike=false;')
    cycle = source_block('Side.cpp', 'void Side::run_side()')
    event = cycle[cycle.index('    const uint8_t pulse_side'):cycle.index('    //Calculates')]
    compile_run(tmp_path, context + '''
struct Side {ExoData* _data;SideData* _side_data;bool _is_left;void event();};
void Side::event(){
''' + event + r'''
}
int main(){
 ExoData data;Side side{&data,&data.left_side,true};
 data.perturbation.begin_session(0);data.perturbation.request(1,2,100,1,0,true);
 data.left_side.ground_strike=true;data.left_side.toe_strike=false;
 side.event();assert(data.perturbation.offset(1)==0); // heel-only event cannot fire
 data.left_side.heel_stance=true;data.left_side.ground_strike=false;
 data.left_side.toe_stance=true;data.left_side.toe_strike=true;
 side.event();assert(data.perturbation.offset(1)==2); // unused heel cannot suppress forefoot
}
''')
