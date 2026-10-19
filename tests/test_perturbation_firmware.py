"""Compile and execute production firmware logic on the host (no hardware)."""
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "ExoCode/src"


def source_block(filename, signature):
    text = (SRC / filename).read_text(encoding="utf-8-sig")
    start = text.index(signature)
    opening = text.index("{", start)
    depth, end = 1, opening + 1
    while depth:
        depth += (text[end] == "{") - (text[end] == "}")
        end += 1
    return text[start:end]


def controller_context():
    config = (SRC / "Config.h").read_text()
    import re
    freq = re.search(r"#define LOOP_FREQ_HZ\s+(\S+)", config)[1]
    tolerance = re.search(r"#define LOOP_TIME_TOLERANCE\s+(\S+)", config)[1]
    return "\n".join([
        '#include "AnklePerturbation.h"\n#include <cassert>\n#include <cmath>\n#include <initializer_list>',
        f'#define LOOP_FREQ_HZ {freq}\n#define LOOP_TIME_TOLERANCE {tolerance}',
        'namespace status_defs {' + source_block("StatusDefs.h", "namespace messages") + '}',
        'namespace config_defs {' + source_block("ParseIni.h", "enum class ankle_controllers") + ';}',
        'namespace controller_defs {' + source_block("ControllerData.h", "namespace proportional_joint_moment")
        + source_block("ControllerData.h", "namespace pjmc_plus") + '}',
        r'''
template<class A,class B> auto min(A a,B b){return a<b?a:b;}
template<class A,class B> auto max(A a,B b){return a>b?a:b;}
template<class A,class B,class C> auto constrain(A a,B b,C c){return min(max(a,b),c);}
struct SerialStub {template<class T> void print(T) {}} Serial;
uint32_t now_ms=0;
uint32_t millis(){return now_ms;}
struct FakeTime {float tick(float){return 2000;}} fake_time;
namespace utils {float ewma(float a,float b,float c){return a*c+b*(1-c);}}
struct ControllerData {
 uint8_t controller=3;
 float parameters[20]={},filtered_torque_reading=0,max_measured=0,max_setpoint=0;
 float prev_max_measured=0,prev_max_setpoint=0,kf=1,filtered_setpoint=0;
 float ff_setpoint=0,filtered_cmd=0,desired_torque=0,setpoint=0;
};
struct JointData {
 bool is_used=true,is_left=true,calibrate_torque_sensor=false,calibrate_angle_sensor=false;
 float torque_reading=0,torque_offset_reading=1;
 ControllerData controller;
 struct {bool enabled=true,do_zero=false; int timeout_count=0,timeout_count_max=40;} motor;
};
struct SideData {
 bool is_used=true,toe_stance=false,heel_stance=false,ground_strike=false;
 bool do_calibration_toe_fsr=false,do_calibration_heel_fsr=false;
 bool do_calibration_refinement_toe_fsr=false,do_calibration_refinement_heel_fsr=false;
 bool reset_fsr_calibration=false;
 float toe_fsr=0,heel_fsr=0,toe_fsr_upper_threshold=.25f;
 JointData ankle,hip,knee,elbow,arm_1,arm_2;
};
#define ExoData_h
struct ExoData {
 bool estop=false,user_paused=false; int error_code=0;
 uint16_t status=status_defs::messages::trial_on;
 uint16_t get_status(){return status;}
 SideData left_side,right_side;
 ankle_perturbation::Engine perturbation;
};
#include "AnklePerturbationSupport.h"
struct _Controller {
 ExoData* _data; JointData* _joint_data; SideData* _side_data; ControllerData* _controller_data;
 FakeTime* _t_helper=&fake_time; float _t_helper_context=0;
 float _pid_error_sum=0,_prev_input=0,_prev_de_dt=0;
 float _pid(float,float,float,float,float);
 float _pjmc_generic(float,float,float,float);
};
struct ProportionalJointMoment:_Controller {float calc_motor_cmd();};
struct PJMC_PLUS:_Controller {float calc_motor_cmd();};
''',
        source_block("Controller.cpp", "float _Controller::_pid("),
        source_block("Controller.cpp", "float _Controller::_pjmc_generic("),
        source_block("Controller.cpp", "float ProportionalJointMoment::calc_motor_cmd("),
        source_block("Controller.cpp", "float PJMC_PLUS::calc_motor_cmd("),
    ])


def test_real_pjmc_and_pid_track_additive_pulse_and_return_to_assistance(tmp_path):
    compile_run(tmp_path, controller_context() + r'''
int main(){
 ExoData data; auto& side=data.left_side; auto& joint=side.ankle; auto& ctl=joint.controller;
 ProportionalJointMoment c; c._data=&data; c._side_data=&side; c._joint_data=&joint; c._controller_data=&ctl;
 ctl.parameters[0]=10; ctl.parameters[1]=0; ctl.parameters[2]=1;
 ctl.parameters[3]=1; ctl.parameters[4]=.5f;
 for(bool stance:{false,true}) {
   side.toe_stance=stance; side.toe_fsr=.8f;
   float baseline=c.calc_motor_cmd();
   data.perturbation.begin_session(0);
   assert(ankle_perturbation::allowed(&data,1));
   data.perturbation.request(1,2,100,stance ? 2 : 1,0,true);
   data.perturbation.tick(1,1,true,true);
   float with_pulse=c.calc_motor_cmd();
   assert(std::abs(with_pulse-baseline-3)<.0001f); // +2 Nm target + P(.5)*2
   assert(data.perturbation.offset(2)==0);
   data.perturbation.tick(101,1,false,true);
   assert(c.calc_motor_cmd()==baseline);
 }
 // Reaching a supported controller is required; do not silently bypass it.
 for(int id:{1,4,5,6,7,8,9,10,12}) {
   ctl.controller=id; assert(!ankle_perturbation::allowed(&data,1));
 }
 ctl.controller=3;
 joint.motor.enabled=false; assert(!ankle_perturbation::allowed(&data,1)); joint.motor.enabled=true;
 data.estop=true; assert(!ankle_perturbation::allowed(&data,1)); data.estop=false;
 data.right_side.do_calibration_toe_fsr=true; assert(!ankle_perturbation::allowed(&data,1));
 data.right_side.do_calibration_toe_fsr=false;
 data.status=status_defs::messages::trial_off; assert(!ankle_perturbation::allowed(&data,1));
}
''')


def test_wire_codec_preserves_large_ids_and_hundredth_nm(tmp_path):
    compile_run(tmp_path, r'''
#include "AnklePerturbation.h"
#include <cassert>
#include <cstring>
#define UARTMSG_H
struct UART_msg_t {uint8_t command=0,joint_id=0; float data[128]={}; uint8_t len=0;};
#include "AnklePerturbationProtocol.h"
int main(){
 using namespace ankle_perturbation;
 for(int id:{1,255,256,9999}) for(int cents:{-999,-256,-1,1,256,999}) {
   Status s{(uint16_t)id,2,armed,cents/100.0f,200};
   auto msg=wire::pack_status(s);
   assert(msg.len==7);
   for(int i=0;i<msg.len;++i) {
     assert(fabsf(msg.data[i])<=255);
     // Actual UART representation is signed short fixed point *100.
     msg.data[i]=int16_t(msg.data[i]*100)/100.0f;
   }
   Status roundtrip;
   assert(wire::unpack_status(msg,roundtrip));
   assert(roundtrip.request_id==id && roundtrip.magnitude==s.magnitude && roundtrip.duration_ms==200);
   char frame[80];
   assert(wire::format_status(roundtrip,frame,sizeof(frame))>0);
   char expected[80];
   snprintf(expected,sizeof(expected),"Sp5c%dn200n100n%dn20000n",id*100,cents);
   assert(strcmp(frame,expected)==0);
 }
 float fields[4]={1,2.56f,100,9999}; UART_msg_t request;
 assert(wire::from_ble(fields,4,request));
 assert(request.joint_id==39 && request.data[0]==15 && request.data[1]==1 && request.data[2]==2.56f);
 fields[0]=1.5f; assert(!wire::from_ble(fields,4,request));
}
''')


def test_uart_request_to_real_controller_lifecycle_and_legacy_cancel(tmp_path):
    context = controller_context() + r'''
#include <vector>
#define UARTMSG_H
struct UART_msg_t {uint8_t command=0,joint_id=0; float data[128]={}; uint8_t len=0;};
struct FakeUart {std::vector<UART_msg_t> sent; void UART_msg(UART_msg_t msg){sent.push_back(msg);}};
#include "AnklePerturbationUart.h"
'''
    compile_run(tmp_path, context + r'''
int main(){
 using namespace ankle_perturbation;
 ExoData data; FakeUart uart; Status event;
 UART_msg_t msg{}; msg.command=wire::query;
 assert(handle_uart(&uart,&data,msg));
 assert(wire::unpack_status(uart.sent.back(),event) && event.request_id==0 && event.state==0);
 float fields[4]={1,-2.56f,100,9999};
 assert(wire::from_ble(fields,4,msg));
 assert(handle_uart(&uart,&data,msg));
 assert(wire::unpack_status(uart.sent.back(),event) && event.request_id==9999 && event.state==armed);
 data.perturbation.tick(1,1,true,allowed(&data,1));
 flush_events(&uart,&data);
 assert(wire::unpack_status(uart.sent.back(),event) && event.state==active);
 auto& side=data.left_side; auto& joint=side.ankle;
 ProportionalJointMoment c; c._data=&data;c._side_data=&side;c._joint_data=&joint;c._controller_data=&joint.controller;
 joint.controller.parameters[3]=1;joint.controller.parameters[4]=.5f;
 assert(fabsf(c.calc_motor_cmd()-(-3.84f))<.001f);
 msg.command=0x0E; msg.len=1; msg.data[0]=0;
 assert(!handle_uart(&uart,&data,msg)); // legacy motor-off handler still gets the command
 assert(wire::unpack_status(uart.sent.back(),event) && event.state==cancelled);
 assert(c.calc_motor_cmd()==0);
 // Legacy sessions never negotiate or emit any additional status/command.
 ExoData legacy; FakeUart old;
 msg.command=0x11;msg.len=0;
 assert(!handle_uart(&old,&legacy,msg));
 flush_events(&old,&legacy);
 assert(old.sent.empty() && legacy.perturbation.offset(1)==0);
}
''')


def test_production_wiring_covers_ble_uart_gait_and_control_loop():
    ble = (SRC / "ble_commands.h").read_text()
    coms = (SRC / "ComsMCU.cpp").read_text()
    uart = (SRC / "uart_commands.h").read_text()
    side = source_block("Side.cpp", "void Side::run_side()")
    exo = source_block("Exo.cpp", "bool Exo::run()")
    assert "perturbation_query" in ble and "perturbation_request" in ble
    assert "wire::from_ble" in coms and "send_perturbation_status" in coms
    assert "ankle_perturbation::handle_uart(handler, exo_data, msg)" in uart
    assert side.index("read_data();") < side.index("perturbation.tick") < side.index("update_motor_cmds();")
    assert "ankle_perturbation::flush_events(handler, data)" in exo
    assert "perturbation.tick" in exo


def test_real_ble_status_sender_chunks_every_byte_without_legacy_changes(tmp_path):
    body = source_block("ExoBLE.cpp", "bool ExoBLE::send_perturbation_status(")
    code = r'''
#include "AnklePerturbation.h"
#define UARTMSG_H
struct UART_msg_t {uint8_t command=0,joint_id=0; float data[128]={}; uint8_t len=0;};
#include "AnklePerturbationProtocol.h"
#include <cassert>
#include <string>
#include <vector>
void delay(int){}
struct Char {std::vector<std::string> chunks; int writeValue(const uint8_t* p,int n){assert(n<=20);chunks.emplace_back((const char*)p,n);return 1;}};
struct ExoBLE {bool _connected=true;struct {Char TXChar, PerturbationStatusChar;} _gatt_db;
 bool send_perturbation_status(const ankle_perturbation::Status& event);
};
'''
    compile_run(tmp_path, code + body + r'''
int main(){
 ExoBLE ble; ankle_perturbation::Status event{9999,2,1,-9.99f,200};
 assert(ble.send_perturbation_status(event));
 std::string output;for(auto& chunk:ble._gatt_db.TXChar.chunks)output+=chunk;
 assert(output=="Sp5c999900n200n100n-999n20000n");
 assert(ble._gatt_db.PerturbationStatusChar.chunks.size()==1);
 assert(ble._gatt_db.PerturbationStatusChar.chunks[0].size()==12);
 ble._connected=false;assert(!ble.send_perturbation_status(event));
}
''')


def test_real_ground_strike_uses_heel_or_toe_swing_to_contact_not_toe_off(tmp_path):
    body = source_block("Side.cpp", "bool Side::_check_ground_strike()")
    compile_run(tmp_path, r'''
#include <cassert>
#include <cstdint>
uint32_t millis(){return 100;}
struct SideData {bool prev_heel_stance=false,prev_toe_stance=false,heel_stance=false,toe_stance=false,toe_strike=false;};
struct FSR {bool value=false;bool get_ground_contact(){return value;}};
struct Side {SideData data;SideData* _side_data=&data;FSR _heel_fsr,_toe_fsr;
 bool _prev_heel_contact_state=false,_prev_toe_contact_state=false;
 uint32_t _ground_strike_timestamp=0,_prev_ground_strike_timestamp=0;
 bool _check_ground_strike();};
''' + body + r'''
int main(){
 Side s;
 assert(!s._check_ground_strike());
 s._heel_fsr.value=true;assert(s._check_ground_strike());
 assert(!s._check_ground_strike());
 s._toe_fsr.value=true;assert(!s._check_ground_strike());
 s._heel_fsr.value=false;assert(!s._check_ground_strike());
 s._toe_fsr.value=false;assert(!s._check_ground_strike());
 s._toe_fsr.value=true;assert(s._check_ground_strike());
}
''')


def test_pjmc_plus_pid_and_negative_increment_preserve_base_gains(tmp_path):
    compile_run(tmp_path, controller_context() + r'''
int main(){
 ExoData data;auto& side=data.right_side;auto& j=side.ankle;auto& ctl=j.controller;
 ctl.controller=11;ctl.parameters[0]=10;ctl.parameters[1]=1;ctl.parameters[controller_defs::pjmc_plus::kp]=.5f;
 PJMC_PLUS c;c._data=&data;c._joint_data=&j;c._side_data=&side;c._controller_data=&ctl;
 assert(ankle_perturbation::allowed(&data,2));
 float baseline=c.calc_motor_cmd();
 data.perturbation.begin_session(0);data.perturbation.request(2,-2,50,1,0,true);
 data.perturbation.tick(1,2,true,true);
 assert(fabsf(c.calc_motor_cmd()-baseline+3)<.001f);
 data.perturbation.tick(51,2,false,true);
 assert(c.calc_motor_cmd()==baseline);
 assert(ctl.controller==11 && ctl.parameters[0]==10 && ctl.parameters[controller_defs::pjmc_plus::kp]==.5f);
}
''')


def test_real_nano_dispatch_negotiation_and_disconnect_supervision(tmp_path):
    dispatch = source_block("ComsMCU.cpp", "void ComsMCU::_process_complete_gui_command(")
    new_cases = dispatch[dispatch.index("    case ble_names::perturbation_query:"):
                         dispatch.index("    case ble_names::reset_system:")]
    code = r'''
#include "AnklePerturbation.h"
#define UARTMSG_H
struct UART_msg_t {uint8_t command=0,joint_id=0;float data[128]={};uint8_t len=0;};
#include "AnklePerturbationProtocol.h"
#include "FsrThresholds.h"
#include <cassert>
#include <vector>
uint32_t now=0;uint32_t millis(){return now;}
struct ExoData {};
struct BleMessage {char command=0;bool is_complete=true;int expecting=0;float data[32]={};};
namespace ble_queue {std::vector<BleMessage> queue;int size(){return queue.size();}
 BleMessage pop(){auto m=queue.front();queue.erase(queue.begin());return m;}}
struct UARTHandler {
 std::vector<UART_msg_t> sent,received;
 static UARTHandler* get_instance(){static UARTHandler instance;return &instance;}
 void UART_msg(UART_msg_t msg){sent.push_back(msg);}
 UART_msg_t poll(float){if(received.empty())return UART_msg_t{};auto m=received.front();received.erase(received.begin());return m;}
};
struct Time_Helper {static Time_Helper* get_instance(){static Time_Helper t;return &t;}
 float generate_new_context(){return 0;}float tick(float){return 1000;}};
namespace UART_times {constexpr int UPDATE_PERIOD=0,COMS_MCU_TIMEOUT=0;}
namespace UART_command_utils {void handle_msg(UARTHandler*,ExoData*,UART_msg_t){}}
struct ExoBLE {bool connected_flag=true;std::vector<ankle_perturbation::Status> events;
 std::vector<fsr_thresholds::Snapshot> settings;
 void clear_fsr_threshold_status(){}
 bool send_fsr_threshold_status(const fsr_thresholds::Snapshot& s){settings.push_back(s);return true;}
 bool connected(){return connected_flag;}bool handle_updates(){return ble_queue::size();}
 bool send_perturbation_status(const ankle_perturbation::Status& s){events.push_back(s);return true;}};
struct ComsMCU {
 ExoData* _data=nullptr;ExoBLE ble;ExoBLE* _exo_ble=&ble;
 bool _perturbation_requested=false,_perturbation_ready=false;uint32_t _last_perturbation_link_ms=0;
 bool _fsr_requested=false,_fsr_ready=false;
 void _send_param_update_ack(UART_msg_t){}
 void _process_complete_gui_command(BleMessage* msg);
 void handle_ble();void update_UART();
};
'''
    code += source_block("ble_commands.h", "namespace ble_names")
    code += source_block("uart_commands.h", "namespace UART_command_names")
    code += "void ComsMCU::_process_complete_gui_command(BleMessage* msg){switch(msg->command){" + new_cases + "default:break;}}"
    code += source_block("ComsMCU.cpp", "void ComsMCU::handle_ble()")
    code += source_block("ComsMCU.cpp", "void ComsMCU::update_UART()")
    compile_run(tmp_path, code + r'''
int main(){
 using namespace ankle_perturbation;
 auto* uart=UARTHandler::get_instance();ComsMCU c;
 c.handle_ble();assert(uart->sent.empty()); // no new traffic with an old GUI
 BleMessage arm;arm.command='j';arm.expecting=4;arm.data[0]=2;arm.data[1]=-2.56f;arm.data[2]=100;arm.data[3]=9999;
 c._process_complete_gui_command(&arm);assert(uart->sent.empty());
 BleMessage query;query.command='J';
 c._process_complete_gui_command(&query);
 assert(uart->sent.back().command==wire::query && !c._perturbation_ready);
 auto n=uart->sent.size();c._process_complete_gui_command(&arm);assert(uart->sent.size()==n);
 uart->received.push_back(wire::pack_status(Status{}));c.update_UART();
 assert(c._perturbation_ready && c.ble.events.size()==1);
 c._process_complete_gui_command(&arm);
 assert(uart->sent.back().command==wire::request && uart->sent.back().joint_id==39);
 now=100;c.handle_ble();assert(uart->sent.back().command==wire::link && uart->sent.back().data[0]==1);
 c.ble.connected_flag=false;c.handle_ble();
 assert(uart->sent.back().command==wire::link && uart->sent.back().data[0]==0);
 assert(!c._perturbation_ready && !c._perturbation_requested);
 n=uart->sent.size();c._process_complete_gui_command(&arm);assert(uart->sent.size()==n);
 c.ble.connected_flag=true;BleMessage config;config.command='k';config.expecting=4;
 config.data[0]=1;config.data[1]=.35f;config.data[2]=.45f;config.data[3]=9999;
 c._process_complete_gui_command(&config);assert(uart->sent.size()==n);
 BleMessage probe;probe.command='K';c._process_complete_gui_command(&probe);
 assert(uart->sent.back().command==fsr_thresholds::query && !c._fsr_ready);
 n=uart->sent.size();c._process_complete_gui_command(&config);assert(uart->sent.size()==n);
 uart->received.push_back(fsr_thresholds::pack(fsr_thresholds::Snapshot{}));c.update_UART();
 assert(c._fsr_ready && c.ble.settings.size()==1);
 c._process_complete_gui_command(&config);
 assert(uart->sent.back().command==fsr_thresholds::request && uart->sent.back().joint_id==39);
 c.ble.connected_flag=false;c.handle_ble();assert(!c._fsr_ready && !c._fsr_requested);
}
''')


def test_inactive_pjmc_matches_no_overlay_on_all_saved_trial_inputs(tmp_path):
    import csv
    import json
    import pytest
    paths = sorted((ROOT / "Python_GUI/Saved_Data").glob("*.csv"))
    if not paths:
        pytest.skip("Local lab recordings are not bundled in this checkout")
    expected = 0
    for path in paths:
        with path.open(newline="") as source:
            expected += sum(1 for _ in csv.DictReader(source))
    baseline = source_block("Controller.cpp", "float ProportionalJointMoment::calc_motor_cmd(")
    baseline = baseline.replace("ProportionalJointMoment::", "LegacyPJMC::").replace(
        "    cmd_ff += ankle_perturbation::offset(_data, _joint_data);", "")
    parameters = (ROOT / "SDCard_asfound_20260825/ankleControllers/PJMC.csv").read_text().splitlines()[5]
    files = ",".join(json.dumps(p.as_posix()) for p in paths)
    code = controller_context() + r'''
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include <cstring>
struct LegacyPJMC:_Controller {float calc_motor_cmd();};
''' + baseline + '\nint main(){const char* paths[]={' + files + '};\n'
    code += 'float parameters[12]={' + parameters + '};\n'
    code += r'''
 size_t rows=0;
 for(const char* path:paths){
   ExoData data,reference;
   auto setup=[&](auto& c,ExoData& d,bool left){
     c._data=&d;c._side_data=left?&d.left_side:&d.right_side;
     c._joint_data=&c._side_data->ankle;c._controller_data=&c._joint_data->controller;
     for(int i=0;i<12;++i)c._controller_data->parameters[i]=parameters[i];
   };
   ProportionalJointMoment actual[2];LegacyPJMC legacy[2];
   for(int side=0;side<2;++side){setup(actual[side],data,side==0);setup(legacy[side],reference,side==0);}
   std::ifstream input(path);assert(input.good());std::string line;
   std::getline(input,line);
   while(std::getline(input,line)){
     std::vector<float> row;std::stringstream fields(line);std::string value;
     while(std::getline(fields,value,','))row.push_back(std::stof(value));
     assert(row.size()>=10);
     for(int side=0;side<2;++side){
       const int offset=side*2;
       for(auto* c:{(_Controller*)&actual[side],(_Controller*)&legacy[side]}){
         c->_side_data->toe_fsr=row[6+offset];c->_side_data->toe_stance=row[7+offset]>.5f;
         c->_joint_data->torque_reading=row[3+offset];
       }
       const float a=actual[side].calc_motor_cmd(),b=legacy[side].calc_motor_cmd();
       assert(std::memcmp(&a,&b,sizeof(float))==0);
       assert(actual[side]._controller_data->desired_torque==legacy[side]._controller_data->desired_torque);
     }
     ++rows;
   }
 }
 std::cout<<rows;
}
'''
    assert compile_run(tmp_path, code).strip() == str(expected)


def test_repeated_capability_probe_cannot_reuse_a_completed_request_id(tmp_path):
    compile_run(tmp_path, r'''
#include "AnklePerturbation.h"
#include <cassert>
using namespace ankle_perturbation;
int main(){
 Engine pulse;Status event;
 pulse.begin_session(0);pulse.request(1,2,10,1,0,true);pulse.pop(event);
 pulse.tick(1,1,true,true);pulse.pop(event);
 pulse.begin_session(2); // probe during pulse must not rearm, erase IDs, or restart it
 pulse.tick(11,1,false,true);
 assert(pulse.pop(event) && event.state==completed);
 pulse.begin_session(12);
 pulse.request(1,2,10,1,12,true);
 assert(pulse.pop(event) && event.state==completed);
 pulse.tick(13,1,true,true);
 assert(pulse.offset(1)==0);
}
''')


def compile_run(tmp_path, code):
    source = tmp_path / "test.cpp"
    binary = tmp_path / "test.exe"
    source.write_text('#include <initializer_list>\n' + code, encoding="utf-8")
    compiler = shutil.which("g++") or shutil.which("clang++")
    assert compiler, "Install a C++ compiler to run firmware host tests"
    result = subprocess.run([compiler, "-std=c++17", "-O0", "-I", str(SRC),
                             str(source), "-o", str(binary)], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr
    result = subprocess.run([str(binary)], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr
    return result.stdout


def test_engine_interlocks_duplicates_expiry_disconnect_and_clock_wrap(tmp_path):
    compile_run(tmp_path, r'''
#include "AnklePerturbation.h"
#include <cassert>
#include <limits>
using namespace ankle_perturbation;
int main() {
    Engine pulse; Status event;
    pulse.request(1, 2, 50, 1, 0, true);
    assert(pulse.pop(event) && event.state==rejected); // no negotiated session
    pulse.begin_session(1);
    pulse.request(1, 2, 50, 2, 1, false);
    assert(pulse.pop(event) && event.state==rejected); // disabled/calibration/error
    pulse.request(1, 2, 50, 3, 2, true);
    assert(pulse.pop(event) && event.state==armed);
    pulse.request(2, 4, 50, 4, 3, true);
    assert(pulse.pop(event) && event.state==rejected); // one pending side only
    pulse.tick(4, 1, true, true);
    assert(pulse.pop(event) && event.state==active && event.side==1);
    pulse.request(1, 2, 50, 3, 5, true); // duplicate never extends or restarts
    assert(pulse.pop(event) && event.state==active);
    pulse.tick(54, 1, true, true);
    assert(pulse.pop(event) && event.state==completed);
    pulse.request(1, 2, 50, 3, 55, true);
    assert(pulse.pop(event) && event.state==completed);
    pulse.tick(56, 1, true, true);
    assert(pulse.offset(1)==0);
    pulse.request(2, -3, 50, 5, 60, true);
    pulse.pop(event);
    pulse.heartbeat(false, 61);
    assert(pulse.pop(event) && event.state==cancelled);
    assert(pulse.offset(2)==0);
    pulse.begin_session(70);
    pulse.request(1, 2, 50, 1, 70, true);
    pulse.pop(event);
    pulse.tick(571, 1, true, true); // 500ms Nano watchdog
    assert(pulse.pop(event) && event.state==cancelled);
    assert(pulse.offset(1)==0);
    pulse.begin_session(1000);
    pulse.request(1, 2, 100, 1, 1000, true);
    pulse.pop(event);
    for(uint32_t t=1100;t<=11000;t+=100) {
        pulse.heartbeat(true,t); pulse.tick(t,1,false,true);
    }
    assert(pulse.pop(event) && event.state==expired);
    pulse.begin_session(UINT32_MAX-50);
    pulse.request(2,-2,100,1,UINT32_MAX-50,true); pulse.pop(event);
    pulse.tick(UINT32_MAX-40,2,true,true); pulse.pop(event);
    pulse.tick(58,2,false,true); assert(pulse.offset(2)==-2);
    pulse.tick(59,2,false,true); assert(pulse.offset(2)==0);
    assert(pulse.pop(event) && event.state==completed);
    pulse.request(2,2,100,2,60,true); pulse.pop(event);
    pulse.tick(61,2,true,false);
    assert(pulse.pop(event) && event.state==cancelled && pulse.offset(2)==0);
}
''')


def test_engine_rejects_invalid_wire_fields_without_narrowing(tmp_path):
    compile_run(tmp_path, r'''
#include "AnklePerturbation.h"
#include <cassert>
#include <limits>
using namespace ankle_perturbation;
int main() {
    Engine pulse; Status event; pulse.begin_session(0);
    float bad[][4]={{3,2,100,1},{1,0,100,2},{1,11,100,3},{1,2,9,4},
        {1,2,201,5},{1,2,10.5f,6},{1.5f,2,100,7},{1,2,100,1.5f},
        {1,std::numeric_limits<float>::quiet_NaN(),100,8},{1,2,100,10000}};
    for(auto& fields:bad) {
        assert(!valid_request(fields[0],fields[1],fields[2],fields[3]));
    }
    assert(valid_request(1,10,200,9999));
    assert(valid_request(2,-10,10,1));
    assert(valid_request(0,0,1,1)); // explicit cancel
}
''')


def test_production_engine_next_step_one_shot_and_default_disabled(tmp_path):
    compile_run(tmp_path, r'''
#include "AnklePerturbation.h"
#include <cassert>
using namespace ankle_perturbation;
int main() {
    Engine pulse;
    Status event;
    assert(pulse.offset(1)==0 && pulse.offset(2)==0);
    pulse.begin_session(10);
    pulse.request(1, 2.5f, 100, 1, 10, true);
    assert(pulse.pop(event) && event.state==1);
    pulse.tick(11, 2, true, true); // opposite step must not trigger
    assert(pulse.offset(1)==0);
    pulse.tick(12, 1, false, true); // existing stance is not a strike
    assert(pulse.offset(1)==0);
    pulse.tick(20, 1, true, true);
    assert(pulse.offset(1)==2.5f && pulse.offset(2)==0);
    assert(pulse.pop(event) && event.state==2);
    pulse.tick(119, 1, false, true);
    assert(pulse.offset(1)==2.5f);
    pulse.tick(120, 1, false, true);
    assert(pulse.offset(1)==0);
    assert(pulse.pop(event) && event.state==3);
    pulse.tick(200, 1, true, true);
    assert(!pulse.pop(event) && pulse.offset(1)==0);
}
''')
