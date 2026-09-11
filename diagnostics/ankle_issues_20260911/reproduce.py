"""Run selected production C++ functions with fake sensors/time, without hardware.

This reproduces source behavior, not the mechanical closed-loop response.
Run from any directory using Python; requires clang++ on PATH.
"""
from pathlib import Path
import subprocess
import re

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
SRC = ROOT / "ExoCode" / "src"


def function(filename, signature):
    text = (SRC / filename).read_text(encoding="utf-8-sig")
    start = text.index(signature)
    opening = text.index("{", start)
    depth = 1
    end = opening + 1
    while depth:
        depth += (text[end] == "{") - (text[end] == "}")
        end += 1
    return text[start:end]


params = function("ControllerData.h", "namespace proportional_joint_moment")
status = function("StatusDefs.h", "namespace messages")
config = (SRC / "Config.h").read_text()
freq = re.search(r"#define LOOP_FREQ_HZ\s+(\S+)", config)[1]
tolerance = re.search(r"#define LOOP_TIME_TOLERANCE\s+(\S+)", config)[1]
exo = (SRC / "Exo.cpp").read_text()
telemetry_gate = re.search(r"const bool correct_status = .*?;", exo)[0]
cal_time = re.search(r"const uint16_t _cal_time = \d+;", (SRC / "TorqueSensor.h").read_text())[0]

stub = r'''
#include <cmath>
#include <cstdint>
#include <iostream>
#include <cassert>
template<class A, class B> auto min(A a,B b) {return a<b?a:b;}
template<class A, class B> auto max(A a,B b) {return a>b?a:b;}
struct SerialStub {template<class T> void print(T) {}} Serial;
namespace torque_calibration {constexpr float AI_CNT_TO_V=3.3f/4095;}
uint32_t now_ms=0;
uint32_t millis() {return now_ms;}
int analogRead(int) {return 1500;}
struct FakeTime {float dt=2000; float tick(float) {return dt;}} clock_stub;
namespace utils {float ewma(float,float,float);}
struct ExoData {
 uint16_t _status=2; bool user_paused=false;
 uint16_t get_status(); void set_status(uint16_t);
};
struct JointData {
 float torque_reading=0, torque_offset_reading=1;
 bool calibrate_torque_sensor=false;
 struct {bool do_zero=false;} motor;
};
struct SideData {
 bool toe_stance=false, ground_strike=false;
 float toe_fsr_upper_threshold=.25f, toe_fsr=0;
};
struct ControllerData {
 float parameters[12]={}, filtered_torque_reading=0;
 float max_measured=0,max_setpoint=0,prev_max_measured=0,prev_max_setpoint=0;
 float kf=1,filtered_setpoint=0,ff_setpoint=0,filtered_cmd=0,desired_torque=0;
};
struct _Controller {
 ExoData* _data; JointData* _joint_data; SideData* _side_data;
 ControllerData* _controller_data; FakeTime* _t_helper=&clock_stub;
 float _t_helper_context=0,_pid_error_sum=0,_prev_input=0,_prev_de_dt=0;
 float _pid(float,float,float,float,float);
};
struct ProportionalJointMoment : _Controller {float calc_motor_cmd();};
struct ZeroTorque : _Controller {float calc_motor_cmd();};
struct TorqueSensor {
 bool _is_used=true,_last_do_calibrate=false;
 int _pin=0; uint32_t _start_time=0;
 float _zero_sum=0,_calibration=0; unsigned _num_calibration_samples=0;
 CAL_TIME
 bool calibrate(bool);
};
struct Motor {void zero() {}} motor;
struct _Joint {
 ExoData* _data; JointData* _joint_data; TorqueSensor _torque_sensor;
 Motor* _motor=&motor; void check_calibration();
};
bool telemetry(ExoData& data) {
 uint16_t exo_status=data.get_status();
 TELEMETRY_GATE
 return correct_status;
}
'''.replace("CAL_TIME", cal_time).replace("TELEMETRY_GATE", telemetry_gate)

main = r'''
int main() {
 ExoData data; JointData joint; SideData side; ControllerData control;
 _Joint j; j._data=&data; j._joint_data=&joint;
 assert(telemetry(data));
 joint.calibrate_torque_sensor=true;
 for(now_ms=0; now_ms<=1500; now_ms+=2) j.check_calibration();
 assert(!joint.calibrate_torque_sensor);
 assert(j._torque_sensor._calibration>0);
 assert(data.get_status()==status_defs::messages::torque_calibration);
 assert(!telemetry(data));
 std::cout << "Calibration finished; status=" << data.get_status()
           << "; telemetry=" << telemetry(data) << "\n";
 data.set_status(status_defs::messages::trial_on);
 assert(telemetry(data));
 std::cout << "Start trial status restores telemetry=" << telemetry(data) << "\n";

 ProportionalJointMoment c; c._data=&data; c._joint_data=&joint;
 c._side_data=&side; c._controller_data=&control;
 // Current SDCard/ankleControllers/PJMC.csv values are inserted by Python.
 float row[12]={PARAMETER_ROW};
 for(int i=0;i<12;i++) control.parameters[i]=row[i];
 auto steady=[&](float measured) {
   joint.torque_reading=measured;
   c._prev_input=measured; // remove derivative so the gain jump is isolated
   return c.calc_motor_cmd();
 };
 float below=steady(3.49f),above=steady(3.51f);
 std::cout << "Gain boundary: measurement 3.49 -> command " << below
           << "; measurement 3.51 -> command " << above << " Nm\n";
 assert(std::abs(above-below)>10);

 clock_stub.dt=2000;
 joint.torque_reading=4.0f; c._prev_input=3.9f;
 float with_d=c.calc_motor_cmd();
 clock_stub.dt=2210;
 joint.torque_reading=4.0f; c._prev_input=3.9f;
 float without_d=c.calc_motor_cmd();
 std::cout << "Same 0.1 Nm measurement change: dt=2000 us -> " << with_d
           << "; dt=2210 us -> " << without_d << " Nm\n";
 assert(std::abs(with_d-without_d)>1.4f);

 control.parameters[3]=0;
 assert(steady(3.49f)==0 && steady(3.51f)==0);
 std::cout << "Use PID=0 with current zero setpoints -> command 0 Nm\n";

 float backup[12]={BACKUP_ROW};
 for(int i=0;i<12;i++) control.parameters[i]=backup[i];
 clock_stub.dt=2000;
 joint.torque_reading=3.49f;
 for(int i=0;i<40;i++) c.calc_motor_cmd();
 below=c.calc_motor_cmd();
 joint.torque_reading=3.51f;
 for(int i=0;i<40;i++) c.calc_motor_cmd();
 above=c.calc_motor_cmd();
 std::cout << "Dated backup: settled measurement 3.49 -> " << below
           << "; 3.51 -> " << above << " Nm (no gain switch)\n";
 assert(std::abs(above-below)<0.02f);

 ZeroTorque zero; zero._data=&data; zero._joint_data=&joint;
 zero._side_data=&side; zero._controller_data=&control;
 for(int i=0;i<12;i++) control.parameters[i]=0;
 control.filtered_torque_reading=-4.9f;
 joint.torque_reading=2.0f;
 assert(zero.calc_motor_cmd()==0.0f);
 assert(control.desired_torque==0.0f);
 assert(control.filtered_torque_reading==-4.9f);
 joint.torque_reading=-8.0f;
 assert(zero.calc_motor_cmd()==0.0f);
 assert(control.filtered_torque_reading==-4.9f);
 std::cout << "Zero Torque: raw torque 2 -> -8 Nm, transmitted measured field remains -4.9 Nm; command=0\n";

 // GUI-only workaround: PJMC, both targets=0, Use PID=0.
 // Check both gait branches keep zero command while refreshing telemetry.
 for(bool stance : {false,true}) {
   side.toe_stance=stance; side.toe_fsr=0.8f;
   joint.torque_reading=2.0f;
   const float before=control.filtered_torque_reading;
   assert(c.calc_motor_cmd()==0.0f);
   assert(control.filtered_torque_reading!=before);
   const float next=control.filtered_torque_reading;
   joint.torque_reading=-8.0f;
   assert(c.calc_motor_cmd()==0.0f);
   assert(control.filtered_torque_reading!=next);
 }
 std::cout << "PJMC with stance=0, swing=0, Use PID=0: zero command and live measured field in stance and swing\n";
 std::cout << "All source-behavior reproductions passed. No hardware accessed.\n";
}
'''
row = (ROOT / "SDCard/ankleControllers/PJMC.csv").read_text().splitlines()[5]
assert len(row.split(",")) == 12
main = main.replace("PARAMETER_ROW", row)
backup = (ROOT / "SDCard_asfound_20260825/ankleControllers/PJMC.csv").read_text().splitlines()[5]
assert len(backup.split(",")) == 12
main = main.replace("BACKUP_ROW", backup)
code = "\n".join([
    f"#define LOOP_FREQ_HZ {freq}\n#define LOOP_TIME_TOLERANCE {tolerance}",
    "#include <cstdint>",
    "namespace status_defs {" + status + "}",
    "namespace controller_defs {" + params + "}",
    "namespace controller_defs {" + function("ControllerData.h", "namespace zero_torque") + "}",
    stub,
    function("ExoData.cpp", "void ExoData::set_status"),
    function("ExoData.cpp", "uint16_t ExoData::get_status"),
    "namespace utils {" + function("Utilities.cpp", "float ewma(") + "}",
    function("TorqueSensor.cpp", "bool TorqueSensor::calibrate"),
    function("Joint.cpp", "void _Joint::check_calibration"),
    function("Controller.cpp", "float _Controller::_pid"),
    function("Controller.cpp", "float ProportionalJointMoment::calc_motor_cmd"),
    function("Controller.cpp", "float ZeroTorque::calc_motor_cmd"),
    main,
])
source = HERE / "source_reproduction.cpp"
source.write_text(code, encoding="utf-8")
binary = HERE / "source_reproduction.exe"
subprocess.run(["clang++", "-std=c++17", str(source), "-o", str(binary)], check=True)
result = subprocess.run([str(binary)], capture_output=True, text=True, check=True)
(HERE / "results.txt").write_text(result.stdout, encoding="utf-8")
print(result.stdout, end="")
