"""Host behavior tests compile actual sketch functions, without Arduino hardware."""
import pathlib
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'ESP32RemoteCarController.ino'


def function(name):
    text = SOURCE.read_text()
    import re
    match = re.search(r'void ' + name + r'\([^;{}]*\)\s*\{', text)
    if not match:
        raise AssertionError('Missing function: ' + name)
    start = match.start()
    end = match.end()
    depth = 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[start:end]


def run_cpp(code):
    with tempfile.TemporaryDirectory() as tmp:
        src = pathlib.Path(tmp) / 'test.cpp'
        exe = pathlib.Path(tmp) / 'test'
        src.write_text(code)
        result = subprocess.run(['g++', '-std=c++11', '-Wall', '-Wextra', '-Werror',
                                 '-I', str(ROOT), str(src), '-o', str(exe)], capture_output=True, text=True)
        assert result.returncode == 0, result.stderr
        result = subprocess.run([str(exe)], capture_output=True, text=True)
        assert result.returncode == 0, result.stderr


RAMP_STUBS = r'''
#include <cassert>
#include <cstdint>
#include "MotorPolicy.h"
struct SteeringGuard {SteeringGuard(){} ~SteeringGuard(){}};
MotorPolicy::SteeringPulse steeringPulse;
void stopSteering();
int driveMotorDriver=1, steerMotorDriver=0;
const int DRIVER_L298N=1, DRIVE_FORWARD=1, DRIVE_BACK=2, STEER_LEFT=1, STEER_RIGHT=2;
int currentDriveState=2, currentSteerState=0;
int driveSpeedPercent=60, reverseSpeedPercent=35, steerSpeedPercent=75;
int driveRampMs=400, steerRampMs=250, currentDrivePwm=0, currentSteerPwm=0;
unsigned long driveRampStartMs=1000, steerRampStartMs=0, clockMs=1200;
const int pinForward=4, pinBack=5, pinLeft=6, pinRight=7;
int pins[8]={};
unsigned long millis(){return clockMs;}
int getMaxDrivePwm(){return 127;}
long map(long x,long a,long b,long c,long d){return (x-a)*(d-c)/(b-a)+c;}
void analogWrite(int pin,int value){pins[pin]=value;}
void setRelayOn(int pin){pins[pin]=255;}
void setRelayOff(int pin){pins[pin]=0;}
'''


class DriveBehavior(unittest.TestCase):
    def test_reverse_ramp_uses_independent_speed_and_same_ceiling(self):
        run_cpp(RAMP_STUBS + 'void stopSteering(){}\n' + function('updateMotorRamping') + r'''
int main(){
  updateMotorRamping(); assert(pins[pinBack]==22); assert(pins[pinForward]==0);
  clockMs=1400; updateMotorRamping(); assert(pins[pinBack]==44);
  reverseSpeedPercent=100; updateMotorRamping(); assert(pins[pinBack]==127);
  currentDriveState=DRIVE_FORWARD; updateMotorRamping(); assert(pins[pinForward]==76);
  assert(pins[pinBack]==0);
  driveRampMs=0; reverseSpeedPercent=20; currentDriveState=DRIVE_BACK;
  updateMotorRamping(); assert(pins[pinBack]==25);
}
''')

    def test_relay_direction_uses_independent_duty(self):
        run_cpp(RAMP_STUBS + 'void stopSteering(){}\n' + function('updateMotorRamping') + '''
int main(){
 driveMotorDriver=0; driveRampMs=0; clockMs=60;
 updateMotorRamping(); assert(pins[pinBack]==0);
 currentDriveState=DRIVE_FORWARD; updateMotorRamping(); assert(pins[pinForward]==255);
 currentDriveState=DRIVE_BACK; clockMs=30; updateMotorRamping();
 assert(pins[pinBack]==255); assert(pins[pinForward]==0);
}
''')

    def test_loop_has_no_second_uncapped_pwm_writer(self):
        self.assertNotIn('analogWrite(', function('loop'))


class PulseBehavior(unittest.TestCase):
    def test_elapsed_cutoff_repeat_latch_release_and_wraparound(self):
        self.assertTrue('class SteeringPulse' in (ROOT / 'MotorPolicy.h').read_text())
        run_cpp(r'''
#include "MotorPolicy.h"
#include <cassert>
int main(){
 MotorPolicy::SteeringPulse pulse;
 assert(pulse.begin(1000,100));
 assert(!pulse.expired(1099)); assert(pulse.expired(1100));
 assert(!pulse.begin(1050,500)); assert(!pulse.begin(5000,100));
 pulse.stop(); assert(!pulse.begin(6000,100));
 pulse.release(); assert(pulse.begin(6000,20));
 assert(!pulse.expired(6019)); assert(pulse.expired(6020));
 pulse.release(); assert(pulse.begin(UINT32_MAX-9,20));
 assert(!pulse.expired(9)); assert(pulse.expired(10));
 pulse.release(); assert(pulse.begin(100,500));
 assert(!pulse.expired(599)); assert(pulse.expired(600));
}
''')


class SettingsBehavior(unittest.TestCase):
    def test_migration_and_strict_validation(self):
        self.assertTrue((ROOT / 'MotorPolicy.h').exists(), 'Missing shared motor settings policy')
        run_cpp(r'''
#include "MotorPolicy.h"
#include <cassert>
#include <map>
#include <string>
struct Prefs {
 std::map<std::string,int> values;
 int getInt(const char* key, int fallback){return values.count(key) ? values[key] : fallback;}
 bool isKey(const char* key){return values.count(key);}
 void putInt(const char* key,int value){values[key]=value;}
};
int main(){
 Prefs p; p.values["speed"]=73;
 auto s=MotorPolicy::load(p); assert(s.forward==73 && s.reverse==73 && s.pulseMs==100);
 p.values["speed"]=80; s=MotorPolicy::load(p); assert(s.forward==80 && s.reverse==73);
 p.values["speed"]=73;
 p.values["rev_spd"]=34; s=MotorPolicy::load(p); assert(s.reverse==34 && s.forward==73);
 p.values["speed"]=-999; p.values["rev_spd"]=999; p.values["s_pulse"]=50000;
 s=MotorPolicy::load(p); assert(s.forward==60 && s.reverse==60 && s.pulseMs==100);
 int value=7;
 for(const char* bad : {"", "abc", "30x", "-1", "19", "101", "9999999999999999", " 30", "30.5"}){
  assert(!MotorPolicy::parseSetting(bad,20,100,value)); assert(value==7);
 }
 assert(MotorPolicy::parseSetting("20",20,100,value) && value==20);
 assert(MotorPolicy::parseSetting("100",20,100,value) && value==100);
 assert(MotorPolicy::parseSetting("020",20,500,value) && value==20);
}
''')


if __name__ == '__main__':
    unittest.main()
