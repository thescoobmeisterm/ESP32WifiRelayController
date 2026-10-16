import unittest
from test_motor_safety import SOURCE, function, run_cpp, RAMP_STUBS

STEER_STUBS = r'''
#include "MotorPolicy.h"
const int STEER_IDLE=0;
using SteerState=int;
int steerPulseMs=100;
long currentPositionMs=0;
unsigned long lastSteerUpdateMs=0;
bool isCentering=false, isAutoHoming=false;
bool steeringMutex=true, steeringTimer=true, timerArmed=false, timerFail=false;
uint64_t timerDelay=0;
const int ESP_OK=0;
int esp_timer_stop(bool){timerArmed=false;return 0;}
int esp_timer_start_once(bool,uint64_t us){timerArmed=!timerFail;timerDelay=us;return timerFail ? -1:0;}
void delay(int ms){clockMs+=ms;}
'''


class SteeringIntegration(unittest.TestCase):
    def test_timer_stops_both_outputs_without_loop_and_repeats_cannot_rearm(self):
        self.assertTrue('void steeringPulseTimeout(' in SOURCE.read_text())
        functions='\n'.join(function(n) for n in ['updateSteeringPosition','stopSteering',
            'releaseSteering','steeringPulseTimeout','startSteering','startSteeringLeft','startSteeringRight',
            'updateMotorRamping'])
        run_cpp(RAMP_STUBS + STEER_STUBS + functions + r'''
int main(){
 for(int driver : {0,1}) {
  steerMotorDriver=driver; steerRampMs=0; currentDriveState=0;
  releaseSteering(); clockMs=1000; startSteeringLeft();
  assert(timerArmed && timerDelay==100000 && pins[pinLeft]>0 && pins[pinRight]==0);
  auto start=steerRampStartMs;
  clockMs=start+50; startSteeringLeft(); startSteeringRight();
  assert(steerRampStartMs==start && currentSteerState==STEER_LEFT);
  clockMs=start+100; steeringPulseTimeout(nullptr); // timer task runs, loop is blocked
  assert(pins[pinLeft]==0 && pins[pinRight]==0 && currentSteerState==STEER_IDLE);
  startSteeringLeft(); startSteeringRight(); updateMotorRamping();
  assert(pins[pinLeft]==0 && pins[pinRight]==0);
  releaseSteering(); startSteeringRight(); assert(pins[pinRight]>0 && pins[pinLeft]==0);
  clockMs=steerRampStartMs+100; steeringPulseTimeout(nullptr);
  assert(pins[pinRight]==0);
  releaseSteering(); timerFail=true; startSteeringLeft();
  assert(pins[pinLeft]==0 && pins[pinRight]==0); timerFail=false;
 }
}
'''.replace('for(int driver : {0,1})','for(int driver=0;driver<2;++driver)'))

    def test_commands_preserve_stop_collision_and_refuse_automatic_motion(self):
        functions='\n'.join(function(n) for n in ['stopDriving','updateSteeringPosition',
            'stopSteering','releaseSteering','steeringPulseTimeout','startSteering',
            'startSteeringLeft','startSteeringRight','updateMotorRamping',
            'startAutoHoming','stopAll','processCommand'])
        run_cpp(RAMP_STUBS + STEER_STUBS + r'''
#include <string>
struct String:std::string {using std::string::string;void trim(){}};
struct Log {void printf(const char*,...){} void println(const char*){}} Serial;
const int DRIVE_STOP=0;
bool collisionAvoidanceEnabled=true, frontBlocked=false,backBlocked=false;
bool steerLimitsEnabled=false;
long steerLimitLeftMs=2500,steerLimitRightMs=2500;
int frontDistanceCm=10,backDistanceCm=10;
unsigned long lastHeartbeatMs=0,forwardCycleStartMs=0;
void playCollisionAlertTone(){} void setLedColor(int,int,int){} void playStopBeep(){}
void triggerSetCenterFlash(){}
''' + functions + r'''
int main(){
 driveRampMs=0; currentDriveState=DRIVE_STOP; steerRampMs=0;
 processCommand("back"); assert(pins[pinBack]==44 && pins[pinForward]==0);
 processCommand("forward"); assert(pins[pinForward]==76 && pins[pinBack]==0);
 processCommand("stop_drive"); assert(pins[pinForward]==0 && pins[pinBack]==0);
 backBlocked=true; processCommand("back"); assert(currentDriveState==DRIVE_STOP);
 frontBlocked=true; processCommand("forward"); assert(currentDriveState==DRIVE_STOP);
 processCommand("left"); assert(currentSteerState==STEER_LEFT);
 processCommand("center"); assert(currentSteerState==STEER_IDLE && !isCentering);
 processCommand("left"); assert(currentSteerState==STEER_IDLE); // still held
 processCommand("stop_steer"); processCommand("right"); assert(currentSteerState==STEER_RIGHT);
 processCommand("auto_home"); assert(!isAutoHoming && currentSteerState==STEER_IDLE);
 currentPositionMs=123; processCommand("set_center"); assert(currentPositionMs==0);
 assert(pins[pinLeft]==0 && pins[pinRight]==0);
 processCommand("stop_steer"); processCommand("left"); stopAll();
 assert(pins[pinLeft]==0 && pins[pinRight]==0);
 processCommand("left"); assert(currentSteerState==STEER_IDLE); // failsafe does not rearm
}
''')

    def test_timer_lock_is_not_held_during_http_or_flash(self):
        from test_settings_routes import route
        body = route('/set_steer_driver')
        self.assertIn('{\n        SteeringGuard guard;\n        stopSteering();', body)
        critical = body[body.index('SteeringGuard guard;'):].split('}', 1)[0]
        self.assertNotIn('prefs.', critical)
        self.assertNotIn('server.send', critical)

    def test_command_paths_cannot_bypass_policy(self):
        command=function('processCommand')
        self.assertNotIn('analogWrite(', command)
        self.assertNotIn('setRelayOn(', command)
        self.assertNotIn('isCentering = true', command)
        self.assertNotIn('startSteeringLeft', function('startAutoHoming'))
        self.assertNotIn('startSteering', function('loop'))
        self.assertIn('releaseSteering();', command)


if __name__ == '__main__':
    unittest.main()
