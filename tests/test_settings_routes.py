import unittest
from test_motor_safety import SOURCE, run_cpp


def route(path):
    text = SOURCE.read_text()
    start = text.index('  server.on("' + path + '", []() {')
    return text[start:text.index('\n  });', start) + len('\n  });')]


STUBS = r'''
#include "MotorPolicy.h"
#include <cassert>
#include <string>
#include <map>
#include <functional>
#include <cstdlib>
struct String : std::string {
 using std::string::string;
 String(std::string s):std::string(s){}
 String(int x):std::string(std::to_string(x)){}
 int toInt() const {return atoi(c_str());}
};
struct Prefs {
 std::map<std::string,int> values;
 bool fail=false;
 bool begin(const char*,bool){return true;}
 void end(){}
 size_t putInt(const char* k,int v){if(fail)return 0; values[k]=v; return sizeof(int);}
} prefs;
struct Server {
 std::map<std::string,std::function<void()>> routes;
 std::map<std::string,String> args;
 int code=0; String body;
 void on(const char* p,std::function<void()> f){routes[p]=f;}
 bool hasArg(const char* k){return args.count(k);}
 String arg(const char* k){return args[k];}
 void send(int c,const char*,String s){code=c;body=s;}
} server;
struct Log {void printf(const char*,...){};} Serial;
int driveSpeedPercent=60, reverseSpeedPercent=60, steerPulseMs=100;
int updates=0, stops=0;
void updateMotorRamping(){++updates;}
void stopSteering(){++stops;}
int constrain(int v,int lo,int hi){return v<lo?lo:(v>hi?hi:v);}
'''


class Routes(unittest.TestCase):
    def test_status_and_config_publish_independent_values(self):
        run_cpp(STUBS + r'''
int currentDriveState=0,currentSteerState=0,currentPositionMs=0;
const int DRIVE_FORWARD=1, DRIVE_BACK=2, STEER_LEFT=1, STEER_RIGHT=2;
int driveMotorDriver=1,steerMotorDriver=0,driveRampMs=400,steerRampMs=250;
int maxDriveVoltageMv=6000,batteryVoltageMv=13000,steerSpeedPercent=75;
int steerLimitLeftMs=2500,steerLimitRightMs=2500,audioVolumePercent=0,currentSoundTheme=0;
bool steerLimitsEnabled=true,audioMuted=true,isAutoHoming=false;
bool collisionAvoidanceEnabled=false,frontBlocked=false,backBlocked=false;
int collisionThresholdCm=30,frontDistanceCm=-1,backDistanceCm=-1;
unsigned long lastHeartbeatMs=0;
unsigned long millis(){return 100;}
void setupRoutes(){
''' + route('/status') + route('/get_config') + r'''
}
int main(){
 setupRoutes(); driveSpeedPercent=73;reverseSpeedPercent=34;steerPulseMs=25;
 for(auto path:{"/status","/get_config"}) {
  server.routes[path](); assert(server.code==200);
  assert(server.body.find("\"speed\":73")!=std::string::npos);
  assert(server.body.find("\"reverse_speed\":34")!=std::string::npos);
  assert(server.body.find("\"steer_pulse_ms\":25")!=std::string::npos);
 }
}
''')

    def test_independent_settings_validate_persist_and_apply(self):
        text = SOURCE.read_text()
        self.assertIn('server.on("/set_reverse_speed"', text)
        self.assertIn('server.on("/set_steer_pulse"', text)
        run_cpp(STUBS + '\nvoid setupRoutes(){\n' + ''.join(route(p) for p in
            ['/set_speed', '/set_reverse_speed', '/set_steer_pulse']) + r'''
}
int main(){
 setupRoutes();
 server.args["val"]="35"; server.routes["/set_reverse_speed"]();
 assert(server.code==200 && reverseSpeedPercent==35 && driveSpeedPercent==60);
 assert(prefs.values["rev_spd"]==35 && updates==1);
 server.args["val"]="80"; server.routes["/set_speed"]();
 assert(server.code==200 && driveSpeedPercent==80 && reverseSpeedPercent==35);
 assert(prefs.values["speed"]==80 && updates==2);
 server.args["val"]="20"; server.routes["/set_steer_pulse"]();
 assert(server.code==200 && steerPulseMs==20 && prefs.values["s_pulse"]==20 && stops==1);
 for(auto path : {"/set_speed", "/set_reverse_speed", "/set_steer_pulse"}) {
  for(auto bad : {"garbage","-1","1000000000000000000","30junk",""}) {
   server.args["val"]=bad; server.routes[path](); assert(server.code==400);
  }
  server.args.clear(); server.routes[path](); assert(server.code==400);
 }
 assert(driveSpeedPercent==80 && reverseSpeedPercent==35 && steerPulseMs==20);
 prefs.fail=true; server.args["val"]="40"; server.routes["/set_reverse_speed"]();
 assert(server.code==500 && reverseSpeedPercent==35);
}
''')


if __name__ == '__main__':
    unittest.main()
