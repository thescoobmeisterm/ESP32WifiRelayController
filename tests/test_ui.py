import re
import subprocess
import unittest
from test_motor_safety import SOURCE


def js_function(name):
    s=SOURCE.read_text()
    m=re.search(r'    const '+name+r' = .*?\n    };',s,re.S)
    if not m: raise AssertionError('Missing JS function '+name)
    return m.group()


class UI(unittest.TestCase):
    def test_controls_status_sync_and_server_acknowledgement(self):
        s=SOURCE.read_text()
        for ident in ['forwardSpeedSetting','reverseSpeedSlider','steerPulseSlider']:
            self.assertTrue('id="'+ident+'"' in s, 'Missing control '+ident)
        self.assertTrue('syncMotorSettings(cfg)' in s)
        self.assertTrue('syncMotorSettings(d)' in s)
        script=r'''
const assert=require('assert');
const nodes=new Map();
const document={activeElement:null,getElementById:(id)=>{
 if(!nodes.has(id)) nodes.set(id,{value:0,innerText:'',classList:{toggle(){}}});
 return nodes.get(id);
}};
let request=''; let response={reverse_speed:35};
const fetch=async(url)=>{request=url;return {ok:true,json:async()=>response};};
const updateSpeedPresetButtons=()=>{};
''' + js_function('syncMotorSettings') + '\n' + js_function('saveMotorSetting') + r'''
(async()=>{
 syncMotorSettings({speed:70,reverse_speed:35,steer_pulse_ms:20});
 assert.equal(nodes.get('forwardSpeedSetting').value,70);
 assert.equal(nodes.get('speedSlider').value,70);
 assert.equal(nodes.get('reverseSpeedSlider').value,35);
 assert.equal(nodes.get('steerPulseSlider').value,20);
 await saveMotorSetting('set_reverse_speed',35);
 assert.equal(request,'/set_reverse_speed?val=35');
 assert.equal(nodes.get('reverseSpeedLabel').innerText,'35% REVERSE');
 assert.equal(nodes.get('speedSlider').value,70);
})().catch(e=>{console.error(e);process.exitCode=1;});
'''
        result=subprocess.run(['node','-e',script],capture_output=True,text=True)
        self.assertEqual(result.returncode,0,result.stderr)

    def test_unsafe_homing_and_voltage_promises_removed(self):
        s=SOURCE.read_text()
        self.assertNotIn('never over-volted',s)
        self.assertNotIn('setVoltageCapPreset(12000)',s)
        self.assertNotIn('setVoltageCapPreset(9000)',s)
        self.assertNotIn('Auto-Homing Started',s)
        self.assertNotIn('"Centering"',s)
        self.assertTrue('id="btnAutoHome" disabled' in s)


if __name__=='__main__': unittest.main()
