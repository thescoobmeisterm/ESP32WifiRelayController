#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BLEScan.h>
#include <Preferences.h>
#include "driver/i2s.h"

// AP Credentials
const char* ssid = "TinkDrive";
const char* password = "tink1234";

// Safe Pins for ESP32-S3 (avoiding Octal SPI pins 26-32)
const int pinForward = 4;  // IN1 Relay
const int pinBack    = 5;  // IN2 Relay
const int pinLeft    = 6;  // IN3 Relay
const int pinRight   = 7;  // IN4 Relay

// MAX98357A I2S Digital Amplifier Pins
#define I2S_NUM         I2S_NUM_0
#define I2S_BCLK_PIN    17 // Bit Clock
#define I2S_LRC_PIN     16 // Left/Right Word Select Clock
#define I2S_DIN_PIN     18 // Digital Data Input

// Standard Active-HIGH Relay Control Helpers (for HIGH-jumpered boards)
void setRelayOn(int pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH); // 3.3V (HIGH) -> Triggers Active-HIGH Relay -> Relay ENGAGED!
}

void setRelayOff(int pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);  // 0.0V (LOW) -> 0V to ground -> Relay DISENGAGED (OFF)!
}

// Onboard RGB LED helper (ESP32-S3 built-in WS2812 LED)
#ifndef RGB_BUILTIN
#define RGB_BUILTIN 48
#endif

void setLedColor(uint8_t r, uint8_t g, uint8_t b) {
  neopixelWrite(RGB_BUILTIN, r, g, b);
}

// Smooth Rainbow HSV to RGB Helper
void setLedHue(uint16_t hue) {
  uint8_t r, g, b;
  uint8_t sector = hue / 60;
  uint8_t rel = (hue % 60) * 255 / 60;
  switch (sector) {
    case 0: r = 255; g = rel; b = 0; break;
    case 1: r = 255 - rel; g = 255; b = 0; break;
    case 2: r = 0; g = 255; b = rel; break;
    case 3: r = 0; g = 255 - rel; b = 255; break;
    case 4: r = rel; g = 0; b = 255; break;
    default: r = 255; g = 0; b = 255 - rel; break;
  }
  neopixelWrite(RGB_BUILTIN, r, g, b);
}

DNSServer dnsServer;
const byte DNS_PORT = 53;
WebServer server(80);

// BLE Nordic UART Service (NUS) & HID UUIDs
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic = NULL;
bool deviceConnected = false;

// Direct BLE Gamepad Client Tracking
BLEClient* pGamepadClient = NULL;
bool isBleScanning = false;

// --- Steering & Drive State Variables ---
enum SteerState { STEER_IDLE, STEER_LEFT, STEER_RIGHT };
SteerState currentSteerState = STEER_IDLE;

enum DriveState { DRIVE_STOP, DRIVE_FORWARD, DRIVE_BACK };
DriveState currentDriveState = DRIVE_STOP;

long currentPositionMs = 0; 
// --- Configurable Drive Speed & Steering Calibration Limits ---
int driveSpeedPercent = 60;          // Default drive throttle: 60% (Range: 20% to 100%)
long steerLimitLeftMs = 600;         // Max Left steering lock travel in ms (Range: 100ms - 3000ms)
long steerLimitRightMs = 600;        // Max Right steering lock travel in ms (Range: 100ms - 3000ms)
unsigned long lastSteerUpdateMs = 0;
bool isCentering = false;

// Non-blocking Animation & LED variables
unsigned long lastBlinkMs = 0;
bool blinkState = false;

unsigned long centerAnimMs = 0;
bool centerColorState = false;

uint16_t rainbowHue = 0;
unsigned long lastRainbowMs = 0;

// Failsafe Connection Tracking
unsigned long lastHeartbeatMs = 0;
const unsigned long FAILSAFE_TIMEOUT_MS = 3000; // 3 seconds connection timeout

// Function declarations
void stopDriving();
void stopSteering();
void stopAll();
void processCommand(String cmd);
void playBootChime();
void playSuccessChime();
void playStopBeep();
void startBleGamepadScan();
void initI2SAudio();
void playI2STone(uint32_t freq, uint32_t durationMs);

bool isDeviceConnected() {
  return (WiFi.softAPgetStationNum() > 0) || deviceConnected;
}

bool isI2SReady = false;

// I2S Hardware Audio Generator for MAX98357A Amplifier
void initI2SAudio() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = 44100,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 64,
    .use_apll = false,
    .tx_desc_auto_clear = true
  };
  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_BCLK_PIN,
    .ws_io_num = I2S_LRC_PIN,
    .data_out_num = I2S_DIN_PIN,
    .data_in_num = I2S_PIN_NO_CHANGE
  };
  i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM, &pin_config);
  i2s_zero_dma_buffer(I2S_NUM);
  isI2SReady = true;
}

int audioVolumePercent = 100; // 0% to 100% full scale
bool audioMuted = false;

inline float getVolumeFactor() {
  if (audioMuted || audioVolumePercent <= 0) return 0.0f;
  return (float)audioVolumePercent / 100.0f;
}

void playI2STone(uint32_t freq, uint32_t durationMs) {
  if (!isI2SReady) return;
  float vol = getVolumeFactor();
  if (vol <= 0.001f) return;

  int sampleRate = 44100;
  int numSamples = (sampleRate * durationMs) / 1000;
  int16_t samples[128];
  
  size_t bytesWritten;
  double phase = 0;
  double phaseInc = (2.0 * M_PI * freq) / sampleRate;

  for (int i = 0; i < numSamples; i += 64) {
    int count = min(64, numSamples - i);
    for (int j = 0; j < count; j++) {
      int32_t raw = (int32_t)(sin(phase) * 28000.0 * vol);
      int16_t sampleVal = (int16_t)constrain(raw, -32767, 32767);
      samples[j * 2] = sampleVal;     // Left
      samples[j * 2 + 1] = sampleVal; // Right
      phase += phaseInc;
      if (phase >= 2.0 * M_PI) phase -= 2.0 * M_PI;
    }
    // NON-BLOCKING TIMEOUT (20ms max wait) to prevent CPU hang if I2S DMA buffers are full
    i2s_write(I2S_NUM, samples, count * 4, &bytesWritten, pdMS_TO_TICKS(20));
  }
}

// --- Racing Simulator Dynamic Sound Synthesizer ---
unsigned long lastAudioSliceMs = 0;
double simEnginePhase = 0;
double simDriftPhase = 0;
unsigned long forwardCycleStartMs = 0;
bool wasConnectedAudio = false;

// Ignition Engine Starter Sound (Crank-crank-crank-VROOOOM!)
void playIgnitionSound() {
  if (!isI2SReady) return;
  float vol = getVolumeFactor();
  if (vol <= 0.001f) return;

  // Starter motor cranking chirps
  playI2STone(115, 80); delay(35);
  playI2STone(110, 80); delay(35);
  playI2STone(105, 90); delay(30);

  // Engine Catch & Roaring Rev
  int sampleRate = 44100;
  int numSamples = (sampleRate * 450) / 1000; // 450ms rev
  int16_t samples[128];
  double phase = 0;
  size_t bytesWritten;

  for (int i = 0; i < numSamples; i += 64) {
    int count = min(64, numSamples - i);
    double prog = (double)i / (double)numSamples;
    double freq = (prog < 0.55) ? (130.0 + (prog / 0.55) * (480.0 - 130.0))
                                : (480.0 - ((prog - 0.55) / 0.45) * (480.0 - 90.0));
    double phaseInc = (2.0 * M_PI * freq) / sampleRate;

    for (int j = 0; j < count; j++) {
      int32_t raw = (int32_t)(
        ((sin(phase) * 20000.0) +
         (sin(phase * 2.0) * 10000.0) +
         (sin(phase * 0.5) * 6000.0)) * vol
      );
      int16_t sampleVal = (int16_t)constrain(raw, -32767, 32767);
      samples[j * 2] = sampleVal;
      samples[j * 2 + 1] = sampleVal;
      phase += phaseInc;
      if (phase >= 2.0 * M_PI) phase -= 2.0 * M_PI;
    }
    i2s_write(I2S_NUM, samples, count * 4, &bytesWritten, pdMS_TO_TICKS(15));
  }
}

void updateSimulatorAudio() {
  if (!isI2SReady) return;
  float vol = getVolumeFactor();

  // Check for device connection event to play Ignition Start sound
  bool nowConnected = isDeviceConnected();
  if (nowConnected && !wasConnectedAudio) {
    wasConnectedAudio = true;
    playIgnitionSound();
    return;
  }
  if (!nowConnected) {
    wasConnectedAudio = false;
  }

  if (vol <= 0.001f) return; // Muted -> 0 CPU sound overhead

  unsigned long now = millis();
  if (now - lastAudioSliceMs < 25) return;
  lastAudioSliceMs = now;

  int sampleRate = 44100;
  int numSamples = (sampleRate * 25) / 1000; // 25ms slice
  int16_t samples[128];
  size_t bytesWritten;

  // 1. TIRE DRIFT SQUEAL (Steering Left or Right)
  if (currentSteerState == STEER_LEFT || currentSteerState == STEER_RIGHT) {
    for (int i = 0; i < numSamples; i += 64) {
      int count = min(64, numSamples - i);
      double squealFreq = 2600.0 + (sin((double)now * 0.04 + (double)i * 0.008) * 350.0) + ((rand() % 220) - 110);
      double phaseInc = (2.0 * M_PI * squealFreq) / sampleRate;

      for (int j = 0; j < count; j++) {
        int16_t noise = (rand() % 4000) - 2000;
        int32_t raw = (int32_t)(((sin(simDriftPhase) * 18000.0) + (sin(simDriftPhase * 1.45) * 9000.0) + noise) * vol);
        int16_t sampleVal = (int16_t)constrain(raw, -32767, 32767);
        samples[j * 2] = sampleVal;
        samples[j * 2 + 1] = sampleVal;
        simDriftPhase += phaseInc;
        if (simDriftPhase >= 2.0 * M_PI) simDriftPhase -= 2.0 * M_PI;
      }
      i2s_write(I2S_NUM, samples, count * 4, &bytesWritten, pdMS_TO_TICKS(10));
    }
    return;
  }

  // 2. CONTINUOUS CYCLIC SHIFTING GEARBOX (Forward Driving)
  if (currentDriveState == DRIVE_FORWARD) {
    unsigned long cycleElapsed = (now - forwardCycleStartMs) % 1500; // 1.5s per gear cycle
    int gearNum = ((now - forwardCycleStartMs) / 1500) % 5;          // Gears 1 through 5 loop

    double baseGearFreq = 95.0 + (gearNum * 22.0);
    double maxGearFreq  = 310.0 + (gearNum * 28.0);
    double engineFreq;

    if (cycleElapsed < 1320) {
      // Accelerating climb in current gear
      double prog = (double)cycleElapsed / 1320.0;
      engineFreq = baseGearFreq + (prog * prog * (maxGearFreq - baseGearFreq));
    } else {
      // Gear shift clutch dip drop
      double dropProg = (double)(cycleElapsed - 1320) / 180.0;
      engineFreq = maxGearFreq - (dropProg * (maxGearFreq - (baseGearFreq + 22.0)));
    }

    double phaseInc = (2.0 * M_PI * engineFreq) / sampleRate;

    for (int i = 0; i < numSamples; i += 64) {
      int count = min(64, numSamples - i);
      for (int j = 0; j < count; j++) {
        int32_t raw = (int32_t)(
          ((sin(simEnginePhase) * 18000.0) +
           (sin(simEnginePhase * 2.0) * 9000.0) +
           (sin(simEnginePhase * 0.5) * 6000.0) +
           (sin(simEnginePhase * 3.0) * 3500.0)) * vol
        );
        int16_t sampleVal = (int16_t)constrain(raw, -32767, 32767);
        samples[j * 2] = sampleVal;
        samples[j * 2 + 1] = sampleVal;
        simEnginePhase += phaseInc;
        if (simEnginePhase >= 2.0 * M_PI) simEnginePhase -= 2.0 * M_PI;
      }
      i2s_write(I2S_NUM, samples, count * 4, &bytesWritten, pdMS_TO_TICKS(10));
    }
    return;
  }

  // 3. V8 CAM-IDLE THROBBING RUMBLE (When Connected & Neutral/Idle)
  if (isDeviceConnected() && currentDriveState == DRIVE_STOP && !isCentering) {
    double idleMod = 0.75 + (sin((double)now * 0.02) * 0.25);
    double idleFreq = 42.0 + (sin((double)now * 0.008) * 3.0);
    double phaseInc = (2.0 * M_PI * idleFreq) / sampleRate;

    for (int i = 0; i < numSamples; i += 64) {
      int count = min(64, numSamples - i);
      for (int j = 0; j < count; j++) {
        int32_t raw = (int32_t)(
          (((sin(simEnginePhase) * 17000.0) +
            (sin(simEnginePhase * 2.0) * 8000.0) +
            (sin(simEnginePhase * 3.0) * 4000.0)) * idleMod) * vol
        );
        int16_t sampleVal = (int16_t)constrain(raw, -32767, 32767);
        samples[j * 2] = sampleVal;
        samples[j * 2 + 1] = sampleVal;
        simEnginePhase += phaseInc;
        if (simEnginePhase >= 2.0 * M_PI) simEnginePhase -= 2.0 * M_PI;
      }
      i2s_write(I2S_NUM, samples, count * 4, &bytesWritten, pdMS_TO_TICKS(10));
    }
    return;
  }
}

// BLE Server Callbacks
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      lastHeartbeatMs = millis();
    };
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      stopAll(); // EMERGENCY STOP ON BLE DISCONNECT!
      pServer->getAdvertising()->start();
    }
};

// BLE RX Callback
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String rxValue = pCharacteristic->getValue();
      if (rxValue.length() > 0) {
        deviceConnected = true;
        lastHeartbeatMs = millis();
        processCommand(rxValue);
      }
    }
};

// Direct BLE HID Gamepad Report Parser
void gamepadNotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
  if (length == 0) return;
  deviceConnected = true;
  lastHeartbeatMs = millis();

  // If text string passed over BLE
  if (length <= 12 && isprint(pData[0])) {
    String cmdStr = "";
    for (size_t i = 0; i < length; i++) cmdStr += (char)pData[i];
    processCommand(cmdStr);
    return;
  }

  // Parse binary HID Report for Wireless Gamepads
  uint8_t b1 = pData[0];
  uint8_t b2 = (length > 1) ? pData[1] : 0;

  // D-Pad / Buttons: Up=0x01, Down=0x02, Left=0x04, Right=0x08, Stop=0x10/0x20
  if (b1 & 0x01) processCommand("forward");
  else if (b1 & 0x02) processCommand("back");

  if (b1 & 0x04) processCommand("left");
  else if (b1 & 0x08) processCommand("right");
  else if (currentSteerState != STEER_IDLE && !(b1 & 0x04) && !(b1 & 0x08)) {
    stopSteering();
  }

  if (b2 & 0x01 || b2 & 0x02 || b1 & 0x10) {
    stopAll();
  }
}

Preferences prefs;
String savedGamepadMac = "";
BLEAdvertisedDevice* targetGamepadDevice = nullptr;
bool doConnectGamepad = false;
String bleStatusMsg = "Searching for controller...";
String connectedGamepadName = "";
unsigned long lastAutoScanMs = 0;

class GamepadClientCallbacks : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
    Serial.println("Gamepad Client Connected Callback");
  }
  void onDisconnect(BLEClient* pclient) {
    Serial.println(">>> WIRELESS GAMEPAD DISCONNECTED!");
    deviceConnected = false;
    connectedGamepadName = "";
    bleStatusMsg = "Gamepad disconnected. Auto-searching...";
    stopAll();
  }
};

// BLE Central Scanner Callbacks for discovering ANY Wireless Gamepad
class GamepadAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      String name = advertisedDevice.getName().c_str();
      String addr = advertisedDevice.getAddress().toString().c_str();
      Serial.print("BLE Device Found: ");
      Serial.print(name);
      Serial.print(" [");
      Serial.print(addr);
      Serial.println("]");

      bool isGamepad = advertisedDevice.isAdvertisingService(BLEUUID((uint16_t)0x1812)) ||
                       (advertisedDevice.haveAppearance() && (advertisedDevice.getAppearance() == 0x03C4 || advertisedDevice.getAppearance() == 0x03C3)) ||
                       name.indexOf("Controller") != -1 || 
                       name.indexOf("Gamepad") != -1 || 
                       name.indexOf("Joy-Con") != -1 || 
                       name.indexOf("Wireless") != -1 || 
                       name.indexOf("Pro") != -1 ||
                       name.indexOf("8BitDo") != -1 ||
                       name.indexOf("Xbox") != -1 ||
                       name.indexOf("DualSense") != -1 ||
                       name.indexOf("DualShock") != -1 ||
                       name.indexOf("DUALSHOCK") != -1 ||
                       name.indexOf("SHANWAN") != -1 ||
                       name.indexOf("ipega") != -1 ||
                       name.indexOf("Mocute") != -1 ||
                       name.indexOf("GameSir") != -1 ||
                       name.indexOf("Flydigi") != -1 ||
                       name.indexOf("GEN GAME") != -1 ||
                       name.indexOf("VR-") != -1 ||
                       name.indexOf("BT-") != -1 ||
                       name.indexOf("STK-") != -1 ||
                       name.indexOf("PG-") != -1;

      // Priority 1: Match saved controller MAC!
      bool isMatch = false;
      if (savedGamepadMac.length() > 0 && addr.equalsIgnoreCase(savedGamepadMac)) {
        isMatch = true;
        Serial.println(">>> MATCHED PREVIOUSLY SAVED GAMEPAD MAC: " + addr);
      } else if (isGamepad) {
        isMatch = true;
      }

      if (isMatch && !doConnectGamepad) {
        String displayName = name.length() > 0 ? name : ("Controller " + addr);
        Serial.println(">>> AUTO-CONNECTING TO GAMEPAD (" + displayName + ")...");
        bleStatusMsg = "Connecting to " + displayName + "...";
        
        targetGamepadDevice = new BLEAdvertisedDevice(advertisedDevice);
        doConnectGamepad = true;
        BLEDevice::getScan()->stop();
        isBleScanning = false;
      }
    }
};

GamepadAdvertisedDeviceCallbacks gamepadCallbacksInstance;

void onScanComplete(BLEScanResults results) {
  isBleScanning = false;
  if (!doConnectGamepad && connectedGamepadName.length() == 0) {
    if (savedGamepadMac.length() > 0) {
      bleStatusMsg = "Auto-searching for paired controller (" + savedGamepadMac + ")...";
    } else {
      bleStatusMsg = "Put controller in pairing mode (auto-scanning)...";
    }
  }
}

void startBleGamepadScan() {
  if (isBleScanning || doConnectGamepad) return;
  if (pGamepadClient != nullptr && pGamepadClient->isConnected()) return;

  isBleScanning = true;
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(&gamepadCallbacksInstance);
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(160); // 160ms interval
  pBLEScan->setWindow(40);    // 40ms window (leaves 75% radio time for Wi-Fi beacon & connection handshakes!)
  pBLEScan->start(2, onScanComplete, false); // Quick 2-second non-blocking auto-scan
}

void handleGamepadConnectionLoop() {
  // 1. Process pending connection
  if (doConnectGamepad && targetGamepadDevice != nullptr) {
    String devName = targetGamepadDevice->getName().c_str();
    String devAddr = targetGamepadDevice->getAddress().toString().c_str();
    if (devName.length() == 0) devName = "Wireless Controller";
    
    Serial.println("Connecting to BLE Gamepad: " + devName + " [" + devAddr + "]");
    if (pGamepadClient == nullptr) {
      pGamepadClient = BLEDevice::createClient();
      pGamepadClient->setClientCallbacks(new GamepadClientCallbacks());
    }
    
    if (pGamepadClient->connect(targetGamepadDevice)) {
      Serial.println(">>> PERMANENTLY CONNECTED TO WIRELESS GAMEPAD: " + devName);
      deviceConnected = true;
      connectedGamepadName = devName;
      bleStatusMsg = "CONNECTED: " + devName;
      playSuccessChime();

      // Persist Gamepad MAC in Non-Volatile Flash Memory!
      prefs.begin("tink_gp", false);
      prefs.putString("mac", devAddr);
      prefs.putString("name", devName);
      prefs.end();
      savedGamepadMac = devAddr;
      Serial.println("Saved Gamepad to Flash Memory: " + devAddr);

      // Discover HID Service & register notifications
      BLERemoteService* pRemoteService = pGamepadClient->getService(BLEUUID((uint16_t)0x1812));
      if (pRemoteService != nullptr) {
        BLERemoteCharacteristic* pChar = pRemoteService->getCharacteristic(BLEUUID((uint16_t)0x2A4D));
        if (pChar != nullptr && pChar->canNotify()) {
          pChar->registerForNotify(gamepadNotifyCallback);
        }
      }
    } else {
      Serial.println("Failed to connect to BLE Gamepad.");
      bleStatusMsg = "Connection attempt failed. Auto-retrying...";
    }
    
    doConnectGamepad = false;
    delete targetGamepadDevice;
    targetGamepadDevice = nullptr;
    return;
  }

  // 2. Standalone Background Auto-Reconnect: Scan automatically if no controller is connected and phone is not busy
  if (WiFi.softAPgetStationNum() == 0 && (pGamepadClient == nullptr || !pGamepadClient->isConnected())) {
    if (millis() - lastAutoScanMs > 6000 && !isBleScanning) {
      lastAutoScanMs = millis();
      startBleGamepadScan();
    }
  }
}

const char* htmlPage = R"HTML(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
  <title>TINK // InMoov Base Controller</title>
  <link href="https://fonts.googleapis.com/css2?family=Orbitron:wght@400;700;900&family=Rajdhani:wght@500;700&display=swap" rel="stylesheet">
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; user-select: none; -webkit-user-select: none; }
    body {
      font-family: 'Rajdhani', sans-serif;
      background-color: #07090e;
      background-image: 
        radial-gradient(circle at 50% 30%, rgba(0, 243, 255, 0.08) 0%, transparent 60%),
        linear-gradient(rgba(0, 243, 255, 0.03) 1px, transparent 1px),
        linear-gradient(90deg, rgba(0, 243, 255, 0.03) 1px, transparent 1px);
      background-size: 100% 100%, 25px 25px, 25px 25px;
      color: #e2e8f0;
      min-height: 100vh;
      display: flex;
      flex-direction: column;
      align-items: center;
      justify-content: space-between;
      padding: 12px;
      touch-action: manipulation;
    }
    
    .hud-header {
      text-align: center;
      margin-top: 5px;
      width: 100%;
    }
    .badge {
      display: inline-block;
      font-family: 'Orbitron', sans-serif;
      font-size: 10px;
      letter-spacing: 2px;
      color: #00f3ff;
      background: rgba(0, 243, 255, 0.1);
      border: 1px solid rgba(0, 243, 255, 0.3);
      padding: 3px 10px;
      border-radius: 4px;
      margin-bottom: 4px;
      text-transform: uppercase;
    }
    h1 {
      font-family: 'Orbitron', sans-serif;
      font-size: 34px;
      font-weight: 900;
      letter-spacing: 3px;
      color: #ffffff;
      text-shadow: 0 0 15px rgba(0, 243, 255, 0.6);
    }
    .subtitle {
      font-size: 11px;
      letter-spacing: 2px;
      color: #94a3b8;
      text-transform: uppercase;
    }

    /* Tab Navigation Bar */
    .tab-bar {
      display: flex;
      width: 100%;
      max-width: 360px;
      gap: 6px;
      margin: 10px 0;
    }
    .tab-btn {
      flex: 1;
      font-family: 'Orbitron', sans-serif;
      font-size: 11px;
      font-weight: 700;
      padding: 10px 0;
      color: #cbd5e1;
      background: rgba(15, 23, 42, 0.9);
      border: 1px solid #00f3ff;
      border-radius: 8px;
      cursor: pointer;
      text-transform: uppercase;
      transition: all 0.2s ease;
    }
    .tab-btn.active {
      color: #07090e;
      background: #00f3ff;
      box-shadow: 0 0 15px rgba(0, 243, 255, 0.6);
    }

    /* Tab Views */
    .tab-content {
      display: none;
      width: 100%;
      max-width: 360px;
      flex-direction: column;
      align-items: center;
    }
    .tab-content.active {
      display: flex;
    }

    /* Steering Section */
    .steering-section {
      position: relative;
      width: 100%;
      display: flex;
      flex-direction: column;
      align-items: center;
      margin: 8px 0;
    }
    
    .wheel-wrapper {
      position: relative;
      width: 140px;
      height: 140px;
      display: flex;
      align-items: center;
      justify-content: center;
      margin-bottom: 10px;
    }
    
    .steering-wheel-svg {
      width: 100%;
      height: 100%;
      transition: transform 0.15s cubic-bezier(0.4, 0, 0.2, 1), filter 0.15s ease;
      filter: drop-shadow(0 0 10px rgba(0, 243, 255, 0.3));
    }
    
    .wheel-wrapper.steer-left .steering-wheel-svg {
      transform: rotate(-35deg);
      filter: drop-shadow(0 0 15px #ffcc00);
    }
    .wheel-wrapper.steer-left .wheel-rim { stroke: #ffcc00; }
    
    .wheel-wrapper.steer-right .steering-wheel-svg {
      transform: rotate(35deg);
      filter: drop-shadow(0 0 15px #00a2ff);
    }
    .wheel-wrapper.steer-right .wheel-rim { stroke: #00a2ff; }

    .steer-controls {
      display: flex;
      width: 100%;
      gap: 10px;
    }

    .btn-steer {
      flex: 1;
      background: rgba(15, 23, 42, 0.8);
      border: 2px solid #334155;
      color: #f8fafc;
      font-family: 'Orbitron', sans-serif;
      font-size: 13px;
      font-weight: 700;
      padding: 14px 0;
      border-radius: 10px;
      cursor: pointer;
      display: flex;
      align-items: center;
      justify-content: center;
      gap: 6px;
      box-shadow: inset 0 0 15px rgba(255,255,255,0.02), 0 4px 12px rgba(0,0,0,0.5);
      transition: all 0.1s ease;
    }
    .btn-steer:active, .btn-steer.active {
      border-color: #00f3ff;
      background: rgba(0, 243, 255, 0.15);
      box-shadow: 0 0 20px rgba(0, 243, 255, 0.4);
      transform: scale(0.97);
    }

    .throttle-section {
      width: 100%;
      display: flex;
      flex-direction: column;
      gap: 10px;
    }

    .btn-throttle {
      font-family: 'Orbitron', sans-serif;
      font-size: 18px;
      font-weight: 900;
      letter-spacing: 2px;
      padding: 16px 0;
      border-radius: 12px;
      cursor: pointer;
      border: 2px solid transparent;
      display: flex;
      align-items: center;
      justify-content: center;
      gap: 8px;
      text-transform: uppercase;
      box-shadow: 0 6px 16px rgba(0,0,0,0.6);
      transition: all 0.1s ease;
    }

    .btn-go {
      background: linear-gradient(135deg, #052e16 0%, #14532d 100%);
      border-color: #22c55e;
      color: #4ade80;
      text-shadow: 0 0 10px rgba(34, 197, 94, 0.5);
    }
    .btn-go.active {
      background: #22c55e;
      color: #052e16;
      box-shadow: 0 0 30px rgba(34, 197, 94, 0.9);
      border-color: #ffffff;
    }

    .btn-stop {
      background: linear-gradient(135deg, #450a0a 0%, #7f1d1d 100%);
      border-color: #ef4444;
      color: #fca5a5;
      font-size: 20px;
      text-shadow: 0 0 10px rgba(239, 68, 68, 0.5);
    }
    .btn-stop:active {
      background: #ef4444;
      color: #ffffff;
      box-shadow: 0 0 30px rgba(239, 68, 68, 0.9);
      transform: scale(0.97);
    }

    .btn-back {
      background: linear-gradient(135deg, #451a03 0%, #78350f 100%);
      border-color: #f59e0b;
      color: #fcd34d;
      text-shadow: 0 0 10px rgba(245, 158, 11, 0.5);
    }
    .btn-back.active {
      background: #f59e0b;
      color: #451a03;
      box-shadow: 0 0 30px rgba(245, 158, 11, 0.9);
      border-color: #ffffff;
    }

    .utility-bar {
      display: flex;
      gap: 8px;
      width: 100%;
      margin-top: 10px;
    }
    .btn-util {
      flex: 1;
      background: rgba(30, 41, 59, 0.7);
      border: 1px solid #475569;
      color: #cbd5e1;
      font-family: 'Rajdhani', sans-serif;
      font-size: 13px;
      font-weight: 700;
      letter-spacing: 1px;
      padding: 10px 4px;
      border-radius: 8px;
      cursor: pointer;
      text-transform: uppercase;
      transition: all 0.15s ease;
    }
    .btn-util:active {
      background: #00f3ff;
      color: #07090e;
      border-color: #00f3ff;
    }

    /* Live GPIO Pin Diagnostics Grid */
    .pin-grid {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 8px;
      width: 100%;
      margin-top: 6px;
    }
    .pin-card {
      background: rgba(15, 23, 42, 0.9);
      border: 1px solid #334155;
      border-radius: 8px;
      padding: 8px 10px;
      display: flex;
      flex-direction: column;
      gap: 3px;
      transition: all 0.2s ease;
    }
    .pin-card.active-low {
      border-color: #22c55e;
      background: rgba(34, 197, 94, 0.15);
      box-shadow: 0 0 12px rgba(34, 197, 94, 0.4);
    }
    .pin-header {
      font-family: 'Orbitron', sans-serif;
      font-size: 10px;
      font-weight: 700;
      color: #94a3b8;
      display: flex;
      justify-content: space-between;
      align-items: center;
    }
    .pin-state-badge {
      font-size: 9px;
      font-weight: 900;
      padding: 2px 5px;
      border-radius: 4px;
      background: #334155;
      color: #cbd5e1;
    }
    .pin-card.active-low .pin-state-badge {
      background: #22c55e;
      color: #052e16;
    }
    .pin-expected {
      font-size: 9px;
      color: #64748b;
      font-family: monospace;
    }
    .pin-card.active-low .pin-expected {
      color: #86efac;
    }

    /* Speed & Steering Calibration Panel Styles */
    .panel-card {
      background: rgba(15, 23, 42, 0.95);
      border: 1px solid #334155;
      border-radius: 10px;
      padding: 12px;
      width: 100%;
      margin-top: 10px;
      box-sizing: border-box;
    }
    .panel-title {
      font-family: 'Orbitron', sans-serif;
      font-size: 11px;
      color: #00f3ff;
      font-weight: 700;
      letter-spacing: 1px;
      display: flex;
      justify-content: space-between;
      align-items: center;
      margin-bottom: 8px;
    }
    .control-slider {
      -webkit-appearance: none;
      appearance: none;
      width: 100%;
      height: 8px;
      border-radius: 4px;
      background: #1e293b;
      outline: none;
      border: 1px solid #475569;
      margin: 8px 0;
    }
    .control-slider::-webkit-slider-thumb {
      -webkit-appearance: none;
      appearance: none;
      width: 22px;
      height: 22px;
      border-radius: 50%;
      background: #00f3ff;
      cursor: pointer;
      box-shadow: 0 0 10px #00f3ff;
    }
    .preset-btn {
      flex: 1;
      background: rgba(30, 41, 59, 0.7);
      border: 1px solid #475569;
      color: #94a3b8;
      font-family: 'Rajdhani', sans-serif;
      font-size: 12px;
      font-weight: 700;
      padding: 6px 0;
      border-radius: 6px;
      cursor: pointer;
      transition: all 0.15s ease;
      text-transform: uppercase;
    }
    .preset-btn:active, .preset-btn.active {
      background: rgba(0, 243, 255, 0.2);
      border-color: #00f3ff;
      color: #00f3ff;
      box-shadow: 0 0 10px rgba(0, 243, 255, 0.4);
    }
    .pos-meter-track {
      width: 100%;
      height: 12px;
      background: #1e293b;
      border: 1px solid #334155;
      border-radius: 6px;
      position: relative;
      overflow: hidden;
      margin: 6px 0;
    }
    .pos-meter-marker {
      position: absolute;
      top: 0;
      bottom: 0;
      width: 8px;
      background: #00f3ff;
      border-radius: 4px;
      box-shadow: 0 0 8px #00f3ff;
      transform: translateX(-50%);
      left: 50%;
      transition: left 0.1s ease-out;
    }

    /* Documentation & Guide Tab Cards */
    .doc-card {
      background: rgba(15, 23, 42, 0.9);
      border: 1px solid #334155;
      border-radius: 10px;
      padding: 14px;
      width: 100%;
      margin-bottom: 10px;
      text-align: left;
    }
    .doc-card h3 {
      font-family: 'Orbitron', sans-serif;
      font-size: 13px;
      color: #00f3ff;
      margin-bottom: 8px;
      border-bottom: 1px solid rgba(0, 243, 255, 0.2);
      padding-bottom: 4px;
    }
    .doc-table {
      width: 100%;
      border-collapse: collapse;
      font-size: 11px;
      margin-top: 6px;
    }
    .doc-table th, .doc-table td {
      border: 1px solid #334155;
      padding: 6px;
      text-align: left;
    }
    .doc-table th {
      background: rgba(0, 243, 255, 0.1);
      color: #00f3ff;
    }
    code {
      font-family: monospace;
      color: #00ff88;
      background: rgba(0,0,0,0.4);
      padding: 2px 4px;
      border-radius: 3px;
    }
    pre {
      font-family: monospace;
      font-size: 11px;
      color: #38bdf8;
      background: rgba(0,0,0,0.5);
      padding: 8px;
      border-radius: 6px;
      overflow-x: auto;
      margin-top: 6px;
      white-space: pre-wrap;
    }

    .telemetry {
      font-family: 'Orbitron', sans-serif;
      font-size: 9px;
      color: #64748b;
      letter-spacing: 1px;
      margin-top: 8px;
      display: flex;
      align-items: center;
      gap: 6px;
    }
    .status-dot {
      width: 8px;
      height: 8px;
      background-color: #00f3ff;
      border-radius: 50%;
      box-shadow: 0 0 8px #00f3ff;
      animation: pulse 2s infinite;
    }
    @keyframes pulse {
      0% { opacity: 0.4; }
      50% { opacity: 1; }
      100% { opacity: 0.4; }
    }
  </style>
</head>
<body>

  <!-- Header -->
  <div class="hud-header">
    <div class="badge">IGNITE ROBOTICS SYSTEM</div>
    <h1>TINK</h1>
    <div class="subtitle">InMoov Base Platform Controller</div>
  </div>

  <!-- Tab Navigation Bar -->
  <div class="tab-bar">
    <button class="tab-btn active" id="btnTab1" onclick="switchTab('tabControls', this)">🎮 Controls</button>
    <button class="tab-btn" id="btnTab2" onclick="switchTab('tabWiring', this)">🔌 Wiring</button>
    <button class="tab-btn" id="btnTab3" onclick="switchTab('tabMRL', this)">🤖 RobotLab</button>
  </div>

  <!-- TAB 1: CONTROLLER UI -->
  <div class="tab-content active" id="tabControls">
    <div class="steering-section">
      <div class="wheel-wrapper" id="wheelWrapper">
        <svg class="steering-wheel-svg" viewBox="0 0 200 200">
          <circle class="wheel-rim" cx="100" cy="100" r="85" fill="none" stroke="#00f3ff" stroke-width="12" stroke-dasharray="140 10 140 10" />
          <circle cx="100" cy="100" r="72" fill="none" stroke="#1e293b" stroke-width="4" />
          <circle cx="100" cy="100" r="28" fill="#0f172a" stroke="#00f3ff" stroke-width="3" />
          <circle cx="100" cy="100" r="14" fill="#00f3ff" opacity="0.8" />
          <rect x="94" y="25" width="12" height="50" fill="#334155" rx="3" />
          <rect x="94" y="125" width="12" height="50" fill="#334155" rx="3" />
          <rect x="25" y="94" width="50" height="12" fill="#334155" rx="3" />
          <rect x="125" y="94" width="50" height="12" fill="#334155" rx="3" />
        </svg>
      </div>

      <div class="steer-controls">
        <button class="btn-steer" id="btnLeft"
          onmousedown="startSteer('left')" onmouseup="stopSteer()"
          ontouchstart="startSteer('left')" ontouchend="stopSteer()">
          &#9668; STEER LEFT (A)
        </button>
        <button class="btn-steer" id="btnRight"
          onmousedown="startSteer('right')" onmouseup="stopSteer()"
          ontouchstart="startSteer('right')" ontouchend="stopSteer()">
          STEER RIGHT (D) &#9658;
        </button>
      </div>
    </div>

    <div class="throttle-section">
      <button class="btn-throttle btn-go" id="btnGo" onclick="setDrive('forward')">
        &#9650; GO / FORWARD (W)
      </button>

      <button class="btn-throttle btn-stop" id="btnStop" onclick="triggerStop()">
        STOP / EMERGENCY (SPACE/X)
      </button>

      <button class="btn-throttle btn-back" id="btnBack" onclick="setDrive('back')">
        &#9660; BACK / REVERSE (S)
      </button>
    </div>

    <div class="utility-bar">
      <button class="btn-util" onclick="sendCommand('center')">Center Steering (C)</button>
      <button class="btn-util" onclick="sendCommand('set_center')">Set Center (Z)</button>
    </div>

    <!-- 1. DYNAMIC DRIVE SPEED GOVERNOR PANEL -->
    <div class="panel-card">
      <div class="panel-title">
        <span>⚡ DRIVE SPEED GOVERNOR</span>
        <span id="speedLabel" style="color:#00ff88; font-size:12px; font-weight:900;">60% SPEED</span>
      </div>
      <input type="range" id="speedSlider" class="control-slider" min="20" max="100" value="60" oninput="onSpeedChange(this.value)">
      <div style="display:flex; gap:6px; margin-top:6px;">
        <button class="preset-btn" id="btnSpdSlow" onclick="setSpeedPreset(35)">🐢 SLOW (35%)</button>
        <button class="preset-btn active" id="btnSpdMed" onclick="setSpeedPreset(65)">🚗 CRUISE (65%)</button>
        <button class="preset-btn" id="btnSpdFast" onclick="setSpeedPreset(100)">⚡ TURBO (100%)</button>
      </div>
    </div>

    <!-- 2. STEERING TRAVEL LIMITS & MIN/MAX CALIBRATION PANEL -->
    <div class="panel-card">
      <div class="panel-title">
        <span>📐 STEERING LIMITS & CALIBRATION</span>
        <span id="posLabel" style="color:#38bdf8; font-size:11px;">POS: 0 ms</span>
      </div>
      
      <!-- Live Steering Travel Bar -->
      <div style="display:flex; justify-content:space-between; font-size:9px; color:#64748b; font-family:monospace;">
        <span id="leftLimitDisplay">LEFT: -600ms</span>
        <span>CENTER: 0</span>
        <span id="rightLimitDisplay">RIGHT: +600ms</span>
      </div>
      <div class="pos-meter-track">
        <div class="pos-meter-marker" id="posMarker" style="left:50%;"></div>
      </div>

      <!-- Limit Sliders -->
      <div style="display:grid; grid-template-columns:1fr 1fr; gap:8px; margin-top:8px;">
        <div>
          <div style="font-size:9px; color:#94a3b8; font-weight:bold;">LEFT MAX LOCK (ms)</div>
          <input type="range" id="leftLimitSlider" class="control-slider" min="100" max="2000" step="25" value="600" oninput="onSteerLimitsChange()">
        </div>
        <div>
          <div style="font-size:9px; color:#94a3b8; font-weight:bold;">RIGHT MAX LOCK (ms)</div>
          <input type="range" id="rightLimitSlider" class="control-slider" min="100" max="2000" step="25" value="600" oninput="onSteerLimitsChange()">
        </div>
      </div>

      <!-- Quick 1-Tap Calibration Locks -->
      <div style="display:flex; gap:6px; margin-top:8px;">
        <button class="preset-btn" onclick="setLeftLimitNow()" style="color:#fcd34d; border-color:#f59e0b;">👈 LOCK LEFT</button>
        <button class="preset-btn" onclick="sendCommand('set_center')" style="color:#00f3ff; border-color:#00f3ff;">🎯 ZERO CENTER</button>
        <button class="preset-btn" onclick="setRightLimitNow()" style="color:#60a5fa; border-color:#3b82f6;">👉 LOCK RIGHT</button>
      </div>
    </div>

    <!-- 3. AUDIO & SOUND VOLUME GOVERNOR -->
    <div class="panel-card">
      <div class="panel-title">
        <span>🔊 AUDIO VOLUME & MUTE</span>
        <span id="volLabel" style="color:#22c55e; font-size:12px; font-weight:900;">100% VOLUME</span>
      </div>
      <input type="range" id="volSlider" class="control-slider" min="0" max="100" value="100" oninput="onVolumeChange(this.value)">
      <div style="display:flex; gap:6px; margin-top:6px;">
        <button class="preset-btn" id="btnMute" onclick="toggleMuteAudio()" style="color:#ef4444; border-color:#ef4444;">🔇 MUTE</button>
        <button class="preset-btn" onclick="setVolumePreset(50)">🔉 50%</button>
        <button class="preset-btn active" id="btnVol100" onclick="setVolumePreset(100)">🔊 100% BOOST</button>
      </div>
    </div>

    <!-- Wireless Gamepad Pairing & Auto-Connect Bar -->
    <div style="display:flex; gap:8px; width:100%; margin-top:8px;">
      <button class="btn-util" onclick="triggerBleScan()" style="border-color:#00f3ff; color:#00f3ff; font-weight:bold; padding:10px 0; flex:1;">
        🎮 PAIR CONTROLLER
      </button>
      <button class="btn-util" onclick="forgetGamepad()" style="border-color:#ef4444; color:#ef4444; font-weight:bold; padding:10px 0; max-width:90px;">
        FORGET
      </button>
    </div>

    <div id="pairStatus" style="font-size:10px; color:#94a3b8; margin-top:4px; font-family:'Orbitron',sans-serif; text-align:center;">
      STANDALONE AUTO-RECONNECT ACTIVE: TURN ON CONTROLLER TO DRIVE!
    </div>

    <!-- LIVE GPIO PIN DIAGNOSTICS HUD -->
    <div style="width:100%; margin-top:12px;">
      <div style="font-family:'Orbitron',sans-serif; font-size:11px; color:#00f3ff; font-weight:700; letter-spacing:1px; display:flex; justify-content:space-between; align-items:center;">
        <span>⚡ LIVE GPIO SIGNAL MONITOR</span>
        <span style="font-size:9px; color:#64748b;">MULTIMETER DIAGNOSTICS</span>
      </div>

      <div class="pin-grid">
        <!-- GPIO 4: Forward -->
        <div class="pin-card" id="cardPin4">
          <div class="pin-header">
            <span>GPIO 4 [IN1]</span>
            <span class="pin-state-badge" id="badgePin4">0.0V (OFF)</span>
          </div>
          <div style="font-size:11px; font-weight:bold; color:#f8fafc;">Forward Drive</div>
          <div class="pin-expected" id="expPin4">Signal: 0.0V (DISENGAGED)</div>
        </div>

        <!-- GPIO 5: Reverse -->
        <div class="pin-card" id="cardPin5">
          <div class="pin-header">
            <span>GPIO 5 [IN2]</span>
            <span class="pin-state-badge" id="badgePin5">0.0V (OFF)</span>
          </div>
          <div style="font-size:11px; font-weight:bold; color:#f8fafc;">Reverse Drive</div>
          <div class="pin-expected" id="expPin5">Signal: 0.0V (DISENGAGED)</div>
        </div>

        <!-- GPIO 6: Steer Left -->
        <div class="pin-card" id="cardPin6">
          <div class="pin-header">
            <span>GPIO 6 [IN3]</span>
            <span class="pin-state-badge" id="badgePin6">0.0V (OFF)</span>
          </div>
          <div style="font-size:11px; font-weight:bold; color:#f8fafc;">Steer Left</div>
          <div class="pin-expected" id="expPin6">Signal: 0.0V (DISENGAGED)</div>
        </div>

        <!-- GPIO 7: Steer Right -->
        <div class="pin-card" id="cardPin7">
          <div class="pin-header">
            <span>GPIO 7 [IN4]</span>
            <span class="pin-state-badge" id="badgePin7">0.0V (OFF)</span>
          </div>
          <div style="font-size:11px; font-weight:bold; color:#f8fafc;">Steer Right</div>
          <div class="pin-expected" id="expPin7">Signal: 0.0V (DISENGAGED)</div>
        </div>
      </div>
    </div>
  </div>

  <!-- TAB 2: WIRING SPECS & DIAGRAM -->
  <div class="tab-content" id="tabWiring">
    <div class="doc-card">
      <h3>ESP32-S3 PINOUT TABLE</h3>
      <table class="doc-table">
        <tr><th>ESP32 Pin</th><th>Target Module</th><th>Target Function</th></tr>
        <tr><td><code>GPIO 4</code></td><td>IN1 Relay</td><td>Drive Forward (6V)</td></tr>
        <tr><td><code>GPIO 5</code></td><td>IN2 Relay</td><td>Drive Backward (6V)</td></tr>
        <tr><td><code>GPIO 6</code></td><td>IN3 Relay</td><td>Steer Left (12V)</td></tr>
        <tr><td><code>GPIO 7</code></td><td>IN4 Relay</td><td>Steer Right (12V)</td></tr>
        <tr><td><code>GPIO 18</code></td><td>MAX98357A DIN</td><td>I2S Audio Data</td></tr>
        <tr><td><code>GPIO 17</code></td><td>MAX98357A BCLK</td><td>I2S Bit Clock</td></tr>
        <tr><td><code>GPIO 16</code></td><td>MAX98357A LRC</td><td>I2S Word Select Clock</td></tr>
        <tr><td><code>VIN / 5V</code></td><td>VCC</td><td>Relay & Amp Power</td></tr>
        <tr><td><code>GND</code></td><td>GND</td><td>Common Ground</td></tr>
      </table>
    </div>

    <div class="doc-card">
      <h3>🔊 MAX98357A I2S AUDIO AMP WIRING & HARDWARE GAIN BOOST</h3>
      <p style="font-size:12px; color:#cbd5e1; line-height:1.4;">
        <b>MAX98357A Pinout to ESP32-S3:</b><br>
        • <b>DIN</b> ➔ <code>GPIO 18</code> (I2S Data)<br>
        • <b>BCLK</b> ➔ <code>GPIO 17</code> (I2S Bit Clock)<br>
        • <b>LRC</b> ➔ <code>GPIO 16</code> (I2S Word Select)<br>
        • <b>VIN</b> ➔ <code>5V / VIN</code> (Powering from 5V provides 3.2 Watts vs <1W on 3.3V!)<br>
        • <b>GND</b> ➔ <code>ESP32 GND</code><br><br>
        <b>🚀 Hardware Analog Volume Boost Settings (GAIN Pin):</b><br>
        • <b>Default (Floating):</b> 9dB gain<br>
        • <b>GAIN ➔ GND directly:</b> 12dB gain (+3dB louder)<br>
        • <b>GAIN ➔ GND through 100kΩ resistor:</b> 15dB gain (+6dB MAXIMUM BOOST!)<br>
      </p>
    </div>

    <div class="doc-card">
      <h3>H-BRIDGE & RELAY WIRING</h3>
      <p style="font-size:12px; color:#cbd5e1; line-height:1.4;">
        <b>Relay 1 & 2 (Drive 6V):</b><br>
        Relay 1 COM & Relay 2 COM connect to Motor Terminals A & B.<br>
        Relay NOs = +6V. Relay NCs = Ground.<br><br>
        <b>Relay 3 & 4 (Actuator 12V):</b><br>
        Relay 3 COM & Relay 4 COM connect to Actuator Terminals.<br>
        Relay NOs = +12V. Relay NCs = Ground.<br><br>
        <span style="color:#ef4444;">⚠️ COMMON GROUND REQUIRED: Connect ESP32 GND, Amp GND, 6V GND, and 12V GND together.</span>
      </p>
    </div>
  </div>

  <!-- TAB 3: MYROBOTLAB GUIDE -->
  <div class="tab-content" id="tabMRL">
    <div class="doc-card">
      <h3>MYROBOTLAB BLE / SERIAL SETUP</h3>
      <p style="font-size:12px; color:#cbd5e1; line-height:1.4;">
        TINK supports WASD, single-letter, and word commands via Serial USB (115200) and BLE.<br><br>
        <b>BLE Name:</b> <code>TINK_Base_Platform</code><br>
        <b>BLE Service UUID:</b><br><code>6E400001-B5A3-F393-E0A9-E50E24DCCA9E</code>
      </p>
    </div>

    <div class="doc-card">
      <h3>MRL PYTHON SCRIPT</h3>
      <pre># MyRobotLab Python Script (WASD / Arrows Supported)
serial = runtime.start("serial", "Serial")
serial.connect("COM9", 115200)

# Movement Commands:
serial.write("W\n") # Drive Forward
serial.write("S\n") # Reverse
serial.write("A\n") # Steer Left
serial.write("D\n") # Steer Right
serial.write("X\n") # Emergency Stop
serial.write("C\n") # Auto-Center Steering</pre>
    </div>

    <div class="doc-card">
      <h3>COMMAND REFERENCE CHART</h3>
      <table class="doc-table">
        <tr><th>WASD / Keys</th><th>Single / Full</th><th>Action</th></tr>
        <tr><td><code>W</code> / <code>↑</code></td><td><code>F</code> / <code>forward</code></td><td>Sticky Drive Forward</td></tr>
        <tr><td><code>S</code> / <code>↓</code></td><td><code>B</code> / <code>back</code></td><td>Sticky Drive Reverse</td></tr>
        <tr><td><code>A</code> / <code>←</code></td><td><code>L</code> / <code>left</code></td><td>Steer Left</td></tr>
        <tr><td><code>D</code> / <code>→</code></td><td><code>R</code> / <code>right</code></td><td>Steer Right</td></tr>
        <tr><td><code>Space</code> / <code>X</code></td><td><code>S</code> / <code>stop</code></td><td>Emergency Stop All</td></tr>
        <tr><td><code>C</code></td><td><code>center</code></td><td>Auto-Center Steering</td></tr>
      </table>
    </div>
  </div>

  <!-- Telemetry Status -->
  <div class="telemetry">
    <span class="status-dot"></span>
    <span>SYSTEM ONLINE // LIVE MULTIMETER PIN MONITOR ACTIVE</span>
  </div>

  <script>
    const wheelWrapper = document.getElementById('wheelWrapper');
    const btnGo = document.getElementById('btnGo');
    const btnBack = document.getElementById('btnBack');

    const switchTab = (tabId, btn) => {
      document.querySelectorAll('.tab-content').forEach(el => el.classList.remove('active'));
      document.querySelectorAll('.tab-btn').forEach(el => el.classList.remove('active'));
      document.getElementById(tabId).classList.add('active');
      btn.classList.add('active');
    };

    const updatePinHud = (pinNum, isOn) => {
      const card = document.getElementById('cardPin' + pinNum);
      const badge = document.getElementById('badgePin' + pinNum);
      const exp = document.getElementById('expPin' + pinNum);
      if (isOn) {
        card.classList.add('active-low');
        badge.innerText = '3.3V (HIGH)';
        exp.innerText = 'Signal: 3.3V SOURCE (ENGAGED)';
      } else {
        card.classList.remove('active-low');
        badge.innerText = '0.0V (OFF)';
        exp.innerText = 'Signal: 0.0V (DISENGAGED)';
      }
    };

    const sendCommand = (cmd) => {
      fetch('/' + cmd)
        .then(r => r.text())
        .then(() => fetchStatus())
        .catch(e => console.error(e));
    };

    let currentLeftLimit = 600;
    let currentRightLimit = 600;

    const updateSteerMeter = (pos) => {
      if (pos === undefined) return;
      document.getElementById('posLabel').innerText = 'POS: ' + pos + ' ms';
      const maxSpan = Math.max(200, currentLeftLimit + currentRightLimit);
      let pct = ((pos + currentLeftLimit) / maxSpan) * 100;
      pct = Math.max(5, Math.min(95, pct));
      document.getElementById('posMarker').style.left = pct + '%';
    };

    const onSpeedChange = (val) => {
      document.getElementById('speedLabel').innerText = val + '% SPEED';
      fetch('/set_speed?val=' + val);
      updateSpeedPresetButtons(val);
    };

    const setSpeedPreset = (val) => {
      document.getElementById('speedSlider').value = val;
      onSpeedChange(val);
    };

    const updateSpeedPresetButtons = (val) => {
      val = parseInt(val);
      document.getElementById('btnSpdSlow').classList.toggle('active', val <= 40);
      document.getElementById('btnSpdMed').classList.toggle('active', val > 40 && val < 90);
      document.getElementById('btnSpdFast').classList.toggle('active', val >= 90);
    };

    const onSteerLimitsChange = () => {
      const leftVal = document.getElementById('leftLimitSlider').value;
      const rightVal = document.getElementById('rightLimitSlider').value;
      currentLeftLimit = parseInt(leftVal);
      currentRightLimit = parseInt(rightVal);
      document.getElementById('leftLimitDisplay').innerText = 'LEFT: -' + leftVal + 'ms';
      document.getElementById('rightLimitDisplay').innerText = 'RIGHT: +' + rightVal + 'ms';
      fetch('/set_steer_limits?left=' + leftVal + '&right=' + rightVal);
    };

    const setLeftLimitNow = () => {
      fetch('/set_left_limit')
        .then(r => r.json())
        .then(d => {
          if (d.left) {
            document.getElementById('leftLimitSlider').value = d.left;
            onSteerLimitsChange();
          }
        });
    };

    const setRightLimitNow = () => {
      fetch('/set_right_limit')
        .then(r => r.json())
        .then(d => {
          if (d.right) {
            document.getElementById('rightLimitSlider').value = d.right;
            onSteerLimitsChange();
          }
        });
    };

    let isAudioMuted = false;

    const onVolumeChange = (val) => {
      isAudioMuted = false;
      document.getElementById('volLabel').innerText = val + '% VOLUME';
      document.getElementById('btnMute').classList.remove('active');
      document.getElementById('btnVol100').classList.toggle('active', val == 100);
      fetch('/set_volume?val=' + val);
    };

    const setVolumePreset = (val) => {
      document.getElementById('volSlider').value = val;
      onVolumeChange(val);
    };

    const toggleMuteAudio = () => {
      fetch('/toggle_mute')
        .then(r => r.json())
        .then(d => {
          isAudioMuted = d.muted;
          if (isAudioMuted) {
            document.getElementById('volLabel').innerText = '🔇 MUTED';
            document.getElementById('btnMute').classList.add('active');
          } else {
            document.getElementById('volLabel').innerText = d.vol + '% VOLUME';
            document.getElementById('btnMute').classList.remove('active');
          }
        });
    };

    const fetchConfig = () => {
      fetch('/get_config')
        .then(r => r.json())
        .then(cfg => {
          if (cfg.speed) {
            document.getElementById('speedSlider').value = cfg.speed;
            document.getElementById('speedLabel').innerText = cfg.speed + '% SPEED';
            updateSpeedPresetButtons(cfg.speed);
          }
          if (cfg.steer_left) {
            document.getElementById('leftLimitSlider').value = cfg.steer_left;
            currentLeftLimit = parseInt(cfg.steer_left);
            document.getElementById('leftLimitDisplay').innerText = 'LEFT: -' + cfg.steer_left + 'ms';
          }
          if (cfg.steer_right) {
            document.getElementById('rightLimitSlider').value = cfg.steer_right;
            currentRightLimit = parseInt(cfg.steer_right);
            document.getElementById('rightLimitDisplay').innerText = 'RIGHT: +' + cfg.steer_right + 'ms';
          }
          if (cfg.vol !== undefined) {
            document.getElementById('volSlider').value = cfg.vol;
            document.getElementById('volLabel').innerText = cfg.mute ? '🔇 MUTED' : (cfg.vol + '% VOLUME');
            document.getElementById('btnMute').classList.toggle('active', cfg.mute);
            document.getElementById('btnVol100').classList.toggle('active', !cfg.mute && cfg.vol == 100);
          }
          if (cfg.pos !== undefined) {
            updateSteerMeter(cfg.pos);
          }
        })
        .catch(() => {});
    };
    fetchConfig();

    const fetchStatus = () => {
      fetch('/status')
        .then(r => r.json())
        .then(d => {
          updatePinHud(4, d.pin4 === 'ON');
          updatePinHud(5, d.pin5 === 'ON');
          updatePinHud(6, d.pin6 === 'ON');
          updatePinHud(7, d.pin7 === 'ON');
          if (d.pos !== undefined) {
            updateSteerMeter(d.pos);
          }
        })
        .catch(() => {});
    };

    // Gentle Heartbeat & Live Pin Status Poll (every 1.5 seconds to prevent TCP socket exhaustion)
    setInterval(fetchStatus, 1500);

    let scanPollTimer = null;
    const triggerBleScan = () => {
      const statusEl = document.getElementById('pairStatus');
      statusEl.innerText = '🔍 SCANNING FOR WIRELESS CONTROLLER (10s)...';
      statusEl.style.color = '#00f3ff';
      fetch('/scan_ble')
        .then(res => res.text())
        .then(msg => {
          statusEl.innerText = msg;
          statusEl.style.color = '#00f3ff';
          if (scanPollTimer) clearInterval(scanPollTimer);
          scanPollTimer = setInterval(pollBleStatus, 750);
        })
        .catch(err => {
          statusEl.innerText = 'SCAN ERROR: ' + err;
          statusEl.style.color = '#ef4444';
        });
    };

    const pollBleStatus = () => {
      fetch('/ble_status')
        .then(r => r.json())
        .then(data => {
          const statusEl = document.getElementById('pairStatus');
          if (data.status) {
            statusEl.innerText = data.status;
            if (data.connected && data.connected.length > 0) {
              statusEl.style.color = '#00ff88';
              if (scanPollTimer) { clearInterval(scanPollTimer); scanPollTimer = null; }
            } else if (!data.scanning) {
              statusEl.style.color = '#cbd5e1';
              if (scanPollTimer) { clearInterval(scanPollTimer); scanPollTimer = null; }
            } else {
              statusEl.style.color = '#00f3ff';
            }
          }
        })
        .catch(() => {});
    };

    const forgetGamepad = () => {
      fetch('/forget_gamepad')
        .then(r => r.text())
        .then(() => {
          const statusEl = document.getElementById('pairStatus');
          statusEl.innerText = 'CONTROLLER FORGOTTEN. PUT NEW CONTROLLER IN PAIRING MODE!';
          statusEl.style.color = '#f59e0b';
        });
    };

    const setDrive = (dir) => {
      if (dir === 'forward') {
        btnGo.classList.add('active');
        btnBack.classList.remove('active');
        sendCommand('forward');
      } else if (dir === 'back') {
        btnBack.classList.add('active');
        btnGo.classList.remove('active');
        sendCommand('back');
      }
    };

    const startSteer = (dir) => {
      if (dir === 'left') {
        wheelWrapper.className = 'wheel-wrapper steer-left';
        sendCommand('left');
      } else {
        wheelWrapper.className = 'wheel-wrapper steer-right';
        sendCommand('right');
      }
    };

    const stopSteer = () => {
      wheelWrapper.className = 'wheel-wrapper';
      sendCommand('stop_steer');
    };

    const triggerStop = () => {
      btnGo.classList.remove('active');
      btnBack.classList.remove('active');
      wheelWrapper.className = 'wheel-wrapper';
      sendCommand('stop');
    };

    // Keyboard WASD & Arrow Keys Support
    let isLeftPressed = false;
    let isRightPressed = false;

    document.addEventListener('keydown', (e) => {
      if (e.repeat) return;
      const k = e.key.toLowerCase();
      if (k === 'w' || e.key === 'ArrowUp') {
        setDrive('forward');
      } else if (k === 's' || e.key === 'ArrowDown') {
        setDrive('back');
      } else if (k === 'a' || e.key === 'ArrowLeft') {
        if (!isLeftPressed) {
          isLeftPressed = true;
          startSteer('left');
        }
      } else if (k === 'd' || e.key === 'ArrowRight') {
        if (!isRightPressed) {
          isRightPressed = true;
          startSteer('right');
        }
      } else if (e.key === ' ' || k === 'x') {
        triggerStop();
      } else if (k === 'c') {
        sendCommand('center');
      } else if (k === 'z') {
        sendCommand('set_center');
      }
    });

    document.addEventListener('keyup', (e) => {
      const k = e.key.toLowerCase();
      if (k === 'a' || e.key === 'ArrowLeft') {
        isLeftPressed = false;
        stopSteer();
      } else if (k === 'd' || e.key === 'ArrowRight') {
        isRightPressed = false;
        stopSteer();
      }
    });

    // Native HTML5 Browser Gamepad Support (Bluetooth/USB Controllers on Phone/PC)
    let gpDriveState = 'stop';
    let gpSteerState = 'idle';

    window.addEventListener("gamepadconnected", (e) => {
      const statusEl = document.getElementById('pairStatus');
      statusEl.innerText = '🎮 CONTROLLER CONNECTED VIA PHONE/BROWSER: ' + e.gamepad.id;
      statusEl.style.color = '#00ff88';
    });

    const pollBrowserGamepad = () => {
      const gamepads = navigator.getGamepads ? navigator.getGamepads() : [];
      for (let gp of gamepads) {
        if (!gp) continue;
        const stickY = gp.axes.length > 1 ? gp.axes[1] : 0;
        const stickX = gp.axes.length > 0 ? gp.axes[0] : 0;

        const btnA = gp.buttons[0] && gp.buttons[0].pressed;
        const btnB = gp.buttons[1] && gp.buttons[1].pressed;
        const btnX = gp.buttons[2] && gp.buttons[2].pressed;
        const dUp = gp.buttons[12] && gp.buttons[12].pressed;
        const dDown = gp.buttons[13] && gp.buttons[13].pressed;
        const dLeft = gp.buttons[14] && gp.buttons[14].pressed;
        const dRight = gp.buttons[15] && gp.buttons[15].pressed;

        // Drive (Forward / Back / Stop)
        if (dUp || btnA || stickY < -0.35) {
          if (gpDriveState !== 'forward') {
            setDrive('forward');
            gpDriveState = 'forward';
          }
        } else if (dDown || stickY > 0.35) {
          if (gpDriveState !== 'back') {
            setDrive('back');
            gpDriveState = 'back';
          }
        } else if (btnB || btnX) {
          if (gpDriveState !== 'stop') {
            triggerStop();
            gpDriveState = 'stop';
          }
        } else if (gpDriveState !== 'stop' && Math.abs(stickY) < 0.2 && !dUp && !dDown) {
          sendCommand('stop_drive');
          btnGo.classList.remove('active');
          btnBack.classList.remove('active');
          gpDriveState = 'stop';
        }

        // Steer (Left / Right / Idle)
        if (dLeft || stickX < -0.35) {
          if (gpSteerState !== 'left') {
            startSteer('left');
            gpSteerState = 'left';
          }
        } else if (dRight || stickX > 0.35) {
          if (gpSteerState !== 'right') {
            startSteer('right');
            gpSteerState = 'right';
          }
        } else if (gpSteerState !== 'idle' && Math.abs(stickX) < 0.2 && !dLeft && !dRight) {
          stopSteer();
          gpSteerState = 'idle';
        }
      }
      requestAnimationFrame(pollBrowserGamepad);
    };
    requestAnimationFrame(pollBrowserGamepad);

    document.addEventListener('contextmenu', event => event.preventDefault());
  </script>
</body>
</html>
)HTML";

void stopDriving() {
  setRelayOff(pinForward);
  setRelayOff(pinBack);
  currentDriveState = DRIVE_STOP;
}

void updateSteeringPosition() {
  unsigned long now = millis();
  long delta = now - lastSteerUpdateMs;
  
  if (currentSteerState == STEER_LEFT) {
    currentPositionMs -= delta;
  } else if (currentSteerState == STEER_RIGHT) {
    currentPositionMs += delta;
  }
  lastSteerUpdateMs = now;
}

void stopSteering() {
  if (currentSteerState != STEER_IDLE) {
     updateSteeringPosition();
  }
  setRelayOff(pinLeft);
  setRelayOff(pinRight);
  currentSteerState = STEER_IDLE;
}

void stopAll() {
  stopDriving();
  stopSteering();
  isCentering = false;
  playStopBeep();
}

void playBootChime() {
  playIgnitionSound();
}

void playSuccessChime() {
  playI2STone(523, 100); delay(40);
  playI2STone(1046, 200); delay(50);
}

void playStopBeep() {
  playI2STone(440, 100); delay(40);
  playI2STone(220, 150); delay(50);
}

void triggerSetCenterFlash() {
  playSuccessChime();
  for (int i = 0; i < 3; i++) {
    setLedColor(0, 255, 255); // Cyan
    delay(80);
    setLedColor(0, 0, 0);
    delay(80);
  }
}

void processCommand(String cmd) {
  cmd.trim();
  lastHeartbeatMs = millis(); // Refresh heartbeat on every command

  if (cmd == "forward" || cmd == "F" || cmd == "f" || cmd == "W" || cmd == "w") {
    stopDriving(); delay(30);
    currentDriveState = DRIVE_FORWARD;
    forwardCycleStartMs = millis();
    if (driveSpeedPercent >= 100) setRelayOn(pinForward);
  }
  else if (cmd == "back" || cmd == "B" || cmd == "b" || cmd == "S" || cmd == "s") {
    stopDriving(); delay(30);
    currentDriveState = DRIVE_BACK;
    if (driveSpeedPercent >= 100) setRelayOn(pinBack);
  }
  else if (cmd == "left" || cmd == "L" || cmd == "l" || cmd == "A" || cmd == "a") {
    isCentering = false;
    if (currentPositionMs > -steerLimitLeftMs) {
      stopSteering(); delay(30);
      currentSteerState = STEER_LEFT;
      lastSteerUpdateMs = millis();
      setRelayOn(pinLeft);
    }
  }
  else if (cmd == "right" || cmd == "R" || cmd == "r" || cmd == "D" || cmd == "d") {
    isCentering = false;
    if (currentPositionMs < steerLimitRightMs) {
      stopSteering(); delay(30);
      currentSteerState = STEER_RIGHT;
      lastSteerUpdateMs = millis();
      setRelayOn(pinRight);
    }
  }
  else if (cmd == "stop_steer" || cmd == "t") {
    isCentering = false;
    stopSteering();
  }
  else if (cmd == "stop_drive") {
    stopDriving();
  }
  else if (cmd == "stop" || cmd == "X" || cmd == "x" || cmd == " ") {
    stopAll();
  }
  else if (cmd == "center" || cmd == "C" || cmd == "c") {
    isCentering = true;
    playI2STone(500, 100);
  }
  else if (cmd == "set_center" || cmd == "Z" || cmd == "z") {
    isCentering = false;
    stopSteering();
    currentPositionMs = 0;
    triggerSetCenterFlash();
  }
}

void updateLedStatus() {
  // 0. IDLE RAINBOW FADE: If NO device is connected (0 WiFi stations and 0 BLE connections), run smooth Rainbow Idle Fade!
  if (!isDeviceConnected()) {
    if (millis() - lastRainbowMs >= 15) {
      lastRainbowMs = millis();
      rainbowHue = (rainbowHue + 1) % 360;
      setLedHue(rainbowHue);
    }
    return;
  }

  // 1. If active Centering is running, flash alternating Yellow & Blue + Tone!
  if (isCentering) {
    if (millis() - centerAnimMs >= 120) {
      centerAnimMs = millis();
      centerColorState = !centerColorState;
    }
    if (centerColorState) {
      setLedColor(255, 255, 0); // Yellow
      playI2STone(550, 80);
    } else {
      setLedColor(0, 0, 255);   // Blue
      playI2STone(700, 80);
    }
    return;
  }

  // 2. If actively turning Left, show Yellow
  if (currentSteerState == STEER_LEFT) {
    setLedColor(255, 255, 0); // Yellow
    return;
  }
  // 3. If actively turning Right, show Blue
  if (currentSteerState == STEER_RIGHT) {
    setLedColor(0, 0, 255); // Blue
    return;
  }

  // 4. Steering is idle -> Revert to current Drive state color
  if (currentDriveState == DRIVE_FORWARD) {
    setLedColor(0, 255, 0); // Solid Green for Forward
  } 
  else if (currentDriveState == DRIVE_BACK) {
    if (millis() - lastBlinkMs >= 300) {
      lastBlinkMs = millis();
      blinkState = !blinkState;
      if (blinkState) {
        playI2STone(850, 120); // Backup Warning Beep!
      }
    }
    if (blinkState) {
      setLedColor(255, 0, 0); // Red ON
    } else {
      setLedColor(0, 0, 0);   // Red OFF
    }
  } 
  else {
    setLedColor(255, 0, 0); // Solid Red for Stopped / Neutral
  }
}

void sendHtmlWithNoCache() {
  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Expires", "-1");
  server.send(200, "text/html", htmlPage);
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);
  
  // 1. Initialize WiFi Access Point IMMEDIATELY (Priority #1)
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.softAP(ssid, password, 1, 0, 4);
  Serial.println("Starting TinkDrive Access Point...");
  Serial.print("AP IP Address: ");
  Serial.println(WiFi.softAPIP()); 
  
  // Start DNS Server for Captive Portal
  dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());

  // 2. Initialize MAX98357A I2S Digital Audio
  initI2SAudio();

  // 3. Initialize all relay pins to High-Impedance Pullup DISENGAGED mode
  setRelayOff(pinForward);
  setRelayOff(pinBack);
  setRelayOff(pinLeft);
  setRelayOff(pinRight);
  
  stopDriving();
  stopSteering();
  playBootChime();

  // 4. Initialize BLE Device
  BLEDevice::init("TINK_Base_Platform");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create TX Characteristic
  pTxCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_TX,
                        BLECharacteristic::PROPERTY_NOTIFY
                      );
  pTxCharacteristic->addDescriptor(new BLE2902());

  // Create RX Characteristic
  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
                                           CHARACTERISTIC_UUID_RX,
                                           BLECharacteristic::PROPERTY_WRITE
                                         );
  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // Start BLE Service and Advertising
  pService->start();
  pServer->getAdvertising()->start();
  Serial.println("BLE UART (TINK_Base_Platform) Started");
  
  // 5. Load Persistent Speed, Steering Calibration Limits, & Audio Volume from Flash
  prefs.begin("tink_cfg", true);
  driveSpeedPercent = prefs.getInt("speed", 60);
  steerLimitLeftMs = prefs.getLong("s_left", 600);
  steerLimitRightMs = prefs.getLong("s_right", 600);
  audioVolumePercent = prefs.getInt("vol", 100);
  audioMuted = prefs.getBool("mute", false);
  prefs.end();
  Serial.printf("Loaded Config: Speed=%d%%, SteerLeft=%ldms, SteerRight=%ldms, Vol=%d%%, Mute=%d\n", driveSpeedPercent, steerLimitLeftMs, steerLimitRightMs, audioVolumePercent, audioMuted);

  // Web Server Routes
  server.on("/", sendHtmlWithNoCache);
  server.on("/wiring", sendHtmlWithNoCache);
  server.on("/mrl", sendHtmlWithNoCache);

  server.on("/status", []() {
    lastHeartbeatMs = millis();
    String json = "{";
    json += "\"pin4\":\"" + String(currentDriveState == DRIVE_FORWARD ? "ON" : "OFF") + "\",";
    json += "\"pin5\":\"" + String(currentDriveState == DRIVE_BACK ? "ON" : "OFF") + "\",";
    json += "\"pin6\":\"" + String(currentSteerState == STEER_LEFT ? "ON" : "OFF") + "\",";
    json += "\"pin7\":\"" + String(currentSteerState == STEER_RIGHT ? "ON" : "OFF") + "\",";
    json += "\"pos\":" + String(currentPositionMs);
    json += "}";
    server.send(200, "application/json", json);
  });

  server.on("/set_speed", []() {
    if (server.hasArg("val")) {
      driveSpeedPercent = constrain(server.arg("val").toInt(), 20, 100);
      prefs.begin("tink_cfg", false);
      prefs.putInt("speed", driveSpeedPercent);
      prefs.end();
      Serial.printf("Drive Speed Updated: %d%%\n", driveSpeedPercent);
    }
    server.send(200, "application/json", "{\"speed\":" + String(driveSpeedPercent) + "}");
  });

  server.on("/set_volume", []() {
    if (server.hasArg("val")) {
      audioVolumePercent = constrain(server.arg("val").toInt(), 0, 100);
      if (audioVolumePercent > 0) audioMuted = false;
      prefs.begin("tink_cfg", false);
      prefs.putInt("vol", audioVolumePercent);
      prefs.putBool("mute", audioMuted);
      prefs.end();
      Serial.printf("Audio Volume Updated: %d%% (Mute=%d)\n", audioVolumePercent, audioMuted);
    }
    server.send(200, "application/json", "{\"vol\":" + String(audioVolumePercent) + ",\"muted\":" + String(audioMuted ? "true" : "false") + "}");
  });

  server.on("/toggle_mute", []() {
    audioMuted = !audioMuted;
    prefs.begin("tink_cfg", false);
    prefs.putBool("mute", audioMuted);
    prefs.end();
    Serial.printf("Audio Mute Toggled: %d\n", audioMuted);
    server.send(200, "application/json", "{\"vol\":" + String(audioVolumePercent) + ",\"muted\":" + String(audioMuted ? "true" : "false") + "}");
  });

  server.on("/set_steer_limits", []() {
    if (server.hasArg("left")) {
      steerLimitLeftMs = constrain(server.arg("left").toInt(), 100, 3000);
    }
    if (server.hasArg("right")) {
      steerLimitRightMs = constrain(server.arg("right").toInt(), 100, 3000);
    }
    prefs.begin("tink_cfg", false);
    prefs.putLong("s_left", steerLimitLeftMs);
    prefs.putLong("s_right", steerLimitRightMs);
    prefs.end();
    server.send(200, "application/json", "{\"left\":" + String(steerLimitLeftMs) + ",\"right\":" + String(steerLimitRightMs) + "}");
  });

  server.on("/set_left_limit", []() {
    steerLimitLeftMs = max(100L, abs(currentPositionMs));
    prefs.begin("tink_cfg", false);
    prefs.putLong("s_left", steerLimitLeftMs);
    prefs.end();
    triggerSetCenterFlash();
    server.send(200, "application/json", "{\"left\":" + String(steerLimitLeftMs) + "}");
  });

  server.on("/set_right_limit", []() {
    steerLimitRightMs = max(100L, abs(currentPositionMs));
    prefs.begin("tink_cfg", false);
    prefs.putLong("s_right", steerLimitRightMs);
    prefs.end();
    triggerSetCenterFlash();
    server.send(200, "application/json", "{\"right\":" + String(steerLimitRightMs) + "}");
  });

  server.on("/get_config", []() {
    String json = "{";
    json += "\"speed\":" + String(driveSpeedPercent) + ",";
    json += "\"steer_left\":" + String(steerLimitLeftMs) + ",";
    json += "\"steer_right\":" + String(steerLimitRightMs) + ",";
    json += "\"vol\":" + String(audioVolumePercent) + ",";
    json += "\"mute\":" + String(audioMuted ? "true" : "false") + ",";
    json += "\"pos\":" + String(currentPositionMs);
    json += "}";
    server.send(200, "application/json", json);
  });

  server.on("/scan_ble", []() {
    startBleGamepadScan();
    server.send(200, "text/plain", "BLE Scan Started for Wireless Controllers!");
  });

  server.on("/ble_status", []() {
    lastHeartbeatMs = millis();
    String json = "{";
    json += "\"scanning\":" + String(isBleScanning ? "true" : "false") + ",";
    json += "\"status\":\"" + bleStatusMsg + "\",";
    json += "\"connected\":\"" + connectedGamepadName + "\"";
    json += "}";
    server.send(200, "application/json", json);
  });

  server.on("/forget_gamepad", []() {
    prefs.begin("tink_gp", false);
    prefs.clear();
    prefs.end();
    savedGamepadMac = "";
    connectedGamepadName = "";
    if (pGamepadClient != nullptr && pGamepadClient->isConnected()) {
      pGamepadClient->disconnect();
    }
    bleStatusMsg = "Controller forgotten. Ready to pair a new controller.";
    server.send(200, "text/plain", "Controller Forgotten");
  });

  server.on("/stop", []() { processCommand("stop"); server.send(200, "text/plain", "Stopped"); });
  server.on("/stop_drive", []() { processCommand("stop_drive"); server.send(200, "text/plain", "Stopped Drive"); });
  server.on("/stop_steer", []() { processCommand("stop_steer"); server.send(200, "text/plain", "Stopped Steer"); });
  server.on("/forward", []() { processCommand("forward"); server.send(200, "text/plain", "Forward"); });
  server.on("/back", []() { processCommand("back"); server.send(200, "text/plain", "Back"); });
  server.on("/left", []() { processCommand("left"); server.send(200, "text/plain", "Left"); });
  server.on("/right", []() { processCommand("right"); server.send(200, "text/plain", "Right"); });
  server.on("/center", []() { processCommand("center"); server.send(200, "text/plain", "Centering"); });
  server.on("/set_center", []() { processCommand("set_center"); server.send(200, "text/plain", "Center Set"); });

  // Captive Portal fallback route for iOS / Android auto-popup
  server.onNotFound(sendHtmlWithNoCache);

  server.begin();
  Serial.println("HTTP server started");
  lastHeartbeatMs = millis();

  // 6. Load Saved Controller MAC from Flash
  prefs.begin("tink_gp", true);
  savedGamepadMac = prefs.getString("mac", "");
  String savedName = prefs.getString("name", "");
  prefs.end();

  if (savedGamepadMac.length() > 0) {
    Serial.println("Found Saved Gamepad in Flash Memory: " + savedName + " [" + savedGamepadMac + "]");
    bleStatusMsg = "Auto-searching for saved controller: " + (savedName.length() > 0 ? savedName : savedGamepadMac);
  }
}

void loop() {
  dnsServer.processNextRequest();
  server.handleClient();

  // Safe Asynchronous Gamepad Pairing
  handleGamepadConnectionLoop();

  // Dynamic Relay Drive Speed Governor (Time-sliced Soft PWM)
  if (currentDriveState == DRIVE_FORWARD || currentDriveState == DRIVE_BACK) {
    if (driveSpeedPercent >= 100) {
      if (currentDriveState == DRIVE_FORWARD) setRelayOn(pinForward);
      else setRelayOn(pinBack);
    } else {
      unsigned long cycleMs = millis() % 120; // 120ms smooth time slice cycle
      unsigned long onThresholdMs = (120 * driveSpeedPercent) / 100;
      if (cycleMs < onThresholdMs) {
        if (currentDriveState == DRIVE_FORWARD) setRelayOn(pinForward);
        else setRelayOn(pinBack);
      } else {
        if (currentDriveState == DRIVE_FORWARD) setRelayOff(pinForward);
        else setRelayOff(pinBack);
      }
    }
  }

  // Real-time Racing Simulator Dynamic Soundscape (Idle, Shifting, Drifting)
  updateSimulatorAudio();

  // Handle Hardware USB Serial input (MyRobotLab Serial Service)
  if (Serial.available()) {
    String serialCmd = Serial.readStringUntil('\n');
    lastHeartbeatMs = millis();
    processCommand(serialCmd);
  }

  // Failsafe Check: If driving or steering, but heartbeat times out (>3 sec) or device disconnects
  if (currentDriveState != DRIVE_STOP || currentSteerState != STEER_IDLE) {
    if (!isDeviceConnected() || (millis() - lastHeartbeatMs > FAILSAFE_TIMEOUT_MS)) {
      Serial.println("FAILSAFE TRIGGERED: Connection lost! Stopping all motors!");
      stopAll();
    }
  }
  
  // Dynamic RGB LED & Audio Status (or Idle Rainbow)
  updateLedStatus();
  
  // Steering Limits & Centering Logic
  if (isCentering) {
     if (currentPositionMs > 40) {
         if (currentSteerState != STEER_LEFT) {
             stopSteering(); delay(30);
             currentSteerState = STEER_LEFT;
             lastSteerUpdateMs = millis();
             setRelayOn(pinLeft);
         } else {
             updateSteeringPosition();
         }
     } else if (currentPositionMs < -40) {
         if (currentSteerState != STEER_RIGHT) {
             stopSteering(); delay(30);
             currentSteerState = STEER_RIGHT;
             lastSteerUpdateMs = millis();
             setRelayOn(pinRight);
         } else {
             updateSteeringPosition();
         }
     } else {
         stopSteering();
         isCentering = false;
         currentPositionMs = 0;
         playSuccessChime();
     }
  } else {
      if (currentSteerState == STEER_LEFT) {
        updateSteeringPosition();
        if (currentPositionMs <= -steerLimitLeftMs) {
          stopSteering(); 
        }
      } 
      else if (currentSteerState == STEER_RIGHT) {
        updateSteeringPosition();
        if (currentPositionMs >= steerLimitRightMs) {
          stopSteering(); 
        }
      }
  }
}
