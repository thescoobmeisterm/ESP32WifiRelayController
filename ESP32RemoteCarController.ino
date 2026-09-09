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

// Dual HC-SR04 Ultrasonic Collision Avoidance Pins
const int pinTrigFront = 8;   // Front Sonar Trigger (3.3V out to HC-SR04)
const int pinEchoFront = 9;   // Front Sonar Echo (5V in from HC-SR04 via resistor/divider)
const int pinTrigBack  = 10;  // Back Sonar Trigger (3.3V out to HC-SR04)
const int pinEchoBack  = 11;  // Back Sonar Echo (5V in from HC-SR04 via resistor/divider)

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
// --- Motor Hardware Driver Mode ---
enum MotorDriverType { DRIVER_RELAY = 0, DRIVER_L298N = 1 };
MotorDriverType driveMotorDriver = DRIVER_RELAY; // Default: Relay Module (or L298N)
MotorDriverType steerMotorDriver = DRIVER_RELAY; // Default: Relay Module (or L298N for variable speed steering)

// --- Configurable Drive Speed, Steer Speed & Calibration Limits ---
int driveSpeedPercent = 60;          // Default drive throttle: 60% (Range: 20% to 100%)
int steerSpeedPercent = 75;          // Default steer speed: 75% (Range: 25% to 100%)
long steerLimitLeftMs = 2500;        // Max Left steering lock travel in ms (Range: 100ms - 15000ms)
long steerLimitRightMs = 2500;       // Max Right steering lock travel in ms (Range: 100ms - 15000ms)
bool steerLimitsEnabled = true;      // If false, unrestricted continuous steering is enabled
unsigned long lastSteerUpdateMs = 0;
bool isCentering = false;
bool isAutoHoming = false;
unsigned long autoHomeStartMs = 0;
long autoHomeDurationMs = 3500;

// --- HC-SR04 Dual Ultrasonic Collision Avoidance Variables ---
bool collisionAvoidanceEnabled = true; // Master toggle (Persistent in Flash NVS)
int collisionThresholdCm = 30;         // Sensitivity distance in cm (10cm - 100cm, default 30cm)
int frontDistanceCm = -1;              // -1 = Clear / out of range
int backDistanceCm  = -1;              // -1 = Clear / out of range
bool frontBlocked = false;
bool backBlocked  = false;
unsigned long lastSonarPollMs = 0;
uint8_t sonarPollCycle = 0;            // 0 = Ping Front, 1 = Ping Back (Interleaved)
unsigned long lastCollisionAlertToneMs = 0;

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

// --- Configurable Acceleration Ramp Times (ms) for L298N ---
int driveRampMs = 400;               // Time in ms to reach target drive speed (0 = Instant, 0 - 3000ms)
int steerRampMs = 250;               // Time in ms to reach target steer speed (0 = Instant, 0 - 2000ms)
unsigned long driveRampStartMs = 0;  // Timestamp when current drive motion started
unsigned long steerRampStartMs = 0;  // Timestamp when current steer motion started
int currentDrivePwm = 0;             // Active hardware PWM output (0 - 255)
int currentSteerPwm = 0;             // Active hardware PWM output (0 - 255)

// Function declarations
void stopDriving();
void stopSteering();
void startSteeringLeft();
void startSteeringRight();
void startAutoHoming();
void handleAutoHomingLoop();
void updateMotorRamping();
void stopAll();
void processCommand(String cmd);
void triggerSetCenterFlash();
void playBootChime();
void playSuccessChime();
void playStopBeep();
void playCollisionAlertTone();
void updateSonarCollisionCheck();
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

// --- Dynamic Racing Sound Synthesizer State Variables ---
unsigned long lastAudioSliceMs = 0;
double simEnginePhase = 0;
double simDriftPhase = 0;
unsigned long forwardCycleStartMs = 0;
bool wasConnectedAudio = false;

enum SoundTheme {
  THEME_SIMULATOR = 0, // V8 Muscle Simulator
  THEME_MARIO_KART = 1, // Classic Mario Kart (3-2-1-GO!, 150cc Kart, Hop Drift, Coin)
  THEME_F_ZERO = 2,    // Sci-Fi F-Zero (Plasma Warp, Hover Turbine, Laser Drift, Boost)
  THEME_CRUISN_USA = 3  // 90s Arcade Rocker (Power Chords, Twin Turbo, Screech, Checkpoint)
};
SoundTheme currentSoundTheme = THEME_SIMULATOR;

// Plays the signature Startup Fanfare for the active theme
void playThemeStartFanfare() {
  if (!isI2SReady) return;
  float vol = getVolumeFactor();
  if (vol <= 0.001f) return;

  if (currentSoundTheme == THEME_MARIO_KART) {
    // Mario Kart 3-2-1-GO! Countdown Fanfare
    playI2STone(440, 90); delay(110);
    playI2STone(440, 90); delay(110);
    playI2STone(440, 90); delay(110);
    playI2STone(880, 260); delay(50);
  }
  else if (currentSoundTheme == THEME_F_ZERO) {
    // F-Zero Anti-Grav Reactor Plasma Powerup Sweep
    int sampleRate = 44100;
    int numSamples = (sampleRate * 400) / 1000;
    int16_t samples[128];
    double phase = 0;
    size_t bytesWritten;
    for (int i = 0; i < numSamples; i += 64) {
      int count = min(64, numSamples - i);
      double prog = (double)i / (double)numSamples;
      double freq = 140.0 * pow(12.0, prog); // 140Hz -> 1680Hz sweep
      double phaseInc = (2.0 * M_PI * freq) / sampleRate;
      for (int j = 0; j < count; j++) {
        int32_t raw = (int32_t)(((sin(phase) * 22000.0) + (sin(phase * 2.0) * 8000.0)) * vol * (0.3 + 0.7 * prog));
        int16_t sampleVal = (int16_t)constrain(raw, -32767, 32767);
        samples[j * 2] = sampleVal;
        samples[j * 2 + 1] = sampleVal;
        phase += phaseInc;
        if (phase >= 2.0 * M_PI) phase -= 2.0 * M_PI;
      }
      i2s_write(I2S_NUM, samples, count * 4, &bytesWritten, pdMS_TO_TICKS(15));
    }
  }
  else if (currentSoundTheme == THEME_CRUISN_USA) {
    // 90s Arcade Rock Power Chord Stabs
    playI2STone(165, 80); delay(30);
    playI2STone(220, 80); delay(30);
    playI2STone(330, 80); delay(30);
    playI2STone(440, 200); delay(50);
  }
  else {
    // Default V8 Starter + Rev
    playI2STone(115, 80); delay(35);
    playI2STone(110, 80); delay(35);
    playI2STone(105, 90); delay(30);
    int sampleRate = 44100;
    int numSamples = (sampleRate * 450) / 1000;
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
}

void playIgnitionSound() {
  playThemeStartFanfare();
}

void updateSimulatorAudio() {
  if (!isI2SReady) return;
  float vol = getVolumeFactor();

  // Check for device connection event to play theme startup fanfare
  bool nowConnected = isDeviceConnected();
  if (nowConnected && !wasConnectedAudio) {
    wasConnectedAudio = true;
    playThemeStartFanfare();
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

  // 1. STEERING / DRIFT SOUND BY THEME
  if (currentSteerState == STEER_LEFT || currentSteerState == STEER_RIGHT) {
    if (currentSoundTheme == THEME_MARIO_KART) {
      // Mario Kart Hop & Drift Sparkle Chirp (Alternating high frequency)
      double driftFreq = 1600.0 + (sin((double)now * 0.08) * 600.0);
      double phaseInc = (2.0 * M_PI * driftFreq) / sampleRate;
      for (int i = 0; i < numSamples; i += 64) {
        int count = min(64, numSamples - i);
        for (int j = 0; j < count; j++) {
          int32_t raw = (int32_t)(((sin(simDriftPhase) * 16000.0) + (sin(simDriftPhase * 2.0) * 8000.0)) * vol);
          int16_t sampleVal = (int16_t)constrain(raw, -32767, 32767);
          samples[j * 2] = sampleVal; samples[j * 2 + 1] = sampleVal;
          simDriftPhase += phaseInc;
          if (simDriftPhase >= 2.0 * M_PI) simDriftPhase -= 2.0 * M_PI;
        }
        i2s_write(I2S_NUM, samples, count * 4, &bytesWritten, pdMS_TO_TICKS(10));
      }
      return;
    }
    else if (currentSoundTheme == THEME_F_ZERO) {
      // F-Zero Lateral Magnetic Airbrake Laser Hiss
      for (int i = 0; i < numSamples; i += 64) {
        int count = min(64, numSamples - i);
        double laserFreq = 3400.0 + (sin((double)now * 0.05) * 500.0);
        double phaseInc = (2.0 * M_PI * laserFreq) / sampleRate;
        for (int j = 0; j < count; j++) {
          int16_t noise = (rand() % 5000) - 2500;
          int32_t raw = (int32_t)(((sin(simDriftPhase) * 12000.0) + noise) * vol);
          int16_t sampleVal = (int16_t)constrain(raw, -32767, 32767);
          samples[j * 2] = sampleVal; samples[j * 2 + 1] = sampleVal;
          simDriftPhase += phaseInc;
          if (simDriftPhase >= 2.0 * M_PI) simDriftPhase -= 2.0 * M_PI;
        }
        i2s_write(I2S_NUM, samples, count * 4, &bytesWritten, pdMS_TO_TICKS(10));
      }
      return;
    }
    else {
      // Realistic / Cruis'n Tire Drift Screech
      for (int i = 0; i < numSamples; i += 64) {
        int count = min(64, numSamples - i);
        double squealFreq = 2600.0 + (sin((double)now * 0.04 + (double)i * 0.008) * 350.0) + ((rand() % 220) - 110);
        double phaseInc = (2.0 * M_PI * squealFreq) / sampleRate;
        for (int j = 0; j < count; j++) {
          int16_t noise = (rand() % 4000) - 2000;
          int32_t raw = (int32_t)(((sin(simDriftPhase) * 18000.0) + (sin(simDriftPhase * 1.45) * 9000.0) + noise) * vol);
          int16_t sampleVal = (int16_t)constrain(raw, -32767, 32767);
          samples[j * 2] = sampleVal; samples[j * 2 + 1] = sampleVal;
          simDriftPhase += phaseInc;
          if (simDriftPhase >= 2.0 * M_PI) simDriftPhase -= 2.0 * M_PI;
        }
        i2s_write(I2S_NUM, samples, count * 4, &bytesWritten, pdMS_TO_TICKS(10));
      }
      return;
    }
  }

  // 2. FORWARD DRIVE SOUND BY THEME
  if (currentDriveState == DRIVE_FORWARD) {
    if (currentSoundTheme == THEME_MARIO_KART) {
      // High-revving 2-Stroke 150cc Racing Kart Engine
      unsigned long kartElapsed = (now - forwardCycleStartMs) % 1800;
      double prog = (double)kartElapsed / 1800.0;
      double kartFreq = 180.0 + (prog * 240.0) + (sin((double)now * 0.06) * 15.0);
      double phaseInc = (2.0 * M_PI * kartFreq) / sampleRate;
      for (int i = 0; i < numSamples; i += 64) {
        int count = min(64, numSamples - i);
        for (int j = 0; j < count; j++) {
          int32_t raw = (int32_t)(
            ((sin(simEnginePhase) * 15000.0) +
             (sin(simEnginePhase * 2.0) * 11000.0) +
             (sin(simEnginePhase * 3.0) * 6000.0)) * vol
          );
          int16_t sampleVal = (int16_t)constrain(raw, -32767, 32767);
          samples[j * 2] = sampleVal; samples[j * 2 + 1] = sampleVal;
          simEnginePhase += phaseInc;
          if (simEnginePhase >= 2.0 * M_PI) simEnginePhase -= 2.0 * M_PI;
        }
        i2s_write(I2S_NUM, samples, count * 4, &bytesWritten, pdMS_TO_TICKS(10));
      }
      return;
    }
    else if (currentSoundTheme == THEME_F_ZERO) {
      // Screaming Anti-Gravity Plasma Jet Turbine
      unsigned long fzElapsed = (now - forwardCycleStartMs) % 2000;
      double prog = (double)fzElapsed / 2000.0;
      double turbineFreq = 320.0 + (prog * 750.0);
      double phaseInc = (2.0 * M_PI * turbineFreq) / sampleRate;
      for (int i = 0; i < numSamples; i += 64) {
        int count = min(64, numSamples - i);
        for (int j = 0; j < count; j++) {
          int32_t raw = (int32_t)(
            ((sin(simEnginePhase) * 16000.0) +
             (sin(simEnginePhase * 1.5) * 8000.0) +
             (sin(simEnginePhase * 3.0) * 5000.0)) * vol
          );
          int16_t sampleVal = (int16_t)constrain(raw, -32767, 32767);
          samples[j * 2] = sampleVal; samples[j * 2 + 1] = sampleVal;
          simEnginePhase += phaseInc;
          if (simEnginePhase >= 2.0 * M_PI) simEnginePhase -= 2.0 * M_PI;
        }
        i2s_write(I2S_NUM, samples, count * 4, &bytesWritten, pdMS_TO_TICKS(10));
      }
      return;
    }
    else {
      // Arcade V8 Sim & Cruis'n USA (5-Gear Sequential Climb & Shift)
      unsigned long cycleElapsed = (now - forwardCycleStartMs) % 1500;
      int gearNum = ((now - forwardCycleStartMs) / 1500) % 5;
      double baseGearFreq = (currentSoundTheme == THEME_CRUISN_USA ? 110.0 : 95.0) + (gearNum * 22.0);
      double maxGearFreq  = (currentSoundTheme == THEME_CRUISN_USA ? 360.0 : 310.0) + (gearNum * 28.0);
      double engineFreq;
      if (cycleElapsed < 1320) {
        double prog = (double)cycleElapsed / 1320.0;
        engineFreq = baseGearFreq + (prog * prog * (maxGearFreq - baseGearFreq));
      } else {
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
          samples[j * 2] = sampleVal; samples[j * 2 + 1] = sampleVal;
          simEnginePhase += phaseInc;
          if (simEnginePhase >= 2.0 * M_PI) simEnginePhase -= 2.0 * M_PI;
        }
        i2s_write(I2S_NUM, samples, count * 4, &bytesWritten, pdMS_TO_TICKS(10));
      }
      return;
    }
  }

  // 3. IDLE / NEUTRAL ENGINE RUMBLE BY THEME
  if (isDeviceConnected() && currentDriveState == DRIVE_STOP && !isCentering) {
    double idleFreq = (currentSoundTheme == THEME_MARIO_KART) ? (80.0 + sin((double)now * 0.03) * 6.0) :
                      (currentSoundTheme == THEME_F_ZERO)     ? (55.0 + sin((double)now * 0.015) * 4.0) :
                      (currentSoundTheme == THEME_CRUISN_USA) ? (38.0 + sin((double)now * 0.008) * 3.0) :
                                                                (42.0 + sin((double)now * 0.008) * 3.0);
    double idleMod = 0.75 + (sin((double)now * 0.02) * 0.25);
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
        samples[j * 2] = sampleVal; samples[j * 2 + 1] = sampleVal;
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

const char htmlPage[] PROGMEM = R"HTML(
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
      width: 100%;
      max-width: 360px;
      margin-top: 4px;
      margin-bottom: 6px;
    }
    .header-row {
      display: flex;
      justify-content: space-between;
      align-items: center;
      width: 100%;
    }
    .badge {
      display: inline-block;
      font-family: 'Orbitron', sans-serif;
      font-size: 9px;
      letter-spacing: 2px;
      color: #00f3ff;
      background: rgba(0, 243, 255, 0.1);
      border: 1px solid rgba(0, 243, 255, 0.3);
      padding: 2px 8px;
      border-radius: 4px;
      margin-bottom: 2px;
      text-transform: uppercase;
    }
    h1 {
      font-family: 'Orbitron', sans-serif;
      font-size: 28px;
      font-weight: 900;
      letter-spacing: 3px;
      color: #ffffff;
      margin: 0;
      line-height: 1.1;
      text-shadow: 0 0 15px rgba(0, 243, 255, 0.6);
    }
    .subtitle {
      font-size: 10px;
      letter-spacing: 1.5px;
      color: #94a3b8;
      text-transform: uppercase;
    }
    .btn-gear {
      background: rgba(15, 23, 42, 0.9);
      border: 1.5px solid #00f3ff;
      color: #00f3ff;
      font-family: 'Orbitron', sans-serif;
      font-size: 11px;
      font-weight: 700;
      padding: 8px 12px;
      border-radius: 8px;
      cursor: pointer;
      display: flex;
      align-items: center;
      gap: 6px;
      box-shadow: 0 0 12px rgba(0, 243, 255, 0.35);
      transition: all 0.15s ease;
      text-transform: uppercase;
    }
    .btn-gear:active {
      background: #00f3ff;
      color: #07090e;
      box-shadow: 0 0 22px rgba(0, 243, 255, 0.8);
      transform: scale(0.95);
    }

    /* Cockpit Layout Container */
    .cockpit-container {
      width: 100%;
      max-width: 360px;
      display: flex;
      flex-direction: column;
      align-items: center;
    }

    /* Modal Overlay for Settings */
    .modal-overlay {
      position: fixed;
      top: 0; left: 0; right: 0; bottom: 0;
      background: rgba(7, 9, 14, 0.88);
      backdrop-filter: blur(8px);
      -webkit-backdrop-filter: blur(8px);
      z-index: 999;
      display: none;
      justify-content: center;
      align-items: flex-start;
      padding: 12px 8px;
      overflow-y: auto;
      box-sizing: border-box;
    }
    .modal-overlay.active {
      display: flex;
      animation: modalFadeIn 0.2s ease-out;
    }
    @keyframes modalFadeIn {
      from { opacity: 0; transform: scale(0.97); }
      to { opacity: 1; transform: scale(1); }
    }
    .modal-content {
      background: #0f172a;
      border: 1px solid #38bdf8;
      border-radius: 14px;
      width: 100%;
      max-width: 380px;
      padding: 14px;
      box-shadow: 0 10px 30px rgba(0, 243, 255, 0.25);
      box-sizing: border-box;
      margin: auto 0;
    }
    .modal-header {
      display: flex;
      justify-content: space-between;
      align-items: center;
      border-bottom: 1px solid rgba(0, 243, 255, 0.25);
      padding-bottom: 10px;
      margin-bottom: 10px;
    }
    .modal-title {
      font-family: 'Orbitron', sans-serif;
      font-size: 13px;
      font-weight: 900;
      color: #00f3ff;
      letter-spacing: 1px;
    }
    .modal-close-btn {
      background: rgba(239, 68, 68, 0.15);
      border: 1px solid #ef4444;
      color: #ef4444;
      font-family: 'Orbitron', sans-serif;
      font-size: 11px;
      font-weight: 700;
      padding: 5px 10px;
      border-radius: 6px;
      cursor: pointer;
      transition: all 0.15s ease;
    }
    .modal-close-btn:active {
      background: #ef4444;
      color: #ffffff;
    }
    .settings-tab-bar {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(76px, 1fr));
      gap: 4px;
      margin-bottom: 12px;
    }
    .settings-tab-btn {
      font-family: 'Orbitron', sans-serif;
      font-size: 9px;
      font-weight: 700;
      padding: 7px 2px;
      color: #94a3b8;
      background: rgba(30, 41, 59, 0.8);
      border: 1px solid #334155;
      border-radius: 6px;
      cursor: pointer;
      text-transform: uppercase;
      transition: all 0.15s ease;
      text-align: center;
    }
    .settings-tab-btn.active {
      color: #07090e;
      background: #00f3ff;
      border-color: #00f3ff;
      box-shadow: 0 0 10px rgba(0, 243, 255, 0.5);
    }
    .settings-tab-pane {
      display: none;
      width: 100%;
    }
    .settings-tab-pane.active {
      display: block;
    }
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

  <!-- Header with Gear Settings Button -->
  <div class="hud-header">
    <div class="header-row">
      <div style="text-align:left;">
        <div class="badge">IGNITE ROBOTICS</div>
        <h1>TINK</h1>
        <div class="subtitle">Base Cockpit</div>
      </div>
      <button class="btn-gear" onclick="openSettingsModal()">
        ⚙️ SETTINGS
      </button>
    </div>
  </div>

  <!-- MAIN COCKPIT: Clean, Streamlined Driving Page -->
  <div class="cockpit-container">
    <!-- Steering Section -->
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

    <!-- Steering Centering Quick Actions -->
    <div class="utility-bar">
      <button class="btn-util" onclick="sendCommand('center')">🎯 Recenter Steering (C)</button>
      <button class="btn-util" onclick="sendCommand('set_center')">📍 Set Center (Z)</button>
    </div>

    <!-- Obstacle Hazard Warning Banner (Active Collision Alert) -->
    <div id="obstacleBanner" style="display:none; width:100%; margin-top:8px; padding:8px 10px; background:rgba(239,68,68,0.25); border:1px solid #ef4444; border-radius:8px; box-shadow:0 0 15px rgba(239,68,68,0.5); text-align:center; box-sizing:border-box;">
      <div id="obstacleBannerTitle" style="font-family:'Orbitron',sans-serif; font-size:11px; font-weight:900; color:#ef4444; letter-spacing:1px;">⚠️ COLLISION HAZARD</div>
      <div id="obstacleBannerDesc" style="font-size:10px; color:#fca5a5; margin-top:2px;">Forward drive inhibited</div>
    </div>

    <!-- Drive Throttle Controls -->
    <div class="throttle-section" style="margin-top:10px;">
      <button class="btn-throttle btn-go" id="btnGo" onclick="setDrive('forward')">
        &#9650; GO / FORWARD (W)
      </button>

      <button class="btn-throttle btn-stop" id="btnStop" onclick="triggerStop()">
        🛑 STOP / EMERGENCY (SPACE/X)
      </button>

      <button class="btn-throttle btn-back" id="btnBack" onclick="setDrive('back')">
        &#9660; BACK / REVERSE (S)
      </button>
    </div>

    <!-- 1. DRIVE SPEED GOVERNOR -->
    <div class="panel-card">
      <div class="panel-title">
        <span>⚡ DRIVE SPEED</span>
        <span id="speedLabel" style="color:#00ff88; font-size:12px; font-weight:900;">60% SPEED</span>
      </div>
      <input type="range" id="speedSlider" class="control-slider" min="20" max="100" value="60" oninput="onSpeedChange(this.value)">
      <div style="display:flex; gap:6px; margin-top:4px;">
        <button class="preset-btn" id="btnSpdSlow" onclick="setSpeedPreset(35)">🐢 SLOW (35%)</button>
        <button class="preset-btn active" id="btnSpdMed" onclick="setSpeedPreset(65)">🚗 CRUISE (65%)</button>
        <button class="preset-btn" id="btnSpdFast" onclick="setSpeedPreset(100)">⚡ TURBO (100%)</button>
      </div>
    </div>

    <!-- 2. AUDIO VOLUME GOVERNOR -->
    <div class="panel-card">
      <div class="panel-title">
        <span>🔊 AUDIO VOLUME</span>
        <span id="volLabel" style="color:#22c55e; font-size:12px; font-weight:900;">100% VOLUME</span>
      </div>
      <input type="range" id="volSlider" class="control-slider" min="0" max="100" value="100" oninput="onVolumeChange(this.value)">
      <div style="display:flex; gap:6px; margin-top:4px;">
        <button class="preset-btn" id="btnMute" onclick="toggleMuteAudio()" style="color:#ef4444; border-color:#ef4444;">🔇 MUTE</button>
        <button class="preset-btn" onclick="setVolumePreset(50)">🔉 50%</button>
        <button class="preset-btn active" id="btnVol100" onclick="setVolumePreset(100)">🔊 100% BOOST</button>
      </div>
    </div>

    <!-- Active Hardware Badges -->
    <div style="display:flex; justify-content:space-between; width:100%; margin-top:10px; font-size:9px; font-family:'Orbitron',sans-serif; color:#94a3b8; gap:4px;">
      <span id="mainThemeBadge" onclick="openSettingsTab('paneThemes')" style="flex:1; text-align:center; color:#00f3ff; background:rgba(0,243,255,0.1); padding:3px 4px; border-radius:4px; border:1px solid rgba(0,243,255,0.3); cursor:pointer;">🏎️ V8 SIM</span>
      <span id="mainDriverBadge" onclick="openSettingsTab('paneDriver')" style="flex:1; text-align:center; color:#f59e0b; background:rgba(245,158,11,0.1); padding:3px 4px; border-radius:4px; border:1px solid rgba(245,158,11,0.3); cursor:pointer;">🎛️ RELAYS</span>
      <span id="mainSensorsBadge" onclick="openSettingsTab('paneSensors')" style="flex:1; text-align:center; color:#22c55e; background:rgba(34,197,94,0.1); padding:3px 4px; border-radius:4px; border:1px solid rgba(34,197,94,0.3); cursor:pointer;">🛡️ SENSORS: 30cm</span>
      <span id="mainLimitsBadge" onclick="openSettingsTab('paneSteering')" style="flex:1; text-align:center; color:#38bdf8; background:rgba(56,189,248,0.1); padding:3px 4px; border-radius:4px; border:1px solid rgba(56,189,248,0.3); cursor:pointer;">LIMITS: ACTIVE</span>
    </div>

    <!-- Telemetry Status -->
    <div class="telemetry" style="margin-top:10px;">
      <span class="status-dot"></span>
      <span id="cockpitPairStatus">SYSTEM ONLINE // WIRELESS AUTO-CONNECT ACTIVE</span>
    </div>
  </div>

  <!-- ⚙️ SETTINGS MODAL OVERLAY -->
  <div class="modal-overlay" id="settingsModal">
    <div class="modal-content">
      <div class="modal-header">
        <div class="modal-title">⚙️ VEHICLE SETTINGS</div>
        <button class="modal-close-btn" onclick="closeSettingsModal()">✖ CLOSE</button>
      </div>

      <!-- Settings Sub-Tabs -->
      <div class="settings-tab-bar">
        <button class="settings-tab-btn active" id="stabSteer" onclick="switchSettingsPane('paneSteering', this)">📐 Steering</button>
        <button class="settings-tab-btn" id="stabDriver" onclick="switchSettingsPane('paneDriver', this)">⚙️ Driver</button>
        <button class="settings-tab-btn" id="stabThemes" onclick="switchSettingsPane('paneThemes', this)">🎮 Themes</button>
        <button class="settings-tab-btn" id="stabSensors" onclick="switchSettingsPane('paneSensors', this)">🛡️ Sensors</button>
        <button class="settings-tab-btn" id="stabGamepad" onclick="switchSettingsPane('paneGamepad', this)">📡 Gamepad</button>
        <button class="settings-tab-btn" id="stabDiag" onclick="switchSettingsPane('paneDiag', this)">⚡ Multimeter</button>
        <button class="settings-tab-btn" id="stabWiring" onclick="switchSettingsPane('paneWiring', this)">🔌 Wiring</button>
      </div>

      <!-- PANE 1: STEERING LIMITS, SPEED & CALIBRATION -->
      <div class="settings-tab-pane active" id="paneSteering">
        <div class="panel-card" style="margin-top:0;">
          <div class="panel-title">
            <span>📐 STEERING LIMITS & CALIBRATION</span>
            <span id="posLabel" style="color:#38bdf8; font-size:11px;">POS: 0 ms</span>
          </div>
          
          <div style="display:flex; justify-content:space-between; font-size:9px; color:#64748b; font-family:monospace;">
            <span id="leftLimitDisplay">LEFT: -2500ms</span>
            <span id="limitsModeLabel" style="color:#22c55e; font-weight:bold;">LIMITS: ACTIVE</span>
            <span id="rightLimitDisplay">RIGHT: +2500ms</span>
          </div>
          <div class="pos-meter-track">
            <div class="pos-meter-marker" id="posMarker" style="left:50%;"></div>
          </div>

          <!-- Variable Steering Speed Governor (L298N PWM Scaling) -->
          <div style="margin-top:10px; padding-top:8px; border-top:1px solid #1e293b;">
            <div style="display:flex; justify-content:space-between; font-size:9px; color:#94a3b8; font-weight:bold;">
              <span>⚡ TURNING SPEED (L298N PWM)</span>
              <span id="steerSpeedLabel" style="color:#00ff88; font-weight:900;">75% SPEED</span>
            </div>
            <input type="range" id="steerSpeedSlider" class="control-slider" min="25" max="100" value="75" oninput="onSteerSpeedChange(this.value)">
            <div style="display:flex; gap:4px; margin-top:4px;">
              <button class="preset-btn" id="btnStrSpdSlow" onclick="setSteerSpeedPreset(35)">🐢 35% Precision</button>
              <button class="preset-btn active" id="btnStrSpdMed" onclick="setSteerSpeedPreset(65)">🚗 65% Normal</button>
              <button class="preset-btn" id="btnStrSpdFast" onclick="setSteerSpeedPreset(100)">⚡ 100% Quick</button>
            </div>
          </div>

          <div style="display:grid; grid-template-columns:1fr 1fr; gap:8px; margin-top:8px;">
            <div>
              <div style="display:flex; justify-content:space-between; font-size:9px; color:#94a3b8; font-weight:bold;">
                <span>LEFT MAX LOCK</span>
                <span id="leftValLabel">2500ms</span>
              </div>
              <input type="range" id="leftLimitSlider" class="control-slider" min="100" max="10000" step="50" value="2500" oninput="onSteerLimitsChange()">
            </div>
            <div>
              <div style="display:flex; justify-content:space-between; font-size:9px; color:#94a3b8; font-weight:bold;">
                <span>RIGHT MAX LOCK</span>
                <span id="rightValLabel">2500ms</span>
              </div>
              <input type="range" id="rightLimitSlider" class="control-slider" min="100" max="10000" step="50" value="2500" oninput="onSteerLimitsChange()">
            </div>
          </div>

          <div style="display:flex; gap:4px; margin-top:6px;">
            <button class="preset-btn" onclick="setSteerPreset(1000)">1.0s</button>
            <button class="preset-btn active" id="btnSteer25" onclick="setSteerPreset(2500)">2.5s</button>
            <button class="preset-btn" id="btnSteer50" onclick="setSteerPreset(5000)">5.0s</button>
            <button class="preset-btn" id="btnSteer100" onclick="setSteerPreset(10000)">10.0s</button>
            <button class="preset-btn" onclick="adjustSteerLimits(1000)" style="color:#00f3ff;">+1.0s</button>
          </div>

          <div style="display:flex; gap:6px; margin-top:8px;">
            <button class="preset-btn" onclick="setLeftLimitNow()" style="color:#fcd34d; border-color:#f59e0b;">👈 LOCK LEFT</button>
            <button class="preset-btn" onclick="sendCommand('set_center')" style="color:#00f3ff; border-color:#00f3ff;">🎯 ZERO CENTER</button>
            <button class="preset-btn" onclick="setRightLimitNow()" style="color:#60a5fa; border-color:#3b82f6;">👉 LOCK RIGHT</button>
            <button class="preset-btn" id="btnFreeSteer" onclick="toggleFreeSteer()" style="color:#ec4899; border-color:#ec4899;">🔓 FREE STEER</button>
          </div>

          <!-- 1-Tap Auto-Homing / Calibration -->
          <div style="margin-top:8px;">
            <button class="preset-btn" id="btnAutoHome" onclick="triggerAutoHome()" style="width:100%; padding:9px 0; font-size:10px; color:#ec4899; border-color:#ec4899;">
              🏠 AUTO-CALIBRATE TO HARD LIMIT (Zero Drift & Recenter)
            </button>
          </div>
        </div>
      </div>

      <!-- PANE 2: MOTOR HARDWARE DRIVERS -->
      <div class="settings-tab-pane" id="paneDriver">
        <!-- Drive Motor Driver Card -->
        <div class="panel-card" style="margin-top:0;">
          <div class="panel-title">
            <span>⚙️ DRIVE MOTOR DRIVER</span>
            <span id="driverLabel" style="color:#f59e0b; font-size:11px; font-weight:700;">🎛️ 4-CH RELAYS</span>
          </div>
          <div style="display:grid; grid-template-columns:1fr 1fr; gap:6px; margin-top:6px;">
            <button class="preset-btn active" id="btnDriver0" onclick="setMotorDriver(0)">🎛️ 4-CH RELAYS</button>
            <button class="preset-btn" id="btnDriver1" onclick="setMotorDriver(1)">⚡ L298N H-BRIDGE</button>
          </div>
          <div style="font-size:10px; color:#94a3b8; margin-top:6px; font-family:sans-serif;" id="driverDesc">
            Mode: Relays (Time-Sliced Soft PWM on GPIO 4/5)
          </div>

          <!-- Drive Acceleration Ramp Slider (L298N) -->
          <div style="margin-top:10px; border-top:1px solid #334155; padding-top:8px;" id="driveRampGroup">
            <div style="display:flex; justify-content:space-between; font-size:10px; color:#94a3b8; font-weight:bold;">
              <span>🚀 DRIVE ACCELERATION RAMP</span>
              <span id="driveRampLabel" style="color:#00f3ff;">0.40s (Smooth)</span>
            </div>
            <input type="range" id="driveRampSlider" class="control-slider" min="0" max="3000" step="50" value="400" oninput="onDriveRampChange(this.value)">
            <div style="display:flex; gap:4px; margin-top:6px;">
              <button class="preset-btn" id="btnDRamp0" onclick="setDriveRampPreset(0)">⚡ 0s Instant</button>
              <button class="preset-btn" id="btnDRamp300" onclick="setDriveRampPreset(300)">🏎️ 0.3s Sport</button>
              <button class="preset-btn active" id="btnDRamp600" onclick="setDriveRampPreset(600)">🚗 0.6s Smooth</button>
              <button class="preset-btn" id="btnDRamp1200" onclick="setDriveRampPreset(1200)">🐢 1.2s Soft</button>
            </div>
            <div style="font-size:9px; color:#64748b; margin-top:4px;">
              Controls how smoothly the drive motor accelerates from rest to set speed. Stopping remains instant.
            </div>
          </div>
        </div>

        <!-- Steering Motor Driver Card -->
        <div class="panel-card" style="margin-top:10px;">
          <div class="panel-title">
            <span>⚙️ STEERING ACTUATOR DRIVER</span>
            <span id="steerDriverLabel" style="color:#f59e0b; font-size:11px; font-weight:700;">🎛️ 4-CH RELAYS</span>
          </div>
          <div style="display:grid; grid-template-columns:1fr 1fr; gap:6px; margin-top:6px;">
            <button class="preset-btn active" id="btnSteerDriver0" onclick="setSteerMotorDriver(0)">🎛️ 4-CH RELAYS</button>
            <button class="preset-btn" id="btnSteerDriver1" onclick="setSteerMotorDriver(1)">⚡ L298N H-BRIDGE</button>
          </div>
          <div style="font-size:10px; color:#94a3b8; margin-top:6px; font-family:sans-serif;" id="steerDriverDesc">
            Mode: Relays (Digital ON/OFF on GPIO 6/7)
          </div>

          <!-- Steering Acceleration Ramp Slider (L298N) -->
          <div style="margin-top:10px; border-top:1px solid #334155; padding-top:8px;" id="steerRampGroup">
            <div style="display:flex; justify-content:space-between; font-size:10px; color:#94a3b8; font-weight:bold;">
              <span>📐 STEERING RAMP TIME</span>
              <span id="steerRampLabel" style="color:#00f3ff;">0.25s (Quick)</span>
            </div>
            <input type="range" id="steerRampSlider" class="control-slider" min="0" max="2000" step="50" value="250" oninput="onSteerRampChange(this.value)">
            <div style="display:flex; gap:4px; margin-top:6px;">
              <button class="preset-btn" id="btnSRamp0" onclick="setSteerRampPreset(0)">⚡ 0s Instant</button>
              <button class="preset-btn active" id="btnSRamp200" onclick="setSteerRampPreset(200)">🎯 0.2s Quick</button>
              <button class="preset-btn" id="btnSRamp500" onclick="setSteerRampPreset(500)">📐 0.5s Smooth</button>
              <button class="preset-btn" id="btnSRamp1000" onclick="setSteerRampPreset(1000)">🛡️ 1.0s Soft</button>
            </div>
            <div style="font-size:9px; color:#64748b; margin-top:4px;">
              Controls how smoothly steering ramps up. Reduces gear strain and linkage shock.
            </div>
          </div>
        </div>
      </div>

      <!-- PANE 3: SOUND THEMES -->
      <div class="settings-tab-pane" id="paneThemes">
        <div class="panel-card" style="margin-top:0;">
          <div class="panel-title">
            <span>🎮 RACING SOUND THEMES</span>
            <span id="themeLabel" style="color:#00f3ff; font-size:11px; font-weight:700;">🏎️ V8 SIMULATOR</span>
          </div>
          <div style="display:grid; grid-template-columns:1fr 1fr; gap:6px; margin-top:6px;">
            <button class="preset-btn active" id="btnTheme0" onclick="setSoundTheme(0)">🏎️ V8 SIM</button>
            <button class="preset-btn" id="btnTheme1" onclick="setSoundTheme(1)">🍄 MARIO KART</button>
            <button class="preset-btn" id="btnTheme2" onclick="setSoundTheme(2)">🛸 F-ZERO</button>
            <button class="preset-btn" id="btnTheme3" onclick="setSoundTheme(3)">🌴 CRUIS'N USA</button>
          </div>
          <div style="font-size:10px; color:#64748b; margin-top:6px;">
            Select a theme to play its signature startup fanfare and dynamic engine harmonics!
          </div>
        </div>
      </div>

      <!-- PANE 4: DUAL HC-SR04 COLLISION AVOIDANCE SENSORS -->
      <div class="settings-tab-pane" id="paneSensors">
        <div class="panel-card" style="margin-top:0;">
          <div class="panel-title">
            <span>🛡️ COLLISION AVOIDANCE SENSORS</span>
            <span id="sonarStatusLabel" style="color:#22c55e; font-size:11px; font-weight:700;">ACTIVE</span>
          </div>

          <!-- Master Toggle Button -->
          <div style="margin-top:6px;">
            <button class="preset-btn active" id="btnSonarToggle" onclick="toggleCollisionAvoidance()" style="width:100%; padding:10px 0; font-size:11px; border-color:#22c55e; color:#22c55e;">
              🛡️ COLLISION AVOIDANCE: ACTIVE
            </button>
          </div>

          <!-- Dual Live Sonar Radar Readouts -->
          <div style="display:grid; grid-template-columns:1fr 1fr; gap:8px; margin-top:10px;">
            <!-- Front Sensor -->
            <div style="background:#1e293b; border:1px solid #334155; border-radius:8px; padding:8px; text-align:center;" id="sonarBoxFront">
              <div style="font-size:10px; font-weight:bold; color:#38bdf8;">▲ FRONT SENSOR</div>
              <div id="frontDistVal" style="font-family:'Orbitron',sans-serif; font-size:16px; font-weight:900; color:#22c55e; margin:4px 0;">CLEAR</div>
              <div id="frontDistSub" style="font-size:9px; color:#64748b;">(No obstacle)</div>
            </div>

            <!-- Rear Sensor -->
            <div style="background:#1e293b; border:1px solid #334155; border-radius:8px; padding:8px; text-align:center;" id="sonarBoxBack">
              <div style="font-size:10px; font-weight:bold; color:#f59e0b;">▼ REAR SENSOR</div>
              <div id="backDistVal" style="font-family:'Orbitron',sans-serif; font-size:16px; font-weight:900; color:#22c55e; margin:4px 0;">CLEAR</div>
              <div id="backDistSub" style="font-size:9px; color:#64748b;">(No obstacle)</div>
            </div>
          </div>

          <!-- Sensitivity / Stop Distance Slider -->
          <div style="margin-top:12px;">
            <div style="display:flex; justify-content:space-between; font-size:10px; color:#94a3b8; font-weight:bold;">
              <span>STOP SENSITIVITY DISTANCE</span>
              <span id="sonarDistLabel" style="color:#00f3ff;">30 cm (12 in)</span>
            </div>
            <input type="range" id="sonarDistSlider" class="control-slider" min="10" max="100" step="5" value="30" oninput="onCollisionDistChange(this.value)">
          </div>

          <!-- 1-Tap Sensitivity Presets -->
          <div style="display:flex; gap:4px; margin-top:6px;">
            <button class="preset-btn" id="btnSon15" onclick="setCollisionDistPreset(15)">15cm (Close)</button>
            <button class="preset-btn active" id="btnSon30" onclick="setCollisionDistPreset(30)">30cm (Normal)</button>
            <button class="preset-btn" id="btnSon50" onclick="setCollisionDistPreset(50)">50cm (Cautious)</button>
            <button class="preset-btn" id="btnSon75" onclick="setCollisionDistPreset(75)">75cm (Far)</button>
          </div>

          <div style="font-size:9px; color:#64748b; margin-top:8px; line-height:1.4;">
            💡 When an obstacle enters the stop zone, drive in that direction is automatically disengaged and prevented. Steering and opposite drive remain active to navigate away.
          </div>
        </div>
      </div>

      <!-- PANE 4: GAMEPAD PAIRING -->
      <div class="settings-tab-pane" id="paneGamepad">
        <div class="panel-card" style="margin-top:0;">
          <div class="panel-title">
            <span>📡 CONTROLLER PAIRING</span>
          </div>
          <div style="display:flex; gap:8px; width:100%; margin-top:6px;">
            <button class="btn-util" onclick="triggerBleScan()" style="border-color:#00f3ff; color:#00f3ff; font-weight:bold; padding:10px 0; flex:1;">
              🎮 PAIR CONTROLLER
            </button>
            <button class="btn-util" onclick="forgetGamepad()" style="border-color:#ef4444; color:#ef4444; font-weight:bold; padding:10px 0; max-width:90px;">
              FORGET
            </button>
          </div>
          <div id="pairStatus" style="font-size:10px; color:#94a3b8; margin-top:6px; font-family:'Orbitron',sans-serif; text-align:center;">
            STANDALONE AUTO-RECONNECT ACTIVE: TURN ON CONTROLLER TO DRIVE!
          </div>
        </div>
      </div>

      <!-- PANE 5: MULTIMETER DIAGNOSTICS -->
      <div class="settings-tab-pane" id="paneDiag">
        <div class="panel-card" style="margin-top:0;">
          <div class="panel-title">
            <span>⚡ LIVE GPIO SIGNAL MONITOR</span>
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

      <!-- PANE 6: WIRING & ROBOTLAB GUIDE -->
      <div class="settings-tab-pane" id="paneWiring">
        <div class="doc-card" style="margin-top:0;">
          <h3>ESP32-S3 PINOUT TABLE</h3>
          <table class="doc-table">
            <tr><th>ESP32 Pin</th><th>Target Module</th><th>Function</th></tr>
            <tr><td><code>GPIO 4</code></td><td>IN1 Relay / L298N</td><td>Drive Forward (PWM)</td></tr>
            <tr><td><code>GPIO 5</code></td><td>IN2 Relay / L298N</td><td>Drive Reverse (PWM)</td></tr>
            <tr><td><code>GPIO 6</code></td><td>IN3 Relay / L298N</td><td>Steer Left</td></tr>
            <tr><td><code>GPIO 7</code></td><td>IN4 Relay / L298N</td><td>Steer Right</td></tr>
            <tr><td><code>GPIO 8</code></td><td>Front HC-SR04 TRIG</td><td>Front Sonar Trigger (3.3V)</td></tr>
            <tr><td><code>GPIO 9</code></td><td>Front HC-SR04 ECHO</td><td>Front Sonar Echo (via Resistor)</td></tr>
            <tr><td><code>GPIO 10</code></td><td>Back HC-SR04 TRIG</td><td>Rear Sonar Trigger (3.3V)</td></tr>
            <tr><td><code>GPIO 11</code></td><td>Back HC-SR04 ECHO</td><td>Rear Sonar Echo (via Resistor)</td></tr>
            <tr><td><code>GPIO 18</code></td><td>MAX98357A DIN</td><td>I2S Audio Data</td></tr>
            <tr><td><code>GPIO 17</code></td><td>MAX98357A BCLK</td><td>I2S Bit Clock</td></tr>
            <tr><td><code>GPIO 16</code></td><td>MAX98357A LRC</td><td>I2S Word Select</td></tr>
            <tr><td><code>5V / VIN</code></td><td>VCC (All Modules)</td><td>5V Power Rail (Amp & Sensors)</td></tr>
            <tr><td><code>GND</code></td><td>GND (All Modules)</td><td>Shared Common Ground</td></tr>
          </table>
        </div>

        <div class="doc-card">
          <h3>🛡️ DUAL HC-SR04 ULTRASONIC SENSOR WIRING</h3>
          <p style="font-size:11px; color:#cbd5e1; line-height:1.4;">
            <b>Front Sensor Connections:</b><br>
            • <b>VCC</b> ➔ <code>5V / VIN</code> (HC-SR04 requires 5V power)<br>
            • <b>GND</b> ➔ <code>ESP32 GND</code> (Common Ground)<br>
            • <b>TRIG</b> ➔ <code>GPIO 8</code> (Direct connection)<br>
            • <b>ECHO</b> ➔ <code>GPIO 9</code> (via 1kΩ series resistor or 1kΩ/2kΩ divider)<br><br>
            <b>Rear / Back Sensor Connections:</b><br>
            • <b>VCC</b> ➔ <code>5V / VIN</code><br>
            • <b>GND</b> ➔ <code>ESP32 GND</code><br>
            • <b>TRIG</b> ➔ <code>GPIO 10</code> (Direct connection)<br>
            • <b>ECHO</b> ➔ <code>GPIO 11</code> (via 1kΩ series resistor or 1kΩ/2kΩ divider)<br><br>
            <span style="color:#00ff88;">⚡ 3.3V Protection Tip:</span> Standard HC-SR04 echo pins output 5V. Placing a 1kΩ resistor in line protects the ESP32 input. 3.3V models (HC-SR04P/RCWL-9610) can connect directly.
          </p>
        </div>

        <div class="doc-card">
          <h3>⚡ L298N DUAL H-BRIDGE MOTOR WIRING</h3>
          <p style="font-size:11px; color:#cbd5e1; line-height:1.4;">
            • <b>IN1</b> ➔ <code>GPIO 4</code> (Forward PWM Speed Control)<br>
            • <b>IN2</b> ➔ <code>GPIO 5</code> (Reverse PWM Speed Control)<br>
            • <b>ENA</b> ➔ Keep 5V Jumper ON<br>
            • <b>+12V / VCC</b> ➔ Drive Battery Positive (+6V to +12V)<br>
            • <b>GND</b> ➔ Battery Negative & ESP32 GND (Common Ground)<br>
            • <b>OUT1 & OUT2</b> ➔ Drive DC Motor Terminals (+ / -)<br>
          </p>
        </div>

        <div class="doc-card">
          <h3>H-BRIDGE & RELAY WIRING</h3>
          <p style="font-size:11px; color:#cbd5e1; line-height:1.4;">
            • <b>Relays 1 & 2 (Drive 6V):</b> COM ➔ Motor A/B. NOs = +6V, NCs = Ground.<br>
            • <b>Relays 3 & 4 (Steering 12V):</b> COM ➔ Actuator. NOs = +12V, NCs = Ground.<br>
            <span style="color:#ef4444;">⚠️ Common Ground required across ESP32, 6V, 12V, and Amp.</span>
          </p>
        </div>
      </div>

      <!-- Done Button in Modal -->
      <div style="margin-top:14px;">
        <button class="btn-util" onclick="closeSettingsModal()" style="border-color:#22c55e; color:#22c55e; font-weight:bold; width:100%; padding:10px 0;">
          ✓ DONE / CLOSE SETTINGS
        </button>
      </div>
    </div>
  </div>

  <script>
    const wheelWrapper = document.getElementById('wheelWrapper');
    const btnGo = document.getElementById('btnGo');
    const btnBack = document.getElementById('btnBack');

    const openSettingsModal = () => {
      document.getElementById('settingsModal').classList.add('active');
    };

    const closeSettingsModal = () => {
      document.getElementById('settingsModal').classList.remove('active');
    };

    const switchSettingsPane = (paneId, btn) => {
      document.querySelectorAll('.settings-tab-pane').forEach(el => el.classList.remove('active'));
      document.querySelectorAll('.settings-tab-btn').forEach(el => el.classList.remove('active'));
      const p = document.getElementById(paneId);
      if (p) p.classList.add('active');
      if (btn) btn.classList.add('active');
    };

    const openSettingsTab = (paneId) => {
      openSettingsModal();
      const btnMap = {
        'paneSteering': 'stabSteer',
        'paneDriver': 'stabDriver',
        'paneThemes': 'stabThemes',
        'paneSensors': 'stabSensors',
        'paneGamepad': 'stabGamepad',
        'paneDiag': 'stabDiag',
        'paneWiring': 'stabWiring'
      };
      const b = document.getElementById(btnMap[paneId]);
      switchSettingsPane(paneId, b);
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

    let currentLeftLimit = 2500;
    let currentRightLimit = 2500;
    let isLimitsEnabled = true;

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
      document.getElementById('leftValLabel').innerText = leftVal + 'ms';
      document.getElementById('rightValLabel').innerText = rightVal + 'ms';
      fetch('/set_steer_limits?left=' + leftVal + '&right=' + rightVal);
    };

    const setSteerPreset = (val) => {
      const sL = document.getElementById('leftLimitSlider');
      const sR = document.getElementById('rightLimitSlider');
      if (val > sL.max) sL.max = val;
      if (val > sR.max) sR.max = val;
      sL.value = val;
      sR.value = val;
      onSteerLimitsChange();
    };

    const adjustSteerLimits = (delta) => {
      const sL = document.getElementById('leftLimitSlider');
      const sR = document.getElementById('rightLimitSlider');
      const l = Math.max(100, Math.min(15000, parseInt(sL.value) + delta));
      const r = Math.max(100, Math.min(15000, parseInt(sR.value) + delta));
      if (l > sL.max) sL.max = l;
      if (r > sR.max) sR.max = r;
      sL.value = l;
      sR.value = r;
      onSteerLimitsChange();
    };

    const setLeftLimitNow = () => {
      fetch('/set_left_limit')
        .then(r => r.json())
        .then(d => {
          if (d.left) {
            const s = document.getElementById('leftLimitSlider');
            if (d.left > s.max) s.max = d.left;
            s.value = d.left;
            onSteerLimitsChange();
          }
        });
    };

    const setRightLimitNow = () => {
      fetch('/set_right_limit')
        .then(r => r.json())
        .then(d => {
          if (d.right) {
            const s = document.getElementById('rightLimitSlider');
            if (d.right > s.max) s.max = d.right;
            s.value = d.right;
            onSteerLimitsChange();
          }
        });
    };

    const toggleFreeSteer = () => {
      fetch('/toggle_steer_limits')
        .then(r => r.json())
        .then(d => {
          isLimitsEnabled = d.limits_enabled;
          updateLimitsModeHud(isLimitsEnabled);
        });
    };

    const updateLimitsModeHud = (enabled) => {
      const lbl = document.getElementById('limitsModeLabel');
      const btn = document.getElementById('btnFreeSteer');
      const badge = document.getElementById('mainLimitsBadge');
      if (enabled) {
        if (lbl) { lbl.innerText = 'LIMITS: ACTIVE'; lbl.style.color = '#22c55e'; }
        if (btn) { btn.classList.remove('active'); btn.innerText = '🔓 FREE STEER'; }
        if (badge) { badge.innerText = 'LIMITS: ACTIVE'; badge.style.color = '#22c55e'; }
      } else {
        if (lbl) { lbl.innerText = 'LIMITS: UNRESTRICTED'; lbl.style.color = '#ec4899'; }
        if (btn) { btn.classList.add('active'); btn.innerText = '🔒 RESTRICT'; }
        if (badge) { badge.innerText = 'LIMITS: FREE'; badge.style.color = '#ec4899'; }
      }
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

    const themeNames = ['🏎️ V8 SIM', '🍄 MARIO KART', '🛸 F-ZERO', '🌴 CRUIS\'N USA'];
    const updateThemeButtons = (id) => {
      id = parseInt(id);
      const name = themeNames[id] || '🏎️ V8 SIM';
      const lbl = document.getElementById('themeLabel');
      const badge = document.getElementById('mainThemeBadge');
      if (lbl) lbl.innerText = name;
      if (badge) badge.innerText = name;
      for (let i = 0; i < 4; i++) {
        const b = document.getElementById('btnTheme' + i);
        if (b) b.classList.toggle('active', i === id);
      }
    };

    const setSoundTheme = (id) => {
      updateThemeButtons(id);
      fetch('/set_theme?val=' + id);
    };

    const onSteerSpeedChange = (val) => {
      document.getElementById('steerSpeedLabel').innerText = val + '% SPEED';
      updateSteerSpeedPresetButtons(val);
      fetch('/set_steer_speed?val=' + val);
    };

    const setSteerSpeedPreset = (val) => {
      document.getElementById('steerSpeedSlider').value = val;
      onSteerSpeedChange(val);
    };

    const updateSteerSpeedPresetButtons = (val) => {
      val = parseInt(val);
      const bSlow = document.getElementById('btnStrSpdSlow');
      const bMed = document.getElementById('btnStrSpdMed');
      const bFast = document.getElementById('btnStrSpdFast');
      if (bSlow) bSlow.classList.toggle('active', val <= 40);
      if (bMed) bMed.classList.toggle('active', val > 40 && val < 85);
      if (bFast) bFast.classList.toggle('active', val >= 85);
    };

    const updateSteerMotorDriverHud = (id) => {
      id = parseInt(id);
      const isL298N = (id === 1);
      const lbl = document.getElementById('steerDriverLabel');
      const b0 = document.getElementById('btnSteerDriver0');
      const b1 = document.getElementById('btnSteerDriver1');
      const desc = document.getElementById('steerDriverDesc');
      const rampGrp = document.getElementById('steerRampGroup');
      if (lbl) {
        lbl.innerText = isL298N ? '⚡ L298N H-BRIDGE' : '🎛️ 4-CH RELAYS';
        lbl.style.color = isL298N ? '#00ff88' : '#f59e0b';
      }
      if (b0) b0.classList.toggle('active', !isL298N);
      if (b1) b1.classList.toggle('active', isL298N);
      if (desc) {
        desc.innerText = isL298N ? 
          'Mode: L298N Hardware PWM (Smooth & Variable Turning Speed on GPIO 6/7)' : 
          'Mode: 4-Channel Relays (Digital ON/OFF on GPIO 6/7)';
      }
      if (rampGrp) rampGrp.style.opacity = isL298N ? '1' : '0.4';
    };

    const setSteerMotorDriver = (id) => {
      updateSteerMotorDriverHud(id);
      fetch('/set_steer_driver?val=' + id);
    };

    const triggerAutoHome = () => {
      const btn = document.getElementById('btnAutoHome');
      if (btn) {
        btn.innerText = '⏳ HOMING TO HARD LIMIT...';
        btn.style.color = '#00f3ff';
        btn.style.borderColor = '#00f3ff';
      }
      fetch('/auto_home')
        .then(() => {
          setTimeout(() => {
            if (btn) {
              btn.innerText = '🏠 AUTO-CALIBRATE TO HARD LIMIT (Zero Drift & Recenter)';
              btn.style.color = '#ec4899';
              btn.style.borderColor = '#ec4899';
            }
          }, 4000);
        });
    };

    const updateMotorDriverHud = (id) => {
      id = parseInt(id);
      const isL298N = (id === 1);
      const lbl = document.getElementById('driverLabel');
      const b0 = document.getElementById('btnDriver0');
      const b1 = document.getElementById('btnDriver1');
      const desc = document.getElementById('driverDesc');
      const badge = document.getElementById('mainDriverBadge');
      const rampGrp = document.getElementById('driveRampGroup');
      if (lbl) {
        lbl.innerText = isL298N ? '⚡ L298N H-BRIDGE' : '🎛️ 4-CH RELAYS';
        lbl.style.color = isL298N ? '#00ff88' : '#f59e0b';
      }
      if (badge) {
        badge.innerText = isL298N ? '⚡ L298N' : '🎛️ RELAYS';
        badge.style.color = isL298N ? '#00ff88' : '#f59e0b';
      }
      if (b0) b0.classList.toggle('active', !isL298N);
      if (b1) b1.classList.toggle('active', isL298N);
      if (desc) {
        desc.innerText = isL298N ? 
          'Mode: L298N Solid-State Hardware PWM (Silent & High Torque on GPIO 4/5)' : 
          'Mode: 4-Channel Relay Module (Time-Sliced Soft PWM on GPIO 4/5)';
      }
      if (rampGrp) rampGrp.style.opacity = isL298N ? '1' : '0.4';
    };

    const setMotorDriver = (id) => {
      updateMotorDriverHud(id);
      fetch('/set_driver?val=' + id);
    };

    const updateDriveRampHud = (ms) => {
      ms = parseInt(ms);
      const slider = document.getElementById('driveRampSlider');
      const lbl = document.getElementById('driveRampLabel');
      if (slider) slider.value = ms;
      if (lbl) {
        if (ms === 0) lbl.innerText = '⚡ INSTANT (0s)';
        else lbl.innerText = (ms / 1000).toFixed(2) + 's (' + (ms < 500 ? 'Sport' : ms < 1000 ? 'Smooth' : 'Soft') + ')';
      }
      const presets = [0, 300, 600, 1200];
      for (let p of presets) {
        const b = document.getElementById('btnDRamp' + p);
        if (b) b.classList.toggle('active', Math.abs(ms - p) <= 50);
      }
    };

    const onDriveRampChange = (val) => {
      val = parseInt(val);
      updateDriveRampHud(val);
      fetch('/set_drive_ramp?val=' + val);
    };

    const setDriveRampPreset = (val) => {
      onDriveRampChange(val);
    };

    const updateSteerRampHud = (ms) => {
      ms = parseInt(ms);
      const slider = document.getElementById('steerRampSlider');
      const lbl = document.getElementById('steerRampLabel');
      if (slider) slider.value = ms;
      if (lbl) {
        if (ms === 0) lbl.innerText = '⚡ INSTANT (0s)';
        else lbl.innerText = (ms / 1000).toFixed(2) + 's (' + (ms < 300 ? 'Quick' : ms < 700 ? 'Smooth' : 'Soft') + ')';
      }
      const presets = [0, 200, 500, 1000];
      for (let p of presets) {
        const b = document.getElementById('btnSRamp' + p);
        if (b) b.classList.toggle('active', Math.abs(ms - p) <= 50);
      }
    };

    const onSteerRampChange = (val) => {
      val = parseInt(val);
      updateSteerRampHud(val);
      fetch('/set_steer_ramp?val=' + val);
    };

    const setSteerRampPreset = (val) => {
      onSteerRampChange(val);
    };

    let isSonarEnabled = true;
    let sonarThresholdDist = 30;

    const updateSonarHud = (d) => {
      if (d.sonar_en !== undefined) {
        isSonarEnabled = d.sonar_en;
        const btn = document.getElementById('btnSonarToggle');
        const stat = document.getElementById('sonarStatusLabel');
        const badge = document.getElementById('mainSensorsBadge');
        if (btn) {
          btn.classList.toggle('active', isSonarEnabled);
          btn.innerText = isSonarEnabled ? '🛡️ COLLISION AVOIDANCE: ACTIVE' : '🛡️ COLLISION AVOIDANCE: DISABLED';
          btn.style.borderColor = isSonarEnabled ? '#22c55e' : '#64748b';
          btn.style.color = isSonarEnabled ? '#22c55e' : '#64748b';
        }
        if (stat) {
          stat.innerText = isSonarEnabled ? 'ACTIVE' : 'OFF';
          stat.style.color = isSonarEnabled ? '#22c55e' : '#64748b';
        }
        if (badge && !isSonarEnabled) {
          badge.innerText = '🛡️ SENSORS: OFF';
          badge.style.color = '#94a3b8';
          badge.style.borderColor = 'rgba(148,163,184,0.3)';
          badge.style.background = 'rgba(148,163,184,0.1)';
        }
      }

      if (d.sonar_dist !== undefined) {
        sonarThresholdDist = d.sonar_dist;
        const slider = document.getElementById('sonarDistSlider');
        const lbl = document.getElementById('sonarDistLabel');
        if (slider) slider.value = sonarThresholdDist;
        if (lbl) {
          const inch = Math.round(sonarThresholdDist / 2.54);
          lbl.innerText = sonarThresholdDist + ' cm (' + inch + ' in)';
        }
        updateSonarPresetButtons(sonarThresholdDist);
      }

      // Live Distance Readouts & Radar Boxes
      const fVal = document.getElementById('frontDistVal');
      const fSub = document.getElementById('frontDistSub');
      const fBox = document.getElementById('sonarBoxFront');
      if (fVal && d.front_cm !== undefined) {
        if (d.front_cm > 0) {
          fVal.innerText = d.front_cm + ' cm';
          if (d.front_blk) {
            fVal.style.color = '#ef4444';
            if (fSub) fSub.innerText = '⚠️ OBSTACLE DETECTED!';
            if (fBox) fBox.style.borderColor = '#ef4444';
          } else {
            fVal.style.color = '#22c55e';
            if (fSub) fSub.innerText = '(Clear)';
            if (fBox) fBox.style.borderColor = '#334155';
          }
        } else {
          fVal.innerText = 'CLEAR';
          fVal.style.color = '#38bdf8';
          if (fSub) fSub.innerText = '(No obstacle)';
          if (fBox) fBox.style.borderColor = '#334155';
        }
      }

      const bVal = document.getElementById('backDistVal');
      const bSub = document.getElementById('backDistSub');
      const bBox = document.getElementById('sonarBoxBack');
      if (bVal && d.back_cm !== undefined) {
        if (d.back_cm > 0) {
          bVal.innerText = d.back_cm + ' cm';
          if (d.back_blk) {
            bVal.style.color = '#ef4444';
            if (bSub) bSub.innerText = '⚠️ OBSTACLE DETECTED!';
            if (bBox) bBox.style.borderColor = '#ef4444';
          } else {
            bVal.style.color = '#22c55e';
            if (bSub) bSub.innerText = '(Clear)';
            if (bBox) bBox.style.borderColor = '#334155';
          }
        } else {
          bVal.innerText = 'CLEAR';
          bVal.style.color = '#38bdf8';
          if (bSub) bSub.innerText = '(No obstacle)';
          if (bBox) bBox.style.borderColor = '#334155';
        }
      }

      // Directional Hazard Warning Banner in Main Cockpit
      const banner = document.getElementById('obstacleBanner');
      const bTitle = document.getElementById('obstacleBannerTitle');
      const bDesc = document.getElementById('obstacleBannerDesc');
      const badge = document.getElementById('mainSensorsBadge');
      if (banner && isSonarEnabled) {
        if (d.front_blk && d.back_blk) {
          banner.style.display = 'block';
          bTitle.innerText = '⚠️ FRONT & REAR HAZARD (' + d.front_cm + 'cm / ' + d.back_cm + 'cm)';
          bDesc.innerText = 'Both forward and reverse throttles inhibited';
          if (badge) { badge.innerText = '⚠️ BLOCKED: ' + Math.min(d.front_cm, d.back_cm) + 'cm'; badge.style.color = '#ef4444'; }
        } else if (d.front_blk) {
          banner.style.display = 'block';
          bTitle.innerText = '⚠️ FRONT HAZARD DETECTED (' + d.front_cm + ' cm)';
          bDesc.innerText = 'Forward drive inhibited // Reverse (S) & Steer are clear';
          if (badge) { badge.innerText = '⚠️ FRONT: ' + d.front_cm + 'cm'; badge.style.color = '#ef4444'; }
        } else if (d.back_blk) {
          banner.style.display = 'block';
          bTitle.innerText = '⚠️ REAR HAZARD DETECTED (' + d.back_cm + ' cm)';
          bDesc.innerText = 'Reverse drive inhibited // Forward (W) & Steer are clear';
          if (badge) { badge.innerText = '⚠️ REAR: ' + d.back_cm + 'cm'; badge.style.color = '#ef4444'; }
        } else {
          banner.style.display = 'none';
          if (badge) {
            badge.innerText = '🛡️ SENSORS: ' + sonarThresholdDist + 'cm';
            badge.style.color = '#22c55e';
            badge.style.borderColor = 'rgba(34,197,94,0.3)';
            badge.style.background = 'rgba(34,197,94,0.1)';
          }
        }
      } else if (banner) {
        banner.style.display = 'none';
      }
    };

    const toggleCollisionAvoidance = () => {
      fetch('/toggle_collision')
        .then(r => r.json())
        .then(d => {
          updateSonarHud(d);
        });
    };

    const onCollisionDistChange = (val) => {
      val = parseInt(val);
      const inch = Math.round(val / 2.54);
      document.getElementById('sonarDistLabel').innerText = val + ' cm (' + inch + ' in)';
      updateSonarPresetButtons(val);
      fetch('/set_collision_dist?val=' + val);
    };

    const setCollisionDistPreset = (val) => {
      document.getElementById('sonarDistSlider').value = val;
      onCollisionDistChange(val);
    };

    const updateSonarPresetButtons = (val) => {
      val = parseInt(val);
      const presets = [15, 30, 50, 75];
      for (let p of presets) {
        const b = document.getElementById('btnSon' + p);
        if (b) b.classList.toggle('active', Math.abs(val - p) <= 5);
      }
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
          if (cfg.steer_speed) {
            document.getElementById('steerSpeedSlider').value = cfg.steer_speed;
            document.getElementById('steerSpeedLabel').innerText = cfg.steer_speed + '% SPEED';
            updateSteerSpeedPresetButtons(cfg.steer_speed);
          }
          if (cfg.driver !== undefined) {
            updateMotorDriverHud(cfg.driver);
          }
          if (cfg.steer_driver !== undefined) {
            updateSteerMotorDriverHud(cfg.steer_driver);
          }
          if (cfg.drive_ramp !== undefined) {
            updateDriveRampHud(cfg.drive_ramp);
          }
          if (cfg.steer_ramp !== undefined) {
            updateSteerRampHud(cfg.steer_ramp);
          }
          if (cfg.steer_left) {
            const s = document.getElementById('leftLimitSlider');
            if (cfg.steer_left > s.max) s.max = cfg.steer_left;
            s.value = cfg.steer_left;
            currentLeftLimit = parseInt(cfg.steer_left);
            document.getElementById('leftLimitDisplay').innerText = 'LEFT: -' + cfg.steer_left + 'ms';
            document.getElementById('leftValLabel').innerText = cfg.steer_left + 'ms';
          }
          if (cfg.steer_right) {
            const s = document.getElementById('rightLimitSlider');
            if (cfg.steer_right > s.max) s.max = cfg.steer_right;
            s.value = cfg.steer_right;
            currentRightLimit = parseInt(cfg.steer_right);
            document.getElementById('rightLimitDisplay').innerText = 'RIGHT: +' + cfg.steer_right + 'ms';
            document.getElementById('rightValLabel').innerText = cfg.steer_right + 'ms';
          }
          if (cfg.limits_enabled !== undefined) {
            isLimitsEnabled = cfg.limits_enabled;
            updateLimitsModeHud(isLimitsEnabled);
          }
          if (cfg.vol !== undefined) {
            document.getElementById('volSlider').value = cfg.vol;
            document.getElementById('volLabel').innerText = cfg.mute ? '🔇 MUTED' : (cfg.vol + '% VOLUME');
            document.getElementById('btnMute').classList.toggle('active', cfg.mute);
            document.getElementById('btnVol100').classList.toggle('active', !cfg.mute && cfg.vol == 100);
          }
          if (cfg.theme !== undefined) {
            updateThemeButtons(cfg.theme);
          }
          if (cfg.pos !== undefined) {
            updateSteerMeter(cfg.pos);
          }
          if (cfg.sonar_en !== undefined || cfg.sonar_dist !== undefined) {
            updateSonarHud(cfg);
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
          if (d.sonar_en !== undefined) {
            updateSonarHud(d);
          }
          if (d.steer_driver !== undefined) {
            updateSteerMotorDriverHud(d.steer_driver);
          }
        })
        .catch(() => {});
    };

    // Gentle Heartbeat & Live Pin Status Poll (every 1.5 seconds to prevent TCP socket exhaustion)
    setInterval(fetchStatus, 1500);

    let scanPollTimer = null;
    const triggerBleScan = () => {
      const statusEl = document.getElementById('pairStatus');
      const cockpitEl = document.getElementById('cockpitPairStatus');
      if (statusEl) { statusEl.innerText = '🔍 SCANNING FOR WIRELESS CONTROLLER (10s)...'; statusEl.style.color = '#00f3ff'; }
      if (cockpitEl) { cockpitEl.innerText = '🔍 SCANNING FOR WIRELESS CONTROLLER (10s)...'; }
      fetch('/scan_ble')
        .then(res => res.text())
        .then(msg => {
          if (statusEl) { statusEl.innerText = msg; statusEl.style.color = '#00f3ff'; }
          if (cockpitEl) { cockpitEl.innerText = msg; }
          if (scanPollTimer) clearInterval(scanPollTimer);
          scanPollTimer = setInterval(pollBleStatus, 750);
        })
        .catch(err => {
          if (statusEl) { statusEl.innerText = 'SCAN ERROR: ' + err; statusEl.style.color = '#ef4444'; }
        });
    };

    const pollBleStatus = () => {
      fetch('/ble_status')
        .then(r => r.json())
        .then(data => {
          const statusEl = document.getElementById('pairStatus');
          const cockpitEl = document.getElementById('cockpitPairStatus');
          if (data.status) {
            if (statusEl) statusEl.innerText = data.status;
            if (cockpitEl) cockpitEl.innerText = data.status;
            if (data.connected && data.connected.length > 0) {
              if (statusEl) statusEl.style.color = '#00ff88';
              if (scanPollTimer) { clearInterval(scanPollTimer); scanPollTimer = null; }
            } else if (!data.scanning) {
              if (statusEl) statusEl.style.color = '#cbd5e1';
              if (scanPollTimer) { clearInterval(scanPollTimer); scanPollTimer = null; }
            } else {
              if (statusEl) statusEl.style.color = '#00f3ff';
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
          const cockpitEl = document.getElementById('cockpitPairStatus');
          const msg = 'CONTROLLER FORGOTTEN. PUT NEW CONTROLLER IN PAIRING MODE!';
          if (statusEl) { statusEl.innerText = msg; statusEl.style.color = '#f59e0b'; }
          if (cockpitEl) { cockpitEl.innerText = msg; }
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
  currentDrivePwm = 0;
  if (driveMotorDriver == DRIVER_L298N) {
    analogWrite(pinForward, 0);
    analogWrite(pinBack, 0);
  } else {
    setRelayOff(pinForward);
    setRelayOff(pinBack);
  }
  currentDriveState = DRIVE_STOP;
}

void updateSteeringPosition() {
  unsigned long now = millis();
  long delta = now - lastSteerUpdateMs;
  if (delta > 0) {
    long effectiveDelta = delta;
    if (steerMotorDriver == DRIVER_L298N) {
      if (steerRampMs > 0 && currentSteerPwm > 0) {
        effectiveDelta = (delta * currentSteerPwm) / 255;
      } else {
        effectiveDelta = (delta * steerSpeedPercent) / 100;
      }
    }
    if (currentSteerState == STEER_LEFT) {
      currentPositionMs -= effectiveDelta;
    } else if (currentSteerState == STEER_RIGHT) {
      currentPositionMs += effectiveDelta;
    }
  }
  lastSteerUpdateMs = now;
}

void stopSteering() {
  if (currentSteerState != STEER_IDLE) {
     updateSteeringPosition();
  }
  currentSteerPwm = 0;
  if (steerMotorDriver == DRIVER_L298N) {
    analogWrite(pinLeft, 0);
    analogWrite(pinRight, 0);
  } else {
    setRelayOff(pinLeft);
    setRelayOff(pinRight);
  }
  currentSteerState = STEER_IDLE;
}

void startSteeringLeft() {
  stopSteering(); delay(20);
  currentSteerState = STEER_LEFT;
  lastSteerUpdateMs = millis();
  steerRampStartMs = millis();
  if (steerMotorDriver == DRIVER_L298N) {
    int targetPwm = map(steerSpeedPercent, 0, 100, 0, 255);
    analogWrite(pinRight, 0);
    if (steerRampMs <= 0) {
      currentSteerPwm = targetPwm;
      analogWrite(pinLeft, currentSteerPwm);
    } else {
      currentSteerPwm = 0;
      analogWrite(pinLeft, 0);
    }
  } else {
    setRelayOn(pinLeft);
  }
}

void startSteeringRight() {
  stopSteering(); delay(20);
  currentSteerState = STEER_RIGHT;
  lastSteerUpdateMs = millis();
  steerRampStartMs = millis();
  if (steerMotorDriver == DRIVER_L298N) {
    int targetPwm = map(steerSpeedPercent, 0, 100, 0, 255);
    analogWrite(pinLeft, 0);
    if (steerRampMs <= 0) {
      currentSteerPwm = targetPwm;
      analogWrite(pinRight, currentSteerPwm);
    } else {
      currentSteerPwm = 0;
      analogWrite(pinRight, 0);
    }
  } else {
    setRelayOn(pinRight);
  }
}

void updateMotorRamping() {
  unsigned long now = millis();

  // 1. DRIVE MOTOR ACCELERATION RAMP (L298N Mode)
  if (driveMotorDriver == DRIVER_L298N) {
    if (currentDriveState == DRIVE_FORWARD || currentDriveState == DRIVE_BACK) {
      int targetPwm = map(driveSpeedPercent, 0, 100, 0, 255);
      if (driveRampMs <= 0) {
        currentDrivePwm = targetPwm;
      } else {
        unsigned long elapsed = now - driveRampStartMs;
        if (elapsed >= (unsigned long)driveRampMs) {
          currentDrivePwm = targetPwm;
        } else {
          currentDrivePwm = (int)((targetPwm * elapsed) / driveRampMs);
        }
      }
      if (currentDriveState == DRIVE_FORWARD) {
        analogWrite(pinBack, 0);
        analogWrite(pinForward, currentDrivePwm);
      } else if (currentDriveState == DRIVE_BACK) {
        analogWrite(pinForward, 0);
        analogWrite(pinBack, currentDrivePwm);
      }
    }
  }

  // 2. STEERING MOTOR ACCELERATION RAMP (L298N Mode)
  if (steerMotorDriver == DRIVER_L298N) {
    if (currentSteerState == STEER_LEFT || currentSteerState == STEER_RIGHT) {
      int targetPwm = map(steerSpeedPercent, 0, 100, 0, 255);
      if (steerRampMs <= 0) {
        currentSteerPwm = targetPwm;
      } else {
        unsigned long elapsed = now - steerRampStartMs;
        if (elapsed >= (unsigned long)steerRampMs) {
          currentSteerPwm = targetPwm;
        } else {
          currentSteerPwm = (int)((targetPwm * elapsed) / steerRampMs);
        }
      }
      if (currentSteerState == STEER_LEFT) {
        analogWrite(pinRight, 0);
        analogWrite(pinLeft, currentSteerPwm);
      } else if (currentSteerState == STEER_RIGHT) {
        analogWrite(pinLeft, 0);
        analogWrite(pinRight, currentSteerPwm);
      }
    }
  }
}

void startAutoHoming() {
  isAutoHoming = true;
  isCentering = false;
  autoHomeStartMs = millis();
  autoHomeDurationMs = max(3500L, steerLimitLeftMs + 1000L);
  startSteeringLeft(); // Drive toward the physical hardware limit switch
  Serial.printf("🎯 AUTO-HOMING STARTED: Driving to hard limit for %ld ms...\n", autoHomeDurationMs);
}

void handleAutoHomingLoop() {
  if (!isAutoHoming) return;
  unsigned long elapsed = millis() - autoHomeStartMs;
  if (elapsed >= autoHomeDurationMs) {
    // Reached physical limit switch!
    stopSteering();
    isAutoHoming = false;
    currentPositionMs = -steerLimitLeftMs; // Calibrated to known limit position
    Serial.println("🎯 HARD LIMIT REACHED! Recalibrated position. Centering now...");
    triggerSetCenterFlash();
    isCentering = true; // Automatically center steering from the known home!
  }
}

void stopAll() {
  stopDriving();
  stopSteering();
  isCentering = false;
  isAutoHoming = false;
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

void playCollisionAlertTone() {
  if (millis() - lastCollisionAlertToneMs > 350) {
    lastCollisionAlertToneMs = millis();
    playI2STone(880, 70);
    delay(25);
    playI2STone(440, 90);
  }
}

void updateSonarCollisionCheck() {
  unsigned long now = millis();
  if (now - lastSonarPollMs < 70) return; // 70ms between interleaved pings
  lastSonarPollMs = now;

  if (!collisionAvoidanceEnabled) {
    frontBlocked = false;
    backBlocked = false;
    frontDistanceCm = -1;
    backDistanceCm = -1;
    return;
  }

  if (sonarPollCycle == 0) {
    // Interleaved Ping 1: Front HC-SR04
    digitalWrite(pinTrigFront, LOW);
    delayMicroseconds(2);
    digitalWrite(pinTrigFront, HIGH);
    delayMicroseconds(10);
    digitalWrite(pinTrigFront, LOW);
    unsigned long duration = pulseIn(pinEchoFront, HIGH, 7000); // 7ms max (~120cm)
    if (duration > 0) {
      frontDistanceCm = (int)(duration * 0.0343 / 2.0);
      frontBlocked = (frontDistanceCm > 0 && frontDistanceCm <= collisionThresholdCm);
    } else {
      frontDistanceCm = -1;
      frontBlocked = false;
    }
    sonarPollCycle = 1;
  } else {
    // Interleaved Ping 2: Back HC-SR04
    digitalWrite(pinTrigBack, LOW);
    delayMicroseconds(2);
    digitalWrite(pinTrigBack, HIGH);
    delayMicroseconds(10);
    digitalWrite(pinTrigBack, LOW);
    unsigned long duration = pulseIn(pinEchoBack, HIGH, 7000); // 7ms max (~120cm)
    if (duration > 0) {
      backDistanceCm = (int)(duration * 0.0343 / 2.0);
      backBlocked = (backDistanceCm > 0 && backDistanceCm <= collisionThresholdCm);
    } else {
      backDistanceCm = -1;
      backBlocked = false;
    }
    sonarPollCycle = 0;
  }

  // Active Emergency Braking if traveling toward an obstacle
  if (currentDriveState == DRIVE_FORWARD && frontBlocked) {
    stopDriving();
    playCollisionAlertTone();
    setLedColor(255, 0, 0); // Warning Red
    Serial.printf("⚠️ EMERGENCY BRAKE: Front Obstacle at %d cm!\n", frontDistanceCm);
  } else if (currentDriveState == DRIVE_BACK && backBlocked) {
    stopDriving();
    playCollisionAlertTone();
    setLedColor(255, 0, 0); // Warning Red
    Serial.printf("⚠️ EMERGENCY BRAKE: Rear Obstacle at %d cm!\n", backDistanceCm);
  }
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
    if (collisionAvoidanceEnabled && frontBlocked) {
      playCollisionAlertTone();
      setLedColor(255, 60, 0); // Amber Warning
      Serial.printf("⚠️ FORWARD INHIBITED: Front Obstacle at %d cm!\n", frontDistanceCm);
      return;
    }
    stopDriving(); delay(20);
    currentDriveState = DRIVE_FORWARD;
    forwardCycleStartMs = millis();
    driveRampStartMs = millis();
    if (driveMotorDriver == DRIVER_L298N) {
      int targetPwm = map(driveSpeedPercent, 0, 100, 0, 255);
      analogWrite(pinBack, 0);
      if (driveRampMs <= 0) {
        currentDrivePwm = targetPwm;
        analogWrite(pinForward, currentDrivePwm);
      } else {
        currentDrivePwm = 0;
        analogWrite(pinForward, 0);
      }
    } else {
      setRelayOn(pinForward);
    }
  }
  else if (cmd == "back" || cmd == "B" || cmd == "b" || cmd == "S" || cmd == "s") {
    if (collisionAvoidanceEnabled && backBlocked) {
      playCollisionAlertTone();
      setLedColor(255, 60, 0); // Amber Warning
      Serial.printf("⚠️ REVERSE INHIBITED: Rear Obstacle at %d cm!\n", backDistanceCm);
      return;
    }
    stopDriving(); delay(20);
    currentDriveState = DRIVE_BACK;
    driveRampStartMs = millis();
    if (driveMotorDriver == DRIVER_L298N) {
      int targetPwm = map(driveSpeedPercent, 0, 100, 0, 255);
      analogWrite(pinForward, 0);
      if (driveRampMs <= 0) {
        currentDrivePwm = targetPwm;
        analogWrite(pinBack, currentDrivePwm);
      } else {
        currentDrivePwm = 0;
        analogWrite(pinBack, 0);
      }
    } else {
      setRelayOn(pinBack);
    }
  }
  else if (cmd == "left" || cmd == "L" || cmd == "l" || cmd == "A" || cmd == "a") {
    isCentering = false;
    isAutoHoming = false;
    if (!steerLimitsEnabled || currentPositionMs > -steerLimitLeftMs) {
      startSteeringLeft();
    }
  }
  else if (cmd == "right" || cmd == "R" || cmd == "r" || cmd == "D" || cmd == "d") {
    isCentering = false;
    isAutoHoming = false;
    if (!steerLimitsEnabled || currentPositionMs < steerLimitRightMs) {
      startSteeringRight();
    }
  }
  else if (cmd == "auto_home" || cmd == "home" || cmd == "H" || cmd == "h") {
    startAutoHoming();
  }
  else if (cmd == "stop_steer" || cmd == "t") {
    isCentering = false;
    isAutoHoming = false;
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
    isAutoHoming = false;
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
  server.send_P(200, "text/html", htmlPage);
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

  // Initialize Dual HC-SR04 Ultrasonic Pins
  pinMode(pinTrigFront, OUTPUT);
  pinMode(pinEchoFront, INPUT_PULLDOWN);
  digitalWrite(pinTrigFront, LOW);

  pinMode(pinTrigBack, OUTPUT);
  pinMode(pinEchoBack, INPUT_PULLDOWN);
  digitalWrite(pinTrigBack, LOW);
  
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
  
  // 5. Load Persistent Speed, Steering Limits, Volume, Theme, Driver, & Collision Avoidance from Flash
  prefs.begin("tink_cfg", true);
  driveSpeedPercent = prefs.getInt("speed", 60);
  steerSpeedPercent = prefs.getInt("s_spd", 75);
  driveRampMs = prefs.getInt("d_ramp", 400);
  steerRampMs = prefs.getInt("s_ramp", 250);
  steerLimitLeftMs = prefs.getLong("s_left", 2500);
  steerLimitRightMs = prefs.getLong("s_right", 2500);
  steerLimitsEnabled = prefs.getBool("s_lim_en", true);
  audioVolumePercent = prefs.getInt("vol", 100);
  audioMuted = prefs.getBool("mute", false);
  currentSoundTheme = (SoundTheme)prefs.getInt("theme", 0);
  driveMotorDriver = (MotorDriverType)prefs.getInt("driver", 0);
  steerMotorDriver = (MotorDriverType)prefs.getInt("s_driver", 0);
  collisionAvoidanceEnabled = prefs.getBool("son_en", true);
  collisionThresholdCm = prefs.getInt("son_dist", 30);
  prefs.end();
  Serial.printf("Loaded Config: Speed=%d%%, SteerSpd=%d%%, DriveRamp=%dms, SteerRamp=%dms, SteerLeft=%ldms, SteerRight=%ldms, LimEn=%d, Vol=%d%%, Mute=%d, Theme=%d, DriveDriver=%d, SteerDriver=%d, SonarEn=%d, SonarDist=%dcm\n", driveSpeedPercent, steerSpeedPercent, driveRampMs, steerRampMs, steerLimitLeftMs, steerLimitRightMs, steerLimitsEnabled, audioVolumePercent, audioMuted, (int)currentSoundTheme, (int)driveMotorDriver, (int)steerMotorDriver, collisionAvoidanceEnabled, collisionThresholdCm);

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
    json += "\"pos\":" + String(currentPositionMs) + ",";
    json += "\"steer_driver\":" + String((int)steerMotorDriver) + ",";
    json += "\"drive_driver\":" + String((int)driveMotorDriver) + ",";
    json += "\"drive_ramp\":" + String(driveRampMs) + ",";
    json += "\"steer_ramp\":" + String(steerRampMs) + ",";
    json += "\"steer_speed\":" + String(steerSpeedPercent) + ",";
    json += "\"is_homing\":" + String(isAutoHoming ? "true" : "false") + ",";
    json += "\"sonar_en\":" + String(collisionAvoidanceEnabled ? "true" : "false") + ",";
    json += "\"sonar_dist\":" + String(collisionThresholdCm) + ",";
    json += "\"front_cm\":" + String(frontDistanceCm) + ",";
    json += "\"back_cm\":" + String(backDistanceCm) + ",";
    json += "\"front_blk\":" + String(frontBlocked ? "true" : "false") + ",";
    json += "\"back_blk\":" + String(backBlocked ? "true" : "false");
    json += "}";
    server.send(200, "application/json", json);
  });

  server.on("/set_driver", []() {
    if (server.hasArg("val")) {
      int d = server.arg("val").toInt();
      stopDriving();
      driveMotorDriver = (MotorDriverType)constrain(d, 0, 1);
      prefs.begin("tink_cfg", false);
      prefs.putInt("driver", (int)driveMotorDriver);
      prefs.end();
      Serial.printf("Drive Motor Driver Set: %s\n", driveMotorDriver == DRIVER_L298N ? "L298N H-Bridge (PWM)" : "Relay Module");
    }
    server.send(200, "application/json", "{\"driver\":" + String((int)driveMotorDriver) + "}");
  });

  server.on("/set_steer_driver", []() {
    if (server.hasArg("val")) {
      int d = server.arg("val").toInt();
      stopSteering();
      steerMotorDriver = (MotorDriverType)constrain(d, 0, 1);
      prefs.begin("tink_cfg", false);
      prefs.putInt("s_driver", (int)steerMotorDriver);
      prefs.end();
      Serial.printf("Steer Motor Driver Set: %s\n", steerMotorDriver == DRIVER_L298N ? "L298N H-Bridge (PWM)" : "Relay Module");
    }
    server.send(200, "application/json", "{\"steer_driver\":" + String((int)steerMotorDriver) + "}");
  });

  server.on("/set_steer_speed", []() {
    if (server.hasArg("val")) {
      steerSpeedPercent = constrain(server.arg("val").toInt(), 25, 100);
      prefs.begin("tink_cfg", false);
      prefs.putInt("s_spd", steerSpeedPercent);
      prefs.end();
      Serial.printf("Steer Speed Updated: %d%%\n", steerSpeedPercent);
    }
    server.send(200, "application/json", "{\"steer_speed\":" + String(steerSpeedPercent) + "}");
  });

  server.on("/set_drive_ramp", []() {
    if (server.hasArg("val")) {
      driveRampMs = constrain(server.arg("val").toInt(), 0, 3000);
      prefs.begin("tink_cfg", false);
      prefs.putInt("d_ramp", driveRampMs);
      prefs.end();
      Serial.printf("Drive Ramp Updated: %d ms\n", driveRampMs);
    }
    server.send(200, "application/json", "{\"drive_ramp\":" + String(driveRampMs) + "}");
  });

  server.on("/set_steer_ramp", []() {
    if (server.hasArg("val")) {
      steerRampMs = constrain(server.arg("val").toInt(), 0, 2000);
      prefs.begin("tink_cfg", false);
      prefs.putInt("s_ramp", steerRampMs);
      prefs.end();
      Serial.printf("Steer Ramp Updated: %d ms\n", steerRampMs);
    }
    server.send(200, "application/json", "{\"steer_ramp\":" + String(steerRampMs) + "}");
  });

  server.on("/auto_home", []() {
    processCommand("auto_home");
    server.send(200, "text/plain", "Auto-Homing Started");
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

  server.on("/set_theme", []() {
    if (server.hasArg("val")) {
      int t = server.arg("val").toInt();
      currentSoundTheme = (SoundTheme)constrain(t, 0, 3);
      prefs.begin("tink_cfg", false);
      prefs.putInt("theme", (int)currentSoundTheme);
      prefs.end();
      Serial.printf("Sound Theme Set: %d\n", (int)currentSoundTheme);
      playThemeStartFanfare(); // Immediate signature audio feedback!
    }
    server.send(200, "application/json", "{\"theme\":" + String((int)currentSoundTheme) + "}");
  });

  server.on("/set_steer_limits", []() {
    if (server.hasArg("left")) {
      steerLimitLeftMs = constrain(server.arg("left").toInt(), 100, 15000);
    }
    if (server.hasArg("right")) {
      steerLimitRightMs = constrain(server.arg("right").toInt(), 100, 15000);
    }
    prefs.begin("tink_cfg", false);
    prefs.putLong("s_left", steerLimitLeftMs);
    prefs.putLong("s_right", steerLimitRightMs);
    prefs.end();
    server.send(200, "application/json", "{\"left\":" + String(steerLimitLeftMs) + ",\"right\":" + String(steerLimitRightMs) + "}");
  });

  server.on("/toggle_steer_limits", []() {
    steerLimitsEnabled = !steerLimitsEnabled;
    prefs.begin("tink_cfg", false);
    prefs.putBool("s_lim_en", steerLimitsEnabled);
    prefs.end();
    Serial.printf("Steer Limits Toggled: %d\n", steerLimitsEnabled);
    server.send(200, "application/json", "{\"limits_enabled\":" + String(steerLimitsEnabled ? "true" : "false") + "}");
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
    json += "\"steer_speed\":" + String(steerSpeedPercent) + ",";
    json += "\"steer_left\":" + String(steerLimitLeftMs) + ",";
    json += "\"steer_right\":" + String(steerLimitRightMs) + ",";
    json += "\"limits_enabled\":" + String(steerLimitsEnabled ? "true" : "false") + ",";
    json += "\"driver\":" + String((int)driveMotorDriver) + ",";
    json += "\"steer_driver\":" + String((int)steerMotorDriver) + ",";
    json += "\"drive_ramp\":" + String(driveRampMs) + ",";
    json += "\"steer_ramp\":" + String(steerRampMs) + ",";
    json += "\"vol\":" + String(audioVolumePercent) + ",";
    json += "\"mute\":" + String(audioMuted ? "true" : "false") + ",";
    json += "\"theme\":" + String((int)currentSoundTheme) + ",";
    json += "\"pos\":" + String(currentPositionMs) + ",";
    json += "\"sonar_en\":" + String(collisionAvoidanceEnabled ? "true" : "false") + ",";
    json += "\"sonar_dist\":" + String(collisionThresholdCm);
    json += "}";
    server.send(200, "application/json", json);
  });

  server.on("/toggle_collision", []() {
    collisionAvoidanceEnabled = !collisionAvoidanceEnabled;
    prefs.begin("tink_cfg", false);
    prefs.putBool("son_en", collisionAvoidanceEnabled);
    prefs.end();
    if (!collisionAvoidanceEnabled) {
      frontBlocked = false;
      backBlocked = false;
    }
    Serial.printf("Collision Avoidance Toggled: %d\n", collisionAvoidanceEnabled);
    server.send(200, "application/json", "{\"sonar_en\":" + String(collisionAvoidanceEnabled ? "true" : "false") + "}");
  });

  server.on("/set_collision_dist", []() {
    if (server.hasArg("val")) {
      collisionThresholdCm = constrain(server.arg("val").toInt(), 10, 100);
      prefs.begin("tink_cfg", false);
      prefs.putInt("son_dist", collisionThresholdCm);
      prefs.end();
      Serial.printf("Collision Distance Threshold Set: %d cm\n", collisionThresholdCm);
    }
    server.send(200, "application/json", "{\"sonar_dist\":" + String(collisionThresholdCm) + "}");
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

  // Non-blocking Dual HC-SR04 Sonar Ping & Directional Emergency Braking
  updateSonarCollisionCheck();

  // Safe Asynchronous Gamepad Pairing
  handleGamepadConnectionLoop();

  // Dynamic Drive Speed Controller (L298N Hardware PWM vs Relay Soft PWM)
  if (currentDriveState == DRIVE_FORWARD || currentDriveState == DRIVE_BACK) {
    if (driveMotorDriver == DRIVER_L298N) {
      int pwmVal = map(driveSpeedPercent, 0, 100, 0, 255);
      if (currentDriveState == DRIVE_FORWARD) {
        analogWrite(pinBack, 0);
        analogWrite(pinForward, pwmVal);
      } else {
        analogWrite(pinForward, 0);
        analogWrite(pinBack, pwmVal);
      }
    } else {
      // Relay Module (Time-sliced 120ms Soft PWM)
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
  }

  // Real-time Racing Simulator Dynamic Soundscape (Idle, Shifting, Drifting)
  updateSimulatorAudio();

  // Smooth Hardware Motor & Steering Acceleration Ramping (L298N Mode)
  updateMotorRamping();

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
  
  // Steering Limits, Auto-Homing, & Centering Logic
  if (isAutoHoming) {
     handleAutoHomingLoop();
  } else if (isCentering) {
     if (currentPositionMs > 40) {
         if (currentSteerState != STEER_LEFT) {
             startSteeringLeft();
         } else {
             updateSteeringPosition();
         }
     } else if (currentPositionMs < -40) {
         if (currentSteerState != STEER_RIGHT) {
             startSteeringRight();
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
      if (steerLimitsEnabled) {
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
      } else {
        updateSteeringPosition();
      }
  }
}
