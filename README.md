# TINK // InMoov Base Platform Controller

An advanced ESP32-S3 firmware and web HUD controller for the **TINK / InMoov Humanoid Robot Base Platform**. 

Featuring a standalone Wi-Fi Access Point (`TinkDrive`), captive portal auto-popup, Web Bluetooth & HTML5 Gamepad controller support, direct BLE Wireless Controller scanner, MAX98357A I2S digital audio output, onboard dynamic RGB status animations, and automatic disconnect safety failsafe.

---

## 🌟 Key Features

* **⚡ Standalone Access Point & Captive Portal:** Broadcasts Wi-Fi network `TinkDrive` (Password: `tink1234`). Automatically opens the cybernetic HUD interface upon connecting.
* **🎮 Controls & Keybindings:**
  * **Touch HUD:** Visual 3D steering wheel and interactive touch buttons.
  * **Desktop WASD & Arrow Keys:** Full keyboard support (`W`/`Up` = Forward, `S`/`Down` = Reverse, `A`/`Left` = Steer Left, `D`/`Right` = Steer Right, `Space`/`X` = Emergency Stop).
  * **HTML5 Gamepad API:** Plug & play support for PS4, PS5, Xbox, and Nintendo Switch controllers paired to phone/laptop.
* **📡 Direct BLE Wireless Controller Pairing:** ESP32-S3 functions as a BLE Central Scanner to directly discover and pair with Wireless Gamepads (Switch Pro Controller, Joy-Cons, 8BitDo, Xbox BLE) without needing a phone or PC in the loop.
* **🔊 MAX98357A I2S Digital Audio Output:** Uses ESP32 hardware I2S DMA engine (`driver/i2s.h`) to output digital sound effects, backup warning beeps, boot chimes, and calibration alerts.
* **🌈 Dynamic WS2812 RGB LED Status:**
  * **Idle Rainbow Fade:** Smooth RGB spectrum fade when no device is connected.
  * **Drive Indicators:** Solid Green for Forward, Blinking Red + Backup Beep for Reverse.
  * **Steering Indicators:** Yellow for Steer Left, Blue for Steer Right.
  * **Calibration:** Yellow/Blue strobe during auto-centering, 3x Cyan flash on Set Center.
* **🛑 Automatic Disconnect Failsafe:** 1-second Web & BLE heartbeat ping. If connection drops for > 3 seconds while driving, TINK executes an immediate **Emergency All Stop**!
* **🤖 MyRobotLab Integration:** Full Python command reference and Serial USB (115200 baud) pipe support.

---

## 🔌 ESP32-S3 Pinout & Wiring Specifications

> ⚠️ **IMPORTANT SAFETY NOTE FOR ESP32-S3 (S3-N16R8):** GPIO 26 through 32 are internally connected to Octal SPI Flash/PSRAM. Relay control pins are strictly assigned to safe GPIOs `4, 5, 6, 7`.

### Pinout Table

| ESP32-S3 Pin | Target Board / Component | Target Function |
| :--- | :--- | :--- |
| `GPIO 4` | Relay 1 (IN1) | Drive Motor Forward (6V) |
| `GPIO 5` | Relay 2 (IN2) | Drive Motor Backward (6V) |
| `GPIO 6` | Relay 3 (IN3) | Steering Actuator Left (12V) |
| `GPIO 7` | Relay 4 (IN4) | Steering Actuator Right (12V) |
| `GPIO 18` | MAX98357A Amp `DIN` | I2S Audio Serial Data Input |
| `GPIO 17` | MAX98357A Amp `BCLK` | I2S Audio Bit Clock |
| `GPIO 16` | MAX98357A Amp `LRC` | I2S Audio Word Select Clock |
| `VIN / 5V` | Relay VCC & Amp VIN | 5V Power Input |
| `GND` | System Common Ground | Common Ground Reference |

---

## 🛠️ Relay H-Bridge Schematics

```
                     +-----------------------+
                     | 4-Channel Relay Board |
                     +-----------------------+
                     | IN1  IN2  IN3  IN4    |
                     +---+----+----+----+----+
                         |    |    |    |
    GPIO 4 --------------+    |    |    |
    GPIO 5 -------------------+    |    |
    GPIO 6 ------------------------+    |
    GPIO 7 -----------------------------+

    [Drive Motor 6V H-Bridge]
    Relay 1 COM & Relay 2 COM  --> Drive Motor Terminals A & B
    Relay 1 & 2 NO Pins        --> +6V Power
    Relay 1 & 2 NC Pins        --> Ground

    [Steering Actuator 12V H-Bridge]
    Relay 3 COM & Relay 4 COM  --> Steering Actuator Terminals A & B
    Relay 3 & 4 NO Pins        --> +12V Power
    Relay 3 & 4 NC Pins        --> Ground
```

> ⚠️ **Common Ground Required:** ESP32 GND, Relay Board GND, MAX98357A GND, 6V Battery GND, and 12V Power Supply GND MUST be tied together. Ensure the **`VCC` <-> `JD-VCC`** jumper is plugged into the relay board.

---

## 🔊 MAX98357A Audio Amplifier Wiring

| MAX98357A Pin | ESP32-S3 Connection |
| :--- | :--- |
| `DIN` | `GPIO 18` |
| `BCLK` | `GPIO 17` |
| `LRC` / `LRCLK` | `GPIO 16` |
| `VIN` | `5V / VIN` |
| `GND` | `ESP32 GND` |
| `GAIN` | *Unconnected (Default 12dB Gain)* |
| `SD` | *Unconnected* |

---

## 🤖 MyRobotLab Integration

TINK accepts raw text commands over USB Serial (115200 baud) or BLE UART (`TINK_Base_Platform`).

### Python Integration Script

```python
# MyRobotLab Python Integration Script
serial = runtime.start("serial", "Serial")
serial.connect("COM9", 115200)

# Movement Control Commands:
serial.write("W\n") # Drive Forward
serial.write("S\n") # Drive Reverse
serial.write("A\n") # Steer Left
serial.write("D\n") # Steer Right
serial.write("X\n") # Emergency All Stop
serial.write("C\n") # Auto-Center Steering
```

---

## 🛠️ Build & Flash Instructions

### Arduino CLI

```bash
arduino-cli compile --upload -p COM9 --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc,FlashMode=dio,FlashSize=16M,PSRAM=disabled ESP32RemoteCarController.ino
```

---

## 📄 License

MIT License. Developed for the **TINK / InMoov Humanoid Robot Platform**.
