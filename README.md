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

## Motor safety controls (DC steering / 6V drive conversion)

**This section supersedes the older auto-centering descriptions and `C` example above.** No steering position, limit, or motor-current feedback is implemented. The firmware cannot detect a stalled motor or prove that a hard stop has been reached.

- **Settings → Drive:** separate **Forward** and **Reverse** speed controls, each 20–100%. The cockpit speed control changes forward only. Both L298N directions use the same existing duty ceiling and acceleration ramp, through one output path; there is no high-power startup boost. Changing a setting while driving updates the active direction without changing the other direction's saved speed.
- Forward remains in NVS key `speed`; reverse uses `rev_spd`. On first boot after upgrading, reverse inherits the validated old forward setting and is saved independently. Invalid saved speeds fall back to defaults (forward 60%; reverse the validated forward value).
- **Settings → Steering:** **Maximum pulse**, 20–500 ms in 1 ms increments, default 100 ms; saved as `s_pulse`. This is elapsed on-time, not a PWM-scaled position estimate. The cutoff applies to **both directions and both driver modes**, including when estimated steering limits are disabled.
- Holding a button/stick or repeating left/right commands cannot restart or extend a pulse. Release / return to neutral (`stop_steer` or `t` for serial/BLE text) before pressing again. A direction change without neutral does not start another pulse. Stops/failsafes keep the input latched; an explicit release is required. Changing pulse duration stops the current pulse rather than extending it.
- The steering ramp is still respected. A ramp longer than the pulse can produce little movement; tune pulse length and steering ramp cautiously with the mechanism unloaded. A short pulse is not stall detection or a guarantee against mechanical damage.
- Auto-home and auto-center are **disabled without feedback**. `/auto_home` and `/center` return HTTP 409; equivalent serial/BLE commands stop steering and refuse automatic motion. `set_center` / `Z` only stops steering and resets the estimated reference; it never drives to a hard stop.

### Duty ceiling is not a regulated motor supply

The legacy `v_max / v_bat` setting computes an **L298N PWM duty ceiling**, not actual regulated motor voltage. Pulses still reach the battery voltage less driver losses. A 50% ceiling does **not** guarantee that a 6V motor is protected on a 12–13V battery, and the configured battery value is not measured. The UI no longer offers 7.5V/9V/12V boost presets. Use an appropriately rated supply, driver, fuse, and hardware protection. Legacy relay mode retains coarse 120 ms time slicing with independent directional percentages; it has no L298N duty governor/ramp and is not a regulated supply either.

### Timing and verification limits

Steering cutoff uses a one-shot **ESP timer task**, independent of the main loop's blocking audio, BLE scanning, HTTP, and serial work. A short recursive lock serializes steering GPIO/PWM writes and cutoff; it is not held across audio, networking, flash writes, or delays. If timer allocation/arming fails, steering stays off. Timer expiry stops outputs but does not rearm held input.

This is **not a hard-real-time hardware cutoff**: FreeRTOS scheduling, interrupt/flash stalls, driver calls, and physical relay release can add latency. The existing drive heartbeat/collision processing remains main-loop based and can be delayed by blocking work. With no sensors wired, collision logic provides no obstacle protection. USB-only compilation/tests cannot establish motor torque, current, L298N heating, mechanical travel, actual GPIO pulse width, or relay release timing; those require safely isolated hardware measurements before reconnecting the cart.

### API and host tests

`/set_speed?val=20..100`, `/set_reverse_speed?val=20..100`, and `/set_steer_pulse?val=20..500` require whole decimal integers. Missing, malformed, and out-of-range values return HTTP 400 without changing settings; failed writes return HTTP 500. `/get_config` and `/status` publish `speed`, `reverse_speed`, and `steer_pulse_ms`; the HUD synchronizes these independently.

Run without hardware or extra Python packages (requires Python 3, `g++`, and Node):

```bash
python -m unittest discover -s tests -v
```

The suite compiles actual extracted sketch functions/routes and the shared `MotorPolicy.h`, exercises migration, validation, direction-specific PWM/ramping and relay duty, elapsed cutoff/rollover, held input, release, timer failure, stop/collision command behavior, and runs the settings JavaScript in Node. Stubbed GPIO/NVS/timers verify software behavior, not the electrical hardware or scheduler's timing guarantees.

---

## 🛠️ Build & Flash Instructions

### Arduino CLI

Verified on Linux with Arduino CLI 1.4.1 and `esp32:esp32@3.3.12`, using an ESP32-S3 with 16 MB flash and 8 MB PSRAM (PSRAM remains disabled in this build).

Disconnect motor power before resetting or flashing the controller.

```bash
arduino-cli core update-index --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core install esp32:esp32@3.3.12 --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json

# Run from the project directory. Change PORT for your USB adapter.
PORT=/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0
FQBN=esp32:esp32:esp32s3:CDCOnBoot=cdc,FlashMode=dio,FlashSize=16M,PSRAM=disabled,PartitionScheme=app3M_fat9M_16MB
arduino-cli compile --jobs 4 --fqbn "$FQBN" .
arduino-cli upload --port "$PORT" --fqbn "$FQBN" --upload-property upload.speed=460800 .
```

The explicit partition scheme is required: the default application partition is too small with this core version. This scheme provides 3 MB application slots on the 16 MB flash. Changing partition layouts can invalidate existing filesystem/OTA data; back up flash first.

On Arch/Omarchy, install the CLI with `omarchy pkg add arduino-cli`. Serial access requires membership in `uucp` (`sudo usermod -aG uucp "$USER"`), followed by logout/login. To activate an already-added membership in a terminal without logging out, run `newgrp uucp` and execute the commands in that shell.

`CDCOnBoot=cdc` routes application `Serial` to native USB CDC, not the FTDI UART adapter. FTDI flashing still works; use the native USB port for application serial logs with this configuration.

---

## 📄 License

MIT License. Developed for the **TINK / InMoov Humanoid Robot Platform**.
