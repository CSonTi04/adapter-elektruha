# Tutorial Sketch Curriculum

This folder contains a step-by-step Arduino/ESP32 tutorial curriculum that
breaks down the main [`anxius.ino`](../anxius.ino) project into beginner-friendly sub-sketches.

---

## Why separate folders?

Arduino compiles **all `.ino` files in the same folder** together as one
program.  Each sketch in this curriculum must live in its own folder (named
identically to the `.ino` file) so they compile independently.

```
sketches/
  01_heartbeat_millis/
    v1_serial/
      v1_serial.ino        ← opens with Arduino IDE / arduino-cli
    v2_builtin_led/
      v2_builtin_led.ino
    v3_full_pre_i2c/
      v3_full_pre_i2c.ino
  ...
```

---

## Three variants per sketch

Most sketches come in three variants.  Pick the one that matches your hardware:

| Variant | Hardware required | What you need |
|---------|-------------------|---------------|
| **v1** (Serial-only) | **None** | Just a USB cable and Serial Monitor |
| **v2** (Minimal) | ESP32 board only | Built-in LED, optional touch wire |
| **v3** (Full build) | Complete project | ESP32 + PCF8574 + LEDs + touch + audio + BLE |

---

## How to open a sketch

### Arduino IDE
1. **File → Open** (or double-click the `.ino` file).
2. Select the correct board: **Tools → Board → esp32 → ESP32 Dev Module**.
3. Upload.

### arduino-cli

```powershell
arduino-cli compile --fqbn esp32:esp32:esp32 sketches/01_heartbeat_millis/v1_serial
arduino-cli upload  -p COM5 --fqbn esp32:esp32:esp32 sketches/01_heartbeat_millis/v1_serial
```

---

## Curriculum overview

### 01 · Heartbeat with millis()

**Concept:** Non-blocking timing — the most important ESP32 habit.

- `v1_serial` – prints "BEAT" every 500 ms, no hardware.
- `v2_builtin_led` – blinks the built-in LED (GPIO2) with millis().
- `v3_full_pre_i2c` – adds Serial commands to switch between slow/fast beat.

**Wiring (v2/v3):** none beyond the ESP32 board.

---

### 02 · Verbose macros

**Concept:** Compile-time logging levels (`VERBOSE_LEVEL`) used in anxius.ino.

- `v1_serial` – shows all four levels in Serial; change `VERBOSE_LEVEL` and re-upload.
- `v2_builtin_led` – LED blink speed reflects the active verbosity level.
- `v3_full_pre_i2c` – adds a `MoodState` enum and state-transition logging.

**Wiring (v2/v3):** none.

---

### 03 · Touch smoothing

**Concept:** Capacitive touch with exponential smoothing and hysteresis.

- `v1_simulated` – synthetic touch signal, shows algorithm without hardware.
- `v2_touch_led` – real GPIO4 touch + built-in LED lights when touched.
- `v3_full_pre_i2c` – adds rising-edge event counting as in anxius.ino.

**Wiring (v2/v3):**
- Wire or metal pad → **GPIO4** (touch electrode)
- Built-in LED feedback on **GPIO2**

> **Note:** GPIO4 (T0) is used to match anxius.ino.  The ESP32 supports
> touch sensing on several GPIOs.  **Consult the ESP32 Technical Reference
> Manual** (Table "Touch Sensor Channels") for the complete list before
> changing the touch pin.

---

### 04A · I²C scanner

**Concept:** Scanning the I²C bus before driving any devices.

- `v1_explained` – no hardware; prints a full protocol explanation in Serial.
- `v2_scanner` – real scanner; reports all addresses found on SDA=21/SCL=22.
- `v3_full_scanner` – same scanner plus a readiness check for PCF8574 at 0x20.

**Wiring (v2/v3):**
- SDA → **GPIO21** (4.7 kΩ pull-up to 3.3 V)
- SCL → **GPIO22** (4.7 kΩ pull-up to 3.3 V)

> **Important curriculum note:** Run the I²C scanner and confirm 0x20 appears
> **before** moving to 04B.  Driving PCF8574 outputs without a confirmed
> address is a common debugging dead-end.

---

### 04B · PCF8574 outputs

**Concept:** Active-low LED driving through the I²C GPIO expander.

- `v1_virtual_mask` – no hardware; prints what the PCF8574 *would* do.
- `v2_skip_note` – minimal hardware note; initialises I²C but falls back to
  built-in LED blinking (PCF8574 not required for this variant).
- `v3_chase` – full build; 8-LED active-low chase pattern on PCF8574.

**Wiring (v3):**
- SDA → GPIO21 / SCL → GPIO22 (with pull-ups)
- PCF8574 A0/A1/A2 → GND (address **0x20**)
- PCF8574 P0–P7 → 220 Ω resistor → LED anode → LED cathode → GND

> **Active-low:** writing `LOW` to a PCF8574 pin turns the LED **on**.
> This matches anxius.ino (`pcfWritePin(pin, true)` → `pcf.write(pin, LOW)`).

---

### 05 · Emotions and decay

**Concept:** The continuous emotional model (arousal / affection / anxiety).

- `v1_simulated` – Serial commands 't', 'b', 'r' to inject events.
- `v2_touch_based` – real GPIO4 touch drives affection; decays naturally.
- `v3_full_pre_i2c` – full Emotion struct + fake BLE events + bar graphs.

**Wiring (v2/v3):** wire/pad → GPIO4; built-in LED on GPIO2.

---

### 06 · Choose-state FSM

**Concept:** Priority-based finite state machine selecting among four moods.

- `v1_serial` – Serial commands drive state changes; watch transitions.
- `v2_touch_led` – real touch + distinct LED blink patterns per state.
- `v3_full_pre_i2c` – complete FSM + emotions + fake BLE, no I²C.

**Wiring (v2/v3):** wire/pad → GPIO4; built-in LED on GPIO2.

---

### 07 · Energy buffer

**Concept:** 8-channel transient energy buffer that decays over time.

- `v1_serial_bars` – ASCII bar graph of all 8 channels, no hardware.
- `v2_builtin_led` – total energy mapped to LED blink rate.
- `v3_pcf8574` – energy buffer flushed to PCF8574 in binary ON/OFF mode.

**Wiring (v3):** full PCF8574 wiring (see 04B).

---

### 08 · Binary vs sliced PWM

**Concept:** Two visual output modes; toggle `ENABLE_PWM` to compare.

- `v1_serial` – prints what each mode *would* do for a range of brightness values.
- `v2_builtin_led` – brightness ramp on the built-in LED; snap vs smooth.
- `v3_pcf8574` – ramp on all 8 PCF8574 LEDs simultaneously.

**Wiring (v3):** full PCF8574 wiring.

---

### 09 · Audio events

**Concept:** Event-based tone scheduler (not continuous synthesis).

- `v1_serial_scheduler` – prints scheduler decisions; Serial commands change state.
- `v2_ledc_gpio5` – real LEDC tones on GPIO5; auto-cycles through states.
- `v3_full_lm386` – complete per-state profiles + global volume scale trim.

**Wiring (v2/v3):**
- GPIO5 → speaker/piezo for basic testing
- GPIO5 → 10 µF coupling cap → LM386 pin 3 for proper amplification

---

### 10 · BLE device count

**Concept:** BLE scanning as an environmental sensor for arousal.

- `v1_simulated` – random-walk fake device count; no BLE hardware needed.
- `v2_blocking_scan` – real BLE scan; **warning:** blocks loop() during scan.
- `v3_task_scan` – FreeRTOS task-based scan (non-blocking); matches anxius.ino.

**Wiring:** just the ESP32 board (built-in BLE).

---

### 11 · Anxius-lite integrator

**Concept:** Complete mini-version of anxius.ino with all subsystems.

- `v1_serial` – all logic in Serial; no hardware at all.
- `v2_minimal` – touch + built-in LED + LEDC audio; no PCF8574 or BLE.
- `v3_full` – touch + PCF8574 LEDs + BLE task + LEDC audio (binary mode).

**Wiring (v3):** full project wiring (touch GPIO4, PCF8574, GPIO5 audio).

---

### 12 · Anxius-lite PWM upgrade

**Concept:** Add sliced-PWM mode to the full integrator.

- `v3_pwm_upgrade` – complete anxius.ino-like sketch with `ENABLE_PWM` toggle
  and the full state-machine LED pattern engine (idle/excited/anxious/friendly).
  Toggle `ENABLE_PWM` at the top to switch between binary and sliced modes.

**Wiring:** same as sketch 11 v3.

---

## Wiring quick-reference (full build)

| Signal | ESP32 GPIO | Notes |
|--------|-----------|-------|
| Touch electrode | GPIO4 | capacitive touch (T0) |
| I²C SDA | GPIO21 | 4.7 kΩ pull-up to 3.3 V |
| I²C SCL | GPIO22 | 4.7 kΩ pull-up to 3.3 V |
| PCF8574 address | A0/A1/A2 → GND | default 0x20 |
| PCF8574 P0–P7 | → 220 Ω → LED → GND | active-low |
| Audio output | GPIO5 | LEDC tone → LM386 input |
| Built-in LED | GPIO2 | on DOIT DevKit boards |

---

## Libraries required for full build sketches

| Library | Where to install |
|---------|-----------------|
| PCF8574 by Renzo Mischianti | Arduino Library Manager |
| ESP32 BLE (built-in) | Included with ESP32 Arduino core |

```powershell
# Install external library
arduino-cli lib install PCF8574

# Install ESP32 core if needed
arduino-cli core install esp32:esp32
```

---

## ESP32 Arduino core API note

Sketches in this curriculum use the **ESP32 Arduino core 3.x** LEDC API:
- `ledcAttachChannel(pin, freq, bits, channel)`
- `ledcWriteTone(pin, freq)`
- `ledcWrite(pin, duty)`

If you are using core 2.x, the LEDC functions have a different signature.
Check your installed core version with `arduino-cli core list`.
