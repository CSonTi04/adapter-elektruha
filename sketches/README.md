# Tutorial Sketch Curriculum

This folder contains a step-by-step Arduino/ESP32 tutorial curriculum that
breaks down the main [`anxious.ino`](../anxious.ino) project into beginner-friendly sub-sketches.

---

## 🆕 Never programmed before?  Start here!

**Open sketch `00_language_concepts/v1_reference/v1_reference.ino` first.**

It is a guided, runnable tour of every programming concept used in this
curriculum — variables, loops, functions, structs, enums, and more — all
explained in plain English with no assumed background.

Each v1 sketch in the curriculum also contains `// LEARN:` comment blocks
that explain new language features exactly where they first appear in real
project code.

### Plain-English mini-glossary

| Term | What it means in plain English |
|------|-------------------------------|
| **Variable** | A named box that holds a value you can change later: `int score = 0;` |
| **Constant** | A named box whose value is locked forever: `constexpr int PIN = 2;` |
| **Type** | The kind of value a variable holds. `int` = whole number, `float` = decimal, `bool` = true/false, `char` = single letter |
| **Function** | A named block of instructions you can run (call) from anywhere in your code |
| **`setup()`** | Runs once when the board powers on — use it for one-time initialisation |
| **`loop()`** | Runs over and over forever — this is where your main logic lives |
| **`if` statement** | Runs a block of code only when a condition is true |
| **`for` loop** | Repeats a block of code a fixed number of times, with a counter |
| **`while` loop** | Repeats a block of code as long as a condition stays true |
| **`return`** | Exits a function, optionally handing a value back to the caller |
| **`struct`** | A bundle of related variables grouped under one name (like a form with fields) |
| **`enum class`** | A type that can only hold one of a named list of choices (e.g. Idle/Excited/Anxious) |
| **`#define`** | Tells the compiler to replace a name with a value before compiling (text substitution) |
| **`#include`** | Imports a library of pre-written functions into your sketch |
| **`millis()`** | Returns milliseconds since power-on — the foundation of non-blocking timing |
| **`Serial.println()`** | Prints a line of text to the Serial Monitor in the Arduino IDE |
| **`%` (modulo)** | The remainder after division: `7 % 3 = 1`, `10 % 5 = 0` |
| **`?:` (ternary)** | Compact if/else in one expression: `condition ? if_true : if_false` |
| **`&&` / `\|\|` / `!`** | Logical AND / OR / NOT — combining or flipping conditions |
| **`==` vs `=`** | `==` compares two values (is equal?); `=` stores a value (assign) |

---

## Why separate folders?

Arduino compiles **all `.ino` files in the same folder** together as one
program.  Each sketch in this curriculum must live in its own folder (named
identically to the `.ino` file) so they compile independently.

```
sketches/
  00_language_concepts/
    v1_reference/
      v1_reference.ino     ← start here if you're new to coding
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

### 00 · Language concepts  ← **Start here if you're new to coding**

A complete, runnable language tour.  Open it, upload it, and read the comments
alongside the Serial Monitor output.

- `v1_reference` – covers comments, variables, types, constants, operators,
  if/else, for loops, while loops, functions, arrays, ternary operator,
  structs, enums, #include, and the setup()/loop() Arduino model.

**Wiring:** none.

---

### 01 · Heartbeat with millis()

**Concept:** Non-blocking timing — the most important ESP32 habit.

**New language features introduced (see `// LEARN:` comments in v1):**
- Constants (`constexpr`) and integer types (`uint32_t`)
- Variables and assignment (`=` vs `==`)
- `setup()` and `loop()` — the Arduino two-function model
- Calling a function that returns a value (`millis()`)
- `if`-statement and comparison operators (`>=`)

- `v1_serial` – prints "BEAT" every 500 ms, no hardware.
- `v2_builtin_led` – blinks the built-in LED (GPIO2) with millis().
- `v3_full_pre_i2c` – adds Serial commands to switch between slow/fast beat.

**Wiring (v2/v3):** none beyond the ESP32 board.

---

### 02 · Verbose macros

**Concept:** Compile-time logging levels (`VERBOSE_LEVEL`) used in anxious.ino.

**New language features introduced (see `// LEARN:` comments in v1):**
- `#define` — preprocessor text substitution
- `#if / #else / #endif` — compile-time conditionals
- Early `return` from `loop()`

- `v1_serial` – shows all four levels in Serial; change `VERBOSE_LEVEL` and re-upload.
- `v2_builtin_led` – LED blink speed reflects the active verbosity level.
- `v3_full_pre_i2c` – adds a `MoodState` enum and state-transition logging.

**Wiring (v2/v3):** none.

---

### 03 · Touch smoothing

**Concept:** Capacitive touch with exponential smoothing and hysteresis.

**New language features introduced (see `// LEARN:` comments in v1):**
- `float` — decimal numbers and the `f` suffix
- `bool` — true/false flag values
- Type cast `(float)x`
- Functions with parameters and a return value
- `%` modulo operator
- Ternary `? :` operator

- `v1_simulated` – synthetic touch signal, shows algorithm without hardware.
- `v2_touch_led` – real GPIO4 touch + built-in LED lights when touched.
- `v3_full_pre_i2c` – adds rising-edge event counting as in anxious.ino.

**Wiring (v2/v3):**
- Wire or metal pad → **GPIO4** (touch electrode)
- Built-in LED feedback on **GPIO2**

> **Note:** GPIO4 (T0) is used to match anxious.ino.  The ESP32 supports
> touch sensing on several GPIOs.  **Consult the ESP32 Technical Reference
> Manual** (Table "Touch Sensor Channels") for the complete list before
> changing the touch pin.

---

### 04A · I²C scanner

**Concept:** Scanning the I²C bus before driving any devices.

**New language features introduced (see `// LEARN:` comments in v2):**
- `#include` — importing a library
- `int` type and signed vs unsigned integers
- `for` loop with `uint8_t` counter
- Hexadecimal literals (`0x08`, `0x77`)

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
> This matches anxious.ino (`pcfWritePin(pin, true)` → `pcf.write(pin, LOW)`).

---

### 05 · Emotions and decay

**Concept:** The continuous emotional model (arousal / affection / anxiety).

**New language features introduced (see `// LEARN:` comments in v1):**
- Inline (one-liner) function definition
- Compound assignment (update variable in-place)
- Delta-time (`dt`) for frame-rate-independent updates
- `while` loop and `Serial.available()`
- Chained assignment (`a = b = c = 0`)

- `v1_simulated` – Serial commands 't', 'b', 'r' to inject events.
- `v2_touch_based` – real GPIO4 touch drives affection; decays naturally.
- `v3_full_pre_i2c` – full Emotion struct + fake BLE events + bar graphs.

**Wiring (v2/v3):** wire/pad → GPIO4; built-in LED on GPIO2.

---

### 06 · Choose-state FSM

**Concept:** Priority-based finite state machine selecting among four moods.

**New language features introduced (see `// LEARN:` comments in v1):**
- `enum class` — named set of choices
- `const char*` — text / string return type
- `switch / case` — multi-way branching
- Comparing enum values with `!=`

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
- `v3_task_scan` – FreeRTOS task-based scan (non-blocking); matches anxious.ino.

**Wiring:** just the ESP32 board (built-in BLE).

---

### 11 · Anxius-lite integrator

**Concept:** Complete mini-version of anxious.ino with all subsystems.

- `v1_serial` – all logic in Serial; no hardware at all.
- `v2_minimal` – touch + built-in LED + LEDC audio; no PCF8574 or BLE.
- `v3_full` – touch + PCF8574 LEDs + BLE task + LEDC audio (binary mode).

**Wiring (v3):** full project wiring (touch GPIO4, PCF8574, GPIO5 audio).

---

### 12 · Anxius-lite PWM upgrade

**Concept:** Add sliced-PWM mode to the full integrator.

- `v3_pwm_upgrade` – complete anxious.ino-like sketch with `ENABLE_PWM` toggle
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
