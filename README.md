# anxius.ino

## Overview

This sketch runs an ESP32-based emotional LED sculpture built around a PCF8574 I²C GPIO expander (8 outputs) and a single capacitive touch input. It continuously maintains a small internal emotional model: **arousal**, **anxiety**, and **affection**.

Those values are transformed into living, biological-looking behavior across two LED hemispheres (channels **0–3** and **4–7**) and a matching procedural audio layer.

The result is reactive rather than scripted: the system calms down over time, becomes more animated when nearby BLE activity increases, and enters a distinct friendly response sequence after touch.

## Hardware / Libraries

- **ESP32**
- **PCF8574** at `0x20` driving 8 LED channels (active-low outputs)
- **Capacitive touch** on `GPIO 4` via `touchRead`
- **BLE scanning** with ESP32 BLE stack (`BLEDevice`, `BLEScan`)
- **Audio output** on `GPIO5` for LM386 input (LEDC tone generation)
- I²C pins: `SDA=21`, `SCL=22`

## Emotional model

The sculpture tracks three continuous values in the range 0..1:

- **Arousal**: increases when more BLE devices are detected nearby; slowly decays over time.
- **Affection**: increases on touch; slowly decays over time.
- **Anxiety**: trends upward slowly, but is reduced by affection and strongly reduced when touched.

A priority-based selector chooses one of four modes:

- **Friendly**: briefly after touch (highest priority)
- **Anxious**: when anxiety is high
- **Excited**: when arousal is high
- **Idle**: default baseline behavior

## Light behavior (patterns)

LED channels are split into two readable groups:

- **Group A (impulse):** LED `0–3`
- **Group B (response):** LED `4–7`

Each state has a dedicated pattern generator:

- **Idle:** slow propagation in Group A, then echo in Group B, with rare subtle misfires.
- **Excited:** fast opposing chases, micro-flicker, and short overload flashes when arousal is high.
- **Anxious:** twitchy irregular spikes mostly in Group A, occasional Group B misfires, and rare full-body stress flashes.
- **Friendly:** a multi-phase relief wave after touch: A fills, B fills, alternating paired pulses, short unified glow, then release to calm.

Patterns inject energy into an 8-channel buffer (`ledEnergy`) that naturally decays. This makes motion feel organic even in binary output mode.

## Output modes

Two visual output styles are supported:

- **Binary mode (default):** ON/OFF LEDs using a threshold for clearer state readability.
- **Time-sliced PWM mode:** simulated brightness via rapid slices for smoother gradients.

Toggle with:

- `constexpr bool ENABLE_PWM = false;`

## Audio behavior

Audio is generated as short **events** (tiny tone bursts), not as a continuous synthesized waveform. This gives the output an organic, nervous character that matches the LED behavior.

- **Output path:** ESP32 LEDC on `AUDIO_PIN` (`GPIO5`) into LM386 input.
- **Enable/disable:** `constexpr bool ENABLE_AUDIO = true;`
- **Core functions:**
  - `audioSilence()` sets duty to `0`,
  - `audioTone(hz, duty)` sets frequency with `ledcWriteTone()` and loudness via duty,
  - `audioTick()` schedules and updates events.

### How scheduling works

`audioTick()` tracks:

- `audioNextEventMs` (when next sound event may start),
- `audioEventEndMs` (when current event stops),
- `audioFreqHz` and `audioDuty` (current event parameters).

Behavior per call:

1. If event is still active: keep tone running with tiny drift (`±0.6%`) to avoid robotic steadiness.
2. If between events: stay silent.
3. If time for a new event: choose frequency, duty, duration, and next-gap from current emotional state.

### State-specific sound profiles

- **Idle:** low-frequency, sparse hesitant pulses.
- **Excited:** faster/higher chirps; occasional short high-energy burst.
- **Anxious:** irregular ticks, sharp misfires, and rare brief buzz-like tremor.
- **Friendly:** descending, calming contour after touch.

### Audio tuning knobs

- `ENABLE_AUDIO` — global audio on/off.
- `AUDIO_LEDC_RES_BITS` — duty resolution (current: 8-bit).
- Per-state event durations/gaps/frequency ranges inside `audioTick()`.

### ESP32 core compatibility note

The sketch uses ESP32 Arduino core 3.x LEDC API style (`ledcAttachChannel`, `ledcWriteTone`, `ledcWrite`).

## BLE scanning (non-blocking)

BLE scanning runs in its own FreeRTOS task pinned to the other core. This keeps LED/audio updates responsive while scans are running. Each completed scan updates shared device-count state, and the main loop uses that count to bump arousal.

## Serial verbosity

Logging is controlled by `VERBOSE_LEVEL`:

- `VERBOSE_NONE`: silent
- `VERBOSE_STATE`: state transitions
- `VERBOSE_EMOTION`: periodic emotion/state telemetry + BLE counts
- `VERBOSE_FULL`: extra touch/debug logs

## Code walkthrough

### 1) Core data model

- `Emotion E` is the central state container:
  - `arousal`: rises with BLE activity, decays over time,
  - `anxiety`: rises slowly unless touch/affection reduce it,
  - `affection`: boosted by touch, slowly fades,
  - `lastTouchMs` and `lastSeenDevices` provide short-term context.

### 2) Input processing

- **Touch**:
  - `calibrateTouch()` samples 50 readings on startup to establish a baseline,
  - `isTouched()` applies exponential smoothing (`alpha = 0.15`) and threshold detection.
- **BLE**:
  - `bleScanTask()` runs blocking BLE scans on a separate FreeRTOS task,
  - results are written to shared globals with a critical section,
  - `loop()` reads latest results non-blocking via `getBleDeviceCount()`.

### 3) State selection

- `chooseState(nowMs)` applies simple priority logic:
  1. Recent touch => `Friendly`
  2. High anxiety => `Anxious`
  3. High arousal => `Excited`
  4. Otherwise => `Idle`

This keeps transitions interpretable and easy to tune.

### 4) LED engine

- `ledEnergy[8]` stores per-channel transient energy.
- Pattern functions (`patternIdle`, `patternExcited`, `patternAnxious`, `patternFriendly`) add pulses/tails and decay.
- `copyEnergyToTarget()` maps transient energy into `ledTarget[8]`.
- Output stage:
  - `computeMask()` for sliced PWM mode,
  - `computeMaskBinary()` for thresholded ON/OFF mode,
  - `applyMask()` writes to PCF8574 outputs.

### 5) Audio engine

- `audioTick()` schedules short tone events with jitter (not continuous wave synthesis).
- Every state has its own event profile:
  - `Idle`: low, sparse pulses,
  - `Excited`: denser, higher chirps + occasional bursts,
  - `Anxious`: irregular ticks/misfires,
  - `Friendly`: descending relief-like contour.
- Uses ESP32 3.x LEDC calls: `ledcAttachChannel`, `ledcWriteTone`, `ledcWrite`.

### 6) Main loop timeline

On each `loop()` iteration:
1. Read touch edge events and apply immediate emotional effects.
2. Run periodic emotion decay/update (`EMOTION_TICK_MS`).
3. Ingest latest BLE scan result (if a fresh one exists).
4. Choose current `MoodState`.
5. Generate LED pattern for that state.
6. Generate/sustain state-driven audio event.
7. Emit logs (based on `VERBOSE_LEVEL`).
8. Flush LED outputs (PWM slices or binary update interval).

## Tuning quick-reference

Most useful knobs in `anxius.ino`:

- `ENABLE_PWM`, `BINARY_THRESHOLD`, `BINARY_WRITE_EVERY_MS` (visual style)
- `ENABLE_AUDIO`, `AUDIO_LEDC_RES_BITS` (audio behavior)
- `BLE_SCAN_EVERY_MS`, `BLE_SCAN_FOR_MS` (environment sensitivity)
- `EMOTION_TICK_MS` and decay rates inside `tickEmotions()` (responsiveness)
- thresholds in `chooseState()` (state switching personality)
- `VERBOSE_LEVEL` (debug output volume)

## Hardware / wiring assumptions

- Board: ESP32 (target used: `esp32:esp32:esp32`)
- Touch input: `TOUCH_GPIO = 4` (`touchRead`)
- I2C: `SDA=21`, `SCL=22`
- PCF8574 address: `0x20` (8 LED channels)
- Audio pin: `GPIO5` to LM386 input (through coupling cap recommended)

## Dependencies

- ESP32 Arduino core (3.x)
- Built-in ESP32 BLE library (`BLEDevice.h`, `BLEScan.h`)
- `PCF8574` library

Install the external library if needed:

```powershell
arduino-cli lib install PCF8574
```

## Build with arduino-cli

### 1) Verify setup

```powershell
arduino-cli version
arduino-cli core install esp32:esp32
arduino-cli board listall esp32:esp32
```

### 2) Compile

From repo root:

```powershell
arduino-cli compile --fqbn esp32:esp32:esp32 .
```

If `arduino-cli` is not in `PATH`, use Arduino IDE bundled CLI:

```powershell
& "C:\Program Files\arduino-ide\resources\app\lib\backend\resources\arduino-cli.exe" compile --fqbn esp32:esp32:esp32 .
```

## Upload with arduino-cli

### 1) Find serial port

```powershell
arduino-cli board list
```

### 2) Upload (replace COM port)

```powershell
arduino-cli upload -p COM5 --fqbn esp32:esp32:esp32 .
```

Bundled CLI variant:

```powershell
& "C:\Program Files\arduino-ide\resources\app\lib\backend\resources\arduino-cli.exe" upload -p COM5 --fqbn esp32:esp32:esp32 .
```

## Notes

- This workspace also includes `adapter-elektruha.ino`. Arduino compiles all `.ino` files in the sketch folder, so ensure there is only one active `setup()`/`loop()` pair when building.
- For ESP32 core 3.x, `anxius.ino` uses the newer LEDC API style (`ledcAttachChannel`, `ledcWriteTone`, `ledcWrite`).
- `audioTick()` currently takes a raw `uint8_t` state argument and casts to `MoodState` internally to avoid Arduino auto-generated prototype ordering issues in `.ino` builds.
