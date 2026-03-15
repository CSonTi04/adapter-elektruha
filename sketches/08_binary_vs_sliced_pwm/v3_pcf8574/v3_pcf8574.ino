// =============================================================================
// 08_binary_vs_sliced_pwm – Variant 3: PCF8574 slicing
// =============================================================================
// Concept: Binary vs sliced-PWM mode on all 8 PCF8574 outputs.
//
// This mirrors the computeMask() / computeMaskBinary() output stage from
// anxious.ino.  A brightness ramp runs on all 8 channels simultaneously
// so you can see the visual difference between both modes on real LEDs.
//
// PREREQUISITE: Run 04A v3 first to confirm PCF8574 at 0x20.
//
// Wiring:
//   SDA → GPIO21  (4.7 kΩ pull-up)
//   SCL → GPIO22  (4.7 kΩ pull-up)
//   PCF8574 P0–P7 → 220 Ω → LED active-low
//   PCF8574 A0/A1/A2 → GND (0x20)
//
// Library: "PCF8574" by Renzo Mischianti
// =============================================================================

#include <Wire.h>
#include <PCF8574.h>
#include <math.h>

// ---- Tuning knobs -----------------------------------------------------------
constexpr int SDA_PIN = 21;
constexpr int SCL_PIN = 22;
constexpr uint8_t PCF_ADDR = 0x20;
constexpr bool    ENABLE_PWM       = false;  // <<< TOGGLE THIS
constexpr float   BINARY_THRESHOLD = 0.22f;
constexpr uint8_t PWM_LEVELS       = 16;
constexpr uint32_t PWM_SLICE_US    = 250;   // µs per slice
constexpr uint32_t RAMP_STEP_MS    = 25;    // brightness ramp speed
constexpr uint32_t BINARY_WRITE_MS = 8;     // binary flush interval
// -----------------------------------------------------------------------------

PCF8574 pcf(PCF_ADDR);

float    ledTarget[8];
uint8_t  pwmPhase    = 0;
uint32_t lastSliceUs = 0;
uint32_t lastWriteMs = 0;
uint32_t lastRampMs  = 0;
float    brightness  = 0.0f;
float    rampDir     = 0.01f;

float clamp01f(float x) { return x < 0 ? 0 : (x > 1 ? 1 : x); }

static inline void pcfWritePin(uint8_t pin, bool on) {
  pcf.write(pin, on ? LOW : HIGH);
}

uint8_t computeMask() {
  uint8_t m = 0;
  for (int i = 0; i < 8; i++) {
    float b = ledTarget[i] * ledTarget[i];
    int level = (int)lroundf(b * PWM_LEVELS);
    if (level > (int)PWM_LEVELS) level = PWM_LEVELS;
    if (pwmPhase < level) m |= (1 << i);
  }
  return m;
}

uint8_t computeMaskBinary() {
  uint8_t m = 0;
  for (int i = 0; i < 8; i++) {
    float b = ledTarget[i] * ledTarget[i];
    if (b >= BINARY_THRESHOLD) m |= (1 << i);
  }
  return m;
}

void applyMask(uint8_t m) {
  for (int i = 0; i < 8; i++) pcfWritePin(i, (m >> i) & 1);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  pcf.begin();
  for (int i = 0; i < 8; i++) pcf.write(i, HIGH);
  Serial.print("08_binary_vs_sliced_pwm v3 – PCF8574  ENABLE_PWM=");
  Serial.println(ENABLE_PWM ? "true" : "false");
}

void loop() {
  uint32_t nowUs = micros();
  uint32_t nowMs = millis();

  // Ramp brightness
  if (nowMs - lastRampMs >= RAMP_STEP_MS) {
    lastRampMs = nowMs;
    brightness += rampDir;
    if (brightness >= 1.0f) { brightness = 1.0f; rampDir = -rampDir; }
    if (brightness <= 0.0f) { brightness = 0.0f; rampDir = -rampDir; }
    for (int i = 0; i < 8; i++) ledTarget[i] = brightness;
  }

  if (ENABLE_PWM) {
    if (nowUs - lastSliceUs >= PWM_SLICE_US) {
      lastSliceUs = nowUs;
      pwmPhase = (pwmPhase + 1) % PWM_LEVELS;
      applyMask(computeMask());
    }
  } else {
    if (nowMs - lastWriteMs >= BINARY_WRITE_MS) {
      lastWriteMs = nowMs;
      applyMask(computeMaskBinary());
    }
  }
}
