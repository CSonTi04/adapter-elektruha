// =============================================================================
// 07_energy_buffer – Variant 3: Energy → PCF8574 binary output
// =============================================================================
// Concept: Energy buffer flushed to PCF8574 as binary ON/OFF.
// This is the binary output path from anxious.ino applied to a simple
// chase-and-decay pattern.
//
// PREREQUISITE: Run 04A v3 first to confirm PCF8574 at 0x20.
//
// Wiring:
//   SDA → GPIO21  (4.7 kΩ pull-up to 3.3 V)
//   SCL → GPIO22  (4.7 kΩ pull-up to 3.3 V)
//   PCF8574 P0–P7 → 220 Ω → LED+ → LED- → GND  (active-low)
//   PCF8574 A0/A1/A2 → GND  (address 0x20)
//
// Library: "PCF8574" by Renzo Mischianti
// =============================================================================

#include <Wire.h>
#include <PCF8574.h>

// ---- Tuning knobs -----------------------------------------------------------
constexpr int SDA_PIN = 21;
constexpr int SCL_PIN = 22;
constexpr uint8_t PCF_ADDR = 0x20;
constexpr float BINARY_THRESHOLD = 0.22f; // LED ON if energy >= this value
constexpr float DECAY_PER_SEC   = 1.2f;
constexpr float PULSE_AMP       = 0.55f;
constexpr float TAIL_FRAC       = 0.45f;
constexpr uint32_t STEP_MS      = 130;
constexpr uint32_t WRITE_MS     = 8;  // I2C flush interval
constexpr int LEDA_FIRST = 0, LEDA_LAST = 3;
constexpr int LEDB_FIRST = 4, LEDB_LAST = 7;
// -----------------------------------------------------------------------------

PCF8574 pcf(PCF_ADDR);

float energy[8]    = {0};
float ledTarget[8] = {0};
uint32_t lastEnergyMs = 0;
uint32_t lastStepMs   = 0;
uint32_t lastWriteMs  = 0;
int chaseIdx = 0;

float clamp01f(float x) { return x < 0 ? 0 : (x > 1 ? 1 : x); }

static inline void pcfWritePin(uint8_t pin, bool on) {
  pcf.write(pin, on ? LOW : HIGH);  // active-low
}

void decayEnergy(uint32_t now) {
  if (lastEnergyMs == 0) { lastEnergyMs = now; return; }
  float dt = (now - lastEnergyMs) / 1000.0f; lastEnergyMs = now;
  for (int i = 0; i < 8; i++) energy[i] = clamp01f(energy[i] - DECAY_PER_SEC * dt);
}

void pulseLed(int idx, float amp) {
  if (idx < 0 || idx > 7) return;
  energy[idx] = clamp01f(energy[idx] + amp);
}

void pulseWithTail(int idx, float amp, float tail) {
  pulseLed(idx, amp);
  if (idx >= LEDA_FIRST && idx <= LEDA_LAST) {
    if (idx - 1 >= LEDA_FIRST) pulseLed(idx - 1, amp * tail);
    if (idx + 1 <= LEDA_LAST)  pulseLed(idx + 1, amp * tail);
  } else {
    if (idx - 1 >= LEDB_FIRST) pulseLed(idx - 1, amp * tail);
    if (idx + 1 <= LEDB_LAST)  pulseLed(idx + 1, amp * tail);
  }
}

// Binary mask: on if energy^2 >= threshold
uint8_t computeMaskBinary() {
  uint8_t m = 0;
  for (int i = 0; i < 8; i++) {
    float b = energy[i] * energy[i]; // gamma shaping
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
  for (int i = 0; i < 8; i++) pcf.write(i, HIGH); // all off
  Serial.println("07_energy_buffer v3 – PCF8574 binary output");
}

void loop() {
  uint32_t now = millis();
  decayEnergy(now);

  if (now - lastStepMs >= STEP_MS) {
    lastStepMs = now;
    pulseWithTail(chaseIdx, PULSE_AMP, TAIL_FRAC);
    chaseIdx = (chaseIdx + 1) % 8;
  }

  if (now - lastWriteMs >= WRITE_MS) {
    lastWriteMs = now;
    applyMask(computeMaskBinary());
  }
}
