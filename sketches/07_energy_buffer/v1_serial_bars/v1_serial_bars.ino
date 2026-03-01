// =============================================================================
// 07_energy_buffer – Variant 1: Serial bars (no hardware)
// =============================================================================
// Concept: The 8-channel LED energy buffer from anxius.ino.
//
// Instead of working directly with LED brightness values, anxius.ino
// accumulates "energy" in a float buffer[8].  Each call to pulseLed() adds
// to a channel; decayEnergy() subtracts a little every frame.
// This creates organic fading without any hardware PWM.
//
// This variant prints the 8 channels as ASCII bar graphs in Serial.
// No hardware needed – watch the patterns scroll by.
//
// Wiring: none – open Serial Monitor at 115200 baud.
// =============================================================================

// ---- Tuning knobs -----------------------------------------------------------
constexpr float    DECAY_PER_SEC = 1.5f;  // how fast energy drains
constexpr float    PULSE_AMP     = 0.6f;  // amplitude of each pulse
constexpr float    TAIL_FRAC     = 0.45f; // neighbour bleed fraction
constexpr uint32_t STEP_MS       = 120;   // chase step interval
constexpr uint32_t PRINT_MS      = 100;
// LED groups (matches anxius.ino)
constexpr int LEDA_FIRST = 0, LEDA_LAST = 3;
constexpr int LEDB_FIRST = 4, LEDB_LAST = 7;
// -----------------------------------------------------------------------------

float energy[8] = {0};
uint32_t lastEnergyMs = 0;
uint32_t lastStepMs   = 0;
uint32_t lastPrintMs  = 0;
int chaseIdx = 0;

float clamp01f(float x) { return x < 0 ? 0 : (x > 1 ? 1 : x); }

void decayEnergy(uint32_t now) {
  if (lastEnergyMs == 0) { lastEnergyMs = now; return; }
  float dt = (now - lastEnergyMs) / 1000.0f;
  lastEnergyMs = now;
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

// Print 8 bars, one per channel
void printBars() {
  for (int i = 0; i < 8; i++) {
    Serial.print("LED"); Serial.print(i); Serial.print(" [");
    int bars = (int)(energy[i] * 20);
    for (int b = 0; b < 20; b++) Serial.print(b < bars ? "#" : " ");
    Serial.print("] "); Serial.println(energy[i], 2);
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  Serial.println("07_energy_buffer v1 – Serial bars");
}

void loop() {
  uint32_t now = millis();

  decayEnergy(now);

  // Advance chase every STEP_MS
  if (now - lastStepMs >= STEP_MS) {
    lastStepMs = now;
    pulseWithTail(chaseIdx, PULSE_AMP, TAIL_FRAC);
    chaseIdx = (chaseIdx + 1) % 8;
  }

  if (now - lastPrintMs >= PRINT_MS) {
    lastPrintMs = now;
    printBars();
  }
}
