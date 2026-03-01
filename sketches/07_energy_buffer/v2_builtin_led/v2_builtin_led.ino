// =============================================================================
// 07_energy_buffer – Variant 2: Built-in LED (minimal hardware)
// =============================================================================
// Concept: Energy buffer mapped to the built-in LED.
// The built-in LED cannot show 8 channels, so we map the *sum* of all
// 8 channels to a blink rate: more total energy = faster blinking.
//
// Wiring: just the ESP32 board – built-in LED on GPIO2.
// =============================================================================

// ---- Tuning knobs -----------------------------------------------------------
constexpr int      LED_PIN      = 2;
constexpr float    DECAY_PER_SEC = 0.8f;
constexpr float    PULSE_AMP    = 0.5f;
constexpr float    TAIL_FRAC    = 0.40f;
constexpr uint32_t STEP_MS      = 200;   // pulse injection interval
constexpr uint32_t MIN_BLINK_MS = 80;    // fastest blink (max energy)
constexpr uint32_t MAX_BLINK_MS = 800;   // slowest blink (min energy)
constexpr int LEDA_FIRST = 0, LEDA_LAST = 3;
constexpr int LEDB_FIRST = 4, LEDB_LAST = 7;
// -----------------------------------------------------------------------------

float energy[8] = {0};
uint32_t lastEnergyMs = 0;
uint32_t lastStepMs   = 0;
uint32_t lastBlinkMs  = 0;
bool     ledState     = false;
int      chaseIdx     = 0;

float clamp01f(float x) { return x < 0 ? 0 : (x > 1 ? 1 : x); }

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

float totalEnergy() {
  float s = 0;
  for (int i = 0; i < 8; i++) s += energy[i];
  return s / 8.0f; // normalised 0..1
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  Serial.println("07_energy_buffer v2 – built-in LED (blink rate = energy)");
}

void loop() {
  uint32_t now = millis();
  decayEnergy(now);

  if (now - lastStepMs >= STEP_MS) {
    lastStepMs = now;
    pulseWithTail(chaseIdx, PULSE_AMP, TAIL_FRAC);
    chaseIdx = (chaseIdx + 1) % 8;
  }

  // Blink rate proportional to total energy
  float e = totalEnergy();
  uint32_t blinkMs = (uint32_t)(MAX_BLINK_MS - e * (MAX_BLINK_MS - MIN_BLINK_MS));
  if (now - lastBlinkMs >= blinkMs) {
    lastBlinkMs = now;
    ledState    = !ledState;
    digitalWrite(LED_PIN, ledState ? HIGH : LOW);
  }
}
