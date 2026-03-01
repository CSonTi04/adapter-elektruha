// =============================================================================
// 05_emotions_decay – Variant 1: Simulated (Serial only)
// =============================================================================
// Concept: The emotional model from anxius.ino running in pure software.
//
// Three continuous values (0..1):
//   arousal   – rises with external stimulation, decays over time.
//   affection – rises on touch, decays slowly.
//   anxiety   – drifts upward unless touch/affection suppress it.
//
// This variant has no hardware. You inject events over Serial:
//   't' = touch  (boosts affection, reduces anxiety)
//   'b' = BLE crowd event (boosts arousal)
//   'r' = reset all to 0
//
// Wiring: none – open Serial Monitor at 115200 baud.
// =============================================================================

// ---- Tuning knobs -----------------------------------------------------------
constexpr uint32_t TICK_MS      = 50;   // emotion update interval (ms)
constexpr uint32_t PRINT_MS     = 500;  // Serial print interval (ms)
constexpr float    DECAY_DT     = TICK_MS / 1000.0f; // dt in seconds
// -----------------------------------------------------------------------------

float arousal   = 0.0f;
float affection = 0.0f;
float anxiety   = 0.10f;  // small initial anxiety

uint32_t lastTickMs  = 0;
uint32_t lastPrintMs = 0;

float clamp01(float x) { return x < 0 ? 0 : (x > 1 ? 1 : x); }

void tickEmotions(float dt) {
  arousal   = clamp01(arousal   - 0.06f * dt);
  affection = clamp01(affection - 0.02f * dt);
  anxiety   = clamp01(anxiety   + 0.02f * dt - 0.03f * affection * dt);
}

void setup() {
  Serial.begin(115200);
  Serial.println("05_emotions_decay v1 – simulated");
  Serial.println("Commands: 't'=touch  'b'=BLE crowd  'r'=reset");
  Serial.println("arousal  affection  anxiety");
}

void loop() {
  uint32_t now = millis();

  // Read Serial commands
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 't') {
      affection = clamp01(affection + 0.30f);
      anxiety   = clamp01(anxiety   - 0.40f);
      Serial.println(">> TOUCH applied");
    }
    if (c == 'b') {
      arousal = clamp01(arousal + 0.10f);
      Serial.println(">> BLE crowd event");
    }
    if (c == 'r') {
      arousal = affection = anxiety = 0;
      Serial.println(">> RESET");
    }
  }

  // Periodic emotion tick
  if (now - lastTickMs >= TICK_MS) {
    float dt = (now - lastTickMs) / 1000.0f;
    lastTickMs = now;
    tickEmotions(dt);
  }

  // Periodic print
  if (now - lastPrintMs >= PRINT_MS) {
    lastPrintMs = now;
    Serial.print(arousal,   2); Serial.print("  ");
    Serial.print(affection, 2); Serial.print("  ");
    Serial.println(anxiety, 2);
  }
}
