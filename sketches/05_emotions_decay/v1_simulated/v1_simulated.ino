// =============================================================================
// 05_emotions_decay – Variant 1: Simulated (Serial only)
// =============================================================================
// Concept: The emotional model from anxious.ino running in pure software.
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

// ---- LEARN: Inline function definition (one-liner) -------------------------
// A function whose body fits on one line can be written compactly like this.
// clamp01 takes a float "x" and returns x clamped to the range [0..1]:
//   if x < 0 → return 0
//   if x > 1 → return 1
//   otherwise → return x unchanged
//
// The "? :" ternary operator is used twice here, nested:
//   x < 0 ? 0 : (...)   →  if x<0 give 0, else evaluate (...)
//   (x > 1 ? 1 : x)     →  if x>1 give 1, else give x
// This prevents emotion values from going below 0 or above 1.
// -----------------------------------------------------------------------------
float clamp01(float x) { return x < 0 ? 0 : (x > 1 ? 1 : x); }

void tickEmotions(float dt) {
  // ---- LEARN: Compound assignment (update a variable in-place) -------------
  // "arousal = clamp01(arousal - 0.06f * dt)" means:
  //   1. Calculate  arousal - 0.06 * dt   (subtract a small decay amount)
  //   2. Clamp the result to 0..1
  //   3. Store the result BACK into arousal, replacing the old value
  //
  // "dt" is "delta time" — the number of seconds since the last tick.
  // Multiplying by dt makes the decay speed independent of how often
  // tickEmotions() is called.  This is called "frame-rate independent" update.
  // --------------------------------------------------------------------------
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

  // ---- LEARN: while loop + Serial.available() --------------------------------
  // "Serial.available()" returns the number of bytes waiting to be read from
  // the Serial input buffer.  It returns 0 if you haven't typed anything.
  //
  // "while (Serial.available())" keeps looping as long as there are bytes
  // waiting.  Each call to "Serial.read()" takes ONE byte from the buffer.
  //
  // A while loop is like an if-statement that repeats:
  //   "keep doing this as long as the condition is true"
  //
  // Here it means: "keep reading and processing characters until the buffer
  // is empty."  If you paste multiple characters at once they all get processed.
  // ----------------------------------------------------------------------------
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
      // ---- LEARN: Chained assignment ----------------------------------------
      // "arousal = affection = anxiety = 0" sets all three to zero in one line.
      // Assignment evaluates right-to-left: anxiety=0 returns 0, then
      // affection=0 returns 0, then arousal=0.
      // This is a neat shorthand when multiple variables need the same value.
      // -----------------------------------------------------------------------
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
