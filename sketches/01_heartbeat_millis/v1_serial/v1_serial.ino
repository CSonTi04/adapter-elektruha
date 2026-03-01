// =============================================================================
// 01_heartbeat_millis – Variant 1: Serial-only (no hardware required)
// =============================================================================
// Concept: Non-blocking timing using millis().
// Instead of delay(), we check whether enough time has passed and only then
// act.  This keeps loop() responsive for future sensors/outputs.
//
// Wiring: none – open Serial Monitor at 115200 baud.
// =============================================================================

// ---- Tuning knobs -----------------------------------------------------------
constexpr uint32_t BEAT_INTERVAL_MS = 500;  // how often to "beat" (ms)
// -----------------------------------------------------------------------------

uint32_t lastBeatMs = 0;  // timestamp of the last beat

void setup() {
  Serial.begin(115200);
  Serial.println("01_heartbeat_millis v1 – Serial only");
  Serial.println("Watching millis()... each BEAT = 500 ms elapsed.");
}

void loop() {
  uint32_t now = millis();

  // Only act when BEAT_INTERVAL_MS has passed – never blocks!
  if (now - lastBeatMs >= BEAT_INTERVAL_MS) {
    lastBeatMs = now;
    Serial.print("BEAT  t=");
    Serial.print(now);
    Serial.println(" ms");
  }

  // Nothing else is blocked – you can add more timed events here.
}
