// =============================================================================
// 01_heartbeat_millis – Variant 3: Full build, pre-I2C (built-in LED)
// =============================================================================
// Concept: Same non-blocking heartbeat, but in the context of the full project
// build environment.  PCF8574 / I2C is NOT introduced yet – we use the
// built-in LED only so the sketch compiles and runs on the bare ESP32.
//
// Why "pre-I2C"? The curriculum introduces the I2C bus (sketch 04A) before
// driving PCF8574 outputs (sketch 04B).  Variants 3 of sketches 01–06 use
// only the built-in LED so learners can run them without external hardware.
//
// Wiring: just the ESP32 board.
// =============================================================================

// ---- Tuning knobs -----------------------------------------------------------
constexpr int     LED_PIN         = 2;    // built-in LED (GPIO2)
constexpr uint32_t BEAT_MS        = 500;  // heartbeat period (ms)
constexpr uint32_t FAST_BEAT_MS   = 100;  // "excited" fast beat (ms)
// -----------------------------------------------------------------------------

bool ledState      = false;
bool fastMode      = false;          // toggle in Serial to switch speed
uint32_t lastBeatMs = 0;

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  Serial.println("01_heartbeat_millis v3 – full pre-I2C");
  Serial.println("Send 'f' for fast beat, 's' for slow beat.");
}

void loop() {
  // Check for Serial commands
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'f') { fastMode = true;  Serial.println(">> FAST mode"); }
    if (c == 's') { fastMode = false; Serial.println(">> SLOW mode"); }
  }

  uint32_t now      = millis();
  uint32_t interval = fastMode ? FAST_BEAT_MS : BEAT_MS;

  if (now - lastBeatMs >= interval) {
    lastBeatMs = now;
    ledState   = !ledState;
    digitalWrite(LED_PIN, ledState ? HIGH : LOW);
    Serial.print("BEAT  t="); Serial.print(now);
    Serial.print("  interval="); Serial.println(interval);
  }
}
