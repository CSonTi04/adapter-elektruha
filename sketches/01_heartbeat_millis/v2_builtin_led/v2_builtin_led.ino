// =============================================================================
// 01_heartbeat_millis – Variant 2: ESP32 built-in LED (minimal hardware)
// =============================================================================
// Concept: Non-blocking LED blink with millis().
// Uses the ESP32 dev-board built-in LED (GPIO2 on most DOIT DevKit boards).
// No external components needed.
//
// Wiring: just the ESP32 board – built-in LED is on GPIO2.
// =============================================================================

// ---- Tuning knobs -----------------------------------------------------------
constexpr int     LED_PIN        = 2;    // built-in LED on most DOIT ESP32 boards
constexpr uint32_t BEAT_INTERVAL_MS = 500;  // blink period (ms)
// -----------------------------------------------------------------------------

bool ledState = false;
uint32_t lastBeatMs = 0;

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  Serial.println("01_heartbeat_millis v2 – built-in LED");
}

void loop() {
  uint32_t now = millis();

  if (now - lastBeatMs >= BEAT_INTERVAL_MS) {
    lastBeatMs = now;
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState ? HIGH : LOW);

    Serial.print("BEAT  led=");
    Serial.println(ledState ? "ON" : "OFF");
  }
}
