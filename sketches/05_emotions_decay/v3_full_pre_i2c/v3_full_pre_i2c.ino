// =============================================================================
// 05_emotions_decay – Variant 3: Full pre-I2C
// =============================================================================
// Concept: Complete emotional model (arousal/affection/anxiety) with touch
// input.  Mirrors anxious.ino exactly but without I2C / PCF8574 outputs.
// The built-in LED brightness is simulated via Serial bars.
//
// IMPORTANT – Touch-capable pins on ESP32:
//   GPIO4 (T0) matches anxious.ino.  See ESP32 Technical Reference Manual
//   for the full list of touch-capable GPIOs.
//
// Wiring: wire/metal pad → GPIO4;  built-in LED on GPIO2.
// =============================================================================

// ---- Tuning knobs -----------------------------------------------------------
constexpr int   TOUCH_PIN       = 4;
constexpr int   LED_PIN         = 2;
constexpr float TOUCH_ON_RATIO  = 0.85f;
constexpr float TOUCH_OFF_RATIO = 0.90f;
constexpr float SMOOTH_ALPHA    = 0.15f;
constexpr uint32_t TICK_MS      = 50;
constexpr uint32_t PRINT_MS     = 800;
// Simulated BLE burst every N ms (no real BLE in this sketch)
constexpr uint32_t FAKE_BLE_MS  = 6000;
// -----------------------------------------------------------------------------

struct Emotion {
  float arousal   = 0.0f;
  float affection = 0.0f;
  float anxiety   = 0.10f;
  uint32_t lastTouchMs    = 0;
  int      lastSeenDevices = 0;
} E;

int   touchBaseline = 0;
float touchFiltered = 0.0f;
bool  touchLatched  = false;
uint32_t lastTickMs  = 0;
uint32_t lastPrintMs = 0;
uint32_t lastBleMsLocal = 0;

float clamp01(float x) { return x < 0 ? 0 : (x > 1 ? 1 : x); }

void calibrateTouch() {
  delay(400); long s = 0;
  for (int i = 0; i < 50; i++) { s += touchRead(TOUCH_PIN); delay(10); }
  touchBaseline = (int)(s / 50); touchFiltered = (float)touchBaseline;
  Serial.print("baseline="); Serial.println(touchBaseline);
}

bool isTouched() {
  int raw = touchRead(TOUCH_PIN);
  touchFiltered = (1 - SMOOTH_ALPHA) * touchFiltered + SMOOTH_ALPHA * raw;
  float on = touchBaseline * TOUCH_ON_RATIO, off = touchBaseline * TOUCH_OFF_RATIO;
  if (!touchLatched) { if (touchFiltered < on)  touchLatched = true; }
  else               { if (touchFiltered > off) touchLatched = false; }
  return touchLatched;
}

void tickEmotions(float dt) {
  E.arousal   = clamp01(E.arousal   - 0.06f * dt);
  E.affection = clamp01(E.affection - 0.02f * dt);
  E.anxiety   = clamp01(E.anxiety   + 0.02f * dt - 0.03f * E.affection * dt);
}

// Simple bar representation of a 0..1 float
String bar(float v) {
  int n = (int)(v * 10);
  String s = "[";
  for (int i = 0; i < 10; i++) s += (i < n) ? "#" : " ";
  return s + "] " + String(v, 2);
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  Serial.println("05_emotions_decay v3 – full pre-I2C");
  calibrateTouch();
  E.lastTouchMs = millis();
}

void loop() {
  uint32_t now = millis();

  // Touch
  static bool lastT = false;
  bool t = isTouched();
  if (t && !lastT) {
    E.lastTouchMs = now;
    E.affection   = clamp01(E.affection + 0.30f);
    E.anxiety     = clamp01(E.anxiety   - 0.40f);
    Serial.println(">> TOUCH");
  }
  lastT = t;
  digitalWrite(LED_PIN, t ? HIGH : LOW);

  // Fake BLE event
  if (now - lastBleMsLocal >= FAKE_BLE_MS) {
    lastBleMsLocal = now;
    E.lastSeenDevices = random(0, 8);
    E.arousal = clamp01(E.arousal + 0.10f * clamp01(E.lastSeenDevices / 10.0f));
    Serial.print(">> FAKE BLE devices="); Serial.println(E.lastSeenDevices);
  }

  // Emotion tick
  if (now - lastTickMs >= TICK_MS) {
    float dt = (now - lastTickMs) / 1000.0f; lastTickMs = now;
    tickEmotions(dt);
  }

  // Print bars
  if (now - lastPrintMs >= PRINT_MS) {
    lastPrintMs = now;
    Serial.println("arousal   " + bar(E.arousal));
    Serial.println("affection " + bar(E.affection));
    Serial.println("anxiety   " + bar(E.anxiety));
    Serial.println();
  }
}
