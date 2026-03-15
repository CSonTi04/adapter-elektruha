// =============================================================================
// 05_emotions_decay – Variant 2: Touch-based (real touch input)
// =============================================================================
// Concept: Same emotional model but driven by real capacitive touch.
// Touch the electrode to apply affection (LED on = touched).
//
// IMPORTANT – Touch-capable pins on ESP32:
//   GPIO4 (T0) is used here to match anxious.ino.  Consult the ESP32
//   Technical Reference Manual for the complete list of touch-capable GPIOs
//   before changing TOUCH_PIN.
//
// Wiring:
//   Wire / metal pad → GPIO4  (touch electrode)
//   Built-in LED (GPIO2) lights when touched
// =============================================================================

// ---- Tuning knobs -----------------------------------------------------------
constexpr int   TOUCH_PIN      = 4;    // GPIO4 (T0) – see note above
constexpr int   LED_PIN        = 2;    // built-in LED
constexpr float TOUCH_ON_RATIO = 0.85f;
constexpr float TOUCH_OFF_RATIO= 0.90f;
constexpr float SMOOTH_ALPHA   = 0.15f;
constexpr uint32_t TICK_MS     = 50;
constexpr uint32_t PRINT_MS    = 600;
// -----------------------------------------------------------------------------

float arousal   = 0.0f;
float affection = 0.0f;
float anxiety   = 0.10f;

int   touchBaseline = 0;
float touchFiltered = 0.0f;
bool  touchLatched  = false;

uint32_t lastTickMs  = 0;
uint32_t lastPrintMs = 0;

float clamp01(float x) { return x < 0 ? 0 : (x > 1 ? 1 : x); }

void calibrateTouch() {
  delay(400);
  long sum = 0;
  for (int i = 0; i < 50; i++) { sum += touchRead(TOUCH_PIN); delay(10); }
  touchBaseline = (int)(sum / 50);
  touchFiltered = (float)touchBaseline;
  Serial.print("Touch baseline="); Serial.println(touchBaseline);
}

bool isTouched() {
  int raw = touchRead(TOUCH_PIN);
  touchFiltered = (1 - SMOOTH_ALPHA) * touchFiltered + SMOOTH_ALPHA * raw;
  float on  = touchBaseline * TOUCH_ON_RATIO;
  float off = touchBaseline * TOUCH_OFF_RATIO;
  if (!touchLatched) { if (touchFiltered < on)  touchLatched = true; }
  else               { if (touchFiltered > off) touchLatched = false; }
  return touchLatched;
}

void tickEmotions(float dt) {
  arousal   = clamp01(arousal   - 0.06f * dt);
  affection = clamp01(affection - 0.02f * dt);
  anxiety   = clamp01(anxiety   + 0.02f * dt - 0.03f * affection * dt);
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  Serial.println("05_emotions_decay v2 – touch-based");
  calibrateTouch();
}

void loop() {
  uint32_t now = millis();

  // Touch edge detection
  static bool lastTouched = false;
  bool touched = isTouched();
  if (touched && !lastTouched) {
    affection = clamp01(affection + 0.30f);
    anxiety   = clamp01(anxiety   - 0.40f);
    Serial.println("TOUCH!");
  }
  lastTouched = touched;
  digitalWrite(LED_PIN, touched ? HIGH : LOW);

  // Emotion tick
  if (now - lastTickMs >= TICK_MS) {
    float dt = (now - lastTickMs) / 1000.0f;
    lastTickMs = now;
    tickEmotions(dt);
  }

  // Print
  if (now - lastPrintMs >= PRINT_MS) {
    lastPrintMs = now;
    Serial.print("arous="); Serial.print(arousal,   2);
    Serial.print(" affec="); Serial.print(affection, 2);
    Serial.print(" anxty="); Serial.println(anxiety, 2);
  }
}
