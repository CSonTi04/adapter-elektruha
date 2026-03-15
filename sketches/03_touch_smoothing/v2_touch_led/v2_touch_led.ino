// =============================================================================
// 03_touch_smoothing – Variant 2: Real touch + built-in LED
// =============================================================================
// Concept: Capacitive touch with smoothing and hysteresis on real hardware.
//
// IMPORTANT – Touch-capable pins on ESP32:
//   The ESP32 has several GPIO pins that support capacitive touch sensing via
//   the touchRead() function.  Common touch-capable GPIOs are T0(GPIO4),
//   T1(GPIO0), T2(GPIO2), T3(GPIO15), T4(GPIO13), T5(GPIO12), T6(GPIO14),
//   T7(GPIO27), T8(GPIO33), T9(GPIO32).
//   ** Always check your specific ESP32 board's pinout and the official
//      Espressif ESP32 Technical Reference Manual before wiring. **
//
// This sketch defaults to GPIO4 (T0) to match anxious.ino.
// Connect a wire or small metal pad to GPIO4 as a touch electrode.
//
// Wiring:
//   - Wire or metal pad  → GPIO4  (touch electrode)
//   - Built-in LED (GPIO2) is used for visual feedback (HIGH = touched)
// =============================================================================

// ---- Tuning knobs -----------------------------------------------------------
// See the note above about touch-capable pins before changing TOUCH_PIN.
constexpr int   TOUCH_PIN       = 4;    // GPIO4 (T0) – matches anxious.ino
constexpr int   LED_PIN         = 2;    // built-in LED
constexpr float TOUCH_ON_RATIO  = 0.85f;
constexpr float TOUCH_OFF_RATIO = 0.90f;
constexpr float SMOOTH_ALPHA    = 0.15f;
constexpr uint32_t PRINT_MS     = 200;
// -----------------------------------------------------------------------------

int   touchBaseline = 0;
float touchFiltered = 0.0f;
bool  touchLatched  = false;
uint32_t lastPrintMs = 0;

// Calibrate baseline on startup (same method as anxious.ino)
void calibrateTouch() {
  delay(400);
  long sum = 0;
  for (int i = 0; i < 50; i++) {
    sum += touchRead(TOUCH_PIN);
    delay(10);
  }
  touchBaseline = (int)(sum / 50);
  touchFiltered = (float)touchBaseline;
  Serial.print("Calibrated baseline=");
  Serial.println(touchBaseline);
}

bool isTouched() {
  int raw = touchRead(TOUCH_PIN);
  touchFiltered = (1.0f - SMOOTH_ALPHA) * touchFiltered + SMOOTH_ALPHA * raw;

  float onThresh  = touchBaseline * TOUCH_ON_RATIO;
  float offThresh = touchBaseline * TOUCH_OFF_RATIO;

  if (!touchLatched) {
    if (touchFiltered < onThresh) touchLatched = true;
  } else {
    if (touchFiltered > offThresh) touchLatched = false;
  }
  return touchLatched;
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  Serial.println("03_touch_smoothing v2 – real touch + built-in LED");
  calibrateTouch();
  Serial.println("raw  filtered  touched");
}

void loop() {
  uint32_t now = millis();
  bool touched = isTouched();
  digitalWrite(LED_PIN, touched ? HIGH : LOW);

  if (now - lastPrintMs >= PRINT_MS) {
    lastPrintMs = now;
    int raw = touchRead(TOUCH_PIN);
    Serial.print(raw);
    Serial.print("  ");
    Serial.print((int)touchFiltered);
    Serial.print("  ");
    Serial.println(touched ? "TOUCHED" : "---");
  }
}
