// =============================================================================
// 03_touch_smoothing – Variant 3: Full pre-I2C touch
// =============================================================================
// Concept: Touch smoothing in the full project context.
// Mirrors the touch sensing from anxius.ino exactly, including the rising-edge
// detection used to trigger emotional events.  No I2C / PCF8574 yet.
//
// IMPORTANT – Touch-capable pins on ESP32:
//   GPIO4 (T0) is used here to match anxius.ino.  Before changing the pin,
//   consult the ESP32 Technical Reference Manual for the full list of
//   touch-capable GPIOs on your specific module.
//
// Wiring:
//   - Wire / metal pad → GPIO4
//   - Built-in LED on GPIO2
// =============================================================================

// ---- Tuning knobs -----------------------------------------------------------
constexpr int   TOUCH_PIN       = 4;    // GPIO4 (T0) – see note above
constexpr int   LED_PIN         = 2;    // built-in LED
constexpr float TOUCH_ON_RATIO  = 0.85f;
constexpr float TOUCH_OFF_RATIO = 0.90f;
constexpr float SMOOTH_ALPHA    = 0.15f;
constexpr uint32_t PRINT_MS     = 250;
// -----------------------------------------------------------------------------

int   touchBaseline = 0;
float touchFiltered = 0.0f;
bool  touchLatched  = false;
uint32_t touchEventCount = 0;   // count rising-edge touch events
uint32_t lastPrintMs     = 0;

void calibrateTouch() {
  delay(400);
  long sum = 0;
  for (int i = 0; i < 50; i++) { sum += touchRead(TOUCH_PIN); delay(10); }
  touchBaseline = (int)(sum / 50);
  touchFiltered = (float)touchBaseline;
  Serial.print("baseline="); Serial.println(touchBaseline);
}

bool isTouched() {
  int raw = touchRead(TOUCH_PIN);
  touchFiltered = (1.0f - SMOOTH_ALPHA) * touchFiltered + SMOOTH_ALPHA * raw;
  float on  = touchBaseline * TOUCH_ON_RATIO;
  float off = touchBaseline * TOUCH_OFF_RATIO;
  if (!touchLatched) { if (touchFiltered < on)  touchLatched = true; }
  else               { if (touchFiltered > off) touchLatched = false; }
  return touchLatched;
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  Serial.println("03_touch_smoothing v3 – full pre-I2C");
  calibrateTouch();
}

void loop() {
  uint32_t now = millis();

  static bool lastTouched = false;
  bool touched = isTouched();

  // Rising edge: new touch event
  if (touched && !lastTouched) {
    touchEventCount++;
    Serial.print("TOUCH EVENT #"); Serial.println(touchEventCount);
  }
  lastTouched = touched;

  // Visual feedback
  digitalWrite(LED_PIN, touched ? HIGH : LOW);

  // Periodic status
  if (now - lastPrintMs >= PRINT_MS) {
    lastPrintMs = now;
    Serial.print("filtered="); Serial.print((int)touchFiltered);
    Serial.print("  latched="); Serial.println(touched ? "YES" : "NO");
  }
}
