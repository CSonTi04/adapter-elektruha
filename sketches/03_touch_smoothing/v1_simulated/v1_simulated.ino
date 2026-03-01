// =============================================================================
// 03_touch_smoothing – Variant 1: Simulated (no hardware required)
// =============================================================================
// Concept: How anxius.ino reads capacitive touch robustly.
//
// Real touch sensors are noisy.  anxius.ino uses two techniques:
//   1) Exponential smoothing – averages recent readings to reduce noise.
//   2) Hysteresis latch – separate ON/OFF thresholds to stop rapid toggling.
//
// This sketch *simulates* those techniques with a synthetic signal so you
// can study the algorithm without any hardware.
//
// Wiring: none – open Serial Monitor at 115200 baud.
// =============================================================================

// ---- Tuning knobs -----------------------------------------------------------
constexpr float TOUCH_ON_RATIO  = 0.85f;  // enter-touch below this * baseline
constexpr float TOUCH_OFF_RATIO = 0.90f;  // release-touch above this * baseline
constexpr float SMOOTH_ALPHA    = 0.15f;  // exponential smoothing factor (0..1)
constexpr int   BASELINE        = 500;    // pretend baseline reading
constexpr uint32_t PRINT_MS     = 200;    // how often to print (ms)
// -----------------------------------------------------------------------------

// Simulated "sensor" state
int   simRaw      = BASELINE;
float touchFiltered = (float)BASELINE;
bool  touchLatched  = false;
int   simPhase    = 0;       // 0=normal, 1=simulated touch

uint32_t lastPrintMs = 0;

// Produce a synthetic raw reading that dips when "touched"
int simulateRaw(uint32_t now) {
  // Every 3 seconds pretend a touch for 1.5 seconds
  uint32_t cycle = now % 3000;
  if (cycle < 1500) {
    return BASELINE - (int)(BASELINE * 0.20f) + (int)(random(-20, 20));
  }
  return BASELINE + (int)(random(-15, 15));
}

// Smoothing + hysteresis (same algorithm as anxius.ino)
bool isTouched(int raw) {
  touchFiltered = (1.0f - SMOOTH_ALPHA) * touchFiltered + SMOOTH_ALPHA * raw;

  float onThresh  = BASELINE * TOUCH_ON_RATIO;
  float offThresh = BASELINE * TOUCH_OFF_RATIO;

  if (!touchLatched) {
    if (touchFiltered < onThresh) touchLatched = true;
  } else {
    if (touchFiltered > offThresh) touchLatched = false;
  }
  return touchLatched;
}

void setup() {
  Serial.begin(115200);
  randomSeed(42);
  Serial.println("03_touch_smoothing v1 – simulated");
  Serial.println("raw  filtered  touched");
}

void loop() {
  uint32_t now = millis();
  int raw = simulateRaw(now);
  bool touched = isTouched(raw);

  if (now - lastPrintMs >= PRINT_MS) {
    lastPrintMs = now;
    Serial.print(raw);
    Serial.print("  ");
    Serial.print((int)touchFiltered);
    Serial.print("  ");
    Serial.println(touched ? "TOUCHED" : "---");
  }
}
