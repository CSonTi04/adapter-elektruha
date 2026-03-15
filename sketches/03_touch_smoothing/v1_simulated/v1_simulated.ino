// =============================================================================
// 03_touch_smoothing – Variant 1: Simulated (no hardware required)
// =============================================================================
// Concept: How anxious.ino reads capacitive touch robustly.
//
// Real touch sensors are noisy.  anxious.ino uses two techniques:
//   1) Exponential smoothing – averages recent readings to reduce noise.
//   2) Hysteresis latch – separate ON/OFF thresholds to stop rapid toggling.
//
// This sketch *simulates* those techniques with a synthetic signal so you
// can study the algorithm without any hardware.
//
// Wiring: none – open Serial Monitor at 115200 baud.
// =============================================================================

// ---- LEARN: float (decimal numbers) ----------------------------------------
// "float" stores a decimal number like 0.85, 3.14, or -0.5.
// The "f" suffix on a literal (e.g. 0.85f) tells the compiler it is a
// single-precision float, not a double.  Always include it with float
// constants to avoid accidental type-conversion warnings.
//
// Floats are ideal for percentages, ratios, and physical quantities where
// fractions matter.  For exact whole-number counting (timestamps, indices)
// use int or uint32_t instead.
// ---- Tuning knobs -----------------------------------------------------------
constexpr float TOUCH_ON_RATIO  = 0.85f;  // enter-touch below this * baseline
constexpr float TOUCH_OFF_RATIO = 0.90f;  // release-touch above this * baseline
constexpr float SMOOTH_ALPHA    = 0.15f;  // exponential smoothing factor (0..1)
constexpr int   BASELINE        = 500;    // pretend baseline reading
constexpr uint32_t PRINT_MS     = 200;    // how often to print (ms)
// -----------------------------------------------------------------------------

// ---- LEARN: bool (true/false values) ----------------------------------------
// "bool" (short for "boolean") can only ever be true or false.
// It is perfect for flags like "is the touch currently active?"
// Internally, true = 1 and false = 0, but using bool makes the intent clear.
// -----------------------------------------------------------------------------

// Simulated "sensor" state
int   simRaw      = BASELINE;
float touchFiltered = (float)BASELINE;
bool  touchLatched  = false;
// ---- LEARN: (float) cast ---------------------------------------------------
// "(float)BASELINE" converts the integer BASELINE to a float.
// This is called a "type cast".  We do it here so touchFiltered starts at the
// same value as BASELINE (500), expressed as a decimal (500.0).
// Without the cast some compilers would warn about mixing types.
// -----------------------------------------------------------------------------
int   simPhase    = 0;       // 0=normal, 1=simulated touch

uint32_t lastPrintMs = 0;

// ---- LEARN: Functions with parameters and a return value -------------------
// This function takes one input ("now", the current time in ms) and
// returns a simulated sensor reading (an int).
//
// "int simulateRaw(uint32_t now)" means:
//   int        = it will RETURN a whole number
//   simulateRaw = the function's name
//   uint32_t now = one input parameter named "now"
//
// The "return" statement sends a value back to whoever called the function.
// "return BASELINE + (int)(random(-15, 15))" returns a number near BASELINE.
// ---- Produce a synthetic raw reading that dips when "touched" ---------------
int simulateRaw(uint32_t now) {
  // Every 3 seconds pretend a touch for 1.5 seconds.
  // ---- LEARN: % (modulo operator) ------------------------------------------
  // The % operator gives the REMAINDER after integer division.
  // "now % 3000" cycles through 0, 1, 2, … 2999, then back to 0 every 3 s.
  // It is like the second-hand of a clock: it resets each full rotation.
  // --------------------------------------------------------------------------
  uint32_t cycle = now % 3000;
  if (cycle < 1500) {
    return BASELINE - (int)(BASELINE * 0.20f) + (int)(random(-20, 20));
  }
  return BASELINE + (int)(random(-15, 15));
}

// Smoothing + hysteresis (same algorithm as anxious.ino)
bool isTouched(int raw) {
  // ---- LEARN: Compound expression on one line ------------------------------
  // touchFiltered = (1.0f - SMOOTH_ALPHA) * touchFiltered + SMOOTH_ALPHA * raw
  // This is exponential smoothing.  Think of it as:
  //   new_average = (mostly old average) + (a little bit of new reading)
  // SMOOTH_ALPHA = 0.15 means "take 85% of the old value and 15% of the new".
  // Small alpha = very smooth but slow to react.
  // Large alpha = reacts quickly but more noise.
  // --------------------------------------------------------------------------
  touchFiltered = (1.0f - SMOOTH_ALPHA) * touchFiltered + SMOOTH_ALPHA * raw;

  float onThresh  = BASELINE * TOUCH_ON_RATIO;
  float offThresh = BASELINE * TOUCH_OFF_RATIO;

  // ---- LEARN: if / else with a bool flag (hysteresis) ----------------------
  // "!touchLatched" means "NOT touchLatched" — true when touchLatched is false.
  // The two separate thresholds (onThresh < offThresh) create a "dead zone"
  // that prevents the output from flickering when the signal hovers near
  // the edge.  This trick is called hysteresis.
  // --------------------------------------------------------------------------
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
    // ---- LEARN: Ternary operator ? : ----------------------------------------
    // FORMAT:  condition ? value_if_true : value_if_false
    // This is a compact way to choose between two values in one expression.
    // "touched ? "TOUCHED" : "---""
    //   if touched is true  → print "TOUCHED"
    //   if touched is false → print "---"
    // Equivalent to: if (touched) { print "TOUCHED"; } else { print "---"; }
    // -------------------------------------------------------------------------
    Serial.println(touched ? "TOUCHED" : "---");
  }
}
