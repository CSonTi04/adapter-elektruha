// =============================================================================
// 08_binary_vs_sliced_pwm – Variant 1: Serial slicing (no hardware)
// =============================================================================
// Concept: Binary ON/OFF vs time-sliced PWM for LED brightness.
//
// anxius.ino offers two output modes:
//   ENABLE_PWM = false → computeMaskBinary()  – crisp ON/OFF states
//   ENABLE_PWM = true  → computeMask()        – simulated brightness via
//                         rapid on/off time slices
//
// This sketch simulates both approaches in Serial so you can see the
// difference in how they handle intermediate brightness values.
//
// Toggle ENABLE_PWM and re-upload to compare.
//
// Wiring: none – open Serial Monitor at 115200 baud.
// =============================================================================

// ---- Tuning knobs -----------------------------------------------------------
constexpr bool  ENABLE_PWM       = false;  // <<< TOGGLE THIS
constexpr float BINARY_THRESHOLD = 0.22f;  // threshold for binary mode
constexpr uint8_t PWM_LEVELS     = 16;     // slices per PWM period
constexpr uint32_t PRINT_MS      = 200;
// A synthetic brightness ramp to compare both modes
constexpr float TEST_VALUES[] = {0.0f, 0.1f, 0.2f, 0.3f, 0.5f, 0.7f, 1.0f};
constexpr int   N_VALUES = 7;
// -----------------------------------------------------------------------------

uint8_t pwmPhase   = 0;
uint32_t lastMs    = 0;
int      valueIdx  = 0;

// Binary: ON if brightness^2 >= threshold
bool binaryOn(float b) {
  return (b * b) >= BINARY_THRESHOLD;
}

// PWM: ON in this slice if level > pwmPhase
bool pwmOn(float b) {
  float shaped = b * b;
  int level = (int)(shaped * PWM_LEVELS + 0.5f);
  if (level > PWM_LEVELS) level = PWM_LEVELS;
  return pwmPhase < level;
}

void printRow(float brightness) {
  bool on = ENABLE_PWM ? pwmOn(brightness) : binaryOn(brightness);
  Serial.print("b="); Serial.print(brightness, 1);
  Serial.print("  shaped="); Serial.print(brightness * brightness, 2);
  Serial.print("  output="); Serial.println(on ? "ON " : "OFF");
}

void setup() {
  Serial.begin(115200);
  Serial.print("08_binary_vs_sliced_pwm v1 – ENABLE_PWM=");
  Serial.println(ENABLE_PWM ? "true (sliced)" : "false (binary)");
  Serial.println("brightness  shaped  output");
  Serial.println("-------------------------------");
}

void loop() {
  uint32_t now = millis();
  if (now - lastMs < PRINT_MS) return;
  lastMs = now;

  if (ENABLE_PWM) {
    // Advance one PWM phase and show all test values in that phase
    pwmPhase = (pwmPhase + 1) % PWM_LEVELS;
    Serial.print("--- pwmPhase="); Serial.print(pwmPhase);
    Serial.print(" of "); Serial.println(PWM_LEVELS);
    for (int i = 0; i < N_VALUES; i++) printRow(TEST_VALUES[i]);
  } else {
    // Binary mode: show each test value once
    printRow(TEST_VALUES[valueIdx]);
    valueIdx = (valueIdx + 1) % N_VALUES;
    if (valueIdx == 0) Serial.println("---");
  }
}
