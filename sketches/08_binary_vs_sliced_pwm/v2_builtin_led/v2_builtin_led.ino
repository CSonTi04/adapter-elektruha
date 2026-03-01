// =============================================================================
// 08_binary_vs_sliced_pwm – Variant 2: Built-in LED slicing
// =============================================================================
// Concept: Binary vs sliced-PWM brightness on the built-in LED.
//
// The built-in LED brightness slowly ramps 0→1 then back.
// In BINARY mode you see a sharp on/off snap at the threshold.
// In PWM mode the LED dims gradually via rapid on/off slicing.
//
// Toggle ENABLE_PWM below and re-upload to see the difference.
//
// Wiring: just the ESP32 board – built-in LED on GPIO2.
// =============================================================================

// ---- Tuning knobs -----------------------------------------------------------
constexpr int     LED_PIN         = 2;
constexpr bool    ENABLE_PWM      = false;  // <<< TOGGLE THIS
constexpr float   BINARY_THRESHOLD = 0.22f;
constexpr uint8_t PWM_LEVELS       = 16;
constexpr uint32_t PWM_SLICE_US    = 250;   // each slice = 250 µs → ~250 Hz
constexpr uint32_t RAMP_STEP_MS    = 30;    // how fast brightness ramps
// -----------------------------------------------------------------------------

float    brightness   = 0.0f;
float    rampDir      = 0.01f;     // ramp increment per RAMP_STEP_MS

uint8_t  pwmPhase     = 0;
uint32_t lastSliceUs  = 0;
uint32_t lastRampMs   = 0;

float clamp01f(float x) { return x < 0 ? 0 : (x > 1 ? 1 : x); }

bool shouldLightOn() {
  float shaped = brightness * brightness;
  if (ENABLE_PWM) {
    int level = (int)(shaped * PWM_LEVELS + 0.5f);
    if (level > PWM_LEVELS) level = PWM_LEVELS;
    return pwmPhase < level;
  } else {
    return shaped >= BINARY_THRESHOLD;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  Serial.print("08_binary_vs_sliced_pwm v2 – ENABLE_PWM=");
  Serial.println(ENABLE_PWM ? "true" : "false");
}

void loop() {
  uint32_t nowUs = micros();
  uint32_t nowMs = millis();

  // Advance brightness ramp
  if (nowMs - lastRampMs >= RAMP_STEP_MS) {
    lastRampMs = nowMs;
    brightness += rampDir;
    if (brightness >= 1.0f) { brightness = 1.0f; rampDir = -rampDir; }
    if (brightness <= 0.0f) { brightness = 0.0f; rampDir = -rampDir; }

    Serial.print("brightness="); Serial.print(brightness, 2);
    Serial.print("  led="); Serial.println(shouldLightOn() ? "ON" : "OFF");
  }

  // Advance PWM phase
  if (ENABLE_PWM && nowUs - lastSliceUs >= PWM_SLICE_US) {
    lastSliceUs = nowUs;
    pwmPhase = (pwmPhase + 1) % PWM_LEVELS;
  }

  // Drive LED
  digitalWrite(LED_PIN, shouldLightOn() ? HIGH : LOW);
}
