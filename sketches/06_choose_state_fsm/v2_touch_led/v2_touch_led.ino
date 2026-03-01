// =============================================================================
// 06_choose_state_fsm – Variant 2: Touch + LED patterns
// =============================================================================
// Concept: Same FSM but driven by real touch, with distinct built-in LED
// blink patterns for each state.
//
// LED patterns:
//   Idle     – slow blink (1 s)
//   Excited  – medium blink (250 ms)
//   Anxious  – fast blink (80 ms)
//   Friendly – solid on for 1.5 s then off
//
// IMPORTANT – Touch-capable pins on ESP32:
//   GPIO4 (T0) used here.  Consult the ESP32 Technical Reference Manual
//   for the full list of touch-capable GPIOs before changing TOUCH_PIN.
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
constexpr uint32_t FRIENDLY_WINDOW_MS = 1500;
// Blink period per state (ms): Idle, Excited, Anxious, Friendly=0(solid)
const uint32_t STATE_BLINK_MS[] = {1000, 250, 80, 0};
// -----------------------------------------------------------------------------

enum class MoodState : uint8_t { Idle, Excited, Anxious, Friendly };

float    arousal   = 0.0f;
float    affection = 0.0f;
float    anxiety   = 0.10f;
uint32_t lastTouchMs = 0;

int   touchBaseline = 0;
float touchFiltered = 0.0f;
bool  touchLatched  = false;

uint32_t lastTickMs  = 0;
uint32_t lastBlinkMs = 0;
bool     ledState    = false;
MoodState lastState  = MoodState::Idle;

float clamp01(float x) { return x < 0 ? 0 : (x > 1 ? 1 : x); }

void calibrateTouch() {
  delay(400); long s = 0;
  for (int i = 0; i < 50; i++) { s += touchRead(TOUCH_PIN); delay(10); }
  touchBaseline = (int)(s / 50); touchFiltered = (float)touchBaseline;
}

bool isTouched() {
  int raw = touchRead(TOUCH_PIN);
  touchFiltered = (1 - SMOOTH_ALPHA) * touchFiltered + SMOOTH_ALPHA * raw;
  float on = touchBaseline * TOUCH_ON_RATIO, off = touchBaseline * TOUCH_OFF_RATIO;
  if (!touchLatched) { if (touchFiltered < on)  touchLatched = true; }
  else               { if (touchFiltered > off) touchLatched = false; }
  return touchLatched;
}

MoodState chooseState(uint32_t now) {
  if (now - lastTouchMs < FRIENDLY_WINDOW_MS) return MoodState::Friendly;
  if (anxiety > 0.65f)  return MoodState::Anxious;
  if (arousal > 0.55f)  return MoodState::Excited;
  return MoodState::Idle;
}

void tickEmotions(float dt) {
  arousal   = clamp01(arousal   - 0.06f * dt);
  affection = clamp01(affection - 0.02f * dt);
  anxiety   = clamp01(anxiety   + 0.02f * dt - 0.03f * affection * dt);
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  calibrateTouch();
  lastTouchMs = millis() - 5000;
  Serial.println("06_choose_state_fsm v2 – touch + LED patterns");
}

void loop() {
  uint32_t now = millis();

  // Touch
  static bool lastT = false;
  bool t = isTouched();
  if (t && !lastT) {
    lastTouchMs = now;
    affection   = clamp01(affection + 0.30f);
    anxiety     = clamp01(anxiety   - 0.40f);
  }
  lastT = t;

  // Emotion tick
  if (now - lastTickMs >= TICK_MS) {
    float dt = (now - lastTickMs) / 1000.0f; lastTickMs = now;
    tickEmotions(dt);
  }

  MoodState s = chooseState(now);
  if (s != lastState) { lastState = s; Serial.print("STATE -> "); Serial.println((int)s); }

  // LED pattern
  uint32_t blinkMs = STATE_BLINK_MS[(int)s];
  if (blinkMs == 0) {
    // Friendly: solid on
    digitalWrite(LED_PIN, HIGH);
  } else {
    if (now - lastBlinkMs >= blinkMs) {
      lastBlinkMs = now;
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState ? HIGH : LOW);
    }
  }
}
