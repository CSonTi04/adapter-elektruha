// =============================================================================
// 06_choose_state_fsm – Variant 3: Full pre-I2C
// =============================================================================
// Concept: Complete FSM + emotions in the full build context.
// All emotion logic and state selection is duplicated here (not included from
// another file) so this sketch compiles independently, as Arduino requires.
// I2C / PCF8574 outputs are not yet added – those come in sketch 07+.
//
// IMPORTANT – Touch-capable pins on ESP32:
//   GPIO4 (T0) is used here.  See ESP32 Technical Reference Manual for
//   the full list of touch-capable GPIOs.
//
// Wiring: wire/metal pad → GPIO4;  built-in LED on GPIO2.
// =============================================================================

// ---- Tuning knobs -----------------------------------------------------------
constexpr int   TOUCH_PIN       = 4;
constexpr int   LED_PIN         = 2;
constexpr float TOUCH_ON_RATIO  = 0.85f;
constexpr float TOUCH_OFF_RATIO = 0.90f;
constexpr float SMOOTH_ALPHA    = 0.15f;
constexpr uint32_t EMOTION_TICK_MS      = 50;
constexpr uint32_t PRINT_MS             = 1000;
constexpr uint32_t FRIENDLY_WINDOW_MS   = 1500;
constexpr uint32_t FAKE_BLE_INTERVAL_MS = 7000; // pretend BLE crowd event
// -----------------------------------------------------------------------------

enum class MoodState : uint8_t { Idle, Excited, Anxious, Friendly };

struct Emotion {
  float arousal   = 0.0f;
  float affection = 0.0f;
  float anxiety   = 0.10f;
  uint32_t lastTouchMs     = 0;
  int      lastSeenDevices = 0;
} E;

int   touchBaseline = 0;
float touchFiltered = 0.0f;
bool  touchLatched  = false;
uint32_t lastTickMs     = 0;
uint32_t lastPrintMs    = 0;
uint32_t lastFakeBleMs  = 0;
MoodState lastState     = MoodState::Idle;

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

MoodState chooseState(uint32_t now) {
  if (now - E.lastTouchMs < FRIENDLY_WINDOW_MS) return MoodState::Friendly;
  if (E.anxiety > 0.65f)  return MoodState::Anxious;
  if (E.arousal > 0.55f)  return MoodState::Excited;
  return MoodState::Idle;
}

const char* stateName(MoodState s) {
  switch(s) {
    case MoodState::Idle:     return "Idle";
    case MoodState::Excited:  return "Excited";
    case MoodState::Anxious:  return "Anxious";
    case MoodState::Friendly: return "Friendly";
  }
  return "?";
}

void tickEmotions(float dt) {
  E.arousal   = clamp01(E.arousal   - 0.06f * dt);
  E.affection = clamp01(E.affection - 0.02f * dt);
  E.anxiety   = clamp01(E.anxiety   + 0.02f * dt - 0.03f * E.affection * dt);
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  randomSeed(esp_random());
  calibrateTouch();
  E.lastTouchMs  = millis() - 5000;
  lastFakeBleMs  = millis();
  Serial.println("06_choose_state_fsm v3 – full pre-I2C");
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
    Serial.println("TOUCH!");
  }
  lastT = t;
  digitalWrite(LED_PIN, t ? HIGH : LOW);

  // Fake BLE
  if (now - lastFakeBleMs >= FAKE_BLE_INTERVAL_MS) {
    lastFakeBleMs = now;
    E.lastSeenDevices = random(0, 10);
    E.arousal = clamp01(E.arousal + 0.10f * clamp01(E.lastSeenDevices / 10.0f));
  }

  // Emotion tick
  if (now - lastTickMs >= EMOTION_TICK_MS) {
    float dt = (now - lastTickMs) / 1000.0f; lastTickMs = now;
    tickEmotions(dt);
  }

  // State
  MoodState s = chooseState(now);
  if (s != lastState) {
    lastState = s;
    Serial.print("STATE -> "); Serial.println(stateName(s));
  }

  // Log
  if (now - lastPrintMs >= PRINT_MS) {
    lastPrintMs = now;
    Serial.print("arous="); Serial.print(E.arousal, 2);
    Serial.print(" anxty="); Serial.print(E.anxiety, 2);
    Serial.print(" state="); Serial.println(stateName(s));
  }
}
