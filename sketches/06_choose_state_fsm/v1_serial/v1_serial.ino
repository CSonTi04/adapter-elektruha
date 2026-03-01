// =============================================================================
// 06_choose_state_fsm – Variant 1: Serial FSM (no hardware)
// =============================================================================
// Concept: Priority-based FSM for mood state selection.
//
// anxius.ino uses a simple priority check (not a table or transition graph)
// to select one of four states from continuous emotion values:
//   1) Friendly  – recent touch (highest priority)
//   2) Anxious   – high anxiety
//   3) Excited   – high arousal
//   4) Idle      – default
//
// Type commands in Serial to inject events and watch the state change.
// Commands: 't'=touch, 'b'=BLE crowd, 'r'=reset
//
// Wiring: none – open Serial Monitor at 115200 baud.
// =============================================================================

// ---- Tuning knobs -----------------------------------------------------------
constexpr uint32_t TICK_MS   = 50;
constexpr uint32_t PRINT_MS  = 700;
constexpr float FRIENDLY_WINDOW_MS = 1500; // ms after touch to stay Friendly
// -----------------------------------------------------------------------------

enum class MoodState : uint8_t { Idle, Excited, Anxious, Friendly };

float    arousal   = 0.0f;
float    affection = 0.0f;
float    anxiety   = 0.10f;
uint32_t lastTouchMs = 0;

uint32_t lastTickMs  = 0;
uint32_t lastPrintMs = 0;
MoodState lastState = MoodState::Idle;

float clamp01(float x) { return x < 0 ? 0 : (x > 1 ? 1 : x); }

MoodState chooseState(uint32_t now) {
  if (now - lastTouchMs < (uint32_t)FRIENDLY_WINDOW_MS) return MoodState::Friendly;
  if (anxiety > 0.65f)  return MoodState::Anxious;
  if (arousal > 0.55f)  return MoodState::Excited;
  return MoodState::Idle;
}

const char* stateName(MoodState s) {
  switch (s) {
    case MoodState::Idle:     return "Idle";
    case MoodState::Excited:  return "Excited";
    case MoodState::Anxious:  return "Anxious";
    case MoodState::Friendly: return "Friendly";
  }
  return "?";
}

void tickEmotions(float dt) {
  arousal   = clamp01(arousal   - 0.06f * dt);
  affection = clamp01(affection - 0.02f * dt);
  anxiety   = clamp01(anxiety   + 0.02f * dt - 0.03f * affection * dt);
}

void setup() {
  Serial.begin(115200);
  lastTouchMs = millis() - 5000; // start outside friendly window
  Serial.println("06_choose_state_fsm v1 – Serial FSM");
  Serial.println("Commands: 't'=touch  'b'=BLE crowd  'r'=reset");
}

void loop() {
  uint32_t now = millis();

  while (Serial.available()) {
    char c = Serial.read();
    if (c == 't') {
      lastTouchMs = now;
      affection   = clamp01(affection + 0.30f);
      anxiety     = clamp01(anxiety   - 0.40f);
      Serial.println(">> TOUCH");
    }
    if (c == 'b') {
      arousal = clamp01(arousal + 0.20f);
      Serial.println(">> BLE crowd");
    }
    if (c == 'r') {
      arousal = affection = anxiety = 0;
      lastTouchMs = now - 5000;
      Serial.println(">> RESET");
    }
  }

  if (now - lastTickMs >= TICK_MS) {
    float dt = (now - lastTickMs) / 1000.0f; lastTickMs = now;
    tickEmotions(dt);
  }

  MoodState s = chooseState(now);
  if (s != lastState) {
    lastState = s;
    Serial.print("STATE -> "); Serial.println(stateName(s));
  }

  if (now - lastPrintMs >= PRINT_MS) {
    lastPrintMs = now;
    Serial.print("arous="); Serial.print(arousal, 2);
    Serial.print(" anxty="); Serial.print(anxiety, 2);
    Serial.print(" state="); Serial.println(stateName(s));
  }
}
