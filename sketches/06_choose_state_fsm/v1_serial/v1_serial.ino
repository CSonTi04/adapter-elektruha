// =============================================================================
// 06_choose_state_fsm – Variant 1: Serial FSM (no hardware)
// =============================================================================
// Concept: Priority-based FSM for mood state selection.
//
// anxious.ino uses a simple priority check (not a table or transition graph)
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

// ---- LEARN: enum class (named set of choices) --------------------------------
// An "enum" (short for "enumeration") is a type that can only hold one of a
// fixed list of named values.
//
// Instead of storing 0 for Idle, 1 for Excited, 2 for Anxious, 3 for Friendly
// (which is easy to get wrong), we give each state a descriptive name.
//
// "enum class MoodState" means the names belong to MoodState — you must write
// MoodState::Idle rather than just Idle.  This avoids name clashes if another
// enum in your sketch happens to use the same word.
//
// ": uint8_t" means each value is stored as a single byte (0–255).
// Four states fit easily; using uint8_t saves a tiny bit of memory.
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

// ---- LEARN: const char* (text / string) -------------------------------------
// "const char*" is a pointer to a read-only sequence of characters.
// In plain English: it is a text string.
//
// "const" means we won't modify the text.
// "char" is the character type (one letter/symbol).
// "*" (the asterisk) means it is a POINTER — the variable holds the memory
//     address WHERE the first character lives, not the character itself.
//
// For now, think of "const char*" simply as "a text value."
// You don't need to understand pointers to use it for printing.
//
// The function returns one of the four string literals "Idle", "Excited", etc.
// depending on which MoodState is passed in.
// -----------------------------------------------------------------------------
const char* stateName(MoodState s) {
  // ---- LEARN: switch / case ------------------------------------------------
  // A "switch" statement is a cleaner way to write a chain of if/else when
  // you are comparing ONE variable against SEVERAL specific values.
  //
  // Format:
  //   switch (variable) {
  //     case VALUE_A:  do_something; break;
  //     case VALUE_B:  do_something; break;
  //     default:       do_something; break;   // runs if no case matched
  //   }
  //
  // "break" is CRITICAL — without it the code "falls through" into the next
  // case and keeps running.  Always add break at the end of each case
  // unless you specifically want fall-through behaviour.
  // -------------------------------------------------------------------------
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
  // ---- LEARN: Comparing enum values ----------------------------------------
  // "s != lastState" checks if the current state is different from the last
  // one we remember.  "!=" means "not equal to."
  // If it is different we print the new state name and update lastState.
  // This way we only print when a CHANGE happens, not every loop iteration.
  // --------------------------------------------------------------------------
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
