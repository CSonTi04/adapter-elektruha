// =============================================================================
// 11_anxius_lite – Variant 1: Serial-only integrator
// =============================================================================
// Concept: A minimal but complete version of anxious.ino running entirely in
// Serial – no hardware required.
//
// All major subsystems are present:
//   - Emotional model (arousal / affection / anxiety)
//   - Priority-based state FSM
//   - Simplified LED "pattern" → printed as ASCII bars
//   - Audio "events" → printed as text
//   - Simulated BLE random walk
//
// Send commands over Serial: 't'=touch  'b'=BLE burst  'r'=reset
//
// Wiring: none – open Serial Monitor at 115200 baud.
// =============================================================================

// ---- Tuning knobs -----------------------------------------------------------
constexpr uint32_t EMOTION_TICK_MS   = 50;
constexpr uint32_t PRINT_MS          = 800;
constexpr uint32_t FAKE_BLE_MS       = 4000;
constexpr uint32_t FRIENDLY_WIN_MS   = 1500;
// -----------------------------------------------------------------------------

enum class MoodState : uint8_t { Idle, Excited, Anxious, Friendly };

struct Emotion {
  float arousal   = 0.0f;
  float affection = 0.0f;
  float anxiety   = 0.10f;
  uint32_t lastTouchMs     = 0;
  int      lastSeenDevices = 0;
} E;

// Simplified LED energy buffer (8 channels)
float ledEnergy[8] = {0};
uint32_t lastEnergyMs = 0;

// Audio scheduler
uint32_t audioNextEventMs = 0;
uint32_t audioEventEndMs  = 0;
float    audioFreqHz      = 0.0f;

uint32_t lastTickMs  = 0;
uint32_t lastPrintMs = 0;
uint32_t lastFakeBleMs = 0;
MoodState lastState  = MoodState::Idle;

float clamp01(float x)        { return x < 0 ? 0 : (x > 1 ? 1 : x); }
float frand(float a, float b) { return a + (b - a) * random(0, 10000) / 10000.0f; }
float clamp01f(float x)       { return clamp01(x); }

// ---- Emotion ----------------------------------------------------------------
void tickEmotions(float dt) {
  E.arousal   = clamp01(E.arousal   - 0.06f * dt);
  E.affection = clamp01(E.affection - 0.02f * dt);
  E.anxiety   = clamp01(E.anxiety   + 0.02f * dt - 0.03f * E.affection * dt);
}

// ---- State ------------------------------------------------------------------
MoodState chooseState(uint32_t now) {
  if (now - E.lastTouchMs < FRIENDLY_WIN_MS) return MoodState::Friendly;
  if (E.anxiety > 0.65f) return MoodState::Anxious;
  if (E.arousal > 0.55f) return MoodState::Excited;
  return MoodState::Idle;
}

const char* stateName(MoodState s) {
  switch(s){ case MoodState::Idle: return "Idle"; case MoodState::Excited: return "Excited";
             case MoodState::Anxious: return "Anxious"; case MoodState::Friendly: return "Friendly"; }
  return "?";
}

// ---- LED energy buffer (simplified pattern) ---------------------------------
void decayEnergy(uint32_t now, float rate) {
  if (lastEnergyMs == 0) { lastEnergyMs = now; return; }
  float dt = (now - lastEnergyMs) / 1000.0f; lastEnergyMs = now;
  for (int i = 0; i < 8; i++) ledEnergy[i] = clamp01f(ledEnergy[i] - rate * dt);
}

void pulseLed(int idx, float amp) {
  if (idx >= 0 && idx < 8) ledEnergy[idx] = clamp01f(ledEnergy[idx] + amp);
}

static int chaseIdx = 0;
static uint32_t nextStepMs = 0;

void runPattern(MoodState s, uint32_t now) {
  float rate = (s == MoodState::Anxious) ? 2.0f :
               (s == MoodState::Excited) ? 1.5f : 0.8f;
  decayEnergy(now, rate);
  uint32_t stepMs = (s == MoodState::Excited) ? 100 :
                    (s == MoodState::Anxious)  ? 70  : 200;
  if (now >= nextStepMs) {
    nextStepMs = now + stepMs;
    if (s == MoodState::Anxious) {
      pulseLed(random(0, 4), 0.5f + 0.4f * E.anxiety);
    } else {
      pulseLed(chaseIdx, 0.5f);
      if (chaseIdx > 0) pulseLed(chaseIdx - 1, 0.2f);
      chaseIdx = (chaseIdx + 1) % 8;
    }
  }
}

// ---- Audio scheduler (simplified) ------------------------------------------
void audioTick(uint32_t now, MoodState s) {
  if (now < audioEventEndMs) return;
  if (now < audioNextEventMs) { audioFreqHz = 0; return; }
  float freq = (s == MoodState::Idle)     ? 120.0f + frand(-6, 6)   :
               (s == MoodState::Excited)  ? 400.0f + frand(-40, 60) :
               (s == MoodState::Anxious)  ? 600.0f + frand(-120, 220) :
                                            480.0f + frand(-12, 12);
  audioFreqHz      = freq;
  audioEventEndMs  = now + (uint32_t)(30 + frand(0, 60));
  audioNextEventMs = now + (uint32_t)(80 + frand(0, 300));
  Serial.print("[AUDIO] "); Serial.print((int)freq); Serial.println("Hz");
}

// ---- Print ------------------------------------------------------------------
void printBars() {
  Serial.print("State: "); Serial.println(stateName(chooseState(millis())));
  Serial.print("arous="); Serial.print(E.arousal, 2);
  Serial.print(" affec="); Serial.print(E.affection, 2);
  Serial.print(" anxty="); Serial.println(E.anxiety, 2);
  for (int i = 0; i < 8; i++) {
    Serial.print("L"); Serial.print(i); Serial.print("[");
    int b = (int)(ledEnergy[i] * 10);
    for (int j = 0; j < 10; j++) Serial.print(j < b ? "#" : " ");
    Serial.println("]");
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  randomSeed(42);
  E.lastTouchMs = millis() - 5000;
  Serial.println("11_anxius_lite v1 – Serial-only integrator");
  Serial.println("Commands: 't'=touch  'b'=BLE burst  'r'=reset");
}

void loop() {
  uint32_t now = millis();

  // Serial commands
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 't') { E.lastTouchMs = now; E.affection = clamp01(E.affection + 0.3f); E.anxiety = clamp01(E.anxiety - 0.4f); }
    if (c == 'b') { E.lastSeenDevices = 8; E.arousal = clamp01(E.arousal + 0.15f); }
    if (c == 'r') { E.arousal = E.affection = E.anxiety = 0; }
  }

  // Fake BLE
  if (now - lastFakeBleMs >= FAKE_BLE_MS) {
    lastFakeBleMs = now;
    E.lastSeenDevices = random(0, 8);
    E.arousal = clamp01(E.arousal + 0.1f * clamp01(E.lastSeenDevices / 10.0f));
  }

  // Emotion tick
  if (now - lastTickMs >= EMOTION_TICK_MS) {
    float dt = (now - lastTickMs) / 1000.0f; lastTickMs = now;
    tickEmotions(dt);
  }

  MoodState s = chooseState(now);
  if (s != lastState) {
    lastState = s;
    Serial.print("STATE -> "); Serial.println(stateName(s));
  }

  runPattern(s, now);
  audioTick(now, s);

  if (now - lastPrintMs >= PRINT_MS) { lastPrintMs = now; printBars(); }
}
