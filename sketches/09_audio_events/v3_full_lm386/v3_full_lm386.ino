// =============================================================================
// 09_audio_events – Variant 3: Full LM386 LEDC with volume scale
// =============================================================================
// Concept: Full audio event scheduler matching anxious.ino exactly.
// Includes the complete per-state event profiles (idle, excited, anxious,
// friendly), the global volume scale trim, and micro-drift for organic feel.
//
// The LM386 amplifier is driven via a coupling capacitor from GPIO5.
// Volume is controlled by AUDIO_VOLUME_SCALE (0.0..1.0).
//
// Mood state auto-cycles every STATE_CYCLE_MS so you can hear all states.
// You can also send state commands over Serial: 'i', 'e', 'a', 'f'.
//
// Requires ESP32 Arduino core 3.x.
//
// Wiring:
//   GPIO5 → 10 µF coupling cap → LM386 pin 3 (IN-)
//   LM386 pin 6 → 5–12 V supply
//   LM386 pin 5 → 250 µF cap → speaker → GND
//   LM386 pin 2 → GND
//   (See LM386 datasheet for full schematic)
// =============================================================================

// ---- Tuning knobs -----------------------------------------------------------
constexpr int     AUDIO_PIN         = 5;      // matches anxious.ino
constexpr int     AUDIO_LEDC_CH     = 2;
constexpr int     AUDIO_LEDC_RES    = 8;      // 8-bit duty
constexpr float   AUDIO_VOLUME_SCALE = 0.35f; // global trim (0..1)
constexpr bool    ENABLE_AUDIO      = true;   // <<< TOGGLE THIS
constexpr uint32_t STATE_CYCLE_MS   = 5000;   // auto-cycle interval
// -----------------------------------------------------------------------------

enum class MoodState : uint8_t { Idle, Excited, Anxious, Friendly };

// Simulated emotion values (no real sensors in this sketch)
float arousal   = 0.5f;
float anxiety   = 0.5f;
float affection = 0.3f;
int   lastSeenDevices = 3;
uint32_t lastTouchMs  = 0;

static uint32_t audioNextEventMs = 0;
static uint32_t audioEventEndMs  = 0;
static float    audioFreqHz      = 0.0f;
static uint8_t  audioDuty        = 0;

MoodState currentState   = MoodState::Idle;
uint32_t  lastCycleMs    = 0;

float clamp01(float x)        { return x < 0 ? 0 : (x > 1 ? 1 : x); }
float frand(float a, float b) { return a + (b - a) * random(0, 10000) / 10000.0f; }

void audioSilence() {
  if (!ENABLE_AUDIO) return;
  ledcWrite(AUDIO_PIN, 0);
}

void audioTone(float hz, uint8_t duty) {
  if (!ENABLE_AUDIO || hz < 1.0f || duty == 0) { audioSilence(); return; }
  uint8_t scaled = (uint8_t)(duty * AUDIO_VOLUME_SCALE);
  if (scaled == 0 && duty > 0) scaled = 1;
  ledcWriteTone(AUDIO_PIN, (uint32_t)hz);
  ledcWrite(AUDIO_PIN, scaled);
}

void audioTick(uint32_t now) {
  if (!ENABLE_AUDIO) return;

  // Phase A: sustain with micro-drift
  if (now < audioEventEndMs) {
    float drift = 1.0f + frand(-0.006f, 0.006f);
    audioTone(audioFreqHz * drift, audioDuty);
    return;
  }
  // Phase B: silence
  if (now < audioNextEventMs) { audioSilence(); return; }

  // Phase C: schedule next event
  switch (currentState) {
    case MoodState::Idle: {
      float base = 120.0f + 40.0f * (1.0f - affection);
      audioFreqHz = base + frand(-6.0f, 6.0f);
      audioDuty   = (uint8_t)(18 + 22 * (1.0f - anxiety));
      audioEventEndMs  = now + (uint32_t)(55 + frand(0, 35));
      audioNextEventMs = now + (uint32_t)(260 + frand(0, 380));
      break;
    }
    case MoodState::Excited: {
      float crowd = clamp01(lastSeenDevices / 18.0f);
      float f = 260.0f + 520.0f * arousal + 220.0f * crowd;
      audioFreqHz = f + frand(-40.0f, 60.0f);
      audioDuty   = (uint8_t)(28 + 70 * arousal);
      uint32_t onMs = (uint32_t)(25 + 55 * (1.0f - arousal) + frand(0, 25));
      audioEventEndMs  = now + onMs;
      audioNextEventMs = now + (uint32_t)(70 + 140 * (1.0f - arousal) + frand(0, 60));
      break;
    }
    case MoodState::Anxious: {
      audioFreqHz = 420.0f + 900.0f * anxiety + frand(-120.0f, 220.0f);
      audioDuty   = (uint8_t)(20 + 60 * anxiety);
      audioEventEndMs  = now + (uint32_t)(12 + frand(0, 35));
      audioNextEventMs = now + (uint32_t)(25 + 220 * (1.0f - anxiety) + frand(0, 180));
      break;
    }
    case MoodState::Friendly: {
      uint32_t dt = now - lastTouchMs;
      float hold = clamp01(1.0f - (float)dt / 1200.0f);
      float startF = 520.0f + 240.0f * hold;
      float endF   = 220.0f + 120.0f * (1.0f - hold);
      float t = clamp01(dt / 1600.0f);
      audioFreqHz = (1.0f - t) * startF + t * endF + frand(-12.0f, 12.0f);
      audioDuty   = (uint8_t)(30 + 70 * hold);
      audioEventEndMs  = now + (uint32_t)(60 + 90 * hold + frand(0, 40));
      audioNextEventMs = now + (uint32_t)(110 + 200 * (1.0f - hold) + frand(0, 120));
      break;
    }
  }
  audioTone(audioFreqHz, audioDuty);
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

void setup() {
  Serial.begin(115200);
  if (ENABLE_AUDIO) {
    ledcAttachChannel(AUDIO_PIN, 1000, AUDIO_LEDC_RES, AUDIO_LEDC_CH);
    ledcWrite(AUDIO_PIN, 0);
  }
  randomSeed(esp_random());
  lastTouchMs = millis() - 5000;
  lastCycleMs = millis();
  Serial.println("09_audio_events v3 – full LM386 LEDC");
  Serial.println("Commands: 'i'=Idle 'e'=Excited 'a'=Anxious 'f'=Friendly");
}

void loop() {
  uint32_t now = millis();

  // Serial state override
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'i') { currentState = MoodState::Idle;     lastCycleMs = now; Serial.println(">> Idle"); }
    if (c == 'e') { currentState = MoodState::Excited;  lastCycleMs = now; Serial.println(">> Excited"); }
    if (c == 'a') { currentState = MoodState::Anxious;  lastCycleMs = now; Serial.println(">> Anxious"); }
    if (c == 'f') { currentState = MoodState::Friendly; lastTouchMs = now; lastCycleMs = now; Serial.println(">> Friendly"); }
  }

  // Auto-cycle
  if (now - lastCycleMs >= STATE_CYCLE_MS) {
    lastCycleMs = now;
    currentState = (MoodState)(((int)currentState + 1) % 4);
    if (currentState == MoodState::Friendly) lastTouchMs = now;
    Serial.print("STATE -> "); Serial.println(stateName(currentState));
  }

  audioTick(now);
}
