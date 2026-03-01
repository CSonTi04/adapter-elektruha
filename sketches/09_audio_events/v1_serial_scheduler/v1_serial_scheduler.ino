// =============================================================================
// 09_audio_events – Variant 1: Serial print scheduler (no hardware)
// =============================================================================
// Concept: The audio event scheduler from anxius.ino without any hardware.
//
// anxius.ino generates sound as *events*: short tone bursts with gaps,
// not a continuous wave.  Each state has its own event profile.
// The scheduler has three phases:
//   Phase A: sustain active event (with tiny drift for organic feel)
//   Phase B: silent gap between events
//   Phase C: pick next event from current mood
//
// This variant prints what the scheduler *would* play, to Serial.
// Use 't', 'b', 'r' to change the active mood state.
//
// Wiring: none – open Serial Monitor at 115200 baud.
// =============================================================================

// ---- Tuning knobs -----------------------------------------------------------
constexpr uint32_t PRINT_MS = 100;  // how often to print status
// -----------------------------------------------------------------------------

enum class MoodState : uint8_t { Idle, Excited, Anxious, Friendly };

MoodState currentState = MoodState::Idle;

static uint32_t audioNextEventMs = 0;
static uint32_t audioEventEndMs  = 0;
static float    audioFreqHz      = 0.0f;
static uint8_t  audioDuty        = 0;

uint32_t lastPrintMs = 0;

float clamp01(float x) { return x < 0 ? 0 : (x > 1 ? 1 : x); }
float frand(float a, float b) { return a + (b - a) * random(0, 10000) / 10000.0f; }

void audioTick(uint32_t now) {
  // Phase A: sustain
  if (now < audioEventEndMs) {
    float drift = 1.0f + frand(-0.006f, 0.006f);
    audioFreqHz *= drift;
    return;
  }
  // Phase B: silence
  if (now < audioNextEventMs) {
    audioFreqHz = 0;
    return;
  }
  // Phase C: schedule next event
  switch (currentState) {
    case MoodState::Idle: {
      audioFreqHz = 120.0f + frand(-6.0f, 6.0f);
      audioDuty   = 20;
      audioEventEndMs  = now + (uint32_t)(55 + frand(0, 35));
      audioNextEventMs = now + (uint32_t)(260 + frand(0, 380));
      break;
    }
    case MoodState::Excited: {
      audioFreqHz = 400.0f + frand(-40.0f, 60.0f);
      audioDuty   = 60;
      audioEventEndMs  = now + (uint32_t)(30 + frand(0, 25));
      audioNextEventMs = now + (uint32_t)(80 + frand(0, 60));
      break;
    }
    case MoodState::Anxious: {
      audioFreqHz = 600.0f + frand(-120.0f, 220.0f);
      audioDuty   = 40;
      audioEventEndMs  = now + (uint32_t)(12 + frand(0, 35));
      audioNextEventMs = now + (uint32_t)(30 + frand(0, 180));
      break;
    }
    case MoodState::Friendly: {
      audioFreqHz = 480.0f + frand(-12.0f, 12.0f);
      audioDuty   = 50;
      audioEventEndMs  = now + (uint32_t)(60 + frand(0, 40));
      audioNextEventMs = now + (uint32_t)(120 + frand(0, 120));
      break;
    }
  }
  Serial.print("[AUDIO] freq="); Serial.print((int)audioFreqHz);
  Serial.print(" duty="); Serial.print(audioDuty);
  Serial.print(" for="); Serial.print(audioEventEndMs - now);
  Serial.print("ms  gap="); Serial.print(audioNextEventMs - audioEventEndMs);
  Serial.println("ms");
}

void setup() {
  Serial.begin(115200);
  randomSeed(42);
  Serial.println("09_audio_events v1 – Serial scheduler");
  Serial.println("Commands: 'i'=Idle 'e'=Excited 'a'=Anxious 'f'=Friendly");
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'i') { currentState = MoodState::Idle;     Serial.println(">> Idle"); }
    if (c == 'e') { currentState = MoodState::Excited;  Serial.println(">> Excited"); }
    if (c == 'a') { currentState = MoodState::Anxious;  Serial.println(">> Anxious"); }
    if (c == 'f') { currentState = MoodState::Friendly; Serial.println(">> Friendly"); }
  }

  uint32_t now = millis();
  audioTick(now);

  if (now - lastPrintMs >= PRINT_MS) {
    lastPrintMs = now;
    Serial.print("  playing="); Serial.print(now < audioEventEndMs ? "YES" : "no ");
    Serial.print("  freq="); Serial.print((int)audioFreqHz);
    Serial.println("Hz");
  }
}
