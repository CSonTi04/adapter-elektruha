// =============================================================================
// 09_audio_events – Variant 2: LEDC tone on GPIO5
// =============================================================================
// Concept: Same event scheduler but outputting real tones via ESP32 LEDC.
//
// anxius.ino uses ESP32 LEDC (hardware PWM) to generate tones for the LM386
// audio amplifier.  ledcWriteTone() sets the frequency; ledcWrite() sets the
// duty (volume).
//
// This sketch uses GPIO5 (AUDIO_PIN) to match anxius.ino.  You can connect
// a small speaker or piezo directly to GPIO5 + GND for basic testing,
// or route through an LM386 module for proper amplification.
//
// Requires ESP32 Arduino core 3.x (uses ledcAttachChannel / ledcWriteTone).
//
// Wiring:
//   GPIO5 → speaker/piezo positive terminal → negative → GND
//   (for better output: GPIO5 → coupling cap → LM386 input)
// =============================================================================

// ---- Tuning knobs -----------------------------------------------------------
constexpr int  AUDIO_PIN          = 5;    // must match anxius.ino
constexpr int  AUDIO_LEDC_CH      = 2;
constexpr int  AUDIO_LEDC_RES     = 8;    // 8-bit duty (0..255)
constexpr float AUDIO_VOLUME_SCALE = 0.35f; // global volume trim
// State cycle duration (ms) – cycles through states automatically
constexpr uint32_t STATE_CYCLE_MS  = 4000;
// -----------------------------------------------------------------------------

enum class MoodState : uint8_t { Idle, Excited, Anxious, Friendly };

static uint32_t audioNextEventMs = 0;
static uint32_t audioEventEndMs  = 0;
static float    audioFreqHz      = 0.0f;
static uint8_t  audioDuty        = 0;

uint32_t lastStateCycleMs = 0;
MoodState currentState    = MoodState::Idle;

float clamp01(float x)         { return x < 0 ? 0 : (x > 1 ? 1 : x); }
float frand(float a, float b)  { return a + (b - a) * random(0, 10000) / 10000.0f; }

void audioSilence() { ledcWrite(AUDIO_PIN, 0); }

void audioTone(float hz, uint8_t duty) {
  if (hz < 1.0f || duty == 0) { audioSilence(); return; }
  uint8_t scaled = (uint8_t)(duty * AUDIO_VOLUME_SCALE);
  if (scaled == 0 && duty > 0) scaled = 1;
  ledcWriteTone(AUDIO_PIN, (uint32_t)hz);
  ledcWrite(AUDIO_PIN, scaled);
}

void audioTick(uint32_t now) {
  if (now < audioEventEndMs) {
    float drift = 1.0f + frand(-0.006f, 0.006f);
    audioTone(audioFreqHz * drift, audioDuty);
    return;
  }
  if (now < audioNextEventMs) { audioSilence(); return; }

  switch (currentState) {
    case MoodState::Idle:
      audioFreqHz = 120.0f + frand(-6.0f, 6.0f);
      audioDuty   = 20;
      audioEventEndMs  = now + (uint32_t)(55 + frand(0, 35));
      audioNextEventMs = now + (uint32_t)(260 + frand(0, 380));
      break;
    case MoodState::Excited:
      audioFreqHz = 400.0f + frand(-40.0f, 60.0f);
      audioDuty   = 80;
      audioEventEndMs  = now + (uint32_t)(30 + frand(0, 25));
      audioNextEventMs = now + (uint32_t)(80 + frand(0, 60));
      break;
    case MoodState::Anxious:
      audioFreqHz = 600.0f + frand(-120.0f, 220.0f);
      audioDuty   = 55;
      audioEventEndMs  = now + (uint32_t)(12 + frand(0, 35));
      audioNextEventMs = now + (uint32_t)(30 + frand(0, 180));
      break;
    case MoodState::Friendly:
      audioFreqHz = 480.0f + frand(-12.0f, 12.0f);
      audioDuty   = 50;
      audioEventEndMs  = now + (uint32_t)(60 + frand(0, 40));
      audioNextEventMs = now + (uint32_t)(120 + frand(0, 120));
      break;
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
  ledcAttachChannel(AUDIO_PIN, 1000, AUDIO_LEDC_RES, AUDIO_LEDC_CH);
  ledcWrite(AUDIO_PIN, 0);
  randomSeed(esp_random());
  lastStateCycleMs = millis();
  Serial.println("09_audio_events v2 – LEDC tone GPIO5");
}

void loop() {
  uint32_t now = millis();

  // Auto-cycle through states
  if (now - lastStateCycleMs >= STATE_CYCLE_MS) {
    lastStateCycleMs = now;
    currentState = (MoodState)(((int)currentState + 1) % 4);
    Serial.print("STATE -> "); Serial.println(stateName(currentState));
  }

  audioTick(now);
}
