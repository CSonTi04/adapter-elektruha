// =============================================================================
// 11_anxius_lite – Variant 2: Minimal integrator (no PCF8574)
// =============================================================================
// Concept: Full anxious.ino integration minus PCF8574 and BLE.
// Uses:
//   - Real capacitive touch (GPIO4) for emotional input
//   - Built-in LED for visual state feedback
//   - LEDC tone on GPIO5 for audio feedback
//
// This is the largest single-board sketch before adding I2C peripherals.
//
// IMPORTANT – Touch-capable pins on ESP32:
//   GPIO4 (T0) is used here.  Consult the ESP32 Technical Reference Manual
//   for the complete list of touch-capable GPIOs.
//
// Wiring:
//   Wire/metal pad → GPIO4  (touch electrode)
//   Built-in LED on GPIO2
//   GPIO5 → speaker/piezo (or LM386 input)
// =============================================================================

// ---- Tuning knobs -----------------------------------------------------------
constexpr int   TOUCH_PIN       = 4;
constexpr int   LED_PIN         = 2;
constexpr int   AUDIO_PIN       = 5;
constexpr int   AUDIO_LEDC_CH   = 2;
constexpr int   AUDIO_LEDC_RES  = 8;
constexpr float AUDIO_VOLUME_SCALE = 0.35f;
constexpr bool  ENABLE_AUDIO    = true;
constexpr float TOUCH_ON_RATIO  = 0.85f;
constexpr float TOUCH_OFF_RATIO = 0.90f;
constexpr float SMOOTH_ALPHA    = 0.15f;
constexpr uint32_t EMOTION_TICK_MS  = 50;
constexpr uint32_t FRIENDLY_WIN_MS  = 1500;
constexpr uint32_t FAKE_BLE_MS      = 5000;
constexpr uint32_t LOG_MS           = 1500;
// -----------------------------------------------------------------------------

enum class MoodState : uint8_t { Idle, Excited, Anxious, Friendly };

struct Emotion {
  float arousal   = 0.0f;
  float affection = 0.0f;
  float anxiety   = 0.10f;
  uint32_t lastTouchMs     = 0;
  int      lastSeenDevices = 0;
} E;

// Touch
int   touchBaseline = 0;
float touchFiltered = 0.0f;
bool  touchLatched  = false;

// LED energy
float ledEnergy[8]  = {0};
uint32_t lastEnergyMs = 0;

// Audio
uint32_t audioNextEventMs = 0;
uint32_t audioEventEndMs  = 0;
float    audioFreqHz      = 0.0f;
uint8_t  audioDuty        = 0;

uint32_t lastTickMs    = 0;
uint32_t lastFakeBleMs = 0;
uint32_t lastLogMs     = 0;
MoodState lastState    = MoodState::Idle;

// Pattern state
int      chaseIdx    = 0;
uint32_t nextStepMs  = 0;

float clamp01(float x)        { return x < 0 ? 0 : (x > 1 ? 1 : x); }
float clamp01f(float x)       { return clamp01(x); }
float frand(float a, float b) { return a + (b - a) * random(0, 10000) / 10000.0f; }

// ---- Touch ------------------------------------------------------------------
void calibrateTouch() {
  delay(400); long s = 0;
  for (int i = 0; i < 50; i++) { s += touchRead(TOUCH_PIN); delay(10); }
  touchBaseline = (int)(s / 50); touchFiltered = (float)touchBaseline;
  Serial.print("touch baseline="); Serial.println(touchBaseline);
}

bool isTouched() {
  int raw = touchRead(TOUCH_PIN);
  touchFiltered = (1 - SMOOTH_ALPHA) * touchFiltered + SMOOTH_ALPHA * raw;
  float on = touchBaseline * TOUCH_ON_RATIO, off = touchBaseline * TOUCH_OFF_RATIO;
  if (!touchLatched) { if (touchFiltered < on)  touchLatched = true; }
  else               { if (touchFiltered > off) touchLatched = false; }
  return touchLatched;
}

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
  switch(s) {
    case MoodState::Idle: return "Idle"; case MoodState::Excited: return "Excited";
    case MoodState::Anxious: return "Anxious"; case MoodState::Friendly: return "Friendly";
  }
  return "?";
}

// ---- LED pattern (simplified) -----------------------------------------------
void decayEnergy(uint32_t now, float rate) {
  if (lastEnergyMs == 0) { lastEnergyMs = now; return; }
  float dt = (now - lastEnergyMs) / 1000.0f; lastEnergyMs = now;
  for (int i = 0; i < 8; i++) ledEnergy[i] = clamp01f(ledEnergy[i] - rate * dt);
}

void pulseLed(int idx, float amp) {
  if (idx >= 0 && idx < 8) ledEnergy[idx] = clamp01f(ledEnergy[idx] + amp);
}

void runPattern(MoodState s, uint32_t now) {
  float rate = (s == MoodState::Anxious) ? 2.0f :
               (s == MoodState::Excited) ? 1.5f : 0.8f;
  decayEnergy(now, rate);
  uint32_t stepMs = (s == MoodState::Excited) ? 100 :
                    (s == MoodState::Anxious)  ? 70  : 200;
  if (now >= nextStepMs) {
    nextStepMs = now + stepMs;
    if (s == MoodState::Anxious) pulseLed(random(0, 4), 0.5f + 0.4f * E.anxiety);
    else { pulseLed(chaseIdx, 0.5f); if (chaseIdx > 0) pulseLed(chaseIdx - 1, 0.2f); chaseIdx = (chaseIdx + 1) % 8; }
  }
  // Map overall energy to built-in LED blink rate
  float total = 0; for (int i = 0; i < 8; i++) total += ledEnergy[i]; total /= 8.0f;
  static uint32_t lastBlinkMs = 0; static bool ledSt = false;
  uint32_t blinkMs = (uint32_t)(800 - total * 720);
  if (now - lastBlinkMs >= blinkMs) { lastBlinkMs = now; ledSt = !ledSt; digitalWrite(LED_PIN, ledSt ? HIGH : LOW); }
}

// ---- Audio ------------------------------------------------------------------
void audioSilence()                       { if (ENABLE_AUDIO) ledcWrite(AUDIO_PIN, 0); }
void audioPlay(float hz, uint8_t duty)    {
  if (!ENABLE_AUDIO || hz < 1.0f) { audioSilence(); return; }
  uint8_t sc = (uint8_t)(duty * AUDIO_VOLUME_SCALE); if (sc == 0 && duty > 0) sc = 1;
  ledcWriteTone(AUDIO_PIN, (uint32_t)hz); ledcWrite(AUDIO_PIN, sc);
}

void audioTick(uint32_t now, MoodState s) {
  if (!ENABLE_AUDIO) return;
  if (now < audioEventEndMs) { audioPlay(audioFreqHz * (1.0f + frand(-0.006f, 0.006f)), audioDuty); return; }
  if (now < audioNextEventMs) { audioSilence(); return; }
  switch (s) {
    case MoodState::Idle:     audioFreqHz = 120.0f + frand(-6,6);    audioDuty = 20; audioEventEndMs = now+60;  audioNextEventMs = now+350; break;
    case MoodState::Excited:  audioFreqHz = 400.0f + frand(-40,60);  audioDuty = 70; audioEventEndMs = now+30;  audioNextEventMs = now+90;  break;
    case MoodState::Anxious:  audioFreqHz = 600.0f + frand(-120,220);audioDuty = 50; audioEventEndMs = now+15;  audioNextEventMs = now+60;  break;
    case MoodState::Friendly: audioFreqHz = 480.0f + frand(-12,12);  audioDuty = 50; audioEventEndMs = now+80;  audioNextEventMs = now+150; break;
  }
  audioPlay(audioFreqHz, audioDuty);
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  if (ENABLE_AUDIO) { ledcAttachChannel(AUDIO_PIN, 1000, AUDIO_LEDC_RES, AUDIO_LEDC_CH); ledcWrite(AUDIO_PIN, 0); }
  randomSeed(esp_random());
  calibrateTouch();
  E.lastTouchMs  = millis() - 5000;
  lastFakeBleMs  = millis();
  Serial.println("11_anxius_lite v2 – minimal (no PCF8574)");
}

void loop() {
  uint32_t now = millis();

  // Touch
  static bool lastT = false; bool t = isTouched();
  if (t && !lastT) { E.lastTouchMs = now; E.affection = clamp01(E.affection + 0.3f); E.anxiety = clamp01(E.anxiety - 0.4f); Serial.println("TOUCH"); }
  lastT = t;

  // Fake BLE
  if (now - lastFakeBleMs >= FAKE_BLE_MS) {
    lastFakeBleMs = now; E.lastSeenDevices = random(0, 10);
    E.arousal = clamp01(E.arousal + 0.1f * clamp01(E.lastSeenDevices / 10.0f));
  }

  // Emotion tick
  if (now - lastTickMs >= EMOTION_TICK_MS) { float dt = (now - lastTickMs) / 1000.0f; lastTickMs = now; tickEmotions(dt); }

  MoodState s = chooseState(now);
  if (s != lastState) { lastState = s; Serial.print("STATE -> "); Serial.println(stateName(s)); }

  runPattern(s, now);
  audioTick(now, s);

  if (now - lastLogMs >= LOG_MS) {
    lastLogMs = now;
    Serial.print("arous="); Serial.print(E.arousal, 2);
    Serial.print(" affec="); Serial.print(E.affection, 2);
    Serial.print(" anxty="); Serial.println(E.anxiety, 2);
  }
}
