// =============================================================================
// 12_anxius_lite_pwm_upgrade – Variant 3: Full integrator with ENABLE_PWM
// =============================================================================
// Concept: Adds the time-sliced PWM output mode to the full integrator from
// sketch 11 v3.  Toggle ENABLE_PWM to switch between binary ON/OFF and
// smooth time-sliced brightness, matching anxious.ino's dual-mode output.
//
// The full LED pattern engine from anxious.ino (idle/excited/anxious/friendly
// state machines) is also included here to complete the curriculum.
//
// ENABLE_PWM = false → computeMaskBinary()  – clear ON/OFF (default)
// ENABLE_PWM = true  → computeMask()        – sliced PWM brightness
//
// IMPORTANT – Touch-capable pins on ESP32:
//   GPIO4 (T0) is used here.  See ESP32 Technical Reference Manual for all
//   touch-capable GPIOs.
//
// Wiring:
//   Touch: wire/pad → GPIO4
//   I2C:   SDA=GPIO21  SCL=GPIO22  (4.7 kΩ pull-ups to 3.3 V)
//   PCF8574 P0–P7 → 220 Ω → LED active-low  (A0/A1/A2=GND → 0x20)
//   Audio:  GPIO5 → coupling cap → LM386 input
//
// Libraries: "PCF8574" by Renzo Mischianti
// =============================================================================

#include <Wire.h>
#include <PCF8574.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <math.h>

// ================= OUTPUT MODE =================
constexpr bool ENABLE_PWM = false;   // <<< TOGGLE THIS
// ===============================================

// ---- Tuning knobs -----------------------------------------------------------
constexpr int   TOUCH_PIN        = 4;
constexpr int   AUDIO_PIN        = 5;
constexpr int   AUDIO_LEDC_CH    = 2;
constexpr int   AUDIO_LEDC_RES   = 8;
constexpr float AUDIO_VOLUME_SCALE = 0.35f;
constexpr bool  ENABLE_AUDIO     = true;
constexpr int   SDA_PIN          = 21;
constexpr int   SCL_PIN          = 22;
constexpr uint8_t PCF_ADDR       = 0x20;
constexpr float TOUCH_ON_RATIO   = 0.85f;
constexpr float TOUCH_OFF_RATIO  = 0.90f;
constexpr float SMOOTH_ALPHA     = 0.15f;
constexpr float BINARY_THRESHOLD = 0.22f;
constexpr uint32_t EMOTION_TICK_MS   = 50;
constexpr uint32_t BINARY_WRITE_MS   = 8;
constexpr uint32_t FRIENDLY_WIN_MS   = 1500;
constexpr uint32_t BLE_SCAN_EVERY_MS = 2500;
constexpr uint32_t BLE_SCAN_FOR_MS   = 3000;
constexpr uint32_t LOG_MS            = 1000;
// PWM settings
constexpr uint32_t PWM_PERIOD_US = 4000;
constexpr uint8_t  PWM_LEVELS    = 16;
constexpr uint32_t PWM_SLICE_US  = PWM_PERIOD_US / PWM_LEVELS;
// LED groups (Group A = impulse, Group B = response, mirrors anxious.ino)
constexpr int LEDA_FIRST = 0, LEDA_LAST = 3;
constexpr int LEDB_FIRST = 4, LEDB_LAST = 7;
// -----------------------------------------------------------------------------

PCF8574 pcf(PCF_ADDR);

// ---- Enums / structs --------------------------------------------------------
enum class MoodState : uint8_t { Idle, Excited, Anxious, Friendly };

struct Emotion {
  float arousal   = 0.0f;
  float affection = 0.0f;
  float anxiety   = 0.10f;
  uint32_t lastTouchMs     = 0;
  int      lastSeenDevices = 0;
} E;

// ---- Helpers ----------------------------------------------------------------
float clamp01(float x)        { return x < 0 ? 0 : (x > 1 ? 1 : x); }
float clamp01f(float x)       { return clamp01(x); }
float frand(float a, float b) { return a + (b - a) * random(0, 10000) / 10000.0f; }

static inline uint32_t jitterMs(uint32_t base, uint32_t jMin, uint32_t jMax) {
  int j = (int)random((long)jMin, (long)jMax + 1);
  long v = (long)base + j; if (v < 1) v = 1; return (uint32_t)v;
}

// ---- Touch ------------------------------------------------------------------
int   touchBaseline = 0;
float touchFiltered = 0.0f;
bool  touchLatched  = false;

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

// ---- BLE task ---------------------------------------------------------------
BLEScan* pBLEScan = nullptr;
static volatile int      g_bleCount        = 0;
static volatile uint32_t g_bleLastUpdateMs = 0;
static portMUX_TYPE      g_bleMux          = portMUX_INITIALIZER_UNLOCKED;

void bleScanTask(void* pv) {
  (void)pv;
  vTaskDelay(pdMS_TO_TICKS(500));
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(BLE_SCAN_EVERY_MS));
    if (!pBLEScan) continue;
    uint32_t dur = (BLE_SCAN_FOR_MS + 999) / 1000; if (dur < 1) dur = 1;
    BLEScanResults* found = pBLEScan->start((int)dur, false);
    int count = found ? found->getCount() : 0;
    pBLEScan->clearResults();
    portENTER_CRITICAL(&g_bleMux); g_bleCount = count; g_bleLastUpdateMs = millis(); portEXIT_CRITICAL(&g_bleMux);
  }
}

int getBleDeviceCount(uint32_t* tsOut = nullptr) {
  portENTER_CRITICAL(&g_bleMux); int c = g_bleCount; uint32_t t = g_bleLastUpdateMs; portEXIT_CRITICAL(&g_bleMux);
  if (tsOut) *tsOut = t; return c;
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
  } return "?";
}

// ---- LED energy buffer ------------------------------------------------------
float ledEnergy[8]  = {0};
float ledTarget[8]  = {0};
uint32_t lastEnergyMs = 0;

void decayEnergy(uint32_t now, float rate) {
  if (lastEnergyMs == 0) { lastEnergyMs = now; return; }
  float dt = (now - lastEnergyMs) / 1000.0f; lastEnergyMs = now;
  for (int i = 0; i < 8; i++) ledEnergy[i] = clamp01f(ledEnergy[i] - rate * dt);
}

void pulseLed(int idx, float amp) {
  if (idx >= 0 && idx < 8) ledEnergy[idx] = clamp01f(ledEnergy[idx] + amp);
}

void pulseWithTail(int idx, float amp, float tail) {
  pulseLed(idx, amp);
  if (idx >= LEDA_FIRST && idx <= LEDA_LAST) {
    if (idx - 1 >= LEDA_FIRST) pulseLed(idx - 1, amp * tail);
    if (idx + 1 <= LEDA_LAST)  pulseLed(idx + 1, amp * tail);
  } else {
    if (idx - 1 >= LEDB_FIRST) pulseLed(idx - 1, amp * tail);
    if (idx + 1 <= LEDB_LAST)  pulseLed(idx + 1, amp * tail);
  }
}

void copyEnergyToTarget(float baseFloor = 0.0f) {
  for (int i = 0; i < 8; i++) ledTarget[i] = clamp01f(baseFloor + ledEnergy[i]);
}

// ---- Pattern state machines (mirrors anxious.ino) ----------------------------
static uint32_t idleNextStepMs = 0; static int idlePhase = 0;
static int idleAIdx = LEDA_FIRST; static int idleBIdx = LEDB_LAST;

void patternIdle(uint32_t now) {
  decayEnergy(now, 0.75f);
  if (idleNextStepMs == 0) idleNextStepMs = now;
  if (now >= idleNextStepMs) {
    if (random(0, 1000) < 6) pulseLed(random(0, 8), 0.10f);
    if (idlePhase == 0) {
      pulseWithTail(idleAIdx, 0.28f, 0.45f);
      int step = (random(0, 100) < 12) ? 2 : 1; idleAIdx += step;
      if (idleAIdx > LEDA_LAST) { idleAIdx = LEDA_FIRST; idlePhase = 1; idleNextStepMs = now + jitterMs(220, 10, 50); copyEnergyToTarget(0.01f); return; }
      idleNextStepMs = now + jitterMs(240, 10, 50);
    } else if (idlePhase == 1) { idlePhase = 2; idleBIdx = LEDB_LAST; idleNextStepMs = now + jitterMs(170, 10, 40); }
    else if (idlePhase == 2) {
      pulseWithTail(idleBIdx, 0.24f, 0.40f);
      int step = (random(0, 100) < 10) ? 2 : 1; idleBIdx -= step;
      if (idleBIdx < LEDB_FIRST) { idleBIdx = LEDB_LAST; idlePhase = 3; idleNextStepMs = now + jitterMs(220, 10, 50); copyEnergyToTarget(0.01f); return; }
      idleNextStepMs = now + jitterMs(250, 10, 50);
    } else { idlePhase = 0; idleAIdx = LEDA_FIRST; idleNextStepMs = now + jitterMs(180, 10, 40); }
  }
  copyEnergyToTarget(0.01f);
}

static uint32_t excNextStepMs = 0; static int excAIdx = LEDA_FIRST; static int excBIdx = LEDB_LAST;

void patternExcited(uint32_t now) {
  float a = clamp01f(E.arousal); decayEnergy(now, 1.25f + 0.75f * a);
  if (excNextStepMs == 0) excNextStepMs = now;
  int devices = E.lastSeenDevices; float crowd = clamp01f(devices / 18.0f);
  uint32_t base = (uint32_t)(140 - 70 * a - 40 * crowd); if (base < 35) base = 35;
  if (now >= excNextStepMs) {
    pulseWithTail(excAIdx, 0.40f + 0.55f * a, 0.55f);
    pulseWithTail(excBIdx, 0.38f + 0.55f * a, 0.55f);
    excAIdx++; if (excAIdx > LEDA_LAST) excAIdx = LEDA_FIRST;
    excBIdx--; if (excBIdx < LEDB_FIRST) excBIdx = LEDB_LAST;
    excNextStepMs = now + jitterMs(base, 5, 25);
  }
  copyEnergyToTarget(0.015f);
}

static uint32_t anxNextTwitchMs = 0; static uint32_t anxNextMisfireMs = 0;

void patternAnxious(uint32_t now) {
  float x = clamp01f(E.anxiety); decayEnergy(now, 1.35f + 1.20f * x);
  if (anxNextTwitchMs == 0) { anxNextTwitchMs = now; anxNextMisfireMs = now + 500; }
  if (now >= anxNextTwitchMs) {
    pulseLed(random(LEDA_FIRST, LEDA_LAST + 1), (0.10f + 0.85f * x) * frand(0.4f, 1.0f));
    uint32_t b = (uint32_t)(140 - 80 * x); if (b < 35) b = 35;
    anxNextTwitchMs = now + jitterMs(b, 0, 90);
  }
  if (now >= anxNextMisfireMs) {
    if (random(0, 1000) < (int)(250 * x)) pulseLed(random(LEDB_FIRST, LEDB_LAST + 1), (0.08f + 0.65f * x) * frand(0.3f, 1.0f));
    anxNextMisfireMs = now + jitterMs((uint32_t)(750 + 900 * (1.0f - x)), 0, 600);
  }
  copyEnergyToTarget(0.00f);
}

void patternFriendly(uint32_t now) {
  decayEnergy(now, 0.55f);
  uint32_t dt = now - E.lastTouchMs;
  if (dt < 900) { for (int i = 0; i < 8; i++) pulseLed(i, 0.6f); }
  else          { for (int i = 0; i < 8; i++) pulseLed(i, 0.2f); }
  copyEnergyToTarget(0.00f);
}

// ---- PCF8574 output ---------------------------------------------------------
static inline void pcfWritePin(uint8_t pin, bool on) {
  pcf.write(pin, on ? LOW : HIGH);
}

uint8_t pwmPhase = 0;

uint8_t computeMask() {
  uint8_t m = 0;
  for (int i = 0; i < 8; i++) {
    float b = ledTarget[i] * ledTarget[i];
    int level = (int)lroundf(b * PWM_LEVELS); if (level > (int)PWM_LEVELS) level = PWM_LEVELS;
    if (pwmPhase < level) m |= (1 << i);
  }
  return m;
}

uint8_t computeMaskBinary() {
  uint8_t m = 0;
  for (int i = 0; i < 8; i++) { float b = ledTarget[i] * ledTarget[i]; if (b >= BINARY_THRESHOLD) m |= (1 << i); }
  return m;
}

void applyMask(uint8_t m) { for (int i = 0; i < 8; i++) pcfWritePin(i, (m >> i) & 1); }

// ---- Audio ------------------------------------------------------------------
uint32_t audioNextEventMs = 0, audioEventEndMs = 0;
float    audioFreqHz = 0.0f; uint8_t audioDuty = 0;

void audioSilence() { if (ENABLE_AUDIO) ledcWrite(AUDIO_PIN, 0); }
void audioPlay(float hz, uint8_t duty) {
  if (!ENABLE_AUDIO || hz < 1.0f) { audioSilence(); return; }
  uint8_t sc = (uint8_t)(duty * AUDIO_VOLUME_SCALE); if (sc == 0 && duty > 0) sc = 1;
  ledcWriteTone(AUDIO_PIN, (uint32_t)hz); ledcWrite(AUDIO_PIN, sc);
}

void audioTick(uint32_t now, MoodState s) {
  if (!ENABLE_AUDIO) return;
  if (now < audioEventEndMs) { audioPlay(audioFreqHz * (1.0f + frand(-0.006f, 0.006f)), audioDuty); return; }
  if (now < audioNextEventMs) { audioSilence(); return; }
  switch (s) {
    case MoodState::Idle:     audioFreqHz = 120.0f + frand(-6,6);    audioDuty = (uint8_t)(18 + 22*(1.0f-E.anxiety)); audioEventEndMs = now+60;  audioNextEventMs = now+350; break;
    case MoodState::Excited:  audioFreqHz = 260.0f + 520.0f*E.arousal + frand(-40,60); audioDuty = (uint8_t)(28+70*E.arousal); audioEventEndMs = now+30; audioNextEventMs = now+90; break;
    case MoodState::Anxious:  audioFreqHz = 420.0f + 900.0f*E.anxiety + frand(-120,220); audioDuty = (uint8_t)(20+60*E.anxiety); audioEventEndMs = now+15; audioNextEventMs = now+60; break;
    case MoodState::Friendly: audioFreqHz = 480.0f + frand(-12,12);  audioDuty = 50; audioEventEndMs = now+80; audioNextEventMs = now+150; break;
  }
  audioPlay(audioFreqHz, audioDuty);
}

// ---- Globals ----------------------------------------------------------------
uint32_t lastTickMs = 0, lastPwmSliceUs = 0, lastWriteMs = 0, lastLogMs = 0, lastAppliedBle = 0;
MoodState lastState = MoodState::Idle;
static uint32_t touchNonce = 0;

void setup() {
  Serial.begin(115200);
  if (ENABLE_AUDIO) { ledcAttachChannel(AUDIO_PIN, 1000, AUDIO_LEDC_RES, AUDIO_LEDC_CH); ledcWrite(AUDIO_PIN, 0); }
  Wire.begin(SDA_PIN, SCL_PIN);
  pcf.begin();
  for (int i = 0; i < 8; i++) pcf.write(i, HIGH);
  randomSeed(esp_random());
  calibrateTouch();
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setActiveScan(true); pBLEScan->setInterval(100); pBLEScan->setWindow(80);
  xTaskCreatePinnedToCore(bleScanTask, "bleScan", 4096, nullptr, 1, nullptr, 0);
  E.lastTouchMs = millis() - 5000;
  Serial.print("12_anxius_lite_pwm_upgrade v3  ENABLE_PWM=");
  Serial.println(ENABLE_PWM ? "true (sliced)" : "false (binary)");
}

void loop() {
  uint32_t now = millis(); uint32_t nowUs = micros();

  // Touch
  static bool lastT = false; bool t = isTouched();
  if (t && !lastT) { E.lastTouchMs = now; touchNonce++; E.affection = clamp01(E.affection + 0.3f); E.anxiety = clamp01(E.anxiety - 0.4f); }
  lastT = t;

  // BLE
  uint32_t bleTs = 0; int count = getBleDeviceCount(&bleTs);
  if (bleTs != 0 && bleTs != lastAppliedBle) {
    lastAppliedBle = bleTs; E.lastSeenDevices = count;
    E.arousal = clamp01(E.arousal + 0.1f * clamp01(count / 10.0f));
  }

  // Emotion tick
  if (now - lastTickMs >= EMOTION_TICK_MS) { float dt = (now - lastTickMs) / 1000.0f; lastTickMs = now; tickEmotions(dt); }

  MoodState s = chooseState(now);
  if (s != lastState) { lastState = s; Serial.print("STATE -> "); Serial.println(stateName(s)); }

  // Pattern
  switch (s) {
    case MoodState::Idle:     patternIdle(now); break;
    case MoodState::Excited:  patternExcited(now); break;
    case MoodState::Anxious:  patternAnxious(now); break;
    case MoodState::Friendly: patternFriendly(now); break;
  }

  audioTick(now, s);

  // Output flush
  if (ENABLE_PWM) {
    if (nowUs - lastPwmSliceUs >= PWM_SLICE_US) {
      lastPwmSliceUs = nowUs; pwmPhase = (pwmPhase + 1) % PWM_LEVELS;
      applyMask(computeMask());
    }
  } else {
    if (now - lastWriteMs >= BINARY_WRITE_MS) { lastWriteMs = now; applyMask(computeMaskBinary()); }
  }

  if (now - lastLogMs >= LOG_MS) {
    lastLogMs = now;
    Serial.print("arous="); Serial.print(E.arousal, 2);
    Serial.print(" affec="); Serial.print(E.affection, 2);
    Serial.print(" anxty="); Serial.println(E.anxiety, 2);
  }
}
