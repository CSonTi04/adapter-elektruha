// =============================================================================
// 11_anxius_lite – Variant 3: Full binary integrator (PCF8574 + BLE + audio)
// =============================================================================
// Concept: Complete anxius.ino-lite with all subsystems active.
// Uses:
//   - Capacitive touch (GPIO4)
//   - PCF8574 for 8 LED outputs (binary mode, active-low)
//   - BLE scan task (FreeRTOS, non-blocking)
//   - LEDC audio on GPIO5
//
// This is the closest single-sketch equivalent to anxius.ino before the
// full LED pattern engine from sketch 12.
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
    Serial.print("[BLE] devices="); Serial.println(count);
  }
}

int getBleDeviceCount(uint32_t* tsOut = nullptr) {
  portENTER_CRITICAL(&g_bleMux); int c = g_bleCount; uint32_t t = g_bleLastUpdateMs; portEXIT_CRITICAL(&g_bleMux);
  if (tsOut) *tsOut = t; return c;
}

// ---- Emotion ----------------------------------------------------------------
float clamp01(float x)        { return x < 0 ? 0 : (x > 1 ? 1 : x); }
float clamp01f(float x)       { return clamp01(x); }
float frand(float a, float b) { return a + (b - a) * random(0, 10000) / 10000.0f; }

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

// ---- LED energy buffer ------------------------------------------------------
float ledEnergy[8]  = {0};
float ledTarget[8]  = {0};
uint32_t lastEnergyMs = 0;
int chaseIdx = 0;
uint32_t nextPatternMs = 0;

void decayEnergy(uint32_t now, float rate) {
  if (lastEnergyMs == 0) { lastEnergyMs = now; return; }
  float dt = (now - lastEnergyMs) / 1000.0f; lastEnergyMs = now;
  for (int i = 0; i < 8; i++) ledEnergy[i] = clamp01f(ledEnergy[i] - rate * dt);
}

void pulseLed(int idx, float amp) {
  if (idx >= 0 && idx < 8) ledEnergy[idx] = clamp01f(ledEnergy[idx] + amp);
}

void copyEnergyToTarget() {
  for (int i = 0; i < 8; i++) ledTarget[i] = clamp01f(ledEnergy[i]);
}

void runPattern(MoodState s, uint32_t now) {
  float rate = (s == MoodState::Anxious) ? 2.0f : (s == MoodState::Excited) ? 1.5f : 0.8f;
  decayEnergy(now, rate);
  uint32_t step = (s == MoodState::Excited) ? 100 : (s == MoodState::Anxious) ? 70 : 200;
  if (now >= nextPatternMs) {
    nextPatternMs = now + step;
    if (s == MoodState::Friendly) {
      for (int i = 0; i < 8; i++) pulseLed(i, 0.3f);
    } else if (s == MoodState::Anxious) {
      pulseLed(random(0, 4), 0.5f + 0.4f * E.anxiety);
    } else {
      pulseLed(chaseIdx, 0.5f);
      if (chaseIdx > 0) pulseLed(chaseIdx - 1, 0.22f);
      chaseIdx = (chaseIdx + 1) % 8;
    }
  }
  copyEnergyToTarget();
}

// ---- PCF8574 output ---------------------------------------------------------
static inline void pcfWritePin(uint8_t pin, bool on) {
  pcf.write(pin, on ? LOW : HIGH); // active-low
}

uint8_t computeMaskBinary() {
  uint8_t m = 0;
  for (int i = 0; i < 8; i++) {
    float b = ledTarget[i] * ledTarget[i];
    if (b >= BINARY_THRESHOLD) m |= (1 << i);
  }
  return m;
}

void applyMask(uint8_t m) {
  for (int i = 0; i < 8; i++) pcfWritePin(i, (m >> i) & 1);
}

// ---- Audio ------------------------------------------------------------------
uint32_t audioNextEventMs = 0;
uint32_t audioEventEndMs  = 0;
float    audioFreqHz      = 0.0f;
uint8_t  audioDuty        = 0;

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
    case MoodState::Idle:     audioFreqHz = 120.0f + frand(-6,6);    audioDuty = 20; audioEventEndMs = now+60;  audioNextEventMs = now+350; break;
    case MoodState::Excited:  audioFreqHz = 400.0f + frand(-40,60);  audioDuty = 80; audioEventEndMs = now+30;  audioNextEventMs = now+90;  break;
    case MoodState::Anxious:  audioFreqHz = 600.0f + frand(-120,220);audioDuty = 50; audioEventEndMs = now+15;  audioNextEventMs = now+60;  break;
    case MoodState::Friendly: audioFreqHz = 480.0f + frand(-12,12);  audioDuty = 50; audioEventEndMs = now+80;  audioNextEventMs = now+150; break;
  }
  audioPlay(audioFreqHz, audioDuty);
}

// ---- Globals ----------------------------------------------------------------
uint32_t lastTickMs     = 0;
uint32_t lastWriteMs    = 0;
uint32_t lastLogMs      = 0;
uint32_t lastAppliedBle = 0;
MoodState lastState     = MoodState::Idle;

void setup() {
  Serial.begin(115200);
  if (ENABLE_AUDIO) { ledcAttachChannel(AUDIO_PIN, 1000, AUDIO_LEDC_RES, AUDIO_LEDC_CH); ledcWrite(AUDIO_PIN, 0); }
  Wire.begin(SDA_PIN, SCL_PIN);
  pcf.begin();
  for (int i = 0; i < 8; i++) pcf.write(i, HIGH); // all off
  randomSeed(esp_random());
  calibrateTouch();
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(80);
  xTaskCreatePinnedToCore(bleScanTask, "bleScan", 4096, nullptr, 1, nullptr, 0);
  E.lastTouchMs = millis() - 5000;
  Serial.println("11_anxius_lite v3 – full (PCF8574 + BLE + audio)");
}

void loop() {
  uint32_t now = millis();

  // Touch
  static bool lastT = false; bool t = isTouched();
  if (t && !lastT) { E.lastTouchMs = now; E.affection = clamp01(E.affection + 0.3f); E.anxiety = clamp01(E.anxiety - 0.4f); }
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

  runPattern(s, now);
  audioTick(now, s);

  // Binary LED output
  if (now - lastWriteMs >= BINARY_WRITE_MS) { lastWriteMs = now; applyMask(computeMaskBinary()); }

  if (now - lastLogMs >= LOG_MS) {
    lastLogMs = now;
    Serial.print("arous="); Serial.print(E.arousal, 2);
    Serial.print(" affec="); Serial.print(E.affection, 2);
    Serial.print(" anxty="); Serial.println(E.anxiety, 2);
  }
}
