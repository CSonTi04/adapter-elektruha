enum class MoodState : uint8_t {
  Idle,
  Excited,
  Anxious,
  Friendly
};

#include <Wire.h>
#include <PCF8574.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <math.h>

// -------------------- State --------------------
MoodState chooseState(uint32_t nowMs);

// ================= VERBOSITY CONTROL =================
#define VERBOSE_NONE     0
#define VERBOSE_STATE    1
#define VERBOSE_EMOTION  2
#define VERBOSE_FULL     3

#define VERBOSE_LEVEL VERBOSE_EMOTION   // <<< CHANGE THIS
// =====================================================
// ================= PWM OUTPUT MODE =================
// true  = simulated brightness via time-sliced PWM (smooth but can look "always on")
// false = binary threshold mode (clear ON/OFF states, more readable nervous system)
constexpr bool ENABLE_PWM = false;   // <<< TOGGLE THIS
// ===================================================
#if VERBOSE_LEVEL >= VERBOSE_STATE
  #define LOG_STATE(x) Serial.println(x)
#else
  #define LOG_STATE(x)
#endif

#if VERBOSE_LEVEL >= VERBOSE_EMOTION
  #define LOG_EMOTION(x) Serial.println(x)
#else
  #define LOG_EMOTION(x)
#endif

#if VERBOSE_LEVEL >= VERBOSE_FULL
  #define LOG_FULL(x) Serial.println(x)
#else
  #define LOG_FULL(x)
#endif

// -------------------- Pins --------------------
constexpr int AUDIO_PIN = 5; // LM386 input (through coupling cap recommended)

constexpr int SDA_PIN = 21;
constexpr int SCL_PIN = 22;
constexpr int TOUCH_GPIO = 4;

// -------------------- PCF8574 --------------------
constexpr uint8_t PCF_ADDR = 0x20;
PCF8574 pcf(PCF_ADDR);

static inline void pcfWritePin(uint8_t pin, bool on) {
  pcf.write(pin, on ? LOW : HIGH);
}

// -------------------- Emotional Model --------------------
struct Emotion {
  float arousal   = 0.0f;
  float anxiety   = 0.0f;
  float affection = 0.0f;

  uint32_t lastTouchMs = 0;
  int lastSeenDevices = 0;
} E;

// -------------------- Timing --------------------

// ================= AUDIO OUTPUT MODE =================
// Uses ESP32 LEDC PWM to generate simple tones for LM386.
// If you later move to a DAC pin (GPIO25/26), we can do smoother audio.
constexpr bool ENABLE_AUDIO = true;        // <<< TOGGLE THIS
constexpr int  AUDIO_LEDC_CH = 2;
constexpr int  AUDIO_LEDC_RES_BITS = 8;    // 0..255 duty "volume"
constexpr float AUDIO_VOLUME_SCALE = 0.35f; // global volume trim (0.0..1.0)
// Note: LEDC base freq is set by ledcWriteTone().
// ====================================================

// === Output mode ===
// If PWM is ON: brightness is simulated via fast on/off slices.
// If PWM is OFF: LEDs are driven as simple ON/OFF using a threshold (much clearer "some off" states).
// (ENABLE_PWM defined at top of file)
// constexpr bool ENABLE_PWM = false;  // removed duplicate definition
constexpr float BINARY_THRESHOLD = 0.22f;  // used when ENABLE_PWM == false (0..1)

// When PWM is OFF, limit how often we write I2C outputs (ms)
constexpr uint32_t BINARY_WRITE_EVERY_MS = 8;
uint32_t lastBinaryWriteMs = 0;


uint32_t lastEmotionTickMs = 0;
constexpr uint32_t EMOTION_TICK_MS = 50;

constexpr uint32_t PWM_PERIOD_US = 4000;
constexpr uint8_t  PWM_LEVELS    = 16;
constexpr uint32_t PWM_SLICE_US  = PWM_PERIOD_US / PWM_LEVELS;

uint32_t lastPwmSliceUs = 0;
uint8_t pwmPhase = 0;

constexpr uint32_t BLE_SCAN_EVERY_MS = 2500;
constexpr uint32_t BLE_SCAN_FOR_MS   = 3000;  // phones can be bursty; 3s is more reliable
uint32_t lastBleScanStartMs = 0;

// -------------------- Touch --------------------
int touchBaseline = 0;
float touchFiltered = 0.0f;
bool touchLatched = false;

// Touch hysteresis:
// - enter touch below TOUCH_ON_RATIO * baseline
// - release touch above TOUCH_OFF_RATIO * baseline
// Using two thresholds prevents rapid toggling when signal hovers near the edge.
constexpr float TOUCH_ON_RATIO  = 0.85f;
constexpr float TOUCH_OFF_RATIO = 0.90f;

float clamp01(float x) { return x < 0 ? 0 : (x > 1 ? 1 : x); }
float frand(float a, float b) { return a + (b - a) * (float)random(0, 10000) / 10000.0f; }

void calibrateTouch() {
  delay(400);
  long sum = 0;
  for (int i = 0; i < 50; i++) {
    sum += touchRead(TOUCH_GPIO);
    delay(10);
  }
  touchBaseline = sum / 50;
  touchFiltered = touchBaseline;
}

bool isTouched() {
  int raw = touchRead(TOUCH_GPIO);
  const float alpha = 0.15f;
  touchFiltered = (1 - alpha) * touchFiltered + alpha * raw;

  const float onThreshold = touchBaseline * TOUCH_ON_RATIO;
  const float offThreshold = touchBaseline * TOUCH_OFF_RATIO;

  if (!touchLatched) {
    if (touchFiltered < onThreshold) touchLatched = true;
  } else {
    if (touchFiltered > offThreshold) touchLatched = false;
  }

  return touchLatched;
}

// -------------------- BLE (Bluedroid / core BLE) --------------------
BLEScan* pBLEScan = nullptr;

// Non-blocking scanning: run the blocking scan in its own FreeRTOS task.
static volatile int g_bleCount = 0;
static volatile uint32_t g_bleLastUpdateMs = 0;
static portMUX_TYPE g_bleMux = portMUX_INITIALIZER_UNLOCKED;

void bleScanTask(void* pv) {
  (void)pv;

  // Give setup time to finish serial init etc.
  vTaskDelay(pdMS_TO_TICKS(500));

  for (;;) {
    // Wait until next scan window.
    vTaskDelay(pdMS_TO_TICKS(BLE_SCAN_EVERY_MS));

    if (!pBLEScan) continue;

    uint32_t durSec = (BLE_SCAN_FOR_MS + 999) / 1000; // ceil to seconds
    if (durSec < 1) durSec = 1;

#if VERBOSE_LEVEL >= VERBOSE_EMOTION
    Serial.print("[BLE task] scan ");
    Serial.print(durSec);
    Serial.println("s...");
#endif

    BLEScanResults* found = pBLEScan->start(durSec, false); // BLOCKS THIS TASK ONLY
    int count = found ? found->getCount() : 0;
    pBLEScan->clearResults();

    portENTER_CRITICAL(&g_bleMux);
    g_bleCount = count;
    g_bleLastUpdateMs = millis();
    portEXIT_CRITICAL(&g_bleMux);

#if VERBOSE_LEVEL >= VERBOSE_EMOTION
    Serial.print("[BLE task] devices=");
    Serial.println(count);
#endif
  }
}

// Read the latest scan result (non-blocking)
int getBleDeviceCount(uint32_t* lastUpdateMsOut = nullptr) {
  portENTER_CRITICAL(&g_bleMux);
  int c = g_bleCount;
  uint32_t t = g_bleLastUpdateMs;
  portEXIT_CRITICAL(&g_bleMux);
  if (lastUpdateMsOut) *lastUpdateMsOut = t;
  return c;
}

// -------------------- State --------------------

MoodState chooseState(uint32_t nowMs) {
  if (nowMs - E.lastTouchMs < 1500) return MoodState::Friendly;
  if (E.anxiety > 0.65f) return MoodState::Anxious;
  if (E.arousal > 0.55f) return MoodState::Excited;
  return MoodState::Idle;
}

// -------------------- LED --------------------

// We treat the 8 LEDs as two emotional hemispheres:
// Group A (impulse): 0..3
// Group B (response): 4..7
// Avoid conflicts with Arduino core symbols LEDA_FIRST/LEDB_FIRST/LEDB_LAST etc.
constexpr int LEDA_FIRST = 0, LEDA_LAST = 3;
constexpr int LEDB_FIRST = 4, LEDB_LAST = 7;

float ledTarget[8];
static float ledEnergy[8] = {0};
static uint32_t lastEnergyMs = 0;

// Touch nonce so friendly pattern can vary slightly each touch
static uint32_t touchNonce = 0;

static inline float clamp01f(float x) { return x < 0 ? 0 : (x > 1 ? 1 : x); }

static inline void decayEnergy(uint32_t nowMs, float decayPerSec) {
  if (lastEnergyMs == 0) lastEnergyMs = nowMs;
  float dt = (nowMs - lastEnergyMs) / 1000.0f;
  lastEnergyMs = nowMs;
  // linear decay is fine for "organic" feel
  for (int i = 0; i < 8; i++) {
    ledEnergy[i] = clamp01f(ledEnergy[i] - decayPerSec * dt);
  }
}

static inline void pulseLed(int idx, float amp) {
  if (idx < 0 || idx > 7) return;
  // additive with clamp gives a nice "spiky" biological response
  ledEnergy[idx] = clamp01f(ledEnergy[idx] + amp);
}

static inline void pulseWithTail(int idx, float amp, float tail) {
  pulseLed(idx, amp);
  // tail to neighbors, but keep within hemisphere boundaries for readability
  if (idx >= LEDA_FIRST && idx <= LEDA_LAST) {
    if (idx - 1 >= LEDA_FIRST) pulseLed(idx - 1, amp * tail);
    if (idx + 1 <= LEDA_LAST) pulseLed(idx + 1, amp * tail);
  } else {
    if (idx - 1 >= LEDB_FIRST) pulseLed(idx - 1, amp * tail);
    if (idx + 1 <= LEDB_LAST) pulseLed(idx + 1, amp * tail);
  }
}

static inline void copyEnergyToTarget(float baseFloor = 0.0f) {
  for (int i = 0; i < 8; i++) {
    // small base floor prevents complete deadness
    float v = baseFloor + ledEnergy[i];
    ledTarget[i] = clamp01f(v);
  }
}

// -------------------- Pattern State Machines --------------------

// IDLE: A slowly drifts 0->3, pause, then B echoes 7->4, pause.
static uint32_t idleNextStepMs = 0;
static int idlePhase = 0; // 0=A forward, 1=pause, 2=B backward, 3=pause
static int idleAIdx = LEDA_FIRST;
static int idleBIdx = LEDB_LAST;

// EXCITED: A chases 0->3, B chases 7->4 opposite direction.
static uint32_t excNextStepMs = 0;
static int excAIdx = LEDA_FIRST;
static int excBIdx = LEDB_LAST;
static uint32_t excOverloadUntilMs = 0;

// ANXIOUS: twitchy random; occasional B misfire; rare full-body flash.
static uint32_t anxNextTwitchMs = 0;
static uint32_t anxNextMisfireMs = 0;
static uint32_t anxNextFlashMs = 0;

static inline uint32_t jitterMs(uint32_t base, uint32_t jMin, uint32_t jMax) {
  int j = (int)random((long)jMin, (long)jMax + 1);
  long v = (long)base + j;
  if (v < 1) v = 1;
  return (uint32_t)v;
}

void patternIdle(uint32_t nowMs) {
  // Breathing nerves: propagation, then echo back.
  decayEnergy(nowMs, 0.75f); // slow decay

  if (idleNextStepMs == 0) idleNextStepMs = nowMs;

  if (nowMs >= idleNextStepMs) {
    // occasional rare misfire even while calm
    if (random(0, 1000) < 6) {
      pulseLed(random(0, 8), 0.10f);
    }

    if (idlePhase == 0) {
      // Group A forward 0->3, sometimes skip
      pulseWithTail(idleAIdx, 0.28f, 0.45f);

      int step = (random(0, 100) < 12) ? 2 : 1; // occasional skip
      idleAIdx += step;
      if (idleAIdx > LEDA_LAST) {
        idleAIdx = LEDA_FIRST;
        idlePhase = 1;
        idleNextStepMs = nowMs + jitterMs(220, 10, 50); // pause
        copyEnergyToTarget(0.01f);
        return;
      }

      idleNextStepMs = nowMs + jitterMs(240, 10, 50);
    }
    else if (idlePhase == 1) {
      idlePhase = 2;
      idleBIdx = LEDB_LAST;
      idleNextStepMs = nowMs + jitterMs(170, 10, 40);
    }
    else if (idlePhase == 2) {
      // Group B echo backward 7->4, sometimes skip
      pulseWithTail(idleBIdx, 0.24f, 0.40f);

      int step = (random(0, 100) < 10) ? 2 : 1;
      idleBIdx -= step;
      if (idleBIdx < LEDB_FIRST) {
        idleBIdx = LEDB_LAST;
        idlePhase = 3;
        idleNextStepMs = nowMs + jitterMs(220, 10, 50); // pause
        copyEnergyToTarget(0.01f);
        return;
      }

      idleNextStepMs = nowMs + jitterMs(250, 10, 50);
    }
    else {
      // pause -> back to A
      idlePhase = 0;
      idleAIdx = LEDA_FIRST;
      idleNextStepMs = nowMs + jitterMs(180, 10, 40);
    }
  }

  copyEnergyToTarget(0.01f);
}

void patternExcited(uint32_t nowMs) {
  // Overstimulated cross-talk: opposing chases with occasional overload.
  float a = clamp01f(E.arousal);
  decayEnergy(nowMs, 1.25f + 0.75f * a); // faster decay so motion reads as propagation

  // occasional overload flash when highly aroused
  if (a > 0.75f && random(0, 1000) < (int)(3 + 10 * a)) {
    excOverloadUntilMs = nowMs + 120;
  }

  if (excOverloadUntilMs > nowMs) {
    for (int i = 0; i < 8; i++) {
      ledEnergy[i] = clamp01f(ledEnergy[i] + 0.85f);
    }
    // after overload, add slight scatter
    if (random(0, 1000) < 60) pulseLed(random(0, 8), 0.35f);
    copyEnergyToTarget(0.015f);
    return;
  }

  if (excNextStepMs == 0) excNextStepMs = nowMs;

  // speed scales with arousal and device count
  int devices = E.lastSeenDevices;
  float crowd = clamp01f(devices / 18.0f);
  uint32_t base = (uint32_t)(140 - 70 * a - 40 * crowd);
  if (base < 35) base = 35;

  if (nowMs >= excNextStepMs) {
    float headA = 0.40f + 0.55f * a;
    float headB = 0.38f + 0.55f * a;

    // A: 0->3
    pulseWithTail(excAIdx, headA, 0.55f);
    // B: 7->4 (opposite)
    pulseWithTail(excBIdx, headB, 0.55f);

    // boundary overlap agitation (3 and 4)
    if (random(0, 1000) < (int)(40 + 120 * a)) {
      pulseLed(3, 0.08f + 0.15f * a);
      pulseLed(4, 0.08f + 0.15f * a);
    }

    // micro-flicker during excited
    if (random(0, 1000) < (int)(10 + 80 * a)) {
      pulseLed(random(0, 8), 0.10f + 0.18f * a);
    }

    // advance indices
    excAIdx++;
    if (excAIdx > LEDA_LAST) excAIdx = LEDA_FIRST;

    excBIdx--;
    if (excBIdx < LEDB_FIRST) excBIdx = LEDB_LAST;

    excNextStepMs = nowMs + jitterMs(base, 5, 25);
  }

  copyEnergyToTarget(0.015f);
}

void patternAnxious(uint32_t nowMs) {
  // Twitching: no smooth chase; mostly random flicker in A, occasional misfire in B.
  float x = clamp01f(E.anxiety);
  decayEnergy(nowMs, 1.35f + 1.20f * x);

  if (anxNextTwitchMs == 0) {
    anxNextTwitchMs = nowMs;
    anxNextMisfireMs = nowMs + 500;
    anxNextFlashMs = nowMs + 1800;
  }

  if (nowMs >= anxNextTwitchMs) {
    int aLed = random(LEDA_FIRST, LEDA_LAST + 1);
    pulseLed(aLed, (0.10f + 0.85f * x) * frand(0.4f, 1.0f));
    // sometimes rapid jump to another A LED
    if (random(0, 1000) < (int)(120 + 250 * x)) {
      pulseLed(random(LEDA_FIRST, LEDA_LAST + 1), 0.20f * x);
    }
    uint32_t base = (uint32_t)(140 - 80 * x);
    if (base < 35) base = 35;
    anxNextTwitchMs = nowMs + jitterMs(base, 0, 90);
  }

  if (nowMs >= anxNextMisfireMs) {
    if (random(0, 1000) < (int)(250 * x)) {
      int bLed = random(LEDB_FIRST, LEDB_LAST + 1);
      pulseLed(bLed, (0.08f + 0.65f * x) * frand(0.3f, 1.0f));
    }
    anxNextMisfireMs = nowMs + jitterMs((uint32_t)(750 + 900 * (1.0f - x)), 0, 600);
  }

  // rare full-body flash (stress tremor)
  if (nowMs >= anxNextFlashMs) {
    if (random(0, 1000) < (int)(40 + 180 * x)) {
      for (int i = 0; i < 8; i++) pulseLed(i, 0.35f + 0.45f * x);
    }
    anxNextFlashMs = nowMs + jitterMs(2200, 0, 2200);
  }

  // asymmetry bias: A tends to be more active than B
  copyEnergyToTarget(0.00f);
}

void patternFriendly(uint32_t nowMs) {
  // Wave of relief: fill A 0->3, then B 4->7, then alternating pairs, then brief glow.
  decayEnergy(nowMs, 0.55f);

  uint32_t dt = nowMs - E.lastTouchMs;

  // We add a tiny deterministic jitter based on touchNonce so each touch feels a bit different
  uint32_t seed = touchNonce * 2654435761u;
  uint32_t j1 = 20 + (seed % 35);
  uint32_t j2 = 25 + ((seed >> 8) % 45);

  // Phase 1: A fills quickly outward (0->3)
  if (dt < 420) {
    int step = (int)((dt + j1) / 90); // 0..4
    if (step > 4) step = 4;
    for (int i = LEDA_FIRST; i < LEDA_FIRST + step && i <= LEDA_LAST; i++) {
      pulseLed(i, 0.70f);
    }
    // keep a soft trailing glow
    pulseLed(LEDA_FIRST + (step % 4), 0.25f);
    copyEnergyToTarget(0.00f);
    return;
  }

  // Phase 2: B fills immediately after (4->7)
  if (dt < 900) {
    uint32_t dt2 = dt - 420;
    int stepA = 4;
    for (int i = LEDA_FIRST; i <= LEDA_LAST; i++) pulseLed(i, 0.55f);

    int stepB = (int)((dt2 + j2) / 90);
    if (stepB > 4) stepB = 4;
    for (int i = LEDB_FIRST; i < LEDB_FIRST + stepB && i <= LEDB_LAST; i++) {
      pulseLed(i, 0.70f);
    }
    pulseLed(LEDB_FIRST + (stepB % 4), 0.25f);
    copyEnergyToTarget(0.00f);
    return;
  }

  // Phase 3: alternating pairs fade (about 2 seconds)
  if (dt < 2800) {
    // keep both hemispheres present but alternating emphasis
    uint32_t t3 = dt - 900;
    int phase = (int)(t3 / 320) % 4;

    // base calming glow
    for (int i = 0; i < 8; i++) pulseLed(i, 0.05f);

    if (phase == 0) { pulseLed(0, 0.35f); pulseLed(1, 0.35f); pulseLed(4, 0.30f); pulseLed(5, 0.30f); }
    if (phase == 1) { pulseLed(2, 0.35f); pulseLed(3, 0.35f); pulseLed(6, 0.30f); pulseLed(7, 0.30f); }
    if (phase == 2) { pulseLed(0, 0.28f); pulseLed(2, 0.28f); pulseLed(4, 0.24f); pulseLed(6, 0.24f); }
    if (phase == 3) { pulseLed(1, 0.28f); pulseLed(3, 0.28f); pulseLed(5, 0.24f); pulseLed(7, 0.24f); }

    copyEnergyToTarget(0.00f);
    return;
  }

  // Phase 4: brief unified glow then release back to calm
  if (dt < 3100) {
    for (int i = 0; i < 8; i++) pulseLed(i, 0.30f);
    copyEnergyToTarget(0.00f);
    return;
  }

  // after friendly window, just let it fall back naturally
  copyEnergyToTarget(0.01f);
}

uint8_t computeMask() {
  // PWM mask (uses pwmPhase)
  uint8_t mask = 0;
  for (int i = 0; i < 8; i++) {
    // perceptual shaping (gamma-ish): square the target
    float b = ledTarget[i] * ledTarget[i];

    // Convert brightness to PWM slices [0..PWM_LEVELS]
    int level = (int)lroundf(b * PWM_LEVELS);
    if (level < 0) level = 0;
    if (level > PWM_LEVELS) level = PWM_LEVELS;

    // pwmPhase is [0..PWM_LEVELS-1]
    if (pwmPhase < level) mask |= (1 << i);
  }
  return mask;
}

uint8_t computeMaskBinary() {
  // Simple ON/OFF mask (ignores pwmPhase). Great for readability.
  uint8_t mask = 0;
  for (int i = 0; i < 8; i++) {
    float b = ledTarget[i] * ledTarget[i]; // keep same shaping
    if (b >= BINARY_THRESHOLD) mask |= (1 << i);
  }
  return mask;
}

void applyMask(uint8_t m) {
  for (int i = 0; i < 8; i++) pcfWritePin(i, (m >> i) & 1);
}

// -------------------- Audio --------------------

// Very small "voice" driven by emotional state.
// We intentionally keep it imperfect: jitter, gaps, and non-musical drifts.
static uint32_t audioNextEventMs = 0;
static uint32_t audioEventEndMs  = 0;
static float    audioFreqHz      = 0.0f;
static uint8_t  audioDuty        = 0;      // 0..255

static inline void audioSilence() {
  if (!ENABLE_AUDIO) return;
  ledcWrite(AUDIO_PIN, 0);
}

static inline void audioTone(float hz, uint8_t duty) {
  if (!ENABLE_AUDIO) return;
  if (hz < 1.0f || duty == 0) {
    audioSilence();
    return;
  }
  uint8_t scaledDuty = (uint8_t)(duty * AUDIO_VOLUME_SCALE);
  if (scaledDuty == 0 && duty > 0) scaledDuty = 1;
  ledcWriteTone(AUDIO_PIN, (uint32_t)hz);
  ledcWrite(AUDIO_PIN, scaledDuty);
}

void audioTick(uint32_t nowMs, uint8_t sRaw) {
  if (!ENABLE_AUDIO) return;
  MoodState s = (MoodState)sRaw;

  // If current event is active, keep it running (optionally drift a bit)
  if (nowMs < audioEventEndMs) {
    // micro drift keeps it organic
    float drift = 1.0f + frand(-0.006f, 0.006f);
    audioTone(audioFreqHz * drift, audioDuty);
    return;
  }

  // If we're between events, silence until next event time
  if (nowMs < audioNextEventMs) {
    audioSilence();
    return;
  }

  // Schedule a new event depending on state
  switch (s) {
    case MoodState::Idle: {
      // "Breathing nerves" = quiet, low throb with long gaps
      // Two little pulses that feel like a hesitant heartbeat.
      float base = 120.0f + 40.0f * (1.0f - E.affection);
      audioFreqHz = base + frand(-6.0f, 6.0f);
      audioDuty = (uint8_t)(18 + 22 * (1.0f - E.anxiety)); // quieter when anxious
      uint32_t onMs = (uint32_t)(55 + frand(0, 35));
      audioEventEndMs = nowMs + onMs;
      // next pulse after a soft gap
      audioNextEventMs = nowMs + (uint32_t)(260 + frand(0, 380));
      break;
    }

    case MoodState::Excited: {
      // "Cross-talk" = faster chirps, higher pitch, boundary agitation
      float a = clamp01f(E.arousal);
      float crowd = clamp01f(E.lastSeenDevices / 18.0f);
      float f = 260.0f + 520.0f * a + 220.0f * crowd;
      audioFreqHz = f + frand(-40.0f, 60.0f);
      audioDuty = (uint8_t)(28 + 70 * a);
      uint32_t onMs = (uint32_t)(25 + 55 * (1.0f - a) + frand(0, 25));
      audioEventEndMs = nowMs + onMs;

      // occasional overload "flash" becomes a short burst
      if (a > 0.75f && random(0, 1000) < (int)(15 + 40 * a)) {
        audioFreqHz = 900.0f + 700.0f * frand(0.0f, 1.0f);
        audioDuty = (uint8_t)(90 + 90 * frand(0.0f, 1.0f));
        audioEventEndMs = nowMs + (uint32_t)(35 + frand(0, 45));
        audioNextEventMs = nowMs + (uint32_t)(90 + frand(0, 120));
      } else {
        audioNextEventMs = nowMs + (uint32_t)(70 + 140 * (1.0f - a) + frand(0, 60));
      }
      break;
    }

    case MoodState::Anxious: {
      // "Twitching" = irregular ticks + occasional sharp misfire
      float x = clamp01f(E.anxiety);

      // Most of the time: short tick
      audioFreqHz = 420.0f + 900.0f * x + frand(-120.0f, 220.0f);
      audioDuty = (uint8_t)(20 + 60 * x);
      audioEventEndMs = nowMs + (uint32_t)(12 + frand(0, 35));

      // Sometimes: a misfire spike
      if (random(0, 1000) < (int)(70 + 320 * x)) {
        audioFreqHz = 1200.0f + 1400.0f * frand(0.0f, 1.0f);
        audioDuty = (uint8_t)(70 + 120 * x);
        audioEventEndMs = nowMs + (uint32_t)(10 + frand(0, 25));
      }

      // rare full-body tremor = brief loud buzz
      if (random(0, 1000) < (int)(12 + 80 * x)) {
        audioFreqHz = 180.0f + 90.0f * frand(0.0f, 1.0f);
        audioDuty = (uint8_t)(120 + 80 * x);
        audioEventEndMs = nowMs + (uint32_t)(65 + frand(0, 60));
      }

      // irregular spacing
      audioNextEventMs = nowMs + (uint32_t)(25 + 220 * (1.0f - x) + frand(0, 180));
      break;
    }

    case MoodState::Friendly: {
      // "Relief" = a gentle descending sigh/purr shortly after touch
      uint32_t dt = nowMs - E.lastTouchMs;
      float hold = clamp01f(1.0f - (float)dt / 1200.0f);

      // descending frequency over time since touch
      float startF = 520.0f + 240.0f * hold;
      float endF   = 220.0f + 120.0f * (1.0f - hold);
      float t = clamp01f(dt / 1600.0f);
      audioFreqHz = (1.0f - t) * startF + t * endF + frand(-12.0f, 12.0f);

      audioDuty = (uint8_t)(30 + 70 * hold);
      audioEventEndMs = nowMs + (uint32_t)(60 + 90 * hold + frand(0, 40));

      // after touch: a couple of calmer pulses then fade out
      audioNextEventMs = nowMs + (uint32_t)(110 + 200 * (1.0f - hold) + frand(0, 120));
      break;
    }
  }
}

// -------------------- Emotion Update --------------------

// Periodic emotional state logging (non-spammy)
uint32_t lastEmotionLogMs = 0;
constexpr uint32_t EMOTION_LOG_EVERY_MS = 1000;  // 1s

void logEmotionalState(uint32_t nowMs) {
#if VERBOSE_LEVEL >= VERBOSE_EMOTION
  if (nowMs - lastEmotionLogMs >= EMOTION_LOG_EVERY_MS) {
    lastEmotionLogMs = nowMs;

    Serial.print("EMO | arousal=");
    Serial.print(E.arousal, 2);
    Serial.print(" anxiety=");
    Serial.print(E.anxiety, 2);
    Serial.print(" affection=");
    Serial.print(E.affection, 2);
    Serial.print(" state=");

    MoodState s = chooseState(nowMs);
    switch (s) {
      case MoodState::Idle:     Serial.println("Idle"); break;
      case MoodState::Excited:  Serial.println("Excited"); break;
      case MoodState::Anxious:  Serial.println("Anxious"); break;
      case MoodState::Friendly: Serial.println("Friendly"); break;
    }
  }
#endif
}

void tickEmotions(float dt) {
  E.arousal   = clamp01(E.arousal - 0.06f * dt);
  E.affection = clamp01(E.affection - 0.02f * dt);
  E.anxiety   = clamp01(E.anxiety + 0.02f * dt - 0.03f * E.affection * dt);
}

// -------------------- Setup --------------------
void setup() {
  Serial.begin(115200);

  // Audio (LEDC PWM)
  if (ENABLE_AUDIO) {
    ledcAttachChannel(AUDIO_PIN, 1000, AUDIO_LEDC_RES_BITS, AUDIO_LEDC_CH); // freq will be overridden by ledcWriteTone
    ledcWrite(AUDIO_PIN, 0);
  }
  Wire.begin(SDA_PIN, SCL_PIN);
  pcf.begin();
  for (int i = 0; i < 8; i++) pcf.write(i, HIGH);

  randomSeed(esp_random());
  calibrateTouch();

  // BLE init (Bluedroid)
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(80);

  // Start BLE scan task on the other core so LED PWM never freezes.
  // ESP32 has two cores; Arduino loop runs on core 1 by default.
  xTaskCreatePinnedToCore(
    bleScanTask,
    "bleScanTask",
    4096,
    nullptr,
    1,
    nullptr,
    0
  );

  E.lastTouchMs = millis();
  lastEmotionTickMs = millis();

  LOG_STATE("Sculpture ready.");
}

// -------------------- Loop --------------------
void loop() {
  uint32_t nowMs = millis();
  uint32_t nowUs = micros();

  // Touch
  static bool lastTouch = false;
  bool t = isTouched();
  if (t && !lastTouch) {
    E.lastTouchMs = nowMs;
    touchNonce++;
    E.anxiety = clamp01(E.anxiety - 0.4f);
    E.affection = clamp01(E.affection + 0.3f);
    LOG_FULL("TOUCH detected");
  }
  lastTouch = t;

  // Emotions
  if (nowMs - lastEmotionTickMs >= EMOTION_TICK_MS) {
    float dt = (nowMs - lastEmotionTickMs) / 1000.0f;
    lastEmotionTickMs = nowMs;
    tickEmotions(dt);
  }

  // BLE update (non-blocking; values produced by BLE task)
  static uint32_t lastAppliedBleMs = 0;
  uint32_t bleUpdatedMs = 0;
  int count = getBleDeviceCount(&bleUpdatedMs);

  // Only apply when there's a new scan result
  if (bleUpdatedMs != 0 && bleUpdatedMs != lastAppliedBleMs) {
    lastAppliedBleMs = bleUpdatedMs;

    // Arousal bump (you can later replace with smoother mapping)
    E.lastSeenDevices = count;
    E.arousal = clamp01(E.arousal + 0.1f * clamp01(count / 10.0f));

    if (VERBOSE_LEVEL >= VERBOSE_EMOTION) {
      Serial.print("BLE devices: ");
      Serial.println(count);
    }
  }

  // State
  static MoodState lastState = MoodState::Idle;
  MoodState s = chooseState(nowMs);
  if (s != lastState) {
    LOG_STATE("STATE change");
    lastState = s;
  }

  switch (s) {
    case MoodState::Idle:     patternIdle(nowMs); break;
    case MoodState::Excited:  patternExcited(nowMs); break;
    case MoodState::Anxious:  patternAnxious(nowMs); break;
    case MoodState::Friendly: patternFriendly(nowMs); break;
  }

  // Audio output driven by the same emotional state
  audioTick(nowMs, (uint8_t)s);

  // Emotional state logging
  logEmotionalState(nowMs);

  // Output
  if (ENABLE_PWM) {
    // PWM slice
    if (nowUs - lastPwmSliceUs >= PWM_SLICE_US) {
      lastPwmSliceUs = nowUs;
      pwmPhase = (pwmPhase + 1) % PWM_LEVELS;
      applyMask(computeMask());
    }
  } else {
    // Binary mode (crisper states, less "always on")
    if (nowMs - lastBinaryWriteMs >= BINARY_WRITE_EVERY_MS) {
      lastBinaryWriteMs = nowMs;
      applyMask(computeMaskBinary());
    }
  }
}

