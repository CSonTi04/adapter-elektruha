// =============================================================================
// 02_verbose_macros – Variant 3: Full pre-I2C
// =============================================================================
// Concept: VERBOSE_LEVEL macros in the context of the full build.
// Adds an extra state-machine enum (MoodState) and logs its transitions,
// mirroring what anxius.ino does.  I2C / PCF8574 not yet introduced.
//
// Wiring: just the ESP32 board – built-in LED on GPIO2.
// =============================================================================

#define VERBOSE_NONE     0
#define VERBOSE_STATE    1
#define VERBOSE_EMOTION  2
#define VERBOSE_FULL     3

// <<<< CHANGE THIS >>>>
#define VERBOSE_LEVEL    VERBOSE_EMOTION

#if VERBOSE_LEVEL >= VERBOSE_STATE
  #define LOG_STATE(x)   Serial.println(x)
#else
  #define LOG_STATE(x)
#endif

#if VERBOSE_LEVEL >= VERBOSE_EMOTION
  #define LOG_EMOTION(x) Serial.println(x)
#else
  #define LOG_EMOTION(x)
#endif

#if VERBOSE_LEVEL >= VERBOSE_FULL
  #define LOG_FULL(x)    Serial.println(x)
#else
  #define LOG_FULL(x)
#endif

// ---- Tuning knobs -----------------------------------------------------------
constexpr int     LED_PIN   = 2;
constexpr uint32_t TICK_MS  = 1500;  // emotion log interval
// -----------------------------------------------------------------------------

// Mood state enum – mirrors anxius.ino
enum class MoodState : uint8_t { Idle, Excited, Anxious, Friendly };

// Minimal fake emotion values used to pick a state
float arousal   = 0.2f;
float anxiety   = 0.1f;
MoodState lastState = MoodState::Idle;

uint32_t lastTickMs  = 0;
uint32_t lastBlinkMs = 0;
bool     ledState    = false;

MoodState chooseState() {
  if (anxiety > 0.65f)  return MoodState::Anxious;
  if (arousal > 0.55f)  return MoodState::Excited;
  return MoodState::Idle;
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  Serial.println("02_verbose_macros v3 – full pre-I2C");
  LOG_STATE("Sculpture ready.");
}

void loop() {
  uint32_t now = millis();

  // Slowly grow arousal to show state transitions
  arousal += 0.0003f;
  if (arousal > 1.0f) arousal = 0.0f;

  // LED heartbeat
  if (now - lastBlinkMs >= 500) {
    lastBlinkMs = now;
    ledState    = !ledState;
    digitalWrite(LED_PIN, ledState ? HIGH : LOW);
  }

  // State-transition log
  MoodState s = chooseState();
  if (s != lastState) {
    lastState = s;
    LOG_STATE("STATE change");
  }

  // Periodic emotion log
  if (now - lastTickMs >= TICK_MS) {
    lastTickMs = now;
    LOG_EMOTION("EMOTION: arousal=" + String(arousal, 2) +
                " anxiety=" + String(anxiety, 2));
    LOG_FULL("FULL: raw detail here");
  }
}
