// =============================================================================
// 02_verbose_macros – Variant 2: Built-in LED feedback
// =============================================================================
// Concept: Same VERBOSE_LEVEL macros as v1, plus visual feedback:
// the built-in LED blinks faster when VERBOSE_LEVEL is higher, so you can
// see the verbosity "setting" without reading Serial.
//
// Wiring: just the ESP32 board – built-in LED on GPIO2.
// =============================================================================

#define VERBOSE_NONE     0
#define VERBOSE_STATE    1
#define VERBOSE_EMOTION  2
#define VERBOSE_FULL     3

// <<<< CHANGE THIS to 0, 1, 2, or 3 >>>>
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
constexpr int LED_PIN    = 2;     // built-in LED
constexpr uint32_t TICK_MS = 1000;
// Blink interval per level: level 0=off, 1=slow, 2=medium, 3=fast
constexpr uint32_t BLINK_MS[4] = {0, 800, 400, 100};
// -----------------------------------------------------------------------------

uint32_t lastTickMs  = 0;
uint32_t lastBlinkMs = 0;
bool     ledState    = false;

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  Serial.print("02_verbose_macros v2 – VERBOSE_LEVEL=");
  Serial.println(VERBOSE_LEVEL);
}

void loop() {
  uint32_t now = millis();

  // LED blink – speed reflects VERBOSE_LEVEL
  uint32_t blinkInterval = BLINK_MS[VERBOSE_LEVEL];
  if (blinkInterval > 0 && now - lastBlinkMs >= blinkInterval) {
    lastBlinkMs = now;
    ledState    = !ledState;
    digitalWrite(LED_PIN, ledState ? HIGH : LOW);
  } else if (blinkInterval == 0) {
    digitalWrite(LED_PIN, LOW);  // VERBOSE_NONE: LED stays off
  }

  // Periodic log output
  if (now - lastTickMs >= TICK_MS) {
    lastTickMs = now;
    LOG_STATE("STATE: Idle");
    LOG_EMOTION("EMOTION: arousal=0.3");
    LOG_FULL("FULL: touch raw=450");
  }
}
