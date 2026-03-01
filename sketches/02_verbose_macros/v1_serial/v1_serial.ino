// =============================================================================
// 02_verbose_macros – Variant 1: Serial-only (no hardware required)
// =============================================================================
// Concept: Conditional compile-time logging levels.
// anxius.ino uses VERBOSE_LEVEL macros so you can control how much Serial
// output you see without any runtime overhead.
//
// This sketch lets you experiment with all four levels and see exactly what
// each one prints.  Change VERBOSE_LEVEL and re-upload to see the difference.
//
// Wiring: none – open Serial Monitor at 115200 baud.
// =============================================================================

// ---- Verbosity levels (same names as anxius.ino) ----------------------------
#define VERBOSE_NONE     0
#define VERBOSE_STATE    1
#define VERBOSE_EMOTION  2
#define VERBOSE_FULL     3

// <<<< CHANGE THIS to 0, 1, 2, or 3 and re-upload >>>>
#define VERBOSE_LEVEL    VERBOSE_EMOTION
// -----------------------------------------------------------------------------

// Macro definitions – only expand if the current level is high enough.
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
constexpr uint32_t TICK_MS = 1000;  // how often to emit one round of logs
// -----------------------------------------------------------------------------

uint32_t lastTickMs = 0;

void setup() {
  Serial.begin(115200);
  Serial.print("02_verbose_macros v1 – VERBOSE_LEVEL=");
  Serial.println(VERBOSE_LEVEL);
}

void loop() {
  uint32_t now = millis();
  if (now - lastTickMs < TICK_MS) return;
  lastTickMs = now;

  // These lines only compile in when the corresponding level is active.
  LOG_STATE("STATE: Idle");            // printed at level >= 1
  LOG_EMOTION("EMOTION: arousal=0.3"); // printed at level >= 2
  LOG_FULL("FULL: touch raw=450");     // printed at level >= 3
}
