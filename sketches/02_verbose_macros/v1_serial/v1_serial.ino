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

// ---- LEARN: #define (preprocessor substitution) ----------------------------
// "#define NAME VALUE" tells the compiler: "before compiling, replace every
// occurrence of NAME with VALUE in the source code."
//
// This happens BEFORE the compiler even sees the code — it is called
// "pre-processing" (hence the # prefix).
//
// Here we give friendly names (VERBOSE_NONE, VERBOSE_STATE, …) to the
// numbers 0, 1, 2, 3.  This makes the code readable: you see
// "VERBOSE_EMOTION" and instantly understand it, rather than wondering
// what the magic number "2" means.
// ---- Verbosity levels (same names as anxius.ino) ----------------------------
#define VERBOSE_NONE     0
#define VERBOSE_STATE    1
#define VERBOSE_EMOTION  2
#define VERBOSE_FULL     3

// <<<< CHANGE THIS to 0, 1, 2, or 3 and re-upload >>>>
#define VERBOSE_LEVEL    VERBOSE_EMOTION
// -----------------------------------------------------------------------------

// ---- LEARN: #if / #else / #endif (compile-time conditionals) ---------------
// "#if CONDITION" is like a normal if-statement, but it runs at COMPILE TIME,
// not while the program is running.  Code inside a false #if block is not
// even compiled into the final program — it disappears completely.
//
// This is very useful for debug logging: with VERBOSE_LEVEL 0 you get zero
// overhead because the logging code simply doesn't exist in the binary.
//
// The #define LOG_STATE(x) with nothing after it creates an "empty macro" —
// calling LOG_STATE("anything") compiles to nothing at all.
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
  // ---- LEARN: early return --------------------------------------------------
  // "return" inside loop() exits the current call of loop() immediately.
  // The Arduino framework will call loop() again straight away, so this is
  // a clean way to say "nothing to do yet — come back later."
  // -------------------------------------------------------------------------
  if (now - lastTickMs < TICK_MS) return;
  lastTickMs = now;

  // These lines only compile in when the corresponding level is active.
  LOG_STATE("STATE: Idle");            // printed at level >= 1
  LOG_EMOTION("EMOTION: arousal=0.3"); // printed at level >= 2
  LOG_FULL("FULL: touch raw=450");     // printed at level >= 3
}
