// =============================================================================
// 01_heartbeat_millis – Variant 1: Serial-only (no hardware required)
// =============================================================================
// Concept: Non-blocking timing using millis().
// Instead of delay(), we check whether enough time has passed and only then
// act.  This keeps loop() responsive for future sensors/outputs.
//
// Wiring: none – open Serial Monitor at 115200 baud.
//
// NEW TO CODING?  See sketch 00_language_concepts for a guided tour of every
// language feature used in this curriculum.
// =============================================================================

// ---- LEARN: Constants -------------------------------------------------------
// "constexpr" declares a constant – a value that NEVER changes while the
// program runs.  Give constants descriptive names (ALL_CAPS by convention) so
// you can see at a glance what the number means.
//
// "uint32_t" is the data TYPE.  It means "unsigned 32-bit integer" – a whole
// number from 0 to about 4 billion.  We use it for millisecond timestamps
// because they only count upward and can get very large.
//
// The "= 500" after the name sets the starting value (500 milliseconds).
// ---- Tuning knobs -----------------------------------------------------------
constexpr uint32_t BEAT_INTERVAL_MS = 500;  // how often to "beat" (ms)
// -----------------------------------------------------------------------------

// ---- LEARN: Variables -------------------------------------------------------
// A "variable" is a named box that holds a value you can change later.
// Unlike a constant, its value CAN be updated while the program runs.
//
// "uint32_t lastBeatMs" creates a variable called lastBeatMs that holds a
// positive whole number.  "= 0" sets its starting value to zero.
//
// We use lastBeatMs to remember WHEN the last beat happened so we can
// calculate how much time has passed since then.
// -----------------------------------------------------------------------------
uint32_t lastBeatMs = 0;  // timestamp of the last beat

// ---- LEARN: setup() and loop() – the Arduino model -------------------------
// Every Arduino sketch has two required functions:
//
//   setup()  – runs ONCE when the board powers on or resets.
//              Good place for one-time initialisation (start Serial, set pins…)
//
//   loop()   – runs OVER AND OVER FOREVER after setup() finishes.
//              Think of it as the heartbeat of your program.
//
// You don't call setup() or loop() yourself – the Arduino framework does it.
// -----------------------------------------------------------------------------

void setup() {
  // Serial.begin(115200) opens the USB serial connection at 115200 baud.
  // "Baud" is the speed of the connection (symbols per second).
  // Open Serial Monitor in the IDE (magnifying-glass icon) to see the output.
  Serial.begin(115200);
  Serial.println("01_heartbeat_millis v1 – Serial only");
  Serial.println("Watching millis()... each BEAT = 500 ms elapsed.");
}

void loop() {
  // ---- LEARN: Calling a function that returns a value ---------------------
  // millis() is a built-in Arduino function.  The parentheses () call it.
  // It RETURNS the number of milliseconds since the board powered on.
  // We store that number in the variable "now" so we can use it twice
  // without calling millis() again (which could give a slightly different
  // number the second time).
  // -------------------------------------------------------------------------
  uint32_t now = millis();

  // ---- LEARN: if-statement ------------------------------------------------
  // An "if" block runs its code only when the condition in ( ) is TRUE.
  //
  // Condition:  now - lastBeatMs >= BEAT_INTERVAL_MS
  //   now - lastBeatMs  →  how many ms have passed since the last beat
  //   >=                →  "greater than or equal to"
  //   BEAT_INTERVAL_MS  →  our 500 ms target
  //
  // So: "if at least 500 ms have passed since the last beat, do the beat."
  // -------------------------------------------------------------------------
  if (now - lastBeatMs >= BEAT_INTERVAL_MS) {
    // ---- LEARN: Assignment --------------------------------------------------
    // "lastBeatMs = now" UPDATES the variable lastBeatMs to the current time.
    // A single "=" is assignment (store a value).
    // Two "==" is comparison (check if two things are equal).
    // -------------------------------------------------------------------------
    lastBeatMs = now;
    Serial.print("BEAT  t=");
    Serial.print(now);
    Serial.println(" ms");
  }

  // Nothing else is blocked – you can add more timed events here.
}
