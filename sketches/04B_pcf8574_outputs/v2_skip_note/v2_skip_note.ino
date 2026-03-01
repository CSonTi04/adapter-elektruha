// =============================================================================
// 04B_pcf8574_outputs – Variant 2: Minimal hardware (skip note)
// =============================================================================
// Concept: PCF8574 output control – minimal-hardware note.
//
// The PCF8574 is an I2C device and is part of the *full* build (Variant 3).
// If you only have an ESP32 board (minimal hardware), you cannot run the
// real PCF8574 output patterns from this sketch.
//
// What this sketch does instead:
//   - Confirms I2C is initialised correctly.
//   - Prints a clear message explaining that a PCF8574 is required.
//   - Falls back to blinking the built-in LED so the board appears alive.
//
// When you are ready for the full build, use Variant 3 of this sketch.
//
// Wiring: just the ESP32 board.
// =============================================================================

#include <Wire.h>

// ---- Tuning knobs -----------------------------------------------------------
constexpr int     LED_PIN  = 2;    // built-in LED
constexpr int     SDA_PIN  = 21;
constexpr int     SCL_PIN  = 22;
constexpr uint32_t BLINK_MS = 500;
// -----------------------------------------------------------------------------

uint32_t lastBlinkMs = 0;
bool     ledState    = false;

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);

  // Initialise I2C so the bus is ready even without a PCF8574
  Wire.begin(SDA_PIN, SCL_PIN);

  Serial.println("04B_pcf8574_outputs v2 – minimal hardware note");
  Serial.println();
  Serial.println("This sketch requires a PCF8574 I2C GPIO expander wired to");
  Serial.println("SDA=GPIO21 and SCL=GPIO22 to drive real LED outputs.");
  Serial.println();
  Serial.println("  -> Run 04A v2 (I2C scanner) first to confirm your PCF8574");
  Serial.println("     is detected at address 0x20.");
  Serial.println("  -> Then upload 04B v3 (full build) for the chase pattern.");
  Serial.println();
  Serial.println("For now: built-in LED blinks to show the sketch is running.");
}

void loop() {
  uint32_t now = millis();
  if (now - lastBlinkMs >= BLINK_MS) {
    lastBlinkMs = now;
    ledState    = !ledState;
    digitalWrite(LED_PIN, ledState ? HIGH : LOW);
  }
}
