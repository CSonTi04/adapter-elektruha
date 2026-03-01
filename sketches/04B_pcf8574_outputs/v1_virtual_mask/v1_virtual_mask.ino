// =============================================================================
// 04B_pcf8574_outputs – Variant 1: Virtual mask (Serial only, no hardware)
// =============================================================================
// Concept: PCF8574 active-low output mask explained without hardware.
//
// The PCF8574 drives 8 outputs (P0–P7).  In anxious.ino the outputs are
// ACTIVE-LOW: writing LOW turns the LED on, writing HIGH turns it off.
// This matches the typical open-drain output behaviour of the PCF8574.
//
// A bitmask packs all 8 outputs into one byte:
//   bit 0 = P0, bit 1 = P1, ... bit 7 = P7
//   1 in the mask = LED ON  (we will write LOW)
//   0 in the mask = LED OFF (we will write HIGH)
//
// This sketch simulates writing masks without any hardware so you can see
// what the PCF8574 would do.
//
// Wiring: none – open Serial Monitor at 115200 baud.
// =============================================================================

// ---- Tuning knobs -----------------------------------------------------------
constexpr uint32_t STEP_MS = 600;  // how fast to advance the chase
// -----------------------------------------------------------------------------

// Print an 8-bit mask as a visual LED bar
void printMask(uint8_t mask) {
  Serial.print("P7-P0: ");
  for (int bit = 7; bit >= 0; bit--) {
    Serial.print((mask >> bit) & 1 ? "* " : ". ");
  }
  Serial.print("  (0x");
  if (mask < 0x10) Serial.print("0");
  Serial.print(mask, HEX);
  Serial.println(")");
}

// Helper matching anxius.ino: on -> LOW (active-low), off -> HIGH
// Returns the pin-level that the PCF8574 would receive.
// (In real code we pass this to pcf.write(pin, level))
bool pcfPinLevel(bool on) {
  return on ? LOW : HIGH;  // active-low
}

uint8_t chaseIdx = 0;
uint32_t lastStepMs = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("04B_pcf8574_outputs v1 – virtual mask (no hardware)");
  Serial.println("Active-low: '*' = LED on (PCF pin LOW), '.' = LED off (PCF pin HIGH)");
  Serial.println();
}

void loop() {
  uint32_t now = millis();
  if (now - lastStepMs < STEP_MS) return;
  lastStepMs = now;

  // Simple one-LED chase pattern
  uint8_t mask = (uint8_t)(1 << chaseIdx);
  printMask(mask);

  // Show how pcfPinLevel() maps each bit
  Serial.print("         ");
  for (int bit = 7; bit >= 0; bit--) {
    bool on = (mask >> bit) & 1;
    Serial.print(pcfPinLevel(on) == LOW ? "L " : "H ");
  }
  Serial.println("  (L=LED on, H=LED off)");
  Serial.println();

  chaseIdx = (chaseIdx + 1) % 8;
}
