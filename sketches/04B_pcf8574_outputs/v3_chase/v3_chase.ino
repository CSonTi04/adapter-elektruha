// =============================================================================
// 04B_pcf8574_outputs – Variant 3: Full build – chase pattern (active-low)
// =============================================================================
// Concept: Drive 8 PCF8574 outputs with an active-low chase pattern.
//
// PREREQUISITE: Run 04A v3 (I2C scanner) first and confirm that 0x20 appears
// in the scan results.  Only then wire LEDs and upload this sketch.
//
// Active-low wiring (matches anxius.ino):
//   Each PCF8574 pin → resistor (~220Ω) → LED anode → LED cathode → GND
//   Writing LOW  to the pin  = current flows  = LED ON
//   Writing HIGH to the pin  = no current     = LED OFF
//
// Wiring:
//   SDA → GPIO21  (4.7 kΩ pull-up to 3.3 V)
//   SCL → GPIO22  (4.7 kΩ pull-up to 3.3 V)
//   PCF8574 P0–P7 → 220 Ω → LED+ → LED- → GND
//   PCF8574 A0/A1/A2 → GND  (address 0x20)
//
// Library: install "PCF8574" by Renzo Mischianti via Arduino Library Manager.
// =============================================================================

#include <Wire.h>
#include <PCF8574.h>

// ---- Tuning knobs -----------------------------------------------------------
constexpr int     SDA_PIN  = 21;
constexpr int     SCL_PIN  = 22;
constexpr uint8_t PCF_ADDR = 0x20;
constexpr uint32_t STEP_MS = 150;  // chase speed
// -----------------------------------------------------------------------------

PCF8574 pcf(PCF_ADDR);

// Active-low helper: on=true → LOW (LED on), on=false → HIGH (LED off)
static inline void pcfWritePin(uint8_t pin, bool on) {
  pcf.write(pin, on ? LOW : HIGH);
}

uint8_t chaseIdx = 0;
uint32_t lastStepMs = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  pcf.begin();

  // All LEDs off on startup (HIGH = off for active-low)
  for (int i = 0; i < 8; i++) pcf.write(i, HIGH);

  Serial.println("04B_pcf8574_outputs v3 – chase (active-low)");
  Serial.println("If no LEDs light up: check wiring and confirm 0x20 with 04A v3.");
}

void loop() {
  uint32_t now = millis();
  if (now - lastStepMs < STEP_MS) return;
  lastStepMs = now;

  // Turn the previous LED off and the current LED on
  uint8_t prev = (chaseIdx == 0) ? 7 : chaseIdx - 1;
  pcfWritePin(prev, false);    // off
  pcfWritePin(chaseIdx, true); // on

  Serial.print("LED "); Serial.print(chaseIdx); Serial.println(" ON");

  chaseIdx = (chaseIdx + 1) % 8;
}
