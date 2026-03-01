// =============================================================================
// 04A_i2c_scanner – Variant 3: Full scanner (pre-PCF output control)
// =============================================================================
// Concept: I2C scanner in the full project context, confirming that the
// PCF8574 is wired correctly before we start driving its outputs in 04B.
//
// NOTE: This sketch scans for and *identifies* the PCF8574 address, but
// does NOT write any output to it yet.  Driving PCF8574 outputs is
// introduced in sketch 04B after you have confirmed the address here.
//
// Wiring:
//   SDA → GPIO21  (with 4.7 kΩ pull-up to 3.3 V)
//   SCL → GPIO22  (with 4.7 kΩ pull-up to 3.3 V)
//   PCF8574 VCC → 3.3 V or 5 V (check your module)
//   PCF8574 A0/A1/A2 → GND (default address 0x20)
//
// SAFETY NOTE:
//   ESP32 GPIOs are 3.3 V only – do NOT connect a 5 V I2C bus directly.
//   Always use pull-up resistors (4.7 kΩ) on SDA and SCL.
//   Verify VCC/logic levels before powering up.
//   This sketch is for educational/experimental use, provided "AS IS"
//   with no warranty.  See DISCLAIMER.md in the project root for details.
// =============================================================================

#include <Wire.h>

// ---- Tuning knobs -----------------------------------------------------------
constexpr int SDA_PIN = 21;
constexpr int SCL_PIN = 22;
constexpr uint8_t EXPECTED_PCF_ADDR = 0x20; // default PCF8574 address
constexpr uint32_t SCAN_EVERY_MS = 8000;
// -----------------------------------------------------------------------------

uint32_t lastScanMs = 0;
bool pcfFound = false;

void scanI2C() {
  Serial.println("--- I2C scan ---");
  int found = 0;
  pcfFound = false;

  for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();

    if (err == 0) {
      Serial.print("  0x");
      if (addr < 0x10) Serial.print("0");
      Serial.print(addr, HEX);

      if (addr >= 0x20 && addr <= 0x27) {
        Serial.print("  <- PCF8574");
        if (addr == EXPECTED_PCF_ADDR) {
          Serial.print(" (expected address OK)");
          pcfFound = true;
        }
      }
      if (addr >= 0x38 && addr <= 0x3F) Serial.print("  <- PCF8574A");
      Serial.println();
      found++;
    }
  }

  if (found == 0) {
    Serial.println("  No devices found – check wiring!");
  }

  Serial.print("  PCF8574 ready for output control: ");
  Serial.println(pcfFound ? "YES – proceed to 04B" : "NO – fix wiring first");
  Serial.println("-----------------");
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.println("04A_i2c_scanner v3 – full scanner (pre-PCF control)");
  Serial.println("Once 0x20 is confirmed below, move on to sketch 04B.");
  scanI2C();
  lastScanMs = millis();
}

void loop() {
  if (millis() - lastScanMs >= SCAN_EVERY_MS) {
    lastScanMs = millis();
    scanI2C();
  }
}
