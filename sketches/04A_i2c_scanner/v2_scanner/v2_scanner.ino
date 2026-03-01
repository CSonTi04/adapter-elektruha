// =============================================================================
// 04A_i2c_scanner – Variant 2: Real I2C scanner
// =============================================================================
// Concept: Scan the I2C bus and report what addresses respond.
//
// Run this BEFORE uploading any sketch that drives I2C devices.
// It confirms your wiring is correct and shows the address of every device
// that is connected.
//
// Wiring:
//   SDA → GPIO21  (with 4.7 kΩ pull-up to 3.3 V)
//   SCL → GPIO22  (with 4.7 kΩ pull-up to 3.3 V)
//   Connect your PCF8574 (or any I2C device) to the same bus.
// =============================================================================

#include <Wire.h>

// ---- Tuning knobs -----------------------------------------------------------
constexpr int SDA_PIN = 21;
constexpr int SCL_PIN = 22;
constexpr uint32_t SCAN_EVERY_MS = 5000;  // re-scan interval
// -----------------------------------------------------------------------------

uint32_t lastScanMs = 0;

void scanI2C() {
  Serial.println("--- I2C scan ---");
  int found = 0;

  for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();

    if (err == 0) {
      Serial.print("  Found device at 0x");
      if (addr < 0x10) Serial.print("0");
      Serial.print(addr, HEX);

      // Hint for known project addresses
      if (addr >= 0x20 && addr <= 0x27) Serial.print("  <- PCF8574");
      if (addr >= 0x38 && addr <= 0x3F) Serial.print("  <- PCF8574A");

      Serial.println();
      found++;
    }
  }

  if (found == 0) {
    Serial.println("  No I2C devices found.");
    Serial.println("  Check: wiring, pull-ups, power, and device address pins.");
  } else {
    Serial.print("  Total: ");
    Serial.print(found);
    Serial.println(" device(s).");
  }
  Serial.println("--- scan done ---");
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.println("04A_i2c_scanner v2 – real scanner");
  scanI2C();
  lastScanMs = millis();
}

void loop() {
  if (millis() - lastScanMs >= SCAN_EVERY_MS) {
    lastScanMs = millis();
    scanI2C();
  }
}
