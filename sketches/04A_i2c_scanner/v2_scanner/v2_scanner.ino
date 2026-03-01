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

// ---- LEARN: #include (importing a library) ----------------------------------
// "#include <Wire.h>" makes the Wire library available in this sketch.
// Wire is the Arduino library for I2C communication.
//
// A library is a collection of pre-written functions someone else has already
// tested and packaged.  Using "#include" is like saying "I want to use the
// tools from this toolbox."
//
// The angle brackets < > mean look in the standard system library path.
// Double quotes "MyFile.h" would mean look in the same folder as this sketch.
// -----------------------------------------------------------------------------
#include <Wire.h>

// ---- Tuning knobs -----------------------------------------------------------
// ---- LEARN: int (whole-number type) ----------------------------------------
// "int" is a whole number that can be positive or negative.
// Here we use it to store GPIO pin numbers (21 and 22).
// Pin numbers are always positive, but "int" is fine for small values.
// "constexpr int" makes them compile-time constants that can't be changed.
// -----------------------------------------------------------------------------
constexpr int SDA_PIN = 21;
constexpr int SCL_PIN = 22;
constexpr uint32_t SCAN_EVERY_MS = 5000;  // re-scan interval
// -----------------------------------------------------------------------------

uint32_t lastScanMs = 0;

void scanI2C() {
  Serial.println("--- I2C scan ---");
  int found = 0;

  // ---- LEARN: for loop -------------------------------------------------------
  // A "for" loop repeats a block of code, updating a counter each time.
  //
  // FORMAT: for (STARTING_STATE; KEEP_GOING_WHILE; UPDATE_EACH_TIME) { }
  //
  // Here:
  //   uint8_t addr = 0x08   → start at address 8 (skip reserved 0x00–0x07)
  //   addr <= 0x77          → keep going while addr is ≤ 119 (0x77 in hex)
  //   addr++                → add 1 to addr after each loop body runs
  //
  // "uint8_t" is an unsigned 8-bit integer: a whole number from 0 to 255.
  // I2C addresses are 7 bits (0–127), so uint8_t is a perfect fit.
  //
  // "0x08" is the number 8 written in HEXADECIMAL (base-16).  Hex is common
  // in hardware programming because each hex digit maps to exactly 4 binary
  // bits.  0x20 = 32 decimal, 0x77 = 119 decimal.
  // ---------------------------------------------------------------------------
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
