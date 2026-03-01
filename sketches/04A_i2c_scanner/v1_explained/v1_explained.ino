// =============================================================================
// 04A_i2c_scanner – Variant 1: Explained (no hardware required)
// =============================================================================
// Concept: What is I2C and why do we need to scan for addresses?
//
// I2C (Inter-Integrated Circuit) is a two-wire bus that lets many devices
// share the same SDA (data) and SCL (clock) lines.  Each device has a
// 7-bit address (0x00–0x7F) chosen by the manufacturer or set by hardware
// address pins.
//
// A scanner works by trying to talk to every possible address and listening
// for an ACK (acknowledgement) byte.  If a device responds, we print its
// address – that's how we discover what's wired up.
//
// In the full project, we expect to find the PCF8574 GPIO expander at
// address 0x20 (default with A2/A1/A0 all LOW).
//
// This variant prints a step-by-step explanation in Serial.  Run it on the
// ESP32 and read the output to understand the protocol before wiring I2C.
//
// Wiring: none – open Serial Monitor at 115200 baud.
// =============================================================================

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("=== 04A_i2c_scanner v1 – I2C concept explained ===");
  Serial.println();
  Serial.println("I2C quick facts:");
  Serial.println("  Wire 1: SDA (data)  – GPIO21 on this ESP32 project");
  Serial.println("  Wire 2: SCL (clock) – GPIO22 on this ESP32 project");
  Serial.println("  Devices share BOTH lines (multi-drop bus).");
  Serial.println("  Each device has a 7-bit address: 0x00 to 0x7F.");
  Serial.println();
  Serial.println("How a scanner works:");
  Serial.println("  for addr = 0x08 to 0x77:");
  Serial.println("    START signal");
  Serial.println("    send (addr << 1)  // address byte, write bit");
  Serial.println("    if device ACKs -> found! print address");
  Serial.println("    else           -> no device at this address");
  Serial.println("    STOP signal");
  Serial.println();
  Serial.println("Addresses to expect in this project:");
  Serial.println("  0x20 – PCF8574 (A2=A1=A0=LOW, default address)");
  Serial.println("  0x21..0x27 – PCF8574 with address pins set HIGH");
  Serial.println("  0x38..0x3F – PCF8574A variant");
  Serial.println();
  Serial.println("Next step: wire SDA=GPIO21 and SCL=GPIO22 with 4.7k pull-ups");
  Serial.println("  to 3.3V, then upload 04A v2 (real scanner).");
  Serial.println();
  Serial.println("Why separate scanner from PCF output control?");
  Serial.println("  Scanning first confirms your wiring is correct BEFORE");
  Serial.println("  you try to drive outputs.  It is much easier to debug.");
}

void loop() {
  // Nothing to do – explanation is printed once in setup().
}
