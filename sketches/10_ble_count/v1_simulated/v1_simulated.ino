// =============================================================================
// 10_ble_count – Variant 1: Simulated random walk (no hardware)
// =============================================================================
// Concept: How BLE device count drives arousal in anxius.ino.
//
// In the full project, a background FreeRTOS task periodically scans for
// nearby BLE devices and updates a shared counter.  The main loop reads that
// counter and bumps arousal proportionally.
//
// This variant simulates a "random walk" BLE count so you can study the
// arousal mapping without any real BLE hardware or blocking scan.
//
// Wiring: none – open Serial Monitor at 115200 baud.
// =============================================================================

// ---- Tuning knobs -----------------------------------------------------------
constexpr uint32_t FAKE_SCAN_MS  = 3000;  // pretend scan interval
constexpr int      MAX_DEVICES   = 18;    // full crowd = 18 devices
constexpr uint32_t PRINT_MS      = 500;
// -----------------------------------------------------------------------------

int   fakeDeviceCount = 0;
float arousal         = 0.0f;
uint32_t lastScanMs   = 0;
uint32_t lastPrintMs  = 0;

float clamp01(float x) { return x < 0 ? 0 : (x > 1 ? 1 : x); }

void setup() {
  Serial.begin(115200);
  randomSeed(42);
  Serial.println("10_ble_count v1 – simulated random walk");
  Serial.println("devices  arousal");
}

void loop() {
  uint32_t now = millis();

  // Simulated scan: random walk ±2 devices per scan
  if (now - lastScanMs >= FAKE_SCAN_MS) {
    lastScanMs = now;
    fakeDeviceCount += (int)random(-2, 3);
    if (fakeDeviceCount < 0)            fakeDeviceCount = 0;
    if (fakeDeviceCount > MAX_DEVICES)  fakeDeviceCount = MAX_DEVICES;

    // Arousal update (same formula as anxius.ino)
    arousal = clamp01(arousal + 0.1f * clamp01((float)fakeDeviceCount / 10.0f));
  }

  // Natural arousal decay (same rate as anxius.ino tickEmotions)
  static uint32_t lastDecayMs = 0;
  if (now - lastDecayMs >= 50) {
    float dt = (now - lastDecayMs) / 1000.0f; lastDecayMs = now;
    arousal = clamp01(arousal - 0.06f * dt);
  }

  if (now - lastPrintMs >= PRINT_MS) {
    lastPrintMs = now;
    Serial.print(fakeDeviceCount); Serial.print("  ");
    Serial.println(arousal, 2);
  }
}
