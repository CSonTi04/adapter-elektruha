// =============================================================================
// 10_ble_count – Variant 2: Blocking BLE scan count
// =============================================================================
// Concept: Real BLE scan that counts nearby devices.
// This is a simplified, blocking version – the scan runs in setup() once,
// then prints the result.  In loop() it repeats every BLE_SCAN_EVERY_MS.
//
// Warning: The BLE scan call is BLOCKING for BLE_SCAN_FOR_SEC seconds.
// This freezes everything else while scanning.  That is why the full project
// (variant 3) moves scanning to a separate FreeRTOS task.
//
// Wiring: just the ESP32 board (built-in BLE antenna).
// =============================================================================

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>

// ---- Tuning knobs -----------------------------------------------------------
constexpr uint32_t BLE_SCAN_EVERY_MS = 6000; // ms between scans
constexpr int      BLE_SCAN_FOR_SEC  = 3;    // scan duration (blocking)
// -----------------------------------------------------------------------------

BLEScan* pBLEScan = nullptr;
float    arousal  = 0.0f;
uint32_t lastScanMs = 0;

float clamp01(float x) { return x < 0 ? 0 : (x > 1 ? 1 : x); }

int doBleScan() {
  Serial.print("[BLE] scanning for "); Serial.print(BLE_SCAN_FOR_SEC); Serial.println("s...");
  BLEScanResults* found = pBLEScan->start(BLE_SCAN_FOR_SEC, false);
  int count = found ? found->getCount() : 0;
  pBLEScan->clearResults();
  Serial.print("[BLE] found "); Serial.print(count); Serial.println(" device(s)");
  return count;
}

void setup() {
  Serial.begin(115200);
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(80);
  Serial.println("10_ble_count v2 – blocking scan");
  Serial.println("NOTE: loop() freezes during scan.  See v3 for non-blocking.");

  // First scan at startup
  int count = doBleScan();
  arousal = clamp01(arousal + 0.1f * clamp01(count / 10.0f));
  lastScanMs = millis();
}

void loop() {
  uint32_t now = millis();

  // Decay arousal
  static uint32_t lastDecayMs = 0;
  if (now - lastDecayMs >= 50) {
    float dt = (now - lastDecayMs) / 1000.0f; lastDecayMs = now;
    arousal = clamp01(arousal - 0.06f * dt);
    Serial.print("arousal="); Serial.println(arousal, 2);
  }

  // Next scan (blocks loop for BLE_SCAN_FOR_SEC)
  if (now - lastScanMs >= BLE_SCAN_EVERY_MS) {
    lastScanMs = now;
    int count = doBleScan();
    arousal = clamp01(arousal + 0.1f * clamp01(count / 10.0f));
  }
}
