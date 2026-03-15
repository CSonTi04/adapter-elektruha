// =============================================================================
// 10_ble_count – Variant 3: Task-based non-blocking BLE scan
// =============================================================================
// Concept: BLE scan running in a FreeRTOS task so loop() stays responsive.
//
// This is the exact pattern used in anxious.ino:
//   - bleScanTask() runs on core 0 (pinned).
//   - It sleeps, scans (blocking inside the task only), then updates shared
//     globals protected by a critical section (portMUX).
//   - loop() reads the latest count non-blocking via getBleDeviceCount().
//
// Wiring: just the ESP32 board.
// =============================================================================

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>

// ---- Tuning knobs -----------------------------------------------------------
constexpr uint32_t BLE_SCAN_EVERY_MS = 5000;
constexpr uint32_t BLE_SCAN_FOR_MS   = 3000; // ceil to seconds internally
constexpr uint32_t PRINT_MS          = 1000;
// -----------------------------------------------------------------------------

BLEScan* pBLEScan = nullptr;
float arousal = 0.0f;

// Shared BLE state (written by task, read by loop)
static volatile int      g_bleCount        = 0;
static volatile uint32_t g_bleLastUpdateMs = 0;
static portMUX_TYPE      g_bleMux          = portMUX_INITIALIZER_UNLOCKED;

float clamp01(float x) { return x < 0 ? 0 : (x > 1 ? 1 : x); }

// BLE scan task – runs on core 0, blocking scan stays inside this task only
void bleScanTask(void* pv) {
  (void)pv;
  vTaskDelay(pdMS_TO_TICKS(500)); // wait for setup to finish

  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(BLE_SCAN_EVERY_MS));
    if (!pBLEScan) continue;

    uint32_t durSec = (BLE_SCAN_FOR_MS + 999) / 1000;
    if (durSec < 1) durSec = 1;

    BLEScanResults* found = pBLEScan->start((int)durSec, false);
    int count = found ? found->getCount() : 0;
    pBLEScan->clearResults();

    portENTER_CRITICAL(&g_bleMux);
    g_bleCount        = count;
    g_bleLastUpdateMs = millis();
    portEXIT_CRITICAL(&g_bleMux);

    Serial.print("[BLE task] devices="); Serial.println(count);
  }
}

// Non-blocking read (safe from main loop)
int getBleDeviceCount(uint32_t* lastMsOut = nullptr) {
  portENTER_CRITICAL(&g_bleMux);
  int c      = g_bleCount;
  uint32_t t = g_bleLastUpdateMs;
  portEXIT_CRITICAL(&g_bleMux);
  if (lastMsOut) *lastMsOut = t;
  return c;
}

uint32_t lastPrintMs     = 0;
uint32_t lastDecayMs     = 0;
uint32_t lastAppliedBleMs = 0;

void setup() {
  Serial.begin(115200);
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(80);

  // Pin BLE task to core 0; Arduino loop runs on core 1
  xTaskCreatePinnedToCore(bleScanTask, "bleScanTask", 4096, nullptr, 1, nullptr, 0);

  Serial.println("10_ble_count v3 – task-based non-blocking scan");
}

void loop() {
  uint32_t now = millis();

  // Decay arousal every 50 ms
  if (now - lastDecayMs >= 50) {
    float dt = (now - lastDecayMs) / 1000.0f; lastDecayMs = now;
    arousal = clamp01(arousal - 0.06f * dt);
  }

  // Apply latest BLE result (only when new data arrives)
  uint32_t bleTs = 0;
  int count = getBleDeviceCount(&bleTs);
  if (bleTs != 0 && bleTs != lastAppliedBleMs) {
    lastAppliedBleMs = bleTs;
    arousal = clamp01(arousal + 0.1f * clamp01(count / 10.0f));
  }

  if (now - lastPrintMs >= PRINT_MS) {
    lastPrintMs = now;
    Serial.print("devices="); Serial.print(count);
    Serial.print("  arousal="); Serial.println(arousal, 2);
  }
}
