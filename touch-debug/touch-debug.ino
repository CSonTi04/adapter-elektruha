// ESP32 Touch Debug Sketch
// Opens Serial Monitor at 115200 and prints live touch telemetry.
// Commands in Serial Monitor:
//   c = recalibrate baseline
//   + = increase sensitivity (higher threshold ratio)
//   - = decrease sensitivity (lower threshold ratio)

constexpr int TOUCH_PIN = 4;               // touchRead GPIO
constexpr uint32_t PRINT_EVERY_MS = 60;    // telemetry rate

float thresholdRatio = 0.85f;              // touched when filtered < baseline * ratio
float filterAlpha = 0.18f;                 // smoothing (0..1)

int baseline = 0;
float filtered = 0.0f;
int minSeen = 1000000;
int maxSeen = -1000000;

bool touched = false;
bool lastTouched = false;

uint32_t lastPrintMs = 0;

int calibrateBaseline() {
  delay(300);
  long sum = 0;
  const int samples = 80;
  for (int i = 0; i < samples; i++) {
    int value = touchRead(TOUCH_PIN);
    sum += value;
    delay(8);
  }
  return (int)(sum / samples);
}

void printHelp() {
  Serial.println();
  Serial.println("=== ESP32 TOUCH DEBUG ===");
  Serial.println("Commands:");
  Serial.println("  c  -> recalibrate baseline");
  Serial.println("  +  -> increase sensitivity");
  Serial.println("  -  -> decrease sensitivity");
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(500);

  baseline = calibrateBaseline();
  filtered = baseline;
  minSeen = baseline;
  maxSeen = baseline;

  printHelp();
  Serial.print("Initial baseline: ");
  Serial.println(baseline);
}

void loop() {
  while (Serial.available() > 0) {
    char cmd = (char)Serial.read();
    if (cmd == 'c' || cmd == 'C') {
      baseline = calibrateBaseline();
      filtered = baseline;
      minSeen = baseline;
      maxSeen = baseline;
      Serial.print("Recalibrated baseline: ");
      Serial.println(baseline);
    } else if (cmd == '+') {
      thresholdRatio += 0.01f;
      if (thresholdRatio > 0.98f) thresholdRatio = 0.98f;
      Serial.print("thresholdRatio = ");
      Serial.println(thresholdRatio, 3);
    } else if (cmd == '-') {
      thresholdRatio -= 0.01f;
      if (thresholdRatio < 0.50f) thresholdRatio = 0.50f;
      Serial.print("thresholdRatio = ");
      Serial.println(thresholdRatio, 3);
    }
  }

  int raw = touchRead(TOUCH_PIN);
  filtered = (1.0f - filterAlpha) * filtered + filterAlpha * raw;

  if (raw < minSeen) minSeen = raw;
  if (raw > maxSeen) maxSeen = raw;

  int threshold = (int)(baseline * thresholdRatio);
  touched = (filtered < threshold);

  if (touched != lastTouched) {
    Serial.print("EVENT: ");
    Serial.println(touched ? "TOUCH START" : "TOUCH END");
    lastTouched = touched;
  }

  uint32_t now = millis();
  if (now - lastPrintMs >= PRINT_EVERY_MS) {
    lastPrintMs = now;

    Serial.print("raw=");
    Serial.print(raw);
    Serial.print(" filt=");
    Serial.print(filtered, 1);
    Serial.print(" base=");
    Serial.print(baseline);
    Serial.print(" thr=");
    Serial.print(threshold);
    Serial.print(" min=");
    Serial.print(minSeen);
    Serial.print(" max=");
    Serial.print(maxSeen);
    Serial.print(" touch=");
    Serial.println(touched ? "1" : "0");
  }
}
