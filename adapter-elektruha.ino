const int DELAY_TIME = 500;
const int GPIO4 = 4;
const int GPIO5 = 5;

void setup() {
  // put your setup code here, to run once:
  pinMode(GPIO4, OUTPUT);
  pinMode(GPIO5, OUTPUT);
  tone(GPIO5, 220, 500);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(touchRead(T3));
  // a saját megfigylet touchRead-et kellene, a legalacsonyabb és legmagasabb értéket
  int ledBrightnessValue = map(touchRead(T3), 0, 1000, 0, 255);
  analogWrite(GPIO4, ledBrightnessValue);
  delay(DELAY_TIME);
}
