const int DELAY_TIME = 500;
const int GPIO4 = 4;

void setup() {
  // put your setup code here, to run once:
  pinMode(GPIO4, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(LED_BUILTIN, HIGH);
  delay(DELAY_TIME);  
  digitalWrite(LED_BUILTIN, LOW);
  delay(DELAY_TIME);  
}
