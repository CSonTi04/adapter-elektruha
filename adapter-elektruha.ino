const int DELAY_TIME = 500;
const int GPIO4 = 4;

void setup() {
  // put your setup code here, to run once:
  pinMode(GPIO4, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(GPIO4, HIGH);
  delay(DELAY_TIME);  
  digitalWrite(GPIO4, LOW);
  delay(DELAY_TIME);  
}
