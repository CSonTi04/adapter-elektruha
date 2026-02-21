const int DELAY_TIME = 500;
const int GPIO4 = 4;

void setup() {
  // put your setup code here, to run once:
  pinMode(GPIO4, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(touchRead(T3));
}
