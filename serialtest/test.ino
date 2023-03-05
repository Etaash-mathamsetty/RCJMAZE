void setup() {
  // put your setup code here, to run once:
  Serial2.begin(9600);
  Serial.begin(9600);
  Serial.println("starting the code");
  
}

void loop() {
  // put your main code here, to run repeatedly:
  // Serial.println("test");
  Serial2.println("hello world");
  while (!Serial2.available());
  while (Serial2.available()) {
  char output = Serial2.read();
  Serial.println(output);
  }
  delay(1000);

}
