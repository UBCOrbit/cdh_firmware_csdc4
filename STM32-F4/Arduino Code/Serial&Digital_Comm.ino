void setup() {
  //arduino console
  Serial.begin(115200);
  //huart 1
  Serial1.begin(115200);
  //huart 6
  Serial2.begin(115200);
  //some gpio
  pinMode(30,INPUT);
}

void loop() {
  Serial1.print("bbbbbbb");
  Serial2.print("aaaaaaaa");
  Serial.print(digitalRead(30));
  delay(500);
}



