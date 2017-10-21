void setup() {
  //arduino console
  Serial.begin(115200);
  //huart 1
  Serial1.begin(115200);
//  huart 6
  //Serial2.begin(115200);

  //Board 1B
  /*
  pinMode(18,INPUT);
  //Board 1C
  pinMode(19,INPUT);
  //Board 2B
  pinMode(38,INPUT);
  //Board 2C
  pinMode(42,INPUT);
  */
}

void loop() {
  while(Serial1.available()>0){
    Serial.println(Serial1.readString());
  }
}



