void setup() {
  //arduino console
  Serial.begin(115200);
  //huart 1
  Serial2.begin(115200);
  Serial3.begin(115200);
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
  
  while(Serial2.available()>0){
    Serial.println(Serial2.readString());
  }
  
  /*
  while(Serial3.available()>0){
    Serial.println(Serial3.readString());
  }
  */
  //Serial.println("Waiting\n");
}



