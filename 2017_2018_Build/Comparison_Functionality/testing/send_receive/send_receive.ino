int i =0;

void setup() {
  //arduino console
  Serial.begin(115200);
  Serial1.begin(115200);
  //huart 1
  Serial2.begin(115200);
  Serial3.begin(115200);

  /*
  if(i==0){
    Serial2.write("07");
    i=1;
  }
  */
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
  // Arduino acting as A. Send address and numbytes

  // Arduino acting as B. Send data
  /*
   * if(i==0){
      Serial2.write("07");
      i=1;
    }
   */
  //Serial.println("Reading");
  while(Serial2.available()>0){
      Serial.println(Serial2.readString());
  }

  /*
  if(i==0){
    Serial2.write("07");
    i=0;
  }
  */
  /*
  while(Serial3.available()>0){
    Serial.println(Serial3.readString());
  }
  */
  //Serial.println("Waiting\n");
}



