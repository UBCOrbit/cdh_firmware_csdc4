void setup() {
  //arduino console
  Serial.begin(115200);
  //huart 1
  Serial1.begin(115200);
//  huart 6
  Serial2.begin(115200);
  Serial3.begin(115200);

  
  //Board 1B
  pinMode(30,INPUT);
  //Board 1C
  pinMode(34,INPUT);
  //Board 2B
  pinMode(38,INPUT);
  //Board 2C
  pinMode(42,INPUT);
  //Board 3B
  pinMode(46,INPUT);
  //Board 3C
  pinMode(50,INPUT);
}

void loop() {
//  Serial1.print("EDDEEEEEEEEEEEEEEEEEEDEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE");
//  Serial2.print("EDFEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE");
////  Serial.print("B - ");
////  Serial.print(digitalRead(30));
////  Serial.print("C - ");
////  Serial.print(digitalRead(34));
//  while(Serial3.available()>0){
//    Serial.print((char)Serial3.read());
//  }
  
  Serial1.print("AAHeelloooooHeelloooooHeelloooooHeelloooHeelloooooHeellooYZ");
  Serial3.print("AASuck my toes!YZ");

  int b1 = digitalRead(30);
  int c1 = digitalRead(34);
  int b2 = digitalRead(38);
  int c2 = digitalRead(42);
  int b3 = digitalRead(46);
  int c3 = digitalRead(50);

//  if(b1==1&&c1==1){
//    Serial.print("Reset board 1\n");
//  }else if(b2==1&&c2==1){
//    Serial.print("Reset board 2\n");
//  }else if(b3==1&&c3==1){
//    Serial.print("Reset board 3\n");
//  }else{
//    Serial.print("Alls good\n");
//  }

  while(Serial2.available()>0){
    Serial.print((char)Serial2.read());
  }
}



