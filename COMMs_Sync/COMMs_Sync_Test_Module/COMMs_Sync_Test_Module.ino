/*
 * The test module for the COMMs Sync Code on STM. 
 */


int packet = 0b01110111; // Equivalent to w in ASCII.
int done = 0;

void setup() {
  
  Serial.begin(115200);
  
}

void loop() {

  if ((Serial.available() == 1) && (done == 0)){
    Serial.write(packet);
    done = 1;
  }

  

}
