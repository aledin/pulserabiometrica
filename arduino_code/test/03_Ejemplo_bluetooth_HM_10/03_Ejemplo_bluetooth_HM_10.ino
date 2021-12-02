#include <SoftwareSerial.h>

SoftwareSerial miBT(10, 11); //RX, TX

void setup() {
  // Open serial port
  Serial.begin(115200);
  // begin bluetooth serial port communication
  miBT.begin(9600);
}

// Now for the loop

void loop() {
  if (miBT.available()){
    Serial.write(miBT.read()); // lee BT y envía a Arduino
  }
  if (Serial.available()){
    miBT.write(Serial.read()); //lee Arduino y envia a BT
  }
}
