#include <Wire.h>

void setup(){
  Wire.begin();
  Wire.beginTransmission(0x68);
  Wire.write(107);
  Wire.write(0);
  Wire.endTransmission();
}

void loop(){
  
}