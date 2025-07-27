#include <Wire.h>

void setup(){
  Wire.begin();
  // 레지스터 27
  Wire.beginTransmission(0x68);
  Wire.write(27);
  Wire.write(0);
  //Register 28
  Wire.beginTransmission(0x68);
  Wire.write(28);
  Wire.write(0);
  Wire.endTransmission();
}