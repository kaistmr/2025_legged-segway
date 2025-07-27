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

void loop(){
  uint8_t i;
  int16_t acc_raw[3], gyro_raw[3];

  Wire.beginTransmission(0x68);
  Wire.write(59);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  for(i = 0; i < 3; i++) acc_raw[i] = (Wire.read() << 8) | Wire.read();

  Wire.beginTransmission(0x68);
  Wire.write(67);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  for(i = 0; i < 3; i++) gyro_raw[i] = (Wire.read() << 8) | Wire.read();

  float angle[2], vec;
  vec = sqrt(pow(acc_raw[0], 2) + pow(acc_raw[2], 2));
  angle[0] = angle[0] * 0.98 + atan2(acc_raw[1], vec) * RAD_TO_DEG * 0.02;
  vec = sqrt(pow(acc_raw[1], 2) + pow(acc_raw[2], 2));
  angle[1] = angle[1] * 0.98 + atan2(acc_raw[0], vec) * RAD_TO_DEG * 0.02;

  char str[50], a1[10], a2[10];
  dtostrf(angle[0], 4, 3, a1);
  dtostrf(angle[1], 4, 3, a2);
  sprintf(str, "X:%s Y:%s", a1, a2);
  Serial.println(str);
}