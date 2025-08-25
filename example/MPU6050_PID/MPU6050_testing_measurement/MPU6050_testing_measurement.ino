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

  char str[100] = "";
  sprintf(str, "Acc X:%d Y:%d Z:%d / Gyro X:%d Y:%d Z:%d",
  acc_raw[0], acc_raw[1], acc_raw[2],
  gyro_raw[0], gyro_raw[1], gyro_raw[2]);
  Serial.println(str);
}