// 진동이 심한 경우 사용하는 DLPF 적용 코드
#include <Wire.h>

void setup(){
  Wire.begin();
  // 레지스터 107
  Wire.beginTransmission(0x68);
  Wire.write(107);
  Wire.write(0);
  Wire.enTransmission();
  // 레지스터 26
  for(uint8_t i = 2;, i <= 7; i++){
    Wire.beginTransmission(0x68);
    Wire.write(26);
    Wire.write(i << 3 | 0x02); //높게 설정할 수록 아웃라이어가 미약해지지만 반응속도가 느려짐
    Wire.endTransmission();
  }
  // 레지스터 27
  Wire.beginTransmission(0x68);
  Wire.write(27);
  Wire.write(0); //-> 감도 변환은 가령 2000deg/sec의 경우 3 << 3으로 시행 -> 아래 LPF 변수값(e.g 16.1)도 변환해야 함
  //레지스터 28
  Wire.beginTransmission(0x68);
  Wire.write(28);
  Wire.write(0);
  Wire.endTransmission();
}

int16_t offset[3] = {} //offset은 시험 후 직접 지정

void loop(){
  float SSF = 131.0;

  uint8_t i;
  int16_t acc_raw[3]={0,}, gyro_raw[3]={0,};

  Wire.beginTransmission(0x68);
  Wire.write(59);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  for(i = 0; i < 3; i++) acc_raw[i] = (Wire.read() << 8) | Wire.read();

  Wire.beginTransmission(0x68);
  Wire.write(67);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  for(i = 0; i < 3; i++) gyro_raw[i] = (Wire.read() << 8) | Wire.read() - offset[i];

  //값이 튈 경우 LPF 구조 시행
  // for(i = 0; i < 3; i++) gyro_raw[i] = gyro_raw[i] * 0.8 + 0.2 * (Wire.read() << 8) | Wire.read() - offset[i];

  static unsigned long p = 0;
  unsigned long c = micros();
  float dt = (c-p) * 0.000001F;
  p = c;

  float gyro_rate[3];
  for(i = 0; i<3; i++) gyro_rate[i] = gyro_raw[i] / SSF * dt;

  float angle[3]={0,}, vec;
  vec = sqrt(pow(acc_raw[0], 2) + pow(acc_raw[2], 2));
  angle[0] = (angle[0] + gyro_rate[0]) * 0.98 + atan2(acc_raw[1], vec) * RAD_TO_DEG * 0.02;
  vec = sqrt(pow(acc_raw[1], 2) + pow(acc_raw[2], 2));
  angle[1] = (angle[1] - gyro_rate[1]) * 0.98 + atan2(acc_raw[0], vec) * RAD_TO_DEG * 0.02;

  angle[2] += gyro_rate[2];

  char str[50], a1[10], a2[10], a3[10];
  dtostrf(angle[0], 4, 3, a1);
  dtostrf(angle[1], 4, 3, a2);
  dtostrf(angle[2], 4, 3, a3);
  sprintf(str, "X:%s Y:%s Z:%s", a1, a2, a3);
  Serial.println(str);
}