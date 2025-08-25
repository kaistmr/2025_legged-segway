#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
const uint32_t CAN_BAUD   = 1'000'000;   // 1 Mbps
const uint32_t MOTOR_ID   = 0x141;       // 0x140 + 1  (ID = 1)
const int32_t  TARGET_DPS = 600;         // 원하는 속도 [deg/s]
const int32_t  TARGET_RAW = TARGET_DPS * 100; // 0.01 dps/LSB

void setup() {
  Serial.begin(115200);
  Can1.begin();
  Can1.setBaudRate(CAN_BAUD);

  delay(100);              // 버스 안정 대기
  motorOn();               // ① 모터 ON
  delay(10);

  //setTorque(3.0);  // +3 A
}

void loop() {
  sendSpeed(TARGET_RAW);   // ② 속도 명령 주기적 전송
  delay(10);               // 100 Hz (10 ms 간격)



//  static uint32_t prev = 0;
//  const uint32_t period_ms = 10;   // 100 Hz 주기
//
//  if (millis() - prev >= period_ms) {
//    prev = millis();
//    setTorque(current_cmd_amp);    // 원하는 전류[A]를 변수로
//    requestState();                // 0x9A 등으로 상태 읽기 (옵션)
//  }
}



/* ---------- 유틸리티 함수 ---------- */
void motorOn() {
  CAN_message_t msg{};
  msg.id = MOTOR_ID;  msg.len = 8;
  msg.buf[0] = 0x88;  
  Can1.write(msg);
}

void setTorque(float amp){         // amp: +/- 전류[A]
  int16_t iq = amp / 16.5f * 2048; 
  CAN_message_t m{};
  m.id = MOTOR_ID;  m.len = 8;  m.buf[0] = 0xA1;
  memcpy(&m.buf[4], &iq, 2);       
  Can1.write(m);
}

void sendSpeed(int32_t spd_raw) {
  CAN_message_t msg{};
  msg.id = MOTOR_ID;  msg.len = 8;
  msg.buf[0] = 0xA2;
  /* little-endian으로 0.01 dps 값을 DATA[4-7]에 삽입 */
  msg.buf[4] =  spd_raw        & 0xFF;
  msg.buf[5] = (spd_raw >> 8 ) & 0xFF;
  msg.buf[6] = (spd_raw >> 16) & 0xFF;
  msg.buf[7] = (spd_raw >> 24) & 0xFF;

  //memcpy(&msg.buf[4], &spd_raw, 4); 
  Can1.write(msg);
}
