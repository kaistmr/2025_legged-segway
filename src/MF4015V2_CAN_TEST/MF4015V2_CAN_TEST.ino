/*  MF4015 V2  Speed-Loop 예제 (Teensy 4.0)
 *  핀 : CAN1_RX = 22, CAN1_TX = 23
 *  라이브러리 : FlexCAN_T4 (Library Manager에서 설치)
 */
#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;
const uint32_t CAN_BAUD   = 1'000'000;   // 1 Mbps
const uint32_t MOTOR_ID   = 0x141;       // 0x140 + 1  (ID = 1)
const int32_t  TARGET_DPS = 600;         // 원하는 속도 [deg/s]
const int32_t  TARGET_RAW = TARGET_DPS * 100; // 0.01 dps/LSB

void setup() {
  Serial.begin(115200);
  Can0.begin();
  Can0.setBaudRate(CAN_BAUD);

  delay(100);              // 버스 안정 대기
  motorOn();               // ① 모터 ON
  delay(10);
}

void loop() {
  sendSpeed(TARGET_RAW);   // ② 속도 명령 주기적 전송
  delay(10);               // 100 Hz (10 ms 간격)
}

/* ---------- 유틸리티 함수 ---------- */
void motorOn() {
  CAN_message_t msg{};
  msg.id         = MOTOR_ID;
  msg.len        = 8;
  msg.flags.extended  = 0;      // 표준 ID
  msg.buf[0]     = 0x88;   // Motor ON  :contentReference[oaicite:8]{index=8}
  Can0.write(msg);
}

void sendSpeed(int32_t spd_raw) {
  CAN_message_t msg{};
  msg.id        = MOTOR_ID;
  msg.len       = 8;
  msg.flags.extended = 0;
  msg.buf[0]    = 0xA2;    // Speed closed-loop :contentReference[oaicite:9]{index=9}
  /* little-endian으로 0.01 dps 값을 DATA[4-7]에 삽입 */
  msg.buf[4] =  spd_raw        & 0xFF;
  msg.buf[5] = (spd_raw >> 8 ) & 0xFF;
  msg.buf[6] = (spd_raw >> 16) & 0xFF;
  msg.buf[7] = (spd_raw >> 24) & 0xFF;
  Can0.write(msg);
}
