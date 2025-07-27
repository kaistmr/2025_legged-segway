#include <PID_v1_bc.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <FlexCAN_T4.h>

// --- MPU6050 구성 ---
MPU6050 mpu;
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];
volatile bool mpuInterrupt = false;
void dmpDataReady() { mpuInterrupt = true; }

// --- PID 설정 ---
double setpoint = 180;     // 목표 각도 (평형)
double input, output;      // 센서 입력과 PID 출력
double Kp = 21, Ki = 140, Kd = 0.8;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// --- CAN 구성 ---
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
const uint32_t CAN_BAUD   = 1'000'000;
const uint32_t MOTOR_ID   = 0x141; // 모터 CAN ID

// --- 모터 제어 함수 (CAN 기반) ---
void motorOn() {
  CAN_message_t msg{};
  msg.id = MOTOR_ID; msg.len = 8;
  msg.buf[0] = 0x88;
  Can1.write(msg);
}

void sendSpeed(double spd_deg_per_s) {
  int32_t spd_raw = (int32_t)(spd_deg_per_s * 100); // 0.01 deg/s 단위
  CAN_message_t msg{};
  msg.id = MOTOR_ID; msg.len = 8;
  msg.buf[0] = 0xA2;
  msg.buf[1] = msg.buf[2] = msg.buf[3] = 0; // Reserved
  msg.buf[4] = spd_raw & 0xFF;
  msg.buf[5] = (spd_raw >> 8) & 0xFF;
  msg.buf[6] = (spd_raw >> 16) & 0xFF;
  msg.buf[7] = (spd_raw >> 24) & 0xFF;
  Can1.write(msg);
}

void stopMotor() {
  sendSpeed(0); // 정지
}

// --- 아두이노 초기 설정 ---
void setup() {
  Serial.begin(115200);

  // MPU6050 초기화
  mpu.initialize();
  Serial.println(mpu.testConnection() ? F("MPU6050 연결 성공") : F("MPU6050 연결 실패"));
  devStatus = mpu.dmpInitialize();

  // 옵셋 보정값 (환경에 따라 조정)
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1688);

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.print(F("DMP 초기화 실패 (코드 "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // PID 설정
  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(10);
  pid.SetOutputLimits(-300, 300); // 속도 제한 (deg/s)

  // CAN 초기화
  Can1.begin();
  Can1.setBaudRate(CAN_BAUD);
  delay(100);
  motorOn();
  delay(10);
}

void loop() {
  if (!dmpReady) return;

  // 센서 데이터가 준비되지 않으면 계속 대기
  while (!mpuInterrupt && fifoCount < packetSize) {
    pid.Compute(); // 이전 값 기반으로 제어 출력 계산
    Serial.print("Pitch: "); Serial.print(input);
    Serial.print(" -> Speed: "); Serial.println(output);

    if (input > 150 && input < 210) {
      sendSpeed(output); // 방향 포함 전송
    } else {
      stopMotor(); // 안정 구간 외는 모터 정지
    }
  }

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
  } else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    input = ypr[1] * 180 / M_PI + 180; // pitch (단위: 도)
  }
}
