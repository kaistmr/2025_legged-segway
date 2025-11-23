// sketch_balance.ino
//
// Two-wheeled balancing robot using:
//  - LK CAN motors in torque mode (iq control)
//  - MPU6050 DMP (ElectronicCats library) for pitch angle feedback
//  - Two fixed hip servos: J1 on pin 4 (80°), J2 on pin 5 (120°)

#include <Wire.h>
#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"   // from ElectronicCats/mpu6050
#include "LkMotor.h"                      // LK CAN driver

// -------------------- CAN / Motors --------------------

LkMotorBus can1;  // FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>

const uint8_t MOTOR_LEFT_ID  = 5;   // W1
const uint8_t MOTOR_RIGHT_ID = 4;   // W2

LkCanMotor motorLeft(can1, MOTOR_LEFT_ID);
LkCanMotor motorRight(can1, MOTOR_RIGHT_ID);

// Motor direction (to account for wiring / orientation)
const int LEFT_DIR  = -1;
const int RIGHT_DIR = +1;

// Torque limits (iq units: -2048..2048)
const int16_t IQ_MAX = 500;
const int16_t IQ_MIN = -500;

// -------------------- Hip Servos (J1/J2) --------------------

Servo servoJ1;  // left hip
Servo servoJ2;  // right hip

const int SERVO_J1_PIN = 4;  // J1
const int SERVO_J2_PIN = 5;  // J2

const int H_INIT = 60;       // base hip angle

inline int J1ang(int a) { return a + 20; }    // left: a + 20
inline int J2ang(int a) { return 180 - a; }   // right: 180 - a

// -------------------- HC-06 Bluetooth module --------------------
static const uint8_t BT_RX = 7;  // (HC-06 TXD)
static const uint8_t BT_TX = 8;  // (HC-06 RXD)

HardwareSerial& btSerial = Serial2;

// *** NEW ***: 이동 명령 상태 정의
enum MotionCmd {
  MOTION_STOP,
  MOTION_FWD,
  MOTION_BACK,
  MOTION_LEFT,
  MOTION_RIGHT
};

volatile MotionCmd motionCmd = MOTION_STOP;  // 기본은 정지(S)

unsigned long lastBtCmdMs = 0;

// *** NEW ***: 블루투스에서 명령 문자 읽어서 상태 갱신
void pollBtCommand() {
  while (btSerial.available()) {
    char c = btSerial.read();
    lastBtCmdMs = millis();

    switch (c) {
      case 'S':
        motionCmd = MOTION_STOP;
        break;
      case 'F':
        motionCmd = MOTION_FWD;
        break;
      case 'B':
        motionCmd = MOTION_BACK;
        break;
      case 'L':
        motionCmd = MOTION_LEFT;
        break;
      case 'R':
        motionCmd = MOTION_RIGHT;
        break;
      default:
        // 그 외 문자는 무시
        break;
    }
  }
}

// -------------------- IMU / DMP --------------------

MPU6050 mpu;

bool    dmpReady = false;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];                 // [yaw, pitch, roll] in radians

float imuYawDeg   = 0.0f;
float imuPitchDeg = 0.0f;
float imuRollDeg  = 0.0f;

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

void updateImuDmp() {
  if (!dmpReady) return;

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    imuYawDeg   = ypr[0] * 180.0f / M_PI;
    imuPitchDeg = ypr[1] * 180.0f / M_PI;
    imuRollDeg  = ypr[2] * 180.0f / M_PI;
  }
}

// -------------------- Balance Controller --------------------

// PID on pitch angle (deg)
const float PITCH_TARGET_DEG = -0.0f;   // upright
const float PITCH_SIGN       = -1.0f;  // flip to -1.0f if sense is reversed

// Gains (tune on hardware)
float Kp = 40.0f;
float Ki = 0.5f;
float Kd = 4.0f;

// Safety: if tilt exceeds this, stop trying to balance
const float MAX_TILT_FOR_CONTROL_DEG = 45.0f;

// Integral clamp
const float I_LIMIT = 50.0f;

// Control timing (Hz)
const uint32_t CONTROL_PERIOD_US = 5000;  // 5 ms -> 200 Hz

float err_integral = 0.0f;
float last_error   = 0.0f;

// *** NEW ***: 이동 명령에 따른 pitch / 회전 토크 보정 계산
//  - pitchOffsetDeg: 목표 pitch에 더해지는 각도 (전진/후진)
//  - turnIqOffset  : 좌/우 바퀴 차이를 위해 추가하는 토크
void computeMotionOffsets(float &pitchOffsetDeg, int16_t &turnIqOffset) {
  // 전진/후진용 기울기 (deg) - 필요하면 숫자 더 작게/크게 조정
  const float FWD_TILT_DEG  = 3.0f;
  const float BACK_TILT_DEG = -3.0f;

  // 회전용 토크 오프셋 (iq 단위)
  const int16_t TURN_IQ = 80;  // 너무 세면 흔들리니 필요시 조정

  pitchOffsetDeg = 0.0f;
  turnIqOffset   = 0;

  switch (motionCmd) {
    case MOTION_STOP:
      // 제자리: 추가 기울기/회전 없음
      pitchOffsetDeg = 0.0f;
      turnIqOffset   = 0;
      break;
    case MOTION_FWD:
      // 앞으로 가기 → 약간 앞으로 기울이기
      pitchOffsetDeg = FWD_TILT_DEG;
      turnIqOffset   = 0;
      break;
    case MOTION_BACK:
      // 뒤로 가기 → 약간 뒤로 기울이기
      pitchOffsetDeg = BACK_TILT_DEG;
      turnIqOffset   = 0;
      break;
    case MOTION_LEFT:
      // 제자리 좌회전: 좌/우 바퀴 토크에 차이를 줌
      pitchOffsetDeg = 0.0f;   // 서 있는 상태에서 회전
      turnIqOffset   = +TURN_IQ;
      break;
    case MOTION_RIGHT:
      // 제자리 우회전
      pitchOffsetDeg = 0.0f;
      turnIqOffset   = -TURN_IQ;
      break;
  }
}

void safetyUpdateFromTimeout() {
  const unsigned long TIMEOUT_MS = 800; // 0.5초 동안 명령 없으면 정지
  if (millis() - lastBtCmdMs > TIMEOUT_MS) {
    motionCmd = MOTION_STOP;
  }
}

void balanceControlStep(float dt) {
  if (!dmpReady) return;

  float pitch = imuPitchDeg * PITCH_SIGN;

  if (fabs(pitch) > MAX_TILT_FOR_CONTROL_DEG) {
    motorLeft.setTorqueIq(0);
    motorRight.setTorqueIq(0);
    return;
  }

  // *** NEW ***: 블루투스 명령에 따른 기울기/회전 보정값 계산
  float   cmdPitchOffsetDeg = 0.0f;
  int16_t cmdTurnIqOffset   = 0;
  computeMotionOffsets(cmdPitchOffsetDeg, cmdTurnIqOffset);

  float target = PITCH_TARGET_DEG + cmdPitchOffsetDeg;
  float error  = target - pitch;
  float derr   = (error - last_error) / dt;

  err_integral += error * dt;
  err_integral = constrain(err_integral, -I_LIMIT, I_LIMIT);

  float u = Kp * error + Ki * err_integral + Kd * derr;

  int16_t iq_cmd = (int16_t)roundf(u);

  // 기본 PID 출력에 회전용 토크 오프셋을 더해서 좌/우 바퀴 결정
  int32_t iq_left_raw  = iq_cmd + cmdTurnIqOffset;
  int32_t iq_right_raw = iq_cmd - cmdTurnIqOffset;

  // 개별 바퀴별로 saturate
  if (iq_left_raw > IQ_MAX) iq_left_raw = IQ_MAX;
  if (iq_left_raw < IQ_MIN) iq_left_raw = IQ_MIN;

  if (iq_right_raw > IQ_MAX) iq_right_raw = IQ_MAX;
  if (iq_right_raw < IQ_MIN) iq_right_raw = IQ_MIN;

  int16_t iq_left  = LEFT_DIR  * (int16_t)iq_left_raw;
  int16_t iq_right = RIGHT_DIR * (int16_t)iq_right_raw;

  motorLeft.setTorqueIq(iq_left);
  motorRight.setTorqueIq(iq_right);

  last_error = error;
}

// -------------------- Setup --------------------

void setup() {
  Serial.begin(115200);
  btSerial.begin(9600);
  while (!Serial && millis() < 3000) {
    // wait for USB serial
  }

  // I2C for MPU6050
  Wire.begin();
  Wire.setClock(400000);

  Serial.println(F("Initializing MPU6050..."));
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println(F("MPU6050 connection FAILED"));
  } else {
    Serial.println(F("MPU6050 connection successful"));
  }

  Serial.println(F("Initializing DMP..."));
  uint8_t devStatus = mpu.dmpInitialize();

  // Offsets (example values)
  mpu.setXGyroOffset(40);
  mpu.setYGyroOffset(144);
  mpu.setZGyroOffset(-3);
  mpu.setZAccelOffset(2270);

  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();

    mpu.setDMPEnabled(true);
    dmpReady = true;
    Serial.println(F("DMP ready, balancing controller enabled"));
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // CAN1 for LK hubs
  can1.begin();
  can1.setBaudRate(1000000);

  Serial.println(F("Enabling LK motors (torque mode)..."));
  if (motorLeft.motorOn() && motorRight.motorOn()) {
    Serial.println(F("Motors ON"));
  } else {
    Serial.println(F("Failed to turn motors ON"));
  }

  // ----- Hip servos fixed pose -----
  servoJ1.attach(SERVO_J1_PIN);   // J1 on pin 4
  servoJ2.attach(SERVO_J2_PIN);   // J2 on pin 5

  servoJ1.write(J1ang(H_INIT));   // left: 60 + 20 = 80°
  servoJ2.write(J2ang(H_INIT));   // right: 180 - 60 = 120°
  // No further writes in loop() → they hold 80° / 120° throughout runtime.

  // Reset PID state
  err_integral = 0.0f;
  last_error   = 0.0f;
}

// -------------------- Loop --------------------

void loop() {
  // *** NEW ***: 블루투스 명령 상시 폴링
  pollBtCommand();
  safetyUpdateFromTimeout();
  updateImuDmp();

  static uint32_t lastCtrlMicros = micros();
  uint32_t now    = micros();
  uint32_t dt_us  = now - lastCtrlMicros;

  if (dt_us >= CONTROL_PERIOD_US) {
    float dt = dt_us / 1e6f;
    if (dt <= 0.0f || dt > 0.1f) {
      dt = CONTROL_PERIOD_US / 1e6f;
    }

    lastCtrlMicros = now;

    if (dmpReady) {
      //balanceControlStep(dt);
    } else {
      motorLeft.setTorqueIq(0);
      motorRight.setTorqueIq(0);
    }
  }

  // Debug print at slower rate
  static uint32_t lastPrintMs = 0;
  uint32_t nowMs = millis();
  if (nowMs - lastPrintMs >= 100) {
    lastPrintMs = nowMs;

    /*
    Serial.print(F("Pitch="));
    Serial.print(imuPitchDeg, 2);
    Serial.print(F(" deg   I="));
    Serial.print(err_integral, 2);
    Serial.print(F("   Kp=")); Serial.print(Kp, 1);
    Serial.print(F(" Ki="));   Serial.print(Ki, 3);
    Serial.print(F(" Kd="));   Serial.print(Kd, 1);
    Serial.print(F("   cmd="));
    */
    /*
    switch (motionCmd) {
      case MOTION_STOP:  Serial.print("S"); break;
      case MOTION_FWD:   Serial.print("F"); break;
      case MOTION_BACK:  Serial.print("B"); break;
      case MOTION_LEFT:  Serial.print("L"); break;
      case MOTION_RIGHT: Serial.print("R"); break;
    }
    Serial.println();
    */

    double left_angle_deg = 0;
    double right_angle_deg = 0;
    motorLeft.readMultiTurnAngleDeg(left_angle_deg);
    motorRight.readMultiTurnAngleDeg(right_angle_deg);
    Serial.print(F("Left Angle="));
    Serial.print(left_angle_deg, 2);
    Serial.print(F("  Right Angle="));
    Serial.println(right_angle_deg, 2);

  }
}