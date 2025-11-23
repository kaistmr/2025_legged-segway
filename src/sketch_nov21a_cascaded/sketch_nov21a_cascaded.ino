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

// Encoder 방향 보정 (필요하면 ±1 바꿔서 맞추기)
const int LEFT_ENC_SIGN  = -1;
const int RIGHT_ENC_SIGN = +1;

// Torque limits (iq units: -2048..2048)
const int16_t IQ_MAX = 500;
const int16_t IQ_MIN = -500;

// -------------------- Hip Servos (J1/J2) --------------------

Servo servoJ1;  // left hip
Servo servoJ2;  // right hip

const int SERVO_J1_PIN = 4;  // J1
const int SERVO_J2_PIN = 5;  // J2

const int H_INIT = 60;       // base hip angle

inline int J1ang(int a) { return a + 30; }    // left: a + 30
inline int J2ang(int a) { return 180 - a; }   // right: 180 - a

// -------------------- HC-06 Bluetooth module --------------------
static const uint8_t BT_RX = 7;  // (HC-06 TXD)
static const uint8_t BT_TX = 8;  // (HC-06 RXD)

HardwareSerial& btSerial = Serial2;

// ----- 이동 명령 상태 -----
enum MotionCmd {
  MOTION_STOP,
  MOTION_FWD,
  MOTION_BACK,
  MOTION_LEFT,
  MOTION_RIGHT
};

volatile MotionCmd motionCmd = MOTION_STOP;  // 기본은 정지(S)

unsigned long lastBtCmdMs = 0;

bool serialForwardActive = false;
unsigned long serialForwardStartMs = 0;

void pollBtCommand() {
  while (btSerial.available()) {
    char c = btSerial.read();
    lastBtCmdMs = millis();

    switch (c) {
      case 'S': motionCmd = MOTION_STOP;  break;
      case 'F': motionCmd = MOTION_FWD;   break;
      case 'B': motionCmd = MOTION_BACK;  break;
      case 'L': motionCmd = MOTION_LEFT;  break;
      case 'R': motionCmd = MOTION_RIGHT; break;
      default:  break; // 기타 문자는 무시
    }
  }
}



void safetyUpdateFromTimeout() {
  const unsigned long TIMEOUT_MS = 800; // 0.8초 동안 명령 없으면 정지
  if (millis() - lastBtCmdMs > TIMEOUT_MS) {
    motionCmd = MOTION_STOP;
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

// -------------------- Balance Controller (Inner Loop) --------------------

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

// -------------------- Outer Position Loop (Cascaded) --------------------
// 바퀴 반지름: 0.143 m
const float WHEEL_RADIUS_M = 0.143f;

// 엔코더 기반 로봇 위치 [m] (센터 기준, 앞으로 +)
float robotPos_m = 0.0f;

// 위치 목표 [m]
float posRef_m = 0.0f;

// 외부 위치 PID gain 
float Kp_pos = 0.0f;
float Ki_pos = 0.0f;
float Kd_pos = 0.0f;

// 위치 PID 적분 제한
const float POS_I_LIMIT = 1.0f;

// 위치 PID 출력(목표 pitch) 제한 [deg]
const float POS_PITCH_LIMIT_DEG = 10.0f;

// 외부 루프 상태
float posErrInt  = 0.0f;
float posErrLast = 0.0f;

// outer loop가 만든 delta pitch (deg)
float lastOuterPitchDeg = 0.0f;

// 앱에서 오는 F/B 명령을 속도로 해석 (m/s)
const float CMD_VEL_MPS = 0.5f;

// --- 엔코더 관련 (누적각 0도 오프셋 처리) ---
bool   encZeroed           = false;
double leftAngleOffsetDeg  = 0.0;
double rightAngleOffsetDeg = 0.0;

// 디버깅용 상대각 저장 (원하면 프린트 가능)
float leftAngleRelDeg  = 0.0f;
float rightAngleRelDeg = 0.0f;

// 엔코더 → 각도(상대) → 선형 위치 → robotPos_m 업데이트
void updateRobotPositionFromEncoders() {
  double left_angle_deg_raw  = 0.0;
  double right_angle_deg_raw = 0.0;

  // LK 라이브러리에서 누적각도(deg) 읽기
  motorLeft.readMultiTurnAngleDeg(left_angle_deg_raw);
  motorRight.readMultiTurnAngleDeg(right_angle_deg_raw);

  // 첫 실행 시 오프셋 저장해서 0도로 맞추기
  if (!encZeroed) {
    leftAngleOffsetDeg  = left_angle_deg_raw;
    rightAngleOffsetDeg = right_angle_deg_raw;
    encZeroed           = true;
  }

  double left_rel_deg  = left_angle_deg_raw  - leftAngleOffsetDeg;
  double right_rel_deg = right_angle_deg_raw - rightAngleOffsetDeg;

  leftAngleRelDeg  = (float)left_rel_deg;
  rightAngleRelDeg = (float)right_rel_deg;

  // 방향 보정 적용 후 rad로 변환
  float left_rad  = (float)(LEFT_ENC_SIGN  * left_rel_deg)  * M_PI / 180.0f;
  float right_rad = (float)(RIGHT_ENC_SIGN * right_rel_deg) * M_PI / 180.0f;

  // 각 바퀴 선형 위치 [m]
  float left_pos_m  = left_rad  * WHEEL_RADIUS_M;
  float right_pos_m = right_rad * WHEEL_RADIUS_M;

  // 두 바퀴 평균 → 로봇 중심 위치
  robotPos_m = 0.5f * (left_pos_m + right_pos_m);
}

// F/B 입력에 따라 위치 레퍼런스 업데이트
void updatePositionReference(float dt) {
  float v_cmd = 0.0f;
  switch (motionCmd) {
    case MOTION_FWD:  v_cmd = +CMD_VEL_MPS; break;
    case MOTION_BACK: v_cmd = -CMD_VEL_MPS; break;
    default:          v_cmd = 0.0f;         break; // STOP/L/R에서는 위치 목표 고정
  }
  posRef_m += v_cmd * dt;
}

// 외부 위치 PID → 목표 pitch [deg] 반환
float positionOuterLoopStep(float dt) {
  float error = posRef_m - robotPos_m;
  float derr  = (error - posErrLast) / dt;

  posErrInt += error * dt;
  posErrInt = constrain(posErrInt, -POS_I_LIMIT, POS_I_LIMIT);

  float pitchCmdDeg = Kp_pos * error + Ki_pos * posErrInt + Kd_pos * derr;

  if (pitchCmdDeg >  POS_PITCH_LIMIT_DEG) pitchCmdDeg =  POS_PITCH_LIMIT_DEG;
  if (pitchCmdDeg < -POS_PITCH_LIMIT_DEG) pitchCmdDeg = -POS_PITCH_LIMIT_DEG;

  posErrLast = error;

  lastOuterPitchDeg = pitchCmdDeg;

  return pitchCmdDeg;
}

// -------------------- Motion Offsets (turn, etc.) --------------------
//  - pitchOffsetDeg: (현재는 회전 모드에서만 0, 필요하면 추가 오프셋 사용)
//  - turnIqOffset  : 좌/우 바퀴 차이를 위해 추가하는 토크
void computeMotionOffsets(float &pitchOffsetDeg, int16_t &turnIqOffset) {
  // 회전용 토크 오프셋 (iq 단위)
  const int16_t TURN_IQ = 80;  // 너무 세면 줄이고, 약하면 늘리면 됨

  pitchOffsetDeg = 0.0f;
  turnIqOffset   = 0;

  switch (motionCmd) {
    case MOTION_LEFT:
      // 제자리 좌회전: 좌/우 바퀴 토크에 차이를 줌
      pitchOffsetDeg = 0.0f;
      turnIqOffset   = +TURN_IQ;
      break;
    case MOTION_RIGHT:
      // 제자리 우회전
      pitchOffsetDeg = 0.0f;
      turnIqOffset   = -TURN_IQ;
      break;
    default:
      // STOP/FWD/BACK 등은 여기서는 추가 pitchOffset 없음
      pitchOffsetDeg = 0.0f;
      turnIqOffset   = 0;
      break;
  }
}

void balanceControlStep(float dt) {
  if (!dmpReady) return;

  float pitch = imuPitchDeg * PITCH_SIGN;

  if (fabs(pitch) > MAX_TILT_FOR_CONTROL_DEG) {
    motorLeft.setTorqueIq(0);
    motorRight.setTorqueIq(0);
    // 크게 넘어졌을 때는 적분도 리셋
    err_integral = 0.0f;
    posErrInt    = 0.0f;
    return;
  }

  // 1) 회전/기타 보정 (L/R 등)
  float   cmdPitchOffsetDeg = 0.0f;
  int16_t cmdTurnIqOffset   = 0;
  computeMotionOffsets(cmdPitchOffsetDeg, cmdTurnIqOffset);

  // 2) 외부 위치 루프: 위치오차 → 목표 pitch 생성
  updatePositionReference(dt);                // posRef_m 업데이트 (F/B면 계속 쌓임)
  float pitchFromPosDeg = -positionOuterLoopStep(dt);

  // 최종 목표 pitch = 기본 0도 + 위치루프 + (필요하면 추가 오프셋)
  float target = PITCH_TARGET_DEG + pitchFromPosDeg + cmdPitchOffsetDeg;

  // 3) 내부 pitch PID (현재 pitch → 모터 torque)
  float error = target - pitch;
  float derr  = (error - last_error) / dt;

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

void recalibrateImuAndResetPid() {
  Serial.println(F("[R] Recalibrating IMU (Accel+Gyro)..."));

  dmpReady = false;
  mpu.setDMPEnabled(false);
  mpu.resetFIFO();

  // 다시 캘리 (setup에서 했던 것과 동일하게)
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  mpu.PrintActiveOffsets();

  mpu.setDMPEnabled(true);
  dmpReady = true;

  // PID 상태도 리셋
  err_integral = 0.0f;
  last_error   = 0.0f;

  Serial.println(F("[R] IMU calibration & PID reset done"));
}

void pollSerialCommand() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == 'H' || c == 'h') {
      // 현재 엔코더 누적각 읽기
      double left_angle_deg_raw  = 0.0;
      double right_angle_deg_raw = 0.0;
      motorLeft.readMultiTurnAngleDeg(left_angle_deg_raw);
      motorRight.readMultiTurnAngleDeg(right_angle_deg_raw);

      // 이 값을 기준(0도)으로 삼도록 오프셋 재설정
      leftAngleOffsetDeg  = left_angle_deg_raw;
      rightAngleOffsetDeg = right_angle_deg_raw;
      encZeroed           = true;

      // 위치/레퍼런스 및 PID 상태 초기화
      robotPos_m = 0.0f;
      posRef_m   = 0.0f;

      posErrInt  = 0.0f;
      posErrLast = 0.0f;

      err_integral = 0.0f;
      last_error   = 0.0f;

      Serial.println(F("[H] Home: position & encoder offsets reset"));
    }
    // === NEW: 시리얼에서 f/F 들어오면 2초 전진 시작 ===
    else if (c == 'f' || c == 'F') {
      serialForwardActive  = true;
      serialForwardStartMs = millis();
      motionCmd            = MOTION_FWD;   // 2초 동안 forward 명령
      Serial.println(F("[f] 2s forward command started"));
    }
    else if (c == 'r' || c == 'R') {
      recalibrateImuAndResetPid();
    }
  }
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
    /* 
    // auto calibration
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    */

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

  motorLeft.motorOn();
  motorRight.motorOn();
  motorLeft.motorStop();
  motorRight.motorStop();

  // ----- Hip servos fixed pose -----
  servoJ1.attach(SERVO_J1_PIN);   // J1 on pin 4
  servoJ2.attach(SERVO_J2_PIN);   // J2 on pin 5

  servoJ1.write(J1ang(H_INIT));   // left: 60 + 20 = 80°
  servoJ2.write(J2ang(H_INIT));   // right: 180 - 60 = 120°
  // No further writes in loop() → they hold 80° / 120° throughout runtime.

  // Reset PID state
  err_integral = 0.0f;
  last_error   = 0.0f;

  // 위치 루프 상태도 초기화
  posRef_m   = 0.0f;
  posErrInt  = 0.0f;
  posErrLast = 0.0f;

  robotPos_m = 0.0f;

  encZeroed           = false;
  leftAngleOffsetDeg  = 0.0;
  rightAngleOffsetDeg = 0.0;
}

// -------------------- Loop --------------------

void loop() {
  pollBtCommand();
  pollSerialCommand();
  safetyUpdateFromTimeout();

  // === NEW: Serial 'f' 2초 타이머 처리 ===
  if (serialForwardActive) {
    unsigned long elapsed = millis() - serialForwardStartMs;
    if (elapsed >= 2000) {            // 2초 경과
      serialForwardActive = false;
      motionCmd = MOTION_STOP;        // 다시 정지 모드
      Serial.println(F("[f] 2s forward finished"));
    }
  }

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

    // 컨트롤 주기마다 엔코더 기반 위치 업데이트
    updateRobotPositionFromEncoders();

    if (dmpReady) {
      balanceControlStep(dt);
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

    /*Serial.print(F("cmd="));
    switch (motionCmd) {
      case MOTION_STOP:  Serial.print("S"); break;
      case MOTION_FWD:   Serial.print("F"); break;
      case MOTION_BACK:  Serial.print("B"); break;
      case MOTION_LEFT:  Serial.print("L"); break;
      case MOTION_RIGHT: Serial.print("R"); break;
    }*/
    Serial.print(F("pos="));
    Serial.print(robotPos_m, 3);
    Serial.print(F("  pitch="));
    Serial.print(imuPitchDeg, 2);
    Serial.print(F("  dPitch_outer="));
    Serial.print(lastOuterPitchDeg, 2);
    Serial.println();
  }
}