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

enum ServoPose : uint8_t {
  SERVO_POSE_FOLDED = 0,
  SERVO_POSE_STAND  = 1,
};

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

const int H_STAND_DEG  = 60;   // original upright pose
const int H_FOLDED_DEG = 0;   // folded pose (tune as needed)

inline int J1ang(int a) { return a + 30; }    // left: a + offset
inline int J2ang(int a) { return 180 - a; }   // right: 180 - a

ServoPose currentServoPose = SERVO_POSE_FOLDED;
bool balanceEnabled = false;

void disableBalanceControlOutputs();

void setServoPose(ServoPose pose) {
  int baseAngle = (pose == SERVO_POSE_STAND) ? H_STAND_DEG : H_FOLDED_DEG;
  servoJ1.write(J1ang(baseAngle));
  servoJ2.write(J2ang(baseAngle));
  currentServoPose = pose;

  if (pose == SERVO_POSE_STAND) {
    balanceEnabled = true;
  } else {
    balanceEnabled = false;
    disableBalanceControlOutputs();
  }
}

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

unsigned long lastCmdMs = 0;  // 마지막 명령 시간 (블루투스/시리얼 통합)

// -------------------- 통합 명령 처리 (블루투스 + 시리얼) --------------------

void processCommand(char c, bool fromSerial) {
  lastCmdMs = millis();  // 명령 시간 기록

  if (c == 'U' || c == 'u') {
    setServoPose(SERVO_POSE_STAND);
    if (fromSerial) Serial.println(F("[U] Hip servos -> stand pose"));
  }
  else if (c == 'D' || c == 'd') {
    setServoPose(SERVO_POSE_FOLDED);
    if (fromSerial) Serial.println(F("[D] Hip servos -> folded pose"));
  }
  else if (c == 'F' || c == 'f') {
    motionCmd = MOTION_FWD;
    if (fromSerial) Serial.println(F("[F] Forward"));
  }
  else if (c == 'B' || c == 'b') {
    motionCmd = MOTION_BACK;
    if (fromSerial) Serial.println(F("[B] Backward"));
  }
  else if (c == 'L' || c == 'l') {
    motionCmd = MOTION_LEFT;
    if (fromSerial) Serial.println(F("[L] Turn left"));
  }
  else if (c == 'R' || c == 'r') {
    // R/r: 우회전
    motionCmd = MOTION_RIGHT;
    if (fromSerial) Serial.println(F("[R] Turn right"));
  }
  else if (c == 'S' || c == 's') {
    motionCmd = MOTION_STOP;
    if (fromSerial) Serial.println(F("[S] Stop"));
  }
  else if (c == '0') {
    // 0: IMU 캘리브레이션 (블루투스/시리얼 모두)
    recalibrateImuAndResetPid();
  }
}

void pollAllCommands() {
  // 블루투스 명령 처리
  while (btSerial.available()) {
    char c = btSerial.read();
    processCommand(c, false);
  }

  // 시리얼 명령 처리
  while (Serial.available()) {
    char c = Serial.read();
    processCommand(c, true);
  }
}

void safetyUpdateFromTimeout() {
  const unsigned long TIMEOUT_MS = 800; // 0.8초 동안 명령 없으면 정지

  if (lastCmdMs != 0 && millis() - lastCmdMs > TIMEOUT_MS) {
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

// -------------------- Balance Controller (단일 PID) --------------------

// PID on pitch angle (deg)
const float ZIG_ORIGIN_OFFSET_DEG = -1.06f;   // bias relative to DMP zero (tune)
float       zigOriginAngleDeg     = ZIG_ORIGIN_OFFSET_DEG;
const float PITCH_SIGN            = -1.0f;   // 센서 방향 반전하면 여기 바꾸기

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

void disableBalanceControlOutputs() {
  err_integral = 0.0f;
  last_error   = 0.0f;
  motorLeft.setTorqueIq(0);
  motorRight.setTorqueIq(0);
}

// -------------------- Motion Offsets (이동, 회전) --------------------
// 이동은 단순히 목표 pitch를 약간 앞/뒤로 기울이는 방식으로만 구현
void computeMotionOffsets(float &pitchOffsetDeg, int16_t &turnIqOffset) {
  const float   FWD_TILT_DEG  = -3.0f;   // 전진용 추가 기울기
  const float   BACK_TILT_DEG = 3.0f;  // 후진용 추가 기울기
  const int16_t TURN_IQ       = 60;     // 제자리 회전용 바퀴 토크 차이

  pitchOffsetDeg = 0.0f;
  turnIqOffset   = 0;

  switch (motionCmd) {
    case MOTION_STOP:
      pitchOffsetDeg = 0.0f;
      turnIqOffset   = 0;
      break;
    case MOTION_FWD:
      pitchOffsetDeg = FWD_TILT_DEG;
      turnIqOffset   = 0;
      break;
    case MOTION_BACK:
      pitchOffsetDeg = BACK_TILT_DEG;
      turnIqOffset   = 0;
      break;
    case MOTION_LEFT:
      pitchOffsetDeg = 0.0f;
      turnIqOffset   = +TURN_IQ;
      break;
    case MOTION_RIGHT:
      pitchOffsetDeg = 0.0f;
      turnIqOffset   = -TURN_IQ;
      break;
  }
}

void balanceControlStep(float dt) {
  if (!dmpReady || !balanceEnabled) {
    disableBalanceControlOutputs();
    return;
  }

  float pitch = imuPitchDeg * PITCH_SIGN;

  if (fabs(pitch) > MAX_TILT_FOR_CONTROL_DEG) {
    motorLeft.setTorqueIq(0);
    motorRight.setTorqueIq(0);
    err_integral = 0.0f;
    return;
  }

  // 이동/회전 명령에 따른 오프셋 계산
  float   cmdPitchOffsetDeg = 0.0f;
  int16_t cmdTurnIqOffset   = 0;
  computeMotionOffsets(cmdPitchOffsetDeg, cmdTurnIqOffset);

  // 최종 목표 pitch (zigOrigin 기반)
  float target = zigOriginAngleDeg + cmdPitchOffsetDeg;

  // 단일 PID
  float error = target - pitch;
  float derr  = (error - last_error) / dt;

  err_integral += error * dt;
  err_integral = constrain(err_integral, -I_LIMIT, I_LIMIT);

  float u = Kp * error + Ki * err_integral + Kd * derr;

  int16_t iq_cmd = (int16_t)roundf(u);

  // 회전용 토크 오프셋 적용
  int32_t iq_left_raw  = iq_cmd + cmdTurnIqOffset;
  int32_t iq_right_raw = iq_cmd - cmdTurnIqOffset;

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
    // auto calibration
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

  motorLeft.motorOn();
  motorRight.motorOn();
  motorLeft.motorStop();
  motorRight.motorStop();

  // Hip servos start folded; later can command stand via serial 'U'
  servoJ1.attach(SERVO_J1_PIN);
  servoJ2.attach(SERVO_J2_PIN);

  setServoPose(SERVO_POSE_FOLDED);

  err_integral = 0.0f;
  last_error   = 0.0f;
}

// -------------------- Loop --------------------

void loop() {
  pollAllCommands();
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
      balanceControlStep(dt);
    } else {
      motorLeft.setTorqueIq(0);
      motorRight.setTorqueIq(0);
    }
  }

  // Debug print (100ms 주기)
  static uint32_t lastPrintMs = 0;
  uint32_t nowMs = millis();
  if (nowMs - lastPrintMs >= 100) {
    lastPrintMs = nowMs;

    Serial.print(F("pitch="));
    Serial.print(imuPitchDeg, 2);
    Serial.print(F(" deg  target="));
    Serial.print(zigOriginAngleDeg, 2);
    Serial.print(F(" deg  cmd="));
    switch (motionCmd) {
      case MOTION_STOP:  Serial.print("S"); break;
      case MOTION_FWD:   Serial.print("F"); break;
      case MOTION_BACK:  Serial.print("B"); break;
      case MOTION_LEFT:  Serial.print("L"); break;
      case MOTION_RIGHT: Serial.print("R"); break;
    }
    Serial.print(F("  balance="));
    Serial.print(balanceEnabled ? "ON" : "OFF");
    Serial.println();
  }
}
