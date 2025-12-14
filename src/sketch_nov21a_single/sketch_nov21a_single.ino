// sketch_balance.ino
//
// Two-wheeled balancing robot using:
//  - LK CAN motors in torque mode (iq control)
//  - MPU6050 DMP (ElectronicCats library) for pitch angle feedback
//  - Two fixed hip servos: J1 on pin 4 (80°), J2 on pin 5 (120°)
//  - HC-06 Bluetooth module (Slave mode)
//    MAC Address: 98:DA:60:08:61:BB

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
volatile MotionCmd prevMotionCmd = MOTION_STOP;  // 이전 명령 추적

unsigned long lastCmdMs = 0;  // 마지막 명령 시간 (블루투스/시리얼 통합)

// PID 상태 변수 (전역 선언 - 명령 처리에서 사용)
float err_integral = 0.0f;
float last_error   = 0.0f;

// -------------------- Cascade Control Variables --------------------
// Velocity Loop (Outer Loop) - PI controller
float Speed_Kp = 0.03f;        // Velocity loop proportional gain (reduced for stability)
float Speed_Ki = 0.005f;       // Velocity loop integral gain (reduced for stability)
float targetSpeedDps = 0.0f;   // Target speed from motion commands
float currentSpeedDps = 0.0f;  // Current average speed from motors
float speed_integral = 0.0f;   // Velocity loop integral term

// Safety limit for pitch angle output from velocity loop (relaxed for better control)
const float MAX_PITCH_FROM_VELOCITY_DEG = 20.0f;

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
    prevMotionCmd = motionCmd;
    motionCmd = MOTION_FWD;
    if (fromSerial) Serial.println(F("[F] Forward"));
  }
  else if (c == 'B' || c == 'b') {
    prevMotionCmd = motionCmd;
    motionCmd = MOTION_BACK;
    if (fromSerial) Serial.println(F("[B] Backward"));
  }
  else if (c == 'L' || c == 'l') {
    prevMotionCmd = motionCmd;
    motionCmd = MOTION_LEFT;
    if (fromSerial) Serial.println(F("[L] Turn left"));
  }
  else if (c == 'R' || c == 'r') {
    // R/r: 우회전
    prevMotionCmd = motionCmd;
    motionCmd = MOTION_RIGHT;
    if (fromSerial) Serial.println(F("[R] Turn right"));
  }
  else if (c == 'S' || c == 's') {
    // 회전 명령에서 STOP으로 바뀔 때 PID 리셋
    if (prevMotionCmd == MOTION_LEFT || prevMotionCmd == MOTION_RIGHT) {
      err_integral = 0.0f;
      last_error = 0.0f;
    }
    // STOP 명령 시 속도 루프 적분기도 리셋 (활동 제동을 위해)
    speed_integral = 0.0f;
    prevMotionCmd = motionCmd;
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
  const unsigned long TIMEOUT_MS = 400; // 0.4초 동안 명령 없으면 정지

  if (lastCmdMs != 0 && millis() - lastCmdMs > TIMEOUT_MS) {
    // 회전 명령에서 타임아웃으로 STOP으로 바뀔 때 PID 리셋
    if (motionCmd == MOTION_LEFT || motionCmd == MOTION_RIGHT) {
      err_integral = 0.0f;
      last_error = 0.0f;
    }
    // 타임아웃 시 속도 루프 적분기도 리셋
    speed_integral = 0.0f;
    prevMotionCmd = motionCmd;
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

// -------------------- Cascade Balance Controller --------------------
// Architecture: Velocity Loop (Outer) -> Angle Loop (Inner) -> Torque Output
// NOTE: Motors operate in TORQUE MODE ONLY (setTorqueIq).
//       Velocity control is implemented entirely in software on Arduino side.

// PID on pitch angle (deg) - Inner Loop
const float ZIG_ORIGIN_OFFSET_DEG = -1.29f;   // bias relative to DMP zero (tune)
float       zigOriginAngleDeg     = ZIG_ORIGIN_OFFSET_DEG;
const float PITCH_SIGN            = -1.0f;   // 센서 방향 반전하면 여기 바꾸기

// Angle Loop PID Gains (tune on hardware)
float Kp = 40.0f;
float Ki = 0.5f;
float Kd = 4.0f;

// Safety: if tilt exceeds this, stop trying to balance
const float MAX_TILT_FOR_CONTROL_DEG = 45.0f;

// Integral clamp
const float I_LIMIT = 50.0f;

// Control timing (Hz)
const uint32_t CONTROL_PERIOD_US = 5000;  // 5 ms -> 200 Hz

void disableBalanceControlOutputs() {
  err_integral = 0.0f;
  last_error   = 0.0f;
  speed_integral = 0.0f;  // Reset velocity loop integral
  targetSpeedDps = 0.0f;
  currentSpeedDps = 0.0f;
  motorLeft.setTorqueIq(0);
  motorRight.setTorqueIq(0);
}

// -------------------- Motion Commands -> Target Speed --------------------
// Convert motion commands to target speed for Velocity Loop
// This sets the setpoint for the outer (velocity) control loop
void computeTargetSpeed(int16_t &turnIqOffset) {
  const float TARGET_SPEED_FWD_DPS  = 200.0f;   // Forward target speed (deg/s)
  const float TARGET_SPEED_BACK_DPS = -200.0f;  // Backward target speed (deg/s)
  const int16_t TURN_IQ = 30;                    // 제자리 회전용 바퀴 토크 차이

  turnIqOffset = 0;

  switch (motionCmd) {
    case MOTION_STOP:
      targetSpeedDps = 0.0f;  // Active braking: velocity loop will drive to zero
      turnIqOffset   = 0;
      break;
    case MOTION_FWD:
      targetSpeedDps = TARGET_SPEED_FWD_DPS;
      turnIqOffset   = 0;
      break;
    case MOTION_BACK:
      targetSpeedDps = TARGET_SPEED_BACK_DPS;
      turnIqOffset   = 0;
      break;
    case MOTION_LEFT:
      targetSpeedDps = TARGET_SPEED_FWD_DPS / 2;  // Reduced speed for turning
      turnIqOffset   = -TURN_IQ;  // Fixed: reversed sign for correct direction
      break;
    case MOTION_RIGHT:
      targetSpeedDps = TARGET_SPEED_FWD_DPS / 2;  // Reduced speed for turning
      turnIqOffset   = +TURN_IQ;  // Fixed: reversed sign for correct direction
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
    speed_integral = 0.0f;  // Reset velocity loop integral on fall
    return;
  }

  // ==================== STEP 1: Read Actual Motor Speeds ====================
  // Read speed from motors (in torque mode, speed is still reported)
  LkMotorState2 stateLeft, stateRight;
  bool leftOk = motorLeft.readState2(stateLeft);
  bool rightOk = motorRight.readState2(stateRight);

  if (!leftOk || !rightOk) {
    // If we can't read speeds, fall back to zero (safety)
    currentSpeedDps = 0.0f;
  } else {
    // Calculate average speed with direction correction
    // Apply LEFT_DIR and RIGHT_DIR to align motor directions
    float speedL = (float)stateLeft.speed_dps * LEFT_DIR;
    float speedR = (float)stateRight.speed_dps * RIGHT_DIR;
    currentSpeedDps = (speedL + speedR) / 2.0f;
  }

  // ==================== STEP 2: Velocity Loop (Outer Loop) ====================
  // Software-based velocity control (NOT using motor's internal velocity mode)
  // Input:  (targetSpeedDps - currentSpeedDps)
  // Output: targetPitchAngle (pitch offset from zigOrigin)

  // Compute target speed from motion commands
  int16_t cmdTurnIqOffset = 0;
  computeTargetSpeed(cmdTurnIqOffset);

  // Velocity PI controller: Error = TargetSpeed - CurrentSpeed
  float speed_error = targetSpeedDps - currentSpeedDps;

  // When stopped, disable velocity loop to maintain stable standing
  // Only use velocity loop when there's an actual motion command
  float pitchOffsetFromVelocity = 0.0f;

  if (motionCmd != MOTION_STOP) {
    speed_integral += speed_error * dt;

    // Integral clamping (prevent windup)
    const float SPEED_I_LIMIT = 50.0f;
    speed_integral = constrain(speed_integral, -SPEED_I_LIMIT, SPEED_I_LIMIT);

    // PI output = target pitch angle offset (relative to zigOrigin)
    pitchOffsetFromVelocity = Speed_Kp * speed_error + Speed_Ki * speed_integral;

    // Constrain pitch offset to safe range (relative to zigOrigin)
    pitchOffsetFromVelocity = constrain(pitchOffsetFromVelocity,
                                        -MAX_PITCH_FROM_VELOCITY_DEG,
                                        MAX_PITCH_FROM_VELOCITY_DEG);
  } else {
    // When stopped, reset velocity loop integral for clean standing
    speed_integral = 0.0f;
  }

  // ==================== STEP 3: Angle Loop (Inner Loop) ====================
  // Input:  imuPitchDeg (current pitch angle)
  // Setpoint: zigOrigin + pitchOffsetFromVelocity (from velocity loop)
  // Output: iq_cmd (torque current command)

  // Final target pitch = zigOrigin + velocity loop output
  float targetPitch = zigOriginAngleDeg + pitchOffsetFromVelocity;

  // Angle PID controller
  float error = targetPitch - pitch;
  float derr  = (error - last_error) / dt;

  err_integral += error * dt;
  err_integral = constrain(err_integral, -I_LIMIT, I_LIMIT);

  float u = Kp * error + Ki * err_integral + Kd * derr;

  int16_t iq_cmd = (int16_t)roundf(u);

  // ==================== STEP 4: Actuation (Torque Mode) ====================
  // Apply turning offsets (differential torque for rotation)
  int32_t iq_left_raw  = iq_cmd + cmdTurnIqOffset;
  int32_t iq_right_raw = iq_cmd - cmdTurnIqOffset;

  // Clamp to torque limits
  if (iq_left_raw > IQ_MAX) iq_left_raw = IQ_MAX;
  if (iq_left_raw < IQ_MIN) iq_left_raw = IQ_MIN;

  if (iq_right_raw > IQ_MAX) iq_right_raw = IQ_MAX;
  if (iq_right_raw < IQ_MIN) iq_right_raw = IQ_MIN;

  // Apply motor direction correction
  int16_t iq_left  = LEFT_DIR  * (int16_t)iq_left_raw;
  int16_t iq_right = RIGHT_DIR * (int16_t)iq_right_raw;

  // Send torque commands to motors (TORQUE MODE ONLY - no setSpeed() used)
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
  speed_integral = 0.0f;  // Velocity loop integral also reset
  targetSpeedDps = 0.0f;
  currentSpeedDps = 0.0f;

  Serial.println(F("[R] IMU calibration & PID reset done"));
}

// -------------------- HC-06 AT 명령 전송 및 응답 확인 --------------------
bool sendATCommandBT(const char* cmd, const char* expectedResponse, int timeout = 1000) {
  // 버퍼 비우기
  while (btSerial.available()) {
    btSerial.read();
  }

  btSerial.print(cmd);
  btSerial.print("\r\n");

  delay(100);

  unsigned long startTime = millis();
  String response = "";

  while (millis() - startTime < timeout) {
    if (btSerial.available()) {
      char c = btSerial.read();
      response += c;
      if (response.indexOf(expectedResponse) >= 0) {
        Serial.print(F("  -> OK: "));
        Serial.println(cmd);
        return true;
      }
    }
  }

  Serial.print(F("  -> Timeout/No response: "));
  Serial.println(cmd);
  if (response.length() > 0) {
    Serial.print(F("    Response: "));
    Serial.println(response);
  }
  return false;
}

// -------------------- HC-06 슬레이브 모드 설정 --------------------
void setupHC06AsSlave() {
  Serial.println(F("Configuring HC-06 as Slave..."));
  delay(2000);  // HC-06 초기화 대기

  // AT 명령 테스트
  Serial.println(F("Testing AT command..."));
  if (!sendATCommandBT("AT", "OK", 1000)) {
    Serial.println(F("WARNING: HC-06 not responding to AT commands!"));
    Serial.println(F("Make sure HC-06 is powered and connected correctly."));
  }

  // HC-06을 슬레이브 모드로 설정
  Serial.println(F("Setting to Slave mode..."));
  sendATCommandBT("AT+ROLE=0", "OK", 1000);
  delay(500);

  // HC-06 이름 설정
  Serial.println(F("Setting name to HC06..."));
  sendATCommandBT("AT+NAME=jalseobot", "OK", 1000);
  delay(500);

  // MAC 주소 확인
  Serial.println(F("Getting MAC address..."));
  sendATCommandBT("AT+ADDR?", "OK", 1000);
  delay(500);

  Serial.println(F("HC-06 configured as Slave"));
  Serial.println(F("MAC Address: 98:DA:60:08:61:BB"));
  Serial.println(F("Waiting for controller connection..."));
}


// -------------------- Setup --------------------

void setup() {
  Serial.begin(115200);
  btSerial.begin(9600);
  while (!Serial && millis() < 3000) {
    // wait for USB serial
  }

  // HC-06 슬레이브 모드 설정
  setupHC06AsSlave();

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
  speed_integral = 0.0f;
  targetSpeedDps = 0.0f;
  currentSpeedDps = 0.0f;
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
    Serial.print(F(" deg  speed="));
    Serial.print(currentSpeedDps, 1);
    Serial.print(F(" dps  targetSpeed="));
    Serial.print(targetSpeedDps, 1);
    Serial.print(F(" dps  cmd="));
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
