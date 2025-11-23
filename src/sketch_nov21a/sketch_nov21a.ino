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
float Ki = 0.4f;
float Kd = 4.0f;

// Safety: if tilt exceeds this, stop trying to balance
const float MAX_TILT_FOR_CONTROL_DEG = 45.0f;

// Integral clamp
const float I_LIMIT = 50.0f;

// Control timing (Hz)
const uint32_t CONTROL_PERIOD_US = 5000;  // 5 ms -> 200 Hz

float err_integral = 0.0f;
float last_error   = 0.0f;

void balanceControlStep(float dt) {
  if (!dmpReady) return;

  float pitch = imuPitchDeg * PITCH_SIGN;

  if (fabs(pitch) > MAX_TILT_FOR_CONTROL_DEG) {
    motorLeft.setTorqueIq(0);
    motorRight.setTorqueIq(0);
    return;
  }

  float target = PITCH_TARGET_DEG;
  float error  = target - pitch;
  float derr   = (error - last_error) / dt;

  err_integral += error * dt;
  err_integral = constrain(err_integral, -I_LIMIT, I_LIMIT);

  float u = Kp * error + Ki * err_integral + Kd * derr;

  int16_t iq_cmd = (int16_t)roundf(u);
  if (iq_cmd > IQ_MAX) iq_cmd = IQ_MAX;
  if (iq_cmd < IQ_MIN) iq_cmd = IQ_MIN;

  int16_t iq_left  = LEFT_DIR  * iq_cmd;
  int16_t iq_right = RIGHT_DIR * iq_cmd;

  motorLeft.setTorqueIq(iq_left);
  motorRight.setTorqueIq(iq_right);

  last_error = error;
}

// -------------------- Setup --------------------

void setup() {
  Serial.begin(115200);
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

  // Debug print at slower rate
  static uint32_t lastPrintMs = 0;
  uint32_t nowMs = millis();
  if (nowMs - lastPrintMs >= 100) {
    lastPrintMs = nowMs;

    Serial.print(F("Pitch="));
    Serial.print(imuPitchDeg, 2);
    Serial.print(F(" deg   I="));
    Serial.print(err_integral, 2);
    Serial.print(F("   Kp=")); Serial.print(Kp, 1);
    Serial.print(F(" Ki="));   Serial.print(Ki, 3);
    Serial.print(F(" Kd="));   Serial.print(Kd, 1);
    Serial.println();
  }
}