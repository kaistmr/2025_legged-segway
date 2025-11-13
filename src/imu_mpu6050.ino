//#include <Arduino.h>
#include <Wire.h>

// helper_3dmath의 Quaternion/Vector* 타입이 먼저 선언되어야
// 일부 포크의 MPU6050.h가 컴파일됩니다. 반드시 먼저 포함하세요.
#include "helper_3dmath.h"

// --- DMP 선언을 활성화하기 위해 먼저 매크로 정의 ---
#define MPU6050_INCLUDE_DMP_MOTIONAPPS20

// --- 선언 헤더(매크로가 보이는 상태로 포함) ---
#include <MPU6050.h>

// --- 구현 헤더: DMP 함수 "구현"을 이 번역단위 한 곳에서만 컴파일 ---
#include <MPU6050_6Axis_MotionApps20.h>

#include "imu_mpu6050.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// -----------------
// 내부 구현체 (PIMPL)
// -----------------
struct ImuMpu6050::Impl {
  MPU6050  mpu;

  bool     dmpReady = false;
  uint8_t  mpuIntStatus = 0;
  uint8_t  devStatus = 0;
  uint16_t packetSize = 0;
  uint16_t fifoCount  = 0;
  uint8_t  fifoBuffer[64]{};

  Quaternion q;
  VectorInt16 aa;
  VectorInt16 aaReal;
  VectorInt16 aaWorld;
  VectorFloat gravity;
  float euler[3]{};
  float ypr[3]{};

  float gyroLSBperDPS = 16.4f; // default assumes ±2000 dps

  ImuSample latest{};

  void updateGyroScaleFromFSR() {
    static const float kGyroLSB[4] = {131.0f, 65.5f, 32.8f, 16.4f};
    uint8_t fs_g = mpu.getFullScaleGyroRange(); // 0..3
    gyroLSBperDPS = kGyroLSB[fs_g];
  }
};

// -----------------
// API 구현
// -----------------
ImuMpu6050::ImuMpu6050() : impl_(new Impl) {}
ImuMpu6050::~ImuMpu6050() { delete impl_; }

bool ImuMpu6050::begin(uint8_t intPin, int16_t xg, int16_t yg, int16_t zg, int16_t za) {
  intPin_ = intPin;

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz
#endif

  impl_->mpu.initialize();
  pinMode(intPin_, INPUT);

  if (!impl_->mpu.testConnection()) {
    return false;
  }

  impl_->devStatus = impl_->mpu.dmpInitialize();

  // Gyro/Accel 오프셋
  impl_->mpu.setXGyroOffset(xg);
  impl_->mpu.setYGyroOffset(yg);
  impl_->mpu.setZGyroOffset(zg);
  impl_->mpu.setZAccelOffset(za);

  if (impl_->devStatus == 0) {
    impl_->mpu.setDMPEnabled(true);
    impl_->mpuIntStatus = impl_->mpu.getIntStatus();

    impl_->mpu.resetFIFO();
    delay(10);

    impl_->dmpReady   = true;
    impl_->packetSize = impl_->mpu.dmpGetFIFOPacketSize();

    impl_->updateGyroScaleFromFSR();
    return true;
  }
  return false;
}

void ImuMpu6050::service() {
  if (!impl_->dmpReady) return;

  // DMP INT가 오거나, 혹은 이미 FIFO에 한 패킷 이상 있을 때만 처리(기본 예제 방식)
  if (!mpuInterrupt_) {
    impl_->fifoCount = impl_->mpu.getFIFOCount();
    if (impl_->fifoCount < impl_->packetSize) return;
  }

  mpuInterrupt_ = false; // 한 번만 소비
  impl_->mpuIntStatus = impl_->mpu.getIntStatus();
  impl_->fifoCount    = impl_->mpu.getFIFOCount();

  // overflow 처리
  if ((impl_->mpuIntStatus & 0x10) || impl_->fifoCount == 1024) {
    impl_->mpu.resetFIFO();
    return;
  }

  if (impl_->mpuIntStatus & 0x02) {
    while (impl_->fifoCount < impl_->packetSize) impl_->fifoCount = impl_->mpu.getFIFOCount();

    // 한 패킷만 읽기(오래된 패킷 비우기 로직 없음)
    impl_->mpu.getFIFOBytes(impl_->fifoBuffer, impl_->packetSize);
    impl_->fifoCount -= impl_->packetSize;

    impl_->mpu.dmpGetQuaternion(&impl_->q, impl_->fifoBuffer);
    impl_->mpu.dmpGetGravity(&impl_->gravity, &impl_->q);
    impl_->mpu.dmpGetYawPitchRoll(impl_->ypr, &impl_->q, &impl_->gravity);

    // pitch (rad/deg)
    impl_->latest.pitch_rad = impl_->ypr[1];
    impl_->latest.pitch_deg = impl_->latest.pitch_rad * 180.0f / (float)M_PI;

    // --- RAW gyro Y (deg/s, rad/s) ---
    int16_t gx, gy, gz;
    impl_->mpu.getRotation(&gx, &gy, &gz); // raw LSB
    impl_->latest.gyroY_deg_s = gy / impl_->gyroLSBperDPS;
    impl_->latest.gyroY_rad_s = impl_->latest.gyroY_deg_s * ((float)M_PI / 180.0f);

    impl_->latest.ts_us = micros();
    impl_->latest.fresh = true;
  }
}

bool ImuMpu6050::readLatest(ImuSample* out) {
  if (!out) return false;
  *out = impl_->latest;
  bool hadNew = impl_->latest.fresh;
  impl_->latest.fresh = false;
  return hadNew;
}

void ImuMpu6050::printDiag() {
  if (!impl_->dmpReady) {
    Serial.println(F("[IMU] DMP not ready"));
    return;
  }
  static const int   kGyroFS [4]  = { 250  , 500  , 1000 , 2000 };
  static const int   kAccelLSB[4] = {16384 , 8192 , 4096 , 2048 };
  static const int   kAccelFS [4] = { 2    , 4    , 8    , 16   };

  uint8_t fs_g = impl_->mpu.getFullScaleGyroRange();
  uint8_t fs_a = impl_->mpu.getFullScaleAccelRange();
  Serial.print(F("[IMU] Gyro FSR = ±")); Serial.print(kGyroFS[fs_g]);
  Serial.print(F(" dps, scale = "));    Serial.print(impl_->gyroLSBperDPS); Serial.println(F(" LSB/deg/s"));
  Serial.print(F("[IMU] Accel FSR = ±")); Serial.print(kAccelFS[fs_a]);
  Serial.print(F(" g, scale = "));       Serial.print(kAccelLSB[fs_a]); Serial.println(F(" LSB/g"));
  Serial.print(F("[IMU] DMP packet size = ")); Serial.println(impl_->packetSize);
}

void ImuMpu6050::setOffsets(int16_t xg, int16_t yg, int16_t zg, int16_t za) {
  impl_->mpu.setXGyroOffset(xg);
  impl_->mpu.setYGyroOffset(yg);
  impl_->mpu.setZGyroOffset(zg);
  impl_->mpu.setZAccelOffset(za);
}

float ImuMpu6050::gyroLsbPerDps() const {
  return impl_->gyroLSBperDPS;
}
