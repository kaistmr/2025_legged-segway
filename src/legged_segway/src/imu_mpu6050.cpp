#include "imu_mpu6050.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const float kGyroLSB[4]  = {131.0f, 65.5f, 32.8f, 16.4f};
static const int   kGyroFS [4]  = { 250  , 500  , 1000 , 2000 };
static const int   kAccelLSB[4] = {16384 , 8192 , 4096 , 2048 };
static const int   kAccelFS [4] = { 2    , 4    , 8    , 16   };

bool ImuMpu6050::begin(uint8_t intPin, int16_t xg, int16_t yg, int16_t zg, int16_t za) {
  intPin_ = intPin;

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz
#endif

  mpu_.initialize();
  pinMode(intPin_, INPUT);

  if (!mpu_.testConnection()) {
    return false;
  }

  devStatus_ = mpu_.dmpInitialize();

  ImuMpu6050::setOffsets(xg, yg, zg, za);

  if (devStatus_ == 0) {
    mpu_.setDMPEnabled(true);
    mpuIntStatus_ = mpu_.getIntStatus();

    mpu_.resetFIFO();
    delay(10);
    mpuInterrupt_ = false;

    dmpReady_   = true;
    packetSize_ = mpu_.dmpGetFIFOPacketSize();

    updateGyroScaleFromFSR();

    return true;
  }
  return false;
}

void ImuMpu6050::service() {
  if (!dmpReady_) return;

  // DMP INT가 오거나, 혹은 이미 FIFO에 한 패킷 이상 있을 때만 처리(기본 예제 방식)
  if (!mpuInterrupt_) {
    fifoCount_ = mpu_.getFIFOCount();
    if (fifoCount_ < packetSize_) return;
  }

  mpuInterrupt_ = false;
  mpuIntStatus_ = mpu_.getIntStatus();
  fifoCount_    = mpu_.getFIFOCount();

  // overflow 처리
  if ((mpuIntStatus_ & 0x10) || fifoCount_ == 1024) {
    mpu_.resetFIFO();
    return;
  }

  if (mpuIntStatus_ & 0x02) {
    while (fifoCount_ < packetSize_) fifoCount_ = mpu_.getFIFOCount();

    //! 한 패킷만 읽기(오래된 패킷 비우기 로직 없음)
    mpu_.getFIFOBytes(fifoBuffer_, packetSize_);
    fifoCount_ -= packetSize_;

    mpu_.dmpGetQuaternion(&q_, fifoBuffer_);
    mpu_.dmpGetGravity(&gravity_, &q_);
    mpu_.dmpGetYawPitchRoll(ypr_, &q_, &gravity_);

    // pitch (rad/deg)
    latest_.pitch_rad = ypr_[1];
    latest_.pitch_deg = latest_.pitch_rad * 180.0f / (float)M_PI;

    // --- RAW gyro Y (deg/s, rad/s) ---
    int16_t gx, gy, gz;
    mpu_.getRotation(&gx, &gy, &gz); // raw LSB
    latest_.gyroY_deg_s = gy / gyroLSBperDPS_;
    latest_.gyroY_rad_s = latest_.gyroY_deg_s * ((float)M_PI / 180.0f);

    latest_.ts_us = micros();
    latest_.fresh = true;
  }
}

bool ImuMpu6050::readLatest(ImuSample* out) {
  if (!out) return false;
  *out = latest_;
  bool hadNew = latest_.fresh;
  latest_.fresh = false;
  return hadNew;
}

void ImuMpu6050::printDiag() {
  if (!dmpReady_) {
    Serial.println(F("[IMU] DMP not ready"));
    return;
  }
  uint8_t fs_g = mpu_.getFullScaleGyroRange();
  uint8_t fs_a = mpu_.getFullScaleAccelRange();
  Serial.print(F("[IMU] Gyro FSR = ±")); Serial.print(kGyroFS[fs_g]);
  Serial.print(F(" dps, scale = "));    Serial.print(kGyroLSB[fs_g]); Serial.println(F(" LSB/deg/s"));
  Serial.print(F("[IMU] Accel FSR = ±")); Serial.print(kAccelFS[fs_a]);
  Serial.print(F(" g, scale = "));       Serial.print(kAccelLSB[fs_a]); Serial.println(F(" LSB/g"));
  Serial.print(F("[IMU] DMP packet size = ")); Serial.println(packetSize_);
}

void ImuMpu6050::updateGyroScaleFromFSR() {
  uint8_t fs_g = mpu_.getFullScaleGyroRange(); // 0..3
  gyroLSBperDPS_ = kGyroLSB[fs_g];
}
