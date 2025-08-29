#pragma once
#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

struct ImuSample {
  uint32_t ts_us = 0;     // timestamp (micros)
  float    pitch_rad = NAN;
  float    pitch_deg = NAN;
  float    gyroY_rad_s = NAN; // RAW gyro Y (angular rate)
  float    gyroY_deg_s = NAN;
  bool     fresh = false;  // set true when a new packet was consumed
};

class ImuMpu6050 {
public:
  ImuMpu6050() = default;

  // Initialize I2C + MPU + DMP. INT 핀은 외부에서 attachInterrupt()로 연결하세요.
  bool begin(uint8_t intPin=2, int16_t xg=0, int16_t yg=0, int16_t zg=0, int16_t za=0);

  // ISR에서 호출: 아주 가볍게 플래그만 세움
  inline void isr() { mpuInterrupt_ = true; }

  // loop()에서 자주 호출: INT/_FIFO 상태 확인하고 1패킷 처리 (기본 예제 흐름)
  void service();

  // 최신 샘플 가져오기. 새 데이터가 반영됐으면 true를 반환.
  bool readLatest(ImuSample* out);

  // 정보 출력(풀스케일/스케일, 패킷 크기 등)
  void printDiag();

  // 오프셋 수동 적용(캘리브레이션 결과를 넣을 때)
  void setOffsets(int16_t xg, int16_t yg, int16_t zg, int16_t za) {
    mpu_.setXGyroOffset(xg); mpu_.setYGyroOffset(yg); mpu_.setZGyroOffset(zg); mpu_.setZAccelOffset(za);
  }

  // 현재 설정된 Gyro LSB/deg/s 반환(±FSR에 따라 달라짐)
  float gyroLsbPerDps() const { return gyroLSBperDPS_; }

private:
  MPU6050  mpu_;
  uint8_t  intPin_ = 2;

  // DMP/IRQ/FIFO 상태
  volatile bool mpuInterrupt_ = false;
  bool     dmpReady_ = false;
  uint8_t  mpuIntStatus_ = 0;
  uint8_t  devStatus_ = 0;
  uint16_t packetSize_ = 0;
  uint16_t fifoCount_  = 0;
  uint8_t  fifoBuffer_[64];

  // DMP 계산용 컨테이너
  Quaternion q_;
  VectorInt16 aa_;
  VectorInt16 aaReal_;
  VectorInt16 aaWorld_;
  VectorFloat gravity_;
  float euler_[3];
  float ypr_[3];


  float gyroLSBperDPS_ = 16.4f; // default assumes ±2000 dps

  ImuSample latest_{};

  // 내부 헬퍼
  void updateGyroScaleFromFSR();
};

