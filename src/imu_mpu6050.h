#pragma once

#include <cstdint>
#include <cmath>    // for NAN

struct ImuSample {
  uint32_t ts_us = 0;        // timestamp (micros)
  float    pitch_rad = NAN;
  float    pitch_deg = NAN;
  float    gyroY_rad_s = NAN; // RAW gyro Y (angular rate)
  float    gyroY_deg_s = NAN;
  bool     fresh = false;     // set true when a new packet was consumed
};

class ImuMpu6050 {
public:
  ImuMpu6050();
  ~ImuMpu6050();

  // Initialize I2C + MPU + DMP. INT 핀은 외부에서 attachInterrupt()로 연결하세요.
  bool begin(uint8_t intPin = 2, int16_t xg = 0, int16_t yg = 0,
             int16_t zg = 0, int16_t za = 0);

  // ISR에서 호출: 아주 가볍게 플래그만 세움
  inline void isr() { mpuInterrupt_ = true; }

  // loop()에서 자주 호출: INT/_FIFO 상태 확인하고 1패킷 처리 (기본 예제 흐름)
  void service();

  // 최신 샘플 가져오기. 새 데이터가 반영됐으면 true를 반환.
  bool readLatest(ImuSample* out);

  // 정보 출력(풀스케일/스케일, 패킷 크기 등)
  void printDiag();

  // 오프셋 수동 적용(캘리브레이션 결과를 넣을 때)
  void setOffsets(int16_t xg, int16_t yg, int16_t zg, int16_t za);

  // 현재 설정된 Gyro LSB/deg/s 반환(±FSR에 따라 달라짐)
  float gyroLsbPerDps() const;

private:
  struct Impl;       // PIMPL로 내부 의존성 차단
  Impl* impl_ = nullptr;

  // 외부 ISR과의 최소 공유 상태만 헤더에 노출
  uint8_t  intPin_ = 2;
  volatile bool mpuInterrupt_ = false;
};