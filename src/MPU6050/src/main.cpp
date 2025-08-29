#include <Arduino.h>
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
// #include "MPU6050.h" // MotionApps 헤더를 쓰면 별도 포함 불필요

// I2Cdev.h에서 I2CDEV_ARDUINO_WIRE 구현을 쓴다면 Arduino Wire 라이브러리가 필요
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// 기본 I2C 주소는 0x68
// 필요 시 아래 생성자에 주소를 명시할 수 있음
// AD0(주소 선택) 핀이 GND면 0x68(기본), VCC면 0x69
MPU6050 mpu;
// MPU6050 mpu(0x69); // AD0가 HIGH(0x69)일 때 이 줄을 사용


// Yaw/Pitch/Roll(도) 출력(중력 벡터 계산 필요, 짐벌락 주의)
#define OUTPUT_READABLE_YAWPITCHROLL

// 중력 제거한 센서 로컬 좌표 가속도(orientation 보정 없음)
// #define OUTPUT_READABLE_REALACCEL

// 중력 제거 + 월드 좌표계로 회전 보정한 가속도
// (자기센서 없음 → yaw는 초기 자세 기준 상대값)
// #define OUTPUT_READABLE_WORLDACCEL



#define INTERRUPT_PIN 2
#define LED_PIN 10
bool blinkState = false;

// MPU 제어/상태 변수
bool dmpReady = false;   // DMP 초기화 성공 여부
uint8_t mpuIntStatus;    // MPU의 인터럽트 상태 바이트
uint8_t devStatus;       // 각 동작의 반환 상태(0=성공, !0=오류)
uint16_t packetSize;     // DMP 패킷 크기(기본 42 바이트)
uint16_t fifoCount;      // FIFO 내 바이트 수
uint8_t fifoBuffer[64];  // FIFO로부터 읽어올 버퍼

// 자세/가속도 관련 변수
Quaternion q;            // [w, x, y, z] 쿼터니언
VectorInt16 aa;          // [x, y, z] 가속도 원시값
VectorInt16 aaReal;      // [x, y, z] 중력 제거 가속도
VectorInt16 aaWorld;     // [x, y, z] 월드 좌표 가속도
VectorFloat gravity;     // [x, y, z] 중력 벡터
float euler[3];          // [psi, theta, phi] 오일러 각(라디안)
float ypr[3];            // [yaw, pitch, roll] YPR(라디안) + 중력벡터 사용
 // ----- Control-loop friendly outputs -----
 float yaw_rad=0, pitch_rad=0, roll_rad=0;      // radians
 float yaw_deg=0, pitch_deg=0, roll_deg=0;      // degrees
 float ax_world_g=0, ay_world_g=0, az_world_g=0; // in g
 float ax_world_ms2=0, ay_world_ms2=0, az_world_ms2=0; // in m/s^2
 const float ACCEL_LSB_PER_G = 16384.0f; // for ±2g full-scale
// Gyro raw → deg/s, rad/s (FSR ±2000 dps assumed; change constant if FSR differs)
const float GYRO_LSB_PER_DPS = 16.4f;  float gyroY_dps=0.0f, gyroY_rad_s=0.0f;

// InvenSense Teapot 데모용 패킷(시각화 프로그램과 호환)
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };




volatile bool mpuInterrupt = false; // MPU INT 핀이 HIGH가 되었는지 표시
void dmpDataReady() {
    mpuInterrupt = true;           // ISR: 새로운 DMP 데이터가 준비됨
}




void setup() {
    // I2C 버스 시작(I2Cdev 라이브러리는 자동으로 하지 않음)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // I2C 400kHz (문제 있으면 주석 처리)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // 시리얼 초기화(Teapot 데모는 115200 요구, 프로젝트에 따라 조절 가능)
    Serial.begin(115200);
    while (!Serial); // Leonardo는 열거 대기, 그 외 보드는 즉시 진행

    // 주의: 8MHz 이하 MCU(3.3V Pro Mini 등)는 115200 신뢰성 떨어질 수 있음.
    // 그 경우 38400 이하 속도 사용 또는 외부 크리스털 기반 UART 타이머 권장.

    // 디바이스 초기화
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT); // INT 핀 입력으로 설정(필요시 INPUT_PULLUP)

    // 연결 확인
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // 준비 대기(사용자가 문자를 보내면 DMP 프로그래밍/데모 시작)
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // 버퍼 비우기
    while (!Serial.available());                 // 입력 대기
    while (Serial.available() && Serial.read()); // 다시 버퍼 비우기

    // DMP 로드 및 설정
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // 여기서 사용자 보정값(오프셋)을 넣어야 함(민감도 최소 스케일 기준)
    //! 275  -1604 2270  40  144 -3
    mpu.setXGyroOffset(40);
    mpu.setYGyroOffset(144);
    mpu.setZGyroOffset(-3);
    mpu.setZAccelOffset(2270); // 테스트 칩의 공장 기본은 1688

    // devStatus==0 이면 성공
    if (devStatus == 0) {
        // DMP 활성화
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // 외부 인터럽트 감지 활성화
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        Serial.println();
        Serial.println(F("Clearing FIFO..."));
        mpu.resetFIFO();
        delay(10);
        mpuInterrupt = false;

        // 메인 loop에서 사용 가능 표시
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // 예상 DMP 패킷 크기 얻기(일반적으로 42바이트)
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // 오류 코드:
        // 1 = 초기 메모리 로드 실패
        // 2 = DMP 설정 업데이트 실패
        // (대부분 1이 발생)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // 동작 표시용 LED
    pinMode(LED_PIN, OUTPUT);
}




void loop() {
    // DMP 초기화 실패 시 아무 것도 하지 않음
    if (!dmpReady) return;

    // MPU 인터럽트가 오거나, FIFO에 패킷이 쌓일 때까지 대기
    while (!mpuInterrupt && fifoCount < packetSize) {
        // 이 사이에 다른 작업을 넣어도 무방
        // 필요하면 중간중간 mpuInterrupt 플래그를 확인해 즉시 빠져나와 처리 가능
    }

    // 인터럽트 플래그 리셋 및 INT_STATUS 읽기
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // 현재 FIFO 바이트 수
    fifoCount = mpu.getFIFOCount();

    // 오버플로 체크(일반적으로 코드가 너무 느릴 때 발생)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();                 // FIFO 리셋 후
        Serial.println(F("FIFO overflow!")); // 경고 출력
    }
    else if (mpuIntStatus & 0x02) {
        // Wait until at least one complete packet is available
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // Read the latest (newest) packet
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        //! 나중에 수정
        // if(fifoCount / packetSize >= 2) {
        //   mpu.resetFIFO();
        //   fifoCount = 0;
        // }
        // else fifoCount -= packetSize;
        fifoCount -= packetSize;

        // Common math: orientation and gravity
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        // Store angles (radians & degrees) for control loop
        yaw_rad   = ypr[0];
        pitch_rad = ypr[1];
        roll_rad  = ypr[2];
        yaw_deg   = ypr[0] * 180.0f / M_PI;
        pitch_deg = ypr[1] * 180.0f / M_PI;
        roll_deg  = ypr[2] * 180.0f / M_PI;

        // --- RAW gyro read (Y-axis used for balancing) ---
        int16_t gx, gy, gz;
        mpu.getRotation(&gx, &gy, &gz); // raw LSB
        gyroY_dps   = gy / GYRO_LSB_PER_DPS;            // deg/s
        gyroY_rad_s = gyroY_dps * (M_PI / 180.0f);      // rad/s




        // --- Debug output: current angle (pitch, deg) and angular velocity (gyro Y, deg/s) ---
        Serial.print("pitch_deg\t");
        Serial.print(pitch_deg-39);
        Serial.print("\tgyroY_deg_s\t");
        Serial.println(gyroY_dps);



        #ifdef OUTPUT_READABLE_WORLDACCEL
        // World-frame linear acceleration (gravity removed)
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

        // Save acceleration both in g and in m/s^2
        ax_world_g   = aaWorld.x / ACCEL_LSB_PER_G;
        ay_world_g   = aaWorld.y / ACCEL_LSB_PER_G;
        az_world_g   = aaWorld.z / ACCEL_LSB_PER_G;
        ax_world_ms2 = ax_world_g * 9.80665f;
        ay_world_ms2 = ay_world_g * 9.80665f;
        az_world_ms2 = az_world_g * 9.80665f;
        #endif

        // Indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}