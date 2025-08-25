#include <PID_v1_bc.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <FlexCAN_T4.h>

// --- MPU6050 ---
MPU6050 mpu;
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];
volatile bool mpuInterrupt = false;
void dmpDataReady() { mpuInterrupt = true; }

// --- PID ---
double setpoint = 180;    // 균형 각도
double input, output;
double Kp = 21, Ki = 140, Kd = 0.8;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// --- CAN ---
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
const uint32_t CAN_BAUD = 1'000'000;
const uint32_t WHEEL_MOTOR_LEFT_ID  = 0x141;
const uint32_t WHEEL_MOTOR_RIGHT_ID = 0x142;

// --- 모터 ON ---
void motorOn() {
    for (uint32_t id : {WHEEL_MOTOR_LEFT_ID, WHEEL_MOTOR_RIGHT_ID}) {
        CAN_message_t msg{};
        msg.id = id;
        msg.len = 8;
        msg.buf[0] = 0x88;
        Can1.write(msg);
    }
}

// --- 모터 동작 함수 (CAN 버전) ---
void Forward(double speed) { // 전진: 양쪽 모두 +speed
    int32_t spd_raw = (int32_t)(fabs(speed) * 100); // 항상 +값
    for (uint32_t id : {WHEEL_MOTOR_LEFT_ID, WHEEL_MOTOR_RIGHT_ID}) {
        CAN_message_t msg{};
        msg.id = id;
        msg.len = 8;
        msg.buf[0] = 0xA2;
        msg.buf[1] = msg.buf[2] = msg.buf[3] = 0;
        msg.buf[4] = spd_raw & 0xFF;
        msg.buf[5] = (spd_raw >> 8) & 0xFF;
        msg.buf[6] = (spd_raw >> 16) & 0xFF;
        msg.buf[7] = (spd_raw >> 24) & 0xFF;
        Can1.write(msg);
    }
    Serial.println("F");
}

void Reverse(double speed) { // 후진: 양쪽 모두 -speed
    int32_t spd_raw = (int32_t)(-fabs(speed) * 100); // 항상 -값
    for (uint32_t id : {WHEEL_MOTOR_LEFT_ID, WHEEL_MOTOR_RIGHT_ID}) {
        CAN_message_t msg{};
        msg.id = id;
        msg.len = 8;
        msg.buf[0] = 0xA2;
        msg.buf[1] = msg.buf[2] = msg.buf[3] = 0;
        msg.buf[4] = spd_raw & 0xFF;
        msg.buf[5] = (spd_raw >> 8) & 0xFF;
        msg.buf[6] = (spd_raw >> 16) & 0xFF;
        msg.buf[7] = (spd_raw >> 24) & 0xFF;
        Can1.write(msg);
    }
    Serial.println("R");
}

void Stop() { // 정지
    for (uint32_t id : {WHEEL_MOTOR_LEFT_ID, WHEEL_MOTOR_RIGHT_ID}) {
        CAN_message_t msg{};
        msg.id = id;
        msg.len = 8;
        msg.buf[0] = 0xA2;
        msg.buf[1] = msg.buf[2] = msg.buf[3] = 0;
        msg.buf[4] = 0;
        msg.buf[5] = 0;
        msg.buf[6] = 0;
        msg.buf[7] = 0;
        Can1.write(msg);
    }
    Serial.println("S");
}

const int dmpIntPin = 2;

void setup() {
    Serial.begin(115200);

    mpu.initialize();
    Serial.println(mpu.testConnection() ? F("MPU6050 연결 성공") : F("MPU6050 연결 실패"));
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1688);

    if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        pinMode(dmpIntPin, INPUT);
        attachInterrupt(dmpIntPin, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.print(F("DMP 초기화 실패 (코드 "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-300, 300);

    Can1.begin();
    Can1.setBaudRate(CAN_BAUD);
    delay(100);

    motorOn();
    delay(10);
}

void loop() {
    if (!dmpReady) return;

    while (!mpuInterrupt && fifoCount < packetSize) {
        pid.Compute();
        Serial.print("Pitch: "); Serial.print(input);
        Serial.print(" => "); Serial.println(output);

        if (input > 150 && input < 210) { // 안정 범위
            if (output > 0)       Forward(output); // 전진
            else if (output < 0)  Reverse(output); // 후진
            else                  Stop();          // 정지
        } else {
            Stop();
        }
    }

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
    } else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        input = ypr[1] * 180 / M_PI + 180; // pitch (deg)
    }
}
