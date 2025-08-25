#include <PID_v1_bc.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
uint8_t *verifyBuffer = nullptr;

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

 
// 아래의 4개의 값을 본인의 로봇에 맞게 튜닝합니다.
/*********파라메터 튜닝 시작*********/
double setpoint= 180; //로봇이 지면에서 평형을 유지하는 상태의 값입니다.
//다음은 PID 제어기의 Kp, Ki, Kd 파라메타를 설정합니다. 아래의 순서대로 설정합니다.
double Kp = 21; 
double Kd = 0.8; 
double Ki = 140; 
/******파라메터 튜닝 끝*********/

double input, output;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

 

volatile bool mpuInterrupt = false;     // MPU6050의 인터럽트 발생유무 확인
void dmpDataReady()
{
    mpuInterrupt = true;
}

void setup() {
  Serial.begin(115200);

    // MPU6050 초기화
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // MPU6050 통신확인
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // DMP 초기화
    devStatus = mpu.dmpInitialize();

    
    // 기본 옵셋값 설정
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1688); 

    // 정상동작하는 경우
    if (devStatus == 0)
    {
        // DMP 가동
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // 아두이노 인터럽트 설정
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // DMP 사용가능 상태 설정
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // 패킷사이즈 가져오기
        packetSize = mpu.dmpGetFIFOPacketSize();
        
        //PID 설정
        pid.SetMode(AUTOMATIC);
        pid.SetSampleTime(10);
        pid.SetOutputLimits(-255, 255);  
    }
    else
    {
        // 오류시
        // 1 = 초기 메모리 에러
        // 2 = DMP 설정 오류
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    //모터 출력핀 초기화
    pinMode (6, OUTPUT);
    pinMode (9, OUTPUT);
    pinMode (10, OUTPUT);
    pinMode (11, OUTPUT);

   //모터 동작 OFF
    analogWrite(6,LOW);
    analogWrite(9,LOW);
    analogWrite(10,LOW);
    analogWrite(11,LOW);
}

 

void loop() {
 
    // 오류시 작업중지
    if (!dmpReady) return;

    // MPU 인터럽트나 패킷 대기
    while (!mpuInterrupt && fifoCount < packetSize)
    {
        //MPU6050 데이터가 없는 경우 PID 계산
        pid.Compute();   
        
        //시리얼 모니터로 현재 상태 출력
        Serial.print(input); Serial.print(" =>"); Serial.println(output);
               
        if (input>150 && input<200){//로봇이 기울어지는 경우(각도 범위내에서만)
          
        if (output>0) //앞으로 기울어지는 경우
        Forward(); //전진
        else if (output<0) //뒤로 기울어지는 경우
        Reverse(); //후진
        }
        else //로봇이 기울어지지 않은 경우
        Stop(); //모터 정지
        
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    }
    else if (mpuIntStatus & 0x02)
    {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer); //get value for q
        mpu.dmpGetGravity(&gravity, &q); //get value for gravity
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); //get value for ypr

        input = ypr[1] * 180/M_PI + 180;

   }
}

void Forward() //전진
{
    analogWrite(6,output);
    analogWrite(9,0);
    analogWrite(10,output);
    analogWrite(11,0);
    Serial.print("F"); 
}

void Reverse() //후진
{
    analogWrite(6,0);
    analogWrite(9,output*-1);
    analogWrite(10,0);
    analogWrite(11,output*-1); 
    Serial.print("R");
}

void Stop() //정지
{
    analogWrite(6,0);
    analogWrite(9,0);
    analogWrite(10,0);
    analogWrite(11,0); 
    Serial.print("S");
}