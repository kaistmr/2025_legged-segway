//Final
//#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <Servo.h>

#include "config.h"
#include "motor_control.h"
#include "motor_status.h"
#include "motor.h"
#include "imu_mpu6050.h"
#include <cmath>

// HC-06 배선: HC-06 TXD -> 7(Teensy RX), HC-06 RXD -> 8(Teensy TX)
static const uint8_t BT_RX = 7;  // Teensy 쪽 수신핀 (HC-06 TXD 연결)
static const uint8_t BT_TX = 8;  // Teensy 쪽 송신핀 (HC-06 RXD 연결)

// SoftwareSerial btSerial(BT_RX, BT_TX); // (RX, TX)
HardwareSerial& btSerial = Serial2;


ImuMpu6050 IMU;
constexpr int IMU_INT_PIN = 2;
void imuISR() { IMU.isr(); }

// ---------- Inner (theta) PID ----------
float Kp = 5000;
float Ki = 0;
float Kd = 0;
float g_i_accum = 0.0f;
float g_theta_ref = 68.7f * M_PI / 180.0f; // 목표 자세 (rad)
float g_dtheta_ref_rad_s = 0.0f;              // 목표 각속도 (rad/s)

constexpr float U_MAX_DPS = 50000.0f;         // 모터 속도 제한값 (deg/s)
constexpr float E_DEADBAND_RAD = 0.003f;


// ==============================
// PID compute helpers
// ==============================
static float computeInnerThetaPID(float theta_ref_rad,
                                  float theta_meas_rad,
                                  float gyroY_rad_s,
                                  float dt_s) {

  // 기존 내부 PID 동작 유지 (데드밴드 + 안티와인드업 + 포화)
  float e = theta_ref_rad - theta_meas_rad;   // [rad]
  if (fabsf(e) < E_DEADBAND_RAD) e = 0.0f;

  float u_unsat = Kp * e + Ki * g_i_accum + Kd * (g_dtheta_ref_rad_s - gyroY_rad_s);
  float u_cmd   = constrain(u_unsat, -U_MAX_DPS, U_MAX_DPS);

  const bool sat_hi = (u_unsat >  U_MAX_DPS);
  const bool sat_lo = (u_unsat < -U_MAX_DPS);
  const bool clamp  = ((sat_hi && e > 0.0f) || (sat_lo && e < 0.0f));

  if (!clamp) {
    g_i_accum += e * dt_s;
    u_unsat = Kp * e + Ki * g_i_accum + Kd * (g_dtheta_ref_rad_s - gyroY_rad_s);
    u_cmd   = constrain(u_unsat, -U_MAX_DPS, U_MAX_DPS);
  }
  return u_cmd; // [deg/s] (K gains chosen so output is deg/s; inputs are in rad & rad/s)
}


// --------- Servo ---------
Servo J1, J2;
static const uint8_t servo_left_pin = 4;
static const uint8_t servo_right_pin = 5;
// 안전 시작 범위(필요하면 500/2500으로 넓히기)
static const uint16_t MIN_US = 700;
static const uint16_t MAX_US = 2300;

static const uint16_t H_INIT = 1500;


// ---------- CAN / Motors ----------
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;

Motor W1{ID_WHEEL_1, "W1"};
Motor W2{ID_WHEEL_2, "W2"};

// Physical CAN IDs in logical order (from config.h)
static const uint8_t ROLE_IDS[] = {ID_WHEEL_1, ID_WHEEL_2 };
static const size_t  ROLE_COUNT = sizeof(ROLE_IDS)/sizeof(ROLE_IDS[0]);

// Optional groups of Motor objects (for convenient on/off)
static Motor* const ALL[]    = { &W1, &W2 };
static Motor* const WHEELS[] = { &W1, &W2 };

// Small helpers
static inline void MotorOnAll()    { for (auto m : ALL)    m->on();   }
static inline void MotorStopAll()  { for (auto m : ALL)    m->stop(); }
//static inline void Request9AAll()  { for (size_t i=0; i<ROLE_COUNT; ++i) status::sendRead9A(ROLE_IDS[i]); }
static inline void Request9AAll()  { for (auto m : ALL) m->request9A();}
static inline void WheelsStop()  { for (auto m : WHEELS) m->stop(); }

// Translate physical CAN ID to a short role label for prints
static inline const char* roleLabelById(uint8_t phys) {
  if (phys == ID_WHEEL_1) return "W1";
  if (phys == ID_WHEEL_2) return "W2";
  return "?";
}

// ===== Manual safety state machine =====
enum class Mode : uint8_t { RUN = 0, SAFE_HALT = 1 };
static Mode g_mode = Mode::SAFE_HALT;

static uint32_t t_blink      = 0;     // LED blink timer

static uint8_t  g_latched_mask = 0;              // bit i set => ROLE_IDS[i] is latched SAFE
static uint16_t g_latched_err[ROLE_COUNT] = {0}; // per-role latched error snapshot

// Timers (drift-free by += period)
static uint32_t t_ctrl   = 0;   // control loop
static uint32_t t_9a     = 0;   // safety poll (0x9A) round-robin
static uint32_t t_print  = 0;   // optional serial print

const  uint32_t DT_CTRL  = 1;    // control tick (ms)
const  uint32_t DT_9A    = 100;  // health poll (ms)

const  uint32_t DT_PRINT = 200;  // 5 Hz print (optional)

// Manual serial command parser (non-blocking)
static inline void processSerialCommands() {
  while (Serial.available() > 0) {
    int ch = Serial.read();
    if (ch < 0) break;

    switch ((char)ch) {
      case '?':
        Serial.println();
        Serial.println("================================================");
        Serial.println("  Manual Safety Console - Help");
        Serial.println("================================================");
        Serial.printf ("  Mode    : %s\n", (g_mode==Mode::RUN?"RUN":"SAFE_HALT"));
        Serial.println("------------------------------------------------");
        Serial.println("  Per-motor SAFE status:");
        for (size_t i=0;i<ROLE_COUNT;++i){
          uint8_t phys = ROLE_IDS[i];
          bool latched = (g_latched_mask >> i) & 0x1;
          status::Health h; bool hv = status::getHealth(phys, &h);
          Serial.printf("    %s (ID=%u): %s", roleLabelById(phys), phys, latched? "LATCHED" : " - ");
          if (latched) Serial.printf(", latched_err=0x%04X", g_latched_err[i]);
          if (hv)      Serial.printf(", current_err=0x%04X", (unsigned)h.error);
          Serial.println();
        }
        Serial.println("------------------------------------------------");
        Serial.println("  Commands:");
        Serial.println("    ?    : show this help");
        Serial.println("    r    : resume RUN");
        Serial.println("    c    : clear 0x9B for ALL");
        Serial.println("    1..4 : clear 0x9B for ROLE index (order: ROLE_IDS array)");
        Serial.println("    s    : stop all motors");
        Serial.println("    o    : power on all motors");
        Serial.println("    a    : request 0x9A from all");
        Serial.println("================================================\n");
        break;
      case 'e': {
        Serial.println("[CMD] HARD RESET: rebooting MCU to rerun setup()");
        delay(50);
        Serial.flush();
        // Cortex-M7 system reset (same path as power cycle for firmware)
        SCB_AIRCR = 0x05FA0004;   // VECTKEY(0x5FA) | SYSRESETREQ
        while (1) { }
      } break;
      case 'c': { // clear all (use mapped IDs from config)
        for (size_t i = 0; i < ROLE_COUNT; ++i) status::sendClear9B(ROLE_IDS[i]);
        Serial.print("[CMD] 0x9B clear sent to IDs: ");
        for (size_t i = 0; i < ROLE_COUNT; ++i) { Serial.print(ROLE_IDS[i]); Serial.print(i+1<ROLE_COUNT?", ":"\n"); }
      } break;

      case '1': case '2': case '3': case '4': { // map index to physical ID
        size_t idx = (size_t)((char)ch - '1');   // '1'->0, '2'->1, ...
        if (idx < ROLE_COUNT) {
          uint8_t phys = ROLE_IDS[idx];
          status::sendClear9B(phys);
          Serial.printf("[CMD] 0x9B clear sent to physical ID=%u (role index %u)\n", phys, (unsigned)idx);
        }
      } break;

      case 'r': { // resume normal operation and reset latched snapshots
         g_mode = Mode::RUN;
         g_latched_mask = 0;
         for (size_t i=0;i<ROLE_COUNT;++i) {
           g_latched_err[i] = 0;
         }
         g_i_accum = 0.0f;  // reset integral on resume
         Serial.println("[CMD] Resume RUN (latched state cleared)");

        
      } break;

      case 's': // stop all
        MotorStopAll();
        Serial.println("[CMD] StopAll");
        break;

      case 'o': // on all
        MotorOnAll();
        Serial.println("[CMD] OnAll");
        break;

      case 'a': // request 0x9A all
        Request9AAll();
        Serial.println("[CMD] Request 0x9A all");
        break;

      case 'h': {
        Request9AAll();

        Serial.println("----- 0x9A Health snapshot -----");
        for (size_t i = 0; i < ROLE_COUNT; ++i) {
          const uint8_t phys = ROLE_IDS[i];
          status::Health h;
          if (status::getHealth(phys, &h)) {
            const float busV = (float)h.bus_dV * 0.1f;   // 0.1V/LSB
            Serial.printf("  %s (ID=%u): temp=%dC, bus=%.1fV, err=0x%04X\n",
                          roleLabelById(phys), phys,
                          (int)h.temperature, busV, (unsigned)h.error);
          } else {
            Serial.printf("  %s (ID=%u): (no 0x9A yet)\n",
                          roleLabelById(phys), phys);
          }
        }
        break;
      }

      default:
        // ignore others
        break;
    }
  }
}

// Blink LED slowly when SAFE
static inline void blinkWhenSafe(uint32_t now) {
  if (g_mode != Mode::SAFE_HALT) return;
  const uint32_t BLINK = 300; // ms
  if (now - t_blink >= BLINK) {
    t_blink += BLINK;
    digitalWrite(LED_R, !digitalRead(LED_R));
    digitalWrite(LED_G, LOW);
  }
}

// Scan all motors' health and latch SAFE if any error!=0
static inline void checkHealthAndLatch() {
  bool latchedThisTick = false;
  for (size_t i=0;i<ROLE_COUNT;++i) {
    uint8_t phys = ROLE_IDS[i];
    status::Health h;
    if (status::getHealth(phys, &h) && h.error != 0) {
      const bool wasLatched = (g_latched_mask >> i) & 0x1;
      if (!wasLatched) {
        g_latched_mask |= (1u << i);
        g_latched_err[i] = (uint16_t)h.error; // snapshot at first detection
        Serial.printf("[SAFE] Latched by %s (ID=%u), err=0x%04X\n", roleLabelById(phys), phys, (uint16_t)h.error);
      }

      if (g_mode != Mode::SAFE_HALT && !latchedThisTick) {
        Serial.println("Entering SAFE_HALT");
        MotorStopAll();
        g_mode = Mode::SAFE_HALT;
        g_i_accum = 0.0f; // reset integral on safety halt
        latchedThisTick = true;
      }
    }
  }


}

// [PREFLIGHT] Health 체크 유틸
static bool jointsHealthyOnce() {
  // 조인트 둘 다 0x9A 유효 + error==0 이어야 true
  const uint8_t JOINT_IDS[2] = { ID_JOINT_1, ID_JOINT_2 };
  for (int k = 0; k < 2; ++k) {
    status::Health h;
    if (!status::getHealth(JOINT_IDS[k], &h)) return false; // 아직 수신 안됨
    if (h.error != 0) return false;                         // 에러 존재
  }
  return true;
}

#define TEST_MODE
#define DT_CONTROL 1
#define BTSERIAL 0

void setup() {
  Serial.begin(115200);
  btSerial.begin(9600);
  while (!Serial && millis() < 2000) {}

  // CAN init
  Can1.begin();
  Can1.setBaudRate(CAN1_BAUD);

  // LED init
  pinMode(LED_R, OUTPUT);
  pinMode(LED_Y, OUTPUT);
  pinMode(LED_G, OUTPUT);

  // Joint(Servo) init
  J1.attach(servo_left_pin, MIN_US, MAX_US);
  J2.attach(servo_right_pin, MIN_US, MAX_US);

  // IMU initialization
  if (!IMU.begin(IMU_INT_PIN, 40, 144, -3, 2270)) {  //! plugin offset values!!
    Serial.println("[IMU] begin failed");
    while (1) { digitalWrite(LED_Y, HIGH); delay(100); digitalWrite(LED_Y, LOW); delay(100);}   // 초기화 실패 시 멈춤(원하면 제거)
  }

  attachInterrupt(digitalPinToInterrupt(IMU_INT_PIN), imuISR, RISING);
  IMU.printDiag();

  // Encoder resolution: set to match your motor (14 / 15 / 18)
  status::setEncoderBits(18);

  digitalWrite(LED_R, HIGH);
  digitalWrite(LED_G, HIGH);
  digitalWrite(LED_Y, HIGH);

  MotorOnAll();
  delay(500);
  // Power on all motors
  MotorStopAll();

  // Joint angle Init
  J1.writeMicroseconds(H_INIT);
  J2.writeMicroseconds(H_INIT);

  digitalWrite(LED_R, LOW);
  digitalWrite(LED_G, LOW);
  digitalWrite(LED_Y, LOW);
  for (size_t i = 0; i < ROLE_COUNT; ++i) status::sendClear9B(ROLE_IDS[i]);
  Serial.println("Ready (class-based). Manual safety FSM: RUN/SAFE_HALT with latch + serial commands.");
  #if DT_CONTROL==1
  g_mode = Mode::RUN;
  #endif
}



void loop() {
  status::poll();
  IMU.service();
  const uint32_t now = millis();

  if (g_mode == Mode::RUN) {
    digitalWrite(LED_R, LOW);
    digitalWrite(LED_G, HIGH);

    if (now - t_ctrl >= DT_CTRL) {
      t_ctrl += DT_CTRL;

      ImuSample s;
      if (IMU.readLatest(&s)) {
        const float pitch_rad   = s.pitch_rad;     // DMP (ypr[1])
        const float gyroY_rad_s = s.gyroY_rad_s;   // RAW (레지스터)

        // 실제 샘플 간격 사용
        static uint32_t last_ts = 0;
        if (last_ts == 0) { last_ts = s.ts_us; return; }  // 첫 샘플은 기준만 잡고 패스
        float dt_s = (s.ts_us - last_ts) * 1e-6f;
        dt_s = fminf(fmaxf(dt_s, 0.0005f), 0.02f); // 0.5~20 ms로 제한

        last_ts = s.ts_us;

        float u_cmd = computeInnerThetaPID(g_theta_ref, pitch_rad, gyroY_rad_s, dt_s);

        // Wheel commands (signs kept as in original)
        W1.speedDps(-u_cmd);
        W2.speedDps( u_cmd);

        // Saturation indicator: HIGH when saturated
        digitalWrite(LED_Y, (fabsf(u_cmd) >= U_MAX_DPS - 1e-3f) ? HIGH : LOW);

        Serial.println(pitch_rad*180/M_PI);
        Serial.println(gyroY_rad_s*180/M_PI);
        Serial.print("\n\n");

      }
    }

  }
  // In SAFE_HALT: no control commands are sent (only poll, 0x9A polling, serial, LED blink)

  if (now - t_9a >= DT_9A) {
    t_9a += DT_9A;
    static size_t rr = 0; // round-robin index 0..ROLE_COUNT-1
    status::sendRead9A(ROLE_IDS[rr]);
    rr = (rr + 1) % ROLE_COUNT;
    //for(auto m : ALL) m->request9A();
  }

  #if BTSERIAL == 1
  if (btSerial.available()) {
    char c = btSerial.read();
    Serial.println(c);
  }
  #endif

  checkHealthAndLatch();

  #ifdef TEST_MODE
  processSerialCommands();
  #endif

  blinkWhenSafe(now);

}
