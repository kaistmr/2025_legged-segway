#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "config.h"
#include "motor_control.h"
#include "motor_status.h"
#include "motor.h"
#include "imu_mpu6050.h"

ImuMpu6050 IMU;
constexpr int IMU_INT_PIN = 2;
void imuISR() { IMU.isr(); }

float Kp = 1;
float Ki = 0;
float Kd = 0;
float g_i_accum = 0.0f;                 // PID I-term state (rad*s)
float g_theta_ref = 0.0f;               // 목표 자세 (rad). 기본: 직립 0rad
float g_dtheta_ref_rad_s = 0.0f;        // 목표 각속도 (rad/s). 기본: 0
constexpr float U_MAX_DPS = 2000.0f;     // 모터 속도 제한값 (deg/s)
constexpr float E_DEADBAND_RAD = 0.003f; // ~0.17 deg: 작은 오차 무시 (데드밴드)

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;

Motor J1{ID_JOINT_1, "J1"};
Motor J2{ID_JOINT_2, "J2"};
Motor W1{ID_WHEEL_1, "W1"};
Motor W2{ID_WHEEL_2, "W2"};

// Physical CAN IDs in logical order (from config.h)
static const uint8_t ROLE_IDS[] = { ID_JOINT_1, ID_JOINT_2, ID_WHEEL_1, ID_WHEEL_2 };
static const size_t  ROLE_COUNT = sizeof(ROLE_IDS)/sizeof(ROLE_IDS[0]);

// Optional groups of Motor objects (for convenient on/off)
static Motor* const ALL[]    = { &J1, &J2, &W1, &W2 };
static Motor* const WHEELS[] = { &W1, &W2 };
static Motor* const JOINTS[] = { &J1, &J2 };

// Small helpers
static inline void MotorOnAll()    { for (auto m : ALL)    m->on();   }
static inline void MotorStopAll()  { for (auto m : ALL)    m->stop(); }
//static inline void Request9AAll()  { for (size_t i=0; i<ROLE_COUNT; ++i) status::sendRead9A(ROLE_IDS[i]); }
static inline void Request9AAll()  { for (auto m : ALL) m->request9A();}
static inline void Request94All()  { for (auto m : ALL) m->request94();}
static inline void WheelsStop()  { for (auto m : WHEELS) m->stop(); }
static inline void JointsOn()    { for (auto m : JOINTS) m->on();   }

// Translate physical CAN ID to a short role label for prints
static inline const char* roleLabelById(uint8_t phys) {
  if (phys == ID_WHEEL_1) return "W1";
  if (phys == ID_WHEEL_2) return "W2";
  if (phys == ID_JOINT_1) return "J1";
  if (phys == ID_JOINT_2) return "J2";
  return "?";
}

// ===== Manual safety state machine =====
enum class Mode : uint8_t { RUN = 0, SAFE_HALT = 1 };
static Mode g_mode = Mode::RUN;

static uint32_t t_blink      = 0;     // LED blink timer

static uint8_t  g_latched_mask = 0;             // bit i set => ROLE_IDS[i] is latched SAFE
static uint16_t g_latched_err[ROLE_COUNT] = {0}; // per-role latched error snapshot

// Timers (drift-free by += period)
static uint32_t t_ctrl   = 0;   // control loop
static uint32_t t_9a     = 0;   // safety poll (0x9A) round-robin
static uint32_t t_req94  = 0;   // request 0x94 periodically (only when RUN)
static uint32_t t_print  = 0;   // optional serial print

const  uint32_t DT_CTRL  = 10;   // 100 Hz control
const  uint32_t DT_9A    = 100;  // 10 Hz -> ~2.5 Hz/motor via round-robin (4 motors)
const  uint32_t DT_REQ94 = 15;   // request 0x94 at control cadence (e.g., 100 Hz)

const  uint32_t DT_PRINT = 200;  // 5 Hz print (optional)

// Cached joint angles from 0x94 (single‑turn degrees)
static float    g_joint_deg[2] = {NAN, NAN};
static uint32_t g_joint_ts [2] = {0, 0};    // millis() timestamp when snapshot updated

static float INIT_H_DEG = 170;
static float H_DEG = 150;


// Manual serial command parser (non-blocking)
// Commands: c = clear all (0x9B), '1'..'4' = clear specific ROLE index -> physical ID, r = resume RUN, ? = help
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
          //status::sendClear9B(ROLE_IDS[i]);
        }
        g_i_accum = 0.0f;  // reset integral on resume
        Serial.println("[CMD] Resume RUN (latched state cleared)");
        // Optionally re-request health snapshot after resume:
        // Request9AAll();
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

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}

  // CAN init
  Can1.begin();
  Can1.setBaudRate(CAN1_BAUD);

  pinMode(LED_R, OUTPUT);
  pinMode(LED_Y, OUTPUT);
  pinMode(LED_G, OUTPUT);

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
  delay(1000);
  // Power on all motors
  MotorStopAll();
  delay(3000);
  {
    float tmp; bool a = false, b = false;
    for(auto m : JOINTS) m->request94();
    delay(100);
    status::poll();
    if(J1.angle94(&tmp)) { g_joint_deg[0] = tmp; a = true; }
    if(J2.angle94(&tmp)) { g_joint_deg[1] = tmp; b = true; }
    if(a&&b) {
      uint8_t dir1, dir2;
      dirplan::chooseJ1(g_joint_deg[0], INIT_H_DEG, &dir1);
      dirplan::chooseJ2(g_joint_deg[1], INIT_H_DEG, &dir2);
      J1.posSingleDegMax(dir1, mapJ1::toMotorDegClamp(INIT_H_DEG), 400);
      J2.posSingleDegMax(dir2, mapJ2::toMotorDegClamp(INIT_H_DEG), 400);
      // J1.posSingleDeg(dir1, mapJ1::toMotorDegClamp(INIT_H_DEG));
      // J2.posSingleDeg(dir2, mapJ2::toMotorDegClamp(INIT_H_DEG));
      delay(2000);
      J1.posSingleDeg(dir1, mapJ1::toMotorDegClamp(INIT_H_DEG));
      J2.posSingleDeg(dir2, mapJ2::toMotorDegClamp(INIT_H_DEG));
    }
  }

  digitalWrite(LED_R, LOW);
  digitalWrite(LED_G, LOW);
  digitalWrite(LED_Y, LOW);

  Serial.println("Ready (class-based). Manual safety FSM: RUN/SAFE_HALT with latch + serial commands.");
}


//#define MOTOR_CONTROL

#define PRINT_SAVEDARRAY

//#define PRINT_ANGLE94

#define TEST_MODE


void loop() {
  status::poll();
  IMU.service();
  const uint32_t now = millis();


  {
    float tmp;
    if (J1.angle94(&tmp)) { g_joint_deg[0] = tmp; g_joint_ts[0] = now; }
    if (J2.angle94(&tmp)) { g_joint_deg[1] = tmp; g_joint_ts[1] = now; }
  }


  if (g_mode == Mode::RUN) {
    digitalWrite(LED_R, LOW);
    digitalWrite(LED_G, HIGH);


    if (now - t_ctrl >= DT_CTRL) {
      t_ctrl += DT_CTRL;

      ImuSample s;
      if (IMU.readLatest(&s)) {

        float pitch_rad   = s.pitch_rad;     // DMP (ypr[1])
        float gyroY_rad_s = s.gyroY_rad_s;   // RAW (레지스터)

        // 디버그 출력
        // Serial.printf("pitch=%.2f deg\tgyroY=%.2f deg/s\n", s.pitch_deg, s.gyroY_deg_s);

        //! PID calculation
        const float dt_s = DT_CTRL * 0.001f;          // 10 ms -> 0.01 s
        float e    = g_theta_ref - pitch_rad;   // 목표 자세 (rad)
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
        digitalWrite(LED_Y, clamp ? HIGH : LOW);

        // Serial.printf("e=%.4f rad, u_cmd=%.1f deg/s\n", e, u_cmd);
        // 실제 적용(안전 상 주석 처리). W2의 부호가 반대라면 아래처럼 반전하여 사용
        // W1.speedDps(u_cmd);
        // W2.speedDps(-u_cmd);
      }



      #ifdef MOTOR_CONTROL
//      J1.torqueAmp(2.0f);
//      J2.torqueAmp(2.0f);

       uint8_t dir, dir2;
       float   tgt_mdeg, tgt_mdeg2;
       dirplan::chooseJ1(g_joint_deg[0], 200.0f, &dir, &tgt_mdeg);
       //dirplan::chooseJ2(g_joint_deg[1], 200.0f, &dir2, &tgt_mdeg2);

       //J1.posSingleDegMax(dir, tgt_mdeg, /*max_dps*/ 200);
       J1.posSingleDeg(dir, tgt_mdeg);
       //J2.posSingleDeg(dir2, tgt_mdeg2);
      #endif
    }

    // Optional: request 0x94 periodically for joints and snapshot angles
    if (now - t_req94 >= DT_REQ94) {
      t_req94 += DT_REQ94;                 // drift‑free scheduling
      // Request 0x94 from both joints each control cadence
      for(auto m : JOINTS) m->request94();
    }

    // Optional prints
    if (now - t_print >= DT_PRINT) {
      t_print += DT_PRINT;

      #ifdef PRINT_SAVEDARRAY
      {
        const uint32_t STALE_MS = 100;
        float u;
        // J1
        if ((uint32_t)(now - g_joint_ts[0]) <= STALE_MS) {
          mapJ1::toUserDegClamp(g_joint_deg[0], &u);
          Serial.printf("[J1] angle94 = %.2f deg  =>  %.2f\n", g_joint_deg[0], u);
        } else {
          Serial.printf("[J1] angle94 = NaN deg  =>  NaN  (stale %lu ms)\n", (unsigned long)(now - g_joint_ts[0]));
        }

        // J2
        if ((uint32_t)(now - g_joint_ts[1]) <= STALE_MS) {
          mapJ2::toUserDegClamp(g_joint_deg[1], &u);
          Serial.printf("[J2] angle94 = %.2f deg  =>  %.2f\n", g_joint_deg[1], u);
        } else {
          Serial.printf("[J2] angle94 = NaN deg  =>  NaN  (stale %lu ms)\n\n", (unsigned long)(now - g_joint_ts[1]));
        }
      }
      #endif

      #ifdef PRINT_ANGLE94
      float a94, u;
      if (J1.angle94(&a94)) {
        Serial.printf("[J1] angle94 = %.2f deg", a94);
        if(mapJ1::toUserDeg(a94, &u)) {
          Serial.printf("  =>  %.2f\n", u);
        }
        else Serial.printf("\n");
      }
      else Serial.printf("[J1] XXX\n");
      if (J2.angle94(&a94)) {
        Serial.printf("[J2] angle94 = %.2f deg", a94);
        if(mapJ2::toUserDeg(a94, &u)) {
          Serial.printf("  =>  %.2f\n\n", u);
        }
        else Serial.printf("\n\n");
      }
      else Serial.printf("[J2] XXX\n\n");
      #endif
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



  checkHealthAndLatch();

  #ifdef TEST_MODE
  processSerialCommands();
  #endif

  blinkWhenSafe(now);



  delay(1);
}
