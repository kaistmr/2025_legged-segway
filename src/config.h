#pragma once
//#include <Arduino.h>
#include <cstdint>

// Direction of rotation
enum Direction : uint8_t { CW = 0, CCW = 1 };


// Motor IDs for the CAN bus
constexpr uint8_t ID_WHEEL_1 = 5;  // Left Wheel
constexpr uint8_t ID_WHEEL_2 = 4;  // Right Wheel
constexpr uint8_t ID_JOINT_1 = 1;  // Left Joint
constexpr uint8_t ID_JOINT_2 = 2;  // Right Joint


// Communication speed
constexpr uint32_t CAN1_BAUD = 1'000'000;


// LED PINs
constexpr uint8_t LED_R = 11;
constexpr uint8_t LED_Y = 10;
constexpr uint8_t LED_G = 9;



//
// ───────────────────────────────
// Joint angle mapping (no tolerance)
// Paste this block into config.h
// ───────────────────────────────
namespace anglemap_detail {
  static inline float norm360(float a) {
    while (a < 0.0f)    a += 360.0f;
    while (a >= 360.0f) a -= 360.0f;
    return a;
  }
  // Map motor single-turn angle (0..360) to raw user angle measured from START_DEG along +direction.
  static inline float rawUser(float motor_deg, float START_DEG) {
    const float a = norm360(motor_deg);
    return (a >= START_DEG) ? (a - START_DEG) : (360.0f - START_DEG + a);
  }
  // Map user angle (0..SPAN) back to motor single-turn angle (0..360).
  static inline float toMotorCore(float user_deg, float START_DEG) {
    const float CUT = 360.0f - START_DEG;       // length of [START..360)
    if (user_deg < CUT)  return norm360(START_DEG + user_deg);  // [START..360)
    else                 return norm360(user_deg - CUT);        // [0..END]
  }
}

// ── J1: START=288°, SPAN=300° ──
namespace mapJ1 {
  constexpr float START_DEG = 260.0f;
  constexpr float SPAN_DEG  = 300.0f;

  // Strict mapping: returns false if outside [0..SPAN]
  static inline bool toUserDeg(float motor_deg, float* out) {
    float u = anglemap_detail::rawUser(motor_deg, START_DEG);
    if (u < 0.0f || u > SPAN_DEG) return false;
    if (out) *out = u;
    return true;
  }

  // Always returns a value by clamping to [0..SPAN]
  static inline bool toUserDegClamp(float motor_deg, float* out) {
    float u = anglemap_detail::rawUser(motor_deg, START_DEG);
    if (u < 0.0f)      u = 0.0f;
    else if (u > SPAN_DEG) u = SPAN_DEG;
    if (out) *out = u;
    return true;
  }

  // user(0..SPAN) → motor(0..360), input is clamped to [0..SPAN]
  static inline float toMotorDegClamp(float user_deg) {
    if (user_deg < 0.0f)        user_deg = 0.0f;
    else if (user_deg > SPAN_DEG) user_deg = SPAN_DEG;
    return anglemap_detail::toMotorCore(user_deg, START_DEG);
  }

  // user를 SPAN으로 랩해서(원형) 변환하고 싶을 때
  static inline float toMotorDegWrap(float user_deg) {
    while (user_deg < 0.0f)        user_deg += SPAN_DEG;
    while (user_deg >= SPAN_DEG)   user_deg -= SPAN_DEG;
    return anglemap_detail::toMotorCore(user_deg, START_DEG);
  }
}

// ── J2: START=120°, SPAN=300° ──
namespace mapJ2 {
  constexpr float START_DEG = 120.0f;
  constexpr float SPAN_DEG  = 300.0f;

  static inline bool toUserDeg(float motor_deg, float* out) {
    float u = anglemap_detail::rawUser(motor_deg, START_DEG);
    if (u < 0.0f || u > SPAN_DEG) return false;
    if (out) *out = u;
    return true;
  }

  static inline bool toUserDegClamp(float motor_deg, float* out) {
    float u = anglemap_detail::rawUser(motor_deg, START_DEG);
    if (u < 0.0f)      u = 0.0f;
    else if (u > SPAN_DEG) u = SPAN_DEG;
    if (out) *out = u;
    return true;
  }

  static inline float toMotorDegClamp(float user_deg) {
    if (user_deg < 0.0f)        user_deg = 0.0f;
    else if (user_deg > SPAN_DEG) user_deg = SPAN_DEG;
    return anglemap_detail::toMotorCore(user_deg, START_DEG);
  }

  static inline float toMotorDegWrap(float user_deg) {
    while (user_deg < 0.0f)        user_deg += SPAN_DEG;
    while (user_deg >= SPAN_DEG)   user_deg -= SPAN_DEG;
    return anglemap_detail::toMotorCore(user_deg, START_DEG);
  }
}






// ───────────────────────────────
// Single-turn direction & target planner (uses mapJ1/mapJ2)
//  - Direction bit: 0=CW, 1=CCW (driver spec)
//  - Inputs:
//      current_motor_deg : latest 0x94 (single-turn 0..360°)
//      target_user_deg   : your user angle (0..300°). We'll clamp to [0..SPAN].
//  - Outputs:
//      *out_dir          : 0=CW, 1=CCW (shortest user-arc)
//      *out_target_motor : motor absolute angle (0..360°) to send to 0xA5/0xA6
// ───────────────────────────────
namespace dirplan {
  constexpr uint8_t CW  = 0;  // stand up
  constexpr uint8_t CCW = 1;  // sit down

  // ---- J1 (uses mapJ1::START_DEG, SPAN_DEG=300) ----
  static inline bool chooseJ1(float current_motor_deg,
                              float target_user_deg,
                              uint8_t* out_dir)
  {
    // 1) 현재 모터각 → 사용자각(0..SPAN)로 변환 (금지구간이면 경계로 클램프)
    float cur_user;

    mapJ1::toUserDegClamp(current_motor_deg, &cur_user);

    if (target_user_deg < 0.0f)            target_user_deg = 0.0f;
    else if (target_user_deg > mapJ1::SPAN_DEG) target_user_deg = mapJ1::SPAN_DEG;

    // 3) Direction by simple compare in user space: cur < target → CW, else → CCW
    uint8_t dir  = (cur_user < target_user_deg) ? CW : CCW;  // equal: CW (doesn't matter if no move)

    if (out_dir) *out_dir = dir;
    return true;
  }

  // ---- J2 (uses mapJ2::START_DEG, SPAN_DEG=300) ----
  static inline bool chooseJ2(float current_motor_deg,
                              float target_user_deg,
                              uint8_t* out_dir)
  {
    float cur_user;
    mapJ2::toUserDegClamp(current_motor_deg, &cur_user);

    if (target_user_deg < 0.0f)            target_user_deg = 0.0f;
    else if (target_user_deg > mapJ2::SPAN_DEG) target_user_deg = mapJ2::SPAN_DEG;

    uint8_t dir  = (cur_user < target_user_deg) ? CW : CCW;

    if (out_dir) *out_dir = dir;
    return true;
  }
} // namespace dirplan
