#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <cmath>            // lroundf, llround
#include <cstdint>
#include "motor_control.h"
#include "config.h"

extern FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;



// ---------------- Utils ----------------
static inline uint32_t motor_can_id(uint8_t id) { return 0x140u + id; }

static inline void pack_u16(uint8_t *dst, uint16_t v) {
  dst[0] = static_cast<uint8_t>(v & 0xFF);
  dst[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}
static inline void pack_i16(uint8_t *dst, int16_t v) { pack_u16(dst, (uint16_t)v); }

static inline void pack_i32(uint8_t *dst, int32_t v) {
  dst[0] = static_cast<uint8_t>( v        & 0xFF);
  dst[1] = static_cast<uint8_t>((v >> 8 ) & 0xFF);
  dst[2] = static_cast<uint8_t>((v >> 16) & 0xFF);
  dst[3] = static_cast<uint8_t>((v >> 24) & 0xFF);
}
static inline void pack_u32(uint8_t *dst, uint32_t v) { pack_i32(dst, (int32_t)v); }

static inline void make_frame(CAN_message_t &m, uint8_t id, uint8_t cmd) {
  m.id  = motor_can_id(id);
  m.len = 8;
  for (int i=0;i<8;i++) m.buf[i] = 0;
  m.buf[0] = cmd;
}

// ---------------- Conversion (Scale) ----------------
// MF series: ±16.5A <-> ±2048 (0xA1)
static inline int16_t amp_to_iq(float ampA) {
  const float scale = 2048.0f / 16.5f;
  long iq = lroundf(ampA * scale);
  if (iq >  2048) iq =  2048;
  if (iq < -2048) iq = -2048;
  return (int16_t)iq;
}
// Speed (dps) <-> raw (0.01 dps/LSB)
static inline int32_t dps_to_raw(float dps) { return (int32_t)llround((double)dps * 100.0); }
// Angle (deg) <-> raw (0.01 deg/LSB)
static inline int32_t deg_to_raw_0p01(float deg) { return (int32_t)llround((double)deg * 100.0); }

// ---------------- Status: On/Stop ----------------
void motorOn(uint8_t id) {
  CAN_message_t m{};
  make_frame(m, id, 0x88);
  Can1.write(m);
}
void motorStop(uint8_t id) {
  CAN_message_t m{};
  make_frame(m, id, 0x81);
  Can1.write(m);
}
void motorOnAll() {
  motorOn(ID_WHEEL_1);
  motorOn(ID_WHEEL_2);
  motorOn(ID_JOINT_1);
  motorOn(ID_JOINT_2);
}
void motorStopAll() {
  motorStop(ID_WHEEL_1);
  motorStop(ID_WHEEL_2);
  motorStop(ID_JOINT_1);
  motorStop(ID_JOINT_2);
}

// ---------------- Torque control (0xA1) ----------------
void setTorqueAmp(uint8_t id, float ampA) {
  CAN_message_t m{};
  make_frame(m, id, 0xA1);
  int16_t iq = amp_to_iq(ampA);
  pack_i16(&m.buf[4], iq);
  Can1.write(m);
}

// ---------------- Speed control (0xA2; 0.01 dps/LSB) ----------------
void setSpeedDps(uint8_t id, float dps) {
  CAN_message_t m{};
  make_frame(m, id, 0xA2);
  int32_t sp = dps_to_raw(dps);
  pack_i32(&m.buf[4], sp);
  Can1.write(m);
}

// ---------------- Position control ----------------
// (A) Multi-turn absolute angle: 0xA3 (angle only), 0xA4 (max speed + angle)
//   Angle raw unit: 0.01°/LSB (int32)
//   maxSpeed unit for 0xA4: 1 dps/LSB (uint16)
void setPosMultiDeg(uint8_t id, float target_deg) {
  CAN_message_t m{};
  make_frame(m, id, 0xA3);
  int32_t a = deg_to_raw_0p01(target_deg);
  pack_i32(&m.buf[4], a);
  Can1.write(m);
}
void setPosMultiDeg_withMaxSpeed(uint8_t id, float target_deg, uint16_t maxSpeed_dps) {
  CAN_message_t m{};
  make_frame(m, id, 0xA4);
  pack_u16(&m.buf[2], maxSpeed_dps);
  int32_t a = deg_to_raw_0p01(target_deg);
  pack_i32(&m.buf[4], a);
  Can1.write(m);
}

// (B) Single-turn absolute angle: 0xA5 (direction + angle), 0xA6 (direction + max speed + angle)
//   Recommended range 0~360°. Angle raw: 0.01°/LSB (uint32)
//   spinDirection: 0=CW, 1=CCW
void setPosSingleDeg(uint8_t id, uint8_t spinDirection, float target_deg) {
  if (target_deg < 0) target_deg = 0;
  if (target_deg > 360.0f) target_deg = 360.0f;

  CAN_message_t m{};
  make_frame(m, id, 0xA5);
  m.buf[1] = spinDirection;
  uint32_t a = (uint32_t)llround(target_deg * 100.0);
  pack_u32(&m.buf[4], a);
  Can1.write(m);
}
void setPosSingleDeg_withMaxSpeed(uint8_t id, uint8_t spinDirection, float target_deg, uint16_t maxSpeed_dps) {
  if (target_deg < 0) target_deg = 0;
  if (target_deg > 360.0f) target_deg = 360.0f;

  CAN_message_t m{};
  make_frame(m, id, 0xA6);
  m.buf[1] = spinDirection;
  pack_u16(&m.buf[2], maxSpeed_dps);
  uint32_t a = (uint32_t)llround(target_deg * 100.0);
  pack_u32(&m.buf[4], a);
  Can1.write(m);
}

// (C) Incremental (relative) angle: 0xA7 (angle increment), 0xA8 (max speed + angle increment)
//   Angle increment raw: 0.01°/LSB (int32), sign is direction
void moveIncrementDeg(uint8_t id, float delta_deg) {
  CAN_message_t m{};
  make_frame(m, id, 0xA7);
  int32_t inc = deg_to_raw_0p01(delta_deg);
  pack_i32(&m.buf[4], inc);
  Can1.write(m);
}
void moveIncrementDeg_withMaxSpeed(uint8_t id, float delta_deg, uint16_t maxSpeed_dps) {
  CAN_message_t m{};
  make_frame(m, id, 0xA8);
  pack_u16(&m.buf[2], maxSpeed_dps);
  int32_t inc = deg_to_raw_0p01(delta_deg);
  pack_i32(&m.buf[4], inc);
  Can1.write(m);
}
