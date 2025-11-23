// LkMotor.cpp
#include "LkMotor.h"

LkCanMotor::LkCanMotor(LkMotorBus &bus, uint8_t motor_id)
  : bus_(bus),
    motor_id_(motor_id),
    can_id_(uint32_t(0x140) + motor_id)   // LK: 0x140 + ID
{}

// ---------------- Power / state ----------------

bool LkCanMotor::motorOn(uint32_t timeout_ms) {
  CAN_message_t resp;
  return transact(0x88, nullptr, resp, 0x88, timeout_ms);
}

bool LkCanMotor::motorOff(uint32_t timeout_ms) {
  CAN_message_t resp;
  return transact(0x80, nullptr, resp, 0x80, timeout_ms);
}

bool LkCanMotor::motorStop(uint32_t timeout_ms) {
  CAN_message_t resp;
  return transact(0x81, nullptr, resp, 0x81, timeout_ms);
}

// ---------------- Commands ---------------------

bool LkCanMotor::setTorqueIq(int16_t iq, uint32_t timeout_ms) {
  if (iq < -2048) iq = -2048;
  if (iq >  2048) iq =  2048;

  uint8_t p[7] = {0};
  // iqControl at DATA[4..5] -> payload[3..4]
  p[3] = uint8_t(iq & 0xFF);
  p[4] = uint8_t((iq >> 8) & 0xFF);

  CAN_message_t resp;
  return transact(0xA1, p, resp, 0xA1, timeout_ms);
}

bool LkCanMotor::setSpeed(float speed_dps, uint32_t timeout_ms) {
  // 0.01 dps per LSB
  int32_t sc = (int32_t)(speed_dps * 100.0f);

  uint8_t p[7] = {0};
  // speedControl int32 at DATA[4..7] -> payload[3..6]
  p[3] = uint8_t(sc & 0xFF);
  p[4] = uint8_t((sc >> 8) & 0xFF);
  p[5] = uint8_t((sc >> 16) & 0xFF);
  p[6] = uint8_t((sc >> 24) & 0xFF);

  CAN_message_t resp;
  return transact(0xA2, p, resp, 0xA2, timeout_ms);
}

// ---------------- Position / encoder ------------

bool LkCanMotor::readMultiTurnAngleDeg(double &angle_deg,
                                       uint32_t timeout_ms) {
  CAN_message_t resp;
  if (!transact(0x92, nullptr, resp, 0x92, timeout_ms)) {
    return false;
  }

  // DATA[1..7] = motorAngle (low 7 bytes, little-endian, 0.01°/LSB)
  int64_t angle_raw = 0;
  for (int i = 0; i < 7; ++i) {
    angle_raw |= (int64_t)resp.buf[1 + i] << (8 * i);
  }

  // Optional: sign-extend bit 55 if the protocol actually uses signed 56-bit
  if (resp.buf[7] & 0x80) {
    // top bit of the 7th data byte is set → negative value
    angle_raw |= (int64_t)0xFF << 56;  // extend to full 64 bits
  }

  double deg = (double)angle_raw * 0.01;

  // Sanity clamp: if something is totally insane, bail out.
  // (Example: > +/- 1e6 deg is clearly wrong for a balancing robot.)
  if (deg > 1e6 || deg < -1e6) {
    return false;  // signal failure instead of returning a crazy value
  }

  angle_deg = deg;
  return true;
}

bool LkCanMotor::readSingleTurnAngleDeg(double &angle_deg,
                                        uint32_t timeout_ms) {
  CAN_message_t resp;
  if (!transact(0x94, nullptr, resp, 0x94, timeout_ms)) {
    return false;
  }

  // circleAngle uint32 at DATA[4..7]
  uint32_t circle = 0;
  circle |= (uint32_t)resp.buf[4] << 0;
  circle |= (uint32_t)resp.buf[5] << 8;
  circle |= (uint32_t)resp.buf[6] << 16;
  circle |= (uint32_t)resp.buf[7] << 24;

  angle_deg = (double)circle * 0.01;
  return true;
}

bool LkCanMotor::readEncoder(LkEncoderInfo &enc,
                             uint32_t timeout_ms) {
  CAN_message_t resp;
  if (!transact(0x90, nullptr, resp, 0x90, timeout_ms)) {
    return false;
  }

  enc.encoder        = (uint16_t)resp.buf[2] | ((uint16_t)resp.buf[3] << 8);
  enc.encoder_raw    = (uint16_t)resp.buf[4] | ((uint16_t)resp.buf[5] << 8);
  enc.encoder_offset = (uint16_t)resp.buf[6] | ((uint16_t)resp.buf[7] << 8);
  return true;
}

// ---------------- Status ------------------------

bool LkCanMotor::readState1(LkMotorState1 &st,
                            uint32_t timeout_ms) {
  CAN_message_t resp;
  if (!transact(0x9A, nullptr, resp, 0x9A, timeout_ms)) {
    return false;
  }

  st.temperature_c = (int8_t)resp.buf[1];
  st.voltage_x10   = (uint16_t)resp.buf[3] | ((uint16_t)resp.buf[4] << 8);
  st.error_state   = resp.buf[7];
  return true;
}

bool LkCanMotor::readState2(LkMotorState2 &st,
                            uint32_t timeout_ms) {
  CAN_message_t resp;
  if (!transact(0x9C, nullptr, resp, 0x9C, timeout_ms)) {
    return false;
  }

  st.temperature_c = (int8_t)resp.buf[1];
  st.iq_raw        = (int16_t)((uint16_t)resp.buf[2] | ((uint16_t)resp.buf[3] << 8));
  st.speed_dps     = (int16_t)((uint16_t)resp.buf[4] | ((uint16_t)resp.buf[5] << 8));
  st.encoder       = (uint16_t)resp.buf[6] | ((uint16_t)resp.buf[7] << 8);
  return true;
}

bool LkCanMotor::clearError(uint32_t timeout_ms) {
  // send 0x9B, expect reply with DATA[0] == 0x9A (state1)
  CAN_message_t resp;
  return transact(0x9B, nullptr, resp, 0x9A, timeout_ms);
}

// ---------------- transact helper ----------------

bool LkCanMotor::transact(uint8_t cmd,
                          const uint8_t payload[7],
                          CAN_message_t &resp,
                          uint8_t expectedRespCmd,
                          uint32_t timeout_ms) {
  // TX frame
  CAN_message_t tx = {};
  tx.id  = can_id_;
  tx.len = 8;
  tx.buf[0] = cmd;

  for (int i = 0; i < 7; ++i) {
    tx.buf[1 + i] = payload ? payload[i] : 0;
  }

  bus_.write(tx);

  // RX wait
  uint32_t t0 = millis();
  while (millis() - t0 < timeout_ms) {
    if (bus_.read(resp)) {
      if (resp.id == can_id_ &&
          resp.len == 8 &&
          resp.buf[0] == expectedRespCmd) {
        return true;
      }
    }
  }
  return false;  // timeout
}