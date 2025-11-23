// LkMotor.h
#pragma once

#include <Arduino.h>
#include <FlexCAN_T4.h>

// Alias for the Teensy 4.x CAN1 bus type used for LK motors
using LkMotorBus = FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>;

// --- Status structs -------------------------------------------------

struct LkMotorState1 {
  int8_t   temperature_c;   // °C
  uint16_t voltage_x10;     // 0.1 V per LSB (245 -> 24.5 V)
  uint8_t  error_state;     // bitfield from 0x9A
};

struct LkMotorState2 {
  int8_t   temperature_c;   // °C
  int16_t  iq_raw;          // -2048..2048 (torque current)
  int16_t  speed_dps;       // 1 dps per LSB
  uint16_t encoder;         // 0..16383
};

struct LkEncoderInfo {
  uint16_t encoder;         // offset-corrected
  uint16_t encoder_raw;
  uint16_t encoder_offset;
};

// --- Driver class ---------------------------------------------------

class LkCanMotor {
public:
  // bus: reference to your global CAN1 object
  // motor_id: LK motor CAN ID (1..32)
  LkCanMotor(LkMotorBus &bus, uint8_t motor_id);

  // Power / state
  bool motorOn(uint32_t timeout_ms = 5);
  bool motorOff(uint32_t timeout_ms = 5);
  bool motorStop(uint32_t timeout_ms = 5);

  // Commands
  bool setTorqueIq(int16_t iq, uint32_t timeout_ms = 5);  // -2048..2048
  bool setSpeed(float speed_dps, uint32_t timeout_ms = 5); // closed-loop speed

  // Position / encoder
  bool readMultiTurnAngleDeg(double &angle_deg, uint32_t timeout_ms = 10);
  bool readSingleTurnAngleDeg(double &angle_deg, uint32_t timeout_ms = 10);
  bool readEncoder(LkEncoderInfo &enc, uint32_t timeout_ms = 10);

  // Status
  bool readState1(LkMotorState1 &st, uint32_t timeout_ms = 10);
  bool readState2(LkMotorState2 &st, uint32_t timeout_ms = 10);
  bool clearError(uint32_t timeout_ms = 10);

  uint8_t  id()    const { return motor_id_; }
  uint32_t canId() const { return can_id_;   }

private:
  LkMotorBus &bus_;
  uint8_t  motor_id_;
  uint32_t can_id_;

  // generic request/response helper
  bool transact(uint8_t cmd,
                const uint8_t payload[7],   // pass nullptr for all zeros
                CAN_message_t &resp,
                uint8_t expectedRespCmd,
                uint32_t timeout_ms);
};