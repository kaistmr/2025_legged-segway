#pragma once

#include <Arduino.h>
#include "motor_control.h" // A1~A8: send torque/speed/position commands
#include "motor_status.h"  // poll(), 0x9A/9B/9C, cached state/health access
#include <cstdint>

// Thin wrapper: holds the motor ID and cleanly forwards to global functions.
// No automatic stop/clear actions (manual safety policy).
class Motor {
public:
  explicit Motor(uint8_t id, const char* name = nullptr) : id_(id), name_(name) {}

  // Identification
  uint8_t     id()   const { return id_; }
  const char* name() const { return name_; }

  // ---- Power / stop ----
  void on()   const { motorOn(id_);  }
  void stop() const { motorStop(id_);}

  // ---- Control commands (TX only) ----
  void torqueAmp(float amp)                               const { setTorqueAmp(id_, amp); }
  void speedDps(float dps)                                const { setSpeedDps(id_, dps); }
  void posMultiDeg(float deg)                             const { setPosMultiDeg(id_, deg); }
  void posMultiDegMax(float deg, uint16_t max_dps)        const { setPosMultiDeg_withMaxSpeed(id_, deg, max_dps); }
  void posSingleDeg(uint8_t dir, float deg)               const { setPosSingleDeg(id_, dir, deg); }                 // dir: 0=CW, 1=CCW
  void posSingleDegMax(uint8_t dir, float deg, uint16_t max_dps) const { setPosSingleDeg_withMaxSpeed(id_, dir, deg, max_dps); }
  void moveIncDeg(float delta_deg)                        const { moveIncrementDeg(id_, delta_deg); }
  void moveIncDegMax(float delta_deg, uint16_t max_dps)   const { moveIncrementDeg_withMaxSpeed(id_, delta_deg, max_dps); }

  // ---- Manual read / clear senders ----
  void request9A() const { status::sendRead9A(id_); }   // temperature / bus voltage / error
  void request9C() const { status::sendRead9C(id_); }   // temperature / iq / speed / encoder
  void request94() const { status::sendRead94(id_); }   // single-turn angle (0x94) request
  void clear9B()   const { status::sendClear9B(id_); }  // clear errors (manual only)

  // ---- Cached state / health / angles (non-blocking) ----
  bool state(status::State& out)        const { return status::getState (id_, &out); }
  bool health(status::Health& out)      const { return status::getHealth(id_, &out); }
  bool angles(float* single_deg, /*float* multi_deg,*/ float* enc_raw) const { return status::getAngles(id_, single_deg, /*multi_deg,*/ enc_raw); }
  bool angle94(float* deg) const { return status::getAngle94Deg(id_, deg); }

private:
  uint8_t     id_;
  const char* name_;
};
