#pragma once

#include <cstdint>
#include <cstdbool>


namespace status {

struct State {
  int8_t   temperature;
  int16_t  iq_counts;
  int16_t  speed_dps;
  uint16_t encoder_raw;
  uint32_t circle_0p01;    // from 0x94: single-turn angle in 0.01° LSB
  bool     have_circle_94; // true if circle_0p01 is valid
  uint32_t stamp_ms;
  bool     valid;
};

struct Health {
  int8_t   temperature;
  uint16_t bus_dV;
  uint8_t  error;
  uint32_t stamp_ms;
  bool     valid;
};

// ---- Configuration / utilities ----
// multi-turn accumulation is deprecated/removed; resetMultiTurn is now a no-op for compatibility
void resetMultiTurn(uint8_t id, float preset_deg);

void setEncoderBits(uint8_t bits);

// ---- TX helpers (manual senders) ----
void sendRead9A(uint8_t id);
void sendRead9C(uint8_t id);
void sendRead94(uint8_t id);
void sendClear9B(uint8_t id);

// ---- RX / parsing / cache ----
void poll();

// ---- Queries / getters ----
bool getState(uint8_t id, State* out);
bool getHealth(uint8_t id, Health* out);
bool getAngles(uint8_t id, float* single_deg, /*float* multi_deg,*/ float* enc_raw);
bool getAngle94Deg(uint8_t id, float* deg);

} // namespace status
