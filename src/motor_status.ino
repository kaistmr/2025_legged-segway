#include "motor_status.h"
#include <FlexCAN_T4.h>
#include "config.h"
#include <cstring>
#include <cmath>
#include <cstdint>


extern FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;


namespace {

inline uint32_t status_motor_can_id(uint8_t id) { return 0x140u + id; }

// NOTE: Many firmwares reply on 0x1xx for single-motor responses. If yours uses 0x24x/0x34x,
// widen the mask below accordingly (e.g., accept 0x100/0x200/0x300).
inline bool isSingleMotorRespId(uint32_t rxid) {
  return (rxid & 0xF00u) == 0x100u;
}

// Commands we parse (first byte)
inline bool isStateCmd(uint8_t c) {
  switch (c) { case 0xA1: case 0xA2: case 0xA3: case 0xA4:
               case 0xA5: case 0xA6: case 0xA7: case 0xA8:
               case 0x9C: return true; }
  return false;
}

inline bool isHealthCmd(uint8_t c) { return c == 0x9A; }
inline bool isAngle94Cmd(uint8_t c) { return c == 0x94; }


// Encoder modulus (14→16384, 15→32768, 18→65536; using upper 16 bits for the 18-bit case)
static uint32_t g_enc_mod = 65536u; // default 18-bit

inline float rawToDeg(uint16_t raw)  { return (360.0f * (float)raw) / (float)g_enc_mod; }


} // anon



namespace status {

// ---- Cache ----
static State  g_state [256];
static Health g_health[256];


// ---- Configuration / utilities ----
void setEncoderBits(uint8_t bits){
  if      (bits == 14) g_enc_mod = 16384u;
  else if (bits == 15) g_enc_mod = 32768u;
  else if (bits == 18) g_enc_mod = 65536u;
}

void resetMultiTurn(uint8_t id, float preset_deg){
    (void)id; (void)preset_deg; // multi-turn accumulation removed; no-op
}

// ---- TX helpers (manual senders only) ----
void sendRead9A(uint8_t id){
  CAN_message_t m{}; m.id = status_motor_can_id(id); m.len = 8;
  for (int i=0;i<8;++i) m.buf[i]=0;
  m.buf[0] = 0x9A;
  Can1.write(m);
}
void sendRead9C(uint8_t id){
  CAN_message_t m{}; m.id = status_motor_can_id(id); m.len = 8;
  for (int i=0;i<8;++i) m.buf[i]=0;
  m.buf[0] = 0x9C;
  Can1.write(m);
}

void sendRead94(uint8_t id){
  CAN_message_t m{}; m.id = status_motor_can_id(id); m.len = 8;
  for (int i=0;i<8;++i) m.buf[i]=0;
  m.buf[0] = 0x94;
  Can1.write(m);
}

void sendClear9B(uint8_t id){
  CAN_message_t m{}; m.id = status_motor_can_id(id); m.len = 8;
  for (int i=0;i<8;++i) m.buf[i]=0;
  m.buf[0] = 0x9B;
  Can1.write(m);
}

// ---- RX / parsing / cache ----
static inline void parseStateFromFrame(const CAN_message_t& rx){
  // Assume reply base 0x140: physical ID = rx.id - 0x140
  const uint8_t id = static_cast<uint8_t>(rx.id - 0x140u);
  State& s = g_state[id];

  s.temperature = static_cast<int8_t>(rx.buf[1]);
  s.iq_counts   = static_cast<int16_t>( (int16_t)rx.buf[2] | ((int16_t)rx.buf[3] << 8) );
  s.speed_dps   = static_cast<int16_t>( (int16_t)rx.buf[4] | ((int16_t)rx.buf[5] << 8) );
  s.encoder_raw = static_cast<uint16_t>( (uint16_t)rx.buf[6] | ((uint16_t)rx.buf[7] << 8) );
  s.stamp_ms    = millis();
  s.valid       = true;

}

static inline void parseHealthFromFrame(const CAN_message_t& rx){
  const uint8_t id = static_cast<uint8_t>(rx.id - 0x140u);
  Health& h = g_health[id];

  h.temperature = static_cast<int8_t>(rx.buf[1]);

  // Bus voltage (0.1 V/LSB). Byte layout may vary across boards/firmware.
  // Commonly [2..3] is used. If your documentation differs, switch to [3..4].
  h.bus_dV = (uint16_t)((uint16_t)rx.buf[3] | ((uint16_t)rx.buf[4] << 8));

  h.error   = rx.buf[7];     // errorState bit flags
  h.stamp_ms= millis();
  h.valid   = true;
}

static inline void parseAngle94FromFrame(const CAN_message_t& rx){
  const uint8_t id = static_cast<uint8_t>(rx.id - 0x140u);
  State& s = g_state[id];
  // 0x94: single-turn circle angle in 0.01°/LSB. Typical layout uses DATA[4..7].
  uint32_t a0p01 = (uint32_t)rx.buf[4]
                  | ((uint32_t)rx.buf[5] << 8)
                  | ((uint32_t)rx.buf[6] << 16)
                  | ((uint32_t)rx.buf[7] << 24);
  s.circle_0p01    = a0p01;
  s.have_circle_94 = true;
  s.stamp_ms       = millis();
  //! s.valid          = true; // keep marked valid once we have any state
}


void poll(){
  CAN_message_t rx;
  while (Can1.read(rx)) {
    if (rx.len != 8) continue;
    if (!isSingleMotorRespId(rx.id)) continue;

    const uint8_t cmd = rx.buf[0];
    if      (isStateCmd(cmd)) parseStateFromFrame(rx);
    else if (isHealthCmd(cmd)) parseHealthFromFrame(rx);
    else if (isAngle94Cmd(cmd)) parseAngle94FromFrame(rx);
    // (Extensions: 0x9D, multi-motor responses, etc.)
  }
}

// ---- Queries / getters ----
bool getState(uint8_t id, State* out){
  if (!out) return false;
  const State& s = g_state[id];
  if (!s.valid) return false;
  *out = s; return true;
}

bool getHealth(uint8_t id, Health* out){
  if (!out) return false;
  const Health& h = g_health[id];
  if (!h.valid) return false;
  *out = h; return true;
}

bool getAngles(uint8_t id, float* single_deg, /*float* multi_deg,*/ float* enc_raw){
  const State& s = g_state[id];
  if (!s.valid) return false;
  if (single_deg) *single_deg = rawToDeg(s.encoder_raw);
  //if (multi_deg ) *multi_deg  = NAN; // multi-turn accumulation removed
  if (enc_raw   ) *enc_raw    = s.encoder_raw;
  return true;
}

bool getAngle94Deg(uint8_t id, float* deg){
  const State& s = g_state[id];
  if (!s.have_circle_94) return false; // angle from 0x94 is independent of 0x9C state
  if (deg) *deg = (float)s.circle_0p01 * 0.01f;
  return true;
}


} // namespace status