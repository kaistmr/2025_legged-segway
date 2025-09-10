#pragma once

#include <Arduino.h>
#include <cstdint>

// motor constant
float motor_diameter = 0.065f

// Power/Status
void motorOn(uint8_t id);
void motorStop(uint8_t id);
void motorOnAll();
void motorStopAll();

// Torque/Speed Control commands
void setTorqueAmp(uint8_t id, float ampA); // 0xA1
void setSpeedDps(uint8_t id, float dps);   // 0xA2

// Position Control commands
void setPosMultiDeg(uint8_t id, float target_deg);  // 0xA3
void setPosMultiDeg_withMaxSpeed(uint8_t id, float target_deg, uint16_t maxSpeed_dps); // 0xA4

void setPosSingleDeg(uint8_t id, uint8_t spinDirection, float target_deg);  // 0xA5
void setPosSingleDeg_withMaxSpeed(uint8_t id, uint8_t spinDirection, float target_deg, uint16_t maxSpeed_dps); // 0xA6

void moveIncrementDeg(uint8_t id, float delta_deg);  // 0xA7
void moveIncrementDeg_withMaxSpeed(uint8_t id, float delta_deg, uint16_t maxSpeed_dps); // 0xA8

