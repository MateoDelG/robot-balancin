#pragma once

#include <Arduino.h>

namespace Imu9250Calibration {

constexpr uint16_t VALID_ACCEL = 1U << 0;
constexpr uint16_t VALID_GYRO = 1U << 1;
constexpr uint16_t VALID_MAG = 1U << 2;
constexpr uint16_t VALID_VERTICAL = 1U << 3;

struct Data {
  uint32_t magic;
  uint16_t version;
  uint16_t validFlags;
  float accelOffset[3];
  float accelScale[3];
  float gyroOffset[3];
  float magOffset[3];
  float magScale[3];
  float verticalRollDeg;
  float verticalPitchDeg;
  uint32_t checksum;
};

void begin();
const Data &get();
bool isValid(uint16_t flag);
bool wasLoaded();
bool save(const Data &data);
bool clear();
Data defaults();

}  // namespace Imu9250Calibration
