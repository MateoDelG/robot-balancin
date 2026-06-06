#pragma once

#include <Arduino.h>

namespace Imu6500Test {

struct ImuSample {
  float rawAx = 0.0f;
  float rawAy = 0.0f;
  float rawAz = 0.0f;
  float rawGx = 0.0f;
  float rawGy = 0.0f;
  float rawGz = 0.0f;
  float correctedGx = 0.0f;
  float correctedGy = 0.0f;
  float correctedGz = 0.0f;
  float gyroOffsetX = 0.0f;
  float gyroOffsetY = 0.0f;
  float gyroOffsetZ = 0.0f;
  float angleAccelDeg = 0.0f;
  float angleFilteredDeg = 0.0f;
  float angleComplementaryDeg = 0.0f;
  float angleKalmanDeg = 0.0f;
  float selectedAngleDeg = 0.0f;
  float angleVerticalOffsetDeg = 0.0f;
  float gyroRateDegPerSec = 0.0f;
  float temperatureC = 0.0f;
  bool gyroCalibrated = false;
  bool angleInitialized = false;
  bool useKalmanAngle = false;
  float kalmanQAngle = 0.0f;
  float kalmanQBias = 0.0f;
  float kalmanRMeasure = 0.0f;
};

bool begin();
bool calibrateGyro();
void calibrateVerticalAngle();
bool update();
ImuSample getSample();
void printSampleIfDue();
bool isReady();
uint8_t detectedAddress();
void setFilterAlpha(float alpha);
float getFilterAlpha();
float getVerticalOffsetDeg();
void setAngleFilterMode(bool useKalman);
bool isUsingKalmanAngle();
void setKalmanParameters(float qAngle, float qBias, float rMeasure);
float getKalmanQAngle();
float getKalmanQBias();
float getKalmanRMeasure();
void resetKalman();

}  // namespace Imu6500Test
