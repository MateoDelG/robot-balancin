#pragma once

#include <Arduino.h>

namespace Imu9250 {

struct RawSample {
  float axG = 0.0f;
  float ayG = 0.0f;
  float azG = 0.0f;
  float accelNormG = 0.0f;
  float gxDps = 0.0f;
  float gyDps = 0.0f;
  float gzDps = 0.0f;
  float mx = 0.0f;
  float my = 0.0f;
  float mz = 0.0f;
  float magneticDirectionDeg = 0.0f;

  float correctedAxG = 0.0f;
  float correctedAyG = 0.0f;
  float correctedAzG = 0.0f;
  float correctedGxDps = 0.0f;
  float correctedGyDps = 0.0f;
  float correctedGzDps = 0.0f;
  float correctedMxUt = 0.0f;
  float correctedMyUt = 0.0f;
  float correctedMzUt = 0.0f;
  float magneticNormUt = 0.0f;

  float accelRollDeg = 0.0f;
  float accelPitchDeg = 0.0f;
  float filteredRollDeg = 0.0f;
  float filteredPitchDeg = 0.0f;
  float relativeRollDeg = 0.0f;
  float relativePitchDeg = 0.0f;
  float headingDeg = 0.0f;
  float filterAlpha = 0.98f;

  float accelOffset[3] = {};
  float accelScale[3] = {1.0f, 1.0f, 1.0f};
  float gyroOffset[3] = {};
  float magOffset[3] = {};
  float magScale[3] = {1.0f, 1.0f, 1.0f};
  float verticalRollDeg = 0.0f;
  float verticalPitchDeg = 0.0f;

  uint16_t sampleRateHz = 0;
  uint16_t accelRateHz = 0;
  uint16_t gyroRateHz = 0;
  uint16_t magRateHz = 0;
  uint32_t calibrationSamples = 0;
  unsigned long lastSampleMs = 0;
  uint8_t address = 0;
  uint8_t imuId = 0;
  uint8_t magnetometerId = 0;
  uint8_t accelPoseIndex = 0;
  bool accelReady = false;
  bool gyroReady = false;
  bool magnetometerReady = false;
  bool filterReady = false;
  bool calibrationStored = false;
  bool accelCalibrated = false;
  bool gyroCalibrated = false;
  bool magCalibrated = false;
  bool verticalCalibrated = false;
  bool accelWizardActive = false;
  char calibrationMode[16] = "idle";
  char calibrationStatus[96] = "Boot";
  char accelPoseName[24] = "--";
};

bool begin();
void update();
RawSample getSample();
bool isReady();

bool startGyroCalibration();
bool startAccelCalibration();
bool captureAccelPose();
bool startMagCalibration();
bool saveVertical();
bool clearCalibration();

}  // namespace Imu9250
