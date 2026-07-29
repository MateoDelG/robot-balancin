#include "imu9250.h"

#include <MPU9250_asukiaaa.h>
#include <Wire.h>
#include <math.h>
#include <string.h>

#include "../../include/config.h"
#include "imu9250_calibration.h"
#include "imu9250_filter_settings.h"

namespace {

constexpr uint8_t AK8963_ADDRESS = 0x0C;
constexpr uint8_t AK8963_WHO_AM_I = 0x00;
constexpr uint8_t MAG_MODE_100HZ_16BIT = MAG_MODE_CONTINUOUS_100HZ | 0x10;
constexpr float MAG_16BIT_UT_PER_COUNT = 0.15f;
constexpr float RAD_TO_DEG_F = 57.2957795f;
constexpr float DEG_TO_RAD_F = 0.0174532925f;
constexpr uint8_t ACCEL_POSE_COUNT = 6;

const char *ACCEL_POSE_NAMES[ACCEL_POSE_COUNT] = {
    "+X arriba", "-X arriba", "+Y arriba", "-Y arriba", "+Z arriba", "-Z arriba"};

enum class CalibrationMode { None, Gyro, AccelPose, Magnetometer };

MPU9250_asukiaaa imuAt68(MPU9250_ADDRESS_AD0_LOW);
MPU9250_asukiaaa imuAt69(MPU9250_ADDRESS_AD0_HIGH);
MPU9250_asukiaaa *imu = nullptr;
Imu9250::RawSample latestSample;
CalibrationMode calibrationMode = CalibrationMode::None;

bool freshAccel = false;
bool freshGyro = false;
bool freshMag = false;
bool filterInitialized = false;
float filterAlpha = Config::SENSOR_COMPLEMENTARY_ALPHA;
unsigned long lastFilterUs = 0;
unsigned long calibrationStartMs = 0;
unsigned long lastMagCalibrationSampleMs = 0;
uint32_t calibrationSamples = 0;
double sampleSum[3] = {};
double sampleSquareSum[3] = {};
float accelPoseMean[ACCEL_POSE_COUNT][3] = {};
float magMinimum[3] = {};
float magMaximum[3] = {};

uint32_t accelReads = 0;
uint32_t gyroReads = 0;
uint32_t magReads = 0;
uint32_t previousAccelReads = 0;
uint32_t previousGyroReads = 0;
uint32_t previousMagReads = 0;
unsigned long previousRateMs = 0;

void setStatus(const String &status) {
  strlcpy(latestSample.calibrationStatus, status.c_str(), sizeof(latestSample.calibrationStatus));
}

void setMode(const char *mode) {
  strlcpy(latestSample.calibrationMode, mode, sizeof(latestSample.calibrationMode));
}

uint8_t readRegister(uint8_t address, uint8_t reg) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(address, static_cast<uint8_t>(1)) != 1) return 0;
  return Wire.read();
}

void selectImu() {
  uint8_t idAt68 = 0;
  uint8_t idAt69 = 0;
  const bool foundAt68 = imuAt68.readId(&idAt68) == 0;
  const bool foundAt69 = imuAt69.readId(&idAt69) == 0;
  if (foundAt68 && (idAt68 == 0x71 || idAt68 == 0x73)) {
    imu = &imuAt68;
    latestSample.address = MPU9250_ADDRESS_AD0_LOW;
    latestSample.imuId = idAt68;
  } else if (foundAt69 && (idAt69 == 0x71 || idAt69 == 0x73)) {
    imu = &imuAt69;
    latestSample.address = MPU9250_ADDRESS_AD0_HIGH;
    latestSample.imuId = idAt69;
  }
}

void copyCalibrationToSample() {
  const Imu9250Calibration::Data &data = Imu9250Calibration::get();
  for (uint8_t axis = 0; axis < 3; ++axis) {
    latestSample.accelOffset[axis] = data.accelOffset[axis];
    latestSample.accelScale[axis] = data.accelScale[axis];
    latestSample.gyroOffset[axis] = data.gyroOffset[axis];
    latestSample.magOffset[axis] = data.magOffset[axis];
    latestSample.magScale[axis] = data.magScale[axis];
  }
  latestSample.verticalRollDeg = data.verticalRollDeg;
  latestSample.verticalPitchDeg = data.verticalPitchDeg;
  latestSample.calibrationStored = Imu9250Calibration::wasLoaded();
  latestSample.accelCalibrated = Imu9250Calibration::isValid(Imu9250Calibration::VALID_ACCEL);
  latestSample.gyroCalibrated = Imu9250Calibration::isValid(Imu9250Calibration::VALID_GYRO);
  latestSample.magCalibrated = Imu9250Calibration::isValid(Imu9250Calibration::VALID_MAG);
  latestSample.verticalCalibrated = Imu9250Calibration::isValid(Imu9250Calibration::VALID_VERTICAL);
}

void applyCalibration() {
  const Imu9250Calibration::Data &data = Imu9250Calibration::get();
  latestSample.correctedAxG = (latestSample.axG - data.accelOffset[0]) * data.accelScale[0];
  latestSample.correctedAyG = (latestSample.ayG - data.accelOffset[1]) * data.accelScale[1];
  latestSample.correctedAzG = (latestSample.azG - data.accelOffset[2]) * data.accelScale[2];
  latestSample.correctedGxDps = latestSample.gxDps - data.gyroOffset[0];
  latestSample.correctedGyDps = latestSample.gyDps - data.gyroOffset[1];
  latestSample.correctedGzDps = latestSample.gzDps - data.gyroOffset[2];
  latestSample.correctedMxUt = (latestSample.mx - data.magOffset[0]) * data.magScale[0] * MAG_16BIT_UT_PER_COUNT;
  latestSample.correctedMyUt = (latestSample.my - data.magOffset[1]) * data.magScale[1] * MAG_16BIT_UT_PER_COUNT;
  latestSample.correctedMzUt = (latestSample.mz - data.magOffset[2]) * data.magScale[2] * MAG_16BIT_UT_PER_COUNT;
  latestSample.accelNormG = sqrtf(latestSample.correctedAxG * latestSample.correctedAxG +
                                  latestSample.correctedAyG * latestSample.correctedAyG +
                                  latestSample.correctedAzG * latestSample.correctedAzG);
  latestSample.magneticNormUt = sqrtf(latestSample.correctedMxUt * latestSample.correctedMxUt +
                                      latestSample.correctedMyUt * latestSample.correctedMyUt +
                                      latestSample.correctedMzUt * latestSample.correctedMzUt);
}

float wrapAngle180(float angleDeg) {
  while (angleDeg > 180.0f) angleDeg -= 360.0f;
  while (angleDeg <= -180.0f) angleDeg += 360.0f;
  return angleDeg;
}

float unwrapAngleNear(float angleDeg, float referenceDeg) {
  while (angleDeg - referenceDeg > 180.0f) angleDeg -= 360.0f;
  while (angleDeg - referenceDeg < -180.0f) angleDeg += 360.0f;
  return angleDeg;
}

void updateOrientation() {
  if (!latestSample.accelReady || !latestSample.gyroReady) return;
  latestSample.accelRollDeg = atan2f(latestSample.correctedAyG, latestSample.correctedAzG) * RAD_TO_DEG_F;
  latestSample.accelPitchDeg = atan2f(-latestSample.correctedAxG,
      sqrtf(latestSample.correctedAyG * latestSample.correctedAyG + latestSample.correctedAzG * latestSample.correctedAzG)) * RAD_TO_DEG_F;
  const unsigned long nowUs = micros();
  if (!filterInitialized) {
    latestSample.filteredRollDeg = latestSample.accelRollDeg;
    latestSample.filteredPitchDeg = latestSample.accelPitchDeg;
    filterInitialized = true;
  } else {
    const float dt = constrain((nowUs - lastFilterUs) / 1000000.0f, 0.0005f, 0.05f);
    const float continuousAccelRoll = unwrapAngleNear(latestSample.accelRollDeg, latestSample.filteredRollDeg);
    latestSample.filteredRollDeg = filterAlpha *
        (latestSample.filteredRollDeg + latestSample.correctedGxDps * dt) +
        (1.0f - filterAlpha) * continuousAccelRoll;
    latestSample.filteredPitchDeg = filterAlpha *
        (latestSample.filteredPitchDeg + latestSample.correctedGyDps * dt) +
        (1.0f - filterAlpha) * latestSample.accelPitchDeg;
  }
  lastFilterUs = nowUs;
  latestSample.filterReady = true;
  const Imu9250Calibration::Data &data = Imu9250Calibration::get();
  latestSample.relativeRollDeg = wrapAngle180(latestSample.filteredRollDeg - data.verticalRollDeg);
  latestSample.relativePitchDeg = wrapAngle180(latestSample.filteredPitchDeg - data.verticalPitchDeg);

  if (latestSample.magnetometerReady) {
    const float roll = latestSample.filteredRollDeg * DEG_TO_RAD_F;
    const float pitch = latestSample.filteredPitchDeg * DEG_TO_RAD_F;
    const float horizontalX = latestSample.correctedMxUt * cosf(pitch) + latestSample.correctedMzUt * sinf(pitch);
    const float horizontalY = latestSample.correctedMxUt * sinf(roll) * sinf(pitch) +
                              latestSample.correctedMyUt * cosf(roll) -
                              latestSample.correctedMzUt * sinf(roll) * cosf(pitch);
    latestSample.headingDeg = atan2f(horizontalY, horizontalX) * RAD_TO_DEG_F;
    if (latestSample.headingDeg < 0.0f) latestSample.headingDeg += 360.0f;
    latestSample.magneticDirectionDeg = latestSample.headingDeg;
  }
}

void resetAccumulator() {
  calibrationSamples = 0;
  latestSample.calibrationSamples = 0;
  for (uint8_t axis = 0; axis < 3; ++axis) sampleSum[axis] = sampleSquareSum[axis] = 0.0;
}

void finishGyroCalibration() {
  calibrationMode = CalibrationMode::None;
  setMode("idle");
  if (calibrationSamples < 100) { setStatus("Gyro FAIL: pocas muestras"); return; }
  float mean[3];
  float maximumDeviation = 0.0f;
  for (uint8_t axis = 0; axis < 3; ++axis) {
    mean[axis] = sampleSum[axis] / calibrationSamples;
    const float variance = max(0.0, sampleSquareSum[axis] / calibrationSamples - mean[axis] * mean[axis]);
    maximumDeviation = max(maximumDeviation, sqrtf(variance));
  }
  if (maximumDeviation > 1.5f) { setStatus("Gyro FAIL: sensor en movimiento"); return; }
  Imu9250Calibration::Data data = Imu9250Calibration::get();
  for (uint8_t axis = 0; axis < 3; ++axis) data.gyroOffset[axis] = mean[axis];
  data.validFlags |= Imu9250Calibration::VALID_GYRO;
  setStatus(Imu9250Calibration::save(data) ? "Gyro PASS y guardado" : "Gyro PASS, error NVS");
  copyCalibrationToSample();
}

bool accelPoseIsValid(const float mean[3]) {
  const uint8_t axis = latestSample.accelPoseIndex / 2;
  const float expectedSign = latestSample.accelPoseIndex % 2 == 0 ? 1.0f : -1.0f;
  if (mean[axis] * expectedSign < 0.7f) return false;
  for (uint8_t other = 0; other < 3; ++other) if (other != axis && fabsf(mean[other]) > 0.55f) return false;
  return true;
}

void finishAccelPose() {
  calibrationMode = CalibrationMode::None;
  setMode("idle");
  if (calibrationSamples < 100) { setStatus("Accel FAIL: pocas muestras"); return; }
  float mean[3];
  for (uint8_t axis = 0; axis < 3; ++axis) mean[axis] = sampleSum[axis] / calibrationSamples;
  if (!accelPoseIsValid(mean)) {
    setStatus(String("Posicion incorrecta: ") + ACCEL_POSE_NAMES[latestSample.accelPoseIndex]);
    return;
  }
  for (uint8_t axis = 0; axis < 3; ++axis) accelPoseMean[latestSample.accelPoseIndex][axis] = mean[axis];
  ++latestSample.accelPoseIndex;
  if (latestSample.accelPoseIndex < ACCEL_POSE_COUNT) {
    strlcpy(latestSample.accelPoseName, ACCEL_POSE_NAMES[latestSample.accelPoseIndex], sizeof(latestSample.accelPoseName));
    setStatus(String("Siguiente: ") + latestSample.accelPoseName);
    return;
  }
  Imu9250Calibration::Data data = Imu9250Calibration::get();
  for (uint8_t axis = 0; axis < 3; ++axis) {
    const float positive = accelPoseMean[axis * 2][axis];
    const float negative = accelPoseMean[axis * 2 + 1][axis];
    const float span = positive - negative;
    if (span < 1.5f) { latestSample.accelWizardActive = false; setStatus("Accel FAIL: rango insuficiente"); return; }
    data.accelOffset[axis] = (positive + negative) * 0.5f;
    data.accelScale[axis] = 2.0f / span;
  }
  data.validFlags |= Imu9250Calibration::VALID_ACCEL;
  latestSample.accelWizardActive = false;
  strlcpy(latestSample.accelPoseName, "--", sizeof(latestSample.accelPoseName));
  setStatus(Imu9250Calibration::save(data) ? "Acelerometro PASS y guardado" : "Acelerometro PASS, error NVS");
  copyCalibrationToSample();
}

void finishMagCalibration() {
  calibrationMode = CalibrationMode::None;
  setMode("idle");
  float halfRange[3];
  for (uint8_t axis = 0; axis < 3; ++axis) halfRange[axis] = (magMaximum[axis] - magMinimum[axis]) * 0.5f;
  if (calibrationSamples < 100 || halfRange[0] < 20.0f || halfRange[1] < 20.0f || halfRange[2] < 20.0f) {
    setStatus("Mag FAIL: movimiento insuficiente");
    return;
  }
  const float averageRange = (halfRange[0] + halfRange[1] + halfRange[2]) / 3.0f;
  Imu9250Calibration::Data data = Imu9250Calibration::get();
  for (uint8_t axis = 0; axis < 3; ++axis) {
    data.magOffset[axis] = (magMaximum[axis] + magMinimum[axis]) * 0.5f;
    data.magScale[axis] = averageRange / halfRange[axis];
  }
  data.validFlags |= Imu9250Calibration::VALID_MAG;
  setStatus(Imu9250Calibration::save(data) ? "Mag PASS y guardado" : "Mag PASS, error NVS");
  copyCalibrationToSample();
}

void updateCalibration() {
  const unsigned long now = millis();
  if (calibrationMode == CalibrationMode::Gyro) {
    if (freshGyro) {
      const float values[3] = {latestSample.gxDps, latestSample.gyDps, latestSample.gzDps};
      for (uint8_t axis = 0; axis < 3; ++axis) { sampleSum[axis] += values[axis]; sampleSquareSum[axis] += values[axis] * values[axis]; }
      ++calibrationSamples;
    }
    if (now - calibrationStartMs >= Config::SENSOR_GYRO_CALIBRATION_MS) finishGyroCalibration();
  } else if (calibrationMode == CalibrationMode::AccelPose) {
    if (freshAccel) {
      const float values[3] = {latestSample.axG, latestSample.ayG, latestSample.azG};
      for (uint8_t axis = 0; axis < 3; ++axis) sampleSum[axis] += values[axis];
      ++calibrationSamples;
    }
    if (now - calibrationStartMs >= Config::SENSOR_ACCEL_POSE_CAPTURE_MS) finishAccelPose();
  } else if (calibrationMode == CalibrationMode::Magnetometer) {
    const unsigned long elapsed = now - calibrationStartMs;
    if (freshMag && now - lastMagCalibrationSampleMs >= 20) {
      lastMagCalibrationSampleMs = now;
      const float values[3] = {latestSample.mx, latestSample.my, latestSample.mz};
      for (uint8_t axis = 0; axis < 3; ++axis) {
        magMinimum[axis] = min(magMinimum[axis], values[axis]);
        magMaximum[axis] = max(magMaximum[axis], values[axis]);
      }
      ++calibrationSamples;
    }
    if (elapsed >= Config::SENSOR_MAG_CALIBRATION_MS) finishMagCalibration();
    else setStatus(String("Mag ") + String(elapsed * 100UL / Config::SENSOR_MAG_CALIBRATION_MS) + "%: gire en 3 ejes");
  }
  latestSample.calibrationSamples = calibrationSamples;
}

void updateRates() {
  const unsigned long now = millis();
  if (now - previousRateMs < 1000) return;
  const unsigned long elapsed = previousRateMs == 0 ? 1000 : now - previousRateMs;
  latestSample.accelRateHz = (accelReads - previousAccelReads) * 1000UL / elapsed;
  latestSample.gyroRateHz = (gyroReads - previousGyroReads) * 1000UL / elapsed;
  latestSample.magRateHz = (magReads - previousMagReads) * 1000UL / elapsed;
  latestSample.sampleRateHz = min(latestSample.accelRateHz, latestSample.gyroRateHz);
  previousAccelReads = accelReads; previousGyroReads = gyroReads; previousMagReads = magReads; previousRateMs = now;
}

}  // namespace

namespace Imu9250 {

bool begin() {
  Wire.begin(Config::PIN_I2C_SDA, Config::PIN_I2C_SCL);
  Wire.setClock(Config::I2C_CLOCK_HZ);
  Imu9250Calibration::begin();
  Imu9250FilterSettings::begin(Config::SENSOR_COMPLEMENTARY_ALPHA);
  filterAlpha = Imu9250FilterSettings::alpha();
  latestSample.filterAlpha = filterAlpha;
  imuAt68.setWire(&Wire);
  imuAt69.setWire(&Wire);
  selectImu();
  if (imu == nullptr) { setStatus("MPU9250 no detectado"); return false; }
  imu->beginAccel(ACC_FULL_SCALE_4_G);
  imu->beginGyro(GYRO_FULL_SCALE_500_DPS);
  imu->beginMag(MAG_MODE_100HZ_16BIT);
  latestSample.magnetometerId = readRegister(AK8963_ADDRESS, AK8963_WHO_AM_I);
  latestSample.magnetometerReady = latestSample.magnetometerId == 0x48;
  copyCalibrationToSample();
  setStatus(latestSample.calibrationStored ? "Calibracion cargada desde NVS" : "Sin calibracion guardada");
  previousRateMs = millis();
  Serial.printf("MPU9250 calibrated mode address=0x%02X id=0x%02X AK8963=0x%02X\n",
                latestSample.address, latestSample.imuId, latestSample.magnetometerId);
  return true;
}

void update() {
  if (imu == nullptr) return;
  freshAccel = imu->accelUpdate() == 0;
  freshGyro = imu->gyroUpdate() == 0;
  freshMag = latestSample.magnetometerId == 0x48 && imu->magUpdate() == 0;
  if (freshAccel) { latestSample.axG = imu->accelX(); latestSample.ayG = imu->accelY(); latestSample.azG = imu->accelZ(); latestSample.accelReady = true; ++accelReads; }
  if (freshGyro) { latestSample.gxDps = imu->gyroX(); latestSample.gyDps = imu->gyroY(); latestSample.gzDps = imu->gyroZ(); latestSample.gyroReady = true; ++gyroReads; }
  if (freshMag) { latestSample.mx = imu->magX(); latestSample.my = imu->magY(); latestSample.mz = imu->magZ(); latestSample.magnetometerReady = true; ++magReads; }
  if (freshAccel && freshGyro) latestSample.lastSampleMs = millis();
  applyCalibration();
  updateOrientation();
  updateCalibration();
  updateRates();
}

RawSample getSample() { return latestSample; }
bool isReady() { return imu != nullptr && latestSample.accelReady && latestSample.gyroReady; }

bool startGyroCalibration() {
  if (calibrationMode != CalibrationMode::None || imu == nullptr) return false;
  calibrationMode = CalibrationMode::Gyro; setMode("gyro"); calibrationStartMs = millis(); resetAccumulator();
  setStatus("Gyro: mantenga el robot inmovil"); return true;
}

bool startAccelCalibration() {
  if (calibrationMode != CalibrationMode::None || imu == nullptr) return false;
  latestSample.accelWizardActive = true; latestSample.accelPoseIndex = 0;
  strlcpy(latestSample.accelPoseName, ACCEL_POSE_NAMES[0], sizeof(latestSample.accelPoseName));
  setStatus(String("Acelerometro: coloque ") + latestSample.accelPoseName); return true;
}

bool captureAccelPose() {
  if (!latestSample.accelWizardActive || calibrationMode != CalibrationMode::None || !latestSample.accelReady) return false;
  calibrationMode = CalibrationMode::AccelPose; setMode("accelerometer"); calibrationStartMs = millis(); resetAccumulator();
  setStatus(String("Capturando ") + latestSample.accelPoseName); return true;
}

bool startMagCalibration() {
  if (calibrationMode != CalibrationMode::None || !latestSample.magnetometerReady) return false;
  calibrationMode = CalibrationMode::Magnetometer; setMode("magnetometer"); calibrationStartMs = millis();
  lastMagCalibrationSampleMs = 0; calibrationSamples = 0;
  for (uint8_t axis = 0; axis < 3; ++axis) { magMinimum[axis] = INFINITY; magMaximum[axis] = -INFINITY; }
  setStatus("Mag 0%: gire en 3 ejes"); return true;
}

bool saveVertical() {
  if (calibrationMode != CalibrationMode::None || !latestSample.filterReady) return false;
  Imu9250Calibration::Data data = Imu9250Calibration::get();
  data.verticalRollDeg = latestSample.filteredRollDeg; data.verticalPitchDeg = latestSample.filteredPitchDeg;
  data.validFlags |= Imu9250Calibration::VALID_VERTICAL;
  const bool saved = Imu9250Calibration::save(data);
  setStatus(saved ? "Vertical guardada" : "Error guardando vertical"); copyCalibrationToSample(); return saved;
}

bool clearCalibration() {
  if (calibrationMode != CalibrationMode::None) return false;
  const bool cleared = Imu9250Calibration::clear();
  filterInitialized = false; latestSample.accelWizardActive = false; latestSample.accelPoseIndex = 0;
  copyCalibrationToSample(); setStatus("Calibracion borrada"); return cleared;
}

}  // namespace Imu9250
