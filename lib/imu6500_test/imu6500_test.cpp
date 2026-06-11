#include "imu6500_test.h"

#include <Wire.h>
#include <mpu6500.h>
#include <math.h>

#include "../../include/config.h"

namespace {

bfs::Mpu6500 imu;
bool imuReady = false;
uint8_t imuAddress = 0;
unsigned long lastPrintMs = 0;
float gyroOffsetX = 0.0f;
float gyroOffsetY = 0.0f;
float gyroOffsetZ = 0.0f;
Imu6500Test::ImuSample latestSample;
bool gyroCalibrated = false;
bool angleInitialized = false;
float angleFilteredInternalDeg = 0.0f;
float filterAlpha = Config::COMPLEMENTARY_FILTER_ALPHA;
float angleVerticalOffsetDeg = Config::ANGLE_VERTICAL_OFFSET_DEG;
uint32_t lastUpdateMicros = 0;
bool useKalmanAngle = false;
bool kalmanInitialized = false;
float kalmanAngleDeg = 0.0f;
float kalmanBias = 0.0f;
float kalmanRate = 0.0f;
float kalmanP00 = 0.0f;
float kalmanP01 = 0.0f;
float kalmanP10 = 0.0f;
float kalmanP11 = 0.0f;
float kalmanQAngle = Config::KALMAN_Q_ANGLE;
float kalmanQBias = Config::KALMAN_Q_BIAS;
float kalmanRMeasure = Config::KALMAN_R_MEASURE;

bool isDevicePresent(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

void scanI2cBus() {
  Serial.println(F("Scanning I2C bus..."));
  uint8_t found = 0;

  for (uint8_t address = 1; address < 127; ++address) {
    if (isDevicePresent(address)) {
      Serial.print(F("I2C device found at 0x"));
      if (address < 16) {
        Serial.print('0');
      }
      Serial.println(address, HEX);
      ++found;
    }
  }

  if (found == 0) {
    Serial.println(F("No I2C devices found"));
  }
}

bool tryBeginAt(uint8_t address) {
  const auto mpuAddress = address == 0x69
                              ? bfs::Mpu6500::I2C_ADDR_SEC
                              : bfs::Mpu6500::I2C_ADDR_PRIM;

  imu.Config(&Wire, mpuAddress);
  if (!imu.Begin()) {
    return false;
  }

  imu.ConfigAccelRange(bfs::Mpu6500::ACCEL_RANGE_4G);
  imu.ConfigGyroRange(bfs::Mpu6500::GYRO_RANGE_500DPS);
  imu.ConfigDlpfBandwidth(bfs::Mpu6500::DLPF_BANDWIDTH_20HZ);
  imu.ConfigSrd(19);
  imuAddress = address;
  return true;
}

void printFloatLabel(const __FlashStringHelper *label, float value, uint8_t decimals = 3) {
  Serial.print(label);
  Serial.print(value, decimals);
}

float applyBalanceAngleSign(float angleDeg) {
  return Config::INVERT_BALANCE_ANGLE ? -angleDeg : angleDeg;
}

float applyGyroRateSign(float gyroRateDegPerSec) {
  return Config::INVERT_GYRO_RATE ? -gyroRateDegPerSec : gyroRateDegPerSec;
}

float calculateAccelAngleDeg(float ay, float az) {
  return applyBalanceAngleSign(atan2f(ay, -az) * RAD_TO_DEG);
}

float calculateDtSeconds(uint32_t nowMicros) {
  if (lastUpdateMicros == 0) {
    lastUpdateMicros = nowMicros;
    return 0.0f;
  }

  const uint32_t elapsedMicros = nowMicros - lastUpdateMicros;
  lastUpdateMicros = nowMicros;
  float dt = static_cast<float>(elapsedMicros) / 1000000.0f;

  if (dt <= 0.0f || dt > 0.2f) {
    dt = static_cast<float>(Config::CONTROL_TASK_PERIOD_MS) / 1000.0f;
  }

  return dt;
}

void resetKalmanInternal(float angleDeg) {
  kalmanAngleDeg = angleDeg;
  kalmanBias = 0.0f;
  kalmanRate = 0.0f;
  kalmanP00 = 0.0f;
  kalmanP01 = 0.0f;
  kalmanP10 = 0.0f;
  kalmanP11 = 0.0f;
  kalmanInitialized = true;
}

float updateKalman(float accelAngleDeg, float gyroRateDegPerSec, float dt) {
  if (!kalmanInitialized) {
    resetKalmanInternal(accelAngleDeg);
    return kalmanAngleDeg;
  }
  if (dt <= 0.0f || dt > 0.2f) {
    return kalmanAngleDeg;
  }

  kalmanRate = gyroRateDegPerSec - kalmanBias;
  kalmanAngleDeg += dt * kalmanRate;

  kalmanP00 += dt * (dt * kalmanP11 - kalmanP01 - kalmanP10 + kalmanQAngle);
  kalmanP01 -= dt * kalmanP11;
  kalmanP10 -= dt * kalmanP11;
  kalmanP11 += kalmanQBias * dt;

  const float innovation = accelAngleDeg - kalmanAngleDeg;
  const float innovationCovariance = kalmanP00 + kalmanRMeasure;
  if (innovationCovariance == 0.0f) {
    return kalmanAngleDeg;
  }

  const float k0 = kalmanP00 / innovationCovariance;
  const float k1 = kalmanP10 / innovationCovariance;

  kalmanAngleDeg += k0 * innovation;
  kalmanBias += k1 * innovation;

  const float p00Temp = kalmanP00;
  const float p01Temp = kalmanP01;
  kalmanP00 -= k0 * p00Temp;
  kalmanP01 -= k0 * p01Temp;
  kalmanP10 -= k1 * p00Temp;
  kalmanP11 -= k1 * p01Temp;

  return kalmanAngleDeg;
}

float clampFloat(float value, float minValue, float maxValue) {
  if (value < minValue) {
    return minValue;
  }
  if (value > maxValue) {
    return maxValue;
  }
  return value;
}

}  // namespace

namespace Imu6500Test {

bool begin() {
  Wire.begin(Config::PIN_I2C_SDA, Config::PIN_I2C_SCL);
  Wire.setClock(Config::I2C_CLOCK_HZ);

  scanI2cBus();

  imuReady = false;
  if (isDevicePresent(0x68) && tryBeginAt(0x68)) {
    imuReady = true;
  } else if (isDevicePresent(0x69) && tryBeginAt(0x69)) {
    imuReady = true;
  }

  if (imuReady) {
    Serial.print(F("MPU6500 initialized at 0x"));
    Serial.println(imuAddress, HEX);
    Serial.println(F("Gyro calibration deferred to auto recovery"));
  } else {
    Serial.println(F("MPU6500 initialization failed at 0x68 and 0x69"));
  }

  return imuReady;
}

bool calibrateGyro() {
  if (!imuReady) {
    Serial.println(F("Cannot calibrate gyro: IMU not ready"));
    return false;
  }

  Serial.println(F("Keep robot still: calibrating gyro..."));
  double sumX = 0.0;
  double sumY = 0.0;
  double sumZ = 0.0;
  uint16_t validSamples = 0;

  for (uint16_t sample = 0; sample < Config::GYRO_CALIBRATION_SAMPLES; ++sample) {
    if (imu.Read()) {
      sumX += imu.gyro_x_radps();
      sumY += imu.gyro_y_radps();
      sumZ += imu.gyro_z_radps();
      ++validSamples;
    }
    vTaskDelay(pdMS_TO_TICKS(Config::GYRO_CALIBRATION_SAMPLE_DELAY_MS));
  }

  if (validSamples == 0) {
    Serial.println(F("Gyro calibration failed: no valid samples"));
    return false;
  }

  gyroOffsetX = static_cast<float>(sumX / validSamples);
  gyroOffsetY = static_cast<float>(sumY / validSamples);
  gyroOffsetZ = static_cast<float>(sumZ / validSamples);
  gyroCalibrated = true;
  latestSample.gyroOffsetX = gyroOffsetX;
  latestSample.gyroOffsetY = gyroOffsetY;
  latestSample.gyroOffsetZ = gyroOffsetZ;
  latestSample.gyroCalibrated = gyroCalibrated;
  lastUpdateMicros = micros();

  Serial.print(F("Gyro offsets rad/s: x="));
  Serial.print(gyroOffsetX, 5);
  Serial.print(F(" y="));
  Serial.print(gyroOffsetY, 5);
  Serial.print(F(" z="));
  Serial.println(gyroOffsetZ, 5);
  Serial.println(F("Gyro calibration complete"));
  return true;
}

bool update() {
  if (!imuReady || !imu.Read()) {
    return false;
  }

  const uint32_t nowMicros = micros();
  const float dt = calculateDtSeconds(nowMicros);

  latestSample.rawAx = imu.accel_x_mps2();
  latestSample.rawAy = imu.accel_y_mps2();
  latestSample.rawAz = imu.accel_z_mps2();
  latestSample.rawGx = imu.gyro_x_radps();
  latestSample.rawGy = imu.gyro_y_radps();
  latestSample.rawGz = imu.gyro_z_radps();
  latestSample.correctedGx = latestSample.rawGx - gyroOffsetX;
  latestSample.correctedGy = latestSample.rawGy - gyroOffsetY;
  latestSample.correctedGz = latestSample.rawGz - gyroOffsetZ;
  latestSample.gyroOffsetX = gyroOffsetX;
  latestSample.gyroOffsetY = gyroOffsetY;
  latestSample.gyroOffsetZ = gyroOffsetZ;
  latestSample.angleAccelDeg = calculateAccelAngleDeg(latestSample.rawAy, latestSample.rawAz);
  latestSample.gyroRateDegPerSec = applyGyroRateSign(latestSample.correctedGx * RAD_TO_DEG);

  if (!angleInitialized || dt == 0.0f) {
    angleFilteredInternalDeg = latestSample.angleAccelDeg;
    resetKalmanInternal(latestSample.angleAccelDeg);
    angleInitialized = true;
  } else {
    angleFilteredInternalDeg =
        filterAlpha *
            (angleFilteredInternalDeg + latestSample.gyroRateDegPerSec * dt) +
        (1.0f - filterAlpha) * latestSample.angleAccelDeg;
    updateKalman(latestSample.angleAccelDeg, latestSample.gyroRateDegPerSec, dt);
  }

  latestSample.angleComplementaryDeg = angleFilteredInternalDeg - angleVerticalOffsetDeg;
  latestSample.angleKalmanDeg = kalmanAngleDeg - angleVerticalOffsetDeg;
  latestSample.selectedAngleDeg = useKalmanAngle ? latestSample.angleKalmanDeg
                                                 : latestSample.angleComplementaryDeg;
  latestSample.angleFilteredDeg = latestSample.selectedAngleDeg;
  latestSample.angleVerticalOffsetDeg = angleVerticalOffsetDeg;
  latestSample.temperatureC = imu.die_temp_c();
  latestSample.gyroCalibrated = gyroCalibrated;
  latestSample.angleInitialized = angleInitialized;
  latestSample.useKalmanAngle = useKalmanAngle;
  latestSample.kalmanQAngle = kalmanQAngle;
  latestSample.kalmanQBias = kalmanQBias;
  latestSample.kalmanRMeasure = kalmanRMeasure;
  return true;
}

ImuSample getSample() {
  return latestSample;
}

void calibrateVerticalAngle() {
  if (!angleInitialized) {
    Serial.println(F("Cannot calibrate vertical angle: angle not initialized"));
    return;
  }

  angleVerticalOffsetDeg = angleFilteredInternalDeg;
  resetKalmanInternal(latestSample.angleAccelDeg);
  latestSample.angleVerticalOffsetDeg = angleVerticalOffsetDeg;
  latestSample.angleFilteredDeg = 0.0f;
  latestSample.angleComplementaryDeg = 0.0f;
  latestSample.angleKalmanDeg = kalmanAngleDeg - angleVerticalOffsetDeg;
  latestSample.selectedAngleDeg = useKalmanAngle ? latestSample.angleKalmanDeg
                                                 : latestSample.angleComplementaryDeg;
  Serial.print(F("Vertical angle offset deg: "));
  Serial.println(angleVerticalOffsetDeg, 3);
}

void setFilterAlpha(float alpha) {
  if (alpha < Config::FILTER_ALPHA_MIN) {
    alpha = Config::FILTER_ALPHA_MIN;
  } else if (alpha > Config::FILTER_ALPHA_MAX) {
    alpha = Config::FILTER_ALPHA_MAX;
  }
  filterAlpha = alpha;
}

float getFilterAlpha() {
  return filterAlpha;
}

float getVerticalOffsetDeg() {
  return angleVerticalOffsetDeg;
}

void setAngleFilterMode(bool newUseKalman) {
  useKalmanAngle = newUseKalman;
}

bool isUsingKalmanAngle() {
  return useKalmanAngle;
}

void setKalmanParameters(float qAngle, float qBias, float rMeasure) {
  kalmanQAngle = clampFloat(qAngle, Config::KALMAN_Q_ANGLE_MIN,
                            Config::KALMAN_Q_ANGLE_MAX);
  kalmanQBias = clampFloat(qBias, Config::KALMAN_Q_BIAS_MIN,
                           Config::KALMAN_Q_BIAS_MAX);
  kalmanRMeasure = clampFloat(rMeasure, Config::KALMAN_R_MEASURE_MIN,
                              Config::KALMAN_R_MEASURE_MAX);
}

float getKalmanQAngle() {
  return kalmanQAngle;
}

float getKalmanQBias() {
  return kalmanQBias;
}

float getKalmanRMeasure() {
  return kalmanRMeasure;
}

void resetKalman() {
  resetKalmanInternal(latestSample.angleAccelDeg);
}

void printSampleIfDue() {
  const unsigned long now = millis();
  if (now - lastPrintMs < Config::IMU_PRINT_INTERVAL_MS) {
    return;
  }
  lastPrintMs = now;

  if (!imuReady) {
    Serial.println(F("IMU not ready"));
    return;
  }

  if (!update()) {
    Serial.println(F("IMU read failed"));
    return;
  }

  const ImuSample sample = getSample();

  Serial.print(F("IMU raw_ax="));
  Serial.print(sample.rawAx, 3);
  printFloatLabel(F(" raw_ay="), sample.rawAy);
  printFloatLabel(F(" raw_az="), sample.rawAz);
  printFloatLabel(F(" raw_gx="), sample.rawGx);
  printFloatLabel(F(" raw_gy="), sample.rawGy);
  printFloatLabel(F(" raw_gz="), sample.rawGz);
  printFloatLabel(F(" corr_gx="), sample.correctedGx);
  printFloatLabel(F(" corr_gy="), sample.correctedGy);
  printFloatLabel(F(" corr_gz="), sample.correctedGz);
  printFloatLabel(F(" angle_accel="), sample.angleAccelDeg);
  printFloatLabel(F(" angle_complementary="), sample.angleComplementaryDeg);
  printFloatLabel(F(" angle_kalman="), sample.angleKalmanDeg);
  printFloatLabel(F(" angle_selected="), sample.selectedAngleDeg);
  printFloatLabel(F(" gyro_rate="), sample.gyroRateDegPerSec);
  Serial.print(F(" filter="));
  Serial.print(sample.useKalmanAngle ? F("kalman") : F("complementary"));
  printFloatLabel(F(" temp="), sample.temperatureC, 1);
  Serial.println();
}

bool isReady() {
  return imuReady;
}

uint8_t detectedAddress() {
  return imuAddress;
}

}  // namespace Imu6500Test
