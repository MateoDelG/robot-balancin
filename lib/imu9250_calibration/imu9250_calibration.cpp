#include "imu9250_calibration.h"

#include <Preferences.h>
#include <stddef.h>
#include <string.h>

namespace {

constexpr uint32_t CALIBRATION_MAGIC = 0x9250CA1B;
constexpr uint16_t CALIBRATION_VERSION = 1;
constexpr char NVS_NAMESPACE[] = "imu9250";
constexpr char NVS_KEY[] = "calibration";

Imu9250Calibration::Data currentData;
bool loaded = false;

uint32_t calculateChecksum(const Imu9250Calibration::Data &data) {
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&data);
  const size_t length = offsetof(Imu9250Calibration::Data, checksum);
  uint32_t hash = 2166136261UL;
  for (size_t index = 0; index < length; ++index) {
    hash ^= bytes[index];
    hash *= 16777619UL;
  }
  return hash;
}

bool dataIsValid(const Imu9250Calibration::Data &data) {
  return data.magic == CALIBRATION_MAGIC && data.version == CALIBRATION_VERSION &&
         data.checksum == calculateChecksum(data);
}

}  // namespace

namespace Imu9250Calibration {

Data defaults() {
  Data data{};
  data.magic = CALIBRATION_MAGIC;
  data.version = CALIBRATION_VERSION;
  for (uint8_t axis = 0; axis < 3; ++axis) {
    data.accelScale[axis] = 1.0f;
    data.magScale[axis] = 1.0f;
  }
  data.checksum = calculateChecksum(data);
  return data;
}

void begin() {
  currentData = defaults();
  Preferences preferences;
  if (!preferences.begin(NVS_NAMESPACE, true)) return;
  Data stored{};
  const size_t bytesRead = preferences.getBytes(NVS_KEY, &stored, sizeof(stored));
  preferences.end();
  if (bytesRead == sizeof(stored) && dataIsValid(stored)) {
    currentData = stored;
    loaded = true;
  }
}

const Data &get() {
  return currentData;
}

bool isValid(uint16_t flag) {
  return (currentData.validFlags & flag) != 0;
}

bool wasLoaded() {
  return loaded;
}

bool save(const Data &data) {
  Data value = data;
  value.magic = CALIBRATION_MAGIC;
  value.version = CALIBRATION_VERSION;
  value.checksum = calculateChecksum(value);
  Preferences preferences;
  if (!preferences.begin(NVS_NAMESPACE, false)) return false;
  const bool success = preferences.putBytes(NVS_KEY, &value, sizeof(value)) == sizeof(value);
  preferences.end();
  if (success) {
    currentData = value;
    loaded = true;
  }
  return success;
}

bool clear() {
  Preferences preferences;
  if (!preferences.begin(NVS_NAMESPACE, false)) return false;
  const bool success = preferences.remove(NVS_KEY);
  preferences.end();
  currentData = defaults();
  loaded = false;
  return success;
}

}  // namespace Imu9250Calibration
