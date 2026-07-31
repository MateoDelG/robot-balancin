#include "state_feedback_settings.h"

#include <Preferences.h>
#include <math.h>

#include "../../include/config.h"

namespace {

constexpr char NVS_NAMESPACE[] = "statefb";
constexpr char NVS_KEY[] = "settings";
constexpr char NVS_SNAPSHOT_KEY[] = "snapshot";
constexpr uint32_t VERSION = 1;

struct StoredSettings {
  uint32_t version;
  double gains[5];
  float filters[2];
  uint32_t checksum;
};

StateFeedbackSettings::Settings currentSettings{};

uint32_t checksum(const StoredSettings &stored) {
  uint32_t hash = 2166136261u;
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&stored);
  const size_t length = offsetof(StoredSettings, checksum);
  for (size_t index = 0; index < length; ++index) {
    hash ^= bytes[index];
    hash *= 16777619u;
  }
  return hash;
}

bool valid(const StateFeedbackSettings::Settings &settings) {
  const double values[] = {settings.gains.position, settings.gains.velocity,
                           settings.gains.angle, settings.gains.angularVelocity,
                           settings.gains.angularAcceleration};
  for (double value : values) {
    if (!isfinite(value) || value < Config::STATE_GAIN_MIN ||
        value > Config::STATE_GAIN_MAX) return false;
  }
  return isfinite(settings.velocityFilterBeta) &&
         settings.velocityFilterBeta >= Config::STATE_FILTER_BETA_MIN &&
         settings.velocityFilterBeta <= Config::STATE_FILTER_BETA_MAX &&
         isfinite(settings.angularAccelerationFilterBeta) &&
         settings.angularAccelerationFilterBeta >= Config::STATE_FILTER_BETA_MIN &&
         settings.angularAccelerationFilterBeta <= Config::STATE_FILTER_BETA_MAX;
}

StoredSettings encode(const StateFeedbackSettings::Settings &settings) {
  StoredSettings stored{};
  stored.version = VERSION;
  stored.gains[0] = settings.gains.position;
  stored.gains[1] = settings.gains.velocity;
  stored.gains[2] = settings.gains.angle;
  stored.gains[3] = settings.gains.angularVelocity;
  stored.gains[4] = settings.gains.angularAcceleration;
  stored.filters[0] = settings.velocityFilterBeta;
  stored.filters[1] = settings.angularAccelerationFilterBeta;
  stored.checksum = checksum(stored);
  return stored;
}

StateFeedbackSettings::Settings decode(const StoredSettings &stored) {
  return {{stored.gains[0], stored.gains[1], stored.gains[2], stored.gains[3],
           stored.gains[4]},
          stored.filters[0], stored.filters[1]};
}

}  // namespace

namespace StateFeedbackSettings {

void begin(double defaultAngleGain, double defaultAngularVelocityGain) {
  currentSettings = {{0.0, 0.0, defaultAngleGain, defaultAngularVelocityGain, 0.0},
                     Config::INITIAL_VELOCITY_FILTER_BETA,
                     Config::INITIAL_ANGULAR_ACCEL_FILTER_BETA};
  Preferences preferences;
  if (!preferences.begin(NVS_NAMESPACE, true)) return;
  StoredSettings stored{};
  const bool loaded = preferences.getBytesLength(NVS_KEY) == sizeof(stored) &&
                      preferences.getBytes(NVS_KEY, &stored, sizeof(stored)) == sizeof(stored);
  preferences.end();
  if (!loaded || stored.version != VERSION || stored.checksum != checksum(stored)) return;
  const Settings candidate = decode(stored);
  if (valid(candidate)) currentSettings = candidate;
}

Settings get() {
  return currentSettings;
}

bool save(const Settings &settings) {
  if (!valid(settings)) return false;
  const StoredSettings stored = encode(settings);
  Preferences preferences;
  if (!preferences.begin(NVS_NAMESPACE, false)) return false;
  const bool success = preferences.putBytes(NVS_KEY, &stored, sizeof(stored)) == sizeof(stored);
  preferences.end();
  if (success) currentSettings = settings;
  return success;
}

bool saveSnapshot(const Settings &settings) {
  if (!valid(settings)) return false;
  const StoredSettings stored = encode(settings);
  Preferences preferences;
  if (!preferences.begin(NVS_NAMESPACE, false)) return false;
  const bool success = preferences.putBytes(NVS_SNAPSHOT_KEY, &stored, sizeof(stored)) == sizeof(stored);
  preferences.end();
  return success;
}

bool loadSnapshot(Settings &settings) {
  Preferences preferences;
  if (!preferences.begin(NVS_NAMESPACE, true)) return false;
  StoredSettings stored{};
  const bool loaded = preferences.getBytesLength(NVS_SNAPSHOT_KEY) == sizeof(stored) &&
                      preferences.getBytes(NVS_SNAPSHOT_KEY, &stored, sizeof(stored)) == sizeof(stored);
  preferences.end();
  if (!loaded || stored.version != VERSION || stored.checksum != checksum(stored)) return false;
  const Settings candidate = decode(stored);
  if (!valid(candidate)) return false;
  settings = candidate;
  return true;
}

}  // namespace StateFeedbackSettings
