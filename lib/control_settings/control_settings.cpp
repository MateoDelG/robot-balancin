#include "control_settings.h"

#include <Arduino.h>
#include <Preferences.h>
#include <math.h>

#include "../../include/config.h"

namespace {

constexpr char NVS_NAMESPACE[] = "mpu9250ctl";
constexpr char LEGACY_NAMESPACE[] = "controlcfg";
constexpr char NVS_SETTINGS_KEY[] = "settings";
constexpr uint32_t SETTINGS_VERSION = 3;

struct StoredSettingsV1 {
  uint32_t version;
  double kp;
  double ki;
  double kd;
  double setpoint;
  int32_t pidMaxPwm;
  int32_t leftMinPwm;
  int32_t leftMaxPwm;
  int32_t rightMinPwm;
  int32_t rightMaxPwm;
  uint32_t checksum;
};

struct StoredSettings {
  uint32_t version;
  double kp;
  double ki;
  double kd;
  double setpoint;
  int32_t pidMaxPwm;
  int32_t leftMinPwm;
  int32_t leftMaxPwm;
  int32_t rightMinPwm;
  int32_t rightMaxPwm;
  double leftCompensation;
  double rightCompensation;
  uint32_t checksum;
};

ControlSettings::Settings currentSettings{};

void hashBytes(uint32_t &hash, const void *value, size_t length) {
  const uint8_t *bytes = static_cast<const uint8_t *>(value);
  for (size_t index = 0; index < length; ++index) {
    hash ^= bytes[index];
    hash *= 16777619u;
  }
}

uint32_t checksum(const StoredSettings &stored) {
  uint32_t hash = 2166136261u;
  hashBytes(hash, &stored.version, sizeof(stored.version));
  hashBytes(hash, &stored.kp, sizeof(stored.kp));
  hashBytes(hash, &stored.ki, sizeof(stored.ki));
  hashBytes(hash, &stored.kd, sizeof(stored.kd));
  hashBytes(hash, &stored.setpoint, sizeof(stored.setpoint));
  hashBytes(hash, &stored.pidMaxPwm, sizeof(stored.pidMaxPwm));
  hashBytes(hash, &stored.leftMinPwm, sizeof(stored.leftMinPwm));
  hashBytes(hash, &stored.leftMaxPwm, sizeof(stored.leftMaxPwm));
  hashBytes(hash, &stored.rightMinPwm, sizeof(stored.rightMinPwm));
  hashBytes(hash, &stored.rightMaxPwm, sizeof(stored.rightMaxPwm));
  hashBytes(hash, &stored.leftCompensation, sizeof(stored.leftCompensation));
  hashBytes(hash, &stored.rightCompensation, sizeof(stored.rightCompensation));
  return hash;
}

uint32_t checksum(const StoredSettingsV1 &stored) {
  uint32_t hash = 2166136261u;
  hashBytes(hash, &stored.version, sizeof(stored.version));
  hashBytes(hash, &stored.kp, sizeof(stored.kp));
  hashBytes(hash, &stored.ki, sizeof(stored.ki));
  hashBytes(hash, &stored.kd, sizeof(stored.kd));
  hashBytes(hash, &stored.setpoint, sizeof(stored.setpoint));
  hashBytes(hash, &stored.pidMaxPwm, sizeof(stored.pidMaxPwm));
  hashBytes(hash, &stored.leftMinPwm, sizeof(stored.leftMinPwm));
  hashBytes(hash, &stored.leftMaxPwm, sizeof(stored.leftMaxPwm));
  hashBytes(hash, &stored.rightMinPwm, sizeof(stored.rightMinPwm));
  hashBytes(hash, &stored.rightMaxPwm, sizeof(stored.rightMaxPwm));
  return hash;
}

bool validPid(double kp, double ki, double kd) {
  return isfinite(kp) && isfinite(ki) && isfinite(kd) &&
         kp >= Config::PID_KP_MIN && kp <= Config::PID_KP_MAX &&
         ki >= Config::PID_KI_MIN && ki <= Config::PID_KI_MAX &&
         kd >= Config::PID_KD_MIN && kd <= Config::PID_KD_MAX;
}

bool validSettings(const ControlSettings::Settings &settings) {
  return validPid(settings.kp, settings.ki, settings.kd) && isfinite(settings.setpoint) &&
         settings.setpoint >= Config::SETPOINT_MIN_DEG &&
         settings.setpoint <= Config::SETPOINT_MAX_DEG &&
         settings.pidMaxPwm >= Config::PID_MAX_PWM_MIN &&
         settings.pidMaxPwm <= Config::PID_MAX_PWM_MAX &&
         settings.leftMinPwm >= Config::MOTOR_PWM_LIMIT_MIN &&
         settings.leftMaxPwm <= Config::MOTOR_PWM_LIMIT_MAX &&
         settings.leftMinPwm <= settings.leftMaxPwm &&
         settings.rightMinPwm >= Config::MOTOR_PWM_LIMIT_MIN &&
         settings.rightMaxPwm <= Config::MOTOR_PWM_LIMIT_MAX &&
         settings.rightMinPwm <= settings.rightMaxPwm &&
         isfinite(settings.leftCompensation) &&
         settings.leftCompensation >= Config::MOTOR_COMPENSATION_MIN &&
         settings.leftCompensation <= Config::MOTOR_COMPENSATION_MAX &&
         isfinite(settings.rightCompensation) &&
         settings.rightCompensation >= Config::MOTOR_COMPENSATION_MIN &&
         settings.rightCompensation <= Config::MOTOR_COMPENSATION_MAX;
}

StoredSettings toStored(const ControlSettings::Settings &settings) {
  StoredSettings stored{};
  stored.version = SETTINGS_VERSION;
  stored.kp = settings.kp;
  stored.ki = settings.ki;
  stored.kd = settings.kd;
  stored.setpoint = settings.setpoint;
  stored.pidMaxPwm = settings.pidMaxPwm;
  stored.leftMinPwm = settings.leftMinPwm;
  stored.leftMaxPwm = settings.leftMaxPwm;
  stored.rightMinPwm = settings.rightMinPwm;
  stored.rightMaxPwm = settings.rightMaxPwm;
  stored.leftCompensation = settings.leftCompensation;
  stored.rightCompensation = settings.rightCompensation;
  stored.checksum = checksum(stored);
  return stored;
}

ControlSettings::Settings fromStored(const StoredSettings &stored) {
  return {stored.kp,          stored.ki,          stored.kd,       stored.setpoint,
          stored.pidMaxPwm,  stored.leftMinPwm,  stored.leftMaxPwm,
          stored.rightMinPwm, stored.rightMaxPwm, stored.leftCompensation,
          stored.rightCompensation};
}

ControlSettings::Settings scaleLegacySettings(ControlSettings::Settings settings) {
  settings.kp = Config::scaleLegacyPwmGain(settings.kp);
  settings.ki = Config::scaleLegacyPwmGain(settings.ki);
  settings.kd = Config::scaleLegacyPwmGain(settings.kd);
  settings.pidMaxPwm = Config::scaleLegacyPwm(settings.pidMaxPwm);
  settings.leftMinPwm = Config::scaleLegacyPwm(settings.leftMinPwm);
  settings.leftMaxPwm = Config::scaleLegacyPwm(settings.leftMaxPwm);
  settings.rightMinPwm = Config::scaleLegacyPwm(settings.rightMinPwm);
  settings.rightMaxPwm = Config::scaleLegacyPwm(settings.rightMaxPwm);
  return settings;
}

bool writeSettings(const ControlSettings::Settings &settings) {
  if (!validSettings(settings)) return false;
  const StoredSettings stored = toStored(settings);
  Preferences preferences;
  if (!preferences.begin(NVS_NAMESPACE, false)) return false;
  const bool success = preferences.putBytes(NVS_SETTINGS_KEY, &stored, sizeof(stored)) == sizeof(stored);
  preferences.end();
  return success;
}

bool loadStoredSettings(ControlSettings::Settings &settings, bool &migrated) {
  Preferences preferences;
  if (!preferences.begin(NVS_NAMESPACE, true)) return false;
  const size_t storedLength = preferences.getBytesLength(NVS_SETTINGS_KEY);
  if (storedLength == sizeof(StoredSettings)) {
    StoredSettings stored{};
    const bool read = preferences.getBytes(NVS_SETTINGS_KEY, &stored, sizeof(stored)) == sizeof(stored);
    preferences.end();
    if (!read || stored.checksum != checksum(stored)) return false;
    if (stored.version != 2 && stored.version != SETTINGS_VERSION) return false;
    ControlSettings::Settings candidate = fromStored(stored);
    if (stored.version == 2) candidate = scaleLegacySettings(candidate);
    if (!validSettings(candidate)) return false;
    settings = candidate;
    migrated = stored.version != SETTINGS_VERSION;
    return true;
  }
  if (storedLength == sizeof(StoredSettingsV1)) {
    StoredSettingsV1 stored{};
    const bool read = preferences.getBytes(NVS_SETTINGS_KEY, &stored, sizeof(stored)) == sizeof(stored);
    preferences.end();
    if (!read || stored.version != 1 || stored.checksum != checksum(stored)) return false;
    ControlSettings::Settings candidate = {
        stored.kp, stored.ki, stored.kd, stored.setpoint, stored.pidMaxPwm,
        stored.leftMinPwm, stored.leftMaxPwm, stored.rightMinPwm, stored.rightMaxPwm,
        Config::INITIAL_MOTOR_LEFT_COMPENSATION, Config::INITIAL_MOTOR_RIGHT_COMPENSATION};
    candidate = scaleLegacySettings(candidate);
    if (!validSettings(candidate)) return false;
    settings = candidate;
    migrated = true;
    return true;
  }
  preferences.end();
  return false;
}

void migrateLegacySettings(ControlSettings::Settings &settings) {
  Preferences preferences;
  if (!preferences.begin(LEGACY_NAMESPACE, true)) return;
  const double kp = preferences.getDouble("kp", NAN);
  const double ki = preferences.getDouble("ki", NAN);
  const double kd = preferences.getDouble("kd", NAN);
  const int leftMinPwm = preferences.getInt("lmin", -1);
  const int leftMaxPwm = preferences.getInt("lmax", -1);
  const int rightMinPwm = preferences.getInt("rmin", -1);
  const int rightMaxPwm = preferences.getInt("rmax", -1);
  preferences.end();

  ControlSettings::Settings candidate = settings;
  if (isfinite(kp) && isfinite(ki) && isfinite(kd)) {
    candidate.kp = Config::scaleLegacyPwmGain(kp);
    candidate.ki = Config::scaleLegacyPwmGain(ki);
    candidate.kd = Config::scaleLegacyPwmGain(kd);
  }
  candidate.leftMinPwm = Config::scaleLegacyPwm(leftMinPwm);
  candidate.leftMaxPwm = Config::scaleLegacyPwm(leftMaxPwm);
  candidate.rightMinPwm = Config::scaleLegacyPwm(rightMinPwm);
  candidate.rightMaxPwm = Config::scaleLegacyPwm(rightMaxPwm);
  if (validSettings(candidate)) settings = candidate;
}

bool commit(const ControlSettings::Settings &candidate) {
  if (!writeSettings(candidate)) return false;
  currentSettings = candidate;
  return true;
}

}  // namespace

namespace ControlSettings {

void begin(double defaultKp, double defaultKi, double defaultKd, double defaultSetpoint,
           int defaultPidMaxPwm) {
  currentSettings = {defaultKp,
                     defaultKi,
                     defaultKd,
                     defaultSetpoint,
                     defaultPidMaxPwm,
                     Config::INITIAL_MOTOR_LEFT_MIN_PWM,
                     Config::INITIAL_MOTOR_LEFT_MAX_PWM,
                     Config::INITIAL_MOTOR_RIGHT_MIN_PWM,
                     Config::INITIAL_MOTOR_RIGHT_MAX_PWM,
                     Config::INITIAL_MOTOR_LEFT_COMPENSATION,
                     Config::INITIAL_MOTOR_RIGHT_COMPENSATION};
  bool migrated = false;
  if (loadStoredSettings(currentSettings, migrated)) {
    if (migrated) writeSettings(currentSettings);
    return;
  }
  migrateLegacySettings(currentSettings);
  writeSettings(currentSettings);
}

Settings get() {
  return currentSettings;
}

bool savePid(double kp, double ki, double kd) {
  Settings candidate = currentSettings;
  candidate.kp = kp;
  candidate.ki = ki;
  candidate.kd = kd;
  return commit(candidate);
}

bool saveSetpoint(double setpoint) {
  Settings candidate = currentSettings;
  candidate.setpoint = setpoint;
  return commit(candidate);
}

bool savePidMaxPwm(int maxPwm) {
  Settings candidate = currentSettings;
  candidate.pidMaxPwm = maxPwm;
  return commit(candidate);
}

bool saveMotorConfig(int leftMinPwm, int leftMaxPwm, int rightMinPwm, int rightMaxPwm,
                     double leftCompensation, double rightCompensation) {
  Settings candidate = currentSettings;
  candidate.leftMinPwm = leftMinPwm;
  candidate.leftMaxPwm = leftMaxPwm;
  candidate.rightMinPwm = rightMinPwm;
  candidate.rightMaxPwm = rightMaxPwm;
  candidate.leftCompensation = leftCompensation;
  candidate.rightCompensation = rightCompensation;
  return commit(candidate);
}

}  // namespace ControlSettings
