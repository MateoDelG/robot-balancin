#include "imu9250_filter_settings.h"

#include <Preferences.h>
#include <math.h>

namespace {

constexpr char NVS_NAMESPACE[] = "imu9250flt";
constexpr char NVS_ALPHA_KEY[] = "alpha";
float currentAlpha = 0.98f;
bool loaded = false;

bool validAlpha(float alpha) {
  return isfinite(alpha) && alpha >= 0.80f && alpha <= 0.999f;
}

}  // namespace

namespace Imu9250FilterSettings {

void begin(float defaultAlpha) {
  currentAlpha = defaultAlpha;
  loaded = false;
  Preferences preferences;
  if (!preferences.begin(NVS_NAMESPACE, true)) return;
  const float stored = preferences.getFloat(NVS_ALPHA_KEY, NAN);
  preferences.end();
  if (validAlpha(stored)) {
    currentAlpha = stored;
    loaded = true;
  }
}

float alpha() {
  return currentAlpha;
}

bool wasLoaded() {
  return loaded;
}

bool save(float alpha) {
  if (!validAlpha(alpha)) return false;
  Preferences preferences;
  if (!preferences.begin(NVS_NAMESPACE, false)) return false;
  const bool success = preferences.putFloat(NVS_ALPHA_KEY, alpha) == sizeof(float);
  preferences.end();
  if (success) {
    currentAlpha = alpha;
    loaded = true;
  }
  return success;
}

bool clear(float defaultAlpha) {
  Preferences preferences;
  if (!preferences.begin(NVS_NAMESPACE, false)) return false;
  preferences.remove(NVS_ALPHA_KEY);
  preferences.end();
  currentAlpha = defaultAlpha;
  loaded = false;
  return true;
}

}  // namespace Imu9250FilterSettings
