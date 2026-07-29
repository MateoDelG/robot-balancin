#pragma once

namespace Imu9250FilterSettings {

void begin(float defaultAlpha);
float alpha();
bool wasLoaded();
bool save(float alpha);
bool clear(float defaultAlpha);

}  // namespace Imu9250FilterSettings
