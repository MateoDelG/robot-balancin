#pragma once

#include "state_feedback.h"

namespace StateFeedbackSettings {

struct Settings {
  StateFeedback::Gains gains;
  float velocityFilterBeta;
  float angularAccelerationFilterBeta;
};

void begin(double defaultAngleGain, double defaultAngularVelocityGain);
Settings get();
bool save(const Settings &settings);
bool saveSnapshot(const Settings &settings);
bool loadSnapshot(Settings &settings);

}  // namespace StateFeedbackSettings
