#pragma once

#include <Arduino.h>

namespace StateFeedback {

struct Gains {
  double position;
  double velocity;
  double angle;
  double angularVelocity;
  double angularAcceleration;
};

struct State {
  float position;
  float rawVelocity;
  float velocity;
  float angleError;
  float angularVelocity;
  float rawAngularAcceleration;
  float angularAcceleration;
  double positionTerm;
  double velocityTerm;
  double angleTerm;
  double angularVelocityTerm;
  double angularAccelerationTerm;
  double outputBeforeLimit;
  int output;
  bool saturated;
  double saturationCorrection;
};

void begin(const Gains &initialGains, float velocityFilterBeta,
           float angularAccelerationFilterBeta, int outputLimit);
void reset(long leftCount, long rightCount, float angularVelocityDegPerSec);
void update(long leftCount, long rightCount, float angleDeg, float angleSetpointDeg,
            float angularVelocityDegPerSec, float dtSeconds);

void setGains(const Gains &newGains);
void setFilters(float velocityFilterBeta, float angularAccelerationFilterBeta);
void setOutputLimit(int maxPwm);

Gains getGains();
State getState();
float getVelocityFilterBeta();
float getAngularAccelerationFilterBeta();

}  // namespace StateFeedback
