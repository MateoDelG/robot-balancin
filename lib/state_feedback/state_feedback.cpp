#include "state_feedback.h"

#include <math.h>

#include "../../include/config.h"

namespace {

StateFeedback::Gains gains{};
StateFeedback::State state{};
float velocityBeta = Config::INITIAL_VELOCITY_FILTER_BETA;
float angularAccelerationBeta = Config::INITIAL_ANGULAR_ACCEL_FILTER_BETA;
float previousPosition = 0.0f;
float previousAngularVelocity = 0.0f;
bool initialized = false;
int maximumOutput = Config::SHADOW_PID_MAX_PWM;

float averageCounts(long leftCount, long rightCount) {
  return (static_cast<float>(leftCount) + static_cast<float>(rightCount)) * 0.5f;
}

}  // namespace

namespace StateFeedback {

void begin(const Gains &initialGains, float velocityFilterBeta,
           float angularAccelerationFilterBeta, int outputLimit) {
  setGains(initialGains);
  setFilters(velocityFilterBeta, angularAccelerationFilterBeta);
  setOutputLimit(outputLimit);
  state = State{};
  initialized = false;
}

void reset(long leftCount, long rightCount, float angularVelocityDegPerSec) {
  state = State{};
  previousPosition = averageCounts(leftCount, rightCount);
  previousAngularVelocity = isfinite(angularVelocityDegPerSec) ? angularVelocityDegPerSec : 0.0f;
  initialized = true;
}

void update(long leftCount, long rightCount, float angleDeg, float angleSetpointDeg,
            float angularVelocityDegPerSec, float dtSeconds) {
  const bool timingInvalid = dtSeconds <= 0.0f || dtSeconds > 0.1f;
  if (timingInvalid) {
    dtSeconds = static_cast<float>(Config::CONTROL_TASK_PERIOD_MS) / 1000.0f;
  }

  if (!isfinite(angleDeg) || !isfinite(angleSetpointDeg) ||
      !isfinite(angularVelocityDegPerSec)) {
    reset(leftCount, rightCount, 0.0f);
    state.output = 0;
    state.outputBeforeLimit = 0.0;
    return;
  }

  state.position = averageCounts(leftCount, rightCount);
  state.angleError = angleDeg - angleSetpointDeg;
  state.angularVelocity = angularVelocityDegPerSec;
  if (!initialized) {
    reset(leftCount, rightCount, angularVelocityDegPerSec);
    state.position = averageCounts(leftCount, rightCount);
    state.angleError = angleDeg - angleSetpointDeg;
    state.angularVelocity = angularVelocityDegPerSec;
  }

  if (timingInvalid) {
    previousPosition = state.position;
    previousAngularVelocity = state.angularVelocity;
    state.rawVelocity = 0.0f;
    state.velocity = 0.0f;
    state.rawAngularAcceleration = 0.0f;
    state.angularAcceleration = 0.0f;
  }

  state.rawVelocity = (state.position - previousPosition) / dtSeconds;
  state.rawAngularAcceleration =
      (state.angularVelocity - previousAngularVelocity) / dtSeconds;
  state.velocity += velocityBeta * (state.rawVelocity - state.velocity);
  state.angularAcceleration += angularAccelerationBeta *
                               (state.rawAngularAcceleration - state.angularAcceleration);
  previousPosition = state.position;
  previousAngularVelocity = state.angularVelocity;

  const double termLimit = static_cast<double>(maximumOutput);
  const double rawPositionTerm = -gains.position * state.position;
  const double rawVelocityTerm = -gains.velocity * state.velocity;
  const double rawAngleTerm = -gains.angle * state.angleError;
  const double rawAngularVelocityTerm = -gains.angularVelocity * state.angularVelocity;
  const double rawAngularAccelerationTerm =
      -gains.angularAcceleration * state.angularAcceleration;
  state.positionTerm = constrain(rawPositionTerm, -termLimit, termLimit);
  state.velocityTerm = constrain(rawVelocityTerm, -termLimit, termLimit);
  state.angleTerm = constrain(rawAngleTerm, -termLimit, termLimit);
  state.angularVelocityTerm = constrain(rawAngularVelocityTerm, -termLimit, termLimit);
  state.angularAccelerationTerm = constrain(rawAngularAccelerationTerm, -termLimit, termLimit);
  const bool termSaturated = fabs(rawPositionTerm - state.positionTerm) > 0.5 ||
                             fabs(rawVelocityTerm - state.velocityTerm) > 0.5 ||
                             fabs(rawAngleTerm - state.angleTerm) > 0.5 ||
                             fabs(rawAngularVelocityTerm - state.angularVelocityTerm) > 0.5 ||
                             fabs(rawAngularAccelerationTerm - state.angularAccelerationTerm) > 0.5;
  state.outputBeforeLimit = state.positionTerm + state.velocityTerm + state.angleTerm +
                            state.angularVelocityTerm + state.angularAccelerationTerm;
  if (!isfinite(state.outputBeforeLimit)) {
    state.outputBeforeLimit = 0.0;
    state.output = 0;
    return;
  }
  state.output = static_cast<int>(lround(constrain(
      state.outputBeforeLimit, -static_cast<double>(maximumOutput),
      static_cast<double>(maximumOutput))));
  state.saturated = termSaturated ||
                    fabs(state.outputBeforeLimit - static_cast<double>(state.output)) > 0.5;
  state.saturationCorrection = static_cast<double>(state.output) - state.outputBeforeLimit;
}

void setGains(const Gains &newGains) {
  gains = newGains;
}

void setFilters(float velocityFilterBeta, float angularAccelerationFilterBeta) {
  velocityBeta = constrain(velocityFilterBeta, Config::STATE_FILTER_BETA_MIN,
                           Config::STATE_FILTER_BETA_MAX);
  angularAccelerationBeta = constrain(angularAccelerationFilterBeta,
                                      Config::STATE_FILTER_BETA_MIN,
                                      Config::STATE_FILTER_BETA_MAX);
}

void setOutputLimit(int maxPwm) {
  maximumOutput = constrain(abs(maxPwm), Config::PID_MAX_PWM_MIN,
                            Config::PID_MAX_PWM_MAX);
}

Gains getGains() {
  return gains;
}

State getState() {
  return state;
}

float getVelocityFilterBeta() {
  return velocityBeta;
}

float getAngularAccelerationFilterBeta() {
  return angularAccelerationBeta;
}

}  // namespace StateFeedback
