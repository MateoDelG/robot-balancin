#include "balance_pid.h"

#include <Arduino.h>
#include <math.h>

#include "../../include/config.h"

namespace {

double pidSetpoint = Config::INITIAL_ANGLE_SETPOINT_DEG;
double pidInput = 0.0;
double pidOutput = 0.0;
double pidKp = Config::INITIAL_PID_KP;
double pidKi = Config::INITIAL_PID_KI;
double pidKd = Config::INITIAL_PID_KD;
double error = 0.0;
double pTerm = 0.0;
double iTerm = 0.0;
double dTerm = 0.0;
double outputBeforeLimit = 0.0;
double outputAfterLimit = 0.0;
float integral = 0.0f;
double integralLimit = Config::INITIAL_INTEGRAL_LIMIT;
double iTermLimit = Config::INITIAL_I_TERM_LIMIT;
bool integralEnabled = true;
int outputMin = -Config::INITIAL_PID_MAX_PWM;
int outputMax = Config::INITIAL_PID_MAX_PWM;
int motorDeadzonePwm = Config::INITIAL_MOTOR_DEADZONE_PWM;

int clampMaxPwm(int maxPwm) {
  if (maxPwm < 0) {
    maxPwm = -maxPwm;
  }
  if (maxPwm > Config::PWM_MAX_DUTY) {
    maxPwm = Config::PWM_MAX_DUTY;
  }
  return maxPwm;
}

double applyMotorDeadzone(double output) {
  if (output == 0.0 || motorDeadzonePwm <= 0) {
    return output;
  }
  if (output > 0.0) {
    return output + static_cast<double>(motorDeadzonePwm);
  }
  return output - static_cast<double>(motorDeadzonePwm);
}

}  // namespace

namespace BalancePid {

void begin() {
  setOutputLimit(Config::INITIAL_PID_MAX_PWM);
}

void update(float angleDeg, float gyroRateDegPerSec, float dtSeconds, bool motorsEnabled,
            bool safetyStop) {
  if (dtSeconds <= 0.0f || dtSeconds > 1.0f) {
    dtSeconds = static_cast<float>(Config::CONTROL_TASK_PERIOD_MS) / 1000.0f;
  }

  pidInput = static_cast<double>(angleDeg);
  error = pidSetpoint - pidInput;
  pTerm = pidKp * error;
  dTerm = -pidKd * static_cast<double>(gyroRateDegPerSec);

  if (!motorsEnabled || safetyStop || fabs(error) > Config::MAX_SAFE_ANGLE_DEG) {
    resetIntegral();
  } else if (integralEnabled) {
    const double candidateIntegral = constrain(static_cast<double>(integral) + error * dtSeconds,
                                               -integralLimit, integralLimit);
    const double candidateITerm = constrain(pidKi * candidateIntegral, -iTermLimit, iTermLimit);
    double candidateOutput = pTerm + candidateITerm + dTerm;
    if (Config::INVERT_PID_OUTPUT) {
      candidateOutput = -candidateOutput;
    }

    const bool outputSaturatedHigh = candidateOutput >= static_cast<double>(outputMax);
    const bool outputSaturatedLow = candidateOutput <= static_cast<double>(outputMin);
    const double outputPushDirection = Config::INVERT_PID_OUTPUT ? -error : error;
    const bool pushesHigh = outputSaturatedHigh && outputPushDirection > 0.0;
    const bool pushesLow = outputSaturatedLow && outputPushDirection < 0.0;
    if (!pushesHigh && !pushesLow) {
      integral = static_cast<float>(candidateIntegral);
    }
  } else {
    integral = 0.0f;
  }
  iTerm = constrain(pidKi * static_cast<double>(integral), -iTermLimit, iTermLimit);

  double output = pTerm + iTerm + dTerm;
  if (Config::INVERT_PID_OUTPUT) {
    output = -output;
  }
  outputBeforeLimit = output;
  output = applyMotorDeadzone(output);
  pidOutput = constrain(output, static_cast<double>(outputMin), static_cast<double>(outputMax));
  outputAfterLimit = pidOutput;
}

void resetIntegral() {
  integral = 0.0f;
  iTerm = 0.0;
}

int getOutput() {
  return static_cast<int>(lround(pidOutput));
}

double getOutputRaw() {
  return pidOutput;
}

double getInput() {
  return pidInput;
}

double getSetpoint() {
  return pidSetpoint;
}

double getError() {
  return error;
}

double getRawError() {
  return error;
}

double getDeadbandedError() {
  return error;
}

double getPTerm() {
  return pTerm;
}

double getITerm() {
  return iTerm;
}

double getDTerm() {
  return dTerm;
}

double getOutputBeforeLimit() {
  return outputBeforeLimit;
}

double getOutputAfterLimit() {
  return outputAfterLimit;
}

double getKp() {
  return pidKp;
}

double getKi() {
  return pidKi;
}

double getKd() {
  return pidKd;
}

float getIntegral() {
  return integral;
}

double getIntegralLimit() {
  return integralLimit;
}

double getITermLimit() {
  return iTermLimit;
}

bool isIntegralEnabled() {
  return integralEnabled;
}

int getOutputMin() {
  return outputMin;
}

int getOutputMax() {
  return outputMax;
}

int getMotorDeadzonePwm() {
  return motorDeadzonePwm;
}

void setTunings(double kp, double ki, double kd) {
  pidKp = constrain(kp, Config::PID_KP_MIN, Config::PID_KP_MAX);
  pidKi = constrain(ki, Config::PID_KI_MIN, Config::PID_KI_MAX);
  pidKd = constrain(kd, Config::PID_KD_MIN, Config::PID_KD_MAX);
}

void setSetpoint(double setpointDeg) {
  pidSetpoint = setpointDeg;
}

void setOutputLimit(int maxPwm) {
  const int limit = clampMaxPwm(maxPwm);
  outputMin = -limit;
  outputMax = limit;
  pidOutput = constrain(pidOutput, static_cast<double>(outputMin), static_cast<double>(outputMax));
}

void setMotorDeadzonePwm(int pwm) {
  motorDeadzonePwm = constrain(abs(pwm), Config::MOTOR_DEADZONE_PWM_MIN,
                               Config::MOTOR_DEADZONE_PWM_MAX);
}

void setIntegralLimit(double limit) {
  integralLimit = constrain(fabs(limit), Config::INTEGRAL_LIMIT_MIN, Config::INTEGRAL_LIMIT_MAX);
  integral = static_cast<float>(constrain(static_cast<double>(integral), -integralLimit,
                                          integralLimit));
}

void setITermLimit(double limit) {
  iTermLimit = constrain(fabs(limit), Config::I_TERM_LIMIT_MIN, Config::I_TERM_LIMIT_MAX);
  iTerm = constrain(iTerm, -iTermLimit, iTermLimit);
}

void setIntegralEnabled(bool enabled) {
  integralEnabled = enabled;
  if (!integralEnabled) {
    resetIntegral();
  }
}

}  // namespace BalancePid
