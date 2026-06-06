#pragma once

namespace BalancePid {

void begin();
void update(float angleDeg, float gyroRateDegPerSec, float dtSeconds, bool motorsEnabled,
            bool safetyStop);
void resetIntegral();

int getOutput();
double getOutputRaw();
double getInput();
double getSetpoint();
double getError();
double getRawError();
double getDeadbandedError();
double getPTerm();
double getITerm();
double getDTerm();
double getOutputBeforeLimit();
double getOutputAfterLimit();
double getKp();
double getKi();
double getKd();
float getIntegral();
double getIntegralLimit();
double getITermLimit();
bool isIntegralEnabled();
int getOutputMin();
int getOutputMax();
int getMotorDeadzonePwm();

void setTunings(double kp, double ki, double kd);
void setSetpoint(double setpointDeg);
void setOutputLimit(int maxPwm);
void setMotorDeadzonePwm(int pwm);
void setIntegralLimit(double limit);
void setITermLimit(double limit);
void setIntegralEnabled(bool enabled);

}  // namespace BalancePid
