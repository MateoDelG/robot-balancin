#pragma once

namespace ControlSettings {

struct Settings {
  double kp;
  double ki;
  double kd;
  double setpoint;
  int pidMaxPwm;
  int leftMinPwm;
  int leftMaxPwm;
  int rightMinPwm;
  int rightMaxPwm;
  double leftCompensation;
  double rightCompensation;
};

void begin(double defaultKp, double defaultKi, double defaultKd, double defaultSetpoint,
           int defaultPidMaxPwm);
Settings get();
bool savePid(double kp, double ki, double kd);
bool saveSetpoint(double setpoint);
bool savePidMaxPwm(int maxPwm);
bool saveMotorConfig(int leftMinPwm, int leftMaxPwm, int rightMinPwm, int rightMaxPwm,
                     double leftCompensation, double rightCompensation);

}  // namespace ControlSettings
