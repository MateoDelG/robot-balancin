#pragma once

#include <Arduino.h>

struct RobotState {
  float rawAx = 0.0f;
  float rawAy = 0.0f;
  float rawAz = 0.0f;
  float rawGx = 0.0f;
  float rawGy = 0.0f;
  float rawGz = 0.0f;
  float correctedGx = 0.0f;
  float correctedGy = 0.0f;
  float correctedGz = 0.0f;
  float angleAccelDeg = 0.0f;
  float angleFilteredDeg = 0.0f;
  float angleComplementaryDeg = 0.0f;
  float angleKalmanDeg = 0.0f;
  float selectedAngleDeg = 0.0f;
  float gyroRateDegPerSec = 0.0f;

  long rawLeftEncoder = 0;
  long rawRightEncoder = 0;
  long correctedLeftEncoder = 0;
  long correctedRightEncoder = 0;
  float leftSpeed = 0.0f;
  float rightSpeed = 0.0f;
  float speedAverage = 0.0f;
  float speedDifference = 0.0f;
  float encoderSyncError = 0.0f;
  int encoderSyncCorrection = 0;
  bool encoderSyncEnabled = false;
  double encoderSyncKp = 0.0;
  float encoderSyncDeadband = 0.0f;
  int encoderSyncMaxCorrection = 0;
  float encoderSyncTargetDifference = 0.0f;

  int leftPwm = 0;
  int rightPwm = 0;
  int balancePwm = 0;

  double pidSetpoint = 0.0;
  double pidInput = 0.0;
  double pidError = 0.0;
  double pidOutput = 0.0;
  double pidPTerm = 0.0;
  double pidITerm = 0.0;
  double pidDTerm = 0.0;
  double pidOutputBeforeLimit = 0.0;
  double pidOutputAfterLimit = 0.0;
  double pidKp = 0.0;
  double pidKi = 0.0;
  double pidKd = 0.0;
  float pidIntegral = 0.0f;
  double pidIntegralLimit = 0.0;
  double pidITermLimit = 0.0;
  bool pidIntegralEnabled = true;
  int pidOutputMin = 0;
  int pidOutputMax = 0;
  int motorDeadzonePwm = 0;
  uint32_t controlPeriodMs = 0;

  bool imuReady = false;
  bool gyroCalibrated = false;
  bool angleInitialized = false;
  bool motorsEnabled = false;
  bool safetyStop = true;
  bool safetyFault = true;
  bool otaAvailable = false;
  bool otaUpdating = false;
  char faultMessage[80] = "Boot";
};

struct RobotCommand {
  bool enableMotors = false;
  bool disableMotors = false;
  bool stopMotors = true;
  bool resetEncoders = false;
  bool calibrateGyro = false;
  bool calibrateVertical = false;
  bool otaStart = false;
  bool otaEnd = false;
  bool testLeftMotor = false;
  bool testRightMotor = false;
  bool testBothMotors = false;
  bool updatePidTunings = false;
  bool updatePidSetpoint = false;
  bool updatePidMaxPwm = false;
  bool updateMotorDeadzonePwm = false;
  bool updateIntegralLimit = false;
  bool updateITermLimit = false;
  bool updateIntegralEnabled = false;
  bool resetIntegral = false;
  bool updateEncoderSyncEnabled = false;
  bool updateEncoderSyncConfig = false;
  bool updateEncoderSyncTarget = false;
  double pidKp = 0.0;
  double pidKi = 0.0;
  double pidKd = 0.0;
  double pidSetpoint = 0.0;
  double pidIntegralLimit = 0.0;
  double pidITermLimit = 0.0;
  bool pidIntegralEnabled = true;
  bool encoderSyncEnabled = false;
  double encoderSyncKp = 0.0;
  float encoderSyncDeadband = 0.0f;
  int encoderSyncMaxCorrection = 0;
  float encoderSyncTargetDifference = 0.0f;
  int pidMaxPwm = 0;
  int motorDeadzonePwm = 0;
};

namespace SharedState {

void begin();

RobotState getState();
void setState(const RobotState &state);
RobotCommand consumeCommand();

void requestEnableMotors();
void requestDisableMotors();
void requestStop();
void requestEncoderReset();
void requestGyroCalibration();
void requestVerticalCalibration();
void requestOtaStart();
void requestOtaEnd();
void setOtaAvailable(bool available);
void setOtaUpdating(bool updating);
bool isOtaUpdating();
void requestLeftMotorTest();
void requestRightMotorTest();
void requestBothMotorsTest();
void requestPidTunings(double kp, double ki, double kd);
void requestPidSetpoint(double setpoint);
void requestPidMaxPwm(int maxPwm);
void requestMotorDeadzonePwm(int pwm);
void requestIntegralLimit(double limit);
void requestITermLimit(double limit);
void requestIntegralEnabled(bool enabled);
void requestIntegralReset();
void requestEncoderSyncEnabled(bool enabled);
void requestEncoderSyncConfig(double kp, float deadband, int maxCorrection);
void requestEncoderSyncTarget(float targetDifference);

}  // namespace SharedState
