#include "shared_state.h"

namespace {

SemaphoreHandle_t stateMutex = nullptr;
RobotState robotState;
RobotCommand robotCommand;

void withLock(void (*callback)()) {
  if (stateMutex == nullptr) {
    return;
  }
  if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
    callback();
    xSemaphoreGive(stateMutex);
  }
}

}  // namespace

namespace SharedState {

void begin() {
  stateMutex = xSemaphoreCreateMutex();
  robotState = RobotState{};
  robotCommand = RobotCommand{};
}

RobotState getState() {
  RobotState copy;
  if (stateMutex != nullptr && xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
    copy = robotState;
    xSemaphoreGive(stateMutex);
  }
  return copy;
}

void setState(const RobotState &state) {
  if (stateMutex != nullptr && xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
    robotState = state;
    xSemaphoreGive(stateMutex);
  }
}

RobotCommand consumeCommand() {
  RobotCommand copy;
  if (stateMutex != nullptr && xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
    copy = robotCommand;
    robotCommand.enableMotors = false;
    robotCommand.disableMotors = false;
    robotCommand.resetEncoders = false;
    robotCommand.calibrateGyro = false;
    robotCommand.calibrateVertical = false;
    robotCommand.otaStart = false;
    robotCommand.otaEnd = false;
    robotCommand.testLeftMotor = false;
    robotCommand.testRightMotor = false;
    robotCommand.testBothMotors = false;
    robotCommand.updatePidTunings = false;
    robotCommand.updatePidSetpoint = false;
    robotCommand.updatePidMaxPwm = false;
    robotCommand.updateMotorDeadzonePwm = false;
    robotCommand.updateIntegralLimit = false;
    robotCommand.updateITermLimit = false;
    robotCommand.updateIntegralEnabled = false;
    robotCommand.resetIntegral = false;
    robotCommand.updateEncoderSyncEnabled = false;
    robotCommand.updateEncoderSyncConfig = false;
    robotCommand.updateEncoderSyncTarget = false;
    robotCommand.updateGyroZHoldEnabled = false;
    robotCommand.updateGyroZHoldConfig = false;
    xSemaphoreGive(stateMutex);
  }
  return copy;
}

void requestEnableMotors() {
  withLock([]() {
    robotCommand.enableMotors = true;
    robotCommand.disableMotors = false;
    robotCommand.stopMotors = false;
  });
}

void requestDisableMotors() {
  withLock([]() {
    robotCommand.enableMotors = false;
    robotCommand.disableMotors = true;
    robotCommand.stopMotors = false;
    robotCommand.testLeftMotor = false;
    robotCommand.testRightMotor = false;
    robotCommand.testBothMotors = false;
  });
}

void requestStop() {
  withLock([]() {
    robotCommand.enableMotors = false;
    robotCommand.disableMotors = false;
    robotCommand.stopMotors = true;
    robotCommand.testLeftMotor = false;
    robotCommand.testRightMotor = false;
    robotCommand.testBothMotors = false;
  });
}

void requestEncoderReset() {
  withLock([]() { robotCommand.resetEncoders = true; });
}

void requestGyroCalibration() {
  withLock([]() { robotCommand.calibrateGyro = true; });
}

void requestVerticalCalibration() {
  withLock([]() { robotCommand.calibrateVertical = true; });
}

void requestOtaStart() {
  if (stateMutex != nullptr && xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
    robotCommand.otaStart = true;
    robotCommand.enableMotors = false;
    robotCommand.disableMotors = true;
    robotCommand.stopMotors = true;
    robotCommand.testLeftMotor = false;
    robotCommand.testRightMotor = false;
    robotCommand.testBothMotors = false;
    robotState.otaUpdating = true;
    xSemaphoreGive(stateMutex);
  }
}

void requestOtaEnd() {
  withLock([]() { robotCommand.otaEnd = true; });
}

void setOtaAvailable(bool available) {
  if (stateMutex != nullptr && xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
    robotState.otaAvailable = available;
    xSemaphoreGive(stateMutex);
  }
}

void setOtaUpdating(bool updating) {
  if (stateMutex != nullptr && xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
    robotState.otaUpdating = updating;
    xSemaphoreGive(stateMutex);
  }
}

bool isOtaUpdating() {
  bool updating = false;
  if (stateMutex != nullptr && xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
    updating = robotState.otaUpdating;
    xSemaphoreGive(stateMutex);
  }
  return updating;
}

void requestLeftMotorTest() {
  withLock([]() { robotCommand.testLeftMotor = true; });
}

void requestRightMotorTest() {
  withLock([]() { robotCommand.testRightMotor = true; });
}

void requestBothMotorsTest() {
  withLock([]() { robotCommand.testBothMotors = true; });
}

void requestPidTunings(double kp, double ki, double kd) {
  if (stateMutex != nullptr && xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
    robotCommand.pidKp = kp;
    robotCommand.pidKi = ki;
    robotCommand.pidKd = kd;
    robotCommand.updatePidTunings = true;
    xSemaphoreGive(stateMutex);
  }
}

void requestPidSetpoint(double setpoint) {
  if (stateMutex != nullptr && xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
    robotCommand.pidSetpoint = setpoint;
    robotCommand.updatePidSetpoint = true;
    xSemaphoreGive(stateMutex);
  }
}

void requestPidMaxPwm(int maxPwm) {
  if (stateMutex != nullptr && xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
    robotCommand.pidMaxPwm = maxPwm;
    robotCommand.updatePidMaxPwm = true;
    xSemaphoreGive(stateMutex);
  }
}

void requestMotorDeadzonePwm(int pwm) {
  if (stateMutex != nullptr && xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
    robotCommand.motorDeadzonePwm = pwm;
    robotCommand.updateMotorDeadzonePwm = true;
    xSemaphoreGive(stateMutex);
  }
}

void requestIntegralLimit(double limit) {
  if (stateMutex != nullptr && xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
    robotCommand.pidIntegralLimit = limit;
    robotCommand.updateIntegralLimit = true;
    xSemaphoreGive(stateMutex);
  }
}

void requestITermLimit(double limit) {
  if (stateMutex != nullptr && xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
    robotCommand.pidITermLimit = limit;
    robotCommand.updateITermLimit = true;
    xSemaphoreGive(stateMutex);
  }
}

void requestIntegralEnabled(bool enabled) {
  if (stateMutex != nullptr && xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
    robotCommand.pidIntegralEnabled = enabled;
    robotCommand.updateIntegralEnabled = true;
    xSemaphoreGive(stateMutex);
  }
}

void requestIntegralReset() {
  withLock([]() { robotCommand.resetIntegral = true; });
}

void requestEncoderSyncEnabled(bool enabled) {
  if (stateMutex != nullptr && xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
    robotCommand.encoderSyncEnabled = enabled;
    robotCommand.updateEncoderSyncEnabled = true;
    xSemaphoreGive(stateMutex);
  }
}

void requestEncoderSyncConfig(double kp, float deadband, int maxCorrection) {
  if (stateMutex != nullptr && xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
    robotCommand.encoderSyncKp = kp;
    robotCommand.encoderSyncDeadband = deadband;
    robotCommand.encoderSyncMaxCorrection = maxCorrection;
    robotCommand.updateEncoderSyncConfig = true;
    xSemaphoreGive(stateMutex);
  }
}

void requestEncoderSyncTarget(float targetDifference) {
  if (stateMutex != nullptr && xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
    robotCommand.encoderSyncTargetDifference = targetDifference;
    robotCommand.updateEncoderSyncTarget = true;
    xSemaphoreGive(stateMutex);
  }
}

void requestGyroZHoldEnabled(bool enabled) {
  if (stateMutex != nullptr && xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
    robotCommand.gyroZHoldEnabled = enabled;
    robotCommand.updateGyroZHoldEnabled = true;
    xSemaphoreGive(stateMutex);
  }
}

void requestGyroZHoldConfig(double kp, int maxCorrection) {
  if (stateMutex != nullptr && xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
    robotCommand.gyroZHoldKp = kp;
    robotCommand.gyroZHoldMaxCorrection = maxCorrection;
    robotCommand.updateGyroZHoldConfig = true;
    xSemaphoreGive(stateMutex);
  }
}

}  // namespace SharedState
