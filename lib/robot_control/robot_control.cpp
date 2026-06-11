#include "robot_control.h"

#include <Arduino.h>
#include <math.h>
#include <string.h>

#include "balance_pid.h"
#include "encoders_test.h"
#include "imu6500_test.h"
#include "motors_test.h"
#include "shared_state.h"
#include "../../include/config.h"

namespace {

bool motorsEnabled = false;
bool safetyStop = true;
bool safetyFault = true;
char faultMessage[80] = "Boot";
unsigned long lastPidPrintMs = 0;
unsigned long previousControlMs = 0;
unsigned long previousEncoderMs = 0;
long previousLeftEncoder = 0;
long previousRightEncoder = 0;
float leftSpeed = 0.0f;
float rightSpeed = 0.0f;
float averageSpeed = 0.0f;
float speedDifference = 0.0f;
float encoderSyncError = 0.0f;
bool encoderSyncEnabled = Config::INITIAL_ENCODER_SYNC_ENABLED;
double encoderSyncKp = Config::INITIAL_ENCODER_SYNC_KP;
float encoderSyncDeadband = Config::INITIAL_ENCODER_SYNC_DEADBAND;
int encoderSyncMaxCorrection = Config::INITIAL_ENCODER_SYNC_MAX_CORRECTION;
float encoderSyncTargetDifference = Config::INITIAL_ENCODER_SYNC_TARGET_DIFFERENCE;
int encoderSyncCorrection = 0;
bool gyroZHoldEnabled = Config::INITIAL_GYRO_Z_HOLD_ENABLED;
double gyroZHoldKp = Config::INITIAL_GYRO_Z_HOLD_KP;
int gyroZHoldMaxCorrection = Config::INITIAL_GYRO_Z_HOLD_MAX_CORRECTION;
int gyroZHoldCorrection = 0;
bool speedHoldEnabled = Config::INITIAL_SPEED_HOLD_ENABLED;
double speedHoldKp = Config::INITIAL_SPEED_HOLD_KP;
double speedHoldMaxAngleDeg = Config::INITIAL_SPEED_HOLD_MAX_ANGLE_DEG;
double manualAngleSetpointDeg = Config::INITIAL_ANGLE_SETPOINT_DEG;
double speedHoldAngleCorrectionDeg = 0.0;
float targetDriveForward = 0.0f;
float targetDriveTurn = 0.0f;
float currentDriveForward = 0.0f;
float currentDriveTurn = 0.0f;
float driveAngleOffsetDeg = 0.0f;
int driveTurnPwm = 0;
unsigned long lastDriveCommandMs = 0;
int balancePwm = 0;
int finalLeftPwm = 0;
int finalRightPwm = 0;

enum class RecoveryState {
  WaitingUpright,
  Settling,
  Calibrating,
  Running,
};

RecoveryState recoveryState = RecoveryState::WaitingUpright;
unsigned long recoveryStableStartMs = 0;
unsigned long recoveryStableMs = 0;
bool autoRecoveryCalibrating = false;

const char *recoveryStateText() {
  switch (recoveryState) {
    case RecoveryState::WaitingUpright:
      return "Waiting";
    case RecoveryState::Settling:
      return "Settling";
    case RecoveryState::Calibrating:
      return "Calibrating";
    case RecoveryState::Running:
      return "Running";
    default:
      return "Unknown";
  }
}

void clearDriveCommand() {
  targetDriveForward = 0.0f;
  targetDriveTurn = 0.0f;
  currentDriveForward = 0.0f;
  currentDriveTurn = 0.0f;
  driveAngleOffsetDeg = 0.0f;
  driveTurnPwm = 0;
}

float turnRateFromGyroZ(const Imu6500Test::ImuSample &imu) {
  float turnRate = imu.correctedGz * RAD_TO_DEG;
  if (Config::INVERT_TURN_GYRO) {
    turnRate = -turnRate;
  }
  return turnRate;
}

const char *turnDirectionFromRate(float turnRateDegPerSec) {
  if (fabsf(turnRateDegPerSec) < Config::TURN_GYRO_DEADBAND_DPS) {
    return "quieto";
  }
  return turnRateDegPerSec > 0.0f ? "derecha" : "izquierda";
}

int gyroZHoldCorrectionFromRate(float turnRateDegPerSec) {
  if (!gyroZHoldEnabled || !motorsEnabled || safetyStop || safetyFault) {
    return 0;
  }
  if (fabsf(turnRateDegPerSec) < Config::GYRO_Z_HOLD_DEADBAND_DPS) {
    return 0;
  }

  const int correction = static_cast<int>(lround(gyroZHoldKp * turnRateDegPerSec));
  return constrain(correction, -gyroZHoldMaxCorrection, gyroZHoldMaxCorrection);
}

float approachValue(float current, float target, float step) {
  if (current < target) {
    return min(current + step, target);
  }
  if (current > target) {
    return max(current - step, target);
  }
  return current;
}

void updateDriveCommandState() {
  const unsigned long now = millis();
  if (!motorsEnabled || safetyStop || safetyFault ||
      now - lastDriveCommandMs > Config::DRIVE_COMMAND_TIMEOUT_MS) {
    targetDriveForward = 0.0f;
    targetDriveTurn = 0.0f;
  }

  currentDriveForward = approachValue(currentDriveForward, targetDriveForward,
                                      Config::DRIVE_COMMAND_STEP);
  currentDriveTurn = approachValue(currentDriveTurn, targetDriveTurn,
                                   Config::DRIVE_COMMAND_STEP);

  float driveForward = currentDriveForward;
  float driveTurn = currentDriveTurn;
  if (Config::INVERT_DRIVE_FORWARD) {
    driveForward = -driveForward;
  }
  if (Config::INVERT_DRIVE_TURN) {
    driveTurn = -driveTurn;
  }

  driveAngleOffsetDeg = driveForward * Config::INITIAL_MAX_DRIVE_ANGLE_DEG;
  driveTurnPwm = static_cast<int>(lround(driveTurn * Config::INITIAL_MAX_DRIVE_TURN_PWM));
}

void updateBalanceSetpointFromSpeed() {
  speedHoldAngleCorrectionDeg = 0.0;
  if (speedHoldEnabled && motorsEnabled && !safetyStop && !safetyFault) {
    float activeSpeed = averageSpeed;
    if (fabsf(activeSpeed) < Config::SPEED_HOLD_DEADBAND_COUNTS_PER_SEC) {
      activeSpeed = 0.0f;
    }

    speedHoldAngleCorrectionDeg = -speedHoldKp * static_cast<double>(activeSpeed);
    if (Config::INVERT_SPEED_HOLD_CORRECTION) {
      speedHoldAngleCorrectionDeg = -speedHoldAngleCorrectionDeg;
    }
    speedHoldAngleCorrectionDeg = constrain(speedHoldAngleCorrectionDeg,
                                            -speedHoldMaxAngleDeg,
                                            speedHoldMaxAngleDeg);
  }

  const double setpoint = constrain(manualAngleSetpointDeg + speedHoldAngleCorrectionDeg +
                                        static_cast<double>(driveAngleOffsetDeg),
                                    Config::SETPOINT_MIN_DEG,
                                    Config::SETPOINT_MAX_DEG);
  BalancePid::setSetpoint(setpoint);
}

void setFault(const char *message) {
  safetyStop = true;
  safetyFault = true;
  motorsEnabled = false;
  strlcpy(faultMessage, message, sizeof(faultMessage));
}

void clearFault() {
  safetyStop = false;
  safetyFault = false;
  strlcpy(faultMessage, "OK", sizeof(faultMessage));
}

void enterRecoveryWaiting(const char *message) {
  recoveryState = RecoveryState::WaitingUpright;
  recoveryStableStartMs = 0;
  recoveryStableMs = 0;
  autoRecoveryCalibrating = false;
  clearDriveCommand();
  BalancePid::resetIntegral();
  MotorsTest::disable();
  motorsEnabled = false;
  safetyStop = true;
  safetyFault = true;
  strlcpy(faultMessage, message, sizeof(faultMessage));
}

bool isUprightForRecovery(const Imu6500Test::ImuSample &imu) {
  return imu.angleInitialized &&
         fabsf(imu.selectedAngleDeg) <= Config::AUTO_RECOVERY_ANGLE_WINDOW_DEG;
}

void runAutoRecovery(const Imu6500Test::ImuSample &imu) {
  if (!Config::AUTO_RECOVERY_ENABLED) {
    return;
  }

  const unsigned long now = millis();

  if (SharedState::isOtaUpdating()) {
    enterRecoveryWaiting("OTA update in progress");
    return;
  }

  if (!Imu6500Test::isReady()) {
    enterRecoveryWaiting("IMU not ready");
    return;
  }

  const bool upright = isUprightForRecovery(imu);
  const bool safeAngle = imu.angleInitialized &&
                         fabsf(imu.selectedAngleDeg) <= Config::MAX_SAFE_ANGLE_DEG;

  if (recoveryState == RecoveryState::Running &&
      (!imu.gyroCalibrated || !imu.angleInitialized)) {
    enterRecoveryWaiting("IMU not calibrated");
    return;
  }

  if (recoveryState == RecoveryState::Running && !safeAngle) {
    enterRecoveryWaiting("Angle exceeds safe limit");
    return;
  }

  switch (recoveryState) {
    case RecoveryState::WaitingUpright:
      MotorsTest::disable();
      motorsEnabled = false;
      safetyStop = true;
      safetyFault = true;
      clearDriveCommand();
      if (upright) {
        recoveryStableStartMs = now;
        recoveryStableMs = 0;
        recoveryState = RecoveryState::Settling;
        strlcpy(faultMessage, "Auto recovery settling", sizeof(faultMessage));
      } else {
        strlcpy(faultMessage, "Waiting upright", sizeof(faultMessage));
      }
      break;

    case RecoveryState::Settling:
      MotorsTest::disable();
      motorsEnabled = false;
      safetyStop = true;
      safetyFault = true;
      clearDriveCommand();
      if (!upright) {
        recoveryState = RecoveryState::WaitingUpright;
        recoveryStableStartMs = 0;
        recoveryStableMs = 0;
        strlcpy(faultMessage, "Waiting upright", sizeof(faultMessage));
        break;
      }
      recoveryStableMs = now - recoveryStableStartMs;
      if (recoveryStableMs >= Config::AUTO_RECOVERY_SETTLE_MS) {
        recoveryState = RecoveryState::Calibrating;
      }
      break;

    case RecoveryState::Calibrating:
      autoRecoveryCalibrating = true;
      MotorsTest::disable();
      motorsEnabled = false;
      safetyStop = true;
      safetyFault = true;
      clearDriveCommand();
      BalancePid::resetIntegral();
      strlcpy(faultMessage, "Auto calibrating", sizeof(faultMessage));

      if (!Imu6500Test::calibrateGyro()) {
        enterRecoveryWaiting("Gyro calibration failed");
        break;
      }

      for (uint8_t sample = 0; sample < 5; ++sample) {
        Imu6500Test::update();
        vTaskDelay(pdMS_TO_TICKS(Config::CONTROL_TASK_PERIOD_MS));
      }
      Imu6500Test::calibrateVerticalAngle();
      BalancePid::resetIntegral();
      clearDriveCommand();
      clearFault();
      motorsEnabled = true;
      autoRecoveryCalibrating = false;
      recoveryStableMs = Config::AUTO_RECOVERY_SETTLE_MS;
      recoveryState = RecoveryState::Running;
      break;

    case RecoveryState::Running:
      clearFault();
      break;
  }
}

void updateEncoderDiagnostics() {
  const unsigned long now = millis();
  const long left = EncodersTest::leftCount();
  const long right = EncodersTest::rightCount();

  if (previousEncoderMs == 0) {
    previousEncoderMs = now;
    previousLeftEncoder = left;
    previousRightEncoder = right;
    return;
  }

  const unsigned long elapsedMs = now - previousEncoderMs;
  if (elapsedMs == 0) {
    return;
  }

  const float dt = static_cast<float>(elapsedMs) / 1000.0f;
  leftSpeed = static_cast<float>(left - previousLeftEncoder) / dt;
  rightSpeed = static_cast<float>(right - previousRightEncoder) / dt;
  averageSpeed = (leftSpeed + rightSpeed) * 0.5f;
  speedDifference = leftSpeed - rightSpeed;
  previousLeftEncoder = left;
  previousRightEncoder = right;
  previousEncoderMs = now;
}

void fillSharedState() {
  const Imu6500Test::ImuSample imu = Imu6500Test::getSample();
  const RobotState previousState = SharedState::getState();
  const float turnRate = turnRateFromGyroZ(imu);

  RobotState state;
  state.rawAx = imu.rawAx;
  state.rawAy = imu.rawAy;
  state.rawAz = imu.rawAz;
  state.rawGx = imu.rawGx;
  state.rawGy = imu.rawGy;
  state.rawGz = imu.rawGz;
  state.correctedGx = imu.correctedGx;
  state.correctedGy = imu.correctedGy;
  state.correctedGz = imu.correctedGz;
  state.angleAccelDeg = imu.angleAccelDeg;
  state.angleFilteredDeg = imu.angleFilteredDeg;
  state.angleComplementaryDeg = imu.angleComplementaryDeg;
  state.angleKalmanDeg = imu.angleKalmanDeg;
  state.selectedAngleDeg = imu.selectedAngleDeg;
  state.gyroRateDegPerSec = imu.gyroRateDegPerSec;
  state.turnRateDegPerSec = turnRate;
  strlcpy(state.turnDirection, turnDirectionFromRate(turnRate), sizeof(state.turnDirection));
  state.rawLeftEncoder = EncodersTest::rawLeftCount();
  state.rawRightEncoder = EncodersTest::rawRightCount();
  state.correctedLeftEncoder = EncodersTest::leftCount();
  state.correctedRightEncoder = EncodersTest::rightCount();
  state.leftSpeed = leftSpeed;
  state.rightSpeed = rightSpeed;
  state.speedAverage = averageSpeed;
  state.speedDifference = speedDifference;
  state.encoderSyncError = encoderSyncError;
  state.encoderSyncCorrection = encoderSyncCorrection;
  state.encoderSyncEnabled = encoderSyncEnabled;
  state.encoderSyncKp = encoderSyncKp;
  state.encoderSyncDeadband = encoderSyncDeadband;
  state.encoderSyncMaxCorrection = encoderSyncMaxCorrection;
  state.encoderSyncTargetDifference = encoderSyncTargetDifference;
  state.gyroZHoldEnabled = gyroZHoldEnabled;
  state.gyroZHoldKp = gyroZHoldKp;
  state.gyroZHoldDeadband = Config::GYRO_Z_HOLD_DEADBAND_DPS;
  state.gyroZHoldMaxCorrection = gyroZHoldMaxCorrection;
  state.gyroZHoldCorrection = gyroZHoldCorrection;
  state.speedHoldEnabled = speedHoldEnabled;
  state.speedHoldKp = speedHoldKp;
  state.speedHoldDeadband = Config::SPEED_HOLD_DEADBAND_COUNTS_PER_SEC;
  state.speedHoldMaxAngleDeg = speedHoldMaxAngleDeg;
  state.speedHoldAngleCorrectionDeg = speedHoldAngleCorrectionDeg;
  state.driveForward = currentDriveForward;
  state.driveTurn = currentDriveTurn;
  state.driveAngleOffsetDeg = driveAngleOffsetDeg;
  state.driveTurnPwm = driveTurnPwm;
  state.driveCommandActive = targetDriveForward != 0.0f || targetDriveTurn != 0.0f ||
                             currentDriveForward != 0.0f || currentDriveTurn != 0.0f;
  state.autoRecoveryEnabled = Config::AUTO_RECOVERY_ENABLED;
  state.autoRecoveryWaiting = recoveryState == RecoveryState::WaitingUpright ||
                              recoveryState == RecoveryState::Settling;
  state.autoRecoveryCalibrating = autoRecoveryCalibrating;
  state.autoRecoveryStableMs = recoveryStableMs;
  strlcpy(state.autoRecoveryState, recoveryStateText(), sizeof(state.autoRecoveryState));
  state.leftPwm = MotorsTest::getLeftPwm();
  state.rightPwm = MotorsTest::getRightPwm();
  state.balancePwm = balancePwm;
  state.pidSetpoint = BalancePid::getSetpoint();
  state.pidInput = BalancePid::getInput();
  state.pidError = BalancePid::getError();
  state.pidOutput = BalancePid::getOutputRaw();
  state.pidPTerm = BalancePid::getPTerm();
  state.pidITerm = BalancePid::getITerm();
  state.pidDTerm = BalancePid::getDTerm();
  state.pidOutputBeforeLimit = BalancePid::getOutputBeforeLimit();
  state.pidOutputAfterLimit = BalancePid::getOutputAfterLimit();
  state.pidKp = BalancePid::getKp();
  state.pidKi = BalancePid::getKi();
  state.pidKd = BalancePid::getKd();
  state.pidIntegral = BalancePid::getIntegral();
  state.pidIntegralLimit = BalancePid::getIntegralLimit();
  state.pidITermLimit = BalancePid::getITermLimit();
  state.pidIntegralEnabled = BalancePid::isIntegralEnabled();
  state.pidOutputMin = BalancePid::getOutputMin();
  state.pidOutputMax = BalancePid::getOutputMax();
  state.motorDeadzonePwm = BalancePid::getMotorDeadzonePwm();
  state.controlPeriodMs = Config::CONTROL_TASK_PERIOD_MS;
  state.imuReady = Imu6500Test::isReady();
  state.gyroCalibrated = imu.gyroCalibrated;
  state.angleInitialized = imu.angleInitialized;
  state.motorsEnabled = motorsEnabled;
  state.safetyStop = safetyStop;
  state.safetyFault = safetyFault;
  state.otaAvailable = previousState.otaAvailable;
  state.otaUpdating = previousState.otaUpdating;
  strlcpy(state.faultMessage, faultMessage, sizeof(state.faultMessage));
  SharedState::setState(state);
}

bool canRunMotorTest() {
  if (SharedState::isOtaUpdating()) {
    Serial.println(F("Motor test ignored: OTA update in progress"));
    return false;
  }
  if (!motorsEnabled || safetyStop) {
    Serial.println(F("Motor test ignored: motors are not enabled"));
    return false;
  }
  return true;
}

void handleCommand(const RobotCommand &command) {
  if (command.otaStart) {
    setFault("OTA update in progress");
    MotorsTest::disable();
  }

  if (command.otaEnd) {
    motorsEnabled = false;
    safetyStop = true;
    safetyFault = true;
    MotorsTest::disable();
    strlcpy(faultMessage, "OTA complete", sizeof(faultMessage));
  }

  if (command.stopMotors && !command.otaStart) {
    enterRecoveryWaiting("Stop requested");
  }

  if (command.disableMotors) {
    enterRecoveryWaiting("Motors disabled");
  }

  if (command.updatePidTunings) {
    BalancePid::setTunings(command.pidKp, command.pidKi, command.pidKd);
  }

  if (command.updatePidSetpoint) {
    manualAngleSetpointDeg = command.pidSetpoint;
    BalancePid::setSetpoint(manualAngleSetpointDeg);
  }

  if (command.updatePidMaxPwm) {
    BalancePid::setOutputLimit(command.pidMaxPwm);
  }

  if (command.updateMotorDeadzonePwm) {
    BalancePid::setMotorDeadzonePwm(command.motorDeadzonePwm);
  }

  if (command.updateIntegralLimit) {
    BalancePid::setIntegralLimit(command.pidIntegralLimit);
  }

  if (command.updateITermLimit) {
    BalancePid::setITermLimit(command.pidITermLimit);
  }

  if (command.updateIntegralEnabled) {
    BalancePid::setIntegralEnabled(command.pidIntegralEnabled);
  }

  if (command.resetIntegral) {
    BalancePid::resetIntegral();
  }

  if (command.updateEncoderSyncEnabled) {
    encoderSyncEnabled = command.encoderSyncEnabled;
  }

  if (command.updateEncoderSyncConfig) {
    encoderSyncKp = constrain(command.encoderSyncKp, Config::ENCODER_SYNC_KP_MIN,
                              Config::ENCODER_SYNC_KP_MAX);
    encoderSyncDeadband = constrain(command.encoderSyncDeadband,
                                    Config::ENCODER_SYNC_DEADBAND_MIN,
                                    Config::ENCODER_SYNC_DEADBAND_MAX);
    encoderSyncMaxCorrection = constrain(command.encoderSyncMaxCorrection,
                                         Config::ENCODER_SYNC_MAX_CORRECTION_MIN,
                                         Config::ENCODER_SYNC_MAX_CORRECTION_MAX);
  }

  if (command.updateEncoderSyncTarget) {
    encoderSyncTargetDifference = constrain(command.encoderSyncTargetDifference,
                                            Config::ENCODER_SYNC_TARGET_DIFFERENCE_MIN,
                                            Config::ENCODER_SYNC_TARGET_DIFFERENCE_MAX);
  }

  if (command.updateGyroZHoldEnabled) {
    gyroZHoldEnabled = command.gyroZHoldEnabled;
  }

  if (command.updateGyroZHoldConfig) {
    gyroZHoldKp = constrain(command.gyroZHoldKp, Config::GYRO_Z_HOLD_KP_MIN,
                            Config::GYRO_Z_HOLD_KP_MAX);
    gyroZHoldMaxCorrection = constrain(command.gyroZHoldMaxCorrection,
                                       Config::GYRO_Z_HOLD_MAX_CORRECTION_MIN,
                                       Config::GYRO_Z_HOLD_MAX_CORRECTION_MAX);
  }

  if (command.updateSpeedHoldEnabled) {
    speedHoldEnabled = command.speedHoldEnabled;
  }

  if (command.updateSpeedHoldConfig) {
    speedHoldKp = constrain(command.speedHoldKp, Config::SPEED_HOLD_KP_MIN,
                            Config::SPEED_HOLD_KP_MAX);
    speedHoldMaxAngleDeg = constrain(command.speedHoldMaxAngleDeg,
                                     Config::SPEED_HOLD_MAX_ANGLE_MIN_DEG,
                                     Config::SPEED_HOLD_MAX_ANGLE_MAX_DEG);
  }

  if (command.updateDriveCommand) {
    targetDriveForward = constrain(command.driveForward, -1.0f, 1.0f);
    targetDriveTurn = constrain(command.driveTurn, -1.0f, 1.0f);
    lastDriveCommandMs = millis();
  }

  if (command.resetEncoders) {
    EncodersTest::reset();
    Serial.println(F("Encoder counts reset by command"));
  }

  if (command.calibrateGyro) {
    MotorsTest::disable();
    Imu6500Test::calibrateGyro();
  }

  if (command.calibrateVertical) {
    MotorsTest::disable();
    Imu6500Test::calibrateVerticalAngle();
  }

  if (command.enableMotors) {
    if (SharedState::isOtaUpdating()) {
      enterRecoveryWaiting("OTA update in progress");
      return;
    }
    motorsEnabled = true;
    recoveryState = RecoveryState::Running;
    clearFault();
  }

  if (command.testLeftMotor && canRunMotorTest()) {
    MotorsTest::runLeftTest();
  }

  if (command.testRightMotor && canRunMotorTest()) {
    MotorsTest::runRightTest();
  }

  if (command.testBothMotors && canRunMotorTest()) {
    MotorsTest::runBothTest();
  }
}

void updateSafety(const Imu6500Test::ImuSample &imu) {
  if (SharedState::isOtaUpdating()) {
    setFault("OTA update in progress");
  } else if (!Imu6500Test::isReady()) {
    setFault("IMU not ready");
  } else if (!imu.gyroCalibrated) {
    setFault("Gyro not calibrated");
  } else if (!imu.angleInitialized) {
    setFault("Angle not initialized");
  } else if (fabsf(imu.selectedAngleDeg) > Config::MAX_SAFE_ANGLE_DEG) {
    setFault("Angle exceeds safe limit");
  } else if (!safetyFault || motorsEnabled) {
    clearFault();
  }
}

void applyPidToMotors() {
  balancePwm = BalancePid::getOutput();
  encoderSyncCorrection = 0;
  gyroZHoldCorrection = 0;
  encoderSyncError = speedDifference - encoderSyncTargetDifference;
  if (encoderSyncEnabled && motorsEnabled && !safetyStop && !safetyFault) {
    float activeSyncError = encoderSyncError;
    if (fabsf(activeSyncError) < encoderSyncDeadband) {
      activeSyncError = 0.0f;
    }
    encoderSyncCorrection = constrain(static_cast<int>(lround(encoderSyncKp * activeSyncError)),
                                      -encoderSyncMaxCorrection,
                                      encoderSyncMaxCorrection);
  }
  const Imu6500Test::ImuSample imu = Imu6500Test::getSample();
  gyroZHoldCorrection = gyroZHoldCorrectionFromRate(turnRateFromGyroZ(imu));
  finalLeftPwm = balancePwm - encoderSyncCorrection - gyroZHoldCorrection - driveTurnPwm;
  finalRightPwm = balancePwm + encoderSyncCorrection + gyroZHoldCorrection + driveTurnPwm;

  if (motorsEnabled && !safetyStop && !safetyFault) {
    MotorsTest::setLeftPwm(finalLeftPwm);
    MotorsTest::setRightPwm(finalRightPwm);
  } else {
    BalancePid::resetIntegral();
    MotorsTest::disable();
  }
}

void printPidIfDue() {
  const unsigned long now = millis();
  if (now - lastPidPrintMs < Config::PID_SERIAL_PRINT_INTERVAL_MS) {
    return;
  }
  lastPidPrintMs = now;

  const Imu6500Test::ImuSample imu = Imu6500Test::getSample();
  Serial.print(F("PID angle="));
  Serial.print(imu.selectedAngleDeg, 2);
  Serial.print(F(" gyroRate="));
  Serial.print(imu.gyroRateDegPerSec, 2);
  Serial.print(F(" setpoint="));
  Serial.print(BalancePid::getSetpoint(), 2);
  Serial.print(F(" error="));
  Serial.print(BalancePid::getError(), 2);
  Serial.print(F(" P="));
  Serial.print(BalancePid::getPTerm(), 2);
  Serial.print(F(" I="));
  Serial.print(BalancePid::getITerm(), 2);
  Serial.print(F(" D="));
  Serial.print(BalancePid::getDTerm(), 2);
  Serial.print(F(" integral="));
  Serial.print(BalancePid::getIntegral(), 4);
  Serial.print(F(" integralLimit="));
  Serial.print(BalancePid::getIntegralLimit(), 4);
  Serial.print(F(" iTermLimit="));
  Serial.print(BalancePid::getITermLimit(), 2);
  Serial.print(F(" beforeLimit="));
  Serial.print(BalancePid::getOutputBeforeLimit(), 2);
  Serial.print(F(" afterLimit="));
  Serial.print(BalancePid::getOutputAfterLimit(), 2);
  Serial.print(F(" output="));
  Serial.print(BalancePid::getOutput());
  Serial.print(F(" speedDiff="));
  Serial.print(speedDifference, 2);
  Serial.print(F(" speedHoldAngle="));
  Serial.print(speedHoldAngleCorrectionDeg, 3);
  Serial.print(F(" driveF="));
  Serial.print(currentDriveForward, 2);
  Serial.print(F(" driveT="));
  Serial.print(currentDriveTurn, 2);
  Serial.print(F(" driveTurnPwm="));
  Serial.print(driveTurnPwm);
  Serial.print(F(" syncTarget="));
  Serial.print(encoderSyncTargetDifference, 2);
  Serial.print(F(" syncError="));
  Serial.print(encoderSyncError, 2);
  Serial.print(F(" syncCorrection="));
  Serial.print(encoderSyncCorrection);
  Serial.print(F(" gyroZHold="));
  Serial.print(gyroZHoldCorrection);
  Serial.print(F(" leftPwm="));
  Serial.print(MotorsTest::getLeftPwm());
  Serial.print(F(" rightPwm="));
  Serial.print(MotorsTest::getRightPwm());
  Serial.print(F(" motors="));
  Serial.print(motorsEnabled ? 1 : 0);
  Serial.print(F(" safety="));
  Serial.print(safetyStop ? 1 : 0);
  Serial.print(F(" fault="));
  Serial.println(faultMessage);
}

}  // namespace

namespace RobotControl {

void begin() {
  Serial.print(F("RobotControl running on core "));
  Serial.println(xPortGetCoreID());

  MotorsTest::begin();
  EncodersTest::begin();
  Imu6500Test::begin();
  BalancePid::begin();
  BalancePid::setSetpoint(manualAngleSetpointDeg);
  enterRecoveryWaiting("Waiting upright");
  fillSharedState();
}

void update() {
  const RobotCommand command = SharedState::consumeCommand();
  handleCommand(command);

  Imu6500Test::update();
  updateEncoderDiagnostics();
  updateDriveCommandState();
  Imu6500Test::ImuSample imu = Imu6500Test::getSample();
  if (Config::AUTO_RECOVERY_ENABLED) {
    runAutoRecovery(imu);
    imu = Imu6500Test::getSample();
  }

  if (!SharedState::isOtaUpdating()) {
    const unsigned long now = millis();
    float dtSeconds = static_cast<float>(Config::CONTROL_TASK_PERIOD_MS) / 1000.0f;
    if (previousControlMs != 0 && now > previousControlMs) {
      dtSeconds = static_cast<float>(now - previousControlMs) / 1000.0f;
    }
    previousControlMs = now;
    updateBalanceSetpointFromSpeed();
    BalancePid::update(imu.selectedAngleDeg, imu.gyroRateDegPerSec, dtSeconds, motorsEnabled,
                       safetyStop);
  }

  if (!Config::AUTO_RECOVERY_ENABLED) {
    updateSafety(imu);
  }
  if (safetyStop || safetyFault || fabsf(imu.selectedAngleDeg) > Config::MAX_SAFE_ANGLE_DEG ||
      fabs(BalancePid::getError()) > Config::MAX_SAFE_ANGLE_DEG) {
    BalancePid::resetIntegral();
  }
  applyPidToMotors();
  fillSharedState();
  printPidIfDue();
}

}  // namespace RobotControl
