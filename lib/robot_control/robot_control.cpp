#include "robot_control.h"

#include <Arduino.h>
#include <math.h>
#include <string.h>

#include "balance_pid.h"
#include "control_settings.h"
#include "encoders_test.h"
#include "imu6500_test.h"
#include "imu9250.h"
#include "motors_test.h"
#include "shared_state.h"
#include "state_feedback.h"
#include "state_feedback_settings.h"
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
bool autoTrimEnabled = Config::INITIAL_AUTO_TRIM_ENABLED;
double autoTrimOffsetDeg = 0.0;
double autoTrimScore = 0.0;
double autoTrimPositiveScore = 0.0;
double autoTrimNegativeScore = 0.0;
double autoTrimBestScore = 999999.0;
double autoTrimScoreSum = 0.0;
uint16_t autoTrimScoreSamples = 0;
uint8_t autoTrimNoImprovementCycles = 0;
unsigned long autoTrimPhaseStartMs = 0;
unsigned long autoTrimStableStartMs = 0;
unsigned long autoTrimStableElapsedMs = 0;
char autoTrimBlockReason[40] = "disabled";
char autoTrimStopReason[40] = "";
int balancePwm = 0;
int finalLeftPwm = 0;
int finalRightPwm = 0;
bool shadowControlReady = false;
int shadowPidOutput = 0;
char shadowStatus[80] = "Boot";
bool balanceControlEnabled = false;
bool balanceEnableRequested = false;
bool autoStartPending = Config::AUTO_START_CONTROL_ENABLED;
unsigned long autoStartStableStartMs = 0;
unsigned long autoStartStableMs = 0;
double targetBalanceSetpointDeg = 0.0;
bool controlSettingsSaved = true;
char controlSettingsMessage[48] = "loaded";
bool stateCalibrationWizardActive = false;
uint8_t stateCalibrationStage = 0;
StateFeedbackSettings::Settings stateCalibrationSnapshot{};
bool benchTestArmed = false;
bool benchTestActive = false;
int benchLeftPwm = 0;
int benchRightPwm = 0;
unsigned long benchArmExpiresMs = 0;
unsigned long benchLastHeartbeatMs = 0;
char benchTestCommand[24] = "stopped";

const char *shadowDirection(int output) {
  if (output > 2) return "PWM positivo";
  if (output < -2) return "PWM negativo";
  return "cero";
}

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

enum class AutoTrimPhase {
  Idle,
  WaitingStable,
  TestPositive,
  TestNegative,
  Done,
};

AutoTrimPhase autoTrimPhase = AutoTrimPhase::Idle;

double autoTrimTestOffset();

const char *autoTrimPhaseText() {
  switch (autoTrimPhase) {
    case AutoTrimPhase::Idle:
      return "Idle";
    case AutoTrimPhase::WaitingStable:
      return "Waiting";
    case AutoTrimPhase::TestPositive:
      return "Test +";
    case AutoTrimPhase::TestNegative:
      return "Test -";
    case AutoTrimPhase::Done:
      return "Done";
    default:
      return "Unknown";
  }
}

const char *autoTrimDirectionText() {
  if (autoTrimPhase == AutoTrimPhase::TestPositive) {
    return "+";
  }
  if (autoTrimPhase == AutoTrimPhase::TestNegative) {
    return "-";
  }
  return "none";
}

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

void resetAutoTrimState() {
  autoTrimOffsetDeg = 0.0;
  autoTrimScore = 0.0;
  autoTrimPositiveScore = 0.0;
  autoTrimNegativeScore = 0.0;
  autoTrimBestScore = 999999.0;
  autoTrimScoreSum = 0.0;
  autoTrimScoreSamples = 0;
  autoTrimNoImprovementCycles = 0;
  autoTrimPhaseStartMs = 0;
  autoTrimStableStartMs = 0;
  autoTrimStableElapsedMs = 0;
  strlcpy(autoTrimBlockReason, "reset", sizeof(autoTrimBlockReason));
  strlcpy(autoTrimStopReason, "", sizeof(autoTrimStopReason));
  autoTrimPhase = AutoTrimPhase::Idle;
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

  const double setpoint = constrain(manualAngleSetpointDeg + autoTrimOffsetDeg +
                                        autoTrimTestOffset() +
                                        speedHoldAngleCorrectionDeg +
                                        static_cast<double>(driveAngleOffsetDeg),
                                    Config::SETPOINT_MIN_DEG,
                                    Config::SETPOINT_MAX_DEG);
  BalancePid::setSetpoint(setpoint);
}

bool canRunAutoTrim(const Imu6500Test::ImuSample &imu) {
  if (!autoTrimEnabled) {
    strlcpy(autoTrimBlockReason, "disabled", sizeof(autoTrimBlockReason));
    return false;
  }
  if (recoveryState != RecoveryState::Running) {
    strlcpy(autoTrimBlockReason, "not running", sizeof(autoTrimBlockReason));
    return false;
  }
  if (!motorsEnabled) {
    strlcpy(autoTrimBlockReason, "motors off", sizeof(autoTrimBlockReason));
    return false;
  }
  if (safetyStop || safetyFault) {
    strlcpy(autoTrimBlockReason, "safety", sizeof(autoTrimBlockReason));
    return false;
  }
  if (SharedState::isOtaUpdating()) {
    strlcpy(autoTrimBlockReason, "ota", sizeof(autoTrimBlockReason));
    return false;
  }
  if (fabsf(targetDriveForward) > 0.01f || fabsf(targetDriveTurn) > 0.01f ||
      fabsf(currentDriveForward) > 0.01f || fabsf(currentDriveTurn) > 0.01f) {
    strlcpy(autoTrimBlockReason, "drive active", sizeof(autoTrimBlockReason));
    return false;
  }
  if (!imu.angleInitialized) {
    strlcpy(autoTrimBlockReason, "angle not ready", sizeof(autoTrimBlockReason));
    return false;
  }
  if (fabsf(imu.selectedAngleDeg) > Config::AUTO_RECOVERY_ANGLE_WINDOW_DEG) {
    strlcpy(autoTrimBlockReason, "angle too high", sizeof(autoTrimBlockReason));
    return false;
  }
  if (fabsf(averageSpeed) > Config::AUTO_TRIM_MAX_SPEED_COUNTS_PER_SEC) {
    strlcpy(autoTrimBlockReason, "speed too high", sizeof(autoTrimBlockReason));
    return false;
  }
  if (abs(BalancePid::getOutput()) > Config::AUTO_TRIM_MAX_PWM_FOR_TEST) {
    strlcpy(autoTrimBlockReason, "pwm too high", sizeof(autoTrimBlockReason));
    return false;
  }
  strlcpy(autoTrimBlockReason, "ready", sizeof(autoTrimBlockReason));
  return true;
}

void resetAutoTrimMeasurement() {
  autoTrimScoreSum = 0.0;
  autoTrimScoreSamples = 0;
}

void addAutoTrimSample() {
  const double sampleScore = fabs(static_cast<double>(averageSpeed)) +
                             0.02 * fabs(static_cast<double>(BalancePid::getOutput()));
  autoTrimScoreSum += sampleScore;
  ++autoTrimScoreSamples;
  autoTrimScore = sampleScore;
}

double finishAutoTrimMeasurement() {
  if (autoTrimScoreSamples == 0) {
    return 999999.0;
  }
  return autoTrimScoreSum / static_cast<double>(autoTrimScoreSamples);
}

void updateAutoTrim(const Imu6500Test::ImuSample &imu) {
  if (!autoTrimEnabled) {
    autoTrimPhase = AutoTrimPhase::Idle;
    autoTrimStableElapsedMs = 0;
    strlcpy(autoTrimBlockReason, "disabled", sizeof(autoTrimBlockReason));
    return;
  }

  if (autoTrimPhase == AutoTrimPhase::Done) {
    strlcpy(autoTrimBlockReason, "done", sizeof(autoTrimBlockReason));
    return;
  }

  const unsigned long now = millis();
  if (!canRunAutoTrim(imu)) {
    autoTrimPhase = AutoTrimPhase::WaitingStable;
    autoTrimStableStartMs = 0;
    autoTrimStableElapsedMs = 0;
    resetAutoTrimMeasurement();
    return;
  }

  switch (autoTrimPhase) {
    case AutoTrimPhase::Idle:
    case AutoTrimPhase::WaitingStable:
      if (autoTrimStableStartMs == 0) {
        autoTrimStableStartMs = now;
      }
      autoTrimStableElapsedMs = now - autoTrimStableStartMs;
      strlcpy(autoTrimBlockReason, "waiting stable", sizeof(autoTrimBlockReason));
      if (autoTrimStableElapsedMs >= Config::AUTO_TRIM_STABLE_BEFORE_START_MS) {
        autoTrimPhase = AutoTrimPhase::TestPositive;
        autoTrimPhaseStartMs = now;
        autoTrimStableElapsedMs = Config::AUTO_TRIM_STABLE_BEFORE_START_MS;
        strlcpy(autoTrimBlockReason, "testing +", sizeof(autoTrimBlockReason));
        resetAutoTrimMeasurement();
      } else {
        autoTrimPhase = AutoTrimPhase::WaitingStable;
      }
      break;

    case AutoTrimPhase::TestPositive:
      strlcpy(autoTrimBlockReason, "testing +", sizeof(autoTrimBlockReason));
      addAutoTrimSample();
      if (now - autoTrimPhaseStartMs >= Config::AUTO_TRIM_TEST_WINDOW_MS) {
        autoTrimPositiveScore = finishAutoTrimMeasurement();
        autoTrimPhase = AutoTrimPhase::TestNegative;
        autoTrimPhaseStartMs = now;
        resetAutoTrimMeasurement();
      }
      break;

    case AutoTrimPhase::TestNegative:
      strlcpy(autoTrimBlockReason, "testing -", sizeof(autoTrimBlockReason));
      addAutoTrimSample();
      if (now - autoTrimPhaseStartMs >= Config::AUTO_TRIM_TEST_WINDOW_MS) {
        autoTrimNegativeScore = finishAutoTrimMeasurement();
        const double bestCycleScore = min(autoTrimPositiveScore, autoTrimNegativeScore);
        const double scoreDelta = fabs(autoTrimPositiveScore - autoTrimNegativeScore);
        const bool improved = bestCycleScore + Config::AUTO_TRIM_MIN_SCORE_DELTA < autoTrimBestScore;

        if (bestCycleScore < autoTrimBestScore) {
          autoTrimBestScore = bestCycleScore;
        }

        if (scoreDelta < Config::AUTO_TRIM_MIN_SCORE_DELTA) {
          ++autoTrimNoImprovementCycles;
        } else if (improved) {
          autoTrimNoImprovementCycles = 0;
        } else {
          ++autoTrimNoImprovementCycles;
        }

        if (scoreDelta >= Config::AUTO_TRIM_MIN_SCORE_DELTA && autoTrimPositiveScore < autoTrimNegativeScore) {
          autoTrimOffsetDeg += Config::AUTO_TRIM_APPLY_STEP_DEG;
        } else if (scoreDelta >= Config::AUTO_TRIM_MIN_SCORE_DELTA && autoTrimNegativeScore < autoTrimPositiveScore) {
          autoTrimOffsetDeg -= Config::AUTO_TRIM_APPLY_STEP_DEG;
        }
        autoTrimOffsetDeg = constrain(autoTrimOffsetDeg, -Config::AUTO_TRIM_MAX_OFFSET_DEG,
                                      Config::AUTO_TRIM_MAX_OFFSET_DEG);
        autoTrimScore = bestCycleScore;

        if (autoTrimBestScore <= Config::AUTO_TRIM_TARGET_SCORE) {
          autoTrimPhase = AutoTrimPhase::Done;
          strlcpy(autoTrimStopReason, "target score", sizeof(autoTrimStopReason));
          strlcpy(autoTrimBlockReason, "done", sizeof(autoTrimBlockReason));
          resetAutoTrimMeasurement();
          break;
        }
        if (fabs(autoTrimOffsetDeg) >= Config::AUTO_TRIM_MAX_OFFSET_DEG) {
          autoTrimPhase = AutoTrimPhase::Done;
          strlcpy(autoTrimStopReason, "max offset", sizeof(autoTrimStopReason));
          strlcpy(autoTrimBlockReason, "done", sizeof(autoTrimBlockReason));
          resetAutoTrimMeasurement();
          break;
        }
        if (autoTrimNoImprovementCycles >= Config::AUTO_TRIM_MAX_NO_IMPROVEMENT_CYCLES) {
          autoTrimPhase = AutoTrimPhase::Done;
          strlcpy(autoTrimStopReason, "no improvement", sizeof(autoTrimStopReason));
          strlcpy(autoTrimBlockReason, "done", sizeof(autoTrimBlockReason));
          resetAutoTrimMeasurement();
          break;
        }

        autoTrimPhase = AutoTrimPhase::WaitingStable;
        autoTrimStableStartMs = now;
        autoTrimStableElapsedMs = 0;
        strlcpy(autoTrimBlockReason, "applied", sizeof(autoTrimBlockReason));
        resetAutoTrimMeasurement();
      }
      break;

    case AutoTrimPhase::Done:
      strlcpy(autoTrimBlockReason, "done", sizeof(autoTrimBlockReason));
      break;
  }
}

double autoTrimTestOffset() {
  if (autoTrimPhase == AutoTrimPhase::TestPositive) {
    return Config::AUTO_TRIM_TEST_OFFSET_DEG;
  }
  if (autoTrimPhase == AutoTrimPhase::TestNegative) {
    return -Config::AUTO_TRIM_TEST_OFFSET_DEG;
  }
  return 0.0;
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

void stopBenchTest(bool disarm, const char *reason) {
  MotorsTest::disable();
  benchTestActive = false;
  benchLeftPwm = 0;
  benchRightPwm = 0;
  benchLastHeartbeatMs = 0;
  if (disarm) {
    benchTestArmed = false;
    benchArmExpiresMs = 0;
  }
  strlcpy(benchTestCommand, reason, sizeof(benchTestCommand));
}

void stopMpu9250Balance(const char *reason) {
  balanceControlEnabled = false;
  balanceEnableRequested = false;
  motorsEnabled = false;
  safetyStop = true;
  BalancePid::resetIntegral();
  if (!benchTestActive) MotorsTest::disable();
  strlcpy(shadowStatus, reason, sizeof(shadowStatus));
}

void setControlSettingsResult(bool saved, const char *message) {
  controlSettingsSaved = saved;
  strlcpy(controlSettingsMessage, message, sizeof(controlSettingsMessage));
}

void applyStateFeedbackSettings(const StateFeedbackSettings::Settings &settings) {
  StateFeedback::setGains(settings.gains);
  StateFeedback::setFilters(settings.velocityFilterBeta,
                            settings.angularAccelerationFilterBeta);
}

const char *benchCommandName(int leftPwm, int rightPwm) {
  if (leftPwm > 0 && rightPwm == 0) return "left positive";
  if (leftPwm < 0 && rightPwm == 0) return "left negative";
  if (rightPwm > 0 && leftPwm == 0) return "right positive";
  if (rightPwm < 0 && leftPwm == 0) return "right negative";
  if (leftPwm > 0 && rightPwm > 0) return "both positive";
  if (leftPwm < 0 && rightPwm < 0) return "both negative";
  return "custom";
}

void updateBenchTestSafety() {
  const unsigned long now = millis();
  const Imu9250::RawSample imu = Imu9250::getSample();
  if (!benchTestArmed) {
    if (!balanceControlEnabled) MotorsTest::disable();
    return;
  }
  if (SharedState::isOtaUpdating()) {
    stopBenchTest(true, "stopped by OTA");
  } else if (strcmp(imu.calibrationMode, "idle") != 0) {
    stopBenchTest(true, "stopped by calibration");
  } else if (static_cast<long>(now - benchArmExpiresMs) >= 0) {
    stopBenchTest(true, "arm expired");
  } else if (benchTestActive && now - benchLastHeartbeatMs > Config::BENCH_TEST_WATCHDOG_MS) {
    stopBenchTest(false, "watchdog stop");
  } else if (!benchTestActive) {
    MotorsTest::disable();
  }
}

void fillRawImuState() {
  const Imu9250::RawSample imu = Imu9250::getSample();
  const RobotState previousState = SharedState::getState();
  RobotState state;
  state.imuRawAxG = imu.axG;
  state.imuRawAyG = imu.ayG;
  state.imuRawAzG = imu.azG;
  state.imuRawAccelNormG = imu.accelNormG;
  state.imuRawGxDps = imu.gxDps;
  state.imuRawGyDps = imu.gyDps;
  state.imuRawGzDps = imu.gzDps;
  state.imuRawMx = imu.mx;
  state.imuRawMy = imu.my;
  state.imuRawMz = imu.mz;
  state.imuRawMagDirectionDeg = imu.magneticDirectionDeg;
  state.imuCorrectedAxG = imu.correctedAxG;
  state.imuCorrectedAyG = imu.correctedAyG;
  state.imuCorrectedAzG = imu.correctedAzG;
  state.imuCorrectedGxDps = imu.correctedGxDps;
  state.imuCorrectedGyDps = imu.correctedGyDps;
  state.imuCorrectedGzDps = imu.correctedGzDps;
  state.imuCorrectedMxUt = imu.correctedMxUt;
  state.imuCorrectedMyUt = imu.correctedMyUt;
  state.imuCorrectedMzUt = imu.correctedMzUt;
  state.imuMagNormUt = imu.magneticNormUt;
  state.imuAccelRollDeg = imu.accelRollDeg;
  state.imuAccelPitchDeg = imu.accelPitchDeg;
  state.imuFilteredRollDeg = imu.filteredRollDeg;
  state.imuFilteredPitchDeg = imu.filteredPitchDeg;
  state.imuRelativeRollDeg = imu.relativeRollDeg;
  state.imuRelativePitchDeg = imu.relativePitchDeg;
  state.imuHeadingDeg = imu.headingDeg;
  state.imuFilterAlpha = imu.filterAlpha;
  for (uint8_t axis = 0; axis < 3; ++axis) {
    state.imuAccelOffset[axis] = imu.accelOffset[axis];
    state.imuAccelScale[axis] = imu.accelScale[axis];
    state.imuGyroOffset[axis] = imu.gyroOffset[axis];
    state.imuMagOffset[axis] = imu.magOffset[axis];
    state.imuMagScale[axis] = imu.magScale[axis];
  }
  state.imuVerticalRollDeg = imu.verticalRollDeg;
  state.imuVerticalPitchDeg = imu.verticalPitchDeg;
  state.imuRawSampleRateHz = imu.sampleRateHz;
  state.imuAccelRateHz = imu.accelRateHz;
  state.imuGyroRateHz = imu.gyroRateHz;
  state.imuMagRateHz = imu.magRateHz;
  state.imuCalibrationSamples = imu.calibrationSamples;
  state.imuSampleAgeMs = imu.lastSampleMs == 0 ? ULONG_MAX : millis() - imu.lastSampleMs;
  state.imuRawAddress = imu.address;
  state.imuRawId = imu.imuId;
  state.imuRawMagId = imu.magnetometerId;
  state.imuRawAccelReady = imu.accelReady;
  state.imuRawGyroReady = imu.gyroReady;
  state.imuRawMagReady = imu.magnetometerReady;
  state.imuFilterReady = imu.filterReady;
  state.imuCalibrationStored = imu.calibrationStored;
  state.imuAccelCalibrated = imu.accelCalibrated;
  state.imuGyroCalibrated = imu.gyroCalibrated;
  state.imuMagCalibrated = imu.magCalibrated;
  state.imuVerticalCalibrated = imu.verticalCalibrated;
  state.imuAccelWizardActive = imu.accelWizardActive;
  state.imuAccelPoseIndex = imu.accelPoseIndex;
  strlcpy(state.imuCalibrationMode, imu.calibrationMode, sizeof(state.imuCalibrationMode));
  strlcpy(state.imuCalibrationStatus, imu.calibrationStatus, sizeof(state.imuCalibrationStatus));
  strlcpy(state.imuAccelPoseName, imu.accelPoseName, sizeof(state.imuAccelPoseName));
  state.angleAccelDeg = imu.accelPitchDeg - imu.verticalPitchDeg;
  state.angleFilteredDeg = imu.relativePitchDeg;
  state.angleComplementaryDeg = imu.relativePitchDeg;
  state.selectedAngleDeg = imu.relativePitchDeg;
  state.gyroRateDegPerSec = imu.correctedGyDps;
  state.imuReady = Imu9250::isReady();
  state.gyroCalibrated = imu.gyroCalibrated;
  state.angleInitialized = imu.filterReady && imu.verticalCalibrated;
  state.shadowControlReady = shadowControlReady;
  state.shadowPidOutput = shadowPidOutput;
  strlcpy(state.shadowDirection, shadowDirection(shadowPidOutput), sizeof(state.shadowDirection));
  state.balanceControlEnabled = balanceControlEnabled;
  state.pidTargetSetpoint = targetBalanceSetpointDeg;
  state.controlSettingsSaved = controlSettingsSaved;
  strlcpy(state.controlSettingsMessage, controlSettingsMessage,
          sizeof(state.controlSettingsMessage));
  const StateFeedback::State feedbackState = StateFeedback::getState();
  const StateFeedback::Gains feedbackGains = StateFeedback::getGains();
  state.statePositionCounts =
      (static_cast<float>(EncodersTest::leftCount()) +
       static_cast<float>(EncodersTest::rightCount())) * 0.5f;
  state.statePosition = feedbackState.position;
  state.stateRawVelocity = feedbackState.rawVelocity;
  state.stateVelocity = feedbackState.velocity;
  state.stateAngleError = feedbackState.angleError;
  state.stateAngularVelocity = feedbackState.angularVelocity;
  state.stateRawAngularAcceleration = feedbackState.rawAngularAcceleration;
  state.stateAngularAcceleration = feedbackState.angularAcceleration;
  state.statePositionTerm = feedbackState.positionTerm;
  state.stateVelocityTerm = feedbackState.velocityTerm;
  state.stateAngleTerm = feedbackState.angleTerm;
  state.stateAngularVelocityTerm = feedbackState.angularVelocityTerm;
  state.stateAngularAccelerationTerm = feedbackState.angularAccelerationTerm;
  state.stateOutputBeforeLimit = feedbackState.outputBeforeLimit;
  state.stateOutputSaturated = feedbackState.saturated;
  state.stateSaturationCorrection = feedbackState.saturationCorrection;
  state.stateGainPosition = feedbackGains.position;
  state.stateGainVelocity = feedbackGains.velocity;
  state.stateGainAngle = feedbackGains.angle;
  state.stateGainAngularVelocity = feedbackGains.angularVelocity;
  state.stateGainAngularAcceleration = feedbackGains.angularAcceleration;
  state.stateVelocityFilterBeta = StateFeedback::getVelocityFilterBeta();
  state.stateAngularAccelerationFilterBeta =
      StateFeedback::getAngularAccelerationFilterBeta();
  state.stateCalibrationWizardActive = stateCalibrationWizardActive;
  state.stateCalibrationStage = stateCalibrationStage;
  state.benchTestArmed = benchTestArmed;
  state.benchTestActive = benchTestActive;
  state.benchArmRemainingMs = benchTestArmed && static_cast<long>(benchArmExpiresMs - millis()) > 0
                                  ? benchArmExpiresMs - millis()
                                  : 0;
  state.benchWatchdogAgeMs = benchTestActive ? millis() - benchLastHeartbeatMs : 0;
  strlcpy(state.benchTestCommand, benchTestCommand, sizeof(state.benchTestCommand));
  state.pidSetpoint = BalancePid::getSetpoint();
  state.pidInput = imu.relativePitchDeg;
  state.pidError = -feedbackState.angleError;
  state.pidOutput = feedbackState.output;
  state.pidPTerm = feedbackState.angleTerm;
  state.pidITerm = feedbackState.positionTerm + feedbackState.velocityTerm;
  state.pidDTerm = feedbackState.angularVelocityTerm + feedbackState.angularAccelerationTerm;
  state.pidOutputBeforeLimit = feedbackState.outputBeforeLimit;
  state.pidOutputAfterLimit = feedbackState.output;
  state.pidKp = BalancePid::getKp();
  state.pidKi = BalancePid::getKi();
  state.pidKd = BalancePid::getKd();
  state.pidOutputMin = BalancePid::getOutputMin();
  state.pidOutputMax = BalancePid::getOutputMax();
  state.motorLeftMinPwm = MotorsTest::getLeftMinPwm();
  state.motorLeftMaxPwm = MotorsTest::getLeftMaxPwm();
  state.motorRightMinPwm = MotorsTest::getRightMinPwm();
  state.motorRightMaxPwm = MotorsTest::getRightMaxPwm();
  state.motorLeftCompensation = MotorsTest::getLeftCompensation();
  state.motorRightCompensation = MotorsTest::getRightCompensation();
  state.pidIntegralEnabled = BalancePid::isIntegralEnabled();
  state.motorsEnabled = balanceControlEnabled;
  state.safetyStop = !balanceControlEnabled;
  state.safetyFault = !shadowControlReady;
  state.autoRecoveryEnabled = Config::AUTO_START_CONTROL_ENABLED;
  state.autoRecoveryWaiting = autoStartPending && !balanceControlEnabled;
  state.autoRecoveryCalibrating = false;
  state.autoRecoveryStableMs = autoStartStableMs;
  strlcpy(state.autoRecoveryState,
          balanceControlEnabled ? "Running" : (autoStartPending ? "Waiting" : "Disabled"),
          sizeof(state.autoRecoveryState));
  state.rawLeftEncoder = EncodersTest::rawLeftCount();
  state.rawRightEncoder = EncodersTest::rawRightCount();
  state.correctedLeftEncoder = EncodersTest::leftCount();
  state.correctedRightEncoder = EncodersTest::rightCount();
  state.leftDistanceMm = EncodersTest::leftDistanceMm();
  state.rightDistanceMm = EncodersTest::rightDistanceMm();
  state.leftSpeed = leftSpeed;
  state.rightSpeed = rightSpeed;
  state.speedAverage = averageSpeed;
  state.speedDifference = speedDifference;
  state.leftSpeedMmPerSec = EncodersTest::countsToMillimeters(leftSpeed);
  state.rightSpeedMmPerSec = EncodersTest::countsToMillimeters(rightSpeed);
  state.speedAverageMmPerSec = EncodersTest::countsToMillimeters(averageSpeed);
  state.speedDifferenceMmPerSec = EncodersTest::countsToMillimeters(speedDifference);
  state.leftPwm = MotorsTest::getLeftPwm();
  state.rightPwm = MotorsTest::getRightPwm();
  state.balancePwm = balancePwm;
  state.otaAvailable = previousState.otaAvailable;
  state.otaUpdating = previousState.otaUpdating;
  strlcpy(state.faultMessage, shadowStatus, sizeof(state.faultMessage));
  SharedState::setState(state);
}

void handleRawImuCommand(const RobotCommand &command) {
  const bool calibrationRequested = command.calibrateGyro || command.startAccelCalibration ||
                                    command.captureAccelPose || command.calibrateMagnetometer ||
                                    command.calibrateVertical || command.clearImuCalibration;
  if (calibrationRequested) {
    stopMpu9250Balance("PID disabled: calibration");
    stopBenchTest(true, "stopped by calibration");
  }
  if (command.otaStart) {
    stopMpu9250Balance("PID disabled: OTA");
    stopBenchTest(true, "stopped by OTA");
  }
  if (command.stopMotors || command.disableMotors) {
    autoStartStableStartMs = 0;
    autoStartStableMs = 0;
    stopBenchTest(true, command.stopMotors ? "global stop" : "motors disabled");
    stopMpu9250Balance(command.stopMotors ? "PID stopped" : "PID disabled");
  }
  if (command.enableMotors && !command.stopMotors && !command.disableMotors) {
    autoStartStableStartMs = 0;
    autoStartStableMs = 0;
    stopBenchTest(true, "disarmed by PID");
    balanceEnableRequested = true;
  }
  if (command.calibrateGyro) Imu9250::startGyroCalibration();
  if (command.startAccelCalibration) Imu9250::startAccelCalibration();
  if (command.captureAccelPose) Imu9250::captureAccelPose();
  if (command.calibrateMagnetometer) Imu9250::startMagCalibration();
  if (command.calibrateVertical) Imu9250::saveVertical();
  if (command.clearImuCalibration) Imu9250::clearCalibration();
  if (command.resetEncoders) {
    stopBenchTest(true, "encoders reset");
    stopMpu9250Balance("Control disabled: encoders reset");
    EncodersTest::reset();
    previousLeftEncoder = 0;
    previousRightEncoder = 0;
    leftSpeed = 0.0f;
    rightSpeed = 0.0f;
    averageSpeed = 0.0f;
    speedDifference = 0.0f;
    previousEncoderMs = millis();
    const Imu9250::RawSample imu = Imu9250::getSample();
    StateFeedback::reset(0, 0, imu.correctedGyDps);
  }
  if (command.startStateCalibrationWizard) {
    stopBenchTest(true, "wizard started");
    stopMpu9250Balance("Control disabled: calibration wizard");
    if (!stateCalibrationWizardActive) {
      stateCalibrationSnapshot = StateFeedbackSettings::get();
      const bool saved = StateFeedbackSettings::saveSnapshot(stateCalibrationSnapshot);
      setControlSettingsResult(saved, saved ? "Wizard snapshot saved" : "Snapshot save failed");
    }
    stateCalibrationWizardActive = true;
    stateCalibrationStage = 0;
  }
  if (command.updateStateCalibrationStage) {
    stopBenchTest(true, "wizard stage changed");
    stopMpu9250Balance("Control disabled: wizard stage changed");
    stateCalibrationWizardActive = true;
    stateCalibrationStage = constrain(command.stateCalibrationStage, 0, 7);
  }
  if (command.restoreStateCalibrationSnapshot && stateCalibrationWizardActive) {
    stopBenchTest(true, "wizard settings restored");
    stopMpu9250Balance("Control disabled: settings restored");
    StateFeedbackSettings::Settings snapshot = stateCalibrationSnapshot;
    const bool loaded = StateFeedbackSettings::loadSnapshot(snapshot);
    const bool saved = loaded && StateFeedbackSettings::save(snapshot);
    setControlSettingsResult(saved, saved ? "State gains restored" : "Restore failed");
    if (saved) {
      stateCalibrationSnapshot = snapshot;
      applyStateFeedbackSettings(snapshot);
    }
  } else if (command.updateStateFeedback) {
    const StateFeedbackSettings::Settings requested = {
        {command.stateGainPosition, command.stateGainVelocity, command.stateGainAngle,
         command.stateGainAngularVelocity, command.stateGainAngularAcceleration},
        command.stateVelocityFilterBeta, command.stateAngularAccelerationFilterBeta};
    const bool saved = StateFeedbackSettings::save(requested);
    setControlSettingsResult(saved, saved ? "State gains saved" : "State gains save failed");
    if (saved) applyStateFeedbackSettings(requested);
    previousControlMs = millis();
  }
  if (command.finishStateCalibrationWizard) {
    stopBenchTest(true, "wizard completed");
    stopMpu9250Balance("Control disabled: calibration completed");
    stateCalibrationStage = 7;
    stateCalibrationWizardActive = false;
  }
  if (command.updatePidTunings) {
    const bool saved = ControlSettings::savePid(command.pidKp, command.pidKi, command.pidKd);
    setControlSettingsResult(saved, saved ? "PID saved" : "PID save failed");
    if (saved) BalancePid::setTunings(command.pidKp, command.pidKi, command.pidKd);
    previousControlMs = millis();
  }
  if (command.updatePidSetpoint) {
    const bool saved = ControlSettings::saveSetpoint(command.pidSetpoint);
    setControlSettingsResult(saved, saved ? "Setpoint saved" : "Setpoint save failed");
    if (saved) targetBalanceSetpointDeg = command.pidSetpoint;
    previousControlMs = millis();
  }
  if (command.updatePidMaxPwm) {
    const bool saved = ControlSettings::savePidMaxPwm(command.pidMaxPwm);
    setControlSettingsResult(saved, saved ? "PID PWM saved" : "PID PWM save failed");
    if (saved) {
      BalancePid::setOutputLimit(command.pidMaxPwm);
      StateFeedback::setOutputLimit(command.pidMaxPwm);
    }
    previousControlMs = millis();
  }
  if (command.updateMotorPwmLimits) {
    const bool saved = ControlSettings::saveMotorConfig(
        command.motorLeftMinPwm, command.motorLeftMaxPwm,
        command.motorRightMinPwm, command.motorRightMaxPwm,
        command.motorLeftCompensation, command.motorRightCompensation);
    setControlSettingsResult(saved, saved ? "Motor limits saved" : "Motor limits save failed");
    previousControlMs = millis();
  }

  if (command.disarmBenchTest) {
    stopBenchTest(true, "disarmed");
  } else if (command.armBenchTest) {
    stopMpu9250Balance("PID disabled: bench test");
    const Imu9250::RawSample imu = Imu9250::getSample();
    if (!SharedState::isOtaUpdating() && strcmp(imu.calibrationMode, "idle") == 0) {
      stopBenchTest(false, "armed");
      benchTestArmed = true;
      benchArmExpiresMs = millis() + Config::BENCH_TEST_ARM_TIMEOUT_MS;
    }
  }

  if (command.updateBenchTest) {
    const int leftPwm = constrain(command.benchLeftPwm, -Config::BENCH_TEST_MAX_PWM,
                                  Config::BENCH_TEST_MAX_PWM);
    const int rightPwm = constrain(command.benchRightPwm, -Config::BENCH_TEST_MAX_PWM,
                                   Config::BENCH_TEST_MAX_PWM);
    if (leftPwm == 0 && rightPwm == 0) {
      stopBenchTest(false, "stopped");
    } else if (benchTestArmed && !SharedState::isOtaUpdating()) {
      benchLeftPwm = leftPwm;
      benchRightPwm = rightPwm;
      benchLastHeartbeatMs = millis();
      benchTestActive = true;
      strlcpy(benchTestCommand, benchCommandName(leftPwm, rightPwm), sizeof(benchTestCommand));
      MotorsTest::setLeftPwm(leftPwm, Config::BENCH_TEST_MAX_PWM);
      MotorsTest::setRightPwm(rightPwm, Config::BENCH_TEST_MAX_PWM);
    }
  }
}

void updateShadowControl() {
  const Imu9250::RawSample imu = Imu9250::getSample();
  const unsigned long now = millis();
  const unsigned long sampleAgeMs = imu.lastSampleMs == 0 ? ULONG_MAX : now - imu.lastSampleMs;
  const char *blockReason = nullptr;
  if (SharedState::isOtaUpdating()) blockReason = "OTA active";
  else if (!Imu9250::isReady()) blockReason = "IMU not ready";
  else if (!isfinite(imu.relativePitchDeg) || !isfinite(imu.correctedGyDps) ||
           !isfinite(targetBalanceSetpointDeg))
    blockReason = "invalid IMU value";
  else if (!imu.accelCalibrated) blockReason = "accel not calibrated";
  else if (!imu.gyroCalibrated) blockReason = "gyro not calibrated";
  else if (!imu.verticalCalibrated) blockReason = "vertical not calibrated";
  else if (!imu.filterReady) blockReason = "filter not ready";
  else if (strcmp(imu.calibrationMode, "idle") != 0) blockReason = "calibration active";
  else if (sampleAgeMs > Config::SHADOW_IMU_TIMEOUT_MS) blockReason = "stale IMU sample";
  else if (fabsf(imu.relativePitchDeg) > Config::MAX_SAFE_ANGLE_DEG) blockReason = "unsafe pitch";
  else if (fabs(targetBalanceSetpointDeg - imu.relativePitchDeg) > Config::MAX_SAFE_ANGLE_DEG)
    blockReason = "unsafe PID error";
  float measuredDtSeconds = static_cast<float>(Config::CONTROL_TASK_PERIOD_MS) / 1000.0f;
  if (previousControlMs != 0 && now > previousControlMs)
    measuredDtSeconds = static_cast<float>(now - previousControlMs) / 1000.0f;
  if ((measuredDtSeconds <= 0.0f || measuredDtSeconds > 0.1f) && blockReason == nullptr)
    blockReason = "control timing overrun";
  shadowControlReady = blockReason == nullptr;
  const float dtSeconds = measuredDtSeconds <= 0.0f || measuredDtSeconds > 0.1f
                              ? static_cast<float>(Config::CONTROL_TASK_PERIOD_MS) / 1000.0f
                              : measuredDtSeconds;
  previousControlMs = now;
  const double setpointStep = Config::PID_SETPOINT_SLEW_DEG_PER_SEC * dtSeconds;
  const double setpointDelta = targetBalanceSetpointDeg - BalancePid::getSetpoint();
  BalancePid::setSetpoint(BalancePid::getSetpoint() + constrain(setpointDelta, -setpointStep, setpointStep));
  const float safePitchDeg = isfinite(imu.relativePitchDeg) ? imu.relativePitchDeg : 0.0f;
  const float safeGyroDps = isfinite(imu.correctedGyDps) ? imu.correctedGyDps : 0.0f;
  StateFeedback::update(EncodersTest::leftCount(), EncodersTest::rightCount(),
                        safePitchDeg, BalancePid::getSetpoint(), safeGyroDps,
                        measuredDtSeconds);

  const bool autoStartReady = autoStartPending && !balanceControlEnabled &&
                              !balanceEnableRequested && blockReason == nullptr &&
                              !benchTestArmed && !benchTestActive &&
                              !stateCalibrationWizardActive &&
                              fabsf(averageSpeed) <= Config::STATE_ARM_MAX_SPEED_COUNTS_PER_SEC &&
                              fabs(BalancePid::getSetpoint() - imu.relativePitchDeg) <=
                                  Config::AUTO_START_ANGLE_WINDOW_DEG;
  if (autoStartReady) {
    if (autoStartStableStartMs == 0) autoStartStableStartMs = now;
    autoStartStableMs = now - autoStartStableStartMs;
    if (autoStartStableMs >= Config::AUTO_START_STABLE_MS) {
      autoStartStableMs = Config::AUTO_START_STABLE_MS;
      balanceEnableRequested = true;
    }
  } else if (autoStartPending) {
    autoStartStableStartMs = 0;
    autoStartStableMs = 0;
  }

  bool enableRejected = false;
  if (balanceEnableRequested) {
    balanceEnableRequested = false;
    if (blockReason != nullptr) {
      snprintf(shadowStatus, sizeof(shadowStatus), "PID blocked: %s", blockReason);
      enableRejected = true;
    } else if (benchTestArmed || benchTestActive) {
      strlcpy(shadowStatus, "PID blocked: bench test armed", sizeof(shadowStatus));
      enableRejected = true;
    } else if (fabsf(averageSpeed) > Config::STATE_ARM_MAX_SPEED_COUNTS_PER_SEC) {
      strlcpy(shadowStatus, "Control blocked: wheels moving", sizeof(shadowStatus));
      enableRejected = true;
    } else if (fabs(BalancePid::getSetpoint() - imu.relativePitchDeg) >
               Config::PID_ARM_MAX_ERROR_DEG) {
      strlcpy(shadowStatus, "PID blocked: move near setpoint", sizeof(shadowStatus));
      enableRejected = true;
    } else {
      EncodersTest::reset();
      previousLeftEncoder = 0;
      previousRightEncoder = 0;
      leftSpeed = 0.0f;
      rightSpeed = 0.0f;
      averageSpeed = 0.0f;
      speedDifference = 0.0f;
      previousEncoderMs = now;
      StateFeedback::reset(0, 0, imu.correctedGyDps);
      balanceControlEnabled = true;
      autoStartStableStartMs = 0;
      motorsEnabled = true;
      safetyStop = false;
      safetyFault = false;
      BalancePid::resetIntegral();
    }
  }

  if (balanceControlEnabled) {
    if (blockReason != nullptr) {
      char reason[80];
      snprintf(reason, sizeof(reason), "PID stopped: %s", blockReason);
      stopMpu9250Balance(reason);
    }
  }

  shadowPidOutput = StateFeedback::getState().output;
  if (balanceControlEnabled) {
    balancePwm = shadowPidOutput;
    finalLeftPwm = shadowPidOutput;
    finalRightPwm = shadowPidOutput;
    MotorsTest::setLeftPwm(finalLeftPwm);
    MotorsTest::setRightPwm(finalRightPwm);
    strlcpy(shadowStatus, "PID ACTIVE", sizeof(shadowStatus));
  } else {
    balancePwm = 0;
    finalLeftPwm = 0;
    finalRightPwm = 0;
    if (!benchTestActive) MotorsTest::disable();
    if (blockReason != nullptr && !enableRejected) {
      snprintf(shadowStatus, sizeof(shadowStatus), "PID blocked: %s", blockReason);
    } else if (!enableRejected && strcmp(shadowStatus, "Boot") == 0) {
      strlcpy(shadowStatus, "PID ready: disabled", sizeof(shadowStatus));
    }
    if (autoStartPending && blockReason == nullptr) {
      if (autoStartStableStartMs != 0) {
        const float remainingSeconds =
            static_cast<float>(Config::AUTO_START_STABLE_MS - autoStartStableMs) / 1000.0f;
        snprintf(shadowStatus, sizeof(shadowStatus), "Auto start in %.1f s", remainingSeconds);
      } else if (benchTestArmed || benchTestActive) {
        strlcpy(shadowStatus, "Auto start blocked: bench test", sizeof(shadowStatus));
      } else if (stateCalibrationWizardActive) {
        strlcpy(shadowStatus, "Auto start blocked: wizard", sizeof(shadowStatus));
      } else if (fabsf(averageSpeed) > Config::STATE_ARM_MAX_SPEED_COUNTS_PER_SEC) {
        strlcpy(shadowStatus, "Auto start: stop wheels", sizeof(shadowStatus));
      } else {
        strlcpy(shadowStatus, "Auto start: move near setpoint", sizeof(shadowStatus));
      }
    }
  }
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
  state.leftDistanceMm = EncodersTest::leftDistanceMm();
  state.rightDistanceMm = EncodersTest::rightDistanceMm();
  state.leftSpeed = leftSpeed;
  state.rightSpeed = rightSpeed;
  state.speedAverage = averageSpeed;
  state.speedDifference = speedDifference;
  state.leftSpeedMmPerSec = EncodersTest::countsToMillimeters(leftSpeed);
  state.rightSpeedMmPerSec = EncodersTest::countsToMillimeters(rightSpeed);
  state.speedAverageMmPerSec = EncodersTest::countsToMillimeters(averageSpeed);
  state.speedDifferenceMmPerSec = EncodersTest::countsToMillimeters(speedDifference);
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
  state.autoTrimEnabled = autoTrimEnabled;
  state.autoTrimDone = autoTrimPhase == AutoTrimPhase::Done;
  state.autoTrimOffsetDeg = autoTrimOffsetDeg;
  state.autoTrimScore = autoTrimScore;
  state.autoTrimBestScore = autoTrimBestScore >= 999999.0 ? 0.0 : autoTrimBestScore;
  state.autoTrimNoImprovementCycles = autoTrimNoImprovementCycles;
  state.autoTrimStableElapsedMs = autoTrimStableElapsedMs;
  strlcpy(state.autoTrimPhase, autoTrimPhaseText(), sizeof(state.autoTrimPhase));
  strlcpy(state.autoTrimDirection, autoTrimDirectionText(), sizeof(state.autoTrimDirection));
  strlcpy(state.autoTrimBlockReason, autoTrimBlockReason, sizeof(state.autoTrimBlockReason));
  strlcpy(state.autoTrimStopReason, autoTrimStopReason, sizeof(state.autoTrimStopReason));
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
  state.motorLeftMinPwm = MotorsTest::getLeftMinPwm();
  state.motorLeftMaxPwm = MotorsTest::getLeftMaxPwm();
  state.motorRightMinPwm = MotorsTest::getRightMinPwm();
  state.motorRightMaxPwm = MotorsTest::getRightMaxPwm();
  state.motorLeftCompensation = MotorsTest::getLeftCompensation();
  state.motorRightCompensation = MotorsTest::getRightCompensation();
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
    ControlSettings::savePid(BalancePid::getKp(), BalancePid::getKi(), BalancePid::getKd());
    previousControlMs = millis();
  }

  if (command.updatePidSetpoint) {
    manualAngleSetpointDeg = command.pidSetpoint;
    BalancePid::setSetpoint(manualAngleSetpointDeg);
  }

  if (command.updatePidMaxPwm) {
    BalancePid::setOutputLimit(command.pidMaxPwm);
  }

  if (command.updateMotorPwmLimits) {
    ControlSettings::saveMotorConfig(command.motorLeftMinPwm, command.motorLeftMaxPwm,
                                     command.motorRightMinPwm, command.motorRightMaxPwm,
                                     command.motorLeftCompensation,
                                     command.motorRightCompensation);
    previousControlMs = millis();
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

  if (command.updateAutoTrimEnabled) {
    autoTrimEnabled = command.autoTrimEnabled;
    if (autoTrimEnabled && autoTrimPhase != AutoTrimPhase::Done) {
      autoTrimPhase = AutoTrimPhase::WaitingStable;
    } else if (!autoTrimEnabled) {
      autoTrimPhase = AutoTrimPhase::Idle;
    }
    autoTrimStableStartMs = 0;
    resetAutoTrimMeasurement();
  }

  if (command.resetAutoTrim) {
    const bool keepEnabled = autoTrimEnabled;
    resetAutoTrimState();
    autoTrimEnabled = keepEnabled;
    autoTrimPhase = autoTrimEnabled ? AutoTrimPhase::WaitingStable : AutoTrimPhase::Idle;
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
  Serial.print(F(" autoTrim="));
  Serial.print(autoTrimOffsetDeg, 3);
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

  ControlSettings::begin(
      Config::RAW_IMU_DASHBOARD_ONLY ? Config::SHADOW_PID_KP : Config::INITIAL_PID_KP,
      Config::RAW_IMU_DASHBOARD_ONLY ? Config::SHADOW_PID_KI : Config::INITIAL_PID_KI,
      Config::RAW_IMU_DASHBOARD_ONLY ? Config::SHADOW_PID_KD : Config::INITIAL_PID_KD,
      Config::RAW_IMU_DASHBOARD_ONLY ? 0.0 : Config::INITIAL_ANGLE_SETPOINT_DEG,
      Config::RAW_IMU_DASHBOARD_ONLY ? Config::SHADOW_PID_MAX_PWM : Config::INITIAL_PID_MAX_PWM);
  const ControlSettings::Settings settings = ControlSettings::get();
  MotorsTest::begin();
  if (Config::RAW_IMU_DASHBOARD_ONLY) {
    motorsEnabled = false;
    safetyStop = true;
    safetyFault = true;
    MotorsTest::disable();
    EncodersTest::begin();
    Imu9250::begin();
    BalancePid::begin();
    BalancePid::setTunings(settings.kp, settings.ki, settings.kd);
    targetBalanceSetpointDeg = settings.setpoint;
    BalancePid::setSetpoint(settings.setpoint);
    BalancePid::setOutputLimit(settings.pidMaxPwm);
    BalancePid::setIntegralEnabled(true);
    StateFeedbackSettings::begin(settings.kp, settings.kd);
    const StateFeedbackSettings::Settings feedbackSettings =
        StateFeedbackSettings::get();
    StateFeedback::begin(feedbackSettings.gains,
                         feedbackSettings.velocityFilterBeta,
                         feedbackSettings.angularAccelerationFilterBeta,
                         settings.pidMaxPwm);
    previousControlMs = millis();
    fillRawImuState();
    Serial.println(F("MPU9250 state feedback ready; automatic activation enabled"));
    return;
  }
  EncodersTest::begin();
  Imu6500Test::begin();
  BalancePid::begin();
  BalancePid::setTunings(settings.kp, settings.ki, settings.kd);
  BalancePid::setOutputLimit(settings.pidMaxPwm);
  BalancePid::setSetpoint(manualAngleSetpointDeg);
  enterRecoveryWaiting("Waiting upright");
  fillSharedState();
}

void update() {
  if (Config::RAW_IMU_DASHBOARD_ONLY) {
    const RobotCommand command = SharedState::consumeCommand();
    handleRawImuCommand(command);
    Imu9250::update();
    updateBenchTestSafety();
    updateEncoderDiagnostics();
    updateShadowControl();
    fillRawImuState();
    return;
  }

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
    updateAutoTrim(imu);
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
