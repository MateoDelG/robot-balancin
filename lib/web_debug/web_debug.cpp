#include "web_debug.h"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <WiFi.h>

#include "../../include/config.h"
#include "ota_manager.h"
#include "shared_state.h"

namespace {

WebServer server(80);
WebSocketsServer webSocket(81);
unsigned long lastStateSendMs = 0;

template <typename T>
T clampValue(T value, T minValue, T maxValue) {
  if (value < minValue) {
    return minValue;
  }
  if (value > maxValue) {
    return maxValue;
  }
  return value;
}

String stateAsJson() {
  const RobotState state = SharedState::getState();
  JsonDocument doc;

  doc["angle"] = state.selectedAngleDeg;
  doc["selectedAngle"] = state.selectedAngleDeg;
  doc["filteredAngle"] = state.angleFilteredDeg;
  doc["accelAngle"] = state.angleAccelDeg;
  doc["complementaryAngle"] = state.angleComplementaryDeg;
  doc["kalmanAngle"] = state.angleKalmanDeg;
  doc["gyroRate"] = state.gyroRateDegPerSec;
  doc["turnRate"] = state.turnRateDegPerSec;
  doc["turnDirection"] = state.turnDirection;
  doc["ax"] = state.rawAx;
  doc["ay"] = state.rawAy;
  doc["az"] = state.rawAz;
  doc["gx"] = state.correctedGx;
  doc["gy"] = state.correctedGy;
  doc["gz"] = state.correctedGz;
  doc["rawImuMode"] = Config::RAW_IMU_DASHBOARD_ONLY;
  doc["imuRawAxG"] = state.imuRawAxG;
  doc["imuRawAyG"] = state.imuRawAyG;
  doc["imuRawAzG"] = state.imuRawAzG;
  doc["imuRawAccelNormG"] = state.imuRawAccelNormG;
  doc["imuRawGxDps"] = state.imuRawGxDps;
  doc["imuRawGyDps"] = state.imuRawGyDps;
  doc["imuRawGzDps"] = state.imuRawGzDps;
  doc["imuRawMx"] = state.imuRawMx;
  doc["imuRawMy"] = state.imuRawMy;
  doc["imuRawMz"] = state.imuRawMz;
  doc["imuRawMagDirectionDeg"] = state.imuRawMagDirectionDeg;
  doc["imuCorrectedAxG"] = state.imuCorrectedAxG;
  doc["imuCorrectedAyG"] = state.imuCorrectedAyG;
  doc["imuCorrectedAzG"] = state.imuCorrectedAzG;
  doc["imuCorrectedGxDps"] = state.imuCorrectedGxDps;
  doc["imuCorrectedGyDps"] = state.imuCorrectedGyDps;
  doc["imuCorrectedGzDps"] = state.imuCorrectedGzDps;
  doc["imuCorrectedMxUt"] = state.imuCorrectedMxUt;
  doc["imuCorrectedMyUt"] = state.imuCorrectedMyUt;
  doc["imuCorrectedMzUt"] = state.imuCorrectedMzUt;
  doc["imuMagNormUt"] = state.imuMagNormUt;
  doc["imuAccelRollDeg"] = state.imuAccelRollDeg;
  doc["imuAccelPitchDeg"] = state.imuAccelPitchDeg;
  doc["imuFilteredRollDeg"] = state.imuFilteredRollDeg;
  doc["imuFilteredPitchDeg"] = state.imuFilteredPitchDeg;
  doc["imuRelativeRollDeg"] = state.imuRelativeRollDeg;
  doc["imuRelativePitchDeg"] = state.imuRelativePitchDeg;
  doc["imuHeadingDeg"] = state.imuHeadingDeg;
  doc["imuFilterAlpha"] = state.imuFilterAlpha;
  doc["imuSampleAgeMs"] = state.imuSampleAgeMs;
  JsonArray imuAccelOffset = doc["imuAccelOffset"].to<JsonArray>();
  JsonArray imuAccelScale = doc["imuAccelScale"].to<JsonArray>();
  JsonArray imuGyroOffset = doc["imuGyroOffset"].to<JsonArray>();
  JsonArray imuMagOffset = doc["imuMagOffset"].to<JsonArray>();
  JsonArray imuMagScale = doc["imuMagScale"].to<JsonArray>();
  for (uint8_t axis = 0; axis < 3; ++axis) {
    imuAccelOffset.add(state.imuAccelOffset[axis]);
    imuAccelScale.add(state.imuAccelScale[axis]);
    imuGyroOffset.add(state.imuGyroOffset[axis]);
    imuMagOffset.add(state.imuMagOffset[axis]);
    imuMagScale.add(state.imuMagScale[axis]);
  }
  doc["imuVerticalRollDeg"] = state.imuVerticalRollDeg;
  doc["imuVerticalPitchDeg"] = state.imuVerticalPitchDeg;
  doc["imuRawSampleRateHz"] = state.imuRawSampleRateHz;
  doc["imuAccelRateHz"] = state.imuAccelRateHz;
  doc["imuGyroRateHz"] = state.imuGyroRateHz;
  doc["imuMagRateHz"] = state.imuMagRateHz;
  doc["imuCalibrationSamples"] = state.imuCalibrationSamples;
  doc["imuRawAddress"] = state.imuRawAddress;
  doc["imuRawId"] = state.imuRawId;
  doc["imuRawMagId"] = state.imuRawMagId;
  doc["imuRawAccelReady"] = state.imuRawAccelReady;
  doc["imuRawGyroReady"] = state.imuRawGyroReady;
  doc["imuRawMagReady"] = state.imuRawMagReady;
  doc["imuFilterReady"] = state.imuFilterReady;
  doc["imuCalibrationStored"] = state.imuCalibrationStored;
  doc["imuAccelCalibrated"] = state.imuAccelCalibrated;
  doc["imuGyroCalibrated"] = state.imuGyroCalibrated;
  doc["imuMagCalibrated"] = state.imuMagCalibrated;
  doc["imuVerticalCalibrated"] = state.imuVerticalCalibrated;
  doc["imuAccelWizardActive"] = state.imuAccelWizardActive;
  doc["imuAccelPoseIndex"] = state.imuAccelPoseIndex;
  doc["imuCalibrationMode"] = state.imuCalibrationMode;
  doc["imuCalibrationStatus"] = state.imuCalibrationStatus;
  doc["imuAccelPoseName"] = state.imuAccelPoseName;
  doc["shadowControlReady"] = state.shadowControlReady;
  doc["shadowPidOutput"] = state.shadowPidOutput;
  doc["shadowDirection"] = state.shadowDirection;
  doc["balanceControlEnabled"] = state.balanceControlEnabled;
  doc["targetSetpoint"] = state.pidTargetSetpoint;
  doc["controlSettingsSaved"] = state.controlSettingsSaved;
  doc["controlSettingsMessage"] = state.controlSettingsMessage;
  doc["statePosition"] = state.statePosition;
  doc["stateRawVelocity"] = state.stateRawVelocity;
  doc["stateVelocity"] = state.stateVelocity;
  doc["stateAngleError"] = state.stateAngleError;
  doc["stateAngularVelocity"] = state.stateAngularVelocity;
  doc["stateRawAngularAcceleration"] = state.stateRawAngularAcceleration;
  doc["stateAngularAcceleration"] = state.stateAngularAcceleration;
  doc["statePositionTerm"] = state.statePositionTerm;
  doc["stateVelocityTerm"] = state.stateVelocityTerm;
  doc["stateAngleTerm"] = state.stateAngleTerm;
  doc["stateAngularVelocityTerm"] = state.stateAngularVelocityTerm;
  doc["stateAngularAccelerationTerm"] = state.stateAngularAccelerationTerm;
  doc["stateOutputBeforeLimit"] = state.stateOutputBeforeLimit;
  doc["stateOutputSaturated"] = state.stateOutputSaturated;
  doc["stateSaturationCorrection"] = state.stateSaturationCorrection;
  doc["stateGainPosition"] = state.stateGainPosition;
  doc["stateGainVelocity"] = state.stateGainVelocity;
  doc["stateGainAngle"] = state.stateGainAngle;
  doc["stateGainAngularVelocity"] = state.stateGainAngularVelocity;
  doc["stateGainAngularAcceleration"] = state.stateGainAngularAcceleration;
  doc["stateVelocityFilterBeta"] = state.stateVelocityFilterBeta;
  doc["stateAngularAccelerationFilterBeta"] = state.stateAngularAccelerationFilterBeta;
  doc["stateCalibrationWizardActive"] = state.stateCalibrationWizardActive;
  doc["stateCalibrationStage"] = state.stateCalibrationStage;
  doc["benchTestArmed"] = state.benchTestArmed;
  doc["benchTestActive"] = state.benchTestActive;
  doc["benchArmRemainingMs"] = state.benchArmRemainingMs;
  doc["benchWatchdogAgeMs"] = state.benchWatchdogAgeMs;
  doc["benchTestCommand"] = state.benchTestCommand;
  doc["leftEncoder"] = state.correctedLeftEncoder;
  doc["rightEncoder"] = state.correctedRightEncoder;
  doc["rawLeftEncoder"] = state.rawLeftEncoder;
  doc["rawRightEncoder"] = state.rawRightEncoder;
  doc["leftSpeed"] = state.leftSpeed;
  doc["rightSpeed"] = state.rightSpeed;
  doc["speedAverage"] = state.speedAverage;
  doc["speedDifference"] = state.speedDifference;
  doc["encoderSyncTargetDifference"] = state.encoderSyncTargetDifference;
  doc["encoderSyncError"] = state.encoderSyncError;
  doc["encoderSyncCorrection"] = state.encoderSyncCorrection;
  doc["encoderSyncEnabled"] = state.encoderSyncEnabled;
  doc["encoderSyncKp"] = state.encoderSyncKp;
  doc["encoderSyncDeadband"] = state.encoderSyncDeadband;
  doc["encoderSyncMaxCorrection"] = state.encoderSyncMaxCorrection;
  doc["gyroZHoldEnabled"] = state.gyroZHoldEnabled;
  doc["gyroZHoldKp"] = state.gyroZHoldKp;
  doc["gyroZHoldDeadband"] = state.gyroZHoldDeadband;
  doc["gyroZHoldMaxCorrection"] = state.gyroZHoldMaxCorrection;
  doc["gyroZHoldCorrection"] = state.gyroZHoldCorrection;
  doc["speedHoldEnabled"] = state.speedHoldEnabled;
  doc["speedHoldKp"] = state.speedHoldKp;
  doc["speedHoldDeadband"] = state.speedHoldDeadband;
  doc["speedHoldMaxAngle"] = state.speedHoldMaxAngleDeg;
  doc["speedHoldAngleCorrection"] = state.speedHoldAngleCorrectionDeg;
  doc["driveForward"] = state.driveForward;
  doc["driveTurn"] = state.driveTurn;
  doc["driveAngleOffset"] = state.driveAngleOffsetDeg;
  doc["driveTurnPwm"] = state.driveTurnPwm;
  doc["driveCommandActive"] = state.driveCommandActive;
  doc["autoRecoveryEnabled"] = state.autoRecoveryEnabled;
  doc["autoRecoveryWaiting"] = state.autoRecoveryWaiting;
  doc["autoRecoveryCalibrating"] = state.autoRecoveryCalibrating;
  doc["autoRecoveryStableMs"] = state.autoRecoveryStableMs;
  doc["autoRecoveryState"] = state.autoRecoveryState;
  doc["autoTrimEnabled"] = state.autoTrimEnabled;
  doc["autoTrimDone"] = state.autoTrimDone;
  doc["autoTrimOffset"] = state.autoTrimOffsetDeg;
  doc["autoTrimScore"] = state.autoTrimScore;
  doc["autoTrimBestScore"] = state.autoTrimBestScore;
  doc["autoTrimNoImprovementCycles"] = state.autoTrimNoImprovementCycles;
  doc["autoTrimStableElapsedMs"] = state.autoTrimStableElapsedMs;
  doc["autoTrimPhase"] = state.autoTrimPhase;
  doc["autoTrimDirection"] = state.autoTrimDirection;
  doc["autoTrimBlockReason"] = state.autoTrimBlockReason;
  doc["autoTrimStopReason"] = state.autoTrimStopReason;
  doc["leftPwm"] = state.leftPwm;
  doc["rightPwm"] = state.rightPwm;
  doc["basePwm"] = state.balancePwm;
  doc["pidOutput"] = state.pidOutput;
  doc["balancePidOutput"] = state.balancePwm;
  doc["pTerm"] = state.pidPTerm;
  doc["iTerm"] = state.pidITerm;
  doc["dTerm"] = state.pidDTerm;
  doc["integral"] = state.pidIntegral;
  doc["integralLimit"] = state.pidIntegralLimit;
  doc["iTermLimit"] = state.pidITermLimit;
  doc["integralEnabled"] = state.pidIntegralEnabled;
  doc["outputBeforeLimit"] = state.pidOutputBeforeLimit;
  doc["outputAfterLimit"] = state.pidOutputAfterLimit;
  doc["kp"] = state.pidKp;
  doc["ki"] = state.pidKi;
  doc["kd"] = state.pidKd;
  doc["setpoint"] = state.pidSetpoint;
  doc["angleSetpoint"] = state.pidSetpoint;
  doc["pidError"] = state.pidError;
  doc["angleError"] = state.pidError;
  doc["pidMin"] = state.pidOutputMin;
  doc["maxPwm"] = state.pidOutputMax;
  doc["motorLeftMinPwm"] = state.motorLeftMinPwm;
  doc["motorLeftMaxPwm"] = state.motorLeftMaxPwm;
  doc["motorRightMinPwm"] = state.motorRightMinPwm;
  doc["motorRightMaxPwm"] = state.motorRightMaxPwm;
  doc["motorLeftCompensation"] = state.motorLeftCompensation;
  doc["motorRightCompensation"] = state.motorRightCompensation;
  doc["controlPeriodMs"] = state.controlPeriodMs;
  doc["motorsEnabled"] = state.motorsEnabled;
  doc["safetyStop"] = state.safetyStop;
  doc["safetyFault"] = state.safetyFault;
  doc["otaAvailable"] = state.otaAvailable;
  doc["otaUpdating"] = state.otaUpdating;
  doc["faultMessage"] = state.faultMessage;
  doc["imuReady"] = state.imuReady;
  doc["gyroCalibrated"] = state.gyroCalibrated;
  doc["angleInitialized"] = state.angleInitialized;

  String json;
  serializeJson(doc, json);
  return json;
}

void sendMessage(uint8_t clientId, const char *type, const char *message) {
  JsonDocument doc;
  doc["type"] = type;
  doc["message"] = message;
  String json;
  serializeJson(doc, json);
  webSocket.sendTXT(clientId, json);
}

bool requireNumber(JsonDocument &doc, const char *key) {
  return !doc[key].isNull() && doc[key].is<float>();
}

void handleMessage(uint8_t clientId, const char *payload) {
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, payload);
  if (error) {
    sendMessage(clientId, "error", "invalid json");
    return;
  }

  const char *type = doc["type"] | "";
  if (strcmp(type, "enable_motors") == 0) {
    SharedState::requestEnableMotors();
    sendMessage(clientId, "ack", "motors enable requested");
  } else if (strcmp(type, "disable_motors") == 0) {
    SharedState::requestDisableMotors();
    sendMessage(clientId, "ack", "motors disable requested");
  } else if (strcmp(type, "stop") == 0) {
    SharedState::requestStop();
    sendMessage(clientId, "ack", "stop requested");
  } else if (strcmp(type, "reset_encoders") == 0) {
    SharedState::requestEncoderReset();
    sendMessage(clientId, "ack", "encoder reset requested");
  } else if (strcmp(type, "calibrate_gyro") == 0) {
    SharedState::requestGyroCalibration();
    sendMessage(clientId, "ack", "gyro calibration requested");
  } else if (strcmp(type, "calibrate_vertical") == 0) {
    SharedState::requestVerticalCalibration();
    sendMessage(clientId, "ack", "vertical calibration requested");
  } else if (strcmp(type, "start_accel_calibration") == 0) {
    SharedState::requestAccelCalibrationStart();
    sendMessage(clientId, "ack", "accelerometer wizard requested");
  } else if (strcmp(type, "capture_accel_pose") == 0) {
    SharedState::requestAccelPoseCapture();
    sendMessage(clientId, "ack", "accelerometer pose capture requested");
  } else if (strcmp(type, "calibrate_magnetometer") == 0) {
    SharedState::requestMagnetometerCalibration();
    sendMessage(clientId, "ack", "magnetometer calibration requested");
  } else if (strcmp(type, "clear_imu_calibration") == 0) {
    SharedState::requestImuCalibrationClear();
    sendMessage(clientId, "ack", "IMU calibration clear requested");
  } else if (strcmp(type, "set_shadow_pid") == 0) {
    if (!requireNumber(doc, "kp") || !requireNumber(doc, "ki") || !requireNumber(doc, "kd")) {
      sendMessage(clientId, "error", "invalid shadow PID values");
      return;
    }
    const double kp = clampValue(doc["kp"].as<double>(), Config::PID_KP_MIN, Config::PID_KP_MAX);
    const double ki = clampValue(doc["ki"].as<double>(), Config::PID_KI_MIN, Config::PID_KI_MAX);
    const double kd = clampValue(doc["kd"].as<double>(), Config::PID_KD_MIN, Config::PID_KD_MAX);
    SharedState::requestPidTunings(kp, ki, kd);
    sendMessage(clientId, "ack", "shadow PID update requested");
  } else if (strcmp(type, "set_motor_pwm_limits") == 0) {
    if (!requireNumber(doc, "leftMin") || !requireNumber(doc, "leftMax") ||
        !requireNumber(doc, "rightMin") || !requireNumber(doc, "rightMax") ||
        !requireNumber(doc, "leftCompensation") ||
        !requireNumber(doc, "rightCompensation")) {
      sendMessage(clientId, "error", "invalid motor PWM limits");
      return;
    }
    const int leftMin = clampValue(doc["leftMin"].as<int>(), Config::MOTOR_PWM_LIMIT_MIN,
                                   Config::MOTOR_PWM_LIMIT_MAX);
    const int leftMax = clampValue(doc["leftMax"].as<int>(), Config::MOTOR_PWM_LIMIT_MIN,
                                   Config::MOTOR_PWM_LIMIT_MAX);
    const int rightMin = clampValue(doc["rightMin"].as<int>(), Config::MOTOR_PWM_LIMIT_MIN,
                                    Config::MOTOR_PWM_LIMIT_MAX);
    const int rightMax = clampValue(doc["rightMax"].as<int>(), Config::MOTOR_PWM_LIMIT_MIN,
                                    Config::MOTOR_PWM_LIMIT_MAX);
    const double leftCompensation = clampValue(doc["leftCompensation"].as<double>(),
                                                Config::MOTOR_COMPENSATION_MIN,
                                                Config::MOTOR_COMPENSATION_MAX);
    const double rightCompensation = clampValue(doc["rightCompensation"].as<double>(),
                                                 Config::MOTOR_COMPENSATION_MIN,
                                                 Config::MOTOR_COMPENSATION_MAX);
    if (leftMin > leftMax || rightMin > rightMax) {
      sendMessage(clientId, "error", "motor PWM minimum cannot exceed maximum");
      return;
    }
    SharedState::requestMotorPwmLimits(leftMin, leftMax, rightMin, rightMax,
                                       leftCompensation, rightCompensation);
    sendMessage(clientId, "ack", "motor PWM limits update requested");
  } else if (strcmp(type, "set_state_feedback") == 0) {
    if (!requireNumber(doc, "kx") || !requireNumber(doc, "kv") ||
        !requireNumber(doc, "ktheta") || !requireNumber(doc, "komega") ||
        !requireNumber(doc, "kalpha") || !requireNumber(doc, "velocityBeta") ||
        !requireNumber(doc, "angularAccelerationBeta")) {
      sendMessage(clientId, "error", "invalid state feedback values");
      return;
    }
    const double kx = clampValue(doc["kx"].as<double>(), Config::STATE_GAIN_MIN,
                                 Config::STATE_GAIN_MAX);
    const double kv = clampValue(doc["kv"].as<double>(), Config::STATE_GAIN_MIN,
                                 Config::STATE_GAIN_MAX);
    const double ktheta = clampValue(doc["ktheta"].as<double>(), Config::STATE_GAIN_MIN,
                                     Config::STATE_GAIN_MAX);
    const double komega = clampValue(doc["komega"].as<double>(), Config::STATE_GAIN_MIN,
                                     Config::STATE_GAIN_MAX);
    const double kalpha = clampValue(doc["kalpha"].as<double>(), Config::STATE_GAIN_MIN,
                                     Config::STATE_GAIN_MAX);
    const float velocityBeta = clampValue(doc["velocityBeta"].as<float>(),
                                          Config::STATE_FILTER_BETA_MIN,
                                          Config::STATE_FILTER_BETA_MAX);
    const float angularAccelerationBeta = clampValue(
        doc["angularAccelerationBeta"].as<float>(), Config::STATE_FILTER_BETA_MIN,
        Config::STATE_FILTER_BETA_MAX);
    SharedState::requestStateFeedbackConfig(kx, kv, ktheta, komega, kalpha,
                                            velocityBeta, angularAccelerationBeta);
    sendMessage(clientId, "ack", "state feedback update requested");
  } else if (strcmp(type, "state_wizard_start") == 0) {
    SharedState::requestStateCalibrationWizardStart();
    sendMessage(clientId, "ack", "state calibration wizard started");
  } else if (strcmp(type, "state_wizard_stage") == 0) {
    if (!requireNumber(doc, "stage")) {
      sendMessage(clientId, "error", "invalid wizard stage");
      return;
    }
    SharedState::requestStateCalibrationStage(
        clampValue(doc["stage"].as<int>(), 0, 7));
    sendMessage(clientId, "ack", "wizard stage updated");
  } else if (strcmp(type, "state_wizard_restore") == 0) {
    SharedState::requestStateCalibrationRestore();
    sendMessage(clientId, "ack", "wizard snapshot restore requested");
  } else if (strcmp(type, "state_wizard_finish") == 0) {
    SharedState::requestStateCalibrationFinish();
    sendMessage(clientId, "ack", "state calibration wizard completed");
  } else if (strcmp(type, "bench_arm") == 0) {
    SharedState::requestBenchTestArm();
    sendMessage(clientId, "ack", "bench test arm requested");
  } else if (strcmp(type, "bench_disarm") == 0) {
    SharedState::requestBenchTestDisarm();
    sendMessage(clientId, "ack", "bench test disarmed");
  } else if (strcmp(type, "bench_stop") == 0) {
    SharedState::requestBenchTestPwm(0, 0);
    sendMessage(clientId, "ack", "bench motors stopped");
  } else if (strcmp(type, "bench_drive") == 0) {
    if (!requireNumber(doc, "leftPwm") || !requireNumber(doc, "rightPwm")) {
      sendMessage(clientId, "error", "invalid bench PWM");
      return;
    }
    const int leftPwm = clampValue(doc["leftPwm"].as<int>(), -Config::BENCH_TEST_MAX_PWM,
                                   Config::BENCH_TEST_MAX_PWM);
    const int rightPwm = clampValue(doc["rightPwm"].as<int>(), -Config::BENCH_TEST_MAX_PWM,
                                    Config::BENCH_TEST_MAX_PWM);
    SharedState::requestBenchTestPwm(leftPwm, rightPwm);
  } else if (strcmp(type, "test_left_motor") == 0) {
    SharedState::requestLeftMotorTest();
    sendMessage(clientId, "ack", "left motor test requested");
  } else if (strcmp(type, "test_right_motor") == 0) {
    SharedState::requestRightMotorTest();
    sendMessage(clientId, "ack", "right motor test requested");
  } else if (strcmp(type, "test_both_motors") == 0) {
    SharedState::requestBothMotorsTest();
    sendMessage(clientId, "ack", "both motors test requested");
  } else if (strcmp(type, "set_pid") == 0) {
    if (!requireNumber(doc, "kp") || !requireNumber(doc, "ki") || !requireNumber(doc, "kd")) {
      sendMessage(clientId, "error", "invalid PID values");
      return;
    }
    const double kp = clampValue(doc["kp"].as<double>(), Config::PID_KP_MIN, Config::PID_KP_MAX);
    const double ki = clampValue(doc["ki"].as<double>(), Config::PID_KI_MIN, Config::PID_KI_MAX);
    const double kd = clampValue(doc["kd"].as<double>(), Config::PID_KD_MIN, Config::PID_KD_MAX);
    SharedState::requestPidTunings(kp, ki, kd);
    sendMessage(clientId, "ack", "PID updated");
  } else if (strcmp(type, "set_pwm_limit") == 0) {
    if (!requireNumber(doc, "maxPwm")) {
      sendMessage(clientId, "error", "invalid max PWM");
      return;
    }
    const int maxPwm = clampValue(doc["maxPwm"].as<int>(), Config::PID_MAX_PWM_MIN,
                                  Config::PID_MAX_PWM_MAX);
    SharedState::requestPidMaxPwm(maxPwm);
    sendMessage(clientId, "ack", "PWM limit updated");
  } else if (strcmp(type, "set_integral_limit") == 0) {
    if (!requireNumber(doc, "integralLimit")) {
      sendMessage(clientId, "error", "invalid integral limit");
      return;
    }
    const double limit = clampValue(doc["integralLimit"].as<double>(),
                                    Config::INTEGRAL_LIMIT_MIN,
                                    Config::INTEGRAL_LIMIT_MAX);
    SharedState::requestIntegralLimit(limit);
    sendMessage(clientId, "ack", "integral limit updated");
  } else if (strcmp(type, "set_i_term_limit") == 0) {
    if (!requireNumber(doc, "iTermLimit")) {
      sendMessage(clientId, "error", "invalid I term limit");
      return;
    }
    const double limit = clampValue(doc["iTermLimit"].as<double>(),
                                    Config::I_TERM_LIMIT_MIN,
                                    Config::I_TERM_LIMIT_MAX);
    SharedState::requestITermLimit(limit);
    sendMessage(clientId, "ack", "I term limit updated");
  } else if (strcmp(type, "enable_integral") == 0) {
    const bool enabled = doc["enabled"] | false;
    SharedState::requestIntegralEnabled(enabled);
    sendMessage(clientId, "ack", enabled ? "integral enabled" : "integral disabled");
  } else if (strcmp(type, "reset_integral") == 0) {
    SharedState::requestIntegralReset();
    sendMessage(clientId, "ack", "integral reset requested");
  } else if (strcmp(type, "enable_encoder_sync") == 0) {
    const bool enabled = doc["enabled"] | false;
    SharedState::requestEncoderSyncEnabled(enabled);
    sendMessage(clientId, "ack", enabled ? "encoder sync enabled" : "encoder sync disabled");
  } else if (strcmp(type, "set_encoder_sync") == 0) {
    if (!requireNumber(doc, "kp") || !requireNumber(doc, "deadband") ||
        !requireNumber(doc, "maxCorrection")) {
      sendMessage(clientId, "error", "invalid encoder sync values");
      return;
    }
    const double kp = clampValue(doc["kp"].as<double>(), Config::ENCODER_SYNC_KP_MIN,
                                 Config::ENCODER_SYNC_KP_MAX);
    const float deadband = clampValue(doc["deadband"].as<float>(),
                                      Config::ENCODER_SYNC_DEADBAND_MIN,
                                      Config::ENCODER_SYNC_DEADBAND_MAX);
    const int maxCorrection = clampValue(doc["maxCorrection"].as<int>(),
                                         Config::ENCODER_SYNC_MAX_CORRECTION_MIN,
                                         Config::ENCODER_SYNC_MAX_CORRECTION_MAX);
    SharedState::requestEncoderSyncConfig(kp, deadband, maxCorrection);
    sendMessage(clientId, "ack", "encoder sync updated");
  } else if (strcmp(type, "set_encoder_sync_target") == 0) {
    if (!requireNumber(doc, "targetDifference")) {
      sendMessage(clientId, "error", "invalid encoder sync target");
      return;
    }
    const float targetDifference = clampValue(doc["targetDifference"].as<float>(),
                                              Config::ENCODER_SYNC_TARGET_DIFFERENCE_MIN,
                                              Config::ENCODER_SYNC_TARGET_DIFFERENCE_MAX);
    SharedState::requestEncoderSyncTarget(targetDifference);
    sendMessage(clientId, "ack", "encoder sync target updated");
  } else if (strcmp(type, "enable_gyro_z_hold") == 0) {
    const bool enabled = doc["enabled"] | false;
    SharedState::requestGyroZHoldEnabled(enabled);
    sendMessage(clientId, "ack", enabled ? "gyro Z hold enabled" : "gyro Z hold disabled");
  } else if (strcmp(type, "set_gyro_z_hold") == 0) {
    if (!requireNumber(doc, "kp") || !requireNumber(doc, "maxCorrection")) {
      sendMessage(clientId, "error", "invalid gyro Z hold values");
      return;
    }
    const double kp = clampValue(doc["kp"].as<double>(), Config::GYRO_Z_HOLD_KP_MIN,
                                 Config::GYRO_Z_HOLD_KP_MAX);
    const int maxCorrection = clampValue(doc["maxCorrection"].as<int>(),
                                         Config::GYRO_Z_HOLD_MAX_CORRECTION_MIN,
                                         Config::GYRO_Z_HOLD_MAX_CORRECTION_MAX);
    SharedState::requestGyroZHoldConfig(kp, maxCorrection);
    sendMessage(clientId, "ack", "gyro Z hold updated");
  } else if (strcmp(type, "enable_speed_hold") == 0) {
    const bool enabled = doc["enabled"] | false;
    SharedState::requestSpeedHoldEnabled(enabled);
    sendMessage(clientId, "ack", enabled ? "speed hold enabled" : "speed hold disabled");
  } else if (strcmp(type, "set_speed_hold") == 0) {
    if (!requireNumber(doc, "kp") || !requireNumber(doc, "maxAngle")) {
      sendMessage(clientId, "error", "invalid speed hold values");
      return;
    }
    const double kp = clampValue(doc["kp"].as<double>(), Config::SPEED_HOLD_KP_MIN,
                                 Config::SPEED_HOLD_KP_MAX);
    const double maxAngle = clampValue(doc["maxAngle"].as<double>(),
                                       Config::SPEED_HOLD_MAX_ANGLE_MIN_DEG,
                                       Config::SPEED_HOLD_MAX_ANGLE_MAX_DEG);
    SharedState::requestSpeedHoldConfig(kp, maxAngle);
    sendMessage(clientId, "ack", "speed hold updated");
  } else if (strcmp(type, "drive") == 0) {
    if (!requireNumber(doc, "forward") || !requireNumber(doc, "turn")) {
      sendMessage(clientId, "error", "invalid drive command");
      return;
    }
    const float forward = clampValue(doc["forward"].as<float>(), -1.0f, 1.0f);
    const float turn = clampValue(doc["turn"].as<float>(), -1.0f, 1.0f);
    SharedState::requestDriveCommand(forward, turn);
  } else if (strcmp(type, "enable_auto_trim") == 0) {
    const bool enabled = doc["enabled"] | false;
    SharedState::requestAutoTrimEnabled(enabled);
    sendMessage(clientId, "ack", enabled ? "auto trim enabled" : "auto trim disabled");
  } else if (strcmp(type, "reset_auto_trim") == 0) {
    SharedState::requestAutoTrimReset();
    sendMessage(clientId, "ack", "auto trim reset requested");
  } else if (strcmp(type, "set_setpoint") == 0) {
    if (!requireNumber(doc, "setpoint")) {
      sendMessage(clientId, "error", "invalid setpoint");
      return;
    }
    const double setpoint = clampValue(doc["setpoint"].as<double>(), Config::SETPOINT_MIN_DEG,
                                       Config::SETPOINT_MAX_DEG);
    SharedState::requestPidSetpoint(setpoint);
    sendMessage(clientId, "ack", "setpoint updated");
  } else {
    sendMessage(clientId, "error", "unknown command");
  }
}

const char PAGE[] PROGMEM = R"rawliteral(
<!doctype html>
<html lang="es">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Robot Balancin - Sensor MPU9250</title>
  <style>
    body{font-family:Arial,sans-serif;margin:0;background:#111827;color:#e5e7eb}
    header{padding:16px 20px;background:#020617;border-bottom:1px solid #334155}
    main{padding:16px;display:grid;gap:16px;grid-template-columns:repeat(auto-fit,minmax(280px,1fr))}
    section{background:#1f2937;border:1px solid #374151;border-radius:12px;padding:14px}
    h1{font-size:20px;margin:0} h2{font-size:16px;margin:0 0 10px}
    .grid{display:grid;grid-template-columns:1fr 1fr;gap:8px}
    .item{background:#111827;border-radius:8px;padding:8px}.label{color:#9ca3af;font-size:12px}.value{font-size:20px;font-weight:700}
    .turn-right{color:#60a5fa}.turn-left{color:#fbbf24}.turn-still{color:#86efac}
    .drive-pad{display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px;align-items:center}.drive-pad button{width:100%;min-height:46px;user-select:none;-webkit-user-select:none;-webkit-touch-callout:none;touch-action:none}.drive-stop{grid-column:2;background:#dc2626}.drive-forward{grid-column:2}.drive-left{grid-column:1}.drive-right{grid-column:3}.drive-back{grid-column:2}
    input,select{width:100%;box-sizing:border-box;margin:4px 0 8px;padding:8px;border-radius:6px;border:1px solid #4b5563;background:#0f172a;color:#e5e7eb}
    button{margin:4px 4px 4px 0;padding:9px 11px;border:0;border-radius:8px;background:#2563eb;color:white;font-weight:700;cursor:pointer}
    button:disabled{opacity:.4;cursor:not-allowed}
    button.stop{background:#dc2626}.ok{background:#16a34a}.warn{background:#ca8a04}
    #status{color:#93c5fd;font-size:13px}.fault{color:#fca5a5}
    .sensor-mode section:not(.sensor-section){display:none}.sensor-mode main{grid-template-columns:repeat(auto-fit,minmax(360px,1fr))}
    .sensor-status{padding:10px;background:#0f172a;border-radius:8px;color:#fbbf24;min-height:20px}.cal-values{white-space:pre-line;font:12px Consolas,monospace;color:#9ca3af;line-height:1.5}.wide{grid-column:1/-1}
    #stateChart{display:block;width:100%;height:420px;background:#07101f;border:1px solid #334155;border-radius:8px}
  </style>
</head>
<body>
<header><h1 id="pageTitle">Robot Balancin - Sensor MPU9250</h1><div id="status">Conectando...</div></header>
<main>
  <section><h2>Estado</h2><div class="grid">
    <div class="item"><div class="label">Angulo</div><div class="value" id="angle">--</div></div>
    <div class="item"><div class="label">Gyro rate</div><div class="value" id="gyroRate">--</div></div>
    <div class="item"><div class="label">Giro</div><div class="value" id="turnDirection">--</div></div>
    <div class="item"><div class="label">Giro deg/s</div><div class="value" id="turnRate">--</div></div>
    <div class="item"><div class="label">PWM L/R</div><div class="value"><span id="leftPwm">--</span>/<span id="rightPwm">--</span></div></div>
    <div class="item"><div class="label">Motores</div><div class="value" id="motorsEnabled">--</div></div>
    <div class="item"><div class="label">IMU</div><div class="value" id="imuReady">--</div></div>
    <div class="item"><div class="label">Seguridad</div><div class="value" id="safetyStop">--</div></div>
    <div class="item"><div class="label">Auto recovery</div><div class="value" id="autoRecoveryState">--</div></div>
    <div class="item"><div class="label">Stable ms</div><div class="value" id="autoRecoveryStableMs">--</div></div>
  </div><p class="fault" id="faultMessage">--</p></section>
  <section class="sensor-section wide"><h2>Angulo de inclinacion</h2>
    <div class="grid">
      <div class="item"><div class="label">Roll acelerometro</div><div class="value" id="imuAccelRollDeg">--</div></div>
      <div class="item"><div class="label">Roll filtrado</div><div class="value" id="imuFilteredRollDeg">--</div></div>
      <div class="item"><div class="label">Roll relativo vertical</div><div class="value" id="imuRelativeRollDeg">--</div></div>
      <div class="item"><div class="label">Velocidad GX</div><div class="value" id="imuCorrectedGxDps">--</div></div>
      <div class="item"><div class="label">Pitch acelerometro</div><div class="value" id="imuAccelPitchDeg">--</div></div>
      <div class="item"><div class="label">Pitch filtrado</div><div class="value" id="imuFilteredPitchDeg">--</div></div>
      <div class="item"><div class="label">Pitch relativo vertical</div><div class="value" id="imuRelativePitchDeg">--</div></div>
      <div class="item"><div class="label">Rumbo compensado</div><div class="value" id="imuHeadingDeg">--</div></div>
    </div>
    <p>La entrada de equilibrio es <b>pitch relativo</b> con velocidad <b>GY</b>. Los motores solo reciben la salida cuando el PID se activa explicitamente.</p>
  </section>
  <section class="sensor-section wide"><h2>Realimentacion de estados</h2>
    <div class="sensor-status" id="shadowStatus">Esperando estado</div>
    <div class="grid">
      <div class="item"><div class="label">Posicion (cuentas)</div><div class="value" id="statePosition">--</div></div>
      <div class="item"><div class="label">Velocidad cruda / filtrada</div><div class="value"><span id="stateRawVelocity">--</span> / <span id="stateVelocity">--</span></div></div>
      <div class="item"><div class="label">Error angular</div><div class="value" id="stateAngleError">--</div></div>
      <div class="item"><div class="label">Velocidad angular</div><div class="value" id="stateAngularVelocity">--</div></div>
      <div class="item"><div class="label">Acel. angular cruda / filtrada</div><div class="value"><span id="stateRawAngularAcceleration">--</span> / <span id="stateAngularAcceleration">--</span></div></div>
      <div class="item"><div class="label">Ux / Uv</div><div class="value"><span id="statePositionTerm">--</span> / <span id="stateVelocityTerm">--</span></div></div>
      <div class="item"><div class="label">Utheta / Uomega</div><div class="value"><span id="stateAngleTerm">--</span> / <span id="stateAngularVelocityTerm">--</span></div></div>
      <div class="item"><div class="label">Ualpha</div><div class="value" id="stateAngularAccelerationTerm">--</div></div>
      <div class="item"><div class="label">Salida antes / despues limite</div><div class="value"><span id="stateOutputBeforeLimit">--</span> / <span id="shadowOutput">--</span></div></div>
      <div class="item"><div class="label">Saturacion / correccion</div><div class="value"><span id="stateOutputSaturated">--</span> / <span id="stateSaturationCorrection">--</span></div></div>
      <div class="item"><div class="label">PWM fisico L/R</div><div class="value"><span id="shadowLeftPwm">--</span>/<span id="shadowRightPwm">--</span></div></div>
      <div class="item"><div class="label">Edad muestra</div><div class="value"><span id="shadowSampleAge">--</span> ms</div></div>
      <div class="item"><div class="label">Setpoint aplicado</div><div class="value" id="shadowAppliedSetpoint">--</div></div>
      <div class="item"><div class="label">Persistencia</div><div class="value" id="controlSettingsStatus">--</div></div>
    </div>
    <div class="grid" style="margin-top:12px">
      <label>Kx posicion<input id="stateGainPosition" type="number" step="0.0001"></label>
      <label>Kv velocidad<input id="stateGainVelocity" type="number" step="0.0001"></label>
      <label>Ktheta angulo<input id="stateGainAngle" type="number" step="0.1"></label>
      <label>Komega velocidad angular<input id="stateGainAngularVelocity" type="number" step="0.01"></label>
      <label>Kalpha aceleracion angular<input id="stateGainAngularAcceleration" type="number" step="0.0001"></label>
      <label>Beta velocidad<input id="stateVelocityFilterBeta" type="number" min="0.01" max="1" step="0.01"></label>
      <label>Beta aceleracion angular<input id="stateAngularAccelerationFilterBeta" type="number" min="0.01" max="1" step="0.01"></label>
    </div>
    <button onclick="applyStateFeedback()">Guardar ganancias y filtros</button>
    <div class="grid" style="margin-top:12px"><label>Setpoint objetivo (-10 a 10)<input id="shadowSetpoint" type="number" step="0.01" min="-10" max="10"></label><label>Maximo PWM (0-255)<input id="shadowMaxPwm" type="number" step="1" min="0" max="255"></label></div>
    <button onclick="applyShadowControl()">Guardar setpoint y maximo PWM</button>
    <div class="actions"><button class="ok" onclick="enableBalance()">ACTIVAR CONTROL</button><button class="warn" onclick="disableBalance()">Desactivar control</button><button class="stop" onclick="stopBalance()">STOP</button></div>
    <p>Al activar se ponen los encoders en cero. Cambiar ganancias, filtros, setpoint o limites desactiva el control.</p>
    <p><b>Advertencia:</b> 255 PWM permite potencia completa.</p>
  </section>
  <section class="sensor-section wide"><h2>Grafica de estados</h2>
    <canvas id="stateChart"></canvas>
    <div class="actions"><button id="stateRecordToggle" onclick="toggleStateRecording()">Iniciar registro</button><button onclick="clearStateHistory()">Limpiar</button><button class="ok" onclick="exportStateCsv()">Exportar CSV</button></div>
    <p>Ventana visible: ultimos 60 s. El CSV conserva hasta 10 minutos. Muestras registradas: <span id="stateSampleCount">0</span>.</p>
    <p>Cada estado usa una escala vertical independiente; los valores exportados no se normalizan.</p>
  </section>
  <section class="sensor-section wide"><h2>Asistente de calibracion por etapas</h2>
    <div class="sensor-status"><b id="wizardStageTitle">Etapa 0: Preparacion</b></div>
    <p id="wizardInstructions">Pulsa Iniciar asistente para guardar una copia de las ganancias actuales.</p>
    <div class="item"><div class="label">Que debes comprobar</div><div id="wizardChecklist">Control apagado, PWM 0/0 y robot sujeto.</div></div>
    <div class="actions"><button onclick="startStateWizard()">Iniciar asistente</button><button onclick="previousStateWizardStage()">Anterior</button><button class="ok" onclick="nextStateWizardStage()">Confirmar y continuar</button><button class="warn" onclick="restoreStateWizard()">Restaurar valores iniciales</button><button class="stop" onclick="stopBalance()">STOP</button></div>
    <p>El avance es manual. Cada cambio de etapa detiene el control. Kalpha puede omitirse dejando su valor en cero.</p>
  </section>
  <section class="sensor-section wide"><h2>Prueba de banco: motores y encoders</h2>
    <div class="sensor-status" id="benchStatus">DESARMADA</div>
    <p><label><input id="benchConfirm" type="checkbox"> Confirmo que el robot esta firmemente soportado y las ruedas estan suspendidas.</label></p>
    <div class="grid">
      <label>PWM de prueba (0-255)<input id="benchPwm" type="number" min="0" max="255" step="1" value="70"></label>
      <div class="item"><div class="label">Armado restante</div><div class="value"><span id="benchRemaining">0</span> s</div></div>
      <div class="item"><div class="label">Watchdog</div><div class="value"><span id="benchWatchdog">0</span> ms</div></div>
      <div class="item"><div class="label">Comando</div><div class="value" id="benchCommand">stopped</div></div>
      <div class="item"><div class="label">PWM real L/R</div><div class="value"><span id="benchLeftPwm">0</span>/<span id="benchRightPwm">0</span></div></div>
      <div class="item"><div class="label">Velocidad L/R</div><div class="value"><span id="benchLeftSpeed">0</span>/<span id="benchRightSpeed">0</span></div></div>
      <div class="item"><div class="label">Encoder crudo L/R</div><div class="value"><span id="benchRawLeft">0</span>/<span id="benchRawRight">0</span></div></div>
      <div class="item"><div class="label">Encoder corregido L/R</div><div class="value"><span id="benchCorrectedLeft">0</span>/<span id="benchCorrectedRight">0</span></div></div>
    </div>
    <div class="actions"><button id="benchArm" class="warn">Armar 30 s</button><button id="benchDisarm">Desarmar</button><button id="benchStop" class="stop">PARADA</button><button onclick="send({type:'reset_encoders'})">Poner encoders en 0</button></div>
    <div class="actions"><button class="bench-drive" id="benchLeftPositive">Izquierdo +</button><button class="bench-drive" id="benchLeftNegative">Izquierdo -</button><button class="bench-drive" id="benchRightPositive">Derecho +</button><button class="bench-drive" id="benchRightNegative">Derecho -</button><button class="bench-drive" id="benchBothPositive">Ambos +</button><button class="bench-drive" id="benchBothNegative">Ambos -</button></div>
    <p>Los motores funcionan solo mientras se mantiene presionado. Soltar, perder WebSocket o dejar de enviar heartbeat detiene inmediatamente la salida.</p>
  </section>
  <section class="sensor-section wide"><h2>Limites PWM por motor</h2>
    <p>Todo comando no cero inferior al minimo se eleva al minimo. El maximo satura la salida. Guardar desarma la prueba de banco.</p>
    <div class="grid">
      <label>Izquierdo minimo<input id="motorLeftMinPwm" type="number" min="0" max="255" step="1"></label>
      <label>Izquierdo maximo<input id="motorLeftMaxPwm" type="number" min="0" max="255" step="1"></label>
      <label>Derecho minimo<input id="motorRightMinPwm" type="number" min="0" max="255" step="1"></label>
      <label>Derecho maximo<input id="motorRightMaxPwm" type="number" min="0" max="255" step="1"></label>
      <label>Compensacion izquierda<input id="motorLeftCompensation" type="number" min="0" max="2" step="0.01"></label>
      <label>Compensacion derecha<input id="motorRightCompensation" type="number" min="0" max="2" step="0.01"></label>
    </div>
    <button onclick="applyMotorLimits()">Guardar limites PWM</button>
    <p>La salida de cada motor se multiplica por su compensacion antes de aplicar minimo y maximo. La prueba de banco permite hasta 255 PWM.</p>
  </section>
  <section class="sensor-section"><h2>Acelerometro</h2><div class="grid">
    <div class="item"><div class="label">AX crudo / corregido</div><div class="value"><span id="imuRawAxG">--</span> / <span id="imuCorrectedAxG">--</span></div></div>
    <div class="item"><div class="label">AY crudo / corregido</div><div class="value"><span id="imuRawAyG">--</span> / <span id="imuCorrectedAyG">--</span></div></div>
    <div class="item"><div class="label">AZ crudo / corregido</div><div class="value"><span id="imuRawAzG">--</span> / <span id="imuCorrectedAzG">--</span></div></div>
    <div class="item"><div class="label">Norma corregida</div><div class="value" id="imuRawAccelNormG">--</div></div>
    <div class="item"><div class="label">Frecuencia</div><div class="value"><span id="imuAccelRateHz">--</span> Hz</div></div>
    <div class="item"><div class="label">Calibracion</div><div class="value" id="imuAccelCalibrated">--</div></div>
  </div></section>
  <section class="sensor-section"><h2>Giroscopio</h2><div class="grid">
    <div class="item"><div class="label">GX crudo / corregido</div><div class="value"><span id="imuRawGxDps">--</span> / <span id="imuCorrectedGxDpsValue">--</span></div></div>
    <div class="item"><div class="label">GY crudo / corregido</div><div class="value"><span id="imuRawGyDps">--</span> / <span id="imuCorrectedGyDps">--</span></div></div>
    <div class="item"><div class="label">GZ crudo / corregido</div><div class="value"><span id="imuRawGzDps">--</span> / <span id="imuCorrectedGzDps">--</span></div></div>
    <div class="item"><div class="label">Frecuencia</div><div class="value"><span id="imuGyroRateHz">--</span> Hz</div></div>
    <div class="item"><div class="label">Calibracion</div><div class="value" id="imuGyroCalibrated">--</div></div>
  </div></section>
  <section class="sensor-section"><h2>Magnetometro</h2><div class="grid">
    <div class="item"><div class="label">MX crudo / corregido uT</div><div class="value"><span id="imuRawMx">--</span> / <span id="imuCorrectedMxUt">--</span></div></div>
    <div class="item"><div class="label">MY crudo / corregido uT</div><div class="value"><span id="imuRawMy">--</span> / <span id="imuCorrectedMyUt">--</span></div></div>
    <div class="item"><div class="label">MZ crudo / corregido uT</div><div class="value"><span id="imuRawMz">--</span> / <span id="imuCorrectedMzUt">--</span></div></div>
    <div class="item"><div class="label">Norma magnetica</div><div class="value" id="imuMagNormUt">--</div></div>
    <div class="item"><div class="label">Frecuencia</div><div class="value"><span id="imuMagRateHz">--</span> Hz</div></div>
    <div class="item"><div class="label">Calibracion</div><div class="value" id="imuMagCalibrated">--</div></div>
  </div></section>
  <section class="sensor-section"><h2>Calibraciones persistentes</h2>
    <div class="sensor-status" id="imuCalibrationStatus">Esperando estado</div>
    <p>Modo: <b id="imuCalibrationMode">--</b> | muestras: <b id="imuCalibrationSamples">0</b> | NVS: <b id="imuCalibrationStored">--</b></p>
    <button id="sensorGyroCal" onclick="send({type:'calibrate_gyro'})">Calibrar gyro 3 s</button>
    <button id="sensorMagCal" onclick="send({type:'calibrate_magnetometer'})">Calibrar magnetometro 30 s</button>
    <button id="sensorVerticalCal" onclick="send({type:'calibrate_vertical'})">Guardar vertical</button><br>
    <button id="sensorAccelStart" onclick="send({type:'start_accel_calibration'})">Iniciar acelerometro 6 posiciones</button>
    <button id="sensorAccelCapture" onclick="send({type:'capture_accel_pose'})">Capturar posicion</button>
    <p>Posicion: <b id="imuAccelPose">No iniciado</b></p>
    <button class="warn" id="sensorClearCal" onclick="send({type:'clear_imu_calibration'})">Borrar calibracion NVS</button>
  </section>
  <section class="sensor-section"><h2>Parametros guardados</h2><div id="imuCalibrationValues" class="cal-values">--</div></section>
  <section class="sensor-section"><h2>Diagnostico MPU9250</h2><div class="grid">
    <div class="item"><div class="label">Direccion I2C</div><div class="value" id="imuRawAddress">--</div></div>
    <div class="item"><div class="label">MPU ID</div><div class="value" id="imuRawId">--</div></div>
    <div class="item"><div class="label">AK8963 ID</div><div class="value" id="imuRawMagId">--</div></div>
    <div class="item"><div class="label">Estado A/G/M</div><div class="value" id="imuRawReady">--</div></div>
    <div class="item"><div class="label">Filtro</div><div class="value" id="imuFilterReady">--</div></div>
    <div class="item"><div class="label">Vertical</div><div class="value" id="imuVerticalCalibrated">--</div></div>
  </div></section>
  <section><h2>PID</h2><div class="grid">
    <div class="item"><div class="label">Error</div><div class="value" id="pidError">--</div></div>
    <div class="item"><div class="label">Output</div><div class="value" id="pidOutput">--</div></div>
    <div class="item"><div class="label">P</div><div class="value" id="pTerm">--</div></div>
    <div class="item"><div class="label">I</div><div class="value" id="iTerm">--</div></div>
    <div class="item"><div class="label">D</div><div class="value" id="dTerm">--</div></div>
    <div class="item"><div class="label">Integral</div><div class="value" id="integral">--</div></div>
    <div class="item"><div class="label">Integral limit</div><div class="value" id="integralLimitValue">--</div></div>
    <div class="item"><div class="label">I term limit</div><div class="value" id="iTermLimitValue">--</div></div>
    <div class="item"><div class="label">Integral activa</div><div class="value" id="integralEnabledValue">--</div></div>
  </div></section>
  <section><h2>Ajuste PID</h2>
    <label>Kp</label><input id="kp" type="number" step="0.01">
    <label>Ki</label><input id="ki" type="number" step="0.01">
    <label>Kd</label><input id="kd" type="number" step="0.01">
    <button onclick="applyPid()">Actualizar PID</button>
    <label>Setpoint</label><input id="setpoint" type="number" step="0.01">
    <button onclick="applySetpoint()">Actualizar setpoint</button>
    <label>Max PWM</label><input id="maxPwm" type="number" step="1">
    <button onclick="applyPwmLimit()">Actualizar limite PWM</button>
  </section>
  <section><h2>Integral Ki</h2>
    <label>Limite integral</label><input id="integralLimit" type="number" step="0.01">
    <button onclick="applyIntegralLimit()">Actualizar limite integral</button>
    <label>Limite termino I</label><input id="iTermLimit" type="number" step="1">
    <button onclick="applyITermLimit()">Actualizar limite I</button><br>
    <button class="ok" onclick="send({type:'enable_integral',enabled:true})">Habilitar integral</button>
    <button class="warn" onclick="send({type:'enable_integral',enabled:false})">Deshabilitar integral</button>
    <button onclick="send({type:'reset_integral'})">Reset integral</button>
  </section>
  <section><h2>Comandos</h2>
    <button class="ok" onclick="send({type:'enable_motors'})">Habilitar motores</button>
    <button class="warn" onclick="send({type:'disable_motors'})">Deshabilitar</button>
    <button class="stop" onclick="send({type:'stop'})">STOP</button><br>
    <button onclick="send({type:'calibrate_gyro'})">Calibrar gyro</button>
    <button onclick="send({type:'calibrate_vertical'})">Calibrar vertical</button>
    <button onclick="send({type:'reset_encoders'})">Reset encoders</button><br>
    <button onclick="send({type:'test_left_motor'})">Test motor izquierdo</button>
    <button onclick="send({type:'test_right_motor'})">Test motor derecho</button>
    <button onclick="send({type:'test_both_motors'})">Test ambos</button>
  </section>
  <section><h2>Control movimiento</h2>
    <p>Los botones solo funcionan mientras estan presionados.</p>
    <div class="drive-pad">
      <button id="driveForward" class="drive-forward">Adelante</button>
      <button id="driveLeft" class="drive-left">Izquierda</button>
      <button id="driveStop" class="drive-stop">STOP movimiento</button>
      <button id="driveRight" class="drive-right">Derecha</button>
      <button id="driveBack" class="drive-back">Atras</button>
    </div>
    <div class="grid">
      <div class="item"><div class="label">Drive activo</div><div class="value" id="driveCommandActive">--</div></div>
      <div class="item"><div class="label">Forward</div><div class="value" id="driveForwardValue">--</div></div>
      <div class="item"><div class="label">Turn</div><div class="value" id="driveTurnValue">--</div></div>
      <div class="item"><div class="label">Turn PWM</div><div class="value" id="driveTurnPwm">--</div></div>
    </div>
  </section>
  <section><h2>Encoders diagnostico</h2><div class="grid">
    <div class="item"><div class="label">Left count</div><div class="value" id="leftEncoder">--</div></div>
    <div class="item"><div class="label">Right count</div><div class="value" id="rightEncoder">--</div></div>
    <div class="item"><div class="label">Left speed</div><div class="value" id="leftSpeed">--</div></div>
    <div class="item"><div class="label">Right speed</div><div class="value" id="rightSpeed">--</div></div>
    <div class="item"><div class="label">Speed diff L-R</div><div class="value" id="speedDifference">--</div></div>
    <div class="item"><div class="label">Target diff</div><div class="value" id="encoderSyncTargetDifference">--</div></div>
    <div class="item"><div class="label">Sync error</div><div class="value" id="encoderSyncError">--</div></div>
    <div class="item"><div class="label">Sync correction</div><div class="value" id="encoderSyncCorrection">--</div></div>
  </div></section>
  <section><h2>Sync por encoders</h2>
    <p>Compensa pequenas diferencias de velocidad entre ruedas sin cambiar el PID de equilibrio.</p>
    <div class="item"><div class="label">Estado sync</div><div class="value" id="encoderSyncEnabled">--</div></div>
    <label>Ksync</label><input id="encoderSyncKp" type="number" step="0.001">
    <label>Deadband velocidad</label><input id="encoderSyncDeadband" type="number" step="1">
    <label>Correccion maxima PWM</label><input id="encoderSyncMaxCorrection" type="number" step="1">
    <button onclick="applyEncoderSync()">Actualizar sync</button><br>
    <label>Motor mas rapido deseado</label>
    <select id="encoderSyncFastMotor">
      <option value="none">Ninguno</option>
      <option value="left">Izquierdo</option>
      <option value="right">Derecho</option>
    </select>
    <label>Diferencia objetivo abs</label><input id="encoderSyncTargetMagnitude" type="number" step="1">
    <button onclick="applyEncoderSyncTarget()">Actualizar motor rapido</button><br>
    <button class="ok" onclick="send({type:'enable_encoder_sync',enabled:true})">Habilitar sync</button>
    <button class="warn" onclick="send({type:'enable_encoder_sync',enabled:false})">Deshabilitar sync</button>
  </section>
  <section><h2>Compensacion giro Z</h2>
    <p>Usa solo el giroscopio Z para frenar rotacion no deseada del robot.</p>
    <div class="grid">
      <div class="item"><div class="label">Estado</div><div class="value" id="gyroZHoldEnabled">--</div></div>
      <div class="item"><div class="label">Correccion PWM</div><div class="value" id="gyroZHoldCorrection">--</div></div>
      <div class="item"><div class="label">Deadband deg/s</div><div class="value" id="gyroZHoldDeadband">--</div></div>
      <div class="item"><div class="label">Giro deg/s</div><div class="value" id="gyroZHoldTurnRate">--</div></div>
    </div>
    <label>Kz gyro</label><input id="gyroZHoldKp" type="number" step="0.01">
    <label>Correccion maxima PWM</label><input id="gyroZHoldMaxCorrection" type="number" step="1">
    <button onclick="applyGyroZHold()">Actualizar gyro Z</button><br>
    <button class="ok" onclick="send({type:'enable_gyro_z_hold',enabled:true})">Habilitar gyro Z</button>
    <button class="warn" onclick="send({type:'enable_gyro_z_hold',enabled:false})">Deshabilitar gyro Z</button>
  </section>
  <section><h2>Quieto por velocidad</h2>
    <p>Usa la velocidad promedio de encoders para inclinar levemente el setpoint y frenar desplazamientos.</p>
    <div class="grid">
      <div class="item"><div class="label">Estado</div><div class="value" id="speedHoldEnabled">--</div></div>
      <div class="item"><div class="label">Correccion angulo</div><div class="value" id="speedHoldAngleCorrection">--</div></div>
      <div class="item"><div class="label">Speed avg</div><div class="value" id="speedHoldAverage">--</div></div>
      <div class="item"><div class="label">Deadband count/s</div><div class="value" id="speedHoldDeadband">--</div></div>
    </div>
    <label>K velocidad</label><input id="speedHoldKp" type="number" step="0.0001">
    <label>Correccion maxima angulo</label><input id="speedHoldMaxAngle" type="number" step="0.1">
    <button onclick="applySpeedHold()">Actualizar velocidad</button><br>
    <button class="ok" onclick="send({type:'enable_speed_hold',enabled:true})">Habilitar velocidad</button>
    <button class="warn" onclick="send({type:'enable_speed_hold',enabled:false})">Deshabilitar velocidad</button>
  </section>
  <section><h2>Auto Trim SP</h2>
    <p>Ajusta lentamente el setpoint base buscando menor velocidad y menor esfuerzo PID.</p>
    <div class="grid">
      <div class="item"><div class="label">Estado</div><div class="value" id="autoTrimEnabled">--</div></div>
      <div class="item"><div class="label">Done</div><div class="value" id="autoTrimDone">--</div></div>
      <div class="item"><div class="label">Offset</div><div class="value" id="autoTrimOffset">--</div></div>
      <div class="item"><div class="label">Score</div><div class="value" id="autoTrimScore">--</div></div>
      <div class="item"><div class="label">Best score</div><div class="value" id="autoTrimBestScore">--</div></div>
      <div class="item"><div class="label">Sin mejora</div><div class="value" id="autoTrimNoImprovementCycles">--</div></div>
      <div class="item"><div class="label">Fase</div><div class="value" id="autoTrimPhase">--</div></div>
      <div class="item"><div class="label">Direccion</div><div class="value" id="autoTrimDirection">--</div></div>
      <div class="item"><div class="label">SP final</div><div class="value" id="autoTrimFinalSetpoint">--</div></div>
      <div class="item"><div class="label">Stable ms</div><div class="value" id="autoTrimStableElapsedMs">--</div></div>
      <div class="item"><div class="label">Bloqueo</div><div class="value" id="autoTrimBlockReason">--</div></div>
      <div class="item"><div class="label">Stop reason</div><div class="value" id="autoTrimStopReason">--</div></div>
    </div>
    <button class="ok" onclick="send({type:'enable_auto_trim',enabled:true})">Habilitar auto trim</button>
    <button class="warn" onclick="send({type:'enable_auto_trim',enabled:false})">Deshabilitar auto trim</button>
    <button onclick="send({type:'reset_auto_trim'})">Reset auto trim</button>
  </section>
</main>
<script>
let ws;
let pidDirty = false;
let setpointDirty = false;
let pwmDirty = false;
let motorLimitsDirty = false;
let integralLimitDirty = false;
let iTermLimitDirty = false;
let encoderSyncDirty = false;
let encoderSyncTargetDirty = false;
let gyroZHoldDirty = false;
let speedHoldDirty = false;
let stateFeedbackDirty = false;
let shadowSetpointDirty = false;
let shadowMaxPwmDirty = false;
let currentWizardStage = 0;
const wizardStages = [
  ['Etapa 0: Preparacion','Sujeta el robot, confirma MPU9250 calibrado, sentidos correctos y revisa limites y compensaciones.','Control apagado, PWM 0/0, banco desarmado y robot firmemente sujeto.'],
  ['Etapa 1: Validacion de estados','Con motores apagados, gira las ruedas e inclina lentamente el robot para comprobar los cinco estados.','Posicion y velocidad cambian con las ruedas; angulo, velocidad y aceleracion angular cambian con la inclinacion.'],
  ['Etapa 2: Ganancia Ktheta','Deja Kx, Kv y Kalpha en cero. Ajusta Ktheta con movimientos pequenos y el robot sujeto.','Aumenta Ktheta si la reaccion es debil; reducelo si la reaccion es demasiado brusca.'],
  ['Etapa 3: Ganancia Komega','Conserva Ktheta y ajusta Komega para amortiguar la oscilacion angular.','Aumenta Komega si oscila; reducelo si la respuesta queda excesivamente frenada.'],
  ['Etapa 4: Ganancia Kv','Coloca el robot en el suelo con Kx y Kalpha en cero. Ajusta Kv para reducir la velocidad horizontal.','La velocidad filtrada debe regresar cerca de cero sin oscilaciones crecientes.'],
  ['Etapa 5: Ganancia Kx','Activa el control para poner encoders en cero y empuja suavemente el robot.','Aumenta Kx si no regresa; reducelo si cruza repetidamente la posicion cero.'],
  ['Etapa 6: Ganancia Kalpha opcional','Empieza con Kalpha en cero y usa incrementos muy pequenos solo si mejora el amortiguamiento.','Conserva Kalpha solo si reduce picos sin amplificar el ruido de aceleracion angular.'],
  ['Etapa 7: Validacion final','Prueba equilibrio quieto y perturbaciones suaves en ambos sentidos.','Confirma posicion y velocidad cercanas a cero, angulo estable y PWM dentro de limites.']
];
const STATE_HISTORY_MAX = 6000;
const STATE_CHART_SAMPLES = 600;
let stateHistory = [];
let stateRecording = false;
let stateRecordingStartMs = 0;
let driveTimer = null;
let benchTimer = null;
let benchArmed = false;
let benchLeftCommand = 0;
let benchRightCommand = 0;
let driveForwardCommand = 0;
let driveTurnCommand = 0;
function num(id){return Number(document.getElementById(id).value)}
function setText(id,value,digits=2){const el=document.getElementById(id); if(!el)return; el.textContent=typeof value==='number'?value.toFixed(digits):value;}
function hexByte(value){if(typeof value!=='number' || value===0)return '--'; return '0x'+value.toString(16).padStart(2,'0').toUpperCase();}
function setInput(id,value,digits=3,dirty=false){const el=document.getElementById(id); if(!el || dirty || document.activeElement===el || typeof value!=='number')return; el.value=value.toFixed(digits);}
function setTurnDirection(direction){
  const el=document.getElementById('turnDirection');
  if(!el)return;
  el.textContent=direction || '--';
  el.className='value '+(direction==='derecha'?'turn-right':(direction==='izquierda'?'turn-left':'turn-still'));
}
function send(obj){if(!ws || ws.readyState!==WebSocket.OPEN){alert('WebSocket no conectado');return;} ws.send(JSON.stringify(obj));}
function benchPwm(){return Math.min(255,Math.max(0,Math.round(num('benchPwm')||0)));}
function sendBenchHeartbeat(){if(ws&&ws.readyState===WebSocket.OPEN)ws.send(JSON.stringify({type:'bench_drive',leftPwm:benchLeftCommand,rightPwm:benchRightCommand}));}
function stopBench(){if(benchTimer){clearInterval(benchTimer);benchTimer=null;}benchLeftCommand=0;benchRightCommand=0;if(ws&&ws.readyState===WebSocket.OPEN)ws.send(JSON.stringify({type:'bench_stop'}));}
function startBench(leftSign,rightSign){
  stopBench();
  if(!benchArmed)return;
  const pwm=benchPwm();
  if(pwm<=0)return;
  benchLeftCommand=leftSign*pwm;benchRightCommand=rightSign*pwm;
  sendBenchHeartbeat();
  benchTimer=setInterval(sendBenchHeartbeat,100);
}
function bindBenchButton(id,leftSign,rightSign){
  const button=document.getElementById(id);
  button.addEventListener('pointerdown',event=>{event.preventDefault();button.setPointerCapture(event.pointerId);startBench(leftSign,rightSign)});
  button.addEventListener('pointerup',stopBench);button.addEventListener('pointercancel',stopBench);button.addEventListener('pointerleave',stopBench);
}
function sendDrive(forward,turn){if(ws && ws.readyState===WebSocket.OPEN){ws.send(JSON.stringify({type:'drive',forward,turn}));}}
function stopDrive(){if(driveTimer){clearInterval(driveTimer);driveTimer=null;} driveForwardCommand=0; driveTurnCommand=0; sendDrive(0,0);}
function startDrive(forward,turn){stopDrive(); driveForwardCommand=forward; driveTurnCommand=turn; sendDrive(forward,turn); driveTimer=setInterval(()=>sendDrive(driveForwardCommand,driveTurnCommand),100);}
function bindDriveButton(id,forward,turn){
  const button=document.getElementById(id); if(!button)return;
  button.addEventListener('pointerdown',(event)=>{event.preventDefault(); button.setPointerCapture(event.pointerId); startDrive(forward,turn);});
  button.addEventListener('pointerup',stopDrive);
  button.addEventListener('pointercancel',stopDrive);
  button.addEventListener('pointerleave',stopDrive);
}
function applyPid(){send({type:'set_pid',kp:num('kp'),ki:num('ki'),kd:num('kd')}); pidDirty=false;}
function applyStateFeedback(){send({type:'set_state_feedback',kx:num('stateGainPosition'),kv:num('stateGainVelocity'),ktheta:num('stateGainAngle'),komega:num('stateGainAngularVelocity'),kalpha:num('stateGainAngularAcceleration'),velocityBeta:num('stateVelocityFilterBeta'),angularAccelerationBeta:num('stateAngularAccelerationFilterBeta')});stateFeedbackDirty=false;}
function renderStateWizard(stage){currentWizardStage=Math.max(0,Math.min(7,stage));const item=wizardStages[currentWizardStage];setText('wizardStageTitle',item[0],0);setText('wizardInstructions',item[1],0);setText('wizardChecklist',item[2],0);}
function startStateWizard(){send({type:'state_wizard_start'});renderStateWizard(0);}
function previousStateWizardStage(){const stage=Math.max(0,currentWizardStage-1);send({type:'state_wizard_stage',stage});renderStateWizard(stage);}
function nextStateWizardStage(){if(!confirm('Confirma que realizaste la prueba y que el comportamiento fisico es correcto.'))return;if(currentWizardStage>=7){send({type:'state_wizard_finish'});return;}const stage=currentWizardStage+1;send({type:'state_wizard_stage',stage});renderStateWizard(stage);}
function restoreStateWizard(){if(confirm('Restaurar las ganancias guardadas al iniciar el asistente?'))send({type:'state_wizard_restore'});}
function finiteValue(value){return typeof value==='number'&&Number.isFinite(value)?value:0;}
function captureStateSample(data){
  if(!stateRecording)return;
  const now=Date.now();if(!stateRecordingStartMs)stateRecordingStartMs=now;
  stateHistory.push({elapsedMs:now-stateRecordingStartMs,epochMs:now,position:finiteValue(data.statePosition),rawVelocity:finiteValue(data.stateRawVelocity),velocity:finiteValue(data.stateVelocity),angleError:finiteValue(data.stateAngleError),angularVelocity:finiteValue(data.stateAngularVelocity),rawAngularAcceleration:finiteValue(data.stateRawAngularAcceleration),angularAcceleration:finiteValue(data.stateAngularAcceleration),ux:finiteValue(data.statePositionTerm),uv:finiteValue(data.stateVelocityTerm),utheta:finiteValue(data.stateAngleTerm),uomega:finiteValue(data.stateAngularVelocityTerm),ualpha:finiteValue(data.stateAngularAccelerationTerm),outputBeforeLimit:finiteValue(data.stateOutputBeforeLimit),output:finiteValue(data.shadowPidOutput),leftPwm:finiteValue(data.leftPwm),rightPwm:finiteValue(data.rightPwm),saturated:data.stateOutputSaturated?1:0,kx:finiteValue(data.stateGainPosition),kv:finiteValue(data.stateGainVelocity),ktheta:finiteValue(data.stateGainAngle),komega:finiteValue(data.stateGainAngularVelocity),kalpha:finiteValue(data.stateGainAngularAcceleration),velocityBeta:finiteValue(data.stateVelocityFilterBeta),angularAccelerationBeta:finiteValue(data.stateAngularAccelerationFilterBeta),setpoint:finiteValue(data.setpoint),targetSetpoint:finiteValue(data.targetSetpoint),maxPwm:finiteValue(data.maxPwm),controlPeriodMs:finiteValue(data.controlPeriodMs),leftMinPwm:finiteValue(data.motorLeftMinPwm),leftMaxPwm:finiteValue(data.motorLeftMaxPwm),rightMinPwm:finiteValue(data.motorRightMinPwm),rightMaxPwm:finiteValue(data.motorRightMaxPwm),leftCompensation:finiteValue(data.motorLeftCompensation),rightCompensation:finiteValue(data.motorRightCompensation)});
  if(stateHistory.length>STATE_HISTORY_MAX)stateHistory.splice(0,stateHistory.length-STATE_HISTORY_MAX);
  setText('stateSampleCount',stateHistory.length,0);drawStateChart();
}
function toggleStateRecording(){stateRecording=!stateRecording;if(stateRecording&&!stateRecordingStartMs)stateRecordingStartMs=Date.now();setText('stateRecordToggle',stateRecording?'Pausar registro':(stateHistory.length?'Reanudar registro':'Iniciar registro'),0);}
function clearStateHistory(){stateHistory=[];stateRecordingStartMs=stateRecording?Date.now():0;setText('stateSampleCount',0,0);setText('stateRecordToggle',stateRecording?'Pausar registro':'Iniciar registro',0);drawStateChart();}
function drawStateChart(){
  const canvas=document.getElementById('stateChart');if(!canvas)return;
  const cssWidth=Math.max(320,canvas.clientWidth||1200),cssHeight=420,dpr=window.devicePixelRatio||1;
  if(canvas.width!==Math.round(cssWidth*dpr)||canvas.height!==Math.round(cssHeight*dpr)){canvas.width=Math.round(cssWidth*dpr);canvas.height=Math.round(cssHeight*dpr);canvas.style.height=cssHeight+'px';}
  const ctx=canvas.getContext('2d');ctx.setTransform(dpr,0,0,dpr,0,0);ctx.clearRect(0,0,cssWidth,cssHeight);ctx.fillStyle='#07101f';ctx.fillRect(0,0,cssWidth,cssHeight);
  const series=[['Posicion',s=>s.position,'#f59e0b'],['Velocidad',s=>s.velocity,'#22c55e'],['Error angular',s=>s.angleError,'#ef4444'],['Vel. angular',s=>s.angularVelocity,'#38bdf8'],['Acel. angular',s=>s.angularAcceleration,'#c084fc']];
  const samples=stateHistory.slice(-STATE_CHART_SAMPLES),left=104,right=12,bandHeight=cssHeight/series.length,plotWidth=cssWidth-left-right;
  ctx.font='11px Arial';
  series.forEach((entry,index)=>{const top=index*bandHeight,bottom=top+bandHeight;ctx.strokeStyle='#1e293b';ctx.beginPath();ctx.moveTo(0,bottom);ctx.lineTo(cssWidth,bottom);ctx.stroke();const values=samples.map(entry[1]);let min=values.length?Math.min(...values):-1,max=values.length?Math.max(...values):1;if(min>0)min=0;if(max<0)max=0;if(max-min<1e-6){min-=1;max+=1;}const padding=(max-min)*0.08;min-=padding;max+=padding;const y=value=>top+8+(max-value)/(max-min)*(bandHeight-16);ctx.strokeStyle='#334155';ctx.beginPath();ctx.moveTo(left,y(0));ctx.lineTo(cssWidth-right,y(0));ctx.stroke();ctx.fillStyle=entry[2];ctx.fillText(entry[0],8,top+17);ctx.fillStyle='#94a3b8';ctx.fillText(max.toFixed(1),8,top+34);ctx.fillText(min.toFixed(1),8,bottom-8);if(samples.length>1){ctx.strokeStyle=entry[2];ctx.lineWidth=1.5;ctx.beginPath();samples.forEach((sample,sampleIndex)=>{const x=left+sampleIndex/(samples.length-1)*plotWidth,py=y(entry[1](sample));if(sampleIndex===0)ctx.moveTo(x,py);else ctx.lineTo(x,py);});ctx.stroke();}});
}
function exportStateCsv(){
  if(!stateHistory.length){alert('No hay muestras para exportar.');return;}
  const header='elapsed_ms,epoch_ms,position_counts,raw_velocity_counts_s,velocity_counts_s,angle_error_deg,angular_velocity_deg_s,raw_angular_accel_deg_s2,angular_accel_deg_s2,Ux,Uv,Utheta,Uomega,Ualpha,output_before_limit,output_limited,left_pwm,right_pwm,saturated,Kx,Kv,Ktheta,Komega,Kalpha,velocity_filter_beta,angular_accel_filter_beta,angle_setpoint_deg,target_setpoint_deg,max_pwm,control_period_ms,left_min_pwm,left_max_pwm,right_min_pwm,right_max_pwm,left_compensation,right_compensation';
  const rows=stateHistory.map(s=>[s.elapsedMs,s.epochMs,s.position,s.rawVelocity,s.velocity,s.angleError,s.angularVelocity,s.rawAngularAcceleration,s.angularAcceleration,s.ux,s.uv,s.utheta,s.uomega,s.ualpha,s.outputBeforeLimit,s.output,s.leftPwm,s.rightPwm,s.saturated,s.kx,s.kv,s.ktheta,s.komega,s.kalpha,s.velocityBeta,s.angularAccelerationBeta,s.setpoint,s.targetSetpoint,s.maxPwm,s.controlPeriodMs,s.leftMinPwm,s.leftMaxPwm,s.rightMinPwm,s.rightMaxPwm,s.leftCompensation,s.rightCompensation].join(','));
  const blob=new Blob([[header,...rows].join('\n')],{type:'text/csv;charset=utf-8'}),url=URL.createObjectURL(blob),link=document.createElement('a');link.href=url;link.download='robot_estados_'+new Date().toISOString().replace(/[:.]/g,'-')+'.csv';document.body.appendChild(link);link.click();link.remove();URL.revokeObjectURL(url);
}
function applyShadowControl(){send({type:'set_setpoint',setpoint:num('shadowSetpoint')});send({type:'set_pwm_limit',maxPwm:num('shadowMaxPwm')});shadowSetpointDirty=false;shadowMaxPwmDirty=false;}
function enableBalance(){send({type:'enable_motors'});}
function disableBalance(){send({type:'disable_motors'});}
function stopBalance(){stopBench();send({type:'stop'});}
function applySetpoint(){send({type:'set_setpoint',setpoint:num('setpoint')}); setpointDirty=false;}
function applyPwmLimit(){send({type:'set_pwm_limit',maxPwm:num('maxPwm')}); pwmDirty=false;}
function applyMotorLimits(){
  const leftMin=Math.round(num('motorLeftMinPwm')),leftMax=Math.round(num('motorLeftMaxPwm')),rightMin=Math.round(num('motorRightMinPwm')),rightMax=Math.round(num('motorRightMaxPwm')),leftCompensation=num('motorLeftCompensation'),rightCompensation=num('motorRightCompensation');
  if(leftMin>leftMax || rightMin>rightMax){alert('El PWM minimo no puede superar el maximo.');return;}
  send({type:'set_motor_pwm_limits',leftMin,leftMax,rightMin,rightMax,leftCompensation,rightCompensation});motorLimitsDirty=false;
}
function applyIntegralLimit(){send({type:'set_integral_limit',integralLimit:num('integralLimit')}); integralLimitDirty=false;}
function applyITermLimit(){send({type:'set_i_term_limit',iTermLimit:num('iTermLimit')}); iTermLimitDirty=false;}
function applyEncoderSync(){send({type:'set_encoder_sync',kp:num('encoderSyncKp'),deadband:num('encoderSyncDeadband'),maxCorrection:num('encoderSyncMaxCorrection')}); encoderSyncDirty=false;}
function applyGyroZHold(){send({type:'set_gyro_z_hold',kp:num('gyroZHoldKp'),maxCorrection:num('gyroZHoldMaxCorrection')}); gyroZHoldDirty=false;}
function applySpeedHold(){send({type:'set_speed_hold',kp:num('speedHoldKp'),maxAngle:num('speedHoldMaxAngle')}); speedHoldDirty=false;}
function applyEncoderSyncTarget(){
  const motor=document.getElementById('encoderSyncFastMotor').value;
  const magnitude=Math.abs(num('encoderSyncTargetMagnitude'));
  const target=motor==='left'?magnitude:(motor==='right'?-magnitude:0);
  send({type:'set_encoder_sync_target',targetDifference:target});
  encoderSyncTargetDirty=false;
}
function updateFastMotorInputs(target){
  if(encoderSyncTargetDirty || typeof target!=='number')return;
  const selector=document.getElementById('encoderSyncFastMotor');
  selector.value=target>0?'left':(target<0?'right':'none');
  setInput('encoderSyncTargetMagnitude',Math.abs(target),1,false);
}
function markDirty(){
  ['kp','ki','kd'].forEach(id=>document.getElementById(id).addEventListener('input',()=>pidDirty=true));
  document.getElementById('setpoint').addEventListener('input',()=>setpointDirty=true);
  document.getElementById('maxPwm').addEventListener('input',()=>pwmDirty=true);
  ['motorLeftMinPwm','motorLeftMaxPwm','motorRightMinPwm','motorRightMaxPwm','motorLeftCompensation','motorRightCompensation'].forEach(id=>document.getElementById(id).addEventListener('input',()=>motorLimitsDirty=true));
  document.getElementById('integralLimit').addEventListener('input',()=>integralLimitDirty=true);
  document.getElementById('iTermLimit').addEventListener('input',()=>iTermLimitDirty=true);
  ['encoderSyncKp','encoderSyncDeadband','encoderSyncMaxCorrection'].forEach(id=>document.getElementById(id).addEventListener('input',()=>encoderSyncDirty=true));
  ['gyroZHoldKp','gyroZHoldMaxCorrection'].forEach(id=>document.getElementById(id).addEventListener('input',()=>gyroZHoldDirty=true));
  ['speedHoldKp','speedHoldMaxAngle'].forEach(id=>document.getElementById(id).addEventListener('input',()=>speedHoldDirty=true));
  ['stateGainPosition','stateGainVelocity','stateGainAngle','stateGainAngularVelocity','stateGainAngularAcceleration','stateVelocityFilterBeta','stateAngularAccelerationFilterBeta'].forEach(id=>document.getElementById(id).addEventListener('input',()=>stateFeedbackDirty=true));
  document.getElementById('shadowSetpoint').addEventListener('input',()=>shadowSetpointDirty=true);
  document.getElementById('shadowMaxPwm').addEventListener('input',()=>shadowMaxPwmDirty=true);
  bindDriveButton('driveForward',1,0);
  bindDriveButton('driveBack',-1,0);
  bindDriveButton('driveLeft',0,-1);
  bindDriveButton('driveRight',0,1);
  bindDriveButton('driveStop',0,0);
  document.getElementById('encoderSyncFastMotor').addEventListener('change',()=>encoderSyncTargetDirty=true);
  document.getElementById('encoderSyncTargetMagnitude').addEventListener('input',()=>encoderSyncTargetDirty=true);
}
function connect(){
  ws=new WebSocket('ws://'+location.hostname+':81/');
  ws.onopen=()=>setText('status','WebSocket conectado',0);
  ws.onclose=()=>{stopBench();setText('status','WebSocket desconectado',0); setTimeout(connect,1000);};
  ws.onmessage=(event)=>{
    const data=JSON.parse(event.data);
    if(data.type==='ack' || data.type==='error'){setText('status',data.message,0); console.log(data);return;}
    captureStateSample(data);
    document.body.classList.toggle('sensor-mode',data.rawImuMode);
    if(data.rawImuMode)setText('pageTitle','Robot Balancin - Realimentacion de estados',0);
    setText('angle',data.selectedAngle); setText('gyroRate',data.gyroRate); setTurnDirection(data.turnDirection); setText('turnRate',data.turnRate); setText('gyroZHoldTurnRate',data.turnRate); setText('leftPwm',data.leftPwm,0); setText('rightPwm',data.rightPwm,0);
    setText('imuRawAxG',data.imuRawAxG,3); setText('imuRawAyG',data.imuRawAyG,3); setText('imuRawAzG',data.imuRawAzG,3); setText('imuRawAccelNormG',data.imuRawAccelNormG,3);
    setText('imuRawGxDps',data.imuRawGxDps,2); setText('imuRawGyDps',data.imuRawGyDps,2); setText('imuRawGzDps',data.imuRawGzDps,2);
    setText('imuRawMx',data.imuRawMagReady?data.imuRawMx:'N/A',1); setText('imuRawMy',data.imuRawMagReady?data.imuRawMy:'N/A',1); setText('imuRawMz',data.imuRawMagReady?data.imuRawMz:'N/A',1);
    setText('imuRawAddress',hexByte(data.imuRawAddress),0); setText('imuRawId',hexByte(data.imuRawId),0); setText('imuRawMagId',hexByte(data.imuRawMagId),0); setText('imuRawReady',(data.imuRawAccelReady?'A':'-')+'/'+(data.imuRawGyroReady?'G':'-')+'/'+(data.imuRawMagReady?'M':'-'),0);
    setText('imuCorrectedAxG',data.imuCorrectedAxG,3);setText('imuCorrectedAyG',data.imuCorrectedAyG,3);setText('imuCorrectedAzG',data.imuCorrectedAzG,3);
    setText('imuCorrectedGxDps',data.imuCorrectedGxDps,2);setText('imuCorrectedGxDpsValue',data.imuCorrectedGxDps,2);setText('imuCorrectedGyDps',data.imuCorrectedGyDps,2);setText('imuCorrectedGzDps',data.imuCorrectedGzDps,2);
    setText('imuCorrectedMxUt',data.imuCorrectedMxUt,2);setText('imuCorrectedMyUt',data.imuCorrectedMyUt,2);setText('imuCorrectedMzUt',data.imuCorrectedMzUt,2);setText('imuMagNormUt',data.imuMagNormUt,2);
    setText('imuAccelRollDeg',data.imuAccelRollDeg,2);setText('imuAccelPitchDeg',data.imuAccelPitchDeg,2);setText('imuFilteredRollDeg',data.imuFilteredRollDeg,2);setText('imuFilteredPitchDeg',data.imuFilteredPitchDeg,2);setText('imuRelativeRollDeg',data.imuRelativeRollDeg,2);setText('imuRelativePitchDeg',data.imuRelativePitchDeg,2);setText('imuHeadingDeg',data.imuHeadingDeg,1);
    setText('imuAccelRateHz',data.imuAccelRateHz,0);setText('imuGyroRateHz',data.imuGyroRateHz,0);setText('imuMagRateHz',data.imuMagRateHz,0);
    setText('imuAccelCalibrated',data.imuAccelCalibrated?'VALIDA':'PENDIENTE',0);setText('imuGyroCalibrated',data.imuGyroCalibrated?'VALIDA':'PENDIENTE',0);setText('imuMagCalibrated',data.imuMagCalibrated?'VALIDA':'PENDIENTE',0);setText('imuVerticalCalibrated',data.imuVerticalCalibrated?'VALIDA':'PENDIENTE',0);setText('imuFilterReady',data.imuFilterReady?'LISTO':'NO',0);
    setText('imuCalibrationStatus',data.imuCalibrationStatus,0);setText('imuCalibrationMode',data.imuCalibrationMode,0);setText('imuCalibrationSamples',data.imuCalibrationSamples,0);setText('imuCalibrationStored',data.imuCalibrationStored?'CARGADA':'VACIA',0);setText('imuAccelPose',data.imuAccelWizardActive?(data.imuAccelPoseIndex+1)+'/6 '+data.imuAccelPoseName:'No iniciado',0);
    setText('shadowStatus',data.faultMessage,0);setText('statePosition',data.statePosition,1);setText('stateRawVelocity',data.stateRawVelocity,1);setText('stateVelocity',data.stateVelocity,1);setText('stateAngleError',data.stateAngleError,3);setText('stateAngularVelocity',data.stateAngularVelocity,2);setText('stateRawAngularAcceleration',data.stateRawAngularAcceleration,1);setText('stateAngularAcceleration',data.stateAngularAcceleration,1);setText('statePositionTerm',data.statePositionTerm,1);setText('stateVelocityTerm',data.stateVelocityTerm,1);setText('stateAngleTerm',data.stateAngleTerm,1);setText('stateAngularVelocityTerm',data.stateAngularVelocityTerm,1);setText('stateAngularAccelerationTerm',data.stateAngularAccelerationTerm,1);setText('stateOutputBeforeLimit',data.stateOutputBeforeLimit,1);setText('stateOutputSaturated',data.stateOutputSaturated?'SI':'NO',0);setText('stateSaturationCorrection',data.stateSaturationCorrection,1);setText('shadowOutput',data.shadowPidOutput,0);setText('shadowLeftPwm',data.leftPwm,0);setText('shadowRightPwm',data.rightPwm,0);setText('shadowSampleAge',data.imuSampleAgeMs,0);setText('shadowAppliedSetpoint',data.setpoint,3);setText('controlSettingsStatus',(data.controlSettingsSaved?'OK: ':'ERROR: ')+data.controlSettingsMessage,0);setInput('stateGainPosition',data.stateGainPosition,5,stateFeedbackDirty);setInput('stateGainVelocity',data.stateGainVelocity,5,stateFeedbackDirty);setInput('stateGainAngle',data.stateGainAngle,3,stateFeedbackDirty);setInput('stateGainAngularVelocity',data.stateGainAngularVelocity,3,stateFeedbackDirty);setInput('stateGainAngularAcceleration',data.stateGainAngularAcceleration,6,stateFeedbackDirty);setInput('stateVelocityFilterBeta',data.stateVelocityFilterBeta,3,stateFeedbackDirty);setInput('stateAngularAccelerationFilterBeta',data.stateAngularAccelerationFilterBeta,3,stateFeedbackDirty);setInput('shadowSetpoint',data.targetSetpoint,3,shadowSetpointDirty);setInput('shadowMaxPwm',data.maxPwm,0,shadowMaxPwmDirty);if(data.stateCalibrationWizardActive)renderStateWizard(data.stateCalibrationStage);
    benchArmed=data.benchTestArmed;setText('benchStatus',data.benchTestArmed?(data.benchTestActive?'ARMADA - MOTOR ACTIVO':'ARMADA'):'DESARMADA',0);setText('benchRemaining',data.benchArmRemainingMs/1000,1);setText('benchWatchdog',data.benchWatchdogAgeMs,0);setText('benchCommand',data.benchTestCommand,0);setText('benchLeftPwm',data.leftPwm,0);setText('benchRightPwm',data.rightPwm,0);setText('benchLeftSpeed',data.leftSpeed,0);setText('benchRightSpeed',data.rightSpeed,0);setText('benchRawLeft',data.rawLeftEncoder,0);setText('benchRawRight',data.rawRightEncoder,0);setText('benchCorrectedLeft',data.leftEncoder,0);setText('benchCorrectedRight',data.rightEncoder,0);document.getElementById('benchArm').disabled=data.benchTestArmed;document.getElementById('benchDisarm').disabled=!data.benchTestArmed;document.querySelectorAll('.bench-drive').forEach(button=>button.disabled=!data.benchTestArmed);
    document.getElementById('imuCalibrationValues').textContent=`A offset: ${data.imuAccelOffset.map(v=>v.toFixed(4)).join(', ')}\nA escala: ${data.imuAccelScale.map(v=>v.toFixed(4)).join(', ')}\nG offset: ${data.imuGyroOffset.map(v=>v.toFixed(3)).join(', ')}\nM offset: ${data.imuMagOffset.map(v=>v.toFixed(1)).join(', ')}\nM escala: ${data.imuMagScale.map(v=>v.toFixed(3)).join(', ')}\nVertical: roll ${data.imuVerticalRollDeg.toFixed(2)}, pitch ${data.imuVerticalPitchDeg.toFixed(2)}`;
    const imuBusy=data.imuCalibrationMode!=='idle';document.getElementById('sensorGyroCal').disabled=imuBusy;document.getElementById('sensorMagCal').disabled=imuBusy||!data.imuRawMagReady;document.getElementById('sensorVerticalCal').disabled=imuBusy||!data.imuFilterReady;document.getElementById('sensorAccelStart').disabled=imuBusy;document.getElementById('sensorAccelCapture').disabled=imuBusy||!data.imuAccelWizardActive;document.getElementById('sensorClearCal').disabled=imuBusy;
    setText('motorsEnabled',data.motorsEnabled?'ON':'OFF',0); setText('imuReady',data.imuReady?'OK':'NO',0); setText('safetyStop',data.safetyStop?'STOP':'OK',0);
    setText('autoRecoveryState',data.autoRecoveryState,0); setText('autoRecoveryStableMs',data.autoRecoveryStableMs,0);
    setText('autoTrimEnabled',data.autoTrimEnabled?'ON':'OFF',0); setText('autoTrimDone',data.autoTrimDone?'YES':'NO',0); setText('autoTrimOffset',data.autoTrimOffset,3); setText('autoTrimScore',data.autoTrimScore,2); setText('autoTrimBestScore',data.autoTrimBestScore,2); setText('autoTrimNoImprovementCycles',data.autoTrimNoImprovementCycles,0); setText('autoTrimPhase',data.autoTrimPhase,0); setText('autoTrimDirection',data.autoTrimDirection,0); setText('autoTrimFinalSetpoint',data.setpoint,3); setText('autoTrimStableElapsedMs',data.autoTrimStableElapsedMs,0); setText('autoTrimBlockReason',data.autoTrimBlockReason,0); setText('autoTrimStopReason',data.autoTrimStopReason || '--',0);
    setText('faultMessage',data.faultMessage,0); setText('pidError',data.pidError); setText('pidOutput',data.pidOutput); setText('pTerm',data.pTerm); setText('iTerm',data.iTerm); setText('dTerm',data.dTerm); setText('integral',data.integral,4);
    setText('integralLimitValue',data.integralLimit,4); setText('iTermLimitValue',data.iTermLimit); setText('integralEnabledValue',data.integralEnabled?'ON':'OFF',0);
    setText('outputBeforeLimit',data.outputBeforeLimit); setText('outputAfterLimit',data.outputAfterLimit);
    setText('leftEncoder',data.leftEncoder,0); setText('rightEncoder',data.rightEncoder,0); setText('leftSpeed',data.leftSpeed); setText('rightSpeed',data.rightSpeed); setText('speedDifference',data.speedDifference); setText('encoderSyncTargetDifference',data.encoderSyncTargetDifference); setText('encoderSyncError',data.encoderSyncError); setText('encoderSyncCorrection',data.encoderSyncCorrection,0); setText('encoderSyncEnabled',data.encoderSyncEnabled?'ON':'OFF',0);
    setText('gyroZHoldEnabled',data.gyroZHoldEnabled?'ON':'OFF',0); setText('gyroZHoldCorrection',data.gyroZHoldCorrection,0); setText('gyroZHoldDeadband',data.gyroZHoldDeadband);
    setText('speedHoldEnabled',data.speedHoldEnabled?'ON':'OFF',0); setText('speedHoldAngleCorrection',data.speedHoldAngleCorrection); setText('speedHoldAverage',data.speedAverage); setText('speedHoldDeadband',data.speedHoldDeadband);
    setText('driveCommandActive',data.driveCommandActive?'ON':'OFF',0); setText('driveForwardValue',data.driveForward); setText('driveTurnValue',data.driveTurn); setText('driveTurnPwm',data.driveTurnPwm,0);
    setInput('kp',data.kp,3,pidDirty); setInput('ki',data.ki,3,pidDirty); setInput('kd',data.kd,3,pidDirty); setInput('setpoint',data.setpoint,3,setpointDirty); setInput('maxPwm',data.maxPwm,0,pwmDirty);
    setInput('motorLeftMinPwm',data.motorLeftMinPwm,0,motorLimitsDirty);setInput('motorLeftMaxPwm',data.motorLeftMaxPwm,0,motorLimitsDirty);setInput('motorRightMinPwm',data.motorRightMinPwm,0,motorLimitsDirty);setInput('motorRightMaxPwm',data.motorRightMaxPwm,0,motorLimitsDirty);setInput('motorLeftCompensation',data.motorLeftCompensation,2,motorLimitsDirty);setInput('motorRightCompensation',data.motorRightCompensation,2,motorLimitsDirty);
    setInput('integralLimit',data.integralLimit,3,integralLimitDirty); setInput('iTermLimit',data.iTermLimit,1,iTermLimitDirty);
    setInput('encoderSyncKp',data.encoderSyncKp,4,encoderSyncDirty); setInput('encoderSyncDeadband',data.encoderSyncDeadband,1,encoderSyncDirty); setInput('encoderSyncMaxCorrection',data.encoderSyncMaxCorrection,0,encoderSyncDirty);
    setInput('gyroZHoldKp',data.gyroZHoldKp,4,gyroZHoldDirty); setInput('gyroZHoldMaxCorrection',data.gyroZHoldMaxCorrection,0,gyroZHoldDirty);
    setInput('speedHoldKp',data.speedHoldKp,5,speedHoldDirty); setInput('speedHoldMaxAngle',data.speedHoldMaxAngle,2,speedHoldDirty);
    updateFastMotorInputs(data.encoderSyncTargetDifference);
  };
}
window.addEventListener('blur',stopDrive);
window.addEventListener('resize',drawStateChart);
window.addEventListener('blur',stopBench);
document.addEventListener('visibilitychange',()=>{if(document.hidden)stopBench()});
window.addEventListener('beforeunload',stopBench);
document.getElementById('benchArm').addEventListener('click',()=>{if(!document.getElementById('benchConfirm').checked){alert('Confirma primero que las ruedas estan suspendidas.');return;}send({type:'bench_arm'})});
document.getElementById('benchDisarm').addEventListener('click',()=>{stopBench();send({type:'bench_disarm'})});
document.getElementById('benchStop').addEventListener('click',stopBench);
bindBenchButton('benchLeftPositive',1,0);bindBenchButton('benchLeftNegative',-1,0);bindBenchButton('benchRightPositive',0,1);bindBenchButton('benchRightNegative',0,-1);bindBenchButton('benchBothPositive',1,1);bindBenchButton('benchBothNegative',-1,-1);
markDirty();
connect();
</script>
</body>
</html>
)rawliteral";

void onWebSocketEvent(uint8_t clientId, WStype_t type, uint8_t *payload, size_t length) {
  if (type == WStype_CONNECTED) {
    String json = stateAsJson();
    webSocket.sendTXT(clientId, json);
  } else if (type == WStype_TEXT) {
    payload[length] = '\0';
    handleMessage(clientId, reinterpret_cast<const char *>(payload));
  } else if (type == WStype_DISCONNECTED) {
    SharedState::requestBenchTestDisarm();
  }
}

void registerRoutes() {
  server.on(F("/"), HTTP_GET, []() { server.send_P(200, PSTR("text/html"), PAGE); });
  server.on(F("/state"), HTTP_GET, []() { server.send(200, F("application/json"), stateAsJson()); });
}

void sendStateIfDue() {
  const unsigned long now = millis();
  if (now - lastStateSendMs < Config::WS_STATE_INTERVAL_MS) {
    return;
  }
  lastStateSendMs = now;
  String json = stateAsJson();
  webSocket.broadcastTXT(json);
}

}  // namespace

namespace WebDebug {

void begin() {
  Serial.print(F("WebDebug running on core "));
  Serial.println(xPortGetCoreID());

  WiFi.mode(WIFI_STA);
  if (!WiFi.config(Config::WIFI_LOCAL_IP,
                   Config::WIFI_GATEWAY,
                   Config::WIFI_SUBNET,
                   Config::WIFI_DNS)) {
    Serial.println(F("Failed to configure static WiFi IP"));
  }
  WiFi.begin(Config::WIFI_SSID, Config::WIFI_PASSWORD);
  Serial.print(F("Connecting to WiFi SSID: "));
  Serial.println(Config::WIFI_SSID);

  const unsigned long startMs = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startMs < Config::WIFI_CONNECT_TIMEOUT_MS) {
    Serial.print('.');
    vTaskDelay(pdMS_TO_TICKS(500));
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print(F("Web debug IP: "));
    Serial.println(WiFi.localIP());
    Serial.println(F("WebSocket port: 81"));
    ota_begin(Config::OTA_HOSTNAME, Config::OTA_PASSWORD);
  } else {
    Serial.println(F("WiFi not connected. Update WIFI_SSID/WIFI_PASSWORD in config.h."));
  }

  registerRoutes();
  server.begin();
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);
  Serial.println(F("Web server and WebSocket started"));
}

void handleClient() {
  server.handleClient();
  webSocket.loop();
  ota_handle();
  sendStateIfDue();
}

}  // namespace WebDebug
