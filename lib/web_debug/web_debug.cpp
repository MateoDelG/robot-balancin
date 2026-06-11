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
  doc["leftEncoder"] = state.correctedLeftEncoder;
  doc["rightEncoder"] = state.correctedRightEncoder;
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
  doc["motorDeadzonePwm"] = state.motorDeadzonePwm;
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
  } else if (strcmp(type, "set_motor_deadzone") == 0) {
    if (!requireNumber(doc, "deadzonePwm")) {
      sendMessage(clientId, "error", "invalid motor deadzone");
      return;
    }
    const int deadzonePwm = clampValue(doc["deadzonePwm"].as<int>(),
                                       Config::MOTOR_DEADZONE_PWM_MIN,
                                       Config::MOTOR_DEADZONE_PWM_MAX);
    SharedState::requestMotorDeadzonePwm(deadzonePwm);
    sendMessage(clientId, "ack", "motor deadzone updated");
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
  <title>Robot Balancin PID Basico</title>
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
    button.stop{background:#dc2626}.ok{background:#16a34a}.warn{background:#ca8a04}
    #status{color:#93c5fd;font-size:13px}.fault{color:#fca5a5}
  </style>
</head>
<body>
<header><h1>Robot Balancin - PID manual basico</h1><div id="status">Conectando...</div></header>
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
  </div><p class="fault" id="faultMessage">--</p></section>
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
  <section><h2>Zona muerta motores</h2>
    <p>Se suma a la salida PID no cero para saltar el rango donde los motores no responden.</p>
    <label>Zona muerta PWM</label><input id="motorDeadzonePwm" type="number" step="1">
    <button onclick="applyMotorDeadzone()">Actualizar zona muerta</button>
    <div class="item"><div class="label">PID puro</div><div class="value" id="outputBeforeLimit">--</div></div>
    <div class="item"><div class="label">PID compensado</div><div class="value" id="outputAfterLimit">--</div></div>
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
</main>
<script>
let ws;
let pidDirty = false;
let setpointDirty = false;
let pwmDirty = false;
let deadzoneDirty = false;
let integralLimitDirty = false;
let iTermLimitDirty = false;
let encoderSyncDirty = false;
let encoderSyncTargetDirty = false;
let gyroZHoldDirty = false;
let speedHoldDirty = false;
let driveTimer = null;
let driveForwardCommand = 0;
let driveTurnCommand = 0;
function num(id){return Number(document.getElementById(id).value)}
function setText(id,value,digits=2){const el=document.getElementById(id); if(!el)return; el.textContent=typeof value==='number'?value.toFixed(digits):value;}
function setInput(id,value,digits=3,dirty=false){const el=document.getElementById(id); if(!el || dirty || document.activeElement===el || typeof value!=='number')return; el.value=value.toFixed(digits);}
function setTurnDirection(direction){
  const el=document.getElementById('turnDirection');
  if(!el)return;
  el.textContent=direction || '--';
  el.className='value '+(direction==='derecha'?'turn-right':(direction==='izquierda'?'turn-left':'turn-still'));
}
function send(obj){if(!ws || ws.readyState!==WebSocket.OPEN){alert('WebSocket no conectado');return;} ws.send(JSON.stringify(obj));}
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
function applySetpoint(){send({type:'set_setpoint',setpoint:num('setpoint')}); setpointDirty=false;}
function applyPwmLimit(){send({type:'set_pwm_limit',maxPwm:num('maxPwm')}); pwmDirty=false;}
function applyMotorDeadzone(){send({type:'set_motor_deadzone',deadzonePwm:num('motorDeadzonePwm')}); deadzoneDirty=false;}
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
  document.getElementById('motorDeadzonePwm').addEventListener('input',()=>deadzoneDirty=true);
  document.getElementById('integralLimit').addEventListener('input',()=>integralLimitDirty=true);
  document.getElementById('iTermLimit').addEventListener('input',()=>iTermLimitDirty=true);
  ['encoderSyncKp','encoderSyncDeadband','encoderSyncMaxCorrection'].forEach(id=>document.getElementById(id).addEventListener('input',()=>encoderSyncDirty=true));
  ['gyroZHoldKp','gyroZHoldMaxCorrection'].forEach(id=>document.getElementById(id).addEventListener('input',()=>gyroZHoldDirty=true));
  ['speedHoldKp','speedHoldMaxAngle'].forEach(id=>document.getElementById(id).addEventListener('input',()=>speedHoldDirty=true));
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
  ws.onclose=()=>{setText('status','WebSocket desconectado',0); setTimeout(connect,1000);};
  ws.onmessage=(event)=>{
    const data=JSON.parse(event.data);
    if(data.type==='ack' || data.type==='error'){setText('status',data.message,0); console.log(data);return;}
    setText('angle',data.selectedAngle); setText('gyroRate',data.gyroRate); setTurnDirection(data.turnDirection); setText('turnRate',data.turnRate); setText('gyroZHoldTurnRate',data.turnRate); setText('leftPwm',data.leftPwm,0); setText('rightPwm',data.rightPwm,0);
    setText('motorsEnabled',data.motorsEnabled?'ON':'OFF',0); setText('imuReady',data.imuReady?'OK':'NO',0); setText('safetyStop',data.safetyStop?'STOP':'OK',0);
    setText('faultMessage',data.faultMessage,0); setText('pidError',data.pidError); setText('pidOutput',data.pidOutput); setText('pTerm',data.pTerm); setText('iTerm',data.iTerm); setText('dTerm',data.dTerm); setText('integral',data.integral,4);
    setText('integralLimitValue',data.integralLimit,4); setText('iTermLimitValue',data.iTermLimit); setText('integralEnabledValue',data.integralEnabled?'ON':'OFF',0);
    setText('outputBeforeLimit',data.outputBeforeLimit); setText('outputAfterLimit',data.outputAfterLimit);
    setText('leftEncoder',data.leftEncoder,0); setText('rightEncoder',data.rightEncoder,0); setText('leftSpeed',data.leftSpeed); setText('rightSpeed',data.rightSpeed); setText('speedDifference',data.speedDifference); setText('encoderSyncTargetDifference',data.encoderSyncTargetDifference); setText('encoderSyncError',data.encoderSyncError); setText('encoderSyncCorrection',data.encoderSyncCorrection,0); setText('encoderSyncEnabled',data.encoderSyncEnabled?'ON':'OFF',0);
    setText('gyroZHoldEnabled',data.gyroZHoldEnabled?'ON':'OFF',0); setText('gyroZHoldCorrection',data.gyroZHoldCorrection,0); setText('gyroZHoldDeadband',data.gyroZHoldDeadband);
    setText('speedHoldEnabled',data.speedHoldEnabled?'ON':'OFF',0); setText('speedHoldAngleCorrection',data.speedHoldAngleCorrection); setText('speedHoldAverage',data.speedAverage); setText('speedHoldDeadband',data.speedHoldDeadband);
    setText('driveCommandActive',data.driveCommandActive?'ON':'OFF',0); setText('driveForwardValue',data.driveForward); setText('driveTurnValue',data.driveTurn); setText('driveTurnPwm',data.driveTurnPwm,0);
    setInput('kp',data.kp,3,pidDirty); setInput('ki',data.ki,3,pidDirty); setInput('kd',data.kd,3,pidDirty); setInput('setpoint',data.setpoint,3,setpointDirty); setInput('maxPwm',data.maxPwm,0,pwmDirty);
    setInput('motorDeadzonePwm',data.motorDeadzonePwm,0,deadzoneDirty);
    setInput('integralLimit',data.integralLimit,3,integralLimitDirty); setInput('iTermLimit',data.iTermLimit,1,iTermLimitDirty);
    setInput('encoderSyncKp',data.encoderSyncKp,4,encoderSyncDirty); setInput('encoderSyncDeadband',data.encoderSyncDeadband,1,encoderSyncDirty); setInput('encoderSyncMaxCorrection',data.encoderSyncMaxCorrection,0,encoderSyncDirty);
    setInput('gyroZHoldKp',data.gyroZHoldKp,4,gyroZHoldDirty); setInput('gyroZHoldMaxCorrection',data.gyroZHoldMaxCorrection,0,gyroZHoldDirty);
    setInput('speedHoldKp',data.speedHoldKp,5,speedHoldDirty); setInput('speedHoldMaxAngle',data.speedHoldMaxAngle,2,speedHoldDirty);
    updateFastMotorInputs(data.encoderSyncTargetDifference);
  };
}
window.addEventListener('blur',stopDrive);
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
