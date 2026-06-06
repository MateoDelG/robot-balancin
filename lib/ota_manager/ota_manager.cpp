#include "ota_manager.h"

#include <Arduino.h>
#include <ArduinoOTA.h>
#include <WiFi.h>

#include "../../include/config.h"
#include "shared_state.h"

namespace {

bool otaAvailable = false;
bool otaUpdating = false;
int lastProgressPercent = -1;

const char *errorToText(ota_error_t error) {
  switch (error) {
    case OTA_AUTH_ERROR:
      return "Auth failed";
    case OTA_BEGIN_ERROR:
      return "Begin failed";
    case OTA_CONNECT_ERROR:
      return "Connect failed";
    case OTA_RECEIVE_ERROR:
      return "Receive failed";
    case OTA_END_ERROR:
      return "End failed";
    default:
      return "Unknown error";
  }
}

}  // namespace

bool ota_begin(const char *hostname, const char *password) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(F("OTA not started: WiFi is not connected"));
    otaAvailable = false;
    SharedState::setOtaAvailable(false);
    return false;
  }

  ArduinoOTA.setHostname(hostname);
  ArduinoOTA.setPassword(password);
  ArduinoOTA.setPort(Config::OTA_PORT);

  ArduinoOTA.onStart([]() {
    otaUpdating = true;
    lastProgressPercent = -1;
    SharedState::requestOtaStart();
    SharedState::setOtaUpdating(true);
    Serial.println(F("OTA start"));
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    const int percent = total == 0 ? 0 : static_cast<int>((progress * 100U) / total);
    if (percent != lastProgressPercent) {
      lastProgressPercent = percent;
      Serial.print(F("OTA progress: "));
      Serial.print(percent);
      Serial.println('%');
    }
  });

  ArduinoOTA.onError([](ota_error_t error) {
    otaUpdating = false;
    SharedState::setOtaUpdating(false);
    Serial.print(F("OTA error: "));
    Serial.println(errorToText(error));
  });

  ArduinoOTA.onEnd([]() {
    otaUpdating = false;
    SharedState::requestOtaEnd();
    SharedState::setOtaUpdating(false);
    Serial.println(F("OTA complete"));
  });

  ArduinoOTA.begin();
  otaAvailable = true;
  SharedState::setOtaAvailable(true);

  Serial.print(F("OTA ready. Hostname: "));
  Serial.print(hostname);
  Serial.print(F(" Port: "));
  Serial.println(Config::OTA_PORT);
  return true;
}

void ota_handle() {
  if (otaAvailable) {
    ArduinoOTA.handle();
  }
}

bool ota_isUpdating() {
  return otaUpdating;
}

bool ota_isAvailable() {
  return otaAvailable;
}
