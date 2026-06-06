#include <Arduino.h>

#include "config.h"
#include "robot_control.h"
#include "shared_state.h"
#include "web_debug.h"

namespace {

void webTask(void *parameter) {
  (void)parameter;
  WebDebug::begin();

  for (;;) {
    WebDebug::handleClient();
    vTaskDelay(pdMS_TO_TICKS(Config::WEB_TASK_PERIOD_MS));
  }
}

void controlTask(void *parameter) {
  (void)parameter;
  RobotControl::begin();

  for (;;) {
    RobotControl::update();
    vTaskDelay(pdMS_TO_TICKS(Config::CONTROL_TASK_PERIOD_MS));
  }
}

}  // namespace

void setup() {
  Serial.begin(Config::SERIAL_BAUD_RATE);
  Serial.println();
  Serial.println(F("Booting Robot Balancin FreeRTOS firmware"));

  SharedState::begin();

  xTaskCreatePinnedToCore(webTask,
                          "WebTask",
                          Config::WEB_TASK_STACK_SIZE,
                          nullptr,
                          Config::WEB_TASK_PRIORITY,
                          nullptr,
                          Config::WEB_TASK_CORE);

  xTaskCreatePinnedToCore(controlTask,
                          "ControlTask",
                          Config::CONTROL_TASK_STACK_SIZE,
                          nullptr,
                          Config::CONTROL_TASK_PRIORITY,
                          nullptr,
                          Config::CONTROL_TASK_CORE);
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}
