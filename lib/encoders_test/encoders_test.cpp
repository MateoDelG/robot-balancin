#include "encoders_test.h"

#include <ESP32Encoder.h>

#include "../../include/config.h"

namespace {

ESP32Encoder leftEncoder;
ESP32Encoder rightEncoder;
unsigned long lastPrintMs = 0;

long applyLeftSign(int64_t count) {
  return static_cast<long>(Config::ENCODER_LEFT_INVERTED ? -count : count);
}

long applyRightSign(int64_t count) {
  return static_cast<long>(Config::ENCODER_RIGHT_INVERTED ? -count : count);
}

}  // namespace

namespace EncodersTest {

void begin() {
  ESP32Encoder::useInternalWeakPullResistors = puType::none;
  leftEncoder.attachFullQuad(Config::PIN_ENCODER_LEFT_A, Config::PIN_ENCODER_LEFT_B);
  rightEncoder.attachFullQuad(Config::PIN_ENCODER_RIGHT_A, Config::PIN_ENCODER_RIGHT_B);
  leftEncoder.setFilter(1023);
  rightEncoder.setFilter(1023);
  reset();
  Serial.println(F("Encoders initialized in full quadrature mode"));
  Serial.printf("Encoder scale: %.2f counts/rev, %.6f mm/count, wheel %.1f mm\n",
                Config::ENCODER_COUNTS_PER_WHEEL_REV,
                Config::MM_PER_ENCODER_COUNT, Config::WHEEL_DIAMETER_MM);
}

void printCountsIfDue() {
  const unsigned long now = millis();
  if (now - lastPrintMs < Config::ENCODER_PRINT_INTERVAL_MS) {
    return;
  }
  lastPrintMs = now;

  Serial.print(F("ENC raw_left="));
  Serial.print(rawLeftCount());
  Serial.print(F(" raw_right="));
  Serial.print(rawRightCount());
  Serial.print(F(" corrected_left="));
  Serial.print(leftCount());
  Serial.print(F(" corrected_right="));
  Serial.println(rightCount());
}

void reset() {
  leftEncoder.clearCount();
  rightEncoder.clearCount();
}

void resetBeforeMotorTest(const char *testName) {
  reset();
  Serial.print(F("Encoder counts reset before "));
  Serial.println(testName);
}

long rawLeftCount() {
  return static_cast<long>(leftEncoder.getCount());
}

long rawRightCount() {
  return static_cast<long>(rightEncoder.getCount());
}

long leftCount() {
  return applyLeftSign(rawLeftCount());
}

long rightCount() {
  return applyRightSign(rawRightCount());
}

float countsToMillimeters(float counts) {
  return counts * static_cast<float>(Config::MM_PER_ENCODER_COUNT);
}

float leftDistanceMm() {
  return countsToMillimeters(static_cast<float>(leftCount()));
}

float rightDistanceMm() {
  return countsToMillimeters(static_cast<float>(rightCount()));
}

}  // namespace EncodersTest
