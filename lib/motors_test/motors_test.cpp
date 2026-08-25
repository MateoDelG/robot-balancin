#include "motors_test.h"

#include <math.h>

#include "../../include/config.h"
#include "control_settings.h"
#include "encoders_test.h"

namespace {

int currentLeftPwm = 0;
int currentRightPwm = 0;

int applyConfiguredLimits(int pwm, double compensation, int minimumPwm, int maximumPwm,
                          int safetyMaxPwm) {
  const int effectiveMaximum = min(maximumPwm, constrain(abs(safetyMaxPwm), 0, Config::PWM_MAX_DUTY));
  if (pwm == 0 || compensation <= 0.0 || effectiveMaximum == 0) return 0;
  const int sign = pwm > 0 ? 1 : -1;
  const int compensatedMagnitude = static_cast<int>(lround(abs(pwm) * compensation));
  if (compensatedMagnitude <= 0) return 0;
  const int magnitude = constrain(compensatedMagnitude, min(minimumPwm, effectiveMaximum),
                                  effectiveMaximum);
  return sign * magnitude;
}

void applyMotor(int channel, int in1Pin, int in2Pin, bool inverted, int pwm) {
  if (inverted) {
    pwm = -pwm;
  }

  if (pwm > 0) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
    ledcWrite(channel, pwm);
  } else if (pwm < 0) {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
    ledcWrite(channel, -pwm);
  } else {
    ledcWrite(channel, 0);
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
  }
}

void pauseWithMotorsOff() {
  MotorsTest::stop();
  vTaskDelay(pdMS_TO_TICKS(Config::MOTOR_TEST_PAUSE_MS));
}

}  // namespace

namespace MotorsTest {

void begin() {
  pinMode(Config::PIN_MOTOR_LEFT_IN1, OUTPUT);
  pinMode(Config::PIN_MOTOR_LEFT_IN2, OUTPUT);
  pinMode(Config::PIN_MOTOR_RIGHT_IN1, OUTPUT);
  pinMode(Config::PIN_MOTOR_RIGHT_IN2, OUTPUT);

  const double leftFrequency =
      ledcSetup(Config::PWM_CHANNEL_MOTOR_LEFT, Config::PWM_FREQUENCY_HZ,
                Config::PWM_RESOLUTION_BITS);
  const double rightFrequency =
      ledcSetup(Config::PWM_CHANNEL_MOTOR_RIGHT, Config::PWM_FREQUENCY_HZ,
                Config::PWM_RESOLUTION_BITS);
  ledcAttachPin(Config::PIN_MOTOR_LEFT_ENABLE_PWM, Config::PWM_CHANNEL_MOTOR_LEFT);
  ledcAttachPin(Config::PIN_MOTOR_RIGHT_ENABLE_PWM, Config::PWM_CHANNEL_MOTOR_RIGHT);

  Serial.printf("Motor PWM: requested=%lu Hz, resolution=%u bits, max=%d, actual L/R=%.0f/%.0f Hz\n",
                static_cast<unsigned long>(Config::PWM_FREQUENCY_HZ),
                Config::PWM_RESOLUTION_BITS, Config::PWM_MAX_DUTY,
                leftFrequency, rightFrequency);
  if (leftFrequency <= 0.0 || rightFrequency <= 0.0) {
    Serial.println(F("ERROR: motor PWM configuration failed"));
  }

  disable();
}

void disable() {
  ledcWrite(Config::PWM_CHANNEL_MOTOR_LEFT, 0);
  ledcWrite(Config::PWM_CHANNEL_MOTOR_RIGHT, 0);
  digitalWrite(Config::PIN_MOTOR_LEFT_IN1, LOW);
  digitalWrite(Config::PIN_MOTOR_LEFT_IN2, LOW);
  digitalWrite(Config::PIN_MOTOR_RIGHT_IN1, LOW);
  digitalWrite(Config::PIN_MOTOR_RIGHT_IN2, LOW);
  currentLeftPwm = 0;
  currentRightPwm = 0;
}

void stop() {
  disable();
}

void setLeftPwm(int pwm) {
  setLeftPwm(pwm, Config::PWM_MAX_DUTY);
}

void setLeftPwm(int pwm, int safetyMaxPwm) {
  const ControlSettings::Settings settings = ControlSettings::get();
  currentLeftPwm = applyConfiguredLimits(pwm, settings.leftCompensation, settings.leftMinPwm,
                                         settings.leftMaxPwm, safetyMaxPwm);
  applyMotor(Config::PWM_CHANNEL_MOTOR_LEFT, Config::PIN_MOTOR_LEFT_IN1,
             Config::PIN_MOTOR_LEFT_IN2, Config::MOTOR_LEFT_INVERTED, currentLeftPwm);
}

void setRightPwm(int pwm) {
  setRightPwm(pwm, Config::PWM_MAX_DUTY);
}

void setRightPwm(int pwm, int safetyMaxPwm) {
  const ControlSettings::Settings settings = ControlSettings::get();
  currentRightPwm = applyConfiguredLimits(pwm, settings.rightCompensation,
                                          settings.rightMinPwm, settings.rightMaxPwm,
                                          safetyMaxPwm);
  applyMotor(Config::PWM_CHANNEL_MOTOR_RIGHT, Config::PIN_MOTOR_RIGHT_IN1,
             Config::PIN_MOTOR_RIGHT_IN2, Config::MOTOR_RIGHT_INVERTED, currentRightPwm);
}

int getLeftPwm() {
  return currentLeftPwm;
}

int getRightPwm() {
  return currentRightPwm;
}

int getLeftMinPwm() {
  return ControlSettings::get().leftMinPwm;
}

int getLeftMaxPwm() {
  return ControlSettings::get().leftMaxPwm;
}

int getRightMinPwm() {
  return ControlSettings::get().rightMinPwm;
}

int getRightMaxPwm() {
  return ControlSettings::get().rightMaxPwm;
}

double getLeftCompensation() {
  return ControlSettings::get().leftCompensation;
}

double getRightCompensation() {
  return ControlSettings::get().rightCompensation;
}

void runLeftTest() {
  EncodersTest::resetBeforeMotorTest("left motor test");
  Serial.println(F("Motor L forward"));
  setLeftPwm(Config::MOTOR_TEST_PWM);
  vTaskDelay(pdMS_TO_TICKS(Config::MOTOR_TEST_RUN_MS));
  pauseWithMotorsOff();

  Serial.println(F("Motor L reverse"));
  setLeftPwm(-Config::MOTOR_TEST_PWM);
  vTaskDelay(pdMS_TO_TICKS(Config::MOTOR_TEST_RUN_MS));
  pauseWithMotorsOff();

  Serial.println(F("Motor L test complete"));
}

void runRightTest() {
  EncodersTest::resetBeforeMotorTest("right motor test");
  Serial.println(F("Motor R forward"));
  setRightPwm(Config::MOTOR_TEST_PWM);
  vTaskDelay(pdMS_TO_TICKS(Config::MOTOR_TEST_RUN_MS));
  pauseWithMotorsOff();

  Serial.println(F("Motor R reverse"));
  setRightPwm(-Config::MOTOR_TEST_PWM);
  vTaskDelay(pdMS_TO_TICKS(Config::MOTOR_TEST_RUN_MS));
  pauseWithMotorsOff();

  Serial.println(F("Motor R test complete"));
}

void runBothTest() {
  EncodersTest::resetBeforeMotorTest("both motors test");
  Serial.println(F("Both motors forward"));
  setLeftPwm(Config::MOTOR_TEST_PWM);
  setRightPwm(Config::MOTOR_TEST_PWM);
  vTaskDelay(pdMS_TO_TICKS(Config::MOTOR_TEST_RUN_MS));
  pauseWithMotorsOff();

  Serial.println(F("Both motors reverse"));
  setLeftPwm(-Config::MOTOR_TEST_PWM);
  setRightPwm(-Config::MOTOR_TEST_PWM);
  vTaskDelay(pdMS_TO_TICKS(Config::MOTOR_TEST_RUN_MS));
  pauseWithMotorsOff();

  Serial.println(F("Both motors test complete"));
}

}  // namespace MotorsTest
