#include "motors_test.h"

#include "../../include/config.h"
#include "encoders_test.h"

namespace {

int currentLeftPwm = 0;
int currentRightPwm = 0;

int clampTestPwm(int pwm) {
  if (pwm > Config::MOTOR_TEST_MAX_PWM) {
    return Config::MOTOR_TEST_MAX_PWM;
  }
  if (pwm < -Config::MOTOR_TEST_MAX_PWM) {
    return -Config::MOTOR_TEST_MAX_PWM;
  }
  return pwm;
}

void applyMotor(int channel, int in1Pin, int in2Pin, bool inverted, int pwm) {
  pwm = clampTestPwm(pwm);
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

  ledcSetup(Config::PWM_CHANNEL_MOTOR_LEFT, Config::PWM_FREQUENCY_HZ,
            Config::PWM_RESOLUTION_BITS);
  ledcSetup(Config::PWM_CHANNEL_MOTOR_RIGHT, Config::PWM_FREQUENCY_HZ,
            Config::PWM_RESOLUTION_BITS);
  ledcAttachPin(Config::PIN_MOTOR_LEFT_ENABLE_PWM, Config::PWM_CHANNEL_MOTOR_LEFT);
  ledcAttachPin(Config::PIN_MOTOR_RIGHT_ENABLE_PWM, Config::PWM_CHANNEL_MOTOR_RIGHT);

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
  currentLeftPwm = clampTestPwm(pwm);
  applyMotor(Config::PWM_CHANNEL_MOTOR_LEFT, Config::PIN_MOTOR_LEFT_IN1,
             Config::PIN_MOTOR_LEFT_IN2, Config::MOTOR_LEFT_INVERTED, pwm);
}

void setRightPwm(int pwm) {
  currentRightPwm = clampTestPwm(pwm);
  applyMotor(Config::PWM_CHANNEL_MOTOR_RIGHT, Config::PIN_MOTOR_RIGHT_IN1,
             Config::PIN_MOTOR_RIGHT_IN2, Config::MOTOR_RIGHT_INVERTED, pwm);
}

int getLeftPwm() {
  return currentLeftPwm;
}

int getRightPwm() {
  return currentRightPwm;
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
