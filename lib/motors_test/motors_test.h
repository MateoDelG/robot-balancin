#pragma once

#include <Arduino.h>

namespace MotorsTest {

void begin();
void disable();
void stop();
void setLeftPwm(int pwm);
void setRightPwm(int pwm);
int getLeftPwm();
int getRightPwm();
void runLeftTest();
void runRightTest();
void runBothTest();

}  // namespace MotorsTest
