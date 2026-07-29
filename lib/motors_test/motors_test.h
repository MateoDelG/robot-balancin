#pragma once

#include <Arduino.h>

namespace MotorsTest {

void begin();
void disable();
void stop();
void setLeftPwm(int pwm);
void setLeftPwm(int pwm, int safetyMaxPwm);
void setRightPwm(int pwm);
void setRightPwm(int pwm, int safetyMaxPwm);
int getLeftPwm();
int getRightPwm();
int getLeftMinPwm();
int getLeftMaxPwm();
int getRightMinPwm();
int getRightMaxPwm();
double getLeftCompensation();
double getRightCompensation();
void runLeftTest();
void runRightTest();
void runBothTest();

}  // namespace MotorsTest
