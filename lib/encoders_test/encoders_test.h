#pragma once

#include <Arduino.h>

namespace EncodersTest {

void begin();
void printCountsIfDue();
void reset();
void resetBeforeMotorTest(const char *testName);
long rawLeftCount();
long rawRightCount();
long leftCount();
long rightCount();

}  // namespace EncodersTest
