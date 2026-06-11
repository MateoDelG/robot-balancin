#pragma once

#include <Arduino.h>

namespace Config {

constexpr unsigned long SERIAL_BAUD_RATE = 115200;

constexpr BaseType_t WEB_TASK_CORE = 0;
constexpr BaseType_t CONTROL_TASK_CORE = 1;
constexpr uint32_t WEB_TASK_STACK_SIZE = 8192;
constexpr uint32_t CONTROL_TASK_STACK_SIZE = 8192;
constexpr UBaseType_t WEB_TASK_PRIORITY = 1;
constexpr UBaseType_t CONTROL_TASK_PRIORITY = 2;
constexpr TickType_t WEB_TASK_PERIOD_MS = 10;
constexpr TickType_t CONTROL_TASK_PERIOD_MS = 10;
constexpr unsigned long WS_STATE_INTERVAL_MS = 100;

constexpr char WIFI_SSID[] = "Delga1213";
constexpr char WIFI_PASSWORD[] = "kike4325";
constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 15000;
constexpr char OTA_HOSTNAME[] = "robot-balancin";
constexpr char OTA_PASSWORD[] = "robot123";
constexpr uint16_t OTA_PORT = 3232;

constexpr int PIN_I2C_SDA = 21;
constexpr int PIN_I2C_SCL = 22;
constexpr uint32_t I2C_CLOCK_HZ = 400000;

constexpr int PIN_ENCODER_LEFT_A = 34;
constexpr int PIN_ENCODER_LEFT_B = 35;
constexpr int PIN_ENCODER_RIGHT_A = 32;
constexpr int PIN_ENCODER_RIGHT_B = 33;

constexpr int PIN_MOTOR_LEFT_ENABLE_PWM = 25;
constexpr int PIN_MOTOR_LEFT_IN1 = 26;
constexpr int PIN_MOTOR_LEFT_IN2 = 27;

constexpr int PIN_MOTOR_RIGHT_ENABLE_PWM = 13;
constexpr int PIN_MOTOR_RIGHT_IN1 = 14;
constexpr int PIN_MOTOR_RIGHT_IN2 = 12;

constexpr int PWM_CHANNEL_MOTOR_LEFT = 0;
constexpr int PWM_CHANNEL_MOTOR_RIGHT = 1;
constexpr uint32_t PWM_FREQUENCY_HZ = 2000;
constexpr uint8_t PWM_RESOLUTION_BITS = 8;
constexpr int PWM_MAX_DUTY = 255;
constexpr int MOTOR_TEST_PWM = 80;
constexpr int MOTOR_TEST_MAX_PWM = 255;
constexpr unsigned long MOTOR_TEST_RUN_MS = 1000;
constexpr unsigned long MOTOR_TEST_PAUSE_MS = 700;

constexpr bool MOTOR_LEFT_INVERTED = false;
constexpr bool MOTOR_RIGHT_INVERTED = false;
constexpr bool ENCODER_LEFT_INVERTED = true;
constexpr bool ENCODER_RIGHT_INVERTED = true;

constexpr unsigned long IMU_PRINT_INTERVAL_MS = 250;
constexpr unsigned long ENCODER_PRINT_INTERVAL_MS = 250;
constexpr uint16_t GYRO_CALIBRATION_SAMPLES = 500;
constexpr unsigned long GYRO_CALIBRATION_SAMPLE_DELAY_MS = 3;
constexpr float COMPLEMENTARY_FILTER_ALPHA = 0.98f;
constexpr float KALMAN_Q_ANGLE = 0.001f;
constexpr float KALMAN_Q_BIAS = 0.003f;
constexpr float KALMAN_R_MEASURE = 0.03f;
constexpr float KALMAN_Q_ANGLE_MIN = 0.00001f;
constexpr float KALMAN_Q_ANGLE_MAX = 0.1f;
constexpr float KALMAN_Q_BIAS_MIN = 0.00001f;
constexpr float KALMAN_Q_BIAS_MAX = 0.1f;
constexpr float KALMAN_R_MEASURE_MIN = 0.0001f;
constexpr float KALMAN_R_MEASURE_MAX = 1.0f;
constexpr float ANGLE_VERTICAL_OFFSET_DEG = -1.78f;
constexpr bool INVERT_BALANCE_ANGLE = true;
constexpr bool INVERT_GYRO_RATE = false;
constexpr bool INVERT_TURN_GYRO = false;
constexpr float TURN_GYRO_DEADBAND_DPS = 5.0f;
constexpr bool INITIAL_GYRO_Z_HOLD_ENABLED = true;
constexpr double INITIAL_GYRO_Z_HOLD_KP = 0.35;
constexpr double GYRO_Z_HOLD_KP_MIN = 0.0;
constexpr double GYRO_Z_HOLD_KP_MAX = 5.0;
constexpr float GYRO_Z_HOLD_DEADBAND_DPS = 3.0f;
constexpr int INITIAL_GYRO_Z_HOLD_MAX_CORRECTION = 40;
constexpr int GYRO_Z_HOLD_MAX_CORRECTION_MIN = 0;
constexpr int GYRO_Z_HOLD_MAX_CORRECTION_MAX = 100;
constexpr bool INITIAL_SPEED_HOLD_ENABLED = true;
constexpr bool INVERT_SPEED_HOLD_CORRECTION = true;
constexpr double INITIAL_SPEED_HOLD_KP = 0.003;
constexpr double SPEED_HOLD_KP_MIN = 0.0;
constexpr double SPEED_HOLD_KP_MAX = 0.05;
constexpr float SPEED_HOLD_DEADBAND_COUNTS_PER_SEC = 8.0f;
constexpr double INITIAL_SPEED_HOLD_MAX_ANGLE_DEG = 3.0;

constexpr double SPEED_HOLD_MAX_ANGLE_MIN_DEG = 0.0;
constexpr double SPEED_HOLD_MAX_ANGLE_MAX_DEG = 6.0;

constexpr float INITIAL_MAX_DRIVE_ANGLE_DEG = 2.0f;

constexpr int INITIAL_MAX_DRIVE_TURN_PWM = 30;
constexpr unsigned long DRIVE_COMMAND_TIMEOUT_MS = 300;

constexpr float DRIVE_COMMAND_STEP = 0.08f;

constexpr bool INVERT_DRIVE_FORWARD = true;
constexpr bool INVERT_DRIVE_TURN = true;

constexpr double INITIAL_PID_KP = 15;
constexpr double INITIAL_PID_KI = 300;
constexpr double INITIAL_PID_KD = 0.9;
constexpr double INITIAL_ANGLE_SETPOINT_DEG = 0.132;
constexpr int INITIAL_PID_MAX_PWM = 200;
constexpr bool INVERT_PID_OUTPUT = true;
constexpr int INITIAL_MOTOR_DEADZONE_PWM = 60;
constexpr int MOTOR_DEADZONE_PWM_MIN = 0;
constexpr int MOTOR_DEADZONE_PWM_MAX = 200;
constexpr double INITIAL_INTEGRAL_LIMIT = 0.25;
constexpr double INTEGRAL_LIMIT_MIN = 0.0;
constexpr double INTEGRAL_LIMIT_MAX = 2.0;
constexpr double INITIAL_I_TERM_LIMIT = 20.0;
constexpr double I_TERM_LIMIT_MIN = 0.0;
constexpr double I_TERM_LIMIT_MAX = 100.0;
constexpr bool INITIAL_ENCODER_SYNC_ENABLED = false;
constexpr double INITIAL_ENCODER_SYNC_KP = 0.02;
constexpr double ENCODER_SYNC_KP_MIN = 0.0;
constexpr double ENCODER_SYNC_KP_MAX = 2.0;
constexpr float INITIAL_ENCODER_SYNC_DEADBAND = 1.0f;
constexpr float ENCODER_SYNC_DEADBAND_MIN = 0.0f;
constexpr float ENCODER_SYNC_DEADBAND_MAX = 300.0f;
constexpr int INITIAL_ENCODER_SYNC_MAX_CORRECTION = 5;
constexpr int ENCODER_SYNC_MAX_CORRECTION_MIN = 0;
constexpr int ENCODER_SYNC_MAX_CORRECTION_MAX = 80;
constexpr float INITIAL_ENCODER_SYNC_TARGET_DIFFERENCE = 1.0f;
constexpr float ENCODER_SYNC_TARGET_DIFFERENCE_MIN = -300.0f;
constexpr float ENCODER_SYNC_TARGET_DIFFERENCE_MAX = 300.0f;
constexpr float MAX_SAFE_ANGLE_DEG = 35.0f;
constexpr unsigned long PID_SERIAL_PRINT_INTERVAL_MS = 100;
constexpr double PID_KP_MIN = 0.0;
constexpr double PID_KP_MAX = 1000.0;
constexpr double PID_KI_MIN = 0.0;
constexpr double PID_KI_MAX = 1000.0;
constexpr double PID_KD_MIN = 0.0;
constexpr double PID_KD_MAX = 1000.0;
constexpr int PID_MAX_PWM_MIN = 0;
constexpr int PID_MAX_PWM_MAX = 255;
constexpr double SETPOINT_MIN_DEG = -10.0;
constexpr double SETPOINT_MAX_DEG = 10.0;
constexpr float FILTER_ALPHA_MIN = 0.80f;
constexpr float FILTER_ALPHA_MAX = 0.995f;

}  // namespace Config
