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
constexpr bool RUN_MPU9250_TEST_DASHBOARD = false;
constexpr bool RAW_IMU_DASHBOARD_ONLY = true;

constexpr char WIFI_SSID[] = "Delga1213";
constexpr char WIFI_PASSWORD[] = "kike4325";
const IPAddress WIFI_LOCAL_IP(192, 168, 137, 243);
const IPAddress WIFI_GATEWAY(192, 168, 137, 1);
const IPAddress WIFI_SUBNET(255, 255, 255, 0);
const IPAddress WIFI_DNS(192, 168, 137, 1);
constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 15000;
constexpr char OTA_HOSTNAME[] = "robot-balancin";
constexpr char OTA_PASSWORD[] = "robot123";
constexpr uint16_t OTA_PORT = 3232;

constexpr int PIN_I2C_SDA = 21;
constexpr int PIN_I2C_SCL = 22;
constexpr uint32_t I2C_CLOCK_HZ = 400000;
constexpr unsigned long SENSOR_GYRO_CALIBRATION_MS = 3000;
constexpr unsigned long SENSOR_ACCEL_POSE_CAPTURE_MS = 2000;
constexpr unsigned long SENSOR_MAG_CALIBRATION_MS = 30000;
constexpr float SENSOR_COMPLEMENTARY_ALPHA = 0.98f;
constexpr float SENSOR_FILTER_ALPHA_MIN = 0.80f;
constexpr float SENSOR_FILTER_ALPHA_MAX = 0.999f;
constexpr unsigned long SENSOR_TRACE_INTERVAL_MS = 19;
constexpr unsigned long SENSOR_TRACE_MIN_DURATION_MS = 5000;
constexpr unsigned long SENSOR_TRACE_MAX_DURATION_MS = 300000;
constexpr double SHADOW_PID_KP = 10.0;
constexpr double SHADOW_PID_KI = 0.0;
constexpr double SHADOW_PID_KD = 0.5;
constexpr int SHADOW_PID_MAX_PWM = 255;
constexpr unsigned long SHADOW_IMU_TIMEOUT_MS = 50;
constexpr float PID_ARM_MAX_ERROR_DEG = 10.0f;
constexpr bool AUTO_START_CONTROL_ENABLED = true;
constexpr float AUTO_START_ANGLE_WINDOW_DEG = 10.0f;
constexpr unsigned long AUTO_START_STABLE_MS = 5000;
constexpr float PID_SETPOINT_SLEW_DEG_PER_SEC = 2.0f;
constexpr int BENCH_TEST_DEFAULT_PWM = 70;
constexpr int BENCH_TEST_MAX_PWM = 255;
constexpr unsigned long BENCH_TEST_ARM_TIMEOUT_MS = 30000;
constexpr unsigned long BENCH_TEST_WATCHDOG_MS = 250;

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
constexpr int MOTOR_TEST_PWM = 255;
constexpr int MOTOR_TEST_MAX_PWM = 255;
constexpr unsigned long MOTOR_TEST_RUN_MS = 1000;
constexpr unsigned long MOTOR_TEST_PAUSE_MS = 700;

constexpr bool MOTOR_LEFT_INVERTED = false;
constexpr bool MOTOR_RIGHT_INVERTED = false;
constexpr int INITIAL_MOTOR_LEFT_MIN_PWM = 58;
constexpr int INITIAL_MOTOR_LEFT_MAX_PWM = 255;
constexpr int INITIAL_MOTOR_RIGHT_MIN_PWM = 58;
constexpr int INITIAL_MOTOR_RIGHT_MAX_PWM = 255;
constexpr int MOTOR_PWM_LIMIT_MIN = 0;
constexpr int MOTOR_PWM_LIMIT_MAX = 255;
constexpr double INITIAL_MOTOR_LEFT_COMPENSATION = 1.0;
constexpr double INITIAL_MOTOR_RIGHT_COMPENSATION = 1.0;
constexpr double MOTOR_COMPENSATION_MIN = 0.0;
constexpr double MOTOR_COMPENSATION_MAX = 2.0;
constexpr double STATE_GAIN_MIN = -1000.0;
constexpr double STATE_GAIN_MAX = 1000.0;
constexpr float INITIAL_VELOCITY_FILTER_BETA = 0.20f;
constexpr float INITIAL_ANGULAR_ACCEL_FILTER_BETA = 0.10f;
constexpr float STATE_FILTER_BETA_MIN = 0.01f;
constexpr float STATE_FILTER_BETA_MAX = 1.0f;
constexpr float STATE_ARM_MAX_SPEED_COUNTS_PER_SEC = 50.0f;
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
constexpr bool AUTO_RECOVERY_ENABLED = true;
constexpr float AUTO_RECOVERY_ANGLE_WINDOW_DEG = 15.0f;
constexpr unsigned long AUTO_RECOVERY_SETTLE_MS = 2000;
constexpr bool INITIAL_AUTO_TRIM_ENABLED = false;

constexpr double AUTO_TRIM_MAX_OFFSET_DEG = 2.0;
constexpr double AUTO_TRIM_TEST_OFFSET_DEG = 0.05;
constexpr double AUTO_TRIM_APPLY_STEP_DEG = 0.01;
constexpr unsigned long AUTO_TRIM_STABLE_BEFORE_START_MS = 500;
constexpr unsigned long AUTO_TRIM_TEST_WINDOW_MS = 500;
constexpr float AUTO_TRIM_MAX_SPEED_COUNTS_PER_SEC = 2000.0f;
constexpr int AUTO_TRIM_MAX_PWM_FOR_TEST = 160;
constexpr double AUTO_TRIM_MIN_SCORE_DELTA = 0.5;
constexpr double AUTO_TRIM_TARGET_SCORE = 8.0;
constexpr uint8_t AUTO_TRIM_MAX_NO_IMPROVEMENT_CYCLES = 50;

constexpr double INITIAL_PID_KP = 15;
constexpr double INITIAL_PID_KI = 300;
constexpr double INITIAL_PID_KD = 0.9;
constexpr double INITIAL_ANGLE_SETPOINT_DEG = 5;

constexpr int INITIAL_PID_MAX_PWM = 200;
constexpr bool INVERT_PID_OUTPUT = false;
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
