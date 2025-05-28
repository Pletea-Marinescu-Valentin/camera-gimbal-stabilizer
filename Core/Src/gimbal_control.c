#include "gimbal_control.h"
#include <math.h>

// Configuration constants
#define SERVO_MIN_PULSE_US    1000  // Minimum servo pulse width
#define SERVO_MAX_PULSE_US    2000  // Maximum servo pulse width
#define SERVO_CENTER_PULSE_US 1500  // Center servo pulse width
#define SERVO_PWM_PERIOD_US   20000 // PWM period (20ms for 50Hz)

#define MAX_TILT_ANGLE        45.0f // Maximum tilt angle in degrees

// Private functions
static void Gimbal_ControlServo(GimbalController_t *gimbal, uint8_t channel, float angle);
static float Gimbal_PIDControl(GimbalController_t *gimbal, float setpoint, float current,
                              float dt, uint8_t axis);

HAL_StatusTypeDef Gimbal_Init(GimbalController_t *gimbal,
                             TIM_HandleTypeDef *htim_servo,
                             I2C_HandleTypeDef *hi2c)
{
    // Initialize structure
    gimbal->htim_servo = htim_servo;
    gimbal->hi2c = hi2c;

    // Initialize Kalman filters
    Kalman_Init(&gimbal->kalman_pitch);
    Kalman_Init(&gimbal->kalman_roll);

    // Default PID settings
    gimbal->kp = 2.0f;
    gimbal->ki = 0.1f;
    gimbal->kd = 0.05f;
    gimbal->integral_pitch = 0.0f;
    gimbal->integral_roll = 0.0f;
    gimbal->last_error_pitch = 0.0f;
    gimbal->last_error_roll = 0.0f;

    // Default targets (center position)
    gimbal->target_pitch = 0.0f;
    gimbal->target_roll = 0.0f;

    // Servo settings
    gimbal->servo_center_us = SERVO_CENTER_PULSE_US;
    gimbal->servo_range_us = (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) / 2;
    gimbal->max_angle = MAX_TILT_ANGLE;

    // Initialize MPU6050
    if (MPU6050_Init(gimbal->hi2c) != HAL_OK) {
        return HAL_ERROR;
    }

    if (MPU6050_IsConnected(gimbal->hi2c) != HAL_OK) {
        return HAL_ERROR;
    }

    // Start PWM for servos
    if (HAL_TIM_PWM_Start(gimbal->htim_servo, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(gimbal->htim_servo, TIM_CHANNEL_2) != HAL_OK) {
        return HAL_ERROR;
    }

    // Position servos to center
    Gimbal_ControlServo(gimbal, 1, 0.0f); // Pitch servo
    Gimbal_ControlServo(gimbal, 2, 0.0f); // Roll servo

    // Read initial value for Kalman filter
    int16_t accel_x, accel_y, accel_z;
    if (MPU6050_ReadAccel(gimbal->hi2c, &accel_x, &accel_y, &accel_z) == HAL_OK) {
        float initial_pitch = atan2f((float)accel_y, (float)accel_z) * 180.0f / M_PI;
        float initial_roll = atan2f((float)-accel_x,
                                   sqrtf((float)(accel_y * accel_y + accel_z * accel_z))) * 180.0f / M_PI;

        Kalman_SetAngle(&gimbal->kalman_pitch, initial_pitch);
        Kalman_SetAngle(&gimbal->kalman_roll, initial_roll);
    }

    gimbal->last_update_time = HAL_GetTick();

    return HAL_OK;
}

HAL_StatusTypeDef Gimbal_Update(GimbalController_t *gimbal)
{
    // Calculate dt
    uint32_t current_time = HAL_GetTick();
    float dt = (float)(current_time - gimbal->last_update_time) / 1000.0f;
    gimbal->last_update_time = current_time;

    // Read sensor data
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;

    if (MPU6050_ReadAccel(gimbal->hi2c, &accel_x, &accel_y, &accel_z) != HAL_OK ||
        MPU6050_ReadGyro(gimbal->hi2c, &gyro_x, &gyro_y, &gyro_z) != HAL_OK) {
        return HAL_ERROR;
    }

    // Convert gyroscope to degrees/sec
    float gyro_pitch = (float)gyro_x / 131.0f;
    float gyro_roll = (float)gyro_y / 131.0f;

    // Calculate angles from accelerometer
    float accel_pitch = atan2f((float)accel_y, (float)accel_z) * 180.0f / M_PI;
    float accel_roll = atan2f((float)-accel_x,
                             sqrtf((float)(accel_y * accel_y + accel_z * accel_z))) * 180.0f / M_PI;

    // Apply Kalman filter
    gimbal->current_pitch = Kalman_GetAngle(&gimbal->kalman_pitch, accel_pitch, gyro_pitch, dt);
    gimbal->current_roll = Kalman_GetAngle(&gimbal->kalman_roll, accel_roll, gyro_roll, dt);

    // Calculate PID output for servos
    float servo_pitch_output = Gimbal_PIDControl(gimbal, gimbal->target_pitch,
                                                gimbal->current_pitch, dt, 0);
    float servo_roll_output = Gimbal_PIDControl(gimbal, gimbal->target_roll,
                                               gimbal->current_roll, dt, 1);

    // Limit servo angles
    if (servo_pitch_output > gimbal->max_angle) servo_pitch_output = gimbal->max_angle;
    if (servo_pitch_output < -gimbal->max_angle) servo_pitch_output = -gimbal->max_angle;
    if (servo_roll_output > gimbal->max_angle) servo_roll_output = gimbal->max_angle;
    if (servo_roll_output < -gimbal->max_angle) servo_roll_output = -gimbal->max_angle;

    // Control servos for stabilization
    Gimbal_ControlServo(gimbal, 1, servo_pitch_output); // Pitch axis
    Gimbal_ControlServo(gimbal, 2, servo_roll_output);  // Roll axis

    return HAL_OK;
}

static void Gimbal_ControlServo(GimbalController_t *gimbal, uint8_t channel, float angle)
{
    // Convert angle to pulse time (microseconds)
    float pulse_us = gimbal->servo_center_us + (angle / gimbal->max_angle) * gimbal->servo_range_us;

    // Limit values
    if (pulse_us < SERVO_MIN_PULSE_US) pulse_us = SERVO_MIN_PULSE_US;
    if (pulse_us > SERVO_MAX_PULSE_US) pulse_us = SERVO_MAX_PULSE_US;

    // Convert to PWM value (assuming TIM3 configured for 20ms period)
    // ARR should be set to achieve 20ms period at current clock frequency
    uint32_t timer_period = __HAL_TIM_GET_AUTORELOAD(gimbal->htim_servo) + 1;
    uint32_t pwm_value = (uint32_t)((pulse_us * timer_period) / SERVO_PWM_PERIOD_US);

    // Set PWM
    if (channel == 1) {
        __HAL_TIM_SET_COMPARE(gimbal->htim_servo, TIM_CHANNEL_1, pwm_value);
    } else if (channel == 2) {
        __HAL_TIM_SET_COMPARE(gimbal->htim_servo, TIM_CHANNEL_2, pwm_value);
    }
}

static float Gimbal_PIDControl(GimbalController_t *gimbal, float setpoint, float current,
                              float dt, uint8_t axis)
{
    float error = setpoint - current;

    float *integral = (axis == 0) ? &gimbal->integral_pitch : &gimbal->integral_roll;
    float *last_error = (axis == 0) ? &gimbal->last_error_pitch : &gimbal->last_error_roll;

    // Integral term with anti-windup
    *integral += error * dt;
    if (*integral > 10.0f) *integral = 10.0f;
    if (*integral < -10.0f) *integral = -10.0f;

    // Derivative term
    float derivative = (error - *last_error) / dt;

    // Calculate PID output
    float output = gimbal->kp * error + gimbal->ki * (*integral) + gimbal->kd * derivative;

    *last_error = error;

    return output;
}

void Gimbal_SetTarget(GimbalController_t *gimbal, float pitch, float roll)
{
    gimbal->target_pitch = pitch;
    gimbal->target_roll = roll;
}

void Gimbal_SetPIDGains(GimbalController_t *gimbal, float kp, float ki, float kd)
{
    gimbal->kp = kp;
    gimbal->ki = ki;
    gimbal->kd = kd;
}

void Gimbal_EmergencyStop(GimbalController_t *gimbal)
{
    // Center servos to safe position
    Gimbal_ControlServo(gimbal, 1, 0.0f); // Pitch servo
    Gimbal_ControlServo(gimbal, 2, 0.0f); // Roll servo

    // Reset PID states
    gimbal->integral_pitch = 0.0f;
    gimbal->integral_roll = 0.0f;
    gimbal->last_error_pitch = 0.0f;
    gimbal->last_error_roll = 0.0f;
}
