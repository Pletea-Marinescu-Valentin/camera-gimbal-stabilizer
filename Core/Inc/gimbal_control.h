#ifndef GIMBAL_CONTROL_H
#define GIMBAL_CONTROL_H

#include "main.h"
#include "kalman.h"
#include "mpu6050.h"

// Gimbal control structure
typedef struct {
    // PWM timer handle for servos
    TIM_HandleTypeDef *htim_servo;  // TIM3 for servo control

    // I2C and sensor handle
    I2C_HandleTypeDef *hi2c;

    // Kalman filters for pitch and roll
    Kalman_t kalman_pitch;
    Kalman_t kalman_roll;

    // PID variables
    float kp, ki, kd;
    float integral_pitch, integral_roll;
    float last_error_pitch, last_error_roll;

    // Target and current values
    float target_pitch, target_roll;
    float current_pitch, current_roll;

    // Limits and settings
    float max_angle;        // Maximum tilt limit
    float servo_center_us;  // Servo center in microseconds (1500us)
    float servo_range_us;   // Servo range in microseconds (±500us)

    // Timing
    uint32_t last_update_time;

} GimbalController_t;

// Public functions
HAL_StatusTypeDef Gimbal_Init(GimbalController_t *gimbal,
                             TIM_HandleTypeDef *htim_servo,
                             I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef Gimbal_Update(GimbalController_t *gimbal);
void Gimbal_SetTarget(GimbalController_t *gimbal, float pitch, float roll);
void Gimbal_SetPIDGains(GimbalController_t *gimbal, float kp, float ki, float kd);
void Gimbal_EmergencyStop(GimbalController_t *gimbal);

#endif
