#ifndef GIMBAL_CONTROL_H
#define GIMBAL_CONTROL_H

#include "main.h"
#include "kalman.h"
#include "mpu6050.h"

// Structure pentru controlul gimbal-ului
typedef struct {
    // Handle-uri pentru timer-ele PWM
    TIM_HandleTypeDef *htim_servo;   // TIM3 pentru servo-uri
    TIM_HandleTypeDef *htim_motor;   // TIM1 pentru L298N

    // Handle pentru I2C și senzor
    I2C_HandleTypeDef *hi2c;

    // Filtre Kalman pentru pitch și roll
    Kalman_t kalman_pitch;
    Kalman_t kalman_roll;

    // Variabile PID
    float kp, ki, kd;
    float integral_pitch, integral_roll;
    float last_error_pitch, last_error_roll;

    // Target-uri și valori curente
    float target_pitch, target_roll;
    float current_pitch, current_roll;

    // Limitele și setările
    float max_angle;        // Limita maximă de înclinare
    float servo_center_us;  // Centru servo în microsecunde
    float servo_range_us;   // Intervalul servo în microsecunde
    float motor_max_speed;  // Viteza maximă motor (0.0-1.0)

    // Timing
    uint32_t last_update_time;

} GimbalController_t;

// Funcții publice
HAL_StatusTypeDef Gimbal_Init(GimbalController_t *gimbal,
                             TIM_HandleTypeDef *htim_servo,
                             TIM_HandleTypeDef *htim_motor,
                             I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef Gimbal_Update(GimbalController_t *gimbal);
void Gimbal_SetTarget(GimbalController_t *gimbal, float pitch, float roll);
void Gimbal_SetPIDGains(GimbalController_t *gimbal, float kp, float ki, float kd);
void Gimbal_EmergencyStop(GimbalController_t *gimbal);

#endif
