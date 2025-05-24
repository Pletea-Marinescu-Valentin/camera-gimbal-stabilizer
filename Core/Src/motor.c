// motor.c
#include "motor.h"
#include <math.h>

// PID controller parameters for stabilization
#define PID_P_GAIN     2.0f    // Proportional gain
#define PID_I_GAIN     0.01f   // Integral gain
#define PID_D_GAIN     0.5f    // Derivative gain
#define MAX_ANGLE      30.0f   // Maximum tilt angle in degrees

static float errorSumX = 0.0f;
static float errorSumY = 0.0f;
static float lastErrorX = 0.0f;
static float lastErrorY = 0.0f;

/**
 * @brief Initialize motors by starting PWM generation
 */
void Motor_Init(TIM_HandleTypeDef *htim)
{
    // Start PWM on both channels
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2);

    // Initialize motors to stopped state
    Motor_Stop(htim);
}

/**
 * @brief Set specific motor speed
 * @param htim: Timer handle
 * @param motor: Motor number (1 or 2)
 * @param speed: Speed value (0-1000)
 */
void Motor_SetSpeed(TIM_HandleTypeDef *htim, uint8_t motor, uint16_t speed)
{
    // Limit speed to valid range
    if (speed > 1000) speed = 1000;

    // Convert 0-1000 range to pulse width
    uint16_t pulse = MOTOR_MIN_PULSE + (speed * (MOTOR_MAX_PULSE - MOTOR_MIN_PULSE)) / 1000;

    // Set the pulse width for the specified motor
    if (motor == 1) {
        __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, pulse);
    } else if (motor == 2) {
        __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, pulse);
    }
}

/**
 * @brief Arm ESCs (send minimum pulse for a period)
 */
void Motor_Arm(TIM_HandleTypeDef *htim)
{
    // Send minimum pulse to arm the ESCs
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, MOTOR_ARM_PULSE);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, MOTOR_ARM_PULSE);

    // Wait for ESCs to recognize the signal
    HAL_Delay(2000);
}

/**
 * @brief Stop motors
 */
void Motor_Stop(TIM_HandleTypeDef *htim)
{
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, MOTOR_MIN_PULSE);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, MOTOR_MIN_PULSE);
}

/**
 * @brief Control motors based on angleX and angleY (PID control)
 * @param angleX: Roll angle from Kalman filter
 * @param angleY: Pitch angle from Kalman filter
 */
void Motor_ControlFromAngle(TIM_HandleTypeDef *htim, float angleX, float angleY)
{
    // Calculate errors (target is 0 degrees - level)
    float errorX = 0 - angleX;
    float errorY = 0 - angleY;

    // Limit error accumulation
    if (fabsf(errorX) < 10.0f) errorSumX += errorX;
    if (fabsf(errorY) < 10.0f) errorSumY += errorY;

    // Limit integral term
    if (errorSumX > 100.0f) errorSumX = 100.0f;
    if (errorSumX < -100.0f) errorSumX = -100.0f;
    if (errorSumY > 100.0f) errorSumY = 100.0f;
    if (errorSumY < -100.0f) errorSumY = -100.0f;

    // Calculate derivative term
    float errorDiffX = errorX - lastErrorX;
    float errorDiffY = errorY - lastErrorY;

    // Save current errors for next iteration
    lastErrorX = errorX;
    lastErrorY = errorY;

    // Calculate PID output
    float outputX = (PID_P_GAIN * errorX) + (PID_I_GAIN * errorSumX) + (PID_D_GAIN * errorDiffX);
    float outputY = (PID_P_GAIN * errorY) + (PID_I_GAIN * errorSumY) + (PID_D_GAIN * errorDiffY);

    // Map PID output to motor speeds
    // Base speed of 500 (adjust as needed)
    int16_t baseSpeed = 500;

    // Calculate motor speeds
    int16_t motor1Speed = baseSpeed + (int16_t)outputY - (int16_t)outputX;
    int16_t motor2Speed = baseSpeed + (int16_t)outputY + (int16_t)outputX;

    // Ensure speeds are within valid range
    if (motor1Speed < 0) motor1Speed = 0;
    if (motor1Speed > 1000) motor1Speed = 1000;
    if (motor2Speed < 0) motor2Speed = 0;
    if (motor2Speed > 1000) motor2Speed = 1000;

    // Apply speeds to motors
    Motor_SetSpeed(htim, 1, (uint16_t)motor1Speed);
    Motor_SetSpeed(htim, 2, (uint16_t)motor2Speed);
}
