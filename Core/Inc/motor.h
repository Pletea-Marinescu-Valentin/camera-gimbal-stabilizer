#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f4xx_hal.h"

// Motor minimum and maximum pulse widths (in microseconds)
// Adjust these based on your ESC specifications
#define MOTOR_MIN_PULSE      1000    // Min pulse width in μs (1ms)
#define MOTOR_MAX_PULSE      2000    // Max pulse width in μs (2ms)
#define MOTOR_ARM_PULSE      1000    // Arming pulse width

// Function prototypes
void Motor_Init(TIM_HandleTypeDef *htim);
void Motor_SetSpeed(TIM_HandleTypeDef *htim, uint8_t motor, uint16_t speed);
void Motor_Arm(TIM_HandleTypeDef *htim);
void Motor_Stop(TIM_HandleTypeDef *htim);
void Motor_ControlFromAngle(TIM_HandleTypeDef *htim, float angleX, float angleY);

#endif // MOTOR_H
