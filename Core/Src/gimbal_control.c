#include "gimbal_control.h"
#include <math.h>

// Folosește definirile generate de CubeMX din main.h
// IN1_Pin, IN2_Pin, IN3_Pin, IN4_Pin sunt deja definite

// Constante pentru configurare
#define SERVO_MIN_PULSE_US    500
#define SERVO_MAX_PULSE_US    2500
#define SERVO_CENTER_PULSE_US 1500
#define SERVO_PWM_PERIOD_US   20000

#define MOTOR_PWM_MAX         1000
#define MAX_TILT_ANGLE        45.0f

// Funcții private
static void Gimbal_ControlServo(GimbalController_t *gimbal, uint8_t channel, float angle);
static void Gimbal_ControlMotor(GimbalController_t *gimbal, uint8_t motor, float speed);
static float Gimbal_PIDControl(GimbalController_t *gimbal, float setpoint, float current,
                              float dt, uint8_t axis);

HAL_StatusTypeDef Gimbal_Init(GimbalController_t *gimbal,
                             TIM_HandleTypeDef *htim_servo,
                             TIM_HandleTypeDef *htim_motor,
                             I2C_HandleTypeDef *hi2c)
{
    // Inițializează structura
    gimbal->htim_servo = htim_servo;
    gimbal->htim_motor = htim_motor;
    gimbal->hi2c = hi2c;

    // Inițializează filtrele Kalman
    Kalman_Init(&gimbal->kalman_pitch);
    Kalman_Init(&gimbal->kalman_roll);

    // Setări PID implicite
    gimbal->kp = 2.0f;
    gimbal->ki = 0.1f;
    gimbal->kd = 0.05f;
    gimbal->integral_pitch = 0.0f;
    gimbal->integral_roll = 0.0f;
    gimbal->last_error_pitch = 0.0f;
    gimbal->last_error_roll = 0.0f;

    // Target-uri implicite (poziție centrală)
    gimbal->target_pitch = 0.0f;
    gimbal->target_roll = 0.0f;

    // Setări servo și motor
    gimbal->servo_center_us = SERVO_CENTER_PULSE_US;
    gimbal->servo_range_us = (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) / 2;
    gimbal->motor_max_speed = 0.8f;
    gimbal->max_angle = MAX_TILT_ANGLE;

    // Inițializează MPU6050
    if (MPU6050_Init(gimbal->hi2c) != HAL_OK) {
        return HAL_ERROR;
    }

    if (MPU6050_IsConnected(gimbal->hi2c) != HAL_OK) {
        return HAL_ERROR;
    }

    // Pornește PWM pentru servo-uri
    if (HAL_TIM_PWM_Start(gimbal->htim_servo, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(gimbal->htim_servo, TIM_CHANNEL_2) != HAL_OK) {
        return HAL_ERROR;
    }

    // Pornește PWM pentru motoare
    if (HAL_TIM_PWM_Start(gimbal->htim_motor, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(gimbal->htim_motor, TIM_CHANNEL_2) != HAL_OK) {
        return HAL_ERROR;
    }

    // Poziționează servo-urile în centru
    Gimbal_ControlServo(gimbal, 1, 0.0f); // Pitch
    Gimbal_ControlServo(gimbal, 2, 0.0f); // Roll

    // Oprește motoarele inițial
    Gimbal_ControlMotor(gimbal, 1, 0.0f);
    Gimbal_ControlMotor(gimbal, 2, 0.0f);

    // Citește valoarea inițială pentru Kalman
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
    // Calculează dt
    uint32_t current_time = HAL_GetTick();
    float dt = (float)(current_time - gimbal->last_update_time) / 1000.0f;
    gimbal->last_update_time = current_time;

    // Citește datele de la senzor
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;

    if (MPU6050_ReadAccel(gimbal->hi2c, &accel_x, &accel_y, &accel_z) != HAL_OK ||
        MPU6050_ReadGyro(gimbal->hi2c, &gyro_x, &gyro_y, &gyro_z) != HAL_OK) {
        return HAL_ERROR;
    }

    // Convertește giroscopul în grade/sec
    float gyro_pitch = (float)gyro_x / 131.0f;
    float gyro_roll = (float)gyro_y / 131.0f;

    // Calculează unghiurile din accelerometru
    float accel_pitch = atan2f((float)accel_y, (float)accel_z) * 180.0f / M_PI;
    float accel_roll = atan2f((float)-accel_x,
                             sqrtf((float)(accel_y * accel_y + accel_z * accel_z))) * 180.0f / M_PI;

    // Aplică filtrul Kalman
    gimbal->current_pitch = Kalman_GetAngle(&gimbal->kalman_pitch, accel_pitch, gyro_pitch, dt);
    gimbal->current_roll = Kalman_GetAngle(&gimbal->kalman_roll, accel_roll, gyro_roll, dt);

    // Calculează output-ul PID pentru servo-uri (stabilizare fină)
    float servo_pitch_output = Gimbal_PIDControl(gimbal, gimbal->target_pitch,
                                                gimbal->current_pitch, dt, 0);
    float servo_roll_output = Gimbal_PIDControl(gimbal, gimbal->target_roll,
                                               gimbal->current_roll, dt, 1);

    // Limitează unghiurile servo-urilor
    if (servo_pitch_output > gimbal->max_angle) servo_pitch_output = gimbal->max_angle;
    if (servo_pitch_output < -gimbal->max_angle) servo_pitch_output = -gimbal->max_angle;
    if (servo_roll_output > gimbal->max_angle) servo_roll_output = gimbal->max_angle;
    if (servo_roll_output < -gimbal->max_angle) servo_roll_output = -gimbal->max_angle;

    // Controlează servo-urile pentru stabilizarea fină
    Gimbal_ControlServo(gimbal, 1, servo_pitch_output);
    Gimbal_ControlServo(gimbal, 2, servo_roll_output);

    // Pentru mișcări mari, folosește și motoarele modificate
    float motor_pitch_speed = 0.0f;
    float motor_roll_speed = 0.0f;

    // Activează motoarele pentru corecții mari
    if (fabsf(gimbal->current_pitch - gimbal->target_pitch) > 10.0f) {
        motor_pitch_speed = (gimbal->current_pitch - gimbal->target_pitch) > 0 ? -0.3f : 0.3f;
    }
    if (fabsf(gimbal->current_roll - gimbal->target_roll) > 10.0f) {
        motor_roll_speed = (gimbal->current_roll - gimbal->target_roll) > 0 ? -0.3f : 0.3f;
    }

    // Controlează motoarele modificate
    Gimbal_ControlMotor(gimbal, 1, motor_pitch_speed);
    Gimbal_ControlMotor(gimbal, 2, motor_roll_speed);

    return HAL_OK;
}

static void Gimbal_ControlServo(GimbalController_t *gimbal, uint8_t channel, float angle)
{
    // Convertește unghiul în timp de puls (microsecunde)
    float pulse_us = gimbal->servo_center_us + (angle / gimbal->max_angle) * gimbal->servo_range_us;

    // Limitează valorile
    if (pulse_us < SERVO_MIN_PULSE_US) pulse_us = SERVO_MIN_PULSE_US;
    if (pulse_us > SERVO_MAX_PULSE_US) pulse_us = SERVO_MAX_PULSE_US;

    // Convertește în valoare PWM (ARR = 19999 pentru 20ms)
    uint32_t pwm_value = (uint32_t)((pulse_us / SERVO_PWM_PERIOD_US) * 20000);

    // Setează PWM
    if (channel == 1) {
        __HAL_TIM_SET_COMPARE(gimbal->htim_servo, TIM_CHANNEL_1, pwm_value);
    } else if (channel == 2) {
        __HAL_TIM_SET_COMPARE(gimbal->htim_servo, TIM_CHANNEL_2, pwm_value);
    }
}

static void Gimbal_ControlMotor(GimbalController_t *gimbal, uint8_t motor, float speed)
{
    // Limitează viteza
    if (speed > 1.0f) speed = 1.0f;
    if (speed < -1.0f) speed = -1.0f;

    uint32_t pwm_value = (uint32_t)(fabsf(speed) * MOTOR_PWM_MAX);

    if (motor == 1) { // Motor Pitch
        __HAL_TIM_SET_COMPARE(gimbal->htim_motor, TIM_CHANNEL_1, pwm_value);

        if (speed > 0.05f) {
            HAL_GPIO_WritePin(GPIOB, IN1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, IN2_Pin, GPIO_PIN_RESET);
        } else if (speed < -0.05f) {
            HAL_GPIO_WritePin(GPIOB, IN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, IN2_Pin, GPIO_PIN_SET);
        } else {
            // Frânare activă
            HAL_GPIO_WritePin(GPIOB, IN1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, IN2_Pin, GPIO_PIN_SET);
        }
    } else if (motor == 2) { // Motor Roll
        __HAL_TIM_SET_COMPARE(gimbal->htim_motor, TIM_CHANNEL_2, pwm_value);

        if (speed > 0.05f) {
            HAL_GPIO_WritePin(GPIOB, IN3_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, IN4_Pin, GPIO_PIN_RESET);
        } else if (speed < -0.05f) {
            HAL_GPIO_WritePin(GPIOB, IN3_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, IN4_Pin, GPIO_PIN_SET);
        } else {
            // Frânare activă
            HAL_GPIO_WritePin(GPIOB, IN3_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, IN4_Pin, GPIO_PIN_SET);
        }
    }
}

static float Gimbal_PIDControl(GimbalController_t *gimbal, float setpoint, float current,
                              float dt, uint8_t axis)
{
    float error = setpoint - current;

    float *integral = (axis == 0) ? &gimbal->integral_pitch : &gimbal->integral_roll;
    float *last_error = (axis == 0) ? &gimbal->last_error_pitch : &gimbal->last_error_roll;

    // Termen integral cu anti-windup
    *integral += error * dt;
    if (*integral > 10.0f) *integral = 10.0f;
    if (*integral < -10.0f) *integral = -10.0f;

    // Termen derivativ
    float derivative = (error - *last_error) / dt;

    // Calculează output-ul PID
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
    // Oprește toate motoarele
    Gimbal_ControlMotor(gimbal, 1, 0.0f);
    Gimbal_ControlMotor(gimbal, 2, 0.0f);

    // Centrează servo-urile
    Gimbal_ControlServo(gimbal, 1, 0.0f);
    Gimbal_ControlServo(gimbal, 2, 0.0f);
}
