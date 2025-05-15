#include "kalman.h"

void Kalman_Init(Kalman_t *kalman) {
    /* Set default values */
    kalman->Q_angle = 0.001f;
    kalman->Q_bias = 0.003f;
    kalman->R_measure = 0.03f;

    kalman->angle = 0.0f; // Reset the angle
    kalman->bias = 0.0f;  // Reset bias

    kalman->P[0][0] = 0.0f;
    kalman->P[0][1] = 0.0f;
    kalman->P[1][0] = 0.0f;
    kalman->P[1][1] = 0.0f;
}

// The angle should be in degrees and the rate in degrees per second and dt in seconds
float Kalman_GetAngle(Kalman_t *kalman, float newAngle, float newRate, float dt) {
    // Discrete Kalman filter time update - "Predict"
    // Update xhat - Project the state ahead
    /* Step 1 */
    kalman->rate = newRate - kalman->bias;
    kalman->angle += dt * kalman->rate;

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    kalman->P[0][0] += dt * (dt * kalman->P[1][1] - kalman->P[0][1] -
                      kalman->P[1][0] + kalman->Q_angle);
    kalman->P[0][1] -= dt * kalman->P[1][1];
    kalman->P[1][0] -= dt * kalman->P[1][1];
    kalman->P[1][1] += kalman->Q_bias * dt;

    // Discrete Kalman filter measurement update - "Correct"
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
    float S = kalman->P[0][0] + kalman->R_measure; // Estimate error
    /* Step 5 */
    float K[2]; // Kalman gain
    K[0] = kalman->P[0][0] / S;
    K[1] = kalman->P[1][0] / S;

    // Calculate angle and bias - Update estimate with measurement
    /* Step 3 */
    float y = newAngle - kalman->angle; // Angle difference
    /* Step 6 */
    kalman->angle += K[0] * y;
    kalman->bias += K[1] * y;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    float P00_temp = kalman->P[0][0];
    float P01_temp = kalman->P[0][1];

    kalman->P[0][0] -= K[0] * P00_temp;
    kalman->P[0][1] -= K[0] * P01_temp;
    kalman->P[1][0] -= K[1] * P00_temp;
    kalman->P[1][1] -= K[1] * P01_temp;

    return kalman->angle;
}

void Kalman_SetAngle(Kalman_t *kalman, float angle) {
    kalman->angle = angle;
}

float Kalman_GetRate(Kalman_t *kalman) {
    return kalman->rate;
}

void Kalman_SetQangle(Kalman_t *kalman, float Q_angle) {
    kalman->Q_angle = Q_angle;
}

void Kalman_SetQbias(Kalman_t *kalman, float Q_bias) {
    kalman->Q_bias = Q_bias;
}

void Kalman_SetRmeasure(Kalman_t *kalman, float R_measure) {
    kalman->R_measure = R_measure;
}

float Kalman_GetQangle(Kalman_t *kalman) {
    return kalman->Q_angle;
}

float Kalman_GetQbias(Kalman_t *kalman) {
    return kalman->Q_bias;
}

float Kalman_GetRmeasure(Kalman_t *kalman) {
    return kalman->R_measure;
}
