#ifndef KALMAN_H
#define KALMAN_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /* Kalman filter variables */
    float Q_angle;   // Process noise variance for the accelerometer
    float Q_bias;    // Process noise variance for the gyro bias
    float R_measure; // Measurement noise variance

    float angle;     // The angle calculated by the Kalman filter - part of the 2x1 state vector
    float bias;      // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
    float rate;      // Unbiased rate calculated from the rate and the calculated bias

    float P[2][2];   // Error covariance matrix - This is a 2x2 matrix
} Kalman_t;

// Initialize Kalman filter
void Kalman_Init(Kalman_t *kalman);

// Calculate the angle using a Kalman filter
float Kalman_GetAngle(Kalman_t *kalman, float newAngle, float newRate, float dt);

// Set the angle
void Kalman_SetAngle(Kalman_t *kalman, float angle);

// Return the unbiased rate
float Kalman_GetRate(Kalman_t *kalman);

// Set parameters
void Kalman_SetQangle(Kalman_t *kalman, float Q_angle);
void Kalman_SetQbias(Kalman_t *kalman, float Q_bias);
void Kalman_SetRmeasure(Kalman_t *kalman, float R_measure);

// Get parameters
float Kalman_GetQangle(Kalman_t *kalman);
float Kalman_GetQbias(Kalman_t *kalman);
float Kalman_GetRmeasure(Kalman_t *kalman);

#ifdef __cplusplus
}
#endif

#endif // KALMAN_H
