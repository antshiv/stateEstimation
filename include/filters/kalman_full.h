#ifndef KALMAN_FULL_H
#define KALMAN_FULL_H

#include "types/state_types.h"

typedef struct {
    double process_noise[10];    // Process noise for quaternion, gyro bias, heading
    double measurement_noise[6]; // Measurement noise for accelerometer, magnetometer
    double P[10][10];            // Covariance matrix
    double dt;                   // Time step
} FullKalmanConfig;

typedef struct {
    double x[10];                // State vector: [q_w, q_x, q_y, q_z, b_x, b_y, b_z, heading]
    double P[10][10];            // Covariance matrix
    FullKalmanConfig config;     // Kalman filter configuration
    int initialized;             // Initialization flag
} FullKalmanFilter;

// Initialize the full Kalman filter
void kalman_full_init(FullKalmanFilter* filter, const FullKalmanConfig* config);

// Update the full Kalman filter with gyroscope, accelerometer, and magnetometer
void kalman_filter_full_update(FullKalmanFilter* filter,
                        const double gyro[3],     // Gyroscope readings
                        const double accel[3],    // Accelerometer readings
                        const double mag[3]);     // Magnetometer readings

// Get the current attitude and heading
void kalman_full_get_orientation(const FullKalmanFilter* filter, double q[4], double* heading);

#endif // FULL_KALMAN_H
