#include "filters/kalman_full.h"
#include <math.h>
#include <string.h>

void kalman_full_init(FullKalmanFilter* filter, const FullKalmanConfig* config) {
    if (!filter || !config) return;
    memset(filter, 0, sizeof(FullKalmanFilter));
    filter->config = *config;
    filter->initialized = 0;
    // Initialize quaternion to identity
    filter->x[0] = 1.0;  // q_w
    filter->x[1] = filter->x[2] = filter->x[3] = 0.0;
    // Initialize covariance matrix
    memcpy(filter->P, config->P, sizeof(filter->P));
}

void kalman_filter_full_update(FullKalmanFilter* filter,
                        const double gyro[3],
                        const double accel[3],
                        const double mag[3]) {
    if (!filter) return;

    // Step 1: Predict step using gyro
    double dt = filter->config.dt;
    double gyro_unbiased[3] = {
        gyro[0] - filter->x[4],  // Remove gyro bias
        gyro[1] - filter->x[5],
        gyro[2] - filter->x[6]
    };

    // Quaternion prediction (simplified for clarity)
    double q_dot[4] = {
        -0.5 * (filter->x[1] * gyro_unbiased[0] + filter->x[2] * gyro_unbiased[1] + filter->x[3] * gyro_unbiased[2]),
         0.5 * (filter->x[0] * gyro_unbiased[0] - filter->x[3] * gyro_unbiased[1] + filter->x[2] * gyro_unbiased[2]),
         0.5 * (filter->x[3] * gyro_unbiased[0] + filter->x[0] * gyro_unbiased[1] - filter->x[1] * gyro_unbiased[2]),
        -0.5 * (filter->x[2] * gyro_unbiased[0] + filter->x[1] * gyro_unbiased[1] + filter->x[0] * gyro_unbiased[2])
    };

    for (int i = 0; i < 4; i++) {
        filter->x[i] += q_dot[i] * dt;
    }

    // Normalize quaternion
    double norm = sqrt(filter->x[0] * filter->x[0] + filter->x[1] * filter->x[1] +
                       filter->x[2] * filter->x[2] + filter->x[3] * filter->x[3]);
    for (int i = 0; i < 4; i++) filter->x[i] /= norm;

    // Step 2: Measurement update (e.g., using accelerometer and magnetometer)
    // Compute gravity direction and magnetic field direction in the body frame...
    // TODO: Implement full correction model

    // Update covariance matrix based on measurement noise and Kalman gain...
}

void kalman_full_get_orientation(const FullKalmanFilter* filter, double q[4], double* heading) {
    if (!filter || !q || !heading) return;
    memcpy(q, filter->x, 4 * sizeof(double));  // Quaternion
    *heading = filter->x[7];                  // Heading
}
