#include "filters/kalman_attitude_full.h"
#include <math.h>
#include <string.h>

// Convert generic attitude estimator config to Kalman-specific config
static void map_attitude_to_kalman(const AttitudeFullEstimator* est, KalmanFullAttitudeFilter* kalman) {
    if (!est || !kalman) return;
    memcpy(kalman->q, est->q, sizeof(est->q));
    memcpy(kalman->gyro_bias, est->gyro_bias, sizeof(est->gyro_bias));
    kalman->heading = est->heading;
    kalman->config = est->config.kalman; // Extract Kalman-specific config
}

// Convert Kalman filter output back to attitude estimator
static void map_kalman_to_attitude(const KalmanFullAttitudeFilter* kalman, AttitudeFullEstimator* est) {
    if (!est || !kalman) return;
    memcpy(est->q, kalman->q, sizeof(kalman->q));
    memcpy(est->gyro_bias, kalman->gyro_bias, sizeof(kalman->gyro_bias));
    est->heading = kalman->heading;
}

// Main Kalman filter update function
void kalman_full_attitude_update(KalmanFullAttitudeFilter* kalman, const double gyro[3], const double accel[3], const double mag[3]) {
    if (!kalman) return;

    // Kalman Prediction Step (gyro integration)
    double dt = kalman->config.dt;
    double q_dot[4] = {
        -0.5 * (kalman->q[1] * gyro[0] + kalman->q[2] * gyro[1] + kalman->q[3] * gyro[2]),
         0.5 * (kalman->q[0] * gyro[0] - kalman->q[3] * gyro[1] + kalman->q[2] * gyro[2]),
         0.5 * (kalman->q[3] * gyro[0] + kalman->q[0] * gyro[1] - kalman->q[1] * gyro[2]),
        -0.5 * (kalman->q[2] * gyro[0] + kalman->q[1] * gyro[1] + kalman->q[0] * gyro[2])
    };

    for (int i = 0; i < 4; i++) {
        kalman->q[i] += q_dot[i] * dt;
    }

    // Normalize quaternion
    double norm = sqrt(kalman->q[0] * kalman->q[0] + kalman->q[1] * kalman->q[1] +
                       kalman->q[2] * kalman->q[2] + kalman->q[3] * kalman->q[3]);
    for (int i = 0; i < 4; i++) kalman->q[i] /= norm;

    // Measurement Update: Magnetometer for Heading Correction
    kalman->heading = atan2(mag[1], mag[0]);
}
