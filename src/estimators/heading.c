#include "estimators/heading.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

// Normalize a 3D vector
static void normalize_vector(double v[3]) {
    double norm = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (norm > 1e-6) {
        v[0] /= norm;
        v[1] /= norm;
        v[2] /= norm;
    }
}

// Rotate a vector using a quaternion
static void quaternion_rotate_vector(const double q[4], const double v[3], double v_out[3]) {
    double t2 = q[0] * q[1];
    double t3 = q[0] * q[2];
    double t4 = q[0] * q[3];
    double t5 = -q[1] * q[1];
    double t6 = q[1] * q[2];
    double t7 = q[1] * q[3];
    double t8 = -q[2] * q[2];
    double t9 = q[2] * q[3];
    double t10 = -q[3] * q[3];

    v_out[0] = 2 * ((t8 + t10) * v[0] + (t6 - t4) * v[1] + (t3 + t7) * v[2]) + v[0];
    v_out[1] = 2 * ((t4 + t6) * v[0] + (t5 + t10) * v[1] + (t9 - t2) * v[2]) + v[1];
    v_out[2] = 2 * ((t7 - t3) * v[0] + (t2 + t9) * v[1] + (t5 + t8) * v[2]) + v[2];
}

// Compute heading from magnetometer readings
static double compute_heading(const double mag[3], const double q[4], double mag_declination) {
    double mag_earth[3];
    quaternion_rotate_vector(q, mag, mag_earth);
    return atan2(mag_earth[1], mag_earth[0]) + mag_declination;
}

// Initialize the heading estimator
void heading_estimator_init(HeadingEstimator* est, const HeadingEstConfig* config) {
    if (!est || !config) return;

    memset(est, 0, sizeof(HeadingEstimator));
    est->config = *config;

    // Copy initial covariance matrix
    memcpy(est->P, config->P, sizeof(est->P));

    est->initialized = 0;
}

void heading_estimator_update(HeadingEstimator* est, const double gyro_z, const double mag[3], const double q[4]) {
    if (!est) return;

    // 1. Initialization
    if (!est->initialized) {
        // Use magnetometer for initial heading
        est->heading = compute_heading(mag, q, est->config.mag_declination);
        est->last_valid_heading = est->heading;
        est->initialized = 1;
        return;
    }

    // 2. Prediction Step
    double dt = est->config.dt;

    // Update heading rate using gyroscope
    est->heading_rate = gyro_z;

    // Predict the new heading
    est->heading += est->heading_rate * dt;

    // Wrap heading to [-pi, pi]
    while (est->heading > M_PI) est->heading -= 2 * M_PI;
    while (est->heading < -M_PI) est->heading += 2 * M_PI;

    // Propagate covariance
    double P00 = est->P[0][0];
    double P01 = est->P[0][1];
    double P10 = est->P[1][0];
    double P11 = est->P[1][1];

    est->P[0][0] = P00 + dt * (P01 + P10 + dt * P11) + est->config.process_noise[0];
    est->P[0][1] = P01 + dt * P11 + est->config.process_noise[1];
    est->P[1][0] = P10 + dt * P11 + est->config.process_noise[1];
    est->P[1][1] = P11 + est->config.process_noise[1];

    // 3. Magnetometer Update
    double heading_measured = compute_heading(mag, q, est->config.mag_declination);

    // Calculate magnetometer disturbance
    double mag_norm = sqrt(mag[0] * mag[0] + mag[1] * mag[1] + mag[2] * mag[2]);
    est->mag_field_norm = mag_norm;

    if (fabs(mag_norm - 1.0) > est->config.mag_threshold) {
        // Magnetometer disturbance detected, skip update
        est->last_valid_heading = est->heading;
        return;
    }

    // Innovation (difference between measured and predicted heading)
    double innovation = heading_measured - est->heading;

    // Normalize innovation to [-pi, pi]
    while (innovation > M_PI) innovation -= 2 * M_PI;
    while (innovation < -M_PI) innovation += 2 * M_PI;

    // Innovation covariance
    double S = est->P[0][0] + est->config.measurement_noise;

    // Kalman gain
    double K[2];
    K[0] = est->P[0][0] / S;
    K[1] = est->P[1][0] / S;

    // Update state
    est->heading += K[0] * innovation;
    est->heading_rate += K[1] * innovation;

    // Update covariance
    est->P[0][0] -= K[0] * est->P[0][0];
    est->P[0][1] -= K[0] * est->P[0][1];
    est->P[1][0] -= K[1] * est->P[0][0];
    est->P[1][1] -= K[1] * est->P[0][1];

    // Update last valid heading
    est->last_valid_heading = est->heading;
}

// Get the current heading
double heading_estimator_get_heading(const HeadingEstimator* est) {
    if (!est) return 0.0;
    return est->heading;
}

// Get the current heading rate
double heading_estimator_get_rate(const HeadingEstimator* est) {
    if (!est) return 0.0;
    return est->heading_rate;
}

// Check if the heading is valid
int heading_estimator_is_valid(const HeadingEstimator* est) {
    if (!est) return 0;
    return fabs(est->mag_field_norm - 1.0) <= est->config.mag_threshold;
}
