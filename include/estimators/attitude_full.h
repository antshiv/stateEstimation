#ifndef ATTITUDE_FULL_H
#define ATTITUDE_FULL_H

#include "types/state_types.h"

// Estimation method enumeration for full attitude
typedef enum {
    FULL_ESTIMATOR_KALMAN,
    FULL_ESTIMATOR_EKF,
    FULL_ESTIMATOR_UKF,
    FULL_ESTIMATOR_MADGWICK,
    FULL_ESTIMATOR_MAHONY
} FullEstimatorType;

// Configuration for full attitude estimation
typedef struct {
    FullEstimatorType type;    // Filter type
    double dt;                 // Time step
    double process_noise[10];  // Process noise for Kalman/EKF/UKF
    double measurement_noise[6]; // Measurement noise for accel/magnetometer
} AttitudeFullConfig;

// Full attitude estimator state
typedef struct {
    double q[4];               // Current quaternion [w, x, y, z]
    double gyro_bias[3];       // Gyroscope bias [x, y, z]
    double heading;            // Current heading (yaw)
    AttitudeFullConfig config; // Filter configuration
    int initialized;           // Initialization flag
} AttitudeFullEstimator;

// Initialize the full attitude estimator
void attitude_full_init(AttitudeFullEstimator* est, const AttitudeFullConfig* config);

// Update the full attitude estimator with sensor data
void attitude_full_update(AttitudeFullEstimator* est,
                          const double gyro[3],     // Gyroscope data
                          const double accel[3],    // Accelerometer data
                          const double mag[3]);     // Magnetometer data

// Get the current quaternion and heading
void attitude_full_get_orientation(const AttitudeFullEstimator* est, double q[4], double* heading);

#endif // ATTITUDE_FULL_H
