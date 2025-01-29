#ifndef ATTITUDE_FULL_H
#define ATTITUDE_FULL_H

#include "attitude/euler.h"
#include "attitude/quaternion.h"
#include "attitude/dcm.h"
#include "attitude/attitude_utils.h"
#include "types/state_types.h"
#include "filters/kalman/kalman_attitude_full.h"

// Estimation method enumeration for full attitude
typedef enum {
    FULL_ESTIMATOR_KALMAN,
    FULL_ESTIMATOR_EKF,
    FULL_ESTIMATOR_UKF,
    FULL_ESTIMATOR_MADGWICK,
    FULL_ESTIMATOR_MAHONY
} FullEstimatorType;

// Common full attitude state representation
typedef struct {
    double quaternion[4];   // Orientation (q_w, q_x, q_y, q_z)
    double gyro_bias[3];    // Gyroscope bias (b_x, b_y, b_z)
    double heading;         // Heading (yaw)
} FullAttitudeState;

typedef struct {
    double beta; // Gain parameter for Madgwick filter
} MadgwickConfig;

typedef struct {
    double kp; // Proportional gain for Mahony filter
    double ki; // Integral gain for Mahony filter
} MahonyConfig;

// Unified configuration structure using a union
typedef struct {
    FullEstimatorType type;    // Selected filter type
    double dt;                 // Time step
    union {
        KalmanFullAttitudeConfig kalman;
        MadgwickConfig madgwick;
        MahonyConfig mahony;
    };
} AttitudeFullConfig;

// Union to store different filters
typedef union {
    KalmanFullAttitudeFilter kalman;
//    EKFFullAttitudeFilter ekf;
//    UKFFullAttitudeFilter ukf;
//    MadgwickFilter madgwick;
//    MahonyFilter mahony;
} FullAttitudeFilterStorage;

// Updated Full Attitude Estimator
typedef struct {
    double q[4];               // Current quaternion [w, x, y, z]
    double gyro_bias[3];       // Gyroscope bias [x, y, z]
    double heading;            // Current heading (yaw)
    AttitudeFullConfig config; // Filter configuration
    FullAttitudeFilterStorage filter; // Stored filter instance
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
