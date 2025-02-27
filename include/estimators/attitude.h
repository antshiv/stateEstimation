#ifndef ATTITUDE_ESTIMATOR_H
#define ATTITUDE_ESTIMATOR_H

#include "attitude/euler.h"
#include "attitude/quaternion.h"
#include "attitude/dcm.h"
#include "attitude/attitude_utils.h"
#include "filters/kalman/kalman_attitude.h"

// Estimation method enumeration
typedef enum {
    ESTIMATOR_COMPLEMENTARY,
    ESTIMATOR_KALMAN
} EstimatorType;


// Attitude estimator configuration
typedef struct {
    EstimatorType type;           // Estimation method to use
    double alpha;                 // Complementary filter weight
    double dt;                    // Sample time
    KalmanConfig kalman;          // Kalman filter specific settings
} AttitudeEstConfig;

// Attitude estimator state
typedef struct {
    double q[4];                  // Current orientation quaternion [w,x,y,z]
    double gyro_bias[3];          // Estimated gyro bias [x,y,z]
    double omega[3];              // Angular velocity [x,y,z]
    double P[7][7];               // State covariance matrix for Kalman filter
    AttitudeEstConfig config;
    double timestamp;
    int initialized;
} AttitudeEstimator;

// Initialize the attitude estimator
void attitude_estimator_init(AttitudeEstimator* est, const AttitudeEstConfig* config);

// Update with new measurements
void attitude_estimator_update(AttitudeEstimator* est,
                             const double gyro[3],    // Gyro measurements [x,y,z]
                             const double accel[3],  // Accelerometer measurements [x,y,z]
                             const double mag[3]);

// Get current attitude in different representations
void attitude_estimator_get_quaternion(const AttitudeEstimator* est, double q[4]);
void attitude_estimator_get_euler(const AttitudeEstimator* est, EulerAngles* euler);
void attitude_estimator_get_dcm(const AttitudeEstimator* est, double dcm[3][3]);

// Get current angular velocity
void attitude_estimator_get_rates(const AttitudeEstimator* est, double omega[3]);

#endif // ATTITUDE_ESTIMATOR_H
