#ifndef ATTITUDE_H
#define ATTITUDE_H

#include "types/state_types.h"
#include <stdbool.h>

// Attitude estimator state
typedef struct {
    Quaternion q;          // Current attitude quaternion
    Vector3d gyro_bias;    // Gyroscope bias estimate
    double dt;             // Sample time
    double alpha;          // Complementary filter weight
    bool initialized;      // Initialization flag
} AttitudeEstimator;

// Initialize the attitude estimator
void attitude_init(AttitudeEstimator* est, double alpha, double dt);

// Update using gyroscope data (prediction step)
void attitude_predict(AttitudeEstimator* est, 
                     double gx, double gy, double gz);

// Update using accelerometer data (correction step)
void attitude_correct(AttitudeEstimator* est,
                     double ax, double ay, double az);

// Get current attitude estimates
void attitude_get_quaternion(const AttitudeEstimator* est, Quaternion* q);
void attitude_get_euler(const AttitudeEstimator* est, EulerAngles* euler);

#endif // ATTITUDE_H
