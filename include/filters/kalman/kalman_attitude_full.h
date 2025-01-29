#ifndef KALMAN_ATTITUDE_FULL_H
#define KALMAN_ATTITUDE_FULL_H

#include "types/state_types.h"

typedef struct {
    double process_noise[10];      // Process noise for quaternion, gyro bias, heading
    double measurement_noise[6];   // Measurement noise for accelerometer, magnetometer
    double P[10][10];              // Covariance matrix
    double dt;                     // Time step
} KalmanFullAttitudeConfig;

typedef struct {
    double q[4];                   // Quaternion [w,x,y,z]
    double gyro_bias[3];           // Gyroscope bias [x,y,z]
    double heading;                // Heading (yaw)
    double P[10][10];              // State covariance matrix
    KalmanFullAttitudeConfig config;
    int initialized;
} KalmanFullAttitudeFilter;

void kalman_full_attitude_init(KalmanFullAttitudeFilter* kalman, double dt);
void kalman_full_attitude_predict(KalmanFullAttitudeFilter* kalman, const double gyro[3]);
void kalman_full_attitude_update(KalmanFullAttitudeFilter* kalman, const double gyro[3], const double accel[3], const double mag[3]);

#endif // KALMAN_ATTITUDE_FULL_H
