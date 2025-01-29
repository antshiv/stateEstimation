#ifndef KALMAN_ATTITUDE_H
#define KALMAN_ATTITUDE_H

#include "types/state_types.h"

typedef struct {
    double process_noise[7];      // Process noise for [q0,q1,q2,q3,bias_x,bias_y,bias_z]
    double measurement_noise[3];   // Measurement noise for accelerometer [x,y,z]
    double P[7][7];               // Initial covariance matrix
} KalmanConfig;


#endif // KALMAN_ATTITUDE_H
