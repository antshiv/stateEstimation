#ifndef KALMAN_ALTITUDE_H
#define KALMAN_ALTITUDE_H


// Kalman filter configuration for altitude
typedef struct {
    double process_noise[5];      // Process noise for [h, v, a, baro_bias, accel_bias_z]
    double measurement_noise[2];   // Measurement noise for [barometer, tof]
    double P[5][5];               // Initial covariance matrix
} AltitudeKalmanConfig;

#endif // KALMAN_ALTITUDE_H
