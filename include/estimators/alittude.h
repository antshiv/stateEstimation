#ifndef ALTITUDE_ESTIMATOR_H
#define ALTITUDE_ESTIMATOR_H

// Estimation method enumeration (similar to attitude estimator)
typedef enum {
    ALT_ESTIMATOR_COMPLEMENTARY,
    ALT_ESTIMATOR_KALMAN
} AltitudeEstimatorType;

// Kalman filter configuration for altitude
typedef struct {
    double process_noise[5];      // Process noise for [h, v, a, baro_bias, accel_bias_z]
    double measurement_noise[2];   // Measurement noise for [barometer, tof]
    double P[5][5];               // Initial covariance matrix
} AltitudeKalmanConfig;

// Altitude estimator configuration
typedef struct {
    AltitudeEstimatorType type;   
    double dt;                    // Sample time
    double tof_angle;            // ToF sensor mounting angle from vertical
    double ground_pressure;      // Reference pressure at ground level
    double ground_temperature;   // Reference temperature
    AltitudeKalmanConfig kalman;  
} AltitudeEstConfig;

// Altitude estimator state
typedef struct {
    // State vector
    double altitude;              // Current altitude
    double vertical_velocity;     // Vertical velocity
    double vertical_accel;        // Vertical acceleration
    double baro_bias;            // Barometer bias
    double accel_bias_z;         // Vertical accelerometer bias
    
    // Kalman filter state
    double P[5][5];              // State covariance matrix
    
    // Configuration and status
    AltitudeEstConfig config;
    double timestamp;
    int initialized;
} AltitudeEstimator;

// Initialize the altitude estimator
void altitude_estimator_init(AltitudeEstimator* est, const AltitudeEstConfig* config);

// Update with new measurements
void altitude_estimator_update(AltitudeEstimator* est,
                             const double accel_z,     // Vertical acceleration (body frame)
                             const double baro_press,  // Barometer pressure
                             const double baro_temp,   // Barometer temperature
                             const double tof_dist);   // ToF distance measurement

// Get current estimates
void altitude_estimator_get_state(const AltitudeEstimator* est,
                                double* altitude,
                                double* vertical_velocity,
                                double* vertical_accel);

// Get bias estimates
void altitude_estimator_get_biases(const AltitudeEstimator* est,
                                 double* baro_bias,
                                 double* accel_bias);

// Reset estimator
void altitude_estimator_reset(AltitudeEstimator* est);

// Set ground reference
void altitude_estimator_set_ground_reference(AltitudeEstimator* est,
                                          double ground_pressure,
                                          double ground_temperature);

#endif // ALTITUDE_ESTIMATOR_H