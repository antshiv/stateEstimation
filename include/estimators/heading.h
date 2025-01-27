#ifndef HEADING_ESTIMATOR_H
#define HEADING_ESTIMATOR_H

typedef struct {
    double process_noise[2];    // [heading, heading_rate]
    double measurement_noise;   // Magnetometer measurement noise
    double P[2][2];            // Initial covariance matrix
    double mag_declination;    // Local magnetic declination
    double mag_threshold;      // Disturbance detection threshold
    double dt;                 // Sample time
} HeadingEstConfig;

typedef struct {
    // State vector
    double heading;            // Current heading angle
    double heading_rate;       // Rate of change
    
    // Filter state
    double P[2][2];           // Covariance matrix
    double gyro_bias_z;       // Gyro bias estimate
    double mag_field_norm;    // For disturbance detection
    
    // Status
    double innovation;         // Last measurement innovation
    double last_valid_heading; // Backup for disturbances
    
    // Configuration
    HeadingEstConfig config;
    int initialized;
} HeadingEstimator;

// Core API
void heading_estimator_init(HeadingEstimator* est, const HeadingEstConfig* config);

// Main update function combining predict and update
void heading_estimator_update(HeadingEstimator* est,
                            const double gyro_z,     // Z-axis gyro
                            const double mag[3],     // Magnetometer readings
                            const double q[4]);      // Current attitude quaternion

// Getters
double heading_estimator_get_heading(const HeadingEstimator* est);
double heading_estimator_get_rate(const HeadingEstimator* est);
int heading_estimator_is_valid(const HeadingEstimator* est);

#endif // HEADING_ESTIMATOR_H