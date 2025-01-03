#ifndef UNIFIED_ESTIMATOR_H
#define UNIFIED_ESTIMATOR_H

#include "types/state_types.h"

// Full state vector indices
enum StateIndex {
    // Attitude quaternion (4)
    STATE_Q0 = 0,
    STATE_Q1 = 1,
    STATE_Q2 = 2,
    STATE_Q3 = 3,
    
    // Gyro bias (3)
    STATE_BIAS_GX = 4,
    STATE_BIAS_GY = 5,
    STATE_BIAS_GZ = 6,
    
    // Position (3)
    STATE_POS_X = 7,
    STATE_POS_Y = 8,
    STATE_POS_Z = 9,
    
    // Velocity (3)
    STATE_VEL_X = 10,
    STATE_VEL_Y = 11,
    STATE_VEL_Z = 12,
    
    // Accelerometer bias (3)
    STATE_BIAS_AX = 13,
    STATE_BIAS_AY = 14,
    STATE_BIAS_AZ = 15,
    
    // Barometer bias (1)
    STATE_BIAS_BARO = 16,
    
    NUM_STATES = 17
};

// Unified estimator configuration
typedef struct {
    // Process noise for each state
    double process_noise[NUM_STATES];
    
    // Measurement noise
    double noise_gyro[3];
    double noise_accel[3];
    double noise_baro;
    double noise_tof;
    double noise_gps[3];
    
    // Initial covariance
    double P[NUM_STATES][NUM_STATES];
    
    // Update rates
    double dt;                    // Primary update rate
    double baro_rate;            // Barometer update rate
    double gps_rate;             // GPS update rate
    
    // Reference values
    double ground_pressure;
    double ground_temperature;
    double home_position[3];
} UnifiedEstConfig;

// Unified estimator state
typedef struct {
    // Full state vector
    double x[NUM_STATES];
    
    // Covariance matrix
    double P[NUM_STATES][NUM_STATES];
    
    // Timing
    double timestamp;
    double last_baro_time;
    double last_gps_time;
    
    // Configuration
    UnifiedEstConfig config;
    
    // Status flags
    int initialized;
} UnifiedEstimator;

// Initialize unified estimator
void unified_estimator_init(UnifiedEstimator* est, 
                          const UnifiedEstConfig* config);

// Main update function - handles all sensors
void unified_estimator_update(UnifiedEstimator* est,
                            const SensorMeasurements* meas,
                            double timestamp);

// Get specific state components
void unified_estimator_get_attitude(const UnifiedEstimator* est, 
                                  double q[4]);

void unified_estimator_get_position(const UnifiedEstimator* est,
                                  double pos[3]);

void unified_estimator_get_velocity(const UnifiedEstimator* est,
                                  double vel[3]);

void unified_estimator_get_biases(const UnifiedEstimator* est,
                                double gyro_bias[3],
                                double accel_bias[3],
                                double* baro_bias);

// Reset functions
void unified_estimator_reset(UnifiedEstimator* est);
void unified_estimator_reset_position(UnifiedEstimator* est);
void unified_estimator_reset_velocity(UnifiedEstimator* est);

#endif // UNIFIED_ESTIMATOR_H