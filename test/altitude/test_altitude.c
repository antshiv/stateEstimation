#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <stdlib.h>  // For rand() and RAND_MAX
#include "estimators/altitude.h"

// Test configuration structure
typedef struct {
    const char* name;          // Test name
    double duration;           // Test duration in seconds
    double init_altitude;      // Initial altitude
    double target_altitude;    // Target altitude
    double max_error;         // Maximum allowed error
} TestConfig;

// Helper function to check if value is within bounds
static int is_within_bounds(double value, double target, double tolerance) {
    return fabs(value - target) <= tolerance;
}

// Add this helper function for generating random noise
static double add_noise(double value, double noise_amplitude) {
    return value + ((double)rand() / RAND_MAX - 0.5) * 2.0 * noise_amplitude;
}

// Simple constant altitude test
static void test_constant_altitude(void) {
    printf("Running constant altitude test...\n");
    
    // Initialize estimator
    AltitudeEstConfig config;
    altitude_estimator_get_default_config(&config);
    
    AltitudeEstimator est;
    altitude_estimator_init(&est, &config);
    
    // Test parameters
    const double TEST_ALTITUDE = 2.0;  // 2 meters
    const double TEST_DURATION = 5.0;  // 5 seconds
    const double dt = 0.01;           // 100Hz
    const int num_steps = (int)(TEST_DURATION / dt);
    
    // Simulated sensor readings for hovering
    const double accel_z = 9.81;      // Gravity compensation
    const double baro_press = 98000;  // Some constant pressure
    const double baro_temp = 20.0;    // 20°C
    const double tof_dist = TEST_ALTITUDE;
    
    // Run test
    for(int i = 0; i < num_steps; i++) {
        altitude_estimator_update(&est, accel_z, baro_press, baro_temp, tof_dist);
        
        // Get estimates
        double altitude, velocity, accel;
        altitude_estimator_get_state(&est, &altitude, &velocity, &accel);
        
        // Check if within bounds
        assert(is_within_bounds(altitude, TEST_ALTITUDE, 0.1));
        assert(is_within_bounds(velocity, 0.0, 0.1));
        assert(is_within_bounds(accel, 0.0, 0.1));
    }
    
    printf("Constant altitude test passed!\n");
}

// Test step response
static void test_step_response(void) {
    printf("Running step response test...\n");
    
    AltitudeEstConfig config;
    altitude_estimator_get_default_config(&config);
    
    AltitudeEstimator est;
    altitude_estimator_init(&est, &config);
    
    // Test parameters
    const double INITIAL_ALTITUDE = 0.0;
    const double TARGET_ALTITUDE = 2.0;
    const double TEST_DURATION = 5.0;
    const double dt = 0.01;
    const int num_steps = (int)(TEST_DURATION / dt);
    
    // Response characteristics to check
    double settling_time = 0.0;
    double overshoot = 0.0;
    double steady_state_error = 0.0;
    
    for(int i = 0; i < num_steps; i++) {
        double time = i * dt;
        
        // Simulate accelerating upward then hovering
        double accel_z;
        if(time < 1.0) {
            accel_z = 9.81 + 2.0;  // Extra acceleration upward
        } else {
            accel_z = 9.81;        // Just hover
        }
        
        // Simulate sensors
        double baro_press = 98000.0 - (i < num_steps/2 ? 100.0 : 200.0); // Crude pressure change
        double baro_temp = 20.0;
        double tof_dist = TARGET_ALTITUDE;  // Assume we reached target
        
        altitude_estimator_update(&est, accel_z, baro_press, baro_temp, tof_dist);
        
        // Get estimates
        double altitude, velocity, accel;
        altitude_estimator_get_state(&est, &altitude, &velocity, &accel);
        
        // Check response characteristics
        if(time > 1.0) {  // After initial acceleration
            if(is_within_bounds(altitude, TARGET_ALTITUDE, 0.1) && settling_time == 0.0) {
                settling_time = time;
            }
            
            overshoot = fmax(overshoot, altitude - TARGET_ALTITUDE);
            
            if(time > 4.0) {  // Check steady state in last second
                steady_state_error = fabs(altitude - TARGET_ALTITUDE);
            }
        }
    }
    
    // Verify response characteristics
    assert(settling_time < 3.0);     // Should settle within 3 seconds
    assert(overshoot < 0.5);         // Overshoot should be less than 0.5m
    assert(steady_state_error < 0.1); // Steady state error should be less than 10cm
    
    printf("Step response test passed!\n");
}

// Test sensor failures/dropouts
static void test_sensor_robustness(void) {
    printf("Running sensor robustness test...\n");
    
    AltitudeEstConfig config;
    altitude_estimator_get_default_config(&config);
    
    AltitudeEstimator est;
    altitude_estimator_init(&est, &config);
    
    // Test with ToF dropout
    double altitude_before, altitude_after;
    altitude_estimator_get_state(&est, &altitude_before, NULL, NULL);
    
    // Update without ToF
    altitude_estimator_update(&est, 9.81, 98000.0, 20.0, -1.0);  // -1 indicates invalid ToF
    
    altitude_estimator_get_state(&est, &altitude_after, NULL, NULL);
    
    // Estimate shouldn't jump dramatically with one sensor missing
    assert(is_within_bounds(altitude_after, altitude_before, 0.1));
    
    printf("Sensor robustness test passed!\n");
}

// Test with noise
static void test_noise_handling(void) {
    printf("Running noise handling test...\n");
    
    AltitudeEstConfig config;
    altitude_estimator_get_default_config(&config);
    
    AltitudeEstimator est;
    altitude_estimator_init(&est, &config);
    
    const int NUM_SAMPLES = 1000;
    double sum_altitude = 0.0;
    double sum_squared_error = 0.0;
    const double TRUE_ALTITUDE = 2.0;
    
    for(int i = 0; i < NUM_SAMPLES; i++) {
        // Add noise to measurements more cleanly
        double noisy_accel = add_noise(9.81, 0.5);    // ±0.5 m/s^2 noise
        double noisy_baro = add_noise(98000.0, 50.0); // ±50 Pa noise
        double noisy_tof = add_noise(TRUE_ALTITUDE, 0.05); // ±5cm noise
        
        altitude_estimator_update(&est, noisy_accel, noisy_baro, 20.0, noisy_tof);
        
        double altitude;
        altitude_estimator_get_state(&est, &altitude, NULL, NULL);
        
        sum_altitude += altitude;
        sum_squared_error += pow(altitude - TRUE_ALTITUDE, 2);
    }
    
    // Check statistics
    double mean_altitude = sum_altitude / NUM_SAMPLES;
    double rmse = sqrt(sum_squared_error / NUM_SAMPLES);
    
    assert(is_within_bounds(mean_altitude, TRUE_ALTITUDE, 0.1));  // Check bias
    assert(rmse < 0.2);  // Check noise reduction
    
    printf("Noise handling test passed!\n");
}

// Main test function
void run_altitude_estimator_tests(void) {
    // Seed random number generator
    srand(time(NULL));  // Add #include <time.h> at the top
    
    printf("Starting altitude estimator tests...\n\n");
    
    test_constant_altitude();
    test_step_response();
    test_sensor_robustness();
    test_noise_handling();
    
    printf("\nAll altitude estimator tests passed!\n");
}

int main() {
    run_altitude_estimator_tests();
    return 0;
}