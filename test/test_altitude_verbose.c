#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <stdbool.h>
#include "estimators/altitude.h"

// Structure to hold test statistics
typedef struct {
    double max_error;          // Maximum error seen
    double mean_error;         // Average error
    double std_dev;           // Standard deviation of error
    double settling_time;      // Time to reach stable value
    double max_overshoot;      // Maximum overshoot
    int num_samples;          // Number of samples collected
} TestStats;

// Helper function to print test header
static void print_test_header(const char* test_name) {
    printf("\n================================================\n");
    printf("Running %s\n", test_name);
    printf("================================================\n");
}

// Helper function to print test results
static void print_test_results(const char* test_name, const TestStats* stats, 
                             double allowed_error, double allowed_overshoot) {
    printf("\nResults for %s:\n", test_name);
    printf("  Maximum Error: %.3f m (Limit: ±%.3f m) %s\n", 
           stats->max_error, allowed_error,
           stats->max_error <= allowed_error ? "✓" : "✗");
    
    printf("  Mean Error: %.3f m\n", stats->mean_error);
    printf("  Standard Deviation: %.3f m\n", stats->std_dev);
    
    if (stats->settling_time > 0) {
        printf("  Settling Time: %.2f s\n", stats->settling_time);
    }
    
    if (stats->max_overshoot > 0) {
        printf("  Maximum Overshoot: %.2f%% (Limit: %.2f%%) %s\n",
               stats->max_overshoot * 100, allowed_overshoot * 100,
               stats->max_overshoot <= allowed_overshoot ? "✓" : "✗");
    }
    
    printf("  Samples Analyzed: %d\n", stats->num_samples);
}

static void test_constant_altitude(void) {
    print_test_header("Constant Altitude Test");
    printf("Purpose: Verify stable hover at 2.0m altitude\n");
    printf("Input Conditions:\n");
    printf("  - Accelerometer: 9.81 m/s² (gravity compensation)\n");
    printf("  - Barometer: 98000 Pa (constant pressure)\n");
    printf("  - ToF: 2.0m (constant reading)\n");
    
    AltitudeEstConfig config;
    altitude_estimator_get_default_config(&config);
    AltitudeEstimator est;
    altitude_estimator_init(&est, &config);
    
    TestStats stats = {0};
    double sum_error = 0;
    double sum_error_squared = 0;
    const double TARGET_ALT = 2.0;
    const double dt = 0.01;
    const int num_steps = 500;  // 5 seconds
    
    // Run test
    for(int i = 0; i < num_steps; i++) {
        altitude_estimator_update(&est, 9.81, 98000.0, 20.0, TARGET_ALT);
        
        double altitude, velocity, accel;
        altitude_estimator_get_state(&est, &altitude, &velocity, &accel);
        
        double error = fabs(altitude - TARGET_ALT);
        stats.max_error = fmax(stats.max_error, error);
        sum_error += error;
        sum_error_squared += error * error;
        
        // Track velocity and acceleration bounds
        if (fabs(velocity) > stats.max_error) {
            stats.max_error = fabs(velocity);
            printf("  Warning: Excessive velocity %.3f m/s at t=%.2fs\n", 
                   velocity, i*dt);
        }
    }
    
    stats.num_samples = num_steps;
    stats.mean_error = sum_error / num_steps;
    stats.std_dev = sqrt(sum_error_squared/num_steps - 
                        stats.mean_error*stats.mean_error);
    
    const double ALLOWED_ERROR = 0.1;  // 10cm
    print_test_results("Constant Altitude", &stats, ALLOWED_ERROR, 0.0);
    
    if (stats.max_error <= ALLOWED_ERROR) {
        printf("\nTest PASSED ✓\n");
    } else {
        printf("\nTest FAILED ✗\n");
        printf("Reason: Maximum error %.3fm exceeds limit of %.3fm\n",
               stats.max_error, ALLOWED_ERROR);
    }
}

static void test_step_response(void) {
    print_test_header("Step Response Test");
    printf("Purpose: Verify response to 2.0m altitude change\n");
    printf("Input Conditions:\n");
    printf("  - Initial altitude: 0.0m\n");
    printf("  - Target altitude: 2.0m\n");
    printf("  - Extra acceleration for first 1.0s\n");
    
    AltitudeEstConfig config;
    altitude_estimator_get_default_config(&config);
    AltitudeEstimator est;
    altitude_estimator_init(&est, &config);
    
    TestStats stats = {0};
    const double TARGET_ALT = 2.0;
    const double dt = 0.01;
    const int num_steps = 500;  // 5 seconds
    bool settled = false;
    
    // Run test
    for(int i = 0; i < num_steps; i++) {
        double time = i * dt;
        double accel_z = time < 1.0 ? 11.81 : 9.81;  // Extra acceleration initially
        
        altitude_estimator_update(&est, accel_z, 98000.0, 20.0, TARGET_ALT);
        
        double altitude, velocity, accel;
        altitude_estimator_get_state(&est, &altitude, &velocity, &accel);
        
        // Track settling time
        if (!settled && fabs(altitude - TARGET_ALT) < 0.1) {
            stats.settling_time = time;
            settled = true;
        }
        
        // Track overshoot
        double overshoot = (altitude - TARGET_ALT) / TARGET_ALT;
        stats.max_overshoot = fmax(stats.max_overshoot, overshoot);
        
        // Track final error
        if (time > 4.0) {  // Last second
            double error = fabs(altitude - TARGET_ALT);
            stats.max_error = fmax(stats.max_error, error);
        }
    }
    
    stats.num_samples = num_steps;
    
    const double ALLOWED_ERROR = 0.1;     // 10cm final error
    const double ALLOWED_OVERSHOOT = 0.25; // 25% overshoot
    const double MAX_SETTLING_TIME = 3.0;  // 3 seconds
    
    print_test_results("Step Response", &stats, ALLOWED_ERROR, ALLOWED_OVERSHOOT);
    
    bool passed = true;
    if (stats.max_error > ALLOWED_ERROR) {
        printf("FAILED: Final error %.3fm exceeds limit of %.3fm\n",
               stats.max_error, ALLOWED_ERROR);
        passed = false;
    }
    if (stats.max_overshoot > ALLOWED_OVERSHOOT) {
        printf("FAILED: Overshoot %.1f%% exceeds limit of %.1f%%\n",
               stats.max_overshoot*100, ALLOWED_OVERSHOOT*100);
        passed = false;
    }
    if (stats.settling_time > MAX_SETTLING_TIME) {
        printf("FAILED: Settling time %.2fs exceeds limit of %.2fs\n",
               stats.settling_time, MAX_SETTLING_TIME);
        passed = false;
    }
    
    if (passed) {
        printf("\nTest PASSED ✓\n");
    } else {
        printf("\nTest FAILED ✗\n");
    }
}

// Similar detailed implementations for other tests...

int main() {
    printf("Starting Altitude Estimator Tests\n");
    printf("=================================\n");
    printf("Test Configuration:\n");
    printf("- Sample rate: 100 Hz\n");
    printf("- Test duration: 5.0 seconds\n");
    printf("- Acceptance criteria based on typical drone requirements\n\n");
    
    test_constant_altitude();
    test_step_response();
    // Other tests...
    
    return 0;
}