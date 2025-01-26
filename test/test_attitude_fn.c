#include <stdio.h>
#include <math.h>
#include "estimators/attitude.h"

// Macros for degree-radian conversion
#define DEG2RAD(x) ((x) * M_PI / 180.0)
#define RAD2DEG(x) ((x) * 180.0 / M_PI)

// Function to check pass/fail with a given tolerance
int pass_fail(double value, double expected, double tolerance) {
    return fabs(value - expected) <= tolerance;
}

// Generalized test case for attitude estimation
void test_case(const char* name,
               const double gyro[3], 
               const double accel[3],
               const double mag[3],
               double duration,
               double dt,
               const EulerAngles* expected,
               double tolerance) {
    printf("\nTest: %s\n", name);

    // Initialize the estimator
    AttitudeEstimator est;
    AttitudeEstConfig config = {
        .alpha = 0.995,  // High trust in gyro
        .dt = dt
    };
    attitude_estimator_init(&est, &config);

    int steps = (int)(duration / dt);

    // Simulate the input data
    for (int i = 0; i < steps; i++) {
        // Calculate expected pitch angle at this step
        double progress = (double)i / steps;
        double current_angle = progress * DEG2RAD(expected->pitch);
        
        // Update accelerometer reference based on current pitch
        double acc_x = sin(current_angle);
        double acc_z = cos(current_angle);
        double current_accel[3] = {acc_x, 0.0, acc_z};
        
        // Normalize accelerometer readings
        double acc_norm = sqrt(acc_x*acc_x + acc_z*acc_z);
        if (acc_norm > 0.0) {
            current_accel[0] /= acc_norm;
            current_accel[2] /= acc_norm;
        }

        attitude_estimator_update(&est, gyro, current_accel, mag);

        // Print progress every 10%
        if (i % (steps / 10) == 0) {
            EulerAngles euler;
            attitude_estimator_get_euler(&est, &euler);
            printf("Step %d - Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n",
                   i, RAD2DEG(euler.roll), RAD2DEG(euler.pitch), RAD2DEG(euler.yaw));
        }
    }

    // Get final orientation
    EulerAngles euler_final;
    attitude_estimator_get_euler(&est, &euler_final);

    printf("Final Orientation - Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n",
           RAD2DEG(euler_final.roll), RAD2DEG(euler_final.pitch), RAD2DEG(euler_final.yaw));
    printf("Expected - Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n",
           expected->roll, expected->pitch, expected->yaw);

    // Validate results
    int pass = pass_fail(RAD2DEG(euler_final.roll), expected->roll, tolerance) &&
               pass_fail(RAD2DEG(euler_final.pitch), expected->pitch, tolerance) &&
               pass_fail(RAD2DEG(euler_final.yaw), expected->yaw, tolerance);

    printf("Result: %s\n", pass ? "PASS" : "FAIL");
}

// Run tests with corrected pitch motion
void run_tests() {
    double dt = 0.01;        // Time step
    double duration = 1.0;    // Duration in seconds

    // Test pitch rotation
    double pitch_gyro[3] = {0.0, DEG2RAD(90.0), 0.0};  // 90 deg/s pitch rate
    double pitch_accel[3] = {0.0, 0.0, 1.0};           // Initial upright position
    double pitch_mag[3] = {1.0, 0.0, 0.0};
    EulerAngles pitch_expected = {0.0, 90.0, 0.0};     // 90-degree pitch
    
    test_case("90-degree Pitch", pitch_gyro, pitch_accel, pitch_mag, duration, dt, 
              &pitch_expected, 2.0);  // 2-degree tolerance
}

// Single test for 90-degree roll
void test_attitude_estimator() {
    printf("\nTest: 90-Degree Roll\n");

    // Test configuration
    double dt = 0.01; // Time step
    double duration = 1.0; // Total simulation time
    int steps = (int)(duration / dt);

    // Initialize estimator
    AttitudeEstimator est;
    AttitudeEstConfig config = {
        .alpha = 0.995,
        .dt = dt
    };
    attitude_estimator_init(&est, &config);

    // Simulate constant roll rate (90 degrees/second)
    double gyro[3] = {M_PI / 2, 0.0, 0.0}; // pi/2 rad/s roll
    double accel[3] = {0.0, 0.0, 1.0};     // Initially upright
    double mag[3] = {1.0, 0.0, 0.0};

    for (int i = 0; i < steps; i++) {
        double angle = (i * dt) * M_PI / 2; // Accumulated roll angle in radians
        accel[1] = -sin(angle);
        accel[2] = cos(angle);

        attitude_estimator_update(&est, gyro, accel, mag);
    }

    // Validate final orientation
    EulerAngles euler;
    attitude_estimator_get_euler(&est, &euler);
    double roll_deg = RAD2DEG(euler.roll);

    printf("Expected Roll: 90.0 degrees\n");
    printf("Estimated Roll: %.2f degrees\n", roll_deg);

    double tolerance = 1.0; // Allow ±1 degree error
    printf("Result: %s\n", pass_fail(roll_deg, 90.0, tolerance) ? "PASS" : "FAIL");
}

// Run multiple test cases

int main() {
    printf("Running Attitude Estimator Tests...\n");

    test_attitude_estimator();
    run_tests();

    printf("Tests completed.\n");
    return 0;
}

