#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "estimators/attitude.h"

// Add these macros at the top
#define DEG2RAD(x) ((x) * M_PI / 180.0)
#define RAD2DEG(x) ((x) * 180.0 / M_PI)

// Add the pass_fail function
static int pass_fail(double value, double expected, double tolerance)
{
    return fabs(value - expected) <= tolerance;
}

void test_kalman_filter()
{
    printf("\n=== Testing Kalman Filter Attitude Estimator ===\n");

    // Initialize random seed for reproducible tests
    srand(42);

    // Initialize estimator with Kalman filter configuration
    AttitudeEstimator est;
    AttitudeEstConfig config = {
        .type = ESTIMATOR_KALMAN,
        .dt = 0.01,
        .kalman = {
            // Reduce process noise for quaternion components
            .process_noise = {1e-7, 1e-7, 1e-7, 1e-7, 1e-8, 1e-8, 1e-8},
            // Increase measurement noise since accelerometer is noisy
            .measurement_noise = {0.5, 0.5, 0.5},
            // Start with smaller initial covariance
            .P = {
                {0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                {0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0},
                {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01}}}};

    attitude_estimator_init(&est, &config);

    // Test cases
    struct TestCase
    {
        const char *name;
        double duration;
        double gyro[3];
        double final_angle;
        double tolerance;
    };

    struct TestCase tests[] = {
        {"90-degree Roll",
         2.0,                  // Increase duration to 2 seconds
         {M_PI / 4, 0.0, 0.0}, // Reduce rotation rate to 45 deg/s
         90.0,
         2.0},
        {
            "45-degree Pitch with Noise",
            0.5,                  // 0.5 second duration
            {0.0, M_PI / 2, 0.0}, // 90 deg/s pitch rate
            45.0,                 // Expected final angle
            2.0                   // Tolerance in degrees
        }};

    // Run each test case
    for (size_t t = 0; t < sizeof(tests) / sizeof(tests[0]); t++)
    {
        printf("\nTest Case: %s\n", tests[t].name);

        // Reset estimator
        attitude_estimator_init(&est, &config);

        int steps = (int)(tests[t].duration / config.dt);

        // Run simulation
        for (int i = 0; i < steps; i++)
        {
            // Calculate current angle for accelerometer reference
            double progress = (double)i / steps;
            double current_angle = progress * DEG2RAD(tests[t].final_angle);

            // Simulate accelerometer readings based on current angle
            double accel[3] = {0.0, 0.0, 1.0};
    
            double mag[3] = {1.0, 0.0, 0.0};

            if (strstr(tests[t].name, "Roll"))
            {
                // For roll motion
                accel[1] = -sin(current_angle);
                accel[2] = cos(current_angle);
            }
            else if (strstr(tests[t].name, "Pitch"))
            {
                // For pitch motion
                accel[0] = sin(current_angle);
                accel[2] = cos(current_angle);
            }

            // Add some noise to accelerometer (optional)
            for (int j = 0; j < 3; j++)
            {
                accel[j] += ((double)rand() / RAND_MAX - 0.5) * 0.1;
            }

            // Normalize accelerometer
            double acc_norm = sqrt(accel[0] * accel[0] +
                                   accel[1] * accel[1] +
                                   accel[2] * accel[2]);
            for (int j = 0; j < 3; j++)
            {
                accel[j] /= acc_norm;
            }

            // Update estimator
            attitude_estimator_update(&est, tests[t].gyro, accel, mag);

            // Print progress every 20%
            if (i % (steps / 5) == 0)
            {
                EulerAngles euler;
                attitude_estimator_get_euler(&est, &euler);
                printf("Progress %.0f%% - Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n",
                       100.0 * i / steps,
                       RAD2DEG(euler.roll),
                       RAD2DEG(euler.pitch),
                       RAD2DEG(euler.yaw));
            }
        }

        // Get final orientation
        EulerAngles euler_final;
        attitude_estimator_get_euler(&est, &euler_final);

        // Check which angle to validate based on test case
        double estimated_angle;
        if (strstr(tests[t].name, "Roll"))
        {
            estimated_angle = RAD2DEG(euler_final.roll);
        }
        else if (strstr(tests[t].name, "Pitch"))
        {
            estimated_angle = RAD2DEG(euler_final.pitch);
        }

        printf("\nFinal Results:\n");
        printf("Expected Angle: %.2f degrees\n", tests[t].final_angle);
        printf("Estimated Angle: %.2f degrees\n", estimated_angle);
        printf("Error: %.2f degrees\n", fabs(estimated_angle - tests[t].final_angle));
        printf("Result: %s\n",
               pass_fail(estimated_angle, tests[t].final_angle, tests[t].tolerance)
                   ? "PASS"
                   : "FAIL");

        // Print covariance diagonal for debugging
        printf("\nFinal State Uncertainties:\n");
        for (int i = 0; i < 7; i++)
        {
            printf("State %d: %.6f\n", i, sqrt(est.P[i][i]));
        }
    }
}

// Add to main:
int main()
{
    printf("Running Attitude Estimator Tests...\n");

    // test_attitude_estimator();  // Original complementary filter test
    test_kalman_filter(); // New Kalman filter test
    // run_tests();

    printf("\nTests completed.\n");
    return 0;
}