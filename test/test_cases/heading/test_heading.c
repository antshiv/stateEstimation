#include <stdio.h>
#include <math.h>
#include "estimators/heading.h"
#include "test_helpers.h"

void test_heading_estimator()
{
    printf("=== Testing Heading Estimator ===\n");

    HeadingEstConfig config = {
        .process_noise = {0.01, 0.001}, // Process noise for heading and heading rate
        .measurement_noise = 0.05,      // Measurement noise for magnetometer
        .P = {{1.0, 0.0}, {0.0, 1.0}},  // Initial covariance matrix
        .mag_declination = 0.0,         // No magnetic declination for simplicity
        .mag_threshold = 0.2,           // Disturbance detection threshold
        .dt = 0.01                      // Sample time
    };

    HeadingEstimator estimator;
    heading_estimator_init(&estimator, &config);

    const double gyro_z = 0.05; // Simulated constant Z-axis gyro rate (rad/s)
    const double mag[3] = {0.707, 0.707, 0.0};
    const double q[4] = {1.0, 0.0, 0.0, 0.0};

    // Simulate gyro and magnetometer dynamics
    for (int i = 0; i < 100; i++)
    {
        double gyro_z = 0.05 + 0.01 * sin(0.1 * i); // Varying gyro rate
        double mag_dynamic[3] = {
            mag[0] * cos(0.01 * i) - mag[1] * sin(0.01 * i),
            mag[0] * sin(0.01 * i) + mag[1] * cos(0.01 * i),
            mag[2]};

        heading_estimator_update(&estimator, gyro_z, mag_dynamic, q);

        if (i % 10 == 0)
        {
            TEST_LOG("Step %d: Heading=%.3f, Rate=%.3f, Valid=%s\n",
                     i,
                     heading_estimator_get_heading(&estimator),
                     heading_estimator_get_rate(&estimator),
                     heading_estimator_is_valid(&estimator) ? "Yes" : "No");
        }
    }

    printf("\nFinal Heading: %.3f radians\n", heading_estimator_get_heading(&estimator));
    printf("Final Heading Rate: %.3f rad/s\n", heading_estimator_get_rate(&estimator));
    printf("Valid: %s\n", heading_estimator_is_valid(&estimator) ? "Yes" : "No");
}

int main()
{
    test_heading_estimator();
    return 0;
}
