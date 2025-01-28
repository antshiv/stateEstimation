#include <stdio.h>
#include "estimators/heading.h"
#include "test_helpers.h"

void test_heading_with_tilts() {
    TEST_LOG("=== Testing Heading Estimator with Various Tilts ===\n");

    HeadingEstConfig config = {
        .process_noise = {0.01, 0.001},      // Process noise for heading and heading rate
        .measurement_noise = 0.05,          // Measurement noise for magnetometer
        .P = {{1.0, 0.0}, {0.0, 1.0}},      // Initial covariance matrix
        .mag_declination = 0.0,             // No magnetic declination for simplicity
        .mag_threshold = 0.2,               // Disturbance detection threshold
        .dt = 0.01                          // Sample time
    };

    HeadingEstimator estimator;
    heading_estimator_init(&estimator, &config);

    // Simulated test data
    const double gyro_z = 0.05; // Simulated Z-axis gyro rate (rad/s)
    const double mag[3] = {0.707, 0.707, 0.0}; // Simulated magnetometer reading (normalized)

    // Predefined quaternions for various tilts
    const double test_quaternions[][4] = {
        {1.0, 0.0, 0.0, 0.0},       // No tilt
        {0.965, 0.258, 0.0, 0.0},   // 30° Roll
        {0.866, 0.0, 0.5, 0.0},     // 60° Pitch
        {0.707, 0.5, 0.5, 0.0},     // 45° Roll, 45° Pitch
        {0.5, 0.707, 0.5, 0.0},     // 90° Roll, 45° Pitch
    };

    // Iterate through the predefined quaternions
    for (int i = 0; i < 5; i++) {
        const double *q = test_quaternions[i];
        TEST_LOG("Testing with Quaternion: [%.3f, %.3f, %.3f, %.3f]\n", q[0], q[1], q[2], q[3]);

        // Reset the estimator for each test quaternion
        heading_estimator_init(&estimator, &config);

        // Simulate updates over 100 steps
        for (int step = 0; step < 100; step++) {
            heading_estimator_update(&estimator, gyro_z, mag, q);

            if (step % 10 == 0) {
                double heading = heading_estimator_get_heading(&estimator);
                double rate = heading_estimator_get_rate(&estimator);
                int valid = heading_estimator_is_valid(&estimator);

                TEST_LOG("Step %d: Heading=%.3f, Rate=%.3f, Valid=%s\n",
                         step, heading, rate, valid ? "Yes" : "No");
            }
        }

        // Final validation
        TEST_LOG("Final Heading: %.3f radians\n", heading_estimator_get_heading(&estimator));
        TEST_LOG("Final Rate: %.3f rad/s\n", heading_estimator_get_rate(&estimator));
        TEST_LOG("Valid: %s\n\n", heading_estimator_is_valid(&estimator) ? "Yes" : "No");
    }
}

int main() {
    test_heading_with_tilts();
    return 0;
}
