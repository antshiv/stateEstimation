#include <stdio.h>
#include "estimators/attitude.h"

void test_initialization() {
    printf("\nTest 1: Initialization\n");
    AttitudeEstimator est;
    AttitudeEstConfig config = {
        .alpha = 0.98,
        .dt = 0.01
    };
    
    attitude_estimator_init(&est, &config);

    // Get quaternion as array
    double q[4];
    attitude_estimator_get_quaternion(&est, q);
    printf("Initial quaternion: w=%.3f, x=%.3f, y=%.3f, z=%.3f\n", 
           q[0], q[1], q[2], q[3]);

    // Get Euler angles
    EulerAngles euler;
    attitude_estimator_get_euler(&est, &euler);
    printf("Initial euler angles: roll=%.3f, pitch=%.3f, yaw=%.3f\n", 
           euler.roll, euler.pitch, euler.yaw);
}

void test_static_position() {
    printf("\nTest 2: Static Position\n");
    AttitudeEstimator est;
    AttitudeEstConfig config = {
        .alpha = 0.98,
        .dt = 0.01
    };
    attitude_estimator_init(&est, &config);

    // Simulate upright position
    double gyro[3] = {0.0, 0.0, 0.0};
    double accel[3] = {0.0, 0.0, 1.0};

    // Update several times
    for(int i = 0; i < 10; i++) {
        attitude_estimator_update(&est, gyro, accel);
    }

    // Get final orientation
    EulerAngles euler;
    attitude_estimator_get_euler(&est, &euler);
    printf("Static position euler angles: roll=%.3f, pitch=%.3f, yaw=%.3f\n", 
           euler.roll, euler.pitch, euler.yaw);
}

void test_rotation() {
    printf("\nTest 3: 90-degree Roll\n");
    AttitudeEstimator est;
    AttitudeEstConfig config = {
        .alpha = 0.9999,  // Give more weight to gyro for dynamic rotation
        .dt = 0.01
    };
    attitude_estimator_init(&est, &config);

    // Simulate 90-degree roll over 1 second
    for (int i = 0; i < 200; i++) {
        double gyro[3] = {1.57, 0.0, 0.0};  // Roll rate of pi/2 rad/s

        // Update accelerometer based on current angle
        double angle = (i * config.dt) * 1.57;  // Current angle in radians
        double ay = -sin(angle);
        double az = cos(angle);
        double accel[3] = {0.0, ay, az};

        attitude_estimator_update(&est, gyro, accel);

        if (i % 10 == 0) {
            EulerAngles euler;
            attitude_estimator_get_euler(&est, &euler);
            printf("Step %d - Roll: %.3f degrees\n", i, euler.roll * 57.3);  // Convert to degrees
        }
    }
}

int main() {
    printf("Starting Attitude Estimator Tests...\n");
    
    test_initialization();
    test_static_position();
    test_rotation();

    printf("\nTests completed.\n");
    return 0;
}
