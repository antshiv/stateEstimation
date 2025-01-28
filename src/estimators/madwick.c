#include "estimators/madgwick.h"
#include <math.h>
#include <string.h>

void madgwick_init(MadgwickFilter* filter, double beta) {
    if (!filter) return;
    filter->beta = beta;
    filter->q[0] = 1.0;  // Initialize quaternion to identity
    filter->q[1] = filter->q[2] = filter->q[3] = 0.0;
}

void madgwick_update_imu(MadgwickFilter* filter, 
                         const double gyro[3], 
                         const double accel[3], 
                         double dt) {
    if (!filter || !gyro || !accel) return;

    // Normalize accelerometer
    double norm = sqrt(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
    if (norm < 1e-6) return;  // Invalid accelerometer data
    double ax = accel[0] / norm;
    double ay = accel[1] / norm;
    double az = accel[2] / norm;

    // Gyroscope rates (converted to radians/sec)
    double gx = gyro[0];
    double gy = gyro[1];
    double gz = gyro[2];

    // Shortcuts for quaternion
    double q1 = filter->q[0], q2 = filter->q[1], q3 = filter->q[2], q4 = filter->q[3];

    // Gradient descent algorithm to adjust quaternion
    double s1 = -2.0 * (q3 * (2.0 * q2 * q4 - 2.0 * q1 * q3 - ax) + q4 * (2.0 * q1 * q2 + 2.0 * q3 * q4 - ay));
    double s2 = 2.0 * (q2 * (2.0 * q2 * q4 - 2.0 * q1 * q3 - ax) - q4 * (2.0 * q1 * q2 + 2.0 * q3 * q4 - ay));
    double s3 = 2.0 * (q1 * (2.0 * q2 * q4 - 2.0 * q1 * q3 - ax) + q3 * (2.0 * q1 * q2 + 2.0 * q3 * q4 - ay));
    double s4 = 2.0 * (q2 * (2.0 * q2 * q4 - 2.0 * q1 * q3 - ax) - q4 * (2.0 * q1 * q2 + 2.0 * q3 * q4 - ay));
    norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);  // Normalize step magnitude
    s1 /= norm; s2 /= norm; s3 /= norm; s4 /= norm;

    // Adjust quaternion rates
    double q_dot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - filter->beta * s1;
    double q_dot2 = 0.5 * ( q1 * gx + q3 * gz - q4 * gy) - filter->beta * s2;
    double q_dot3 = 0.5 * ( q1 * gy - q2 * gz + q4 * gx) - filter->beta * s3;
    double q_dot4 = 0.5 * ( q1 * gz + q2 * gy - q3 * gx) - filter->beta * s4;

    // Integrate to update quaternion
    filter->q[0] += q_dot1 * dt;
    filter->q[1] += q_dot2 * dt;
    filter->q[2] += q_dot3 * dt;
    filter->q[3] += q_dot4 * dt;

    // Normalize quaternion
    norm = sqrt(filter->q[0] * filter->q[0] + filter->q[1] * filter->q[1] +
                filter->q[2] * filter->q[2] + filter->q[3] * filter->q[3]);
    filter->q[0] /= norm;
    filter->q[1] /= norm;
    filter->q[2] /= norm;
    filter->q[3] /= norm;
}

void madgwick_get_orientation(const MadgwickFilter* filter, double q[4]) {
    if (!filter || !q) return;
    memcpy(q, filter->q, 4 * sizeof(double));
}
