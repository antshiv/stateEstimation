#include "estimators/attitude.h"
#include <string.h>
#include <math.h>

void attitude_estimator_init(AttitudeEstimator* est, const AttitudeEstConfig* config) {
    if (!est || !config) return;

    // Initialize quaternion to identity [w,x,y,z]
    est->q[0] = 1.0;
    est->q[1] = 0.0;
    est->q[2] = 0.0;
    est->q[3] = 0.0;

    // Clear gyro bias and angular velocity
    memset(est->gyro_bias, 0, sizeof(est->gyro_bias));
    memset(est->omega, 0, sizeof(est->omega));

    // Copy configuration
    est->config = *config;
    est->initialized = 0;
}

void attitude_estimator_update(AttitudeEstimator* est, const double gyro[3], const double accel[3]) {
    if (!est || !gyro || !accel) return;

    // Remove estimated bias from gyro
    double gyro_unbiased[3];
    for (int i = 0; i < 3; i++) {
        gyro_unbiased[i] = gyro[i] - est->gyro_bias[i];
    }

    // Normalize accelerometer with safety check
    double accel_norm = sqrt(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
    if (accel_norm < 1e-6) {
        // Acceleration too low to be reliable
        return;
    }
    
    double accel_normalized[3] = {
        accel[0] / accel_norm,
        accel[1] / accel_norm,
        accel[2] / accel_norm
    };

    // Initialization
    if (!est->initialized) {
        EulerAngles euler = {
            .roll = atan2(accel_normalized[1], accel_normalized[2]),
            .pitch = atan2(-accel_normalized[0],
                          sqrt(accel_normalized[1] * accel_normalized[1] + 
                               accel_normalized[2] * accel_normalized[2])),
            .yaw = 0.0
        };
        euler_to_quaternion(&euler, est->q);
        est->initialized = 1;
        return;
    }

    // Predict step (gyro integration)
    double dt = est->config.dt;
    double q_dot[4] = {
        -0.5 * (est->q[1] * gyro_unbiased[0] + est->q[2] * gyro_unbiased[1] + 
                est->q[3] * gyro_unbiased[2]),
         0.5 * (est->q[0] * gyro_unbiased[0] + est->q[2] * gyro_unbiased[2] - 
                est->q[3] * gyro_unbiased[1]),
         0.5 * (est->q[0] * gyro_unbiased[1] - est->q[1] * gyro_unbiased[2] + 
                est->q[3] * gyro_unbiased[0]),
         0.5 * (est->q[0] * gyro_unbiased[2] + est->q[1] * gyro_unbiased[1] - 
                est->q[2] * gyro_unbiased[0])
    };

    // Update quaternion
    for (int i = 0; i < 4; i++) {
        est->q[i] += q_dot[i] * dt;
    }
    quaternion_normalize(est->q);

    // Check for rapid motion
    double gyro_magnitude = sqrt(gyro_unbiased[0] * gyro_unbiased[0] + 
                               gyro_unbiased[1] * gyro_unbiased[1] + 
                               gyro_unbiased[2] * gyro_unbiased[2]);
    double threshold = 1.0;  // rad/s
    
    if (gyro_magnitude <= threshold) {
        // Only correct using accelerometer when motion is slow enough
        
        // Calculate gravity direction from current attitude
        double g_body[3];
        double q_inv[4];
        quaternion_inverse(est->q, q_inv);
        
        // Rotate gravity vector to body frame
        double g_ned[3] = {0, 0, 1};  // Gravity in NED frame
        // TODO: Implement quaternion_rotate_vector if not available in AML
        // quaternion_rotate_vector(q_inv, g_ned, g_body);
        
        // Calculate correction quaternion
        double alpha = est->config.alpha;
        double correction[3];
        correction[0] = (1.0 - alpha) * (g_body[0] - accel_normalized[0]);
        correction[1] = (1.0 - alpha) * (g_body[1] - accel_normalized[1]);
        correction[2] = (1.0 - alpha) * (g_body[2] - accel_normalized[2]);
        
        // Apply correction
        est->q[0] -= 0.5 * (correction[0] * est->q[1] + correction[1] * est->q[2] + 
                           correction[2] * est->q[3]);
        est->q[1] += 0.5 * (correction[0] * est->q[0] + correction[2] * est->q[2] - 
                           correction[1] * est->q[3]);
        est->q[2] += 0.5 * (correction[1] * est->q[0] - correction[2] * est->q[1] + 
                           correction[0] * est->q[3]);
        est->q[3] += 0.5 * (correction[2] * est->q[0] + correction[1] * est->q[1] - 
                           correction[0] * est->q[2]);
        
        quaternion_normalize(est->q);
    }

    #ifdef DEBUG
    EulerAngles euler_debug;
    quaternion_to_euler(est->q, &euler_debug.roll, &euler_debug.pitch, &euler_debug.yaw);
    printf("Quaternion: [%.3f, %.3f, %.3f, %.3f]\n", 
           est->q[0], est->q[1], est->q[2], est->q[3]);
    printf("Euler (rad): Roll=%.3f, Pitch=%.3f, Yaw=%.3f\n", 
           euler_debug.roll, euler_debug.pitch, euler_debug.yaw);
    #endif
}

void attitude_estimator_get_quaternion(const AttitudeEstimator* est, double q[4]) {
    if (!est || !q) return;
    memcpy(q, est->q, sizeof(double) * 4);
}

void attitude_estimator_get_euler(const AttitudeEstimator* est, EulerAngles* euler) {
    if (!est || !euler) return;
    // Call the attitude math library function with individual angle pointers
    quaternion_to_euler(est->q, &euler->roll, &euler->pitch, &euler->yaw);
}

void attitude_estimator_get_dcm(const AttitudeEstimator* est, double dcm[3][3]) {
    if (!est || !dcm) return;
    quaternion_to_dcm(est->q, dcm);
}

void attitude_estimator_get_rates(const AttitudeEstimator* est, double omega[3]) {
    if (!est || !omega) return;
    memcpy(omega, est->omega, sizeof(est->omega));
}
