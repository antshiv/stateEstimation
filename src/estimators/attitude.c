#include "estimators/attitude.h"
#include <string.h>
#include <math.h>

// Forward declarations for internal functions
static void complementary_filter_update(AttitudeEstimator* est, 
                                      const double gyro_unbiased[3],
                                      const double accel_normalized[3],
                                      const double mag_normalized[3]);
                                      
static void kalman_filter_update(AttitudeEstimator* est,
                                const double gyro_unbiased[3],
                                const double accel_normalized[3],
                                const double mag_normalized[3]);

// Common utility functions
static void normalize_vector(double v[3]) {
    double norm = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    if(norm > 1e-6) {
        v[0] /= norm;
        v[1] /= norm;
        v[2] /= norm;
    }
}

static void remove_gyro_bias(const double gyro[3], 
                           const double bias[3], 
                           double unbiased[3]) {
    for(int i = 0; i < 3; i++) {
        unbiased[i] = gyro[i] - bias[i];
    }
}

void attitude_estimator_init(AttitudeEstimator* est, const AttitudeEstConfig* config) {
    if (!est || !config) return;
    
    est->config = *config;
    est->initialized = 0;
    est->timestamp = 0;
    
    // Initialize quaternion to identity
    est->q[0] = 1.0;
    est->q[1] = est->q[2] = est->q[3] = 0.0;
    
    // Initialize bias estimate to zero
    est->gyro_bias[0] = est->gyro_bias[1] = est->gyro_bias[2] = 0.0;
    
    // Initialize covariance matrix if using Kalman filter
    if (config->type == ESTIMATOR_KALMAN) {
        for (int i = 0; i < 7; i++) {
            for (int j = 0; j < 7; j++) {
                est->P[i][j] = (i == j) ? config->kalman.P[i][j] : 0.0;
            }
        }
    }
}

//void attitude_estimator_update_old(AttitudeEstimator* est, const double gyro[3], const double accel[3]) {
static void complementary_filter_update(AttitudeEstimator* est,
                                      const double gyro[3],
                                      const double accel[3],
                                      const double mag[3]) {
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

void attitude_estimator_update(AttitudeEstimator* est, 
                             const double gyro[3], 
                             const double accel[3],
                             const double mag[3]) {
    if (!est || !gyro || !accel) return;

    // Common initialization for all filters
    if (!est->initialized) {
        // Initialize using accelerometer for initial attitude
        double accel_norm[3] = {
            accel[0], accel[1], accel[2]
        };
        normalize_vector(accel_norm);
        
        EulerAngles euler = {
            .roll = atan2(accel_norm[1], accel_norm[2]),
            .pitch = atan2(-accel_norm[0],
                          sqrt(accel_norm[1] * accel_norm[1] + 
                               accel_norm[2] * accel_norm[2])),
            .yaw = 0.0
        };
        euler_to_quaternion(&euler, est->q);
        est->initialized = 1;
        return;
    }

    // Update based on selected filter type
    switch(est->config.type) {
        case ESTIMATOR_COMPLEMENTARY:
            complementary_filter_update(est, gyro, accel, mag);
            break;
            
        case ESTIMATOR_KALMAN:
            kalman_filter_update(est, gyro, accel, mag);
            break;
            
        default:
            // Fallback to complementary filter
            complementary_filter_update(est, gyro, accel, mag);
    }
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

static void kalman_filter_update(AttitudeEstimator* est,
                                const double gyro[3],
                                const double accel[3],
                                const double mag[3]) {
    double gyro_unbiased[3];
    remove_gyro_bias(gyro, est->gyro_bias, gyro_unbiased);
    double dt = est->config.dt;

    // 1. Predict Step
    // Predict quaternion using gyro integration
    double q_pred[4];  // Use temporary quaternion
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

    // Update predicted state
    for(int i = 0; i < 4; i++) {
        q_pred[i] = est->q[i] + q_dot[i] * dt;
    }
    quaternion_normalize(q_pred);
        
    // Update state
    memcpy(est->q, q_pred, 4 * sizeof(double));
    
    // 2. Kalman Update
    double accel_norm = sqrt(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
    if(accel_norm > 1e-6) {
        // Use temporary variables for measurement update
        double q_updated[4];
        for(int i = 0; i < 4; i++) {
            q_updated[i] = est->q[i];
        }

        // Normalized accelerometer
        double accel_normalized[3] = {
            accel[0]/accel_norm,
            accel[1]/accel_norm,
            accel[2]/accel_norm
        };

        // Predict gravity direction
        double g_body[3];
        quaternion_rotate_vector(est->q, (double[3]){0,0,1}, g_body);


        // Compute innovation
        double innovation[3];
        for(int i = 0; i < 3; i++) {
            innovation[i] = accel_normalized[i] - g_body[i];
        }

        // Adaptive Kalman gain based on acceleration magnitude
        double K = 0.1 * (1.0 - fabs(accel_norm - 9.81) / 9.81);
        K = K < 0.01 ? 0.01 : (K > 0.1 ? 0.1 : K);

        // Update quaternion
        for(int i = 0; i < 3; i++) {
            double correction = K * innovation[i];
            q_updated[0] -= 0.5 * correction * q_updated[i+1];
            q_updated[i+1] += 0.5 * correction * q_updated[0];
        }

        // Normalize and update state
        quaternion_normalize(est->q);

        // Update covariance with measured acceleration reliability
        double reliability = 1.0 - fabs(accel_norm - 9.81) / 9.81;
        for(int i = 0; i < 7; i++) {
            for(int j = 0; j < 7; j++) {
                est->P[i][j] *= (1.0 - K * reliability);
            }
        }
    }
}