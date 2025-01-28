#include "estimators/attitude_full.h"
#include "filters/kalman.h"
//#include "filters/ekf.h"
//#include "filters/ukf.h"
//#include "filters/madgwick.h"
//#include "filters/mahony.h"
#include "types/state_types.h"
#include <math.h>
#include <string.h>

void attitude_full_init(AttitudeFullEstimator* est, const AttitudeFullConfig* config) {
    if (!est || !config) return;
    memset(est, 0, sizeof(AttitudeFullEstimator));
    est->config = *config;
    est->q[0] = 1.0;  // Initialize quaternion to identity
    est->initialized = 0;
}

void attitude_full_update(AttitudeFullEstimator* est, const double gyro[3], const double accel[3], const double mag[3]) {
    if (!est || !gyro || !accel || !mag) return;

    if (!est->initialized) {
        // Initialize using accelerometer and magnetometer
        double accel_norm[3] = {accel[0], accel[1], accel[2]};
        normalize_vector(accel_norm);

        EulerAngles euler = {
            .roll = atan2(accel_norm[1], accel_norm[2]),
            .pitch = atan2(-accel_norm[0], sqrt(accel_norm[1] * accel_norm[1] + accel_norm[2] * accel_norm[2])),
            .yaw = atan2(mag[1], mag[0])  // Initial heading from magnetometer
        };
        euler_to_quaternion(&euler, est->q);
        est->heading = euler.yaw;
        est->initialized = 1;
        return;
    }

    // Update using the selected filter type
    switch (est->config.type) {
        case FULL_ESTIMATOR_KALMAN:
            kalman_filter_full_update(est, gyro, accel, mag);
            break;
        case FULL_ESTIMATOR_EKF:
            ekf_update(est, gyro, accel, mag);
            break;
        case FULL_ESTIMATOR_UKF:
            ukf_update(est, gyro, accel, mag);
            break;
        case FULL_ESTIMATOR_MADGWICK:
            madgwick_update_imu_mag(est, gyro, accel, mag, est->config.dt);
            break;
        case FULL_ESTIMATOR_MAHONY:
            mahony_update(est, gyro, accel, mag, est->config.dt);
            break;
        default:
            // Default to Kalman filter
            kalman_filter_update(est, gyro, accel, mag);
    }
}

void attitude_full_get_orientation(const AttitudeFullEstimator* est, double q[4], double* heading) {
    if (!est || !q || !heading) return;
    memcpy(q, est->q, 4 * sizeof(double));
    *heading = est->heading;
}
