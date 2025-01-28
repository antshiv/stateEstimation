#ifndef MADGWICK_H
#define MADGWICK_H

typedef struct {
    double beta;      // Gain parameter
    double q[4];      // Quaternion [w, x, y, z]
} MadgwickFilter;

// Initialize the Madgwick filter
void madgwick_init(MadgwickFilter* filter, double beta);

// Update the Madgwick filter with IMU data (gyroscope + accelerometer)
void madgwick_update_imu(MadgwickFilter* filter, 
                         const double gyro[3], 
                         const double accel[3], 
                         double dt);

// Update the Madgwick filter with IMU + magnetometer data
void madgwick_update_imu_mag(MadgwickFilter* filter, 
                             const double gyro[3], 
                             const double accel[3], 
                             const double mag[3], 
                             double dt);

// Get the current orientation as a quaternion
void madgwick_get_orientation(const MadgwickFilter* filter, double q[4]);

#endif // MADGWICK_H
