import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R

def generate_imu_data(duration=10, sample_rate=100):
    """Generate synthetic IMU data for a simple rotation sequence.
    
    Args:
        duration: Time in seconds
        sample_rate: Samples per second
        
    Returns:
        DataFrame with timestamps and IMU readings
    """
    # Time vector
    t = np.linspace(0, duration, int(duration * sample_rate))
    
    # Generate true angular positions (in radians)
    # Simulate a rotation sequence: roll->pitch->yaw
    true_roll = 30 * np.pi/180 * np.sin(2*np.pi*0.5*t)  # oscillating roll
    true_pitch = 20 * np.pi/180 * np.sin(2*np.pi*0.3*t)  # oscillating pitch
    true_yaw = 45 * np.pi/180 * np.sin(2*np.pi*0.2*t)    # oscillating yaw
    
    # Generate true angular velocities (rad/s)
    gyro_x = np.gradient(true_roll, t)
    gyro_y = np.gradient(true_pitch, t)
    gyro_z = np.gradient(true_yaw, t)
    
    # Add noise and bias to gyro
    gyro_noise_std = 0.01  # rad/s
    gyro_bias = np.array([0.02, -0.01, 0.015])  # rad/s
    
    gyro_x_meas = gyro_x + gyro_bias[0] + np.random.normal(0, gyro_noise_std, len(t))
    gyro_y_meas = gyro_y + gyro_bias[1] + np.random.normal(0, gyro_noise_std, len(t))
    gyro_z_meas = gyro_z + gyro_bias[2] + np.random.normal(0, gyro_noise_std, len(t))
    
    # Generate accelerometer readings
    # Start with gravity vector
    g = 9.81
    accel_readings = []
    
    for i in range(len(t)):
        # Create rotation matrix for current attitude
        r = R.from_euler('xyz', [true_roll[i], true_pitch[i], true_yaw[i]])
        
        # Rotate gravity vector
        gravity = r.apply([0, 0, g])
        
        # Add noise to accelerometer
        accel_noise_std = 0.1  # m/s^2
        noisy_accel = gravity + np.random.normal(0, accel_noise_std, 3)
        accel_readings.append(noisy_accel)
    
    accel_readings = np.array(accel_readings)
    
    # Create DataFrame
    data = pd.DataFrame({
        'timestamp': t,
        'gyro_x': gyro_x_meas,
        'gyro_y': gyro_y_meas,
        'gyro_z': gyro_z_meas,
        'accel_x': accel_readings[:, 0],
        'accel_y': accel_readings[:, 1],
        'accel_z': accel_readings[:, 2],
        'true_roll': true_roll,
        'true_pitch': true_pitch,
        'true_yaw': true_yaw,
        'true_gyro_x': gyro_x,
        'true_gyro_y': gyro_y,
        'true_gyro_z': gyro_z
    })
    
    return data

# Generate data
imu_data = generate_imu_data()

# Save to CSV
imu_data.to_csv('imu_data.csv', index=False)

# Print first few rows
print("\nFirst few rows of generated data:")
print(imu_data.head())

# Print statistical summary
print("\nStatistical summary:")
print(imu_data.describe())