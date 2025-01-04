import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

class AltitudeSensorSimulator:
    def __init__(self):
        # Sensor characteristics
        self.TOF_RANGE = 5.0          # Max range in meters
        self.TOF_NOISE = 0.02         # 2cm noise
        self.TOF_RATE = 50            # 50Hz
        
        self.BARO_NOISE = 0.2         # 20cm noise
        self.BARO_BIAS = 0.1          # 10cm bias
        self.BARO_RATE = 20           # 20Hz
        self.BARO_DRIFT = 0.05        # 5cm/s drift
        
        self.IMU_NOISE = 0.02         # 0.02 m/s² noise
        self.IMU_BIAS = 0.01          # 0.01 m/s² bias
        self.IMU_RATE = 200           # 200Hz
        
    def generate_true_trajectory(self, duration):
        """Generate a smooth altitude trajectory."""
        t = np.linspace(0, duration, int(duration * 200))  # 200Hz base rate
        
        # Create some waypoints
        waypoints_t = np.array([0, 2, 4, 6, 8, 10])
        waypoints_h = np.array([0, 2, 2, 1, 3, 2])
        
        # Interpolate for smooth trajectory
        f = interp1d(waypoints_t, waypoints_h, kind='cubic')
        altitude = f(t)
        
        # Calculate derivatives
        velocity = np.gradient(altitude, t)
        acceleration = np.gradient(velocity, t)
        
        return t, altitude, velocity, acceleration

    def simulate_tof(self, t, true_altitude):
        """Simulate ToF sensor readings."""
        # Generate timestamps at sensor rate
        t_tof = np.arange(0, t[-1], 1/self.TOF_RATE)
        
        # Interpolate true altitude to sensor timestamps
        altitude_interp = np.interp(t_tof, t, true_altitude)
        
        # Add noise
        noise = np.random.normal(0, self.TOF_NOISE, len(t_tof))
        measured = altitude_interp + noise
        
        # Apply range limit
        measured[measured > self.TOF_RANGE] = np.nan
        
        return t_tof, measured

    def simulate_baro(self, t, true_altitude):
        """Simulate barometer readings."""
        # Generate timestamps at sensor rate
        t_baro = np.arange(0, t[-1], 1/self.BARO_RATE)
        
        # Interpolate true altitude to sensor timestamps
        altitude_interp = np.interp(t_baro, t, true_altitude)
        
        # Add bias, drift and noise
        drift = self.BARO_DRIFT * t_baro
        noise = np.random.normal(0, self.BARO_NOISE, len(t_baro))
        measured = altitude_interp + self.BARO_BIAS + drift + noise
        
        return t_baro, measured

    def simulate_imu(self, t, true_accel):
        """Simulate IMU readings."""
        # Generate timestamps at sensor rate
        t_imu = np.arange(0, t[-1], 1/self.IMU_RATE)
        
        # Interpolate true acceleration to sensor timestamps
        accel_interp = np.interp(t_imu, t, true_accel)
        
        # Add bias and noise
        noise = np.random.normal(0, self.IMU_NOISE, len(t_imu))
        measured = accel_interp + self.IMU_BIAS + noise
        
        return t_imu, measured

    def generate_dataset(self, duration=10):
        """Generate complete dataset with all sensors."""
        # Generate true trajectory
        t, alt_true, vel_true, acc_true = self.generate_true_trajectory(duration)
        
        # Simulate each sensor
        t_tof, tof_meas = self.simulate_tof(t, alt_true)
        t_baro, baro_meas = self.simulate_baro(t, alt_true)
        t_imu, imu_meas = self.simulate_imu(t, acc_true)
        
        # Create pandas DataFrames for each sensor
        df_tof = pd.DataFrame({
            'timestamp': t_tof,
            'distance': tof_meas
        })
        
        df_baro = pd.DataFrame({
            'timestamp': t_baro,
            'altitude': baro_meas,
            'temperature': 25 + np.random.normal(0, 0.1, len(t_baro))  # Fake temperature data
        })
        
        df_imu = pd.DataFrame({
            'timestamp': t_imu,
            'accel_z': imu_meas
        })
        
        # Ground truth
        df_true = pd.DataFrame({
            'timestamp': t,
            'altitude': alt_true,
            'velocity': vel_true,
            'acceleration': acc_true
        })
        
        return df_tof, df_baro, df_imu, df_true
    
    def plot_data(self, df_tof, df_baro, df_imu, df_true):
        """Plot all sensor data against ground truth."""
        plt.figure(figsize=(15, 10))
        
        # Plot altitude
        plt.subplot(2, 1, 1)
        plt.plot(df_true['timestamp'], df_true['altitude'], 'k-', label='True Altitude')
        plt.plot(df_tof['timestamp'], df_tof['distance'], '.', label='ToF', alpha=0.5)
        plt.plot(df_baro['timestamp'], df_baro['altitude'], '.', label='Baro', alpha=0.5)
        plt.ylabel('Altitude (m)')
        plt.legend()
        plt.grid(True)
        
        # Plot acceleration
        plt.subplot(2, 1, 2)
        plt.plot(df_true['timestamp'], df_true['acceleration'], 'k-', label='True Accel')
        plt.plot(df_imu['timestamp'], df_imu['accel_z'], '.', label='IMU', alpha=0.5)
        plt.xlabel('Time (s)')
        plt.ylabel('Acceleration (m/s²)')
        plt.legend()
        plt.grid(True)
        
        plt.tight_layout()
        plt.show()

# Example usage
if __name__ == "__main__":
    # Create simulator
    sim = AltitudeSensorSimulator()
    
    # Generate data
    df_tof, df_baro, df_imu, df_true = sim.generate_dataset(duration=10)
    
    # Plot results
    sim.plot_data(df_tof, df_baro, df_imu, df_true)
    
    # Save to CSV files
    df_tof.to_csv('tof_data.csv', index=False)
    df_baro.to_csv('baro_data.csv', index=False)
    df_imu.to_csv('imu_data.csv', index=False)
    df_true.to_csv('true_data.csv', index=False)