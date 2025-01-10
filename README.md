# State Estimation Library in C (C-StateEstimator)
A lightweight C library for state estimation focusing on attitude and altitude estimation using sensor fusion techniques.

## Features
- Attitude Estimation
  - Complementary filter
  - Extended Kalman filter
  - Quaternion-based representation
  - Bias estimation
- Altitude Estimation
  - Multi-sensor fusion (IMU, barometer, ToF)
  - Bias compensation
  - Robust to sensor dropouts
- Coming Soon
  - Unscented Kalman filter
  - Position estimation
  - Trajectory tracking

## Dependencies
- [Attitude Math Library](https://github.com/antshiv/attitudeMathLibrary.git)
- Standard C library
- CMake (3.10+)

## Quick Start
```bash
# Clone and build
git clone --recursive https://github.com/antshiv/stateEstimation.git 
cd stateEstimation
mkdir build && cd build
cmake ..
make

# Run tests
./test_attitude
./test_altitude
```

## Usage Examples

### Attitude Estimation
```c
#include "estimators/attitude.h"

// Initialize
AttitudeEstConfig config = {
    .type = ESTIMATOR_KALMAN,
    .dt = 0.01  // 100Hz
};
AttitudeEstimator estimator;
attitude_estimator_init(&estimator, &config);

// Update and get estimate
double gyro[3] = {0.01, 0.02, 0.03};  // rad/s
double accel[3] = {0.0, 0.0, 1.0};    // g
attitude_estimator_update(&estimator, gyro, accel);

double quaternion[4];
attitude_estimator_get_quaternion(&estimator, quaternion);
```

### Altitude Estimation
```c
#include "estimators/altitude.h"

// Initialize with default configuration
AltitudeEstConfig config;
altitude_estimator_get_default_config(&config);
AltitudeEstimator estimator;
altitude_estimator_init(&estimator, &config);

// Update and get estimate
altitude_estimator_update(&estimator, 9.81, 98000.0, 20.0, 2.0);

double altitude, velocity, acceleration;
altitude_estimator_get_state(&estimator, &altitude, &velocity, &acceleration);
```

## Configuration Parameters

### Complementary Filter
```c
typedef struct {
    double alpha;     // Gyro weight [0.0-1.0]
    double dt;        // Sample time (seconds)
} ComplementaryConfig;
```

### Kalman Filter
```c
typedef struct {
    // Process noise covariance
    double Q[7][7];   // State noise
    
    // Measurement noise covariance
    double R[3][3];   // Sensor noise
    
    // Initial state covariance
    double P0[7][7];  // Initial uncertainty
    
    // Filter parameters
    double dt;        // Sample time
    int max_iterations; // For iterative updates
} KalmanConfig;
```

### Altitude Estimator
```c
typedef struct {
    // Process noise for states
    double process_noise[5];     // [h, v, a, baro_bias, accel_bias]
    double measurement_noise[2]; // [barometer, tof]
    
    // Sensor configuration
    double tof_angle;           // ToF mounting angle
    double ground_pressure;     // Reference pressure
    double ground_temperature;  // Reference temperature
    
    // Filter parameters
    double dt;                  // Sample time
} AltitudeEstConfig;
```

## Testing
- Basic tests: `test_attitude.c`, `test_altitude.c`
- Detailed tests: `test_altitude_verbose.c`
- Individual filter tests: `test_kalman.c`
- Test documentation: See `doc/test_altitude.md`

## Project Structure
```
stateEstimation/
├── include/                  # Public headers
│   ├── estimators/          # Main interfaces
│   ├── filters/             # Filter implementations
│   └── types/               # Data structures
├── src/                     # Implementation
├── test/                    # Test suite
└── doc/                     # Documentation
```

## License
MIT License - See LICENSE file

## Contributing
1. Fork repository
2. Create feature branch
3. Commit changes
4. Push to branch
5. Create Pull Request