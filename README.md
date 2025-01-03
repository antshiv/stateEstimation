# State Estimation Library in C (C-StateEstimator)
A lightweight C library for state estimation, focusing on attitude, position, and sensor fusion. This library provides implementations of common filtering algorithms including complementary and Kalman filters.

## Features
- Multiple attitude estimation methods:
  - Complementary filter (simple, efficient)
  - Basic Kalman filter (better accuracy, more computational cost)
  - Extended Kalman filter (coming soon)
  - Unscented Kalman filter (coming soon)
- Position estimation
- Support for various sensor inputs (IMU, accelerometer, gyroscope)
- Configurable filter parameters
- Minimal dependencies
- Platform-independent implementation

## Dependencies
- [Attitude Math Library](https://github.com/antshiv/attitudeMathLibrary.git) (included as submodule)
- Standard C library
- CMake (for building)

## Directory Structure
```bash
stateEstimation/
├── CMakeLists.txt
├── external/
│   └── attitudeMathLibrary/    # Git submodule for attitude mathematics
├── include/
│   ├── estimators/             # Main estimator interfaces
│   │   ├── attitude.h          # Attitude estimation interface
│   │   ├── position.h
│   │   └── trajectory.h
│   ├── filters/                # Filter implementations
│   │   ├── complementary.h     # Complementary filter
│   │   ├── kalman.h           # Kalman filter implementations
│   │   ├── particle.h
│   │   └── vector_ops.h       # Vector operations
│   └── types/
│       └── state_types.h       # Common data structures
├── src/
│   └── estimators/
│       └── attitude.c          # Attitude estimation implementation
└── test/
    ├── scripts/                # Test data generation
    │   ├── data_gen.py
    │   └── imu_data.csv
    ├── test_attitude.c         # Basic attitude tests
    ├── test_attitude_fn.c      # Function-specific tests
    └── test_kalman.c          # Kalman filter tests
```

## Building

1. Clone the repository with submodules:
```bash
git clone --recursive https://github.com/antshiv/stateEstimation.git 
cd state_estimation
```

2. Create and enter build directory:
```bash
mkdir build && cd build
```

3. Build the library:
```bash
cmake ..
make
```

## Usage Example

```c
#include <stdio.h>
#include "estimators/attitude.h"

int main() {
    // Initialize estimator with Kalman filter
    AttitudeEstimator estimator;
    AttitudeEstConfig config = {
        .type = ESTIMATOR_KALMAN,
        .dt = 0.01  // 100Hz sample rate
    };
    attitude_estimator_init(&estimator, &config);

    // Update with sensor data
    double gyro[3] = {0.01, 0.02, 0.03};  // rad/s
    double accel[3] = {0.0, 0.0, 1.0};    // g
    attitude_estimator_update(&estimator, gyro, accel);

    // Get attitude estimate
    double quaternion[4];
    attitude_estimator_get_quaternion(&estimator, quaternion);
    return 0;
}
```

## API Reference

### Attitude Estimation

```c
// Initialize attitude estimator
void attitude_estimator_init(AttitudeEstimator* est, const AttitudeEstConfig* config);

// Update with new measurements
void attitude_estimator_update(AttitudeEstimator* est,
                             const double gyro[3],    // Gyro measurements [x,y,z]
                             const double accel[3]);  // Accelerometer measurements [x,y,z]

// Get current attitude estimates
void attitude_estimator_get_quaternion(const AttitudeEstimator* est, double q[4]);
void attitude_estimator_get_euler(const AttitudeEstimator* est, EulerAngles* euler);
```

## Configuration Parameters

### Complementary Filter
- `alpha`: Weight for gyroscope integration (0.0 - 1.0)
- `dt`: Sample time in seconds

### Attitude Estimator
- Gyroscope bias estimation
- Adaptive filtering during rapid motion
- Quaternion-based attitude representation

## Testing

Run the test suite:
```bash
cd build
./test_attitude
./test_kalman         # Kalman filter specific tests
```

## Contributing

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add some amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- Thank you to the contributors of the Attitude Math Library
- Inspired by various state estimation techniques in robotics and aerospace applications
