# State Estimation Library in C ( C-StateEstimator)

A lightweight C library for state estimation, focusing on attitude, position, and sensor fusion. This library provides implementations of common filtering algorithms including complementary and Kalman filters.

## Features

- Attitude estimation using complementary filter
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

```
stateEstimation/
├── CMakeLists.txt
├── external/
│   └── attitudeMathLibrary/  (git submodule)
├── include/
│   ├── estimators/
│   │   ├── attitude_estimator.h
│   │   ├── position_estimator.h
│   │   └── trajectory_estimator.h
│   ├── filters/
│   │   ├── complementary.h
│   │   ├── kalman.h
│   │   └── particle.h
│   └── types/
│       └── state_types.h
└── src/
    ├── estimators/
    └── filters/
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
#include "estimators/attitude_estimator.h"

int main() {
    // Initialize estimator
    AttitudeEstimator estimator;
    AttitudeEstConfig config = {
        .alpha = 0.98,  // Complementary filter weight
        .dt = 0.01      // 100Hz sample rate
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
