# Understanding Altitude Estimation Tests

## Introduction
In drone navigation, accurate altitude estimation is crucial for stable flight and precise control. Our altitude estimator fuses data from three sensors: an IMU (accelerometer), a barometer, and a Time of Flight (ToF) sensor. Each sensor has its strengths and limitations, which is why we combine them using a Kalman filter for optimal estimation.

## Why We Test
Our testing focuses on four critical aspects of altitude estimation:
1. Stability during hover
2. Response to altitude changes
3. Robustness to sensor failures
4. Handling of sensor noise

These aspects directly impact a drone's ability to maintain position, execute movements, and handle real-world conditions.

## Test Scenarios and Sensor Values

### Constant Altitude (Hover) Test

This test verifies if our estimator can maintain a stable altitude estimate when the drone is hovering. In real flights, even during "stable" hover, sensors show slight variations and noise.

| Sensor | Input Value | Why This Value? |
|--------|-------------|-----------------|
| Accelerometer | 9.81 m/s² | Compensates for gravity during hover |
| Barometer | 98000 Pa | Standard pressure at ~200m altitude |
| ToF | 2.0m | Simulated hovering height |

Pass Criteria:
- Estimated altitude stays within 2.0m ± 0.1m
- Estimated velocity stays within 0.0 m/s ± 0.1m/s
- Estimated acceleration stays within 0.0 m/s² ± 0.1m/s²

### Step Response Test

This test examines how well our estimator tracks sudden altitude changes, similar to when a drone moves from one height to another.

| Time Period | Sensor | Value | Purpose |
|-------------|---------|--------|----------|
| 0-1s | Accelerometer | 11.81 m/s² | Upward acceleration (9.81 + 2.0) |
| 1-5s | Accelerometer | 9.81 m/s² | Hover at new height |
| 0-5s | Barometer | 98000 → 97800 Pa | Pressure change with altitude |
| 0-5s | ToF | 0.0 → 2.0m | Height change |

Pass Criteria:
- Reaches target height (2.0m) within 3 seconds
- Overshoots by less than 0.5m
- Final error less than 0.1m

### Sensor Robustness Test

Real sensors can fail temporarily. This test ensures our estimator remains stable when sensor readings are unreliable.

| Test Phase | Accelerometer | Barometer | ToF | Expected Behavior |
|------------|---------------|-----------|-----|-------------------|
| Normal | 9.81 m/s² | 98000 Pa | 2.0m | Stable estimate |
| ToF Dropout | 9.81 m/s² | 98000 Pa | -1.0m | Maintain estimate using other sensors |
| Recovery | 9.81 m/s² | 98000 Pa | 2.0m | Smooth transition back |

Pass Criteria:
- Estimate doesn't jump when ToF fails
- Maintains reasonable estimate using remaining sensors
- Smoothly incorporates ToF when it recovers

### Noise Handling Test

Real sensors are noisy. This test verifies our filter's noise rejection capabilities.

| Sensor | Base Value | Added Noise | Why This Noise Level? |
|--------|------------|-------------|----------------------|
| Accelerometer | 9.81 m/s² | ±0.5 m/s² | Typical IMU noise |
| Barometer | 98000 Pa | ±50 Pa | Common atmospheric variation |
| ToF | 2.0m | ±0.05m | Typical ToF accuracy |

Pass Criteria:
- Average estimate stays within 2.0m ± 0.1m (unbiased)
- Root Mean Square Error < 0.2m (noise rejection)

## Running the Tests

When you run `./test_altitude`, you'll see output like this:
```
Starting altitude estimator tests...
Running constant altitude test...
Constant altitude test passed!
...
```

Each "passed!" message means:
1. The estimator processed the simulated sensor data
2. The estimates stayed within the specified bounds
3. No numerical issues occurred

## Understanding Test Implementation

Let's look at how the constant altitude test works internally:

```c
void test_constant_altitude(void) {
    // 1. Setup estimator with default configuration
    AltitudeEstConfig config;
    altitude_estimator_get_default_config(&config);
    AltitudeEstimator est;
    altitude_estimator_init(&est, &config);
    
    // 2. Run estimator for 5 seconds at 100Hz
    const double dt = 0.01;           
    const int num_steps = 500;        // 5 seconds * 100Hz
    
    // 3. Feed constant sensor values
    for(int i = 0; i < num_steps; i++) {
        altitude_estimator_update(&est, 9.81, 98000.0, 20.0, 2.0);
        
        // 4. Get and verify estimates
        double altitude, velocity, accel;
        altitude_estimator_get_state(&est, &altitude, &velocity, &accel);
        assert(fabs(altitude - 2.0) <= 0.1);  // Must stay within 10cm
    }
}
```

## When Tests Fail

If a test fails, it means one of these occurred:
1. Estimate diverged from expected values
2. Numerical instability in the Kalman filter
3. Poor handling of sensor variations

Example failure and diagnosis:
```
Running constant altitude test...
test_altitude: test_altitude.c:45: test_constant_altitude: 
Assertion `fabs(altitude - 2.0) <= 0.1' failed.
```
This would indicate the estimate drifted more than 10cm from the target height, suggesting possible issues with:
- Filter tuning parameters
- Sensor noise handling
- Numerical precision

## Future Testing Areas

Our current tests verify basic functionality. Future improvements could include:
1. Wind disturbance simulation
2. Temperature effects on barometer
3. Different surface reflectivity for ToF
4. Multi-rate sensor updates