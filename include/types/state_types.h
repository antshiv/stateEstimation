#ifndef STATE_TYPES_H
#define STATE_TYPES_H

#include <stdint.h>

// Basic vector type for 3D state estimation (meters or radians as applicable)
typedef struct {
    double x; // X component
    double y; // Y component
    double z; // Z component
} Vector3d;

// Quaternion for attitude representation (unitless)
typedef struct {
    double w; // Real component
    double x; // i component
    double y; // j component
    double z; // k component
} Quaternion;

// Basic state vector for position estimation
typedef struct {
    Vector3d position;      // Position (meters)
    Vector3d velocity;      // Velocity (meters/second)
    Vector3d acceleration;  // Acceleration (meters/second^2)
    double timestamp;       // Time (seconds)
    double covariance[9];   // 3x3 covariance matrix for position uncertainty
} PositionState;

// State vector for attitude estimation
typedef struct {
    Quaternion orientation; // Quaternion representation of orientation
    Vector3d angular_velocity;    // Angular velocity (radians/second)
    Vector3d angular_acceleration; // Angular acceleration (radians/second^2)
    double timestamp;        // Time (seconds)
    double covariance[9];    // 3x3 covariance matrix for orientation uncertainty
} AttitudeState;

// Euler angles representation (roll, pitch, yaw in radians)
typedef struct {
    double roll;    // Rotation around X (phi)
    double pitch;   // Rotation around Y (theta)
    double yaw;     // Rotation around Z (psi)
} EulerAngles;

// Combined state for full pose estimation
typedef struct {
    PositionState position_state; // Position-related state
    AttitudeState attitude_state; // Attitude-related state
    double timestamp;             // Time (seconds) for full pose
} PoseState;

// Generic state estimation status
typedef enum {
    STATE_EST_OK = 0,            // No errors
    STATE_EST_ERROR = -1,        // Generic error
    STATE_EST_INVALID_INPUT = -2, // Invalid inputs to estimator
    STATE_EST_INIT_ERROR = -3,   // Initialization failure
    STATE_EST_DIVERGED = -4,     // Filter divergence
    STATE_EST_RESERVED = -99     // Reserved for future expansion
} StateEstStatus;

#endif // STATE_TYPES_H

