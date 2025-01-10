#include "estimators/altitude.h"
#include <math.h>
#include <string.h>

// State vector indices
enum {
    STATE_ALTITUDE = 0,
    STATE_VELOCITY = 1,
    STATE_ACCEL = 2,
    STATE_BARO_BIAS = 3,
    STATE_ACCEL_BIAS = 4,
    NUM_STATES = 5
};

// Local function prototypes
static void predict_state(AltitudeEstimator* est, double dt);
static void update_with_tof(AltitudeEstimator* est, double tof_dist);
static void update_with_baro(AltitudeEstimator* est, double baro_press, double baro_temp);
static void update_with_accel(AltitudeEstimator* est, double accel_z);
static double pressure_to_altitude(double pressure, double temperature, double ground_pressure);

void altitude_estimator_init(AltitudeEstimator* est, const AltitudeEstConfig* config) {
    // Copy configuration
    est->config = *config;
    
    // Initialize state vector
    est->altitude = 0.0;
    est->vertical_velocity = 0.0;
    est->vertical_accel = 0.0;
    est->baro_bias = 0.0;
    est->accel_bias_z = 0.0;
    
    // Initialize covariance matrix
    memcpy(est->P, config->kalman.P, sizeof(double) * NUM_STATES * NUM_STATES);
    
    est->initialized = 1;
    est->timestamp = 0.0;
}

void altitude_estimator_update(AltitudeEstimator* est,
                             const double accel_z,
                             const double baro_press,
                             const double baro_temp,
                             const double tof_dist) {
    if (!est->initialized) return;

    double dt;
    if (est->timestamp == 0.0) {
        dt = est->config.dt;
    } else {
        dt = baro_press - est->timestamp;  // Use barometer timestamp for main loop
    }
    est->timestamp = baro_press;

    // 1. Prediction step
    predict_state(est, dt);
    
    // 2. Measurement updates
    // Note: Order matters - typically do most reliable/accurate first
    
    // Update with ToF if in range
    if (tof_dist > 0.0 && tof_dist < 5.0) {  // Assuming 5m max range
        update_with_tof(est, tof_dist);
    }
    
    // Update with barometer
    update_with_baro(est, baro_press, baro_temp);
    
    // Update with accelerometer
    update_with_accel(est, accel_z);
}

static void predict_state(AltitudeEstimator* est, double dt) {
    // State transition matrix F
    double F[NUM_STATES][NUM_STATES] = {0};
    
    // Position update: h = h + v*dt + 0.5*a*dt^2
    F[0][0] = 1.0;
    F[0][1] = dt;
    F[0][2] = 0.5 * dt * dt;
    
    // Velocity update: v = v + a*dt
    F[1][1] = 1.0;
    F[1][2] = dt;
    
    // Acceleration approximately constant
    F[2][2] = 1.0;
    
    // Biases approximately constant
    F[3][3] = 1.0;  // Baro bias
    F[4][4] = 1.0;  // Accel bias

    // Predict state
    double x_new[NUM_STATES];
    x_new[STATE_ALTITUDE] = est->altitude + est->vertical_velocity * dt +
                           0.5 * est->vertical_accel * dt * dt;
    x_new[STATE_VELOCITY] = est->vertical_velocity + est->vertical_accel * dt;
    x_new[STATE_ACCEL] = est->vertical_accel;
    x_new[STATE_BARO_BIAS] = est->baro_bias;
    x_new[STATE_ACCEL_BIAS] = est->accel_bias_z;

    // Predict covariance: P = F*P*F' + Q
    double P_temp[NUM_STATES][NUM_STATES];
    double P_new[NUM_STATES][NUM_STATES];
    
    // P_temp = F*P
    for (int i = 0; i < NUM_STATES; i++) {
        for (int j = 0; j < NUM_STATES; j++) {
            P_temp[i][j] = 0;
            for (int k = 0; k < NUM_STATES; k++) {
                P_temp[i][j] += F[i][k] * est->P[k][j];
            }
        }
    }
    
    // P_new = P_temp*F' + Q
    for (int i = 0; i < NUM_STATES; i++) {
        for (int j = 0; j < NUM_STATES; j++) {
            P_new[i][j] = 0;
            for (int k = 0; k < NUM_STATES; k++) {
                P_new[i][j] += P_temp[i][k] * F[j][k];  // Note: F[j][k] instead of F[k][j] for transpose
            }
        }
        // Add process noise
        P_new[i][i] += est->config.kalman.process_noise[i];
    }

    // Update state and covariance
    est->altitude = x_new[STATE_ALTITUDE];
    est->vertical_velocity = x_new[STATE_VELOCITY];
    est->vertical_accel = x_new[STATE_ACCEL];
    est->baro_bias = x_new[STATE_BARO_BIAS];
    est->accel_bias_z = x_new[STATE_ACCEL_BIAS];
    memcpy(est->P, P_new, sizeof(P_new));
}

static void update_with_tof(AltitudeEstimator* est, double tof_dist) {
    // Measurement matrix H for ToF: only measures altitude
    double H[1][NUM_STATES] = {0};
    H[0][STATE_ALTITUDE] = 1.0;  // Assuming ToF is vertical. If angled, multiply by cos(angle)
    
    // Compute Kalman gain
    double PHt[NUM_STATES];    // P * H'
    double HPHt_R;            // H * P * H' + R
    double K[NUM_STATES];     // Kalman gain
    
    // PHt = P * H'
    for (int i = 0; i < NUM_STATES; i++) {
        PHt[i] = est->P[i][STATE_ALTITUDE];  // Since H is zeros except H[0][0]=1
    }
    
    // HPHt = H * P * H'
    HPHt_R = est->P[STATE_ALTITUDE][STATE_ALTITUDE] + est->config.kalman.measurement_noise[1];
    
    // K = PHt / (HPHt + R)
    for (int i = 0; i < NUM_STATES; i++) {
        K[i] = PHt[i] / HPHt_R;
    }
    
    // Update state
    double innovation = tof_dist - est->altitude;
    est->altitude += K[STATE_ALTITUDE] * innovation;
    est->vertical_velocity += K[STATE_VELOCITY] * innovation;
    est->vertical_accel += K[STATE_ACCEL] * innovation;
    est->baro_bias += K[STATE_BARO_BIAS] * innovation;
    est->accel_bias_z += K[STATE_ACCEL_BIAS] * innovation;
    
    // Update covariance
    // P = (I - KH)P
    double temp[NUM_STATES][NUM_STATES];
    for (int i = 0; i < NUM_STATES; i++) {
        for (int j = 0; j < NUM_STATES; j++) {
            temp[i][j] = est->P[i][j] - K[i] * est->P[STATE_ALTITUDE][j];
        }
    }
    memcpy(est->P, temp, sizeof(temp));
}

static void update_with_baro(AltitudeEstimator* est, double baro_press, double baro_temp) {
    // Convert pressure to altitude
    double baro_alt = pressure_to_altitude(baro_press, baro_temp, est->config.ground_pressure);
    
    // Measurement matrix H for barometer
    double H[1][NUM_STATES] = {0};
    H[0][STATE_ALTITUDE] = 1.0;
    H[0][STATE_BARO_BIAS] = 1.0;  // Barometer measures altitude plus its bias
    
    // Similar Kalman update as with ToF...
    // (Implementation similar to update_with_tof but with different H matrix)
}

static void update_with_accel(AltitudeEstimator* est, double accel_z) {
    // Measurement matrix H for accelerometer
    double H[1][NUM_STATES] = {0};
    H[0][STATE_ACCEL] = 1.0;
    H[0][STATE_ACCEL_BIAS] = 1.0;  // Accelerometer measures acceleration plus its bias
    
    // Similar Kalman update as above...
    // (Implementation similar to update_with_tof but with different H matrix)
}

static double pressure_to_altitude(double pressure, double temperature, double ground_pressure) {
    // Using barometric formula
    const double R = 287.05;  // Specific gas constant for air
    const double g = 9.81;    // Gravity
    
    return -(R * temperature) / g * log(pressure / ground_pressure);
}

void altitude_estimator_get_state(const AltitudeEstimator* est,
                                double* altitude,
                                double* vertical_velocity,
                                double* vertical_accel) {
    if (altitude) *altitude = est->altitude;
    if (vertical_velocity) *vertical_velocity = est->vertical_velocity;
    if (vertical_accel) *vertical_accel = est->vertical_accel;
}

void altitude_estimator_get_biases(const AltitudeEstimator* est,
                                 double* baro_bias,
                                 double* accel_bias) {
    if (baro_bias) *baro_bias = est->baro_bias;
    if (accel_bias) *accel_bias = est->accel_bias_z;
}

void altitude_estimator_reset(AltitudeEstimator* est) {
    est->altitude = 0.0;
    est->vertical_velocity = 0.0;
    est->vertical_accel = 0.0;
    est->baro_bias = 0.0;
    est->accel_bias_z = 0.0;
    
    // Reset covariance to initial values
    memcpy(est->P, est->config.kalman.P, sizeof(double) * NUM_STATES * NUM_STATES);
}

void altitude_estimator_set_ground_reference(AltitudeEstimator* est,
                                           double ground_pressure,
                                           double ground_temperature) {
    est->config.ground_pressure = ground_pressure;
    est->config.ground_temperature = ground_temperature;
}

void altitude_estimator_get_default_config(AltitudeEstConfig* config) {
    if (config == NULL) return;

    // Initialize estimation type
    config->type = ALT_ESTIMATOR_KALMAN;
    
    // Set default sample time
    config->dt = 0.01;  // 100Hz
    
    // Set default ToF configuration
    config->tof_angle = 0.0;  // Assumes vertical mounting
    
    // Set reference values
    config->ground_pressure = 101325.0;    // Sea level pressure in Pa
    config->ground_temperature = 288.15;   // 15°C in Kelvin

    // Initialize Kalman filter parameters
    // Process noise
    config->kalman.process_noise[0] = 0.01;   // altitude noise
    config->kalman.process_noise[1] = 0.05;   // velocity noise
    config->kalman.process_noise[2] = 0.1;    // acceleration noise
    config->kalman.process_noise[3] = 0.001;  // baro bias noise
    config->kalman.process_noise[4] = 0.001;  // accel bias noise

    // Measurement noise
    config->kalman.measurement_noise[0] = 2.0;  // barometer noise (meters)
    config->kalman.measurement_noise[1] = 0.04; // ToF noise (meters)

    // Initialize covariance matrix
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            config->kalman.P[i][j] = 0.0;
        }
        // Set diagonal elements
        config->kalman.P[i][i] = 1.0;
    }

    // Higher initial uncertainty for biases
    config->kalman.P[3][3] = 5.0;  // baro bias uncertainty
    config->kalman.P[4][4] = 1.0;  // accel bias uncertainty
}