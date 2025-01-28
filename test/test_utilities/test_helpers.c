#include "test_helpers.h"

// Validate a test result against an expected value
int validate_result(double actual, double expected, double tolerance, const char* description) {
    if (FLOAT_EQ(actual, expected, tolerance)) {
        printf("[VALIDATION PASS] %s: Actual=%.5f, Expected=%.5f, Tolerance=%.5f\n",
               description, actual, expected, tolerance);
        return 1; // Pass
    } else {
        printf("[VALIDATION FAIL] %s: Actual=%.5f, Expected=%.5f, Tolerance=%.5f\n",
               description, actual, expected, tolerance);
        return 0; // Fail
    }
}

// Setup the test environment (if needed)
void setup_test_environment() {
    // Example: Initialize random seed for reproducibility
    srand(42);
    printf("[SETUP] Test environment initialized.\n");
}

// Teardown the test environment (if needed)
void teardown_test_environment() {
    // Example: Clean up resources, if any
    printf("[TEARDOWN] Test environment cleaned up.\n");
}
