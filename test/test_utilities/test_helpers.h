#ifndef TEST_HELPERS_H
#define TEST_HELPERS_H

#include <stdio.h>
#include <math.h>
#include <stdlib.h>

// Macro for comparing floating-point numbers
#define FLOAT_EQ(a, b, epsilon) (fabs((a) - (b)) <= (epsilon))

// Logging helper
#define TEST_LOG(msg, ...) printf("[TEST LOG] " msg "\n", ##__VA_ARGS__)

// Pass/Fail macros
#define TEST_PASS() printf("[TEST PASS] %s\n", __func__)
#define TEST_FAIL(msg, ...) do { \
    printf("[TEST FAIL] %s: " msg "\n", __func__, ##__VA_ARGS__); \
    exit(EXIT_FAILURE); \
} while (0)

// Test validation utility
int validate_result(double actual, double expected, double tolerance, const char* description);

// Setup and teardown utilities
void setup_test_environment();
void teardown_test_environment();

#endif // TEST_HELPERS_H
