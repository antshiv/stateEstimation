# Minimum CMake version required
cmake_minimum_required(VERSION 3.10)

# Project name and version
project(StateEstimation VERSION 1.0)

# Set the C standard
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED True)

# Add the attitude math library as a submodule
add_subdirectory(external/attitudeMathLibrary)
include_directories(external/attitudeMathLibrary/include)

# Define source files for the library
set(SOURCES
    src/estimators/attitude.c
)

# Define include directories
set(INCLUDE_DIRS
    include
)

# Add the library
add_library(state_estimation ${SOURCES})

# Include directories for the library
target_include_directories(state_estimation PUBLIC ${INCLUDE_DIRS})

# Create a test executable
add_executable(test_attitude_estimate test/test_attitude.c)
add_executable(test_attitude_estimate_fn test/test_attitude_fn.c)
add_executable(test_attitude_kalman test/test_kalman.c)

# Link the test executable with the library
target_link_libraries(test_attitude_estimate PRIVATE state_estimation m attitude)
target_link_libraries(test_attitude_estimate_fn PRIVATE state_estimation m attitude)
target_link_libraries(test_attitude_kalman PRIVATE state_estimation m attitude)

# Add tests
enable_testing()
add_test(NAME TestAttitude COMMAND test_attitude_estmiate)
add_test(NAME TestAttitudeFn COMMAND test_attitude_estmiate_fn)

