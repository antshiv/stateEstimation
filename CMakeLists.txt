# Minimum CMake version required
cmake_minimum_required(VERSION 3.10)

# Project name and version
project(StateEstimation VERSION 1.0)

# Set the C standard
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED True)

# Enable testing
enable_testing()

# Add the attitude math library as a submodule
add_subdirectory(external/attitudeMathLibrary)
include_directories(external/attitudeMathLibrary/include)

# Define project source and include directories
set(SRC_DIR src/estimators)
set(TEST_DIR test)
set(INCLUDE_DIR include)

# Automatically find all source files in the `src/estimators` folder
file(GLOB ESTIMATOR_SOURCES "${SRC_DIR}/*.c")

# Automatically find all test files in the `test` folder
file(GLOB TEST_FILES "${TEST_DIR}/*.c")

# Define the library target
add_library(state_estimation ${ESTIMATOR_SOURCES})

# Include directories for the library
target_include_directories(state_estimation PUBLIC ${INCLUDE_DIR})

# Link the external library to the state_estimation library
target_link_libraries(state_estimation PUBLIC attitude)

# Function to create test executables for all test files
function(create_test TARGET_NAME TEST_SOURCE)
    # Prefix test targets to avoid naming conflicts
    set(FULL_TARGET_NAME "se_${TARGET_NAME}")
    add_executable(${FULL_TARGET_NAME} ${TEST_SOURCE})
    target_link_libraries(${FULL_TARGET_NAME} PRIVATE state_estimation m attitude)
    add_test(NAME ${FULL_TARGET_NAME} COMMAND ${FULL_TARGET_NAME})
endfunction()

# Iterate over all test files and create corresponding test executables
foreach(TEST_FILE ${TEST_FILES})
    # Extract the base name of the test file (e.g., "test_attitude" from "test_attitude.c")
    get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)
    create_test(${TEST_NAME} ${TEST_FILE})
endforeach()
