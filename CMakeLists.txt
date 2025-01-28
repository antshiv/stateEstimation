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

# Define project source and include directories
set(SRC_DIR src/estimators)
set(INCLUDE_DIR include)

# Automatically find all source files in the `src/estimators` folder
file(GLOB ESTIMATOR_SOURCES "${SRC_DIR}/*.c")

# Define the library target
add_library(state_estimation ${ESTIMATOR_SOURCES})

# Include directories for the library
target_include_directories(state_estimation PUBLIC ${INCLUDE_DIR})

# Link the external library to the state_estimation library
target_link_libraries(state_estimation PUBLIC attitude)

# Add the test folder as a subdirectory
add_subdirectory(test)
