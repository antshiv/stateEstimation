# Enable testing
enable_testing()

# Set directories for utilities and test cases
set(TEST_UTILITIES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/test_utilities)
set(TEST_CASES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/test_cases)

# Add test utilities as a static library
file(GLOB TEST_UTILITY_SOURCES "${TEST_UTILITIES_DIR}/*.c")
add_library(test_utilities STATIC ${TEST_UTILITY_SOURCES})
target_include_directories(test_utilities PUBLIC ${TEST_UTILITIES_DIR})

# Function to create test executables for all test cases
function(create_test TARGET_NAME TEST_SOURCE)
    # Prefix test targets to avoid naming conflicts
    set(FULL_TARGET_NAME "se_${TARGET_NAME}")
    add_executable(${FULL_TARGET_NAME} ${TEST_SOURCE})
    target_link_libraries(${FULL_TARGET_NAME} PRIVATE state_estimation test_utilities m attitude)
    add_test(NAME ${FULL_TARGET_NAME} COMMAND ${FULL_TARGET_NAME})
endfunction()

# Add test cases
file(GLOB_RECURSE TEST_CASE_FILES "${TEST_CASES_DIR}/*/test_*.c")

foreach(TEST_FILE ${TEST_CASE_FILES})
    # Extract the base name of the test file (e.g., "test_kalman" from "test_kalman.c")
    get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)
    create_test(${TEST_NAME} ${TEST_FILE})
endforeach()
