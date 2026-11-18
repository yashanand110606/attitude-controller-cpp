cmake_minimum_required(VERSION 3.16)
project(AttitudeController CXX)

# Find Eigen3
find_package(Eigen3 REQUIRED)

# Add main executable (attitude simulation)
add_executable(attitude_controller
    src/imu_sim.cpp
    src/attitude.cpp
)
target_link_libraries(attitude_controller Eigen3::Eigen)

# Add test executable
add_executable(test_controller
    tests/test_controller.cpp
)
target_include_directories(test_controller PRIVATE
    ${EIGEN3_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/catch2/include
)
target_link_libraries(test_controller Eigen3::Eigen)

# Test configuration
enable_testing()
add_test(NAME IMU_TESTS COMMAND test_controller "IMU_Simulation*")
add_test(NAME Filter_TESTS COMMAND test_controller "ComplementaryFilter*")
add_test(NAME Controller_TESTS COMMAND test_controller "Controller*")