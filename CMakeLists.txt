cmake_minimum_required(VERSION 3.16)
project(AttitudeController CXX)

# Find Eigen3
find_package(Eigen3 REQUIRED)

# Catch2
set(Catch2_DIR "${CMAKE_CURRENT_SOURCE_DIR}/.catch2")
if(NOT EXISTS "${Catch2_DIR}/cmake/catch2-config.cmake"):
  message(STATUS "Downloading Catch2...")
  include(FetchContent)
  fetchContent(
    Catch2
    URL https://github.com/catch2/catch2/archive/refs/tags/v3.4.0.zip
    URL_HASH SHA256=e8c3b8e9e6f9e8e9e8e6e8e9e8e6e8e9e8e6e8e9e8e6e8e9e8e6e8e6e8e9e8e9
  )
endif()

# Add executable and tests
add_executable(attitude_controller
  src/imu_sim.cpp
  src/attitude.cpp
)

target_dependency(Eigen3::Eigen)
target_link_libraries(attitude_controller Eigen3::Eigen)

# Catch2 test executable
add_executable(test_controller
  tests/test_filter.cpp
  tests/test_controller.cpp
  tests/test_IMU.cpp
)

target_dependency(Eigen3::Eigen)
target_link_libraries(test_controller Catch2::Catch2WithMain Eigen3::Eigen)

# Install Catch2
if(NOT EXISTS "${Catch2_DIR}/cmake/catch2-config.cmake"):
  fetchContent(
    Catch2
    URL https://github.com/catch2/catch2/archive/refs/tags/v3.4.0.zip
    URL_HASH SHA256=e8c3b8e9e6f9e8e9e8e6e8e9e8e6e8e9e8e6e8e9e8e6e8e9e8e6e8e6e8e9e9
  )
endif()

# Test configuration
enable_testing()
add_test(NAME filter_test COMMAND test_controller --filter "ComplementaryFilter*")
add_test(NAME controller_test COMMAND test_controller --filter "Controller*")
add_test(NAME IMU_test COMMAND test_controller --filter "IMU_Simulation*")