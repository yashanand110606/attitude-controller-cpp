/**
 * @file test_filter.cpp
 * @brief Complementary filter unit tests
 *
 * Tests for complementary filter attitude estimation:
 * 1. Converges to accelerometer reading under constant input
 * 2. Rejects gyro bias (doesn't diverge infinitely)
 */

#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include <cmath>
#include <chrono>
#include "../src/attitude.cpp"

/**
 * @test ComplementaryFilter_ConvergesToAccelReading
 * @brief Run filter with constant accel reading, verify convergence within 2s
 */
TEST(ComplementaryFilter, ConvergesToAccelReading) {
    // Create filter with alpha=0.98, dt=0.1s
    ComplementaryFilter filter(0.98, 0.1);

    // Simulate constant accelerometer reading: level (no rotation)
    // ax=0, ay=0, az=9.81 means gravity pointing straight down
    double ax = 0.0, ay = 0.0, az = 9.81;

    // Run filter for 20 iterations (2 seconds at 10Hz)
    for (int i = 0; i < 20; ++i) {
        // No gyro rotation (wx=wy=wz=0), constant accel
        filter.update(0.0, 0.0, 0.0, ax, ay, az);
    }

    // Get the estimated roll/pitch
    auto euler = filter.get_euler();

    // With constant accel reading of gravity straight down:
    // - pitch should converge to 0 (gravity along z-axis)
    // - roll should converge to 0 (no roll)
    // The complementary filter should converge within ~2 seconds
    EXPECT_NEAR(euler[0], 0.0, 0.01)  // roll ≈ 0
        << "Roll should converge to 0 within 2s, got " << euler[0];
    EXPECT_NEAR(euler[1], 0.0, 0.01)  // pitch ≈ 0
        << "Pitch should converge to 0 within 2s, got " << euler[1];
}

/**
 * @test ComplementaryFilter_RejectsGyroBias
 * @brief Run filter with biased gyro, verify estimate doesn't diverge infinitely
 */
TEST(ComplementaryFilter, RejectsGyroBias) {
    // Create filter with alpha=0.98, dt=0.1s
    ComplementaryFilter filter(0.98, 0.1);

    // Simulate constant gyro bias: wx=0.1 rad/s constant, but true rotation is 0
    // This tests that the filter doesn't let the estimate diverge indefinitely
    // because the accel reference pulls it back

    // Run filter for 100 iterations (10 seconds at 10Hz)
    // with a constant gyro bias of 0.1 rad/s
    for (int i = 0; i < 100; ++i) {
        // Gyro reads 0.1 rad/s even though there's no actual rotation
        // Accel reads level (gravity straight down)
        filter.update(0.1, 0.0, 0.0, 0.0, 0.0, 9.81);
    }

    // Get the estimated Euler angles
    auto euler = filter.get_euler();

    // The estimate should NOT diverge to infinity
    // With complementary filtering, the accel reference keeps it bounded
    EXPECT_LT(std::abs(euler[0]), M_PI)  // roll should not be π or beyond
        << "Roll should remain bounded, got " << euler[0];
    EXPECT_LT(std::abs(euler[1]), M_PI)  // pitch should not be π or beyond
        << "Pitch should remain bounded, got " << euler[1];

    // After 10s with complementary filtering, the estimate should be roughly
    // within some bound of the true state (level)
    // The bias should not have caused unbounded divergence
    EXPECT_LT(std::abs(euler[0]), 1.0)  // roll should be < 1 radian
        << "Roll should be < 1 rad after 10s with bias, got " << euler[0];
}