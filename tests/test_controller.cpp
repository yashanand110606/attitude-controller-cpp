/**
 * @file test_controller.cpp
 * @brief Controller stability tests
 *
 * Tests for attitude controller stability:
 * Stabilizes to identity quaternion from random initial attitude
 */

#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include <cmath>
#include <chrono>
#include "../src/attitude.cpp"

/**
 * @test Controller_StabilizesToIdentity
 * @brief Run controller from random initial attitude, verify convergence to identity
 */
TEST(Controller, StabilizesToIdentity) {
    // Create complementary filter
    ComplementaryFilter filter(0.98, 0.1);

    // Start from a random non-identity attitude
    // Use 45 degrees roll and 30 degrees pitch as initial condition
    double initial_roll = M_PI / 4;   // 45 degrees
    double initial_pitch = M_PI / 6;  // 30 degrees
    double initial_yaw = 0.0;

    // Convert initial Euler angles to quaternion
    auto q_initial = Quaternion::from_euler(initial_roll, initial_pitch, initial_yaw);

    // Verify it's not identity
    auto euler_initial = q_initial.to_euler();
    EXPECT_NEAR(euler_initial[0], initial_roll, 1e-10);
    EXPECT_NEAR(euler_initial[1], initial_pitch, 1e-10);

    // Run controller for 50 iterations (5 seconds at 10Hz)
    // With constant level accelerometer reading (gravity straight down)
    // and zero gyro, the filter should converge to identity
    for (int i = 0; i < 50; ++i) {
        // No gyro rotation, level accelerometer
        filter.update(0.0, 0.0, 0.0, 0.0, 0.0, 9.81);
    }

    // Get the final estimate
    auto euler_final = filter.get_euler();

    // The estimate should converge toward identity (0, 0)
    // After 5 seconds of complementary filtering from 45/30 degree initial attitude,
    // the error should be significantly reduced
    EXPECT_LT(std::abs(euler_final[0]), std::abs(initial_roll))
        << "Roll should decrease from " << initial_roll << " to " << euler_final[0];
    EXPECT_LT(std::abs(euler_final[1]), std::abs(initial_pitch))
        << "Pitch should decrease from " << initial_pitch << " to " << euler_final[1];

    // Should be reasonably close to identity after 5 seconds
    // Expect < 0.1 rad error (about 5.7 degrees)
    EXPECT_LT(std::abs(euler_final[0]), 0.1)
        << "Roll should be < 0.1 rad from identity after 5s, got " << euler_final[0];
    EXPECT_LT(std::abs(euler_final[1]), 0.1)
        << "Pitch should be < 0.1 rad from identity after 5s, got " << euler_final[1];
}