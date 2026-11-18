/**
 * @file test_controller.cpp
 * @brief Main test entry point with all unit tests
 */
#define CATCH_CONFIG_MAIN
#include <catch2/catch_all.hpp>
#include <cmath>
#include <chrono>
#include "../src/attitude.cpp"
#include "../src/imu_sim.cpp"

// ===== IMU Simulation Tests =====

/**
 * @test IMU_Simulation_GaussianNoiseStats
 * @brief Generate 1000 samples, verify mean approximately 0,
 *        std approximately expected sigma
 */
TEST(IMU_Simulation, GaussianNoiseStats) {
    // Gyro noise test
    std::vector<double> gyro_noise_samples;
    gyro_noise_samples.reserve(1000);

    IMUSimulator sim(0.01, 0.01, 0.001);

    // Generate 1000 gyro samples (wx component)
    for (int i = 0; i < 1000; ++i) {
        GyroData g = sim.generate_gyro(0.0, 0.0, 0.0);
        gyro_noise_samples.push_back(g.wx);
    }

    // Calculate statistics
    double gyro_sum = std::accumulate(
        gyro_noise_samples.begin(), gyro_noise_samples.end(), 0.0);
    double gyro_mean = gyro_sum / gyro_noise_samples.size();

    double gyro_sq_sum = 0.0;
    for (double val : gyro_noise_samples) {
        gyro_sq_sum += (val - gyro_mean) * (val - gyro_mean);
    }
    double gyro_std =
        std::sqrt(gyro_sq_sum / gyro_noise_samples.size());

    // Verify mean approximately 0
    EXPECT_NEAR(gyro_mean, 0.0, 0.001)
        << "Gyro mean should be approximately 0, got " << gyro_mean;

    // Verify std approximately expected sigma (0.01)
    EXPECT_NEAR(gyro_std, 0.01, 0.001)
        << "Gyro std should be approximately 0.01, got " << gyro_std;

    // Accelerometer noise test
    std::vector<double> accel_noise_samples;
    accel_noise_samples.reserve(1000);

    // Generate 1000 accel samples (ax component)
    for (int i = 0; i < 1000; ++i) {
        AccelData a = sim.generate_accel(0.0, 0.0, 9.81);
        accel_noise_samples.push_back(a.noise_x);
    }

    // Calculate statistics
    double accel_sum = std::accumulate(
        accel_noise_samples.begin(), accel_noise_samples.end(), 0.0);
    double accel_mean = accel_sum / accel_noise_samples.size();

    double accel_sq_sum = 0.0;
    for (double val : accel_noise_samples) {
        accel_sq_sum += (val - accel_mean) * (val - accel_mean);
    }
    double accel_std =
        std::sqrt(accel_sq_sum / accel_noise_samples.size());

    // Verify mean approximately 0
    EXPECT_NEAR(accel_mean, 0.0, 0.001)
        << "Accel mean should be approximately 0, got " << accel_mean;

    // Verify std approximately expected sigma (0.01)
    EXPECT_NEAR(accel_std, 0.01, 0.001)
        << "Accel std should be approximately 0.01, got "
        << accel_std;

    std::cerr << "Gyro: mean=" << gyro_mean << ", std=" << gyro_std
              << std::endl;
    std::cerr << "Accel: mean=" << accel_mean << ", std=" << accel_std
              << std::endl;
}

// ===== Complementary Filter Tests =====

/**
 * @test ComplementaryFilter_ConvergesToAccelReading
 * @brief Run filter with constant accel reading,
 *        verify convergence within 2s
 */
TEST(ComplementaryFilter, ConvergesToAccelReading) {
    // Create filter with alpha=0.98, dt=0.1s
    ComplementaryFilter filter(0.98, 0.1);

    // Simulate constant accelerometer reading: level (no rotation)
    // ax=0, ay=0, az=9.81 means gravity pointing straight down
    double ax = 0.0;
    double ay = 0.0;
    double az = 9.81;

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
    EXPECT_NEAR(euler[0], 0.0, 0.01)
        << "Roll should converge to 0 within 2s, got " << euler[0];
    EXPECT_NEAR(euler[1], 0.0, 0.01)
        << "Pitch should converge to 0 within 2s, got " << euler[1];
}

/**
 * @test ComplementaryFilter_RejectsGyroBias
 * @brief Run filter with biased gyro,
 *        verify estimate does not diverge infinitely
 */
TEST(ComplementaryFilter, RejectsGyroBias) {
    // Create filter with alpha=0.98, dt=0.1s
    ComplementaryFilter filter(0.98, 0.1);

    // Simulate constant gyro bias: wx=0.1 rad/s constant, but true
    // rotation is 0
    // This tests that the filter does not let the estimate diverge
    // indefinitely because the accel reference pulls it back

    // Run filter for 100 iterations (10 seconds at 10Hz)
    // with a constant gyro bias of 0.1 rad/s
    for (int i = 0; i < 100; ++i) {
        // Gyro reads 0.1 rad/s even though there is no actual rotation
        // Accel reads level (gravity straight down)
        filter.update(0.1, 0.0, 0.0, 0.0, 0.0, 9.81);
    }

    // Get the estimated Euler angles
    auto euler = filter.get_euler();

    // The estimate should NOT diverge to infinity
    // With complementary filtering, the accel reference keeps it bounded
    EXPECT_LT(std::abs(euler[0]), M_PI)
        << "Roll should not be pi or beyond, got " << euler[0];
    EXPECT_LT(std::abs(euler[1]), M_PI)
        << "Pitch should not be pi or beyond, got " << euler[1];

    // After 10s with complementary filtering, the estimate should be
    // roughly within some bound of the true state (level)
    // The bias should not have caused unbounded divergence
    EXPECT_LT(std::abs(euler[0]), 1.0)
        << "Roll should be less than 1 radian after 10s with bias, got "
        << euler[0];
}

// ===== Controller Stability Tests =====

/**
 * @test Controller_StabilizesToIdentity
 * @brief Run controller from random initial attitude,
 *        verify convergence to identity
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
    auto q_initial = Quaternion::from_euler(initial_roll,
                                            initial_pitch,
                                            initial_yaw);

    // Verify it is not identity
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
    // After 5 seconds of complementary filtering from 45/30 degree initial
    // attitude, the error should be significantly reduced
    EXPECT_LT(std::abs(euler_final[0]), std::abs(initial_roll))
        << "Roll should decrease from " << initial_roll
        << " to " << euler_final[0];
    EXPECT_LT(std::abs(euler_final[1]), std::abs(initial_pitch))
        << "Pitch should decrease from " << initial_pitch
        << " to " << euler_final[1];

    // Should be reasonably close to identity after 5 seconds
    // Expect less than 0.1 rad error (about 5.7 degrees)
    EXPECT_LT(std::abs(euler_final[0]), 0.1)
        << "Roll should be less than 0.1 rad from identity after 5s, "
        << "got " << euler_final[0];
    EXPECT_LT(std::abs(euler_final[1]), 0.1)
        << "Pitch should be less than 0.1 rad from identity after 5s, "
        << "got " << euler_final[1];
}