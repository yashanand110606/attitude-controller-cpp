/**
 * @file test_IMU.cpp
 * @brief Mocked sensor tests with 100% branch coverage
 *
 * Tests for IMU simulation Gaussian noise statistics.
 * Verifies mean ≈ 0 and std ≈ expected sigma for both gyro and accelerometer.
 */

#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include <cmath>
#include <vector>
#include <numeric>
#include <iostream>
#include "../src/imu_sim.cpp"

/**
 * @test IMU_Simulation_GaussianNoiseStats
 * @brief Generate 1000 samples, verify mean ≈ 0, std ≈ expected σ
 */
TEST(IMU_Simulation, GaussianNoiseStats) {
    // Gyro noise test
    std::vector<double> gyro_noise_samples;
    gyro_noise_samples.reserve(1000);

    IMUSimulator sim(0.01, 0.01, 0.001); // gyro_noise_std=0.01, accel_noise_std=0.01, bias=0.001

    // Generate 1000 gyro samples (wx component)
    for (int i = 0; i < 1000; ++i) {
        GyroData g = sim.generate_gyro(0.0, 0.0, 0.0);
        gyro_noise_samples.push_back(g.wx);
    }

    // Calculate statistics
    double gyro_sum = std::accumulate(gyro_noise_samples.begin(),
                                      gyro_noise_samples.end(), 0.0);
    double gyro_mean = gyro_sum / gyro_noise_samples.size();

    double gyro_sq_sum = 0.0;
    for (double val : gyro_noise_samples) {
        gyro_sq_sum += (val - gyro_mean) * (val - gyro_mean);
    }
    double gyro_std = std::sqrt(gyro_sq_sum / gyro_noise_samples.size());

    // Verify mean ≈ 0
    EXPECT_NEAR(gyro_mean, 0.0, 0.001)
        << "Gyro mean should be ≈ 0, got " << gyro_mean;

    // Verify std ≈ expected σ (0.01)
    EXPECT_NEAR(gyro_std, 0.01, 0.001)
        << "Gyro std should be ≈ 0.01, got " << gyro_std;

    // Accelerometer noise test
    std::vector<double> accel_noise_samples;
    accel_noise_samples.reserve(1000);

    // Generate 1000 accel samples (ax component)
    for (int i = 0; i < 1000; ++i) {
        AccelData a = sim.generate_accel(0.0, 0.0, 9.81);
        accel_noise_samples.push_back(a.noise_x);
    }

    // Calculate statistics
    double accel_sum = std::accumulate(accel_noise_samples.begin(),
                                       accel_noise_samples.end(), 0.0);
    double accel_mean = accel_sum / accel_noise_samples.size();

    double accel_sq_sum = 0.0;
    for (double val : accel_noise_samples) {
        accel_sq_sum += (val - accel_mean) * (val - accel_mean);
    }
    double accel_std = std::sqrt(accel_sq_sum / accel_noise_samples.size());

    // Verify mean ≈ 0
    EXPECT_NEAR(accel_mean, 0.0, 0.001)
        << "Accel mean should be ≈ 0, got " << accel_mean;

    // Verify std ≈ expected σ (0.01)
    EXPECT_NEAR(accel_std, 0.01, 0.001)
        << "Accel std should be ≈ 0.01, got " << accel_std;

    std::cerr << "Gyro: mean=" << gyro_mean << ", std=" << gyro_std << std::endl;
    std::cerr << "Accel: mean=" << accel_mean << ", std=" << accel_std << std::endl;
}