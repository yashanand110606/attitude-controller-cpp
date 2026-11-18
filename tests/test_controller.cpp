/**
 * @file test_controller.cpp
 * @brief Main test entry point with all unit tests
 */
#define CATCH_CONFIG_MAIN
#include <catch2/catch_all.hpp>
#include <cmath>
#include <chrono>
#include <numeric>
#include "../src/attitude.h"

// ===== IMU Simulation Tests =====

TEST(IMU_Simulation, GaussianNoiseStats) {
    std::vector<double> gyro_noise_samples;
    gyro_noise_samples.reserve(1000);

    IMUSimulator sim(0.01, 0.01, 0.001);

    for (int i = 0; i < 1000; ++i) {
        GyroData g = sim.generate_gyro(0.0, 0.0, 0.0);
        gyro_noise_samples.push_back(g.wx);
    }

    double gyro_sum = std::accumulate(gyro_noise_samples.begin(),
                                      gyro_noise_samples.end(), 0.0);
    double gyro_mean = gyro_sum / gyro_noise_samples.size();

    double gyro_sq_sum = 0.0;
    for (double val : gyro_noise_samples) {
        gyro_sq_sum += (val - gyro_mean) * (val - gyro_mean);
    }
    double gyro_std = std::sqrt(gyro_sq_sum / gyro_noise_samples.size());

    EXPECT_NEAR(gyro_mean, 0.0, 0.001)
        << "Gyro mean should be approx 0, got " << gyro_mean;
    EXPECT_NEAR(gyro_std, 0.01, 0.001)
        << "Gyro std should be approx 0.01, got " << gyro_std;

    // Accelerometer noise test
    std::vector<double> accel_noise_samples;
    accel_noise_samples.reserve(1000);
    for (int i = 0; i < 1000; ++i) {
        AccelData a = sim.generate_accel(0.0, 0.0, 9.81);
        accel_noise_samples.push_back(a.noise_x);
    }

    double accel_sum = std::accumulate(accel_noise_samples.begin(),
                                       accel_noise_samples.end(), 0.0);
    double accel_mean = accel_sum / accel_noise_samples.size();

    double accel_sq_sum = 0.0;
    for (double val : accel_noise_samples) {
        accel_sq_sum += (val - accel_mean) * (val - accel_mean);
    }
    double accel_std = std::sqrt(accel_sq_sum / accel_noise_samples.size());

    EXPECT_NEAR(accel_mean, 0.0, 0.001)
        << "Accel mean should be approx 0, got " << accel_mean;
    EXPECT_NEAR(accel_std, 0.01, 0.001)
        << "Accel std should be approx 0.01, got " << accel_std;

    std::cerr << "Gyro: mean=" << gyro_mean << ", std=" << gyro_std << "\n";
    std::cerr << "Accel: mean=" << accel_mean << ", std=" << accel_std << "\n";
}

// ===== Complementary Filter Tests =====

TEST(ComplementaryFilter, ConvergesToAccelReading) {
    ComplementaryFilter filter(0.98, 0.1);
    double ax = 0.0, ay = 0.0, az = 9.81;

    for (int i = 0; i < 20; ++i) {
        filter.update(0.0, 0.0, 0.0, ax, ay, az);
    }

    auto euler = filter.get_euler();
    EXPECT_NEAR(euler[0], 0.0, 0.01)
        << "Roll should converge to 0 within 2s, got " << euler[0];
    EXPECT_NEAR(euler[1], 0.0, 0.01)
        << "Pitch should converge to 0 within 2s, got " << euler[1];
}

TEST(ComplementaryFilter, RejectsGyroBias) {
    ComplementaryFilter filter(0.98, 0.1);

    for (int i = 0; i < 100; ++i) {
        filter.update(0.1, 0.0, 0.0, 0.0, 0.0, 9.81);
    }

    auto euler = filter.get_euler();
    EXPECT_LT(std::abs(euler[0]), M_PI)
        << "Roll should not diverge beyond pi, got " << euler[0];
    EXPECT_LT(std::abs(euler[1]), M_PI)
        << "Pitch should not diverge beyond pi, got " << euler[1];
    EXPECT_LT(std::abs(euler[0]), 1.0)
        << "Roll should be bounded after 10s with bias, got " << euler[0];
}

// ===== Controller Stability Tests =====

TEST(Controller, StabilizesToIdentity) {
    ComplementaryFilter filter(0.98, 0.1);
    double initial_roll = M_PI / 4;    // 45 degrees
    double initial_pitch = M_PI / 6;   // 30 degrees
    double initial_yaw = 0.0;

    auto q_initial = Quaternion::from_euler(initial_roll, initial_pitch,
                                            initial_yaw);
    auto euler_initial = q_initial.to_euler();
    EXPECT_NEAR(euler_initial[0], initial_roll, 1e-10);
    EXPECT_NEAR(euler_initial[1], initial_pitch, 1e-10);

    for (int i = 0; i < 50; ++i) {
        filter.update(0.0, 0.0, 0.0, 0.0, 0.0, 9.81);
    }

    auto euler_final = filter.get_euler();
    EXPECT_LT(std::abs(euler_final[0]), std::abs(initial_roll))
        << "Roll should decrease from " << initial_roll << " to "
        << euler_final[0];
    EXPECT_LT(std::abs(euler_final[1]), std::abs(initial_pitch))
        << "Pitch should decrease from " << initial_pitch << " to "
        << euler_final[1];
    EXPECT_LT(std::abs(euler_final[0]), 0.1)
        << "Roll should be < 0.1 rad from identity after 5s, got "
        << euler_final[0];
    EXPECT_LT(std::abs(euler_final[1]), 0.1)
        << "Pitch should be < 0.1 rad from identity after 5s, got "
        << euler_final[1];
}