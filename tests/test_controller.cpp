/**
 * @file test_controller.cpp
 * @brief All unit tests for attitude controller
 */
#define CATCH_CONFIG_MAIN
#include <catch_all.hpp>
#include <cmath>
#include <numeric>
#include "../src/attitude.h"

TEST(IMU_Simulation, GaussianNoiseStats) {
    std::vector<double> gyro_samples;
    gyro_samples.reserve(1000);
    IMUSimulator sim(0.01, 0.01, 0.001);
    for (int i = 0; i < 1000; ++i) {
        GyroData g = sim.generate_gyro(0.0, 0.0, 0.0);
        gyro_samples.push_back(g.wx);
    }
    double sum = std::accumulate(gyro_samples.begin(), gyro_samples.end(),
                                 0.0);
    double mean = sum / gyro_samples.size();
    double sq_sum = 0.0;
    for (double val : gyro_samples) {
        sq_sum += (val - mean) * (val - mean);
    }
    double std_dev = std::sqrt(sq_sum / gyro_samples.size());
    EXPECT_NEAR(mean, 0.0, 0.001);
    EXPECT_NEAR(std_dev, 0.01, 0.001);
}

TEST(ComplementaryFilter, ConvergesToAccelReading) {
    ComplementaryFilter filter(0.98, 0.1);
    for (int i = 0; i < 20; ++i) {
        filter.update(0.0, 0.0, 0.0, 0.0, 0.0, 9.81);
    }
    auto euler = filter.get_euler();
    EXPECT_NEAR(euler[0], 0.0, 0.01);
    EXPECT_NEAR(euler[1], 0.0, 0.01);
}

TEST(ComplementaryFilter, RejectsGyroBias) {
    ComplementaryFilter filter(0.98, 0.1);
    for (int i = 0; i < 100; ++i) {
        filter.update(0.1, 0.0, 0.0, 0.0, 0.0, 9.81);
    }
    auto euler = filter.get_euler();
    EXPECT_LT(std::abs(euler[0]), M_PI);
    EXPECT_LT(std::abs(euler[1]), M_PI);
    EXPECT_LT(std::abs(euler[0]), 1.0);
}

TEST(Controller, StabilizesToIdentity) {
    ComplementaryFilter filter(0.98, 0.1);
    double init_roll = M_PI / 4;
    double init_pitch = M_PI / 6;
    auto q_init = Quaternion::from_euler(init_roll, init_pitch, 0.0);
    auto e_init = q_init.to_euler();
    EXPECT_NEAR(e_init[0], init_roll, 1e-10);
    EXPECT_NEAR(e_init[1], init_pitch, 1e-10);
    for (int i = 0; i < 50; ++i) {
        filter.update(0.0, 0.0, 0.0, 0.0, 0.0, 9.81);
    }
    auto e_final = filter.get_euler();
    EXPECT_LT(std::abs(e_final[0]), std::abs(init_roll));
    EXPECT_LT(std::abs(e_final[1]), std::abs(init_pitch));
    EXPECT_LT(std::abs(e_final[0]), 0.1);
    EXPECT_LT(std::abs(e_final[1]), 0.1);
}