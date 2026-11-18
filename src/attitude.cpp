/**
 * @file attitude.cpp
 * @brief Complementary filter implementation and control loop
 */
#include "attitude.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>
#include <cassert>

// Main executable for running the attitude simulation loop
int main(int argc, char* argv[]) {
    int num_iterations = 100;
    bool verbose = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help") {
            std::cout << "Usage: attitude_controller [--iterations N] [--verbose]"
                      << std::endl;
            std::cout << "  --iterations N: Number of iterations (default: 100)"
                      << std::endl;
            std::cout << "  --verbose: Print detailed output per iteration"
                      << std::endl;
            return 0;
        } else if (arg == "--verbose") {
            verbose = true;
        } else if (arg == "--iterations" && i + 1 < argc) {
            num_iterations = std::stoi(argv[++i]);
        }
    }

    IMUSimulator imu_sim;
    Quaternion q;
    double total_time = 0.0;

    std::cerr << "Running complementary filter control loop..." << std::endl;
    std::cerr << "Iterations: " << num_iterations
              << ", DT: 0.1s (10Hz)" << std::endl;

    for (int i = 0; i < num_iterations; ++i) {
        auto iter_start = std::chrono::high_resolution_clock::now();
        auto [gyro, accel] = imu_sim.generate_orbit_data(total_time);

        ComplementaryFilter filter(0.98, 0.1);
        q = filter.update(gyro.wx, gyro.wy, gyro.wz,
                          accel.ax, accel.ay, accel.az);

        auto iter_end = std::chrono::high_resolution_clock::now();
        double iter_dt =
            std::chrono::duration<double>(iter_end - iter_start).count();
        total_time += iter_dt;

        if (verbose) {
            auto euler = q.to_euler();
            std::cout << "Iter " << std::setw(3) << i << " | wx="
                      << std::fixed << std::setprecision(4) << gyro.wx
                      << " wy=" << gyro.wy << " wz=" << gyro.wz
                      << " | accel=(" << std::setprecision(2) << accel.ax
                      << ", " << accel.ay << ", " << accel.az << ")"
                      << " | roll=" << euler[0] << " pitch=" << euler[1]
                      << " | dt=" << iter_dt << std::endl;
        }

        double sleep_time = 0.1 - iter_dt;
        if (sleep_time > 0.0) {
            std::this_thread::sleep_for(
                std::chrono::duration<double>(sleep_time));
        }
    }

    auto euler_final = q.to_euler();
    double avg_dt = total_time / num_iterations;
    double freq = 1.0 / avg_dt;

    std::cerr << "Final attitude - Roll: " << euler_final[0]
              << " rad (" << euler_final[0] * 180.0 / M_PI
              << " deg), Pitch: " << euler_final[1]
              << " rad (" << euler_final[1] * 180.0 / M_PI << " deg)"
              << std::endl;
    std::cerr << "Avg iteration time: " << avg_dt * 1000.0
              << " ms, Freq: " << freq << " Hz" << std::endl;
    std::cerr << "Quaternion norm^2: " << q.norm_sq() << std::endl;

    return 0;
}