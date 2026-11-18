/**
 * @file imu_sim.h
 * @brief IMU simulation types and declarations
 */
#ifndef IMU_SIM_H
#define IMU_SIM_H

#include <chrono>
#include <cmath>
#include <random>
#include <utility>
#include <vector>

struct GyroData {
    double wx;
    double wy;
    double wz;
    double bias_x;
    double bias_y;
    double bias_z;
};

struct AccelData {
    double ax;
    double ay;
    double az;
    double noise_x;
    double noise_y;
    double noise_z;
};

class IMUSimulator {
public:
    /**
     * @brief Construct an IMU simulator
     * @param gyro_noise_std Standard deviation of gyro noise (rad/s)
     * @param accel_noise_std Standard deviation of accelerometer noise (m/s^2)
     * @param bias_instability Gyro bias instability (rad/s)
     * @param dt Time step (s)
     */
    IMUSimulator(double gyro_noise_std = 0.01,
                 double accel_noise_std = 0.01,
                 double bias_instability = 0.001,
                 double dt = 0.1)
        : gyro_noise_std_(gyro_noise_std),
          accel_noise_std_(accel_noise_std),
          bias_instability_(bias_instability),
          dt_(dt),
          rng_(std::chrono::steady_clock::now().time_since_epoch().count()) {
        gyro_dist_ = std::normal_distribution<double>(0.0, gyro_noise_std_);
        accel_dist_ = std::normal_distribution<double>(0.0, accel_noise_std_);
    }

    /**
     * @brief Generate a gyro reading with noise and bias
     * @param wx_true True angular velocity x (rad/s)
     * @param wy_true True angular velocity y (rad/s)
     * @param wz_true True angular velocity z (rad/s)
     * @return GyroData with noise and bias
     */
    GyroData generate_gyro(double wx_true, double wy_true, double wz_true) {
        GyroData data;
        data.wx = wx_true + gyro_dist_(rng_) + bias_instability_ * (gen_real() - 0.5);
        data.wy = wy_true + gyro_dist_(rng_) + bias_instability_ * (gen_real() - 0.5);
        data.wz = wz_true + gyro_dist_(rng_) + bias_instability_ * (gen_real() - 0.5);
        data.bias_x = bias_instability_;
        data.bias_y = bias_instability_;
        data.bias_z = bias_instability_;
        return data;
    }

    /**
     * @brief Generate an accelerometer reading with noise
     * @param ax_true True specific force x (m/s^2)
     * @param ay_true True specific force y (m/s^2)
     * @param az_true True specific force z (m/s^2)
     * @return AccelData with noise
     */
    AccelData generate_accel(double ax_true, double ay_true, double az_true) {
        AccelData data;
        data.ax = ax_true + accel_dist_(rng_);
        data.ay = ay_true + accel_dist_(rng_);
        data.az = az_true + accel_dist_(rng_);
        data.noise_x = accel_dist_(rng_);
        data.noise_y = accel_dist_(rng_);
        data.noise_z = accel_dist_(rng_);
        return data;
    }

    /**
     * @brief Generate IMU data for orbital rotation simulation
     * @param time Current time (s)
     * @return Pair of GyroData and AccelData
     */
    std::pair<GyroData, AccelData> generate_orbit_data(double time) {
        const double orbital_rate = 7.66;
        const double theta = orbital_rate * time;
        const double wx_true = 0.0;
        const double wy_true = 0.0;
        const double wz_true = orbital_rate;
        const double rx = -std::cos(theta) * 9.81;
        const double ry = -std::sin(theta) * 9.81;
        const double rz = 0.0;
        GyroData gyro = generate_gyro(wx_true, wy_true, wz_true);
        AccelData accel = generate_accel(rx, ry, rz);
        return {gyro, accel};
    }

private:
    double gyro_noise_std_;
    double accel_noise_std_;
    double bias_instability_;
    double dt_;
    std::mt19937 rng_;
    std::normal_distribution<double> gyro_dist_;
    std::normal_distribution<double> accel_dist_;

    double gen_real() {
        std::uniform_real_distribution<double> unif(0.0, 1.0);
        return unif(rng_);
    }
};

#endif  // IMU_SIM_H