/**
 * @file attitude.h
 * @brief Attitude estimation types and declarations
 */
#ifndef ATTITUDE_H
#define ATTITUDE_H

#include <cmath>
#include <vector>
#include <cassert>
#include "imu_sim.h"

struct Quaternion {
    double w;   ///< Scalar part
    double x;   ///< Vector part x
    double y;   ///< Vector part y
    double z;   ///< Vector part z

    Quaternion() : w(1.0), x(0.0), y(0.0), z(0.0) {}
    Quaternion(double w, double x, double y, double z)
        : w(w), x(x), y(y), z(z) {}

    static Quaternion identity() { return Quaternion(1.0, 0.0, 0.0, 0.0); }

    static Quaternion from_euler(double roll, double pitch, double yaw) {
        double cr = std::cos(roll / 2.0);
        double sr = std::sin(roll / 2.0);
        double cp = std::cos(pitch / 2.0);
        double sp = std::sin(pitch / 2.0);
        double cy = std::cos(yaw / 2.0);
        double sy = std::sin(yaw / 2.0);
        double q_w = cy * cp * cr + sy * sp * sr;
        double q_x = cy * cp * sr - sy * sp * cr;
        double q_y = sy * cp * sr + cy * sp * cr;
        double q_z = sy * cp * cr - cy * sp * sr;
        return Quaternion(q_w, q_x, q_y, q_z);
    }

    Quaternion& normalize() {
        double norm = std::sqrt(w * w + x * x + y * y + z * z);
        if (norm > 0.0) {
            double inv_norm = 1.0 / norm;
            w *= inv_norm;
            x *= inv_norm;
            y *= inv_norm;
            z *= inv_norm;
        }
        return *this;
    }

    bool is_normalized(double tolerance = 1e-6) const {
        double norm_sq = w * w + x * x + y * y + z * z;
        return std::abs(norm_sq - 1.0) < tolerance;
    }

    double norm_sq() const { return w * w + x * x + y * y + z * z; }

    std::vector<double> to_euler() const {
        double sinr_cosp = 2.0 * (w * x + y * z);
        double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
        double roll = std::atan2(sinr_cosp, cosr_cosp);
        double sinp = 2.0 * (w * y - z * x);
        if (std::abs(sinp) > 1.0) { sinp = std::copysign(1.0, sinp); }
        double pitch = std::asin(sinp);
        double siny_cosp = 2.0 * (w * z + x * y);
        double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
        double yaw = std::atan2(siny_cosp, cosy_cosp);
        return {roll, pitch, yaw};
    }

    Quaternion operator*(const Quaternion& other) const {
        return Quaternion(
            w * other.w - x * other.x - y * other.y - z * other.z,
            w * other.x + x * other.w + y * other.z - z * other.x,
            w * other.y - x * other.z + y * other.w + z * other.y,
            w * other.z + x * other.y - y * other.x + z * other.w);
    }

    Quaternion integrate_gyro(double wx, double wy, double wz,
                              double dt) const {
        double half_dt = 0.5 * dt;
        double q_x = x;
        double q_y = y;
        double q_z = z;
        double d_w = -half_dt * (wx * q_x + wy * q_y + wz * q_z);
        double d_x = half_dt * (wx * w + wy * q_z - wz * q_y);
        double d_y = half_dt * (-wx * q_z + wy * w + wz * q_x);
        double d_z = half_dt * (wx * q_y - wy * q_x + wz * w);
        return Quaternion(w + d_w, x + d_x, y + d_y, z + d_z).normalize();
    }
};

class ComplementaryFilter {
public:
    ComplementaryFilter(double alpha = 0.98, double dt = 0.1)
        : alpha_(alpha), dt_(dt), q_(Quaternion::identity()) {
        assert(alpha >= 0.0 && alpha <= 1.0);
        assert(dt > 0.0);
    }

    Quaternion update(double wx, double wy, double wz,
                      double ax, double ay, double az) {
        Quaternion q_gyro = q_.integrate_gyro(wx, wy, wz, dt_);
        double roll_accel = std::atan2(ay, std::sqrt(az * az + 1e-10));
        double pitch_accel = std::atan2(-ax, std::sqrt(ay * ay + az * az + 1e-10));
        Quaternion q_accel(
            std::cos(roll_accel / 2.0) * std::cos(pitch_accel / 2.0),
            std::sin(roll_accel / 2.0) * std::cos(pitch_accel / 2.0),
            std::cos(roll_accel / 2.0) * std::sin(pitch_accel / 2.0),
            std::sin(roll_accel / 2.0) * std::sin(pitch_accel / 2.0));
        q_accel.normalize();
        auto euler_gyro = q_gyro.to_euler();
        auto euler_accel = q_accel.to_euler();
        double roll_blended = alpha_ * euler_gyro[0] + (1.0 - alpha_) * euler_accel[0];
        double pitch_blended = alpha_ * euler_gyro[1] + (1.0 - alpha_) * euler_accel[1];
        double yaw_blended = alpha_ * euler_gyro[2];
        q_ = Quaternion::from_euler(roll_blended, pitch_blended, yaw_blended);
        return q_;
    }

    Quaternion get_attitude() const { return q_; }
    std::vector<double> get_euler() const { return q_.to_euler(); }
    void reset() { q_ = Quaternion::identity(); }
    void set_alpha(double alpha) { assert(alpha >= 0.0 && alpha <= 1.0); alpha_ = alpha; }
    void set_dt(double dt) { assert(dt > 0.0); dt_ = dt; }

private:
    double alpha_;
    double dt_;
    Quaternion q_;
};

#endif  // ATTITUDE_H