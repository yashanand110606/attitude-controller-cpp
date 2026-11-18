/**
 * @file attitude.cpp
 * @brief Complementary filter for attitude estimation using IMU data
 *
 * Implements a complementary filter that fuses gyro and accelerometer data
 * to estimate vehicle attitude. The filter uses a weighted average where:
 * - Gyro provides high-frequency response but drifts over time
 * - Accelerometer provides low-frequency reference but is noisy
 *
 * The filter constant alpha = 0.98 gives 98% gyro / 2% accelerometer weighting.
 */

#include <iostream>
#include <cmath>
#include <vector>
#include <complex>
#include <algorithm>
#include <chrono>
#include <thread>
#include <iomanip>
#include <string>
#include <cassert>

/**
 * @struct Quaternion
 * @brief Represents a unit quaternion for attitude representation
 */
struct Quaternion {
    double w; ///< Scalar part
    double x; ///< Vector part x
    double y; ///< Vector part y
    double z; ///< Vector part z

    /**
     * @brief Default constructor (identity quaternion)
     */
    Quaternion() : w(1.0), x(0.0), y(0.0), z(0.0) {}

    /**
     * @brief Construct from components
     * @param w Scalar part
     * @param x Vector part x
     * @param y Vector part y
     * @param z Vector part z
     */
    Quaternion(double w, double x, double y, double z)
        : w(w), x(x), y(y), z(z) {}

    /**
     * @brief Normalize the quaternion to unit length
     * @return Normalized quaternion (this)
     */
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

    /**
     * @brief Check if quaternion is normalized (within tolerance)
     * @param tolerance Maximum allowed deviation from 1.0
     * @return True if ||q|| ≈ 1 within tolerance
     */
    bool is_normalized(double tolerance = 1e-6) const {
        double norm_sq = w * w + x * x + y * y + z * z;
        return std::abs(norm_sq - 1.0) < tolerance;
    }

    /**
     * @brief Get the norm squared of the quaternion
     * @return Norm squared
     */
    double norm_sq() const {
        return w * w + x * x + y * y + z * z;
    }

    /**
     * @brief Convert quaternion to Euler angles (roll, pitch, yaw)
     * @return Euler angles in radians
     */
    std::vector<double> to_euler() const {
        // roll (x-axis rotation)
        double sinr_cosp = 2.0 * (w * x + y * z);
        double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
        double roll = std::atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        double sinp = 2.0 * (w * y - z * x);
        // Clamp to avoid domain errors in asin
        if (std::abs(sinp) > 1.0) {
            sinp = std::copysign(1.0, sinp);
        }
        double pitch = std::asin(sinp);

        // yaw (z-axis rotation)
        double siny_cosp = 2.0 * (w * z + x * y);
        double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
        double yaw = std::atan2(siny_cosp, cosy_cosp);

        return {roll, pitch, yaw};
    }

    /**
     * @brief Multiply two quaternions: q = p * q (rotate by p then by q)
     * @param other Quaternion to multiply by
     * @return Resulting quaternion
     */
    Quaternion operator*(const Quaternion& other) const {
        return Quaternion(
            w * other.w - x * other.x - y * other.y - z * other.z,
            w * other.x + x * other.w + y * other.z - z * other.x,
            w * other.y - x * other.z + y * other.w + z * other.y,
            w * other.z + x * other.y - y * other.x + z * other.w
        );
    }

    /**
     * @brief Integrate gyro measurement to update quaternion
     * @param omega Angular velocity vector (wx, wy, wz) in rad/s
     * @param dt Time step (s)
     * @return Updated quaternion
     */
    Quaternion integrate_gyro(double wx, double wy, double wz, double dt) const {
        // Quaternion derivative: q_dot = 0.5 * omega_q * q
        // where omega_q = (0, wx, wy, wz)
        double half_dt = 0.5 * dt;

        // omega * q product (pure quaternion part)
        double q_x = x, q_y = y, q_z = z;

        // dq = 0.5 * dt * (0 * w - wx * q_x - wy * q_y - wz * q_z,
        //                  0 * q_x + wx * w + wy * q_z - wz * q_y,
        //                  0 * q_y - wx * q_z + wy * w + wz * q_x,
        //                  0 * q_z + wx * q_y - wy * q_x + wz * w)
        double d_w = -half_dt * (wx * q_x + wy * q_y + wz * q_z);
        double d_x = half_dt * (wx * w + wy * q_z - wz * q_y);
        double d_y = half_dt * (-wx * q_z + wy * w + wz * q_x);
        double d_z = half_dt * (wx * q_y - wy * q_x + wz * w);

        return Quaternion(
            w + d_w,
            x + d_x,
            y + d_y,
            z + d_z
        ).normalize();
    }
};

/**
 * @struct AccelReading
 * @brief Accelerometer reading with gravity reference
 */
struct AccelReading {
    double ax; ///< Specific force x (m/s^2)
    double ay; ///< Specific force y (m/s^2)
    double az; ///< Specific force z (m/s^2)
    double roll_from_accel; ///< Roll angle from accelerometer (rad)
    double pitch_from_accel; ///< Pitch angle from accelerometer (rad)
};

/**
 * @brief Compute roll and pitch from accelerometer data
 * @param accel Accelerometer reading (m/s^2)
 * @param g Gravitational acceleration (m/s^2), default 9.81
 * @return Roll and pitch angles from accelerometer
 */
AccelReading compute_accel_angles(const AccelReading& accel, double g = 9.81) {
    AccelReading result = accel;

    // Roll: atan2(accel_y, accel_z) — rotation about x-axis
    result.roll_from_accel = std::atan2(accel.ay, std::sqrt(accel.az * accel.az + 1e-10));

    // Pitch: atan2(-accel_x, sqrt(accel_y^2 + accel_z^2)) — rotation about y-axis
    result.pitch_from_accel = std::atan2(-accel.ax, std::sqrt(accel.ay * accel.ay + accel.az * accel.az + 1e-10));

    return result;
}

/**
 * @class ComplementaryFilter
 * @brief Complementary filter for attitude estimation
 *
 * Combines gyro integration (high-frequency, drifting) with accelerometer
 * reference (low-frequency, noisy) to estimate stable attitude.
 */
class ComplementaryFilter {
public:
    /**
     * @brief Construct a complementary filter
     * @param alpha Filter constant [0, 1]; alpha = 0.98 means 98% gyro, 2% accel
     * @param dt Time step (s), default 0.1s (10Hz)
     */
    ComplementaryFilter(double alpha = 0.98, double dt = 0.1)
        : alpha_(alpha), dt_(dt), q_(Quaternion::identity()) {
        assert(alpha >= 0.0 && alpha <= 1.0);
        assert(dt > 0.0);
    }

    /**
     * @brief Update the filter with new gyro and accelerometer data
     * @param wx Angular velocity x (rad/s)
     * @param wy Angular velocity y (rad/s)
     * @param wz Angular velocity z (rad/s)
     * @param ax Accelerometer x (m/s^2)
     * @param ay Accelerometer y (m/s^2)
     * @param az Accelerometer z (m/s^2)
     * @return Updated quaternion estimate
     */
    Quaternion update(double wx, double wy, double wz,
                      double ax, double ay, double az) {
        // 1. Integrate gyro to predict next state
        Quaternion q_gyro = q_.integrate_gyro(wx, wy, wz, dt_);

        // 2. Compute roll/pitch from accelerometer
        AccelReading accel_readings;
        accel_readings.ax = ax;
        accel_readings.ay = ay;
        accel_readings.az = az;
        compute_accel_angles(accel_readings);

        // 3. Create reference quaternion from accelerometer
        // For roll: rotation about x-axis, pitch: rotation about y-axis
        // We use a simple approach: convert accel angles to a rotation quaternion
        double roll_accel = accel_readings.roll_from_accel;
        double pitch_accel = accel_readings.pitch_from_accel;

        // Reference quaternion from accelerometer (identity when level)
        Quaternion q_accel;
        q_accel = Quaternion(
            std::cos(roll_accel / 2.0) * std::cos(pitch_accel / 2.0),
            std::sin(roll_accel / 2.0) * std::cos(pitch_accel / 2.0),
            std::cos(roll_accel / 2.0) * std::sin(pitch_accel / 2.0),
            std::sin(roll_accel / 2.0) * std::sin(pitch_accel / 2.0)
        ).normalize();

        // 4. Complementary filter: blend gyro and accelerometer
        // q_new = alpha * q_gyro + (1 - alpha) * q_accel (as quaternion blend)
        // We use spherical linear interpolation concept:
        // q_combined = normalize(alpha * q_gyro_vector + (1-alpha) * q_accel_vector)
        // But quaternions don't add linearly. Instead, we use:
        // q_new = q_gyro * (q_accel * q_gyro.inverse())^((1-alpha)) blended

        // Simpler approach: weighted average of the rotation vectors
        // Extract Euler angles from both, blend, then convert back

        auto euler_gyro = q_gyro.to_euler();
        auto euler_accel = q_accel.to_euler();

        // Blend Euler angles
        double roll_blended = alpha_ * euler_gyro[0] + (1.0 - alpha_) * euler_accel[0];
        double pitch_blended = alpha_ * euler_gyro[1] + (1.0 - alpha_) * euler_accel[1];
        double yaw_blended = alpha_ * euler_gyro[2]; // Keep yaw from gyro only

        // Convert blended Euler angles back to quaternion
        q_ = Quaternion::from_euler(roll_blended, pitch_blended, yaw_blended);

        return q_;
    }

    /**
     * @brief Get the current attitude estimate
     * @return Current quaternion
     */
    Quaternion get_attitude() const {
        return q_;
    }

    /**
     * @brief Get current attitude as Euler angles
     * @return Euler angles (roll, pitch, yaw) in radians
     */
    std::vector<double> get_euler() const {
        return q_.to_euler();
    }

    /**
     * @brief Reset the filter state
     */
    void reset() {
        q_ = Quaternion::identity();
    }

    /**
     * @brief Set the filter constant alpha
     * @param alpha New alpha value [0, 1]
     */
    void set_alpha(double alpha) {
        assert(alpha >= 0.0 && alpha <= 1.0);
        alpha_ = alpha;
    }

    /**
     * @brief Set the time step
     * @param dt New time step (s)
     */
    void set_dt(double dt) {
        assert(dt > 0.0);
        dt_ = dt;
    }

private:
    /** Filter constant: [0,1], default 0.98 */
    double alpha_;

    /** Time step in seconds, default 0.1s (10Hz) */
    double dt_;

    /** Current attitude estimate */
    Quaternion q_;

    /**
     * @brief Create quaternion from Euler angles (ZYX convention: yaw * pitch * roll)
     * @param roll Roll angle about x-axis (rad)
     * @param pitch Pitch angle about y-axis (rad)
     * @param yaw Yaw angle about z-axis (rad)
     * @return Quaternion
     */
    static Quaternion from_euler(double roll, double pitch, double yaw) {
        // Individual axis rotations (intrinsic Tait-Bryan, ZYX)
        // R_x(roll) = [1, 0, 0; 0, cr, -sr; 0, sr, cr]
        // R_y(pitch) = [cp, 0, sp; 0, 1, 0; -sp, 0, cp]
        // R_z(yaw) = [cy, -sy, 0; sy, cy, 0; 0, 0, 1]

        double cr = std::cos(roll / 2.0);
        double sr = std::sin(roll / 2.0);
        double cp = std::cos(pitch / 2.0);
        double sp = std::sin(pitch / 2.0);
        double cy = std::cos(yaw / 2.0);
        double sy = std::sin(yaw / 2.0);

        // Combined: R = R_z(yaw) * R_y(pitch) * R_x(roll)
        // As quaternion: q = [w, x, y, z]
        double w = cy * cp * cr + sy * sp * sr;
        double x = cy * cp * sr - sy * sp * cr;
        double y = sy * cp * sr + cy * sp * cr;
        double z = sy * cp * cr - cy * sp * sr;

        return Quaternion(w, x, y, z);
    }
};

/**
 * @brief Run a single control loop iteration at 10Hz
 * @param q Current attitude estimate
 * @param wx Gyro x (rad/s)
 * @param wy Gyro y (rad/s)
 * @param wz Gyro z (rad/s)
 * @param ax Accel x (m/s^2)
 * @param ay Accel y (m/s^2)
 * @param az Accel z (m/s^2)
 * @return Updated attitude estimate
 */
Quaternion control_loop_iteration(Quaternion q, double wx, double wy, double wz,
                                   double ax, double ay, double az) {
    // complementary filter update
    return ComplementaryFilter(0.98, 0.1).update(wx, wy, wz, ax, ay, az);
}

/**
 * @brief Main function: run complementary filter control loop
 * @param argc Argument count
 * @param argv Argument vector
 * @return Exit code
 */
int main(int argc, char* argv[]) {
    // Parse command line arguments
    int num_iterations = 100;    // Default: 100 iterations (10 seconds at 10Hz)
    double gyro_noise = 0.01;    // Default: 0.01 rad/s
    double accel_noise = 0.01;   // Default: 0.01 m/s^2
    double g = 9.81;             // Gravitational acceleration
    bool verbose = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help") {
            std::cout << "Usage: attitude_controller [--iterations N] [--gyro-noise STD] [--accel-noise STD]" << std::endl;
            std::cout << "  --iterations N: Number of control loop iterations (default: 100)" << std::endl;
            std::cout << "  --gyro-noise STD: Gyro noise standard deviation (default: 0.01 rad/s)" << std::endl;
            std::cout << "  --accel-noise STD: Accelerometer noise standard deviation (default: 0.01 m/s^2)" << std::endl;
            std::cout << "  --verbose: Print detailed output per iteration" << std::endl;
            return 0;
        } else if (arg == "--verbose") {
            verbose = true;
        } else if (arg == "--iterations" && i + 1 < argc) {
            num_iterations = std::stoi(argv[++i]);
        } else if (arg == "--gyro-noise" && i + 1 < argc) {
            gyro_noise = std::stod(argv[++i]);
        } else if (arg == "--accel-noise" && i + 1 < argc) {
            accel_noise = std::stod(argv[++i]);
        }
    }

    // Initialize simulated IMU data for orbital rotation
    IMUSimulator imu_sim;

    // Start with identity quaternion (vehicle level)
    Quaternion q;
    std::chrono::steady_clock::time_point loop_start;
    double iteration_time_total = 0.0;

    std::cerr << "Running complementary filter control loop..." << std::endl;
    std::cerr << "Iterations: " << num_iterations << ", DT: 0.1s (10Hz)" << std::endl;

    for (int i = 0; i < num_iterations; ++i) {
        auto iter_start = std::chrono::high_resolution_clock::now();

        // Generate IMU data for orbit scenario
        auto [gyro, accel] = imu_sim.generate_orbit_data(iteration_time_total);

        // Update complementary filter
        q = q.update(gyro.wx, gyro.wy, gyro.wz,
                     accel.ax, accel.ay, accel.az);

        auto iter_end = std::chrono::high_resolution_clock::now();
        double iter_dt = std::chrono::duration<double>(iter_end - iter_start).count();
        iteration_time_total += iter_dt;

        if (verbose) {
            auto euler = q.to_euler();
            std::cout << "Iter " << std::setw(3) << i
                      << " | wx=" << std::fixed << std::setprecision(4) << gyro.wx
                      << " wy=" << gyro.wy
                      << " wz=" << gyro.wz
                      << " | accel=(" << std::setprecision(2) << accel.ax
                      << ", " << accel.ay << ", " << accel.az << ")"
                      << " | roll=" << euler[0]
                      << " pitch=" << euler[1]
                      << " | iter_dt=" << iter_dt << std::endl;
        }

        // Sleep to maintain 10Hz loop rate
        double sleep_time = 0.1 - iter_dt;
        if (sleep_time > 0.0) {
            std::this_thread::sleep_for(std::chrono::duration<double>(sleep_time));
        }
    }

    // Print summary
    auto euler_final = q.to_euler();
    double avg_iter_time = iteration_time_total / num_iterations;
    double freq = 1.0 / avg_iter_time;

    std::cerr << "Final attitude - Roll: " << euler_final[0] << " rad ("
              << euler_final[0] * 180.0 / M_PI << " deg), "
              "Pitch: " << euler_final[1] << " rad ("
              << euler_final[1] * 180.0 / M_PI << " deg)" << std::endl;
    std::cerr << "Average iteration time: " << avg_iter_time * 1000.0 << " ms" << std::endl;
    std::cerr << "Effective loop frequency: " << freq << " Hz" << std::endl;
    std::cerr << "Quaternion norm: " << q.norm_sq() << " (should be ≈ 1.0)" << std::endl;

    return 0;
}