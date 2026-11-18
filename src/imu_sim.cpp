/**
 * @file imu_sim.cpp
 * @brief IMU simulation generator for attitude controller testing
 *
 * Generates realistic IMU data with gyro and accelerometer readings,
 * including Gaussian noise and bias instability.
 *
 * Physics: Uses publicly available gravitational and rotational models.
 * Noise models: Gaussian distributions with specified standard deviations.
 */

#include <iostream>
#include <fstream>
#include <cmath>
#include <random>
#include <chrono>
#include <vector>
#include <iomanip>
#include <string>

/**
 * @struct GyroData
 * @brief 3-axis gyroscope reading with bias
 */
struct GyroData {
    double wx; ///< Angular velocity x (rad/s)
    double wy; ///< Angular velocity y (rad/s)
    double wz; ///< Angular velocity z (rad/s)
    double bias_x; ///< Gyro bias x (rad/s)
    double bias_y; ///< Gyro bias y (rad/s)
    double bias_z; ///< Gyro bias z (rad/s)
};

/**
 * @struct AccelData
 * @brief 3-axis accelerometer reading with noise
 */
struct AccelData {
    double ax; ///< Specific force x (m/s^2)
    double ay; ///< Specific force y (m/s^2)
    double az; ///< Specific force z (m/s^2)
    double noise_x; ///< Accelerometer noise x (m/s^2)
    double noise_y; ///< Accelerometer noise y (m/s^2)
    double noise_z; ///< Accelerometer noise z (m/s^2)
};

/**
 * @class IMUSimulator
 * @brief Simulates an IMU with gyro and accelerometer data
 */
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
        // Initialize noise distributions
        gyro_dist_ = std::normal_distribution<double>(0.0, gyro_noise_std_);
        accel_dist_ = std::normal_distribution<double>(0.0, accel_noise_std_);
    }

    /**
     * @brief Generate a gyro reading with noise and bias
     * @param true_omega True angular velocity (rad/s)
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
     * @param true_accel True specific force (m/s^2)
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
     * @return GyroData and AccelData for circular orbit at altitude 400km
     */
    std::pair<GyroData, AccelData> generate_orbit_data(double time) {
        // Circular orbit at ~400km altitude, ~7.66 rad/s orbital rate
        double orbital_rate = 7.66; // rad/s

        // Rotation angle
        double theta = orbital_rate * time;

        // True angular velocity (body frame for gravity-gradient stabilization)
        double wx_true = 0.0;
        double wy_true = 0.0;
        double wz_true = orbital_rate; // rotation about z-axis

        // True specific force in body frame (gravity toward Earth center)
        double rx = -std::cos(theta) * 9.81; // position x component * g
        double ry = -std::sin(theta) * 9.81; // position y component * g
        double rz = 0.0;

        // Accelerometer reads gravity vector
        GyroData gyro = generate_gyro(wx_true, wy_true, wz_true);
        AccelData accel = generate_accel(rx, ry, rz);

        return {gyro, accel};
    }

    /**
     * @brief Write header to binary output file
     * @param filename Output file path
     */
    void write_header(const std::string& filename) {
        std::ofstream file(filename, std::ios::binary);
        if (!file.is_open()) {
            std::cerr << "Error: Could not open file " << filename << std::endl;
            return;
        }
        // Write header: number of fields (6 for gyro + 6 for accel = 12)
        uint32_t num_fields = 12;
        file.write(reinterpret_cast<const char*>(&num_fields), sizeof(uint32_t));
        file.close();
    }

    /**
     * @brief Append a data point to binary output file
     * @param filename Output file path
     * @param timestamp Current timestamp (s)
     * @param gyro Gyro data
     * @param accel Accel data
     */
    void write_data(const std::string& filename, double timestamp,
                    const GyroData& gyro, const AccelData& accel) {
        std::ofstream file(filename, std::ios::binary | std::ios::app);
        if (!file.is_open()) {
            std::cerr << "Error: Could not open file " << filename << std::endl;
            return;
        }

        // Write: timestamp (double) + wx, wy, wz, bias_x, bias_y, bias_z (6 doubles) +
        //        ax, ay, az, noise_x, noise_y, noise_z (6 doubles) = 13 doubles total
        double values[13] = {timestamp,
                            gyro.wx, gyro.wy, gyro.wz, gyro.bias_x, gyro.bias_y, gyro.bias_z,
                            accel.ax, accel.ay, accel.az, accel.noise_x, accel.noise_y, accel.noise_z};

        file.write(reinterpret_cast<const char*>(values), sizeof(values));
        file.close();
    }

    /**
     * @brief Print a data point to stdout
     * @param timestamp Current timestamp (s)
     * @param gyro Gyro data
     * @param accel Accel data
     */
    void print_data(double timestamp, const GyroData& gyro, const AccelData& accel) {
        std::cout << std::fixed << std::setprecision(6);
        std::cout << timestamp << " "
                  << gyro.wx << " " << gyro.wy << " " << gyro.wz << " "
                  << accel.ax << " " << accel.ay << " " << accel.az << std::endl;
    }

private:
    double gyro_noise_std_;      ///< Gyro noise standard deviation (rad/s)
    double accel_noise_std_;     ///< Accelerometer noise standard deviation (m/s^2)
    double bias_instability_;    ///< Gyro bias instability (rad/s)
    double dt_;                  ///< Time step (s)

    std::mt19937 rng_;           ///< Mersenne Twister random number generator
    std::normal_distribution<double> gyro_dist_; ///< Gyro noise distribution
    std::normal_distribution<double> accel_dist_; ///< Accelerometer noise distribution

    // Helper to generate real-valued uniform [0,1] for bias
    double gen_real() {
        std::uniform_real_distribution<double> unif(0.0, 1.0);
        return unif(rng_);
    }
};

/**
 * @brief Main function: generate and output IMU data
 * @param argc Argument count
 * @param argv Argument vector
 * @return Exit code
 */
int main(int argc, char* argv[]) {
    // Parse command line arguments
    int num_samples = 100;     // Default: 100 samples
    double duration = 10.0;    // Default: 10 seconds
    std::string output_file;   // Default: stdout

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--samples" && i + 1 < argc) {
            num_samples = std::stoi(argv[++i]);
        } else if (arg == "--duration" && i + 1 < argc) {
            duration = std::stod(argv[++i]);
        } else if (arg == "--output" && i + 1 < argc) {
            output_file = argv[++i];
        } else if (arg == "--help") {
            std::cout << "Usage: imu_sim [--samples N] [--duration T] [--output file.bin]" << std::endl;
            std::cout << "  --samples N: Number of samples to generate (default: 100)" << std::endl;
            std::cout << "  --duration T: Total duration in seconds (default: 10)" << std::endl;
            std::cout << "  --output file.bin: Write binary output to file (default: stdout)" << std::endl;
            return 0;
        }
    }

    // Initialize simulator
    IMUSimulator sim;

    // Open output file if specified
    std::ofstream out_file;
    if (!output_file.empty()) {
        out_file.open(output_file, std::ios::binary);
        if (!out_file.is_open()) {
            std::cerr << "Error: Could not open output file " << output_file << std::endl;
            return 1;
        }
        // Write binary header
        sim.write_header(output_file);
    }

    // Generate data
    auto start_time = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < num_samples; ++i) {
        double time = (i * duration) / num_samples;
        auto [gyro, accel] = sim.generate_orbit_data(time);
        double timestamp = time;

        if (!output_file.empty()) {
            sim.write_data(output_file, timestamp, gyro, accel);
        } else {
            sim.print_data(timestamp, gyro, accel);
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    double elapsed = std::chrono::duration<double>(end_time - start_time).count();

    if (!output_file.empty()) {
        out_file.close();
    }

    std::cerr << "Generated " << num_samples << " IMU samples in "
              << elapsed << " seconds" << std::endl;

    return 0;
}