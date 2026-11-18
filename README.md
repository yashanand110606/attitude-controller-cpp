# C++ Attitude Controller with Complementary Filter

## Physics Overview

This project implements a complementary filter for attitude estimation in C++,
directly applicable to SpaceX flight software roles. The filter fuses gyroscope
and accelerometer data to estimate vehicle attitude (roll, pitch, yaw).

### Complementary Filter Equations

The complementary filter combines two sensor sources:

**Gyroscope integration (high-frequency, drifting):**
```
θ_gyro = θ_prev + ω * Δt
```
where ω is the angular velocity rad/s and Δt is the time step.

**Accelerometer reference (low-frequency, noisy):**
```
θ_accel = arcsin(accel_y / g)  // for roll
          arcsin(accel_x / g)  // for pitch
```

**Complementary filter blend:**
```
θ_estimate = α * θ_gyro + (1 - α) * θ_accel
```

where **α = 0.98** (98% gyro, 2% accelerometer weighting).

The high α value gives the gyroscope primary responsibility for short-term
attitude changes, while the accelerometer reference corrects long-term drift.

### Quaternion Mathematics

Attitude is represented using **unit quaternions** to avoid gimbal lock and
provide smooth interpolation:

- **Quaternion multiplication** composes rotations: `q = q1 * q2`
- **Gyro integration** updates: `q_new = q * exp(0.5 * ω * Δt)`
- **Normalization** enforces unit constraint: `||q|| = 1`
- **Euler conversion**: `q → (roll, pitch, yaw)` via standard formulas

The complementary filter blends gyro and accelerometer data in quaternion space
to maintain unit normalization throughout.

## Architecture

```
src/           ← Source code (IMU simulation, attitude control)
tests/         ← Unit tests (Catch2)
build/         ← CMake build directory
.gitignore     ← Standard C++ ignores
CMakeLists.txt ← Build configuration
README.md      ← This file
requirements.txt ← Dependencies
```

## Build Instructions

```bash
# 1. Create build directory and configure CMake
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug

# 2. Build the project
make

# 3. Run all unit tests
./build/tests
```

## Run Instructions

```bash
# Build and run tests
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug
make
./attitude_controller  # Runs the main control loop
./tests/test_controller  # Runs controller stability tests
```

### Sample Output Description

When running `./build/tests`, you should see all 5+ tests pass:

- **IMU_Simulation::GaussianNoiseStats**: Verifies generated noise has mean ≈ 0 and std ≈ 0.01
- **ComplementaryFilter::ConvergesToAccelReading**: Filter converges within 2s of constant input
- **ComplementaryFilter::RejectsGyroBias**: Estimate remains bounded with biased gyro
- **Controller::StabilizesToIdentity**: System converges to identity attitude from 45/30 degree initial state

Passing output indicates healthy operation:
```
[PASS] IMU_Simulation/GaussianNoiseStats
[PASS] ComplementaryFilter/ConvergesToAccelReading
[PASS] ComplementaryFilter/RejectsGyroBias
[PASS] Controller/StabilizesToIdentity
```

## Future Improvements

- Add magnetic field model for yaw reference using star tracker data
- Integrate with actual SpaceX sensor suites
- Add attitude control saturation logic for thruster limits
- Implement rate-limited control commands
- Add Python wrapper for ROS 2 integration

## How to Extend

- **Solar radiation pressure model**: Add perturbing acceleration terms
- **Attitude control saturation**: Limit control output to thruster capabilities
- **Multi-rate operation**: Support inner loop at 100Hz, outer loop at 10Hz
- **Sensor fusion**: Add magnetometer for absolute heading reference
- **Quaternion rate integration**: Implement more sophisticated integration schemes

## License

MIT License - see LICENSE file at root.