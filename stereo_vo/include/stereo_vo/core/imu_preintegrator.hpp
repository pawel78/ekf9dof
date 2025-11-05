/**
 * @file imu_preintegrator.hpp
 * @brief IMU preintegration for visual-inertial odometry
 */

#pragma once

#include <Eigen/Dense>
#include <vector>
#include <deque>

namespace stereo_vo {
namespace core {

/**
 * @brief Preintegrate IMU measurements between keyframes
 */
class IMUPreintegrator {
public:
    struct Config {
        Eigen::Vector3d gyro_noise_density = Eigen::Vector3d(0.001, 0.001, 0.001);
        Eigen::Vector3d accel_noise_density = Eigen::Vector3d(0.01, 0.01, 0.01);
        Eigen::Vector3d gyro_bias_random_walk = Eigen::Vector3d(1e-5, 1e-5, 1e-5);
        Eigen::Vector3d accel_bias_random_walk = Eigen::Vector3d(1e-4, 1e-4, 1e-4);
        Eigen::Vector3d gravity = Eigen::Vector3d(0, 0, -9.81);
    };

    struct ImuMeasurement {
        double timestamp;
        Eigen::Vector3d angular_velocity;  ///< rad/s
        Eigen::Vector3d linear_acceleration; ///< m/s^2
    };

    struct PreintegratedResult {
        double delta_t;                    ///< Time difference
        Eigen::Matrix3d delta_R;           ///< Rotation change
        Eigen::Vector3d delta_v;           ///< Velocity change
        Eigen::Vector3d delta_p;           ///< Position change
        Eigen::Matrix<double, 9, 9> covariance;
    };

    explicit IMUPreintegrator(const Config& config = Config());
    ~IMUPreintegrator() = default;

    /**
     * @brief Add IMU measurement
     */
    void addMeasurement(const ImuMeasurement& meas);

    /**
     * @brief Preintegrate measurements between two timestamps
     * @param t_start Start timestamp
     * @param t_end End timestamp
     * @param result Output preintegrated result
     * @return Whether preintegration succeeded
     */
    bool preintegrate(double t_start, double t_end, PreintegratedResult& result);

    /**
     * @brief Set current biases
     */
    void setBiases(const Eigen::Vector3d& gyro_bias, const Eigen::Vector3d& accel_bias);

    /**
     * @brief Reset preintegrator
     */
    void reset();

private:
    Config config_;
    std::deque<ImuMeasurement> measurements_;
    Eigen::Vector3d gyro_bias_;
    Eigen::Vector3d accel_bias_;

    void integrateStep(const ImuMeasurement& meas1, const ImuMeasurement& meas2,
                      PreintegratedResult& result);
};

} // namespace core
} // namespace stereo_vo
