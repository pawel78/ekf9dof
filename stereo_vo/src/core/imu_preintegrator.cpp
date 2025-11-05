/**
 * @file imu_preintegrator.cpp
 * @brief Implementation of IMU preintegration
 */

#include "stereo_vo/core/imu_preintegrator.hpp"
#include <cmath>

namespace stereo_vo {
namespace core {

IMUPreintegrator::IMUPreintegrator(const Config& config)
    : config_(config),
      gyro_bias_(Eigen::Vector3d::Zero()),
      accel_bias_(Eigen::Vector3d::Zero()) {
}

void IMUPreintegrator::addMeasurement(const ImuMeasurement& meas) {
    measurements_.push_back(meas);
    
    // Keep only recent measurements (last 10 seconds)
    while (!measurements_.empty() &&
           measurements_.back().timestamp - measurements_.front().timestamp > 10.0) {
        measurements_.pop_front();
    }
}

bool IMUPreintegrator::preintegrate(double t_start, double t_end,
                                   PreintegratedResult& result) {
    // Find measurements in the time range
    std::vector<ImuMeasurement> range_meas;
    for (const auto& meas : measurements_) {
        if (meas.timestamp >= t_start && meas.timestamp <= t_end) {
            range_meas.push_back(meas);
        }
    }
    
    if (range_meas.size() < 2) {
        return false;
    }
    
    // Initialize result
    result.delta_t = t_end - t_start;
    result.delta_R = Eigen::Matrix3d::Identity();
    result.delta_v = Eigen::Vector3d::Zero();
    result.delta_p = Eigen::Vector3d::Zero();
    result.covariance = Eigen::Matrix<double, 9, 9>::Zero();
    
    // Integrate using midpoint method
    for (size_t i = 0; i < range_meas.size() - 1; ++i) {
        integrateStep(range_meas[i], range_meas[i + 1], result);
    }
    
    return true;
}

void IMUPreintegrator::integrateStep(const ImuMeasurement& meas1,
                                    const ImuMeasurement& meas2,
                                    PreintegratedResult& result) {
    double dt = meas2.timestamp - meas1.timestamp;
    if (dt <= 0) return;
    
    // Remove biases
    Eigen::Vector3d w = (meas1.angular_velocity + meas2.angular_velocity) * 0.5 - gyro_bias_;
    Eigen::Vector3d a = (meas1.linear_acceleration + meas2.linear_acceleration) * 0.5 - accel_bias_;
    
    // Integrate rotation
    double theta = w.norm() * dt;
    Eigen::Matrix3d dR = Eigen::Matrix3d::Identity();
    if (theta > 1e-8) {
        Eigen::Vector3d axis = w.normalized();
        // Rodrigues' formula
        Eigen::Matrix3d K;
        K << 0, -axis.z(), axis.y(),
             axis.z(), 0, -axis.x(),
             -axis.y(), axis.x(), 0;
        dR = Eigen::Matrix3d::Identity() + std::sin(theta) * K +
             (1.0 - std::cos(theta)) * K * K;
    }
    
    // Update integrated values
    Eigen::Vector3d acc_world = result.delta_R * a;
    result.delta_p += result.delta_v * dt + 0.5 * (acc_world + config_.gravity) * dt * dt;
    result.delta_v += (acc_world + config_.gravity) * dt;
    result.delta_R = result.delta_R * dR;
}

void IMUPreintegrator::setBiases(const Eigen::Vector3d& gyro_bias,
                                const Eigen::Vector3d& accel_bias) {
    gyro_bias_ = gyro_bias;
    accel_bias_ = accel_bias;
}

void IMUPreintegrator::reset() {
    measurements_.clear();
}

} // namespace core
} // namespace stereo_vo
