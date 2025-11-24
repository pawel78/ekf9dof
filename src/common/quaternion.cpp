#include "common/quaternion.hpp"
#include <cmath>

namespace ekf9dof {

QuaternionImpl::QuaternionImpl() : data_{0.0, 0.0, 0.0, 1.0} {
    // Identity quaternion [0, 0, 0, 1]
}

QuaternionImpl::QuaternionImpl(double x, double y, double z, double w) 
    : data_{x, y, z, w} {
    normalize();
}

double QuaternionImpl::norm() const {
    return std::sqrt(data_[0] * data_[0] + 
                     data_[1] * data_[1] + 
                     data_[2] * data_[2] + 
                     data_[3] * data_[3]);
}

void QuaternionImpl::normalize() {
    const double n = norm();
    if (n < 1e-12) {
        // If norm is too small, reset to identity
        data_[0] = 0.0;
        data_[1] = 0.0;
        data_[2] = 0.0;
        data_[3] = 1.0;
        return;
    }
    
    const double inv_n = 1.0 / n;
    data_[0] *= inv_n;
    data_[1] *= inv_n;
    data_[2] *= inv_n;
    data_[3] *= inv_n;
}

QuaternionImpl QuaternionImpl::normalized() const {
    QuaternionImpl result(*this);
    result.normalize();
    return result;
}

QuaternionImpl QuaternionImpl::conjugate() const {
    return QuaternionImpl(-data_[0], -data_[1], -data_[2], data_[3]);
}

QuaternionImpl QuaternionImpl::inverse() const {
    // For unit quaternions, inverse equals conjugate
    return conjugate();
}

QuaternionImpl QuaternionImpl::operator*(const QuaternionImpl& other) const {
    // Hamilton product: q1 * q2
    const double x1 = data_[0];
    const double y1 = data_[1];
    const double z1 = data_[2];
    const double w1 = data_[3];
    
    const double x2 = other.data_[0];
    const double y2 = other.data_[1];
    const double z2 = other.data_[2];
    const double w2 = other.data_[3];
    
    const double x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
    const double y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2;
    const double z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2;
    const double w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;
    
    return QuaternionImpl(x, y, z, w);
}

std::array<std::array<double, 3>, 3> QuaternionImpl::to_rotation_matrix() const {
    const double qx = data_[0];
    const double qy = data_[1];
    const double qz = data_[2];
    const double qw = data_[3];
    
    const double qx2 = qx * qx;
    const double qy2 = qy * qy;
    const double qz2 = qz * qz;
    const double qw2 = qw * qw;
    
    const double qxy = qx * qy;
    const double qxz = qx * qz;
    const double qxw = qx * qw;
    const double qyz = qy * qz;
    const double qyw = qy * qw;
    const double qzw = qz * qw;
    
    std::array<std::array<double, 3>, 3> R;
    
    // First row
    R[0][0] = qw2 + qx2 - qy2 - qz2;
    R[0][1] = 2.0 * (qxy - qzw);
    R[0][2] = 2.0 * (qxz + qyw);
    
    // Second row
    R[1][0] = 2.0 * (qxy + qzw);
    R[1][1] = qw2 - qx2 + qy2 - qz2;
    R[1][2] = 2.0 * (qyz - qxw);
    
    // Third row
    R[2][0] = 2.0 * (qxz - qyw);
    R[2][1] = 2.0 * (qyz + qxw);
    R[2][2] = qw2 - qx2 - qy2 + qz2;
    
    return R;
}

std::array<double, 3> QuaternionImpl::to_euler() const {
    // Convert quaternion to 321 Euler angles (roll, pitch, yaw)
    
    const double qx = data_[0];
    const double qy = data_[1];
    const double qz = data_[2];
    const double qw = data_[3];
    
    // 321 Euler angles from quaternion
    // For quaternion q = qRoll * qPitch * qYaw
    
    // Roll (φ) - rotation about X-axis  
    const double sinr_cosp = 2.0 * (qw * qx - qy * qz);
    const double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
    const double roll = std::atan2(sinr_cosp, cosr_cosp);
    
    // Pitch (θ) - rotation about Y-axis
    const double sinp = 2.0 * (qw * qy + qz * qx);
    double pitch;
    if (std::abs(sinp) >= 1.0) {
        // Gimbal lock case
        pitch = std::copysign(M_PI / 2.0, sinp);  // Use ±90° 
    } else {
        pitch = std::asin(sinp);
    }
    
    // Yaw (ψ) - rotation about Z-axis
    const double siny_cosp = 2.0 * (qw * qz - qx * qy);
    const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    const double yaw = std::atan2(siny_cosp, cosy_cosp);
    
    return {roll, pitch, yaw};
}

} // namespace ekf9dof
