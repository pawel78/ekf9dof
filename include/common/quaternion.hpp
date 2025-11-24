#pragma once

#include <cmath>
#include <stdexcept>
#include <array>

namespace ekf9dof {

/**
 * @brief Quaternion class for 3D rotations following NED frame conventions
 * 
 * This implementation follows:
 * - Hamilton convention: [x, y, z, w] with scalar component last
 * - 321 Euler angle sequence (yaw-pitch-roll / ZYX)
 * - NED (North-East-Down) navigation frame
 * - FRD (Forward-Right-Down) body frame
 * 
 * All quaternions are unit quaternions (norm = 1) representing rotations in SO(3).
 */
class QuaternionImpl {
public:
    /**
     * @brief Default constructor - creates identity quaternion [0, 0, 0, 1]
     */
    QuaternionImpl();

    /**
     * @brief Construct quaternion from components
     * @param x X component (imaginary)
     * @param y Y component (imaginary)
     * @param z Z component (imaginary)
     * @param w W component (scalar/real)
     * 
     * Note: Quaternion will be automatically normalized
     */
    QuaternionImpl(double x, double y, double z, double w);

    /**
     * @brief Construct from 4D array [x, y, z, w]
     * @param vec Array of 4 doubles
     */
    template<typename VectorType>
    explicit QuaternionImpl(const VectorType& vec);

    // Accessors
    double x() const { return data_[0]; }
    double y() const { return data_[1]; }
    double z() const { return data_[2]; }
    double w() const { return data_[3]; }

    /**
     * @brief Access component by index
     * @param i Index (0=x, 1=y, 2=z, 3=w)
     */
    double operator()(int i) const { return data_[i]; }

    /**
     * @brief Get coefficients as array
     */
    const std::array<double, 4>& coeffs() const { return data_; }

    /**
     * @brief Compute quaternion norm (magnitude)
     */
    double norm() const;

    /**
     * @brief Normalize quaternion to unit length
     */
    void normalize();

    /**
     * @brief Get normalized copy of quaternion
     */
    QuaternionImpl normalized() const;

    /**
     * @brief Quaternion conjugate (inverse rotation for unit quaternions)
     * 
     * For unit quaternion q = [x, y, z, w], conjugate is [-x, -y, -z, w]
     */
    QuaternionImpl conjugate() const;

    /**
     * @brief Quaternion inverse (same as conjugate for unit quaternions)
     */
    QuaternionImpl inverse() const;

    /**
     * @brief Rotate a 3D vector using this quaternion
     * 
     * Performs the rotation: v' = q * v * q^(-1)
     * 
     * @param v 3D vector to rotate (as array-like object with [](0), [](1), [](2))
     * @return Rotated vector as array [x, y, z]
     */
    template<typename VectorType>
    std::array<double, 3> rotate(const VectorType& v) const;

    /**
     * @brief Rotate a vector using the inverse of this quaternion
     * 
     * Performs: v' = q^(-1) * v * q
     * Equivalent to: conjugate().rotate(v)
     * 
     * @param v 3D vector to rotate
     * @return Rotated vector as array [x, y, z]
     */
    template<typename VectorType>
    std::array<double, 3> rotate_inverse(const VectorType& v) const;

    /**
     * @brief Quaternion multiplication (composition of rotations)
     * 
     * q1 * q2 represents: first rotate by q2, then by q1
     * 
     * @param other Other quaternion
     * @return Product quaternion
     */
    QuaternionImpl operator*(const QuaternionImpl& other) const;

    /**
     * @brief Convert quaternion to 3x3 rotation matrix
     * 
     * Returns a 3x3 array representing the rotation matrix
     * 
     * @return 3x3 rotation matrix as nested array
     */
    std::array<std::array<double, 3>, 3> to_rotation_matrix() const;

    /**
     * @brief Convert quaternion to Euler angles (321 sequence)
     * 
     * @return Euler angles [roll, pitch, yaw] in radians as array
     * 
     * Ranges:
     * - Roll:  [-π, π]
     * - Pitch: [-π/2, π/2]
     * - Yaw:   [-π, π]
     */
    std::array<double, 3> to_euler() const;

    /**
     * @brief Create quaternion from Euler angles using 321 sequence
     * 
     * @param euler Euler angles [roll, pitch, yaw] in radians (array-like with [](i))
     * @return Quaternion representing the rotation
     * 
     * Rotation sequence (applied in order):
     * 1. Yaw (ψ) about Z-axis
     * 2. Pitch (θ) about Y-axis
     * 3. Roll (φ) about X-axis
     * 
     * Resulting rotation matrix: R = Rx(φ) * Ry(θ) * Rz(ψ)
     */
    template<typename VectorType>
    static QuaternionImpl from_euler(const VectorType& euler);

    /**
     * @brief Create quaternion from rotation matrix
     * 
     * @param R 3x3 rotation matrix (nested array-like with [](i)[](j))
     * @return Quaternion representing the rotation
     */
    template<typename MatrixType>
    static QuaternionImpl from_rotation_matrix(const MatrixType& R);

private:
    std::array<double, 4> data_;  // [x, y, z, w]
};

// Template implementations

template<typename VectorType>
QuaternionImpl::QuaternionImpl(const VectorType& vec) 
    : data_{vec[0], vec[1], vec[2], vec[3]} {
    normalize();
}

template<typename VectorType>
std::array<double, 3> QuaternionImpl::rotate(const VectorType& v) const {
    // Rotate vector using: v' = q * v * q^(-1)
    // Using the formula: v' = v + 2*w*(q_xyz × v) + 2*q_xyz × (q_xyz × v)
    
    const double qx = data_[0];
    const double qy = data_[1];
    const double qz = data_[2];
    const double qw = data_[3];
    
    const double vx = v[0];
    const double vy = v[1];
    const double vz = v[2];
    
    // First cross product: q_xyz × v
    const double cx = qy * vz - qz * vy;
    const double cy = qz * vx - qx * vz;
    const double cz = qx * vy - qy * vx;
    
    // Second cross product: q_xyz × (q_xyz × v)
    const double ccx = qy * cz - qz * cy;
    const double ccy = qz * cx - qx * cz;
    const double ccz = qx * cy - qy * cx;
    
    // Final result: v + 2*w*(q_xyz × v) + 2*(q_xyz × (q_xyz × v))
    return {
        vx + 2.0 * (qw * cx + ccx),
        vy + 2.0 * (qw * cy + ccy),
        vz + 2.0 * (qw * cz + ccz)
    };
}

template<typename VectorType>
std::array<double, 3> QuaternionImpl::rotate_inverse(const VectorType& v) const {
    return conjugate().rotate(v);
}

template<typename VectorType>
QuaternionImpl QuaternionImpl::from_euler(const VectorType& euler) {
    // 321 Euler angle sequence: yaw-pitch-roll (ZYX)
    // euler = [roll, pitch, yaw]
    // Rotations applied in order: Yaw (Z), then Pitch (Y), then Roll (X)
    // Resulting matrix: R = Rx(roll) * Ry(pitch) * Rz(yaw)
    // Quaternion composition: q = qRoll * qPitch * qYaw
    
    const double roll = euler[0];
    const double pitch = euler[1];
    const double yaw = euler[2];
    
    // Half angles
    const double cr = std::cos(roll * 0.5);
    const double sr = std::sin(roll * 0.5);
    const double cp = std::cos(pitch * 0.5);
    const double sp = std::sin(pitch * 0.5);
    const double cy = std::cos(yaw * 0.5);
    const double sy = std::sin(yaw * 0.5);
    
    // Quaternion product: qRoll * qPitch * qYaw
    // qYaw = [0, 0, sin(yaw/2), cos(yaw/2)]
    // qPitch = [0, sin(pitch/2), 0, cos(pitch/2)]
    // qRoll = [sin(roll/2), 0, 0, cos(roll/2)]
    
    const double x = sr * cp * cy + cr * sp * sy;
    const double y = cr * sp * cy - sr * cp * sy;
    const double z = cr * cp * sy + sr * sp * cy;
    const double w = cr * cp * cy - sr * sp * sy;
    
    return QuaternionImpl(x, y, z, w);
}

template<typename MatrixType>
QuaternionImpl QuaternionImpl::from_rotation_matrix(const MatrixType& R) {
    // Shepperd's method for numerical stability
    // Choose the largest diagonal element to avoid division by small numbers
    
    const double trace = R[0][0] + R[1][1] + R[2][2];
    
    double x, y, z, w;
    
    if (trace > 0.0) {
        // w is the largest component
        const double s = std::sqrt(trace + 1.0) * 2.0;  // s = 4*w
        w = 0.25 * s;
        x = (R[2][1] - R[1][2]) / s;
        y = (R[0][2] - R[2][0]) / s;
        z = (R[1][0] - R[0][1]) / s;
    } else if ((R[0][0] > R[1][1]) && (R[0][0] > R[2][2])) {
        // x is the largest component
        const double s = std::sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]) * 2.0;  // s = 4*x
        w = (R[2][1] - R[1][2]) / s;
        x = 0.25 * s;
        y = (R[0][1] + R[1][0]) / s;
        z = (R[0][2] + R[2][0]) / s;
    } else if (R[1][1] > R[2][2]) {
        // y is the largest component
        const double s = std::sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]) * 2.0;  // s = 4*y
        w = (R[0][2] - R[2][0]) / s;
        x = (R[0][1] + R[1][0]) / s;
        y = 0.25 * s;
        z = (R[1][2] + R[2][1]) / s;
    } else {
        // z is the largest component
        const double s = std::sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]) * 2.0;  // s = 4*z
        w = (R[1][0] - R[0][1]) / s;
        x = (R[0][2] + R[2][0]) / s;
        y = (R[1][2] + R[2][1]) / s;
        z = 0.25 * s;
    }
    
    return QuaternionImpl(x, y, z, w);
}

} // namespace ekf9dof
