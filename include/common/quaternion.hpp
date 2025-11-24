#pragma once

#include <cmath>
#include <array>

/**
 * @file quaternion.hpp
 * @brief Quaternion math library for navigation and attitude estimation
 * 
 * This implementation follows:
 * - Hamilton convention: [x, y, z, w] with scalar component last
 * - 321 Euler angle sequence (yaw-pitch-roll / ZYX)
 * - NED (North-East-Down) navigation frame
 * - Frame-explicit operations for navigation purposes
 * 
 * Naming convention: b_q_a represents a quaternion that expresses
 * vectors from frame 'a' in frame 'b', i.e., v_b = b_q_a.transform(v_a)
 * 
 * Note: This transforms the coordinate representation, not the physical vector.
 * The same physical vector is expressed in different coordinate frames.
 */

/**
 * @brief Quaternion class for frame transformations in navigation applications
 * 
 * All quaternions are unit quaternions (norm = 1) representing frame transformations in SO(3).
 * 
 * Frame convention: For a quaternion b_q_a:
 * - Expresses vectors from frame 'a' in frame 'b' coordinates
 * - Usage: v_b = b_q_a.transform(v_a)
 * - Example: n_q_b expresses body frame vectors in navigation frame
 * 
 * Important distinction:
 * - transform(): Expresses the same physical vector in different coordinates
 * - NOT a rotation: The vector's physical direction doesn't change
 */
class quat {
public:
    /**
     * @brief Default constructor - creates identity quaternion [0, 0, 0, 1]
     */
    quat();

    /**
     * @brief Construct quaternion from components
     * @param x X component (imaginary)
     * @param y Y component (imaginary)
     * @param z Z component (imaginary)
     * @param w W component (scalar/real) - LAST
     * 
     * Note: Quaternion will be automatically normalized
     */
    quat(double x, double y, double z, double w);

    /**
     * @brief Construct from 4D array [x, y, z, w]
     * @param vec Array of 4 doubles with scalar last
     */
    template<typename VectorType>
    explicit quat(const VectorType& vec);

    // Accessors
    double x() const { return data_[0]; }
    double y() const { return data_[1]; }
    double z() const { return data_[2]; }
    double w() const { return data_[3]; }

    /**
     * @brief Access component by index (0=x, 1=y, 2=z, 3=w)
     */
    double operator()(int i) const { return data_[i]; }

    /**
     * @brief Get coefficients as array [x, y, z, w]
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
    quat normalized() const;

    /**
     * @brief Quaternion conjugate (inverse rotation for unit quaternions)
     * 
     * For b_q_a, conjugate gives a_q_b (inverse transformation)
     */
    quat conjugate() const;

    /**
     * @brief Quaternion inverse (same as conjugate for unit quaternions)
     */
    quat inverse() const;

    /**
     * @brief Transform (express) a 3D vector from one frame to another
     * 
     * Expresses a vector in a different coordinate frame.
     * The physical vector remains the same; only its coordinate representation changes.
     * 
     * Performs: v' = q * v * q^(-1)
     * For b_q_a: v_b = b_q_a.transform(v_a)
     * 
     * @param v 3D vector in frame 'a' as array [x, y, z]
     * @return Same vector expressed in frame 'b' as array [x, y, z]
     */
    template<typename VectorType>
    std::array<double, 3> transform(const VectorType& v) const;

    /**
     * @brief Transform a vector using the inverse frame transformation
     * 
     * Expresses a vector in the inverse coordinate frame.
     * For b_q_a: v_a = b_q_a.transform_inverse(v_b)
     * Equivalent to: conjugate().transform(v)
     * 
     * @param v 3D vector to transform
     * @return Transformed vector as array [x, y, z]
     */
    template<typename VectorType>
    std::array<double, 3> transform_inverse(const VectorType& v) const;

    /**
     * @brief Quaternion multiplication (composition of rotations)
     * 
     * Frame-explicit composition: c_q_a = c_q_b * b_q_a
     * This means: first rotate from a to b, then from b to c
     * 
     * @param other Other quaternion (right operand)
     * @return Product quaternion
     */
    quat operator*(const quat& other) const;

    /**
     * @brief Convert quaternion to 3x3 rotation matrix
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
     * @param euler Euler angles [roll, pitch, yaw] in radians
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
    static quat from_euler(const VectorType& euler);

    /**
     * @brief Create quaternion from rotation matrix
     * 
     * @param R 3x3 rotation matrix (nested array)
     * @return Quaternion representing the rotation
     */
    template<typename MatrixType>
    static quat from_rotation_matrix(const MatrixType& R);

private:
    std::array<double, 4> data_;  // [x, y, z, w] - scalar LAST
};

// Template implementations

template<typename VectorType>
quat::quat(const VectorType& vec) 
    : data_{vec[0], vec[1], vec[2], vec[3]} {
    normalize();
}

template<typename VectorType>
std::array<double, 3> quat::transform(const VectorType& v) const {
    // Transform vector to different frame: v' = q * v * q^(-1)
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
std::array<double, 3> quat::transform_inverse(const VectorType& v) const {
    return conjugate().transform(v);
}

template<typename VectorType>
quat quat::from_euler(const VectorType& euler) {
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
    const double x = sr * cp * cy + cr * sp * sy;
    const double y = cr * sp * cy - sr * cp * sy;
    const double z = cr * cp * sy + sr * sp * cy;
    const double w = cr * cp * cy - sr * sp * sy;
    
    return quat(x, y, z, w);
}

template<typename MatrixType>
quat quat::from_rotation_matrix(const MatrixType& R) {
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
    
    return quat(x, y, z, w);
}

