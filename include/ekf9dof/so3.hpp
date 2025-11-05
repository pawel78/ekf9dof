#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>

/**
 * @file so3.hpp
 * @brief SO3 library for quaternion-based rotations and coordinate transformations
 * 
 * COORDINATE FRAME CONVENTIONS
 * ============================
 * 
 * This library uses the following coordinate frame conventions for the EKF:
 * 
 * 1. BODY FRAME (b):
 *    - Fixed to the vehicle/sensor body
 *    - X-axis: Forward
 *    - Y-axis: Right
 *    - Z-axis: Down
 *    - Origin: At the IMU sensor location
 * 
 * 2. NAVIGATION FRAME (n) / EARTH FRAME (e):
 *    - North-East-Down (NED) convention
 *    - X-axis: North
 *    - Y-axis: East
 *    - Z-axis: Down (towards Earth center)
 *    - Origin: At the local tangent plane
 * 
 * 3. INERTIAL FRAME (i):
 *    - Earth-Centered Earth-Fixed (ECEF) for GPS integration
 *    - Not used extensively in attitude-only EKF
 * 
 * QUATERNION CONVENTION
 * =====================
 * Quaternions are represented as [x, y, z, w] where w is the scalar component.
 * This follows the Hamilton convention with scalar LAST.
 * 
 * For a rotation from frame A to frame B, the quaternion is denoted as q_B_A
 * and represents "attitude of frame B relative to frame A" or "rotation from A to B".
 * 
 * Example: q_n_b represents the rotation from body frame to navigation frame
 * 
 * ROTATION SEQUENCE
 * =================
 * Euler angles use 3-2-1 (yaw-pitch-roll) rotation sequence:
 * 1. First rotation: Yaw (ψ) about Z-axis
 * 2. Second rotation: Pitch (θ) about Y-axis  
 * 3. Third rotation: Roll (φ) about X-axis
 * 
 * NAMING CONVENTIONS
 * ==================
 * - Vectors: v_frame (e.g., v_b for vector in body frame, v_n for vector in nav frame)
 * - Quaternions: q_to_from (e.g., q_n_b rotates from body to nav frame)
 * - Rotation matrices: R_to_from (e.g., R_n_b rotates from body to nav frame)
 * - Euler angles: [roll, pitch, yaw] in radians
 * 
 * USAGE EXAMPLES
 * ==============
 * 
 * Example 1: Convert Euler angles to quaternion
 * ```cpp
 * Eigen::Vector3d euler(0.1, 0.2, 0.3);  // [roll, pitch, yaw]
 * Quaternion q = Quaternion::from_euler(euler);
 * ```
 * 
 * Example 2: Rotate a vector from body to navigation frame
 * ```cpp
 * Eigen::Vector3d v_b(1.0, 0.0, 0.0);  // Vector in body frame
 * Quaternion q_n_b = Quaternion::from_euler(euler);
 * Eigen::Vector3d v_n = q_n_b.rotate(v_b);  // Vector in nav frame
 * ```
 * 
 * Example 3: Transform between frames
 * ```cpp
 * // Sensor measures in body frame, need in nav frame
 * Eigen::Vector3d accel_b = sensor.get_measurement();
 * Eigen::Vector3d accel_n = q_n_b.rotate(accel_b);
 * ```
 */

namespace ekf9dof {

/**
 * @brief Quaternion class for 3D rotations in SO(3)
 * 
 * Represents rotations using unit quaternions with scalar component last [x, y, z, w].
 * Provides conversions to/from Euler angles (321 sequence) and rotation matrices.
 */
class Quaternion {
public:
    /**
     * @brief Default constructor - creates identity quaternion (no rotation)
     */
    Quaternion() : q_(0.0, 0.0, 0.0, 1.0) {}
    
    /**
     * @brief Construct quaternion from components
     * @param x X component (imaginary i)
     * @param y Y component (imaginary j)
     * @param z Z component (imaginary k)
     * @param w W component (real/scalar)
     */
    Quaternion(double x, double y, double z, double w) : q_(x, y, z, w) {
        normalize();
    }
    
    /**
     * @brief Construct from Eigen vector [x, y, z, w]
     * @param vec 4D vector with quaternion components
     */
    explicit Quaternion(const Eigen::Vector4d& vec) : q_(vec) {
        normalize();
    }
    
    /**
     * @brief Convert Euler angles to quaternion (321 rotation sequence)
     * @param euler Vector3d containing [roll, pitch, yaw] in radians
     * @return Quaternion representing the rotation
     * 
     * The 321 sequence means:
     * 1. Rotate by yaw (ψ) about Z-axis
     * 2. Rotate by pitch (θ) about Y-axis
     * 3. Rotate by roll (φ) about X-axis
     */
    static Quaternion from_euler(const Eigen::Vector3d& euler) {
        double roll = euler(0);
        double pitch = euler(1);
        double yaw = euler(2);
        
        // Half angles
        double cr = std::cos(roll * 0.5);
        double sr = std::sin(roll * 0.5);
        double cp = std::cos(pitch * 0.5);
        double sp = std::sin(pitch * 0.5);
        double cy = std::cos(yaw * 0.5);
        double sy = std::sin(yaw * 0.5);
        
        // 321 rotation sequence (yaw-pitch-roll)
        // Derived from q_roll * q_pitch * q_yaw
        Quaternion q;
        q.q_(0) = cr * sp * sy + sr * cp * cy;  // x
        q.q_(1) = cr * sp * cy - sr * cp * sy;  // y
        q.q_(2) = cr * cp * sy + sr * sp * cy;  // z
        q.q_(3) = cr * cp * cy - sr * sp * sy;  // w
        
        q.normalize();
        return q;
    }
    
    /**
     * @brief Convert quaternion to Euler angles (321 sequence)
     * @return Vector3d containing [roll, pitch, yaw] in radians
     * 
     * Roll: rotation about X-axis, range [-π, π]
     * Pitch: rotation about Y-axis, range [-π/2, π/2]
     * Yaw: rotation about Z-axis, range [-π, π]
     */
    Eigen::Vector3d to_euler() const {
        Eigen::Vector3d euler;
        
        double x = q_(0), y = q_(1), z = q_(2), w = q_(3);
        
        // For 321 (ZYX) rotation sequence: R = Rx * Ry * Rz
        // From rotation matrix elements:
        // R(0,2) = sin(pitch)
        // R(1,2) = -sin(roll)*cos(pitch)
        // R(2,2) = cos(roll)*cos(pitch)
        // R(0,1) = -cos(pitch)*sin(yaw)
        // R(0,0) = cos(pitch)*cos(yaw)
        
        // Pitch (θ) from R(0,2) = 2*(qx*qz + qw*qy)
        double sinp = 2.0 * (x * z + w * y);
        if (std::abs(sinp) >= 1.0) {
            euler(1) = std::copysign(M_PI / 2.0, sinp);  // Use ±90° if out of range
        } else {
            euler(1) = std::asin(sinp);
        }
        
        // Roll (φ) from R(1,2) = 2*(qy*qz - qw*qx) and R(2,2) = 1 - 2*(qx*qx + qy*qy)
        // Note: R(1,2) = -sin(roll)*cos(pitch), so -R(1,2) cancels the negative sign
        euler(0) = std::atan2(-(2.0 * (y * z - w * x)), 1.0 - 2.0 * (x * x + y * y));
        
        // Yaw (ψ) from R(0,1) = 2*(qx*qy - qw*qz) and R(0,0) = 1 - 2*(qy*qy + qz*qz)
        // Note: R(0,1) = -cos(pitch)*sin(yaw), so -R(0,1) cancels the negative sign
        euler(2) = std::atan2(-(2.0 * (x * y - w * z)), 1.0 - 2.0 * (y * y + z * z));
        
        return euler;
    }
    
    /**
     * @brief Convert quaternion to rotation matrix
     * @return 3x3 rotation matrix
     */
    Eigen::Matrix3d to_rotation_matrix() const {
        double x = q_(0), y = q_(1), z = q_(2), w = q_(3);
        
        Eigen::Matrix3d R;
        R(0, 0) = 1.0 - 2.0 * (y * y + z * z);
        R(0, 1) = 2.0 * (x * y - w * z);
        R(0, 2) = 2.0 * (x * z + w * y);
        
        R(1, 0) = 2.0 * (x * y + w * z);
        R(1, 1) = 1.0 - 2.0 * (x * x + z * z);
        R(1, 2) = 2.0 * (y * z - w * x);
        
        R(2, 0) = 2.0 * (x * z - w * y);
        R(2, 1) = 2.0 * (y * z + w * x);
        R(2, 2) = 1.0 - 2.0 * (x * x + y * y);
        
        return R;
    }
    
    /**
     * @brief Create quaternion from rotation matrix
     * @param R 3x3 rotation matrix
     * @return Quaternion representing the rotation
     */
    static Quaternion from_rotation_matrix(const Eigen::Matrix3d& R) {
        Quaternion q;
        double trace = R.trace();
        
        if (trace > 0.0) {
            double s = 0.5 / std::sqrt(trace + 1.0);
            q.q_(3) = 0.25 / s;
            q.q_(0) = (R(2, 1) - R(1, 2)) * s;
            q.q_(1) = (R(0, 2) - R(2, 0)) * s;
            q.q_(2) = (R(1, 0) - R(0, 1)) * s;
        } else if (R(0, 0) > R(1, 1) && R(0, 0) > R(2, 2)) {
            double s = 2.0 * std::sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2));
            q.q_(3) = (R(2, 1) - R(1, 2)) / s;
            q.q_(0) = 0.25 * s;
            q.q_(1) = (R(0, 1) + R(1, 0)) / s;
            q.q_(2) = (R(0, 2) + R(2, 0)) / s;
        } else if (R(1, 1) > R(2, 2)) {
            double s = 2.0 * std::sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2));
            q.q_(3) = (R(0, 2) - R(2, 0)) / s;
            q.q_(0) = (R(0, 1) + R(1, 0)) / s;
            q.q_(1) = 0.25 * s;
            q.q_(2) = (R(1, 2) + R(2, 1)) / s;
        } else {
            double s = 2.0 * std::sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1));
            q.q_(3) = (R(1, 0) - R(0, 1)) / s;
            q.q_(0) = (R(0, 2) + R(2, 0)) / s;
            q.q_(1) = (R(1, 2) + R(2, 1)) / s;
            q.q_(2) = 0.25 * s;
        }
        
        q.normalize();
        return q;
    }
    
    /**
     * @brief Rotate a 3D vector by this quaternion
     * @param v Vector to rotate
     * @return Rotated vector
     * 
     * For a quaternion q_n_b (body to nav), this transforms a vector from body to nav frame:
     * v_n = q_n_b.rotate(v_b)
     */
    Eigen::Vector3d rotate(const Eigen::Vector3d& v) const {
        // Using q * v * q^(-1) where v is treated as quaternion with w=0
        Eigen::Vector3d qv(q_(0), q_(1), q_(2));
        double qw = q_(3);
        
        Eigen::Vector3d t = 2.0 * qv.cross(v);
        return v + qw * t + qv.cross(t);
    }
    
    /**
     * @brief Rotate a vector by the inverse of this quaternion
     * @param v Vector to rotate
     * @return Rotated vector
     * 
     * For a quaternion q_n_b (body to nav), this transforms a vector from nav to body frame:
     * v_b = q_n_b.rotate_inverse(v_n)
     */
    Eigen::Vector3d rotate_inverse(const Eigen::Vector3d& v) const {
        return conjugate().rotate(v);
    }
    
    /**
     * @brief Quaternion multiplication (composition of rotations)
     * @param other Quaternion to multiply with
     * @return Product quaternion
     * 
     * q1 * q2 represents first applying rotation q2, then q1
     */
    Quaternion operator*(const Quaternion& other) const {
        double x1 = q_(0), y1 = q_(1), z1 = q_(2), w1 = q_(3);
        double x2 = other.q_(0), y2 = other.q_(1), z2 = other.q_(2), w2 = other.q_(3);
        
        Quaternion result;
        result.q_(0) = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
        result.q_(1) = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2;
        result.q_(2) = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2;
        result.q_(3) = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;
        
        result.normalize();
        return result;
    }
    
    /**
     * @brief Quaternion conjugate (inverse for unit quaternions)
     * @return Conjugate quaternion
     * 
     * For a unit quaternion, conjugate equals inverse.
     * If q rotates from frame A to B, q* rotates from B to A.
     */
    Quaternion conjugate() const {
        Quaternion result;
        result.q_(0) = -q_(0);
        result.q_(1) = -q_(1);
        result.q_(2) = -q_(2);
        result.q_(3) = q_(3);
        return result;
    }
    
    /**
     * @brief Quaternion inverse
     * @return Inverse quaternion
     */
    Quaternion inverse() const {
        return conjugate();  // For unit quaternions
    }
    
    /**
     * @brief Normalize the quaternion to unit length
     */
    void normalize() {
        double norm = q_.norm();
        if (norm < 1e-10) {
            throw std::runtime_error("Cannot normalize zero quaternion");
        }
        q_ /= norm;
    }
    
    /**
     * @brief Get the norm (magnitude) of the quaternion
     * @return Quaternion norm
     */
    double norm() const {
        return q_.norm();
    }
    
    /**
     * @brief Get quaternion as 4D vector [x, y, z, w]
     * @return Const reference to internal vector
     */
    const Eigen::Vector4d& coeffs() const {
        return q_;
    }
    
    /**
     * @brief Get quaternion as 4D vector [x, y, z, w]
     * @return Reference to internal vector
     */
    Eigen::Vector4d& coeffs() {
        return q_;
    }
    
    /**
     * @brief Access individual quaternion components
     * @param i Index (0=x, 1=y, 2=z, 3=w)
     * @return Component value
     */
    double operator()(int i) const {
        return q_(i);
    }
    
    /**
     * @brief Get x component
     */
    double x() const { return q_(0); }
    
    /**
     * @brief Get y component
     */
    double y() const { return q_(1); }
    
    /**
     * @brief Get z component
     */
    double z() const { return q_(2); }
    
    /**
     * @brief Get w (scalar) component
     */
    double w() const { return q_(3); }
    
private:
    Eigen::Vector4d q_;  ///< Quaternion coefficients [x, y, z, w]
};

} // namespace ekf9dof
