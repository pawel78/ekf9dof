#pragma once

#include "common/quaternion.hpp"
#include <cmath>
#include <stdexcept>
#include <array>

// Eigen compatibility layer - provides types similar to Eigen for use with Quaternion class
namespace Eigen {
    /**
     * @brief 3D vector type (compatible with std::array)
     */
    struct Vector3d {
        std::array<double, 3> data;
        
        Vector3d() : data{0.0, 0.0, 0.0} {}
        Vector3d(double x, double y, double z) : data{x, y, z} {}
        explicit Vector3d(const std::array<double, 3>& arr) : data{arr} {}
        
        double operator[](int i) const { return data[i]; }
        double& operator[](int i) { return data[i]; }
        double operator()(int i) const { return data[i]; }
        double& operator()(int i) { return data[i]; }
        
        double norm() const {
            return std::sqrt(data[0]*data[0] + data[1]*data[1] + data[2]*data[2]);
        }
        
        Vector3d normalized() const {
            const double n = norm();
            if (n < 1e-12) return Vector3d();
            return Vector3d(data[0]/n, data[1]/n, data[2]/n);
        }
        
        Vector3d operator-(const Vector3d& other) const {
            return Vector3d(data[0] - other.data[0], 
                          data[1] - other.data[1], 
                          data[2] - other.data[2]);
        }
    };
    
    /**
     * @brief 4D vector type for quaternion coefficients
     */
    struct Vector4d {
        std::array<double, 4> data;
        
        Vector4d() : data{0.0, 0.0, 0.0, 1.0} {}
        Vector4d(double x, double y, double z, double w) : data{x, y, z, w} {}
        explicit Vector4d(const std::array<double, 4>& arr) : data{arr} {}
        
        double operator[](int i) const { return data[i]; }
        double& operator[](int i) { return data[i]; }
        double operator()(int i) const { return data[i]; }
        double& operator()(int i) { return data[i]; }
    };
    
    /**
     * @brief 3x3 matrix type
     */
    struct Matrix3d {
        std::array<std::array<double, 3>, 3> data;
        
        Matrix3d() {
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j)
                    data[i][j] = 0.0;
        }
        
        explicit Matrix3d(const std::array<std::array<double, 3>, 3>& arr) : data{arr} {}
        
        std::array<double, 3>& operator[](int i) { return data[i]; }
        const std::array<double, 3>& operator[](int i) const { return data[i]; }
        
        double operator()(int i, int j) const { return data[i][j]; }
        double& operator()(int i, int j) { return data[i][j]; }
        
        // Comma initialization helper
        struct CommaInitializer {
            Matrix3d& mat;
            int row, col;
            
            CommaInitializer(Matrix3d& m, int r, int c) : mat(m), row(r), col(c) {}
            
            CommaInitializer& operator,(double val) {
                if (row < 3 && col < 3) {
                    mat.data[row][col] = val;
                    col++;
                    if (col >= 3) {
                        col = 0;
                        row++;
                    }
                }
                return *this;
            }
        };
        
        CommaInitializer operator<<(double val) {
            data[0][0] = val;
            return CommaInitializer(*this, 0, 1);
        }
        
        Matrix3d transpose() const {
            Matrix3d result;
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j)
                    result.data[j][i] = data[i][j];
            return result;
        }
        
        Matrix3d operator*(const Matrix3d& other) const {
            Matrix3d result;
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    result.data[i][j] = 0.0;
                    for (int k = 0; k < 3; ++k) {
                        result.data[i][j] += data[i][k] * other.data[k][j];
                    }
                }
            }
            return result;
        }
        
        Vector3d operator*(const Vector3d& vec) const {
            Vector3d result;
            for (int i = 0; i < 3; ++i) {
                result.data[i] = 0.0;
                for (int j = 0; j < 3; ++j) {
                    result.data[i] += data[i][j] * vec.data[j];
                }
            }
            return result;
        }
        
        Matrix3d operator-(const Matrix3d& other) const {
            Matrix3d result;
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j)
                    result.data[i][j] = data[i][j] - other.data[i][j];
            return result;
        }
        
        double norm() const {
            double sum = 0.0;
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j)
                    sum += data[i][j] * data[i][j];
            return std::sqrt(sum);
        }
        
        double determinant() const {
            return data[0][0] * (data[1][1] * data[2][2] - data[1][2] * data[2][1])
                 - data[0][1] * (data[1][0] * data[2][2] - data[1][2] * data[2][0])
                 + data[0][2] * (data[1][0] * data[2][1] - data[1][1] * data[2][0]);
        }
        
        static Matrix3d Identity() {
            Matrix3d result;
            result.data[0][0] = 1.0;
            result.data[1][1] = 1.0;
            result.data[2][2] = 1.0;
            return result;
        }
    };
}

namespace ekf9dof {

// Import the base Quaternion implementation class
using QuaternionBase = ::ekf9dof::QuaternionImpl;

/**
 * @brief Eigen-compatible Quaternion class
 * 
 * This class extends the core Quaternion class and provides Eigen-like interface
 * for compatibility with existing code that expects Eigen types.
 */
class Quaternion {
public:
    // Constructors
    Quaternion() : quat_(), coeffs_cache_(quat_.coeffs()) {}
    
    Quaternion(double x, double y, double z, double w) : quat_(x, y, z, w) {
        update_coeffs_cache();
    }
    
    explicit Quaternion(const QuaternionBase& q) : quat_(q) {
        update_coeffs_cache();
    }
    
    explicit Quaternion(const Eigen::Vector4d& vec) : quat_(vec.data) {
        update_coeffs_cache();
    }
    
    // Accessors
    double x() const { return quat_.x(); }
    double y() const { return quat_.y(); }
    double z() const { return quat_.z(); }
    double w() const { return quat_.w(); }
    
    double operator()(int i) const { return quat_(i); }
    
    const Eigen::Vector4d& coeffs() const {
        return coeffs_cache_;
    }
    
    double norm() const { return quat_.norm(); }
    
    void normalize() { 
        quat_.normalize(); 
        update_coeffs_cache();
    }
    
    Quaternion normalized() const { 
        return Quaternion(quat_.normalized()); 
    }
    
    Quaternion conjugate() const { return Quaternion(quat_.conjugate()); }
    Quaternion inverse() const { return Quaternion(quat_.inverse()); }
    
    // Eigen-compatible rotate methods
    Eigen::Vector3d rotate(const Eigen::Vector3d& v) const {
        auto result = quat_.rotate(v.data);
        return Eigen::Vector3d(result);
    }
    
    Eigen::Vector3d rotate_inverse(const Eigen::Vector3d& v) const {
        auto result = quat_.rotate_inverse(v.data);
        return Eigen::Vector3d(result);
    }
    
    Quaternion operator*(const Quaternion& other) const {
        return Quaternion(quat_ * other.quat_);
    }
    
    Eigen::Matrix3d to_rotation_matrix() const {
        auto mat = quat_.to_rotation_matrix();
        return Eigen::Matrix3d(mat);
    }
    
    Eigen::Vector3d to_euler() const {
        auto euler = quat_.to_euler();
        return Eigen::Vector3d(euler);
    }
    
    // Static factory methods with Eigen types
    static Quaternion from_euler(const Eigen::Vector3d& euler) {
        return Quaternion(QuaternionBase::from_euler(euler.data));
    }
    
    static Quaternion from_rotation_matrix(const Eigen::Matrix3d& R) {
        return Quaternion(QuaternionBase::from_rotation_matrix(R.data));
    }
    
private:
    QuaternionBase quat_;
    Eigen::Vector4d coeffs_cache_;
    
    void update_coeffs_cache() {
        const auto& c = quat_.coeffs();
        coeffs_cache_ = Eigen::Vector4d(c);
    }
};

} // namespace ekf9dof

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

