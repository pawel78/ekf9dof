#pragma once

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

