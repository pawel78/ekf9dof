#pragma once

#include "common/Quat.hpp"

/**
 * @file so3.hpp
 * @brief SO3 library for quaternion-based frame transformations
 * 
 * COORDINATE FRAME CONVENTIONS
 * ============================
 * 
 * This library uses the following coordinate frame conventions for navigation:
 * 
 * 1. BODY FRAME (b):
 *    - Fixed to the vehicle/sensor body
 *    - X-axis: Forward
 *    - Y-axis: Right
 *    - Z-axis: Down (FRD convention)
 *    - Origin: At the IMU sensor location
 * 
 * 2. NAVIGATION FRAME (n) / EARTH FRAME (e):
 *    - North-East-Down (NED) convention
 *    - X-axis: North
 *    - Y-axis: East
 *    - Z-axis: Down (towards Earth center)
 *    - Origin: At the local tangent plane
 * 
 * QUATERNION CONVENTION
 * =====================
 * Quaternions are represented as [x, y, z, w] where w is the scalar component (LAST).
 * This follows the Hamilton convention with scalar LAST.
 * 
 * FRAME-EXPLICIT NAMING
 * =====================
 * For a quaternion b_q_a:
 * - Expresses vectors from frame 'a' in frame 'b' coordinates
 * - Usage: v_b = b_q_a * v_a (using * operator)
 * - Example: n_q_b expresses body frame vectors in navigation frame coordinates
 * 
 * Important: The * operator changes the coordinate representation, NOT the physical vector.
 * The same physical vector is expressed in different coordinate frames.
 * 
 * Quaternion composition: c_q_a = c_q_b * b_q_a
 * This reads: "express from a in b, then from b in c"
 * 
 * ROTATION SEQUENCE
 * =================
 * Euler angles use 3-2-1 (yaw-pitch-roll) rotation sequence:
 * 1. First rotation: Yaw (ψ) about Z-axis
 * 2. Second rotation: Pitch (θ) about Y-axis  
 * 3. Third rotation: Roll (φ) about X-axis
 * 
 * USAGE EXAMPLES
 * ==============
 * 
 * Example 1: Convert Euler angles to quaternion
 * ```cpp
 * std::array<double, 3> euler{0.1, 0.2, 0.3};  // [roll, pitch, yaw]
 * Quat n_q_b = Quat::from_euler(euler);
 * ```
 * 
 * Example 2: Express a vector from body to navigation frame
 * ```cpp
 * std::array<double, 3> v_b{1.0, 0.0, 0.0};  // Vector in body frame
 * Quat n_q_b = Quat::from_euler(euler);
 * std::array<double, 3> v_n = n_q_b * v_b;  // Same vector in nav frame using * operator
 * ```
 * 
 * Example 3: Compose transformations
 * ```cpp
 * // Transform from body to intermediate, then to navigation
 * Quat i_q_b = ...;  // body to intermediate
 * Quat n_q_i = ...;  // intermediate to navigation
 * Quat n_q_b = n_q_i * i_q_b;  // composed transformation
 * 
 * // Use it: v_n = n_q_b * v_b
 * ```
 */

