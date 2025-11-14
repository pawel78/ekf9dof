#include <iostream>
#include <cassert>
#include <cmath>
#include "ekf9dof/so3.hpp"

using namespace ekf9dof;

// Helper function to compare doubles with tolerance
bool double_equal(double a, double b, double epsilon = 1e-6) {
    return std::fabs(a - b) < epsilon;
}

// Helper to compare vectors
bool vector_equal(const Eigen::Vector3d& a, const Eigen::Vector3d& b, double epsilon = 1e-6) {
    return (a - b).norm() < epsilon;
}

// Helper to compare quaternions
bool quaternion_equal(const Quaternion& a, const Quaternion& b, double epsilon = 1e-6) {
    return std::fabs(a.x() - b.x()) < epsilon &&
           std::fabs(a.y() - b.y()) < epsilon &&
           std::fabs(a.z() - b.z()) < epsilon &&
           std::fabs(a.w() - b.w()) < epsilon;
}

void test_identity_quaternion() {
    std::cout << "Testing identity quaternion..." << std::endl;
    
    Quaternion q;
    assert(double_equal(q.x(), 0.0));
    assert(double_equal(q.y(), 0.0));
    assert(double_equal(q.z(), 0.0));
    assert(double_equal(q.w(), 1.0));
    assert(double_equal(q.norm(), 1.0));
    
    std::cout << "  ✓ Identity quaternion test passed" << std::endl;
}

void test_quaternion_construction() {
    std::cout << "Testing quaternion construction..." << std::endl;
    
    // Test from components
    const double sqrt2_2 = std::sqrt(2.0) / 2.0;  // sin(45°) = cos(45°)
    Quaternion q1(0.0, 0.0, sqrt2_2, sqrt2_2);  // 90° rotation about Z
    assert(double_equal(q1.norm(), 1.0));
    
    // Test from Eigen vector
    Eigen::Vector4d vec(0.0, 0.0, sqrt2_2, sqrt2_2);
    Quaternion q2(vec);
    assert(double_equal(q2.norm(), 1.0));
    
    std::cout << "  ✓ Quaternion construction test passed" << std::endl;
}

void test_euler_to_quaternion_simple() {
    std::cout << "Testing simple Euler to quaternion conversions..." << std::endl;
    
    // Test zero rotation
    Eigen::Vector3d euler_zero(0.0, 0.0, 0.0);
    Quaternion q_zero = Quaternion::from_euler(euler_zero);
    assert(quaternion_equal(q_zero, Quaternion()));
    
    // Test 90° roll (about X-axis)
    const double sqrt2_2 = std::sqrt(2.0) / 2.0;  // sin(45°) = cos(45°)
    Eigen::Vector3d euler_roll(M_PI / 2.0, 0.0, 0.0);
    Quaternion q_roll = Quaternion::from_euler(euler_roll);
    assert(double_equal(q_roll.x(), sqrt2_2, 1e-5));
    assert(double_equal(q_roll.y(), 0.0, 1e-5));
    assert(double_equal(q_roll.z(), 0.0, 1e-5));
    assert(double_equal(q_roll.w(), sqrt2_2, 1e-5));
    
    // Test 90° pitch (about Y-axis)
    Eigen::Vector3d euler_pitch(0.0, M_PI / 2.0, 0.0);
    Quaternion q_pitch = Quaternion::from_euler(euler_pitch);
    assert(double_equal(q_pitch.x(), 0.0, 1e-5));
    assert(double_equal(q_pitch.y(), sqrt2_2, 1e-5));
    assert(double_equal(q_pitch.z(), 0.0, 1e-5));
    assert(double_equal(q_pitch.w(), sqrt2_2, 1e-5));
    
    // Test 90° yaw (about Z-axis)
    Eigen::Vector3d euler_yaw(0.0, 0.0, M_PI / 2.0);
    Quaternion q_yaw = Quaternion::from_euler(euler_yaw);
    assert(double_equal(q_yaw.x(), 0.0, 1e-5));
    assert(double_equal(q_yaw.y(), 0.0, 1e-5));
    assert(double_equal(q_yaw.z(), sqrt2_2, 1e-5));
    assert(double_equal(q_yaw.w(), sqrt2_2, 1e-5));
    
    std::cout << "  ✓ Simple Euler to quaternion test passed" << std::endl;
}

void test_quaternion_to_euler() {
    std::cout << "Testing quaternion to Euler conversions..." << std::endl;
    
    // Test round-trip conversion
    Eigen::Vector3d euler_in(0.1, 0.2, 0.3);  // 10°, 20°, 30°
    Quaternion q = Quaternion::from_euler(euler_in);
    Eigen::Vector3d euler_out = q.to_euler();
    
    assert(vector_equal(euler_in, euler_out, 1e-6));
    
    // Test multiple angles
    Eigen::Vector3d euler_test(0.5, -0.3, 1.2);
    Quaternion q2 = Quaternion::from_euler(euler_test);
    Eigen::Vector3d euler_out2 = q2.to_euler();
    assert(vector_equal(euler_test, euler_out2, 1e-6));
    
    std::cout << "  ✓ Quaternion to Euler test passed" << std::endl;
}

void test_vector_rotation() {
    std::cout << "Testing vector rotation..." << std::endl;
    
    // Rotate vector [1, 0, 0] by 90° about Z-axis
    // Should result in [0, 1, 0]
    Eigen::Vector3d euler(0.0, 0.0, M_PI / 2.0);
    Quaternion q = Quaternion::from_euler(euler);
    
    Eigen::Vector3d v_in(1.0, 0.0, 0.0);
    Eigen::Vector3d v_out = q.rotate(v_in);
    Eigen::Vector3d v_expected(0.0, 1.0, 0.0);
    
    assert(vector_equal(v_out, v_expected, 1e-6));
    
    std::cout << "  ✓ Vector rotation test passed" << std::endl;
}

void test_inverse_rotation() {
    std::cout << "Testing inverse rotation..." << std::endl;
    
    Eigen::Vector3d euler(0.3, 0.4, 0.5);
    Quaternion q = Quaternion::from_euler(euler);
    
    Eigen::Vector3d v_original(1.0, 2.0, 3.0);
    Eigen::Vector3d v_rotated = q.rotate(v_original);
    Eigen::Vector3d v_back = q.rotate_inverse(v_rotated);
    
    assert(vector_equal(v_original, v_back, 1e-6));
    
    std::cout << "  ✓ Inverse rotation test passed" << std::endl;
}

void test_quaternion_multiplication() {
    std::cout << "Testing quaternion multiplication..." << std::endl;
    
    // Two 90° rotations about Z should equal 180° rotation
    Eigen::Vector3d euler1(0.0, 0.0, M_PI / 2.0);
    Quaternion q1 = Quaternion::from_euler(euler1);
    
    Quaternion q2 = Quaternion::from_euler(euler1);
    Quaternion q_combined = q1 * q2;
    
    // Check that it's equivalent to 180° rotation
    Eigen::Vector3d v_in(1.0, 0.0, 0.0);
    Eigen::Vector3d v_out = q_combined.rotate(v_in);
    Eigen::Vector3d v_expected(-1.0, 0.0, 0.0);
    
    assert(vector_equal(v_out, v_expected, 1e-6));
    
    std::cout << "  ✓ Quaternion multiplication test passed" << std::endl;
}

void test_conjugate_inverse() {
    std::cout << "Testing conjugate and inverse..." << std::endl;
    
    Eigen::Vector3d euler(0.2, 0.3, 0.4);
    Quaternion q = Quaternion::from_euler(euler);
    
    Quaternion q_conj = q.conjugate();
    Quaternion q_inv = q.inverse();
    
    // For unit quaternions, conjugate should equal inverse
    assert(quaternion_equal(q_conj, q_inv));
    
    // q * q^(-1) should be identity
    Quaternion q_identity = q * q_inv;
    assert(quaternion_equal(q_identity, Quaternion(), 1e-5));
    
    std::cout << "  ✓ Conjugate and inverse test passed" << std::endl;
}

void test_rotation_matrix_conversion() {
    std::cout << "Testing rotation matrix conversions..." << std::endl;
    
    // Test conversion to rotation matrix
    Eigen::Vector3d euler(0.1, 0.2, 0.3);
    Quaternion q = Quaternion::from_euler(euler);
    Eigen::Matrix3d R = q.to_rotation_matrix();
    
    // Rotation matrix should be orthogonal (R^T * R = I)
    Eigen::Matrix3d I = R.transpose() * R;
    assert((I - Eigen::Matrix3d::Identity()).norm() < 1e-6);
    
    // Determinant should be 1
    assert(double_equal(R.determinant(), 1.0, 1e-6));
    
    // Test round-trip conversion
    Quaternion q2 = Quaternion::from_rotation_matrix(R);
    // Quaternions q and -q represent the same rotation
    Quaternion q2_neg(-q2.x(), -q2.y(), -q2.z(), -q2.w());
    assert(quaternion_equal(q, q2, 1e-5) || quaternion_equal(q, q2_neg, 1e-5));
    
    std::cout << "  ✓ Rotation matrix conversion test passed" << std::endl;
}

void test_rotation_matrix_vector() {
    std::cout << "Testing rotation matrix and vector rotation consistency..." << std::endl;
    
    Eigen::Vector3d euler(0.2, 0.3, 0.4);
    Quaternion q = Quaternion::from_euler(euler);
    Eigen::Matrix3d R = q.to_rotation_matrix();
    
    Eigen::Vector3d v(1.0, 2.0, 3.0);
    Eigen::Vector3d v_rot_quat = q.rotate(v);
    Eigen::Vector3d v_rot_mat = R * v;
    
    assert(vector_equal(v_rot_quat, v_rot_mat, 1e-6));
    
    std::cout << "  ✓ Rotation matrix and vector rotation consistency test passed" << std::endl;
}

void test_321_rotation_sequence() {
    std::cout << "Testing 321 rotation sequence (yaw-pitch-roll)..." << std::endl;
    
    // Create separate rotations
    double roll = 0.1;
    double pitch = 0.2;
    double yaw = 0.3;
    
    // Individual rotation matrices (321 sequence)
    Eigen::Matrix3d Rz;  // Yaw
    Rz << std::cos(yaw), -std::sin(yaw), 0,
          std::sin(yaw), std::cos(yaw), 0,
          0, 0, 1;
    
    Eigen::Matrix3d Ry;  // Pitch
    Ry << std::cos(pitch), 0, std::sin(pitch),
          0, 1, 0,
          -std::sin(pitch), 0, std::cos(pitch);
    
    Eigen::Matrix3d Rx;  // Roll
    Rx << 1, 0, 0,
          0, std::cos(roll), -std::sin(roll),
          0, std::sin(roll), std::cos(roll);
    
    // 321 sequence: R = Rx * Ry * Rz
    Eigen::Matrix3d R_321 = Rx * Ry * Rz;
    
    // Compare with quaternion conversion
    Eigen::Vector3d euler(roll, pitch, yaw);
    Quaternion q = Quaternion::from_euler(euler);
    Eigen::Matrix3d R_quat = q.to_rotation_matrix();
    
    assert((R_321 - R_quat).norm() < 1e-6);
    
    std::cout << "  ✓ 321 rotation sequence test passed" << std::endl;
}

void test_coordinate_frame_transformation() {
    std::cout << "Testing coordinate frame transformation example..." << std::endl;
    
    // Scenario: Vehicle pitched up by 30° (positive pitch)
    // Vector pointing forward in body frame should point up in nav frame
    double pitch = M_PI / 6.0;  // 30°
    Eigen::Vector3d euler(0.0, pitch, 0.0);
    Quaternion q_n_b = Quaternion::from_euler(euler);
    
    // Vector pointing forward (X-axis) in body frame
    Eigen::Vector3d v_body(1.0, 0.0, 0.0);
    
    // Rotate to navigation frame
    Eigen::Vector3d v_nav = q_n_b.rotate(v_body);
    
    // Should have positive X component (north) and negative Z component (up, since Z is down)
    assert(v_nav(0) > 0.5);  // Mostly pointing north
    assert(v_nav(2) < 0.0);  // Pointing up (negative Z)
    
    std::cout << "  ✓ Coordinate frame transformation test passed" << std::endl;
}

void test_normalization() {
    std::cout << "Testing quaternion normalization..." << std::endl;
    
    // Create unnormalized quaternion
    Quaternion q(1.0, 1.0, 1.0, 1.0);
    assert(double_equal(q.norm(), 1.0));  // Should be automatically normalized
    
    // Create another and check norm
    Quaternion q2(0.5, 0.5, 0.5, 0.5);
    assert(double_equal(q2.norm(), 1.0));
    
    std::cout << "  ✓ Quaternion normalization test passed" << std::endl;
}

void test_accessor_methods() {
    std::cout << "Testing accessor methods..." << std::endl;
    
    Quaternion q(0.1, 0.2, 0.3, 0.4);
    
    assert(double_equal(q.x(), q(0)));
    assert(double_equal(q.y(), q(1)));
    assert(double_equal(q.z(), q(2)));
    assert(double_equal(q.w(), q(3)));
    
    const Eigen::Vector4d& coeffs = q.coeffs();
    assert(double_equal(coeffs(0), q.x()));
    assert(double_equal(coeffs(1), q.y()));
    assert(double_equal(coeffs(2), q.z()));
    assert(double_equal(coeffs(3), q.w()));
    
    std::cout << "  ✓ Accessor methods test passed" << std::endl;
}

void test_edge_cases() {
    std::cout << "Testing edge cases..." << std::endl;
    
    // Test gimbal lock (pitch = ±90°)
    Eigen::Vector3d euler_gimbal(0.1, M_PI / 2.0, 0.3);
    Quaternion q_gimbal = Quaternion::from_euler(euler_gimbal);
    Eigen::Vector3d euler_out = q_gimbal.to_euler();
    
    // Should handle gimbal lock gracefully
    assert(double_equal(euler_out(1), M_PI / 2.0, 1e-5));
    
    // Test negative angles
    Eigen::Vector3d euler_neg(-0.5, -0.3, -0.8);
    Quaternion q_neg = Quaternion::from_euler(euler_neg);
    Eigen::Vector3d euler_out_neg = q_neg.to_euler();
    assert(vector_equal(euler_neg, euler_out_neg, 1e-6));
    
    std::cout << "  ✓ Edge cases test passed" << std::endl;
}

int main() {
    std::cout << "Running SO3 Quaternion Library Tests..." << std::endl;
    std::cout << "======================================" << std::endl;
    
    try {
        test_identity_quaternion();
        test_quaternion_construction();
        test_euler_to_quaternion_simple();
        test_quaternion_to_euler();
        test_vector_rotation();
        test_inverse_rotation();
        test_quaternion_multiplication();
        test_conjugate_inverse();
        test_rotation_matrix_conversion();
        test_rotation_matrix_vector();
        test_321_rotation_sequence();
        test_coordinate_frame_transformation();
        test_normalization();
        test_accessor_methods();
        test_edge_cases();
        
        std::cout << "======================================" << std::endl;
        std::cout << "All SO3 tests passed! ✓" << std::endl;
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Test failed with exception: " << e.what() << std::endl;
        return 1;
    }
}
