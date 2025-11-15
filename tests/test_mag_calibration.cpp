#include <iostream>
#include <cassert>
#include <cmath>
#include <array>
#include <vector>
#include "imu/processing/mag_calibration.hpp"

// Helper function to compare floats
bool float_equal(float a, float b, float epsilon = 1e-4f) {
    return std::fabs(a - b) < epsilon;
}

void test_hard_iron_calculation() {
    std::cout << "Testing hard iron offset calculation..." << std::endl;
    
    // Create sample data with a known offset
    std::vector<std::array<float, 3>> samples;
    
    // Simulate a circle with offset (1.0, -0.5, 0.3)
    for (int i = 0; i < 100; ++i) {
        float angle = i * 2.0f * M_PI / 100.0f;
        samples.push_back({
            std::cos(angle) + 1.0f,
            std::sin(angle) - 0.5f,
            0.3f  // z is constant with offset
        });
    }
    
    std::array<float, 3> offset;
    mag_calibration::calculate_hard_iron_offset(samples, offset);
    
    // Check that the calculated offset is close to the known offset
    assert(float_equal(offset[0], 1.0f, 0.1f));
    assert(float_equal(offset[1], -0.5f, 0.1f));
    assert(float_equal(offset[2], 0.3f, 0.1f));
    
    std::cout << "  ✓ Hard iron offset calculation test passed" << std::endl;
}

void test_soft_iron_calculation() {
    std::cout << "Testing soft iron matrix calculation..." << std::endl;
    
    // Create sample data - simplified test
    std::vector<std::array<float, 3>> samples;
    std::array<float, 3> hard_iron_offset = {0.0f, 0.0f, 0.0f};
    
    // Create samples in a sphere-like pattern
    for (int i = 0; i < 50; ++i) {
        float angle = i * 2.0f * M_PI / 50.0f;
        samples.push_back({
            std::cos(angle),
            std::sin(angle),
            0.5f
        });
    }
    
    std::array<float, 9> soft_iron_matrix;
    mag_calibration::calculate_soft_iron_matrix(samples, hard_iron_offset, soft_iron_matrix);
    
    // The diagonal elements should be positive
    assert(soft_iron_matrix[0] > 0.0f);
    assert(soft_iron_matrix[4] > 0.0f);
    assert(soft_iron_matrix[8] > 0.0f);
    
    // Off-diagonal elements should be zero (for our simple case)
    assert(float_equal(soft_iron_matrix[1], 0.0f));
    assert(float_equal(soft_iron_matrix[2], 0.0f));
    assert(float_equal(soft_iron_matrix[3], 0.0f));
    assert(float_equal(soft_iron_matrix[5], 0.0f));
    assert(float_equal(soft_iron_matrix[6], 0.0f));
    assert(float_equal(soft_iron_matrix[7], 0.0f));
    
    std::cout << "  ✓ Soft iron matrix calculation test passed" << std::endl;
}

void test_calibration_application() {
    std::cout << "Testing calibration application..." << std::endl;
    
    // Test data
    std::array<float, 3> raw = {2.0f, 3.0f, 4.0f};
    std::array<float, 3> hard_iron_offset = {1.0f, 0.5f, 0.2f};
    std::array<float, 9> soft_iron_matrix = {
        2.0f, 0.0f, 0.0f,
        0.0f, 1.5f, 0.0f,
        0.0f, 0.0f, 1.0f
    };
    std::array<float, 3> calibrated;
    
    mag_calibration::apply_calibration(raw, hard_iron_offset, soft_iron_matrix, calibrated);
    
    // Expected: (raw - offset) * matrix
    // x: (2.0 - 1.0) * 2.0 = 2.0
    // y: (3.0 - 0.5) * 1.5 = 3.75
    // z: (4.0 - 0.2) * 1.0 = 3.8
    
    assert(float_equal(calibrated[0], 2.0f));
    assert(float_equal(calibrated[1], 3.75f));
    assert(float_equal(calibrated[2], 3.8f));
    
    std::cout << "  ✓ Calibration application test passed" << std::endl;
}

void test_identity_calibration() {
    std::cout << "Testing identity calibration (no correction)..." << std::endl;
    
    // Identity matrix and zero offset should return the same values
    std::array<float, 3> raw = {1.5f, -0.8f, 2.3f};
    std::array<float, 3> hard_iron_offset = {0.0f, 0.0f, 0.0f};
    std::array<float, 9> soft_iron_matrix = {
        1.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 1.0f
    };
    std::array<float, 3> calibrated;
    
    mag_calibration::apply_calibration(raw, hard_iron_offset, soft_iron_matrix, calibrated);
    
    assert(float_equal(calibrated[0], raw[0]));
    assert(float_equal(calibrated[1], raw[1]));
    assert(float_equal(calibrated[2], raw[2]));
    
    std::cout << "  ✓ Identity calibration test passed" << std::endl;
}

void test_empty_samples() {
    std::cout << "Testing with empty sample set..." << std::endl;
    
    std::vector<std::array<float, 3>> empty_samples;
    std::array<float, 3> offset;
    std::array<float, 9> matrix;
    
    mag_calibration::calculate_hard_iron_offset(empty_samples, offset);
    
    // Should return zero offset
    assert(float_equal(offset[0], 0.0f));
    assert(float_equal(offset[1], 0.0f));
    assert(float_equal(offset[2], 0.0f));
    
    mag_calibration::calculate_soft_iron_matrix(empty_samples, offset, matrix);
    
    // Should return identity matrix
    assert(float_equal(matrix[0], 1.0f));
    assert(float_equal(matrix[4], 1.0f));
    assert(float_equal(matrix[8], 1.0f));
    
    std::cout << "  ✓ Empty samples test passed" << std::endl;
}

void test_zero_range_samples() {
    std::cout << "Testing with zero range samples (same value)..." << std::endl;
    
    // Create samples with no variation on one axis
    std::vector<std::array<float, 3>> samples;
    for (int i = 0; i < 10; ++i) {
        samples.push_back({1.0f, 2.0f, 3.0f});  // All identical
    }
    
    std::array<float, 3> hard_iron_offset;
    mag_calibration::calculate_hard_iron_offset(samples, hard_iron_offset);
    
    // Offset should be the value itself
    assert(float_equal(hard_iron_offset[0], 1.0f));
    assert(float_equal(hard_iron_offset[1], 2.0f));
    assert(float_equal(hard_iron_offset[2], 3.0f));
    
    std::array<float, 9> soft_iron_matrix;
    mag_calibration::calculate_soft_iron_matrix(samples, hard_iron_offset, soft_iron_matrix);
    
    // Should not crash or produce NaN/Inf values
    assert(!std::isnan(soft_iron_matrix[0]));
    assert(!std::isnan(soft_iron_matrix[4]));
    assert(!std::isnan(soft_iron_matrix[8]));
    assert(!std::isinf(soft_iron_matrix[0]));
    assert(!std::isinf(soft_iron_matrix[4]));
    assert(!std::isinf(soft_iron_matrix[8]));
    
    std::cout << "  ✓ Zero range samples test passed" << std::endl;
}

int main() {
    std::cout << "Running magnetometer calibration tests..." << std::endl;
    std::cout << "===========================================" << std::endl;
    
    try {
        test_hard_iron_calculation();
        test_soft_iron_calculation();
        test_calibration_application();
        test_identity_calibration();
        test_empty_samples();
        test_zero_range_samples();
        
        std::cout << "===========================================" << std::endl;
        std::cout << "All magnetometer calibration tests passed! ✓" << std::endl;
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Test failed with exception: " << e.what() << std::endl;
        return 1;
    }
}
