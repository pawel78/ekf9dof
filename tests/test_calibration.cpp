/**
 * @file test_calibration.cpp
 * @brief Unit tests for accelerometer bias calibration logic
 */

#include <iostream>
#include <cassert>
#include <cmath>
#include <vector>
#include <array>
#include <fstream>
#include <sstream>

// Helper structures and functions for testing calibration logic
struct AccelData {
    double x, y, z;
};

// Compute mean of accelerometer samples
AccelData compute_mean(const std::vector<AccelData>& samples) {
    AccelData mean = {0.0, 0.0, 0.0};
    
    for (const auto& sample : samples) {
        mean.x += sample.x;
        mean.y += sample.y;
        mean.z += sample.z;
    }
    
    mean.x /= samples.size();
    mean.y /= samples.size();
    mean.z /= samples.size();
    
    return mean;
}

// Compute accelerometer bias using least squares
std::array<double, 3> compute_bias(const std::vector<std::pair<AccelData, AccelData>>& measurements) {
    std::array<double, 3> bias_sum = {0.0, 0.0, 0.0};
    
    for (const auto& [measured, expected] : measurements) {
        bias_sum[0] += measured.x - expected.x;
        bias_sum[1] += measured.y - expected.y;
        bias_sum[2] += measured.z - expected.z;
    }
    
    // Average the bias estimates
    bias_sum[0] /= measurements.size();
    bias_sum[1] /= measurements.size();
    bias_sum[2] /= measurements.size();
    
    return bias_sum;
}

// Update config.yaml file with new bias values
bool update_config_file(const std::array<double, 3>& bias, const std::string& config_file) {
    std::ifstream config_in(config_file);
    if (!config_in.is_open()) {
        return false;
    }
    
    std::vector<std::string> lines;
    std::string line;
    bool found_accel_bias = false;
    
    while (std::getline(config_in, line)) {
        if (line.find("accel_bias:") != std::string::npos) {
            lines.push_back("  accel_bias: [" + 
                          std::to_string(bias[0]) + ", " + 
                          std::to_string(bias[1]) + ", " + 
                          std::to_string(bias[2]) + "]");
            found_accel_bias = true;
        } else {
            lines.push_back(line);
        }
    }
    config_in.close();
    
    if (!found_accel_bias) {
        return false;
    }
    
    // Write back to file
    std::ofstream config_out(config_file);
    if (!config_out.is_open()) {
        return false;
    }
    
    for (const auto& l : lines) {
        config_out << l << "\n";
    }
    config_out.close();
    
    return true;
}

// Helper function to compare doubles
bool double_equal(double a, double b, double epsilon = 1e-6) {
    return std::fabs(a - b) < epsilon;
}

void test_compute_mean() {
    std::cout << "Testing compute_mean..." << std::endl;
    
    std::vector<AccelData> samples = {
        {1.0, 2.0, 3.0},
        {2.0, 3.0, 4.0},
        {3.0, 4.0, 5.0}
    };
    
    auto mean = compute_mean(samples);
    
    assert(double_equal(mean.x, 2.0));
    assert(double_equal(mean.y, 3.0));
    assert(double_equal(mean.z, 4.0));
    
    std::cout << "  ✓ compute_mean test passed" << std::endl;
}

void test_compute_bias_perfect_data() {
    std::cout << "Testing compute_bias with perfect data (no bias)..." << std::endl;
    
    // Simulate perfect measurements with no bias
    std::vector<std::pair<AccelData, AccelData>> measurements = {
        {{0.0, 0.0, 1.0}, {0.0, 0.0, 1.0}},   // +Z up
        {{0.0, 0.0, -1.0}, {0.0, 0.0, -1.0}}, // -Z up
        {{1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}},   // +X up
        {{-1.0, 0.0, 0.0}, {-1.0, 0.0, 0.0}}, // -X up
        {{0.0, 1.0, 0.0}, {0.0, 1.0, 0.0}},   // +Y up
        {{0.0, -1.0, 0.0}, {0.0, -1.0, 0.0}}  // -Y up
    };
    
    auto bias = compute_bias(measurements);
    
    assert(double_equal(bias[0], 0.0));
    assert(double_equal(bias[1], 0.0));
    assert(double_equal(bias[2], 0.0));
    
    std::cout << "  ✓ compute_bias test with perfect data passed" << std::endl;
}

void test_compute_bias_with_constant_bias() {
    std::cout << "Testing compute_bias with constant bias..." << std::endl;
    
    // Simulate measurements with a constant bias of [0.1, -0.05, 0.03]
    double bias_x = 0.1;
    double bias_y = -0.05;
    double bias_z = 0.03;
    
    std::vector<std::pair<AccelData, AccelData>> measurements = {
        {{0.0 + bias_x, 0.0 + bias_y, 1.0 + bias_z}, {0.0, 0.0, 1.0}},
        {{0.0 + bias_x, 0.0 + bias_y, -1.0 + bias_z}, {0.0, 0.0, -1.0}},
        {{1.0 + bias_x, 0.0 + bias_y, 0.0 + bias_z}, {1.0, 0.0, 0.0}},
        {{-1.0 + bias_x, 0.0 + bias_y, 0.0 + bias_z}, {-1.0, 0.0, 0.0}},
        {{0.0 + bias_x, 1.0 + bias_y, 0.0 + bias_z}, {0.0, 1.0, 0.0}},
        {{0.0 + bias_x, -1.0 + bias_y, 0.0 + bias_z}, {0.0, -1.0, 0.0}}
    };
    
    auto computed_bias = compute_bias(measurements);
    
    assert(double_equal(computed_bias[0], bias_x, 1e-5));
    assert(double_equal(computed_bias[1], bias_y, 1e-5));
    assert(double_equal(computed_bias[2], bias_z, 1e-5));
    
    std::cout << "  ✓ compute_bias test with constant bias passed" << std::endl;
}

void test_compute_bias_asymmetric() {
    std::cout << "Testing compute_bias with asymmetric bias..." << std::endl;
    
    // Test with only positive orientations to ensure averaging works
    std::vector<std::pair<AccelData, AccelData>> measurements = {
        {{0.1, 0.0, 1.05}, {0.0, 0.0, 1.0}},   // +Z up with bias
        {{1.1, 0.0, 0.05}, {1.0, 0.0, 0.0}},   // +X up with bias
        {{0.1, 1.0, 0.05}, {0.0, 1.0, 0.0}}    // +Y up with bias
    };
    
    auto bias = compute_bias(measurements);
    
    // Average bias should be [0.1, 0.0, 0.05]
    assert(double_equal(bias[0], 0.1, 1e-5));
    assert(double_equal(bias[1], 0.0, 1e-5));
    assert(double_equal(bias[2], 0.05, 1e-5));
    
    std::cout << "  ✓ compute_bias test with asymmetric data passed" << std::endl;
}

void test_config_file_update() {
    std::cout << "Testing config file update..." << std::endl;
    
    // Create a temporary config file
    const std::string test_config = "/tmp/test_config.yaml";
    
    std::ofstream config_out(test_config);
    config_out << "sensor:\n";
    config_out << "  addr_ag: 0x6B\n";
    config_out << "\n";
    config_out << "calibration:\n";
    config_out << "  gyro_bias: [0.0, 0.0, 0.0]\n";
    config_out << "  accel_bias: [0.0, 0.0, 0.0]\n";
    config_out << "  accel_scale: [1.0, 1.0, 1.0]\n";
    config_out.close();
    
    // Update bias
    std::array<double, 3> new_bias = {0.123, -0.045, 0.067};
    assert(update_config_file(new_bias, test_config));
    
    // Read back and verify
    std::ifstream config_in(test_config);
    std::string line;
    bool found_correct_bias = false;
    
    while (std::getline(config_in, line)) {
        if (line.find("accel_bias:") != std::string::npos) {
            // Check if the line contains our bias values
            if (line.find("0.123") != std::string::npos &&
                line.find("-0.045") != std::string::npos &&
                line.find("0.067") != std::string::npos) {
                found_correct_bias = true;
            }
        }
    }
    config_in.close();
    
    assert(found_correct_bias);
    
    std::cout << "  ✓ config file update test passed" << std::endl;
}

int main() {
    std::cout << "Running Calibration tests..." << std::endl;
    std::cout << "==============================" << std::endl;
    
    try {
        test_compute_mean();
        test_compute_bias_perfect_data();
        test_compute_bias_with_constant_bias();
        test_compute_bias_asymmetric();
        test_config_file_update();
        
        std::cout << "==============================" << std::endl;
        std::cout << "All calibration tests passed! ✓" << std::endl;
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Test failed with exception: " << e.what() << std::endl;
        return 1;
    }
}
