#include <iostream>
#include <fstream>
#include <cassert>
#include <cmath>
#include <array>
#include "../include/config_loader.hpp"

// Helper function to compare floats
bool float_equal(float a, float b, float epsilon = 1e-6f) {
    return std::fabs(a - b) < epsilon;
}

void test_load_default_config() {
    std::cout << "Testing loading from default config.yaml..." << std::endl;
    
    std::array<float, 3> mag_bias;
    std::array<float, 9> mag_matrix;
    
    // Try multiple possible paths
    bool loaded = false;
    const char* paths[] = {
        "../configs/config.yaml",
        "configs/config.yaml",
        "../../configs/config.yaml"
    };
    
    for (const char* path : paths) {
        loaded = config_loader::load_mag_calibration(path, mag_bias, mag_matrix);
        if (loaded) {
            std::cout << "  Found config at: " << path << std::endl;
            break;
        }
    }
    
    if (!loaded) {
        std::cout << "  ⚠ Skipping test - config.yaml not found (OK for CI environment)" << std::endl;
        return;
    }
    
    // Check that values were loaded (default config has zeros and identity)
    std::cout << "  Loaded mag_bias: [" << mag_bias[0] << ", " << mag_bias[1] << ", " << mag_bias[2] << "]" << std::endl;
    std::cout << "  Loaded mag_matrix: [" << mag_matrix[0] << ", " << mag_matrix[1] << ", " << mag_matrix[2] << ", ...]" << std::endl;
    
    // Default config should have identity matrix
    assert(float_equal(mag_matrix[0], 1.0f));
    assert(float_equal(mag_matrix[4], 1.0f));
    assert(float_equal(mag_matrix[8], 1.0f));
    
    std::cout << "  ✓ Default config loading test passed" << std::endl;
}

void test_load_custom_config() {
    std::cout << "Testing loading from custom config file..." << std::endl;
    
    // Create a temporary config file (use /tmp on Unix-like systems, current dir on Windows)
    const char* temp_config = "test_config_temp.yaml";
    std::ofstream file(temp_config);
    file << "calibration:\n";
    file << "  gyro_bias: [0.0, 0.0, 0.0]\n";
    file << "  accel_bias: [0.0, 0.0, 0.0]\n";
    file << "  accel_scale: [1.0, 1.0, 1.0]\n";
    file << "  mag_bias:  [0.123, -0.456, 0.789]\n";
    file << "  mag_matrix: [1.1,0.0,0.0,\n";
    file << "               0.0,0.9,0.0,\n";
    file << "               0.0,0.0,1.05]\n";
    file.close();
    
    std::array<float, 3> mag_bias;
    std::array<float, 9> mag_matrix;
    
    bool loaded = config_loader::load_mag_calibration(temp_config, mag_bias, mag_matrix);
    
    assert(loaded);
    
    // Check loaded values
    assert(float_equal(mag_bias[0], 0.123f));
    assert(float_equal(mag_bias[1], -0.456f));
    assert(float_equal(mag_bias[2], 0.789f));
    
    assert(float_equal(mag_matrix[0], 1.1f));
    assert(float_equal(mag_matrix[4], 0.9f));
    assert(float_equal(mag_matrix[8], 1.05f));
    
    std::cout << "  ✓ Custom config loading test passed" << std::endl;
}

void test_load_nonexistent_file() {
    std::cout << "Testing loading from non-existent file..." << std::endl;
    
    std::array<float, 3> mag_bias;
    std::array<float, 9> mag_matrix;
    
    bool loaded = config_loader::load_mag_calibration("/nonexistent/file.yaml", mag_bias, mag_matrix);
    
    assert(!loaded);
    
    std::cout << "  ✓ Non-existent file test passed" << std::endl;
}

int main() {
    std::cout << "Running config loader tests..." << std::endl;
    std::cout << "==============================" << std::endl;
    
    try {
        test_load_default_config();
        test_load_custom_config();
        test_load_nonexistent_file();
        
        std::cout << "==============================" << std::endl;
        std::cout << "All config loader tests passed! ✓" << std::endl;
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Test failed with exception: " << e.what() << std::endl;
        return 1;
    }
}
