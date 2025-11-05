#include <iostream>
#include <fstream>
#include <vector>
#include <array>
#include <cmath>
#include <chrono>
#include <thread>
#include <iomanip>
#include <sstream>
#include <string>
#include "lsm9ds0_device.hpp"
#include "mag_calibration.hpp"

namespace mag_calibration {

// Helper function to collect magnetometer samples
std::vector<std::array<float, 3>> collect_samples(int num_samples, int sample_rate_ms) {
    std::vector<std::array<float, 3>> samples;
    samples.reserve(num_samples);
    
    std::cout << "  Collecting " << num_samples << " samples";
    std::cout.flush();
    
    for (int i = 0; i < num_samples; ++i) {
        int16_t mx, my, mz;
        if (lsm9ds0_device::read_mag(mx, my, mz)) {
            std::array<float, 3> sample = {
                lsm9ds0_device::raw_to_gauss(mx),
                lsm9ds0_device::raw_to_gauss(my),
                lsm9ds0_device::raw_to_gauss(mz)
            };
            samples.push_back(sample);
            
            // Progress indicator
            if ((i + 1) % 10 == 0) {
                std::cout << ".";
                std::cout.flush();
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(sample_rate_ms));
    }
    
    std::cout << " Done!\n";
    return samples;
}

// Calculate min/max for each axis to determine hard iron offset
void calculate_hard_iron_offset(const std::vector<std::array<float, 3>>& samples,
                                 std::array<float, 3>& offset) {
    if (samples.empty()) {
        offset = {0.0f, 0.0f, 0.0f};
        return;
    }
    
    std::array<float, 3> min_vals = samples[0];
    std::array<float, 3> max_vals = samples[0];
    
    for (const auto& sample : samples) {
        for (size_t i = 0; i < 3; ++i) {
            min_vals[i] = std::min(min_vals[i], sample[i]);
            max_vals[i] = std::max(max_vals[i], sample[i]);
        }
    }
    
    // Hard iron offset is the midpoint
    for (size_t i = 0; i < 3; ++i) {
        offset[i] = (max_vals[i] + min_vals[i]) / 2.0f;
    }
}

// Calculate soft iron correction matrix using ellipsoid fitting
void calculate_soft_iron_matrix(const std::vector<std::array<float, 3>>& samples,
                                 const std::array<float, 3>& hard_iron_offset,
                                 std::array<float, 9>& soft_iron_matrix) {
    // For simplicity, we'll use a diagonal scaling matrix approach
    // A full ellipsoid fit would require more complex linear algebra
    
    if (samples.empty()) {
        // Identity matrix
        soft_iron_matrix = {1.0f, 0.0f, 0.0f,
                            0.0f, 1.0f, 0.0f,
                            0.0f, 0.0f, 1.0f};
        return;
    }
    
    // Calculate the range on each axis after hard iron correction
    std::array<float, 3> min_vals = {1e6f, 1e6f, 1e6f};
    std::array<float, 3> max_vals = {-1e6f, -1e6f, -1e6f};
    
    for (const auto& sample : samples) {
        for (size_t i = 0; i < 3; ++i) {
            float corrected = sample[i] - hard_iron_offset[i];
            min_vals[i] = std::min(min_vals[i], corrected);
            max_vals[i] = std::max(max_vals[i], corrected);
        }
    }
    
    // Calculate ranges
    std::array<float, 3> ranges;
    for (size_t i = 0; i < 3; ++i) {
        ranges[i] = max_vals[i] - min_vals[i];
    }
    
    // Use average range as reference
    float avg_range = (ranges[0] + ranges[1] + ranges[2]) / 3.0f;
    
    // Create diagonal scaling matrix
    soft_iron_matrix = {
        avg_range / ranges[0], 0.0f, 0.0f,
        0.0f, avg_range / ranges[1], 0.0f,
        0.0f, 0.0f, avg_range / ranges[2]
    };
}

// Apply calibration to a raw magnetometer reading
void apply_calibration(const std::array<float, 3>& raw,
                       const std::array<float, 3>& hard_iron_offset,
                       const std::array<float, 9>& soft_iron_matrix,
                       std::array<float, 3>& calibrated) {
    // Step 1: Remove hard iron offset
    std::array<float, 3> temp;
    for (size_t i = 0; i < 3; ++i) {
        temp[i] = raw[i] - hard_iron_offset[i];
    }
    
    // Step 2: Apply soft iron correction matrix (3x3 matrix multiplication)
    calibrated[0] = soft_iron_matrix[0] * temp[0] + soft_iron_matrix[1] * temp[1] + soft_iron_matrix[2] * temp[2];
    calibrated[1] = soft_iron_matrix[3] * temp[0] + soft_iron_matrix[4] * temp[1] + soft_iron_matrix[5] * temp[2];
    calibrated[2] = soft_iron_matrix[6] * temp[0] + soft_iron_matrix[7] * temp[1] + soft_iron_matrix[8] * temp[2];
}

// Interactive calibration process
bool run_calibration(const std::string& config_path) {
    std::cout << "\n";
    std::cout << "========================================\n";
    std::cout << "  Magnetometer Calibration Utility\n";
    std::cout << "========================================\n\n";
    
    std::cout << "This process will calibrate the magnetometer for:\n";
    std::cout << "  - Hard iron effects (offset from nearby ferromagnetic materials)\n";
    std::cout << "  - Soft iron effects (distortion from nearby materials)\n\n";
    
    std::cout << "You will be prompted to move the sensor to various orientations.\n";
    std::cout << "For best results, slowly rotate the sensor to cover all orientations.\n\n";
    
    std::vector<std::array<float, 3>> all_samples;
    
    // Position 1: Random orientation - collect baseline
    std::cout << "[1/6] Position 1: Hold sensor in any starting orientation\n";
    std::cout << "Press Enter when ready...";
    std::cin.ignore();
    auto samples1 = collect_samples(50, 20);
    all_samples.insert(all_samples.end(), samples1.begin(), samples1.end());
    
    // Position 2: Rotate around X-axis
    std::cout << "\n[2/6] Slowly rotate sensor around X-axis (roll) for ~10 seconds\n";
    std::cout << "Press Enter when ready...";
    std::cin.ignore();
    auto samples2 = collect_samples(100, 20);
    all_samples.insert(all_samples.end(), samples2.begin(), samples2.end());
    
    // Position 3: Rotate around Y-axis
    std::cout << "\n[3/6] Slowly rotate sensor around Y-axis (pitch) for ~10 seconds\n";
    std::cout << "Press Enter when ready...";
    std::cin.ignore();
    auto samples3 = collect_samples(100, 20);
    all_samples.insert(all_samples.end(), samples3.begin(), samples3.end());
    
    // Position 4: Rotate around Z-axis
    std::cout << "\n[4/6] Slowly rotate sensor around Z-axis (yaw) for ~10 seconds\n";
    std::cout << "Press Enter when ready...";
    std::cin.ignore();
    auto samples4 = collect_samples(100, 20);
    all_samples.insert(all_samples.end(), samples4.begin(), samples4.end());
    
    // Position 5: Random tumbling
    std::cout << "\n[5/6] Slowly tumble sensor in random orientations for ~10 seconds\n";
    std::cout << "Press Enter when ready...";
    std::cin.ignore();
    auto samples5 = collect_samples(100, 20);
    all_samples.insert(all_samples.end(), samples5.begin(), samples5.end());
    
    // Position 6: Figure-8 motion
    std::cout << "\n[6/6] Move sensor in a figure-8 pattern for ~10 seconds\n";
    std::cout << "Press Enter when ready...";
    std::cin.ignore();
    auto samples6 = collect_samples(100, 20);
    all_samples.insert(all_samples.end(), samples6.begin(), samples6.end());
    
    std::cout << "\n========================================\n";
    std::cout << "  Calculating Calibration Parameters\n";
    std::cout << "========================================\n\n";
    
    // Calculate hard iron offset
    std::array<float, 3> hard_iron_offset;
    calculate_hard_iron_offset(all_samples, hard_iron_offset);
    
    std::cout << "Hard iron offset (bias): ["
              << std::fixed << std::setprecision(4)
              << hard_iron_offset[0] << ", "
              << hard_iron_offset[1] << ", "
              << hard_iron_offset[2] << "]\n";
    
    // Calculate soft iron matrix
    std::array<float, 9> soft_iron_matrix;
    calculate_soft_iron_matrix(all_samples, hard_iron_offset, soft_iron_matrix);
    
    std::cout << "Soft iron matrix:\n";
    std::cout << "  [" << std::setw(7) << soft_iron_matrix[0] << ", "
              << std::setw(7) << soft_iron_matrix[1] << ", "
              << std::setw(7) << soft_iron_matrix[2] << "]\n";
    std::cout << "  [" << std::setw(7) << soft_iron_matrix[3] << ", "
              << std::setw(7) << soft_iron_matrix[4] << ", "
              << std::setw(7) << soft_iron_matrix[5] << "]\n";
    std::cout << "  [" << std::setw(7) << soft_iron_matrix[6] << ", "
              << std::setw(7) << soft_iron_matrix[7] << ", "
              << std::setw(7) << soft_iron_matrix[8] << "]\n\n";
    
    // Update config file with simple text replacement
    try {
        // Read the entire config file
        std::ifstream fin(config_path);
        if (!fin.is_open()) {
            throw std::runtime_error("Failed to open config file: " + config_path);
        }
        
        std::stringstream buffer;
        buffer << fin.rdbuf();
        fin.close();
        std::string content = buffer.str();
        
        // Find and replace mag_bias line
        std::ostringstream mag_bias_str;
        mag_bias_str << std::fixed << std::setprecision(6)
                     << "  mag_bias:  [" << hard_iron_offset[0] << ", "
                     << hard_iron_offset[1] << ", " << hard_iron_offset[2] << "]";
        
        size_t bias_pos = content.find("mag_bias:");
        if (bias_pos != std::string::npos) {
            size_t line_end = content.find('\n', bias_pos);
            content.replace(bias_pos, line_end - bias_pos, mag_bias_str.str().substr(2)); // Remove leading spaces
        }
        
        // Find and replace mag_matrix lines
        std::ostringstream mag_matrix_str;
        mag_matrix_str << std::fixed << std::setprecision(6)
                       << "  mag_matrix: [" << soft_iron_matrix[0] << ","
                       << soft_iron_matrix[1] << "," << soft_iron_matrix[2] << ",\n"
                       << "               " << soft_iron_matrix[3] << ","
                       << soft_iron_matrix[4] << "," << soft_iron_matrix[5] << ",\n"
                       << "               " << soft_iron_matrix[6] << ","
                       << soft_iron_matrix[7] << "," << soft_iron_matrix[8] << "]";
        
        size_t matrix_pos = content.find("mag_matrix:");
        if (matrix_pos != std::string::npos) {
            // Find the closing bracket for the matrix
            size_t bracket_pos = content.find(']', matrix_pos);
            if (bracket_pos != std::string::npos) {
                content.replace(matrix_pos, bracket_pos - matrix_pos + 1, mag_matrix_str.str().substr(2));
            }
        }
        
        // Write back to file
        std::ofstream fout(config_path);
        if (!fout.is_open()) {
            throw std::runtime_error("Failed to write to config file: " + config_path);
        }
        fout << content;
        fout.close();
        
        std::cout << "✓ Calibration values saved to: " << config_path << "\n\n";
        
    } catch (const std::exception& e) {
        std::cerr << "Error updating config file: " << e.what() << "\n";
        std::cerr << "Please manually update the config.yaml file with the values above.\n";
        return false;
    }
    
    std::cout << "========================================\n";
    std::cout << "  Calibration Complete!\n";
    std::cout << "========================================\n\n";
    
    return true;
}

} // namespace mag_calibration
