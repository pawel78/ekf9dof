/**
 * @file calibrate_accel.cpp
 * @brief Accelerometer bias calibration utility
 * 
 * This utility prompts the user to place the IMU in 6 different orientations
 * (+X, -X, +Y, -Y, +Z, -Z facing up) and collects accelerometer data.
 * It then computes the optimal bias estimates using least squares method
 * and updates the config.yaml file with the computed values.
 */

#include <iostream>
#include <fstream>
#include <string>
#include <array>
#include <vector>
#include <numeric>
#include <cmath>
#include <thread>
#include <chrono>
#include <limits>
#include "lsm9ds0_device.hpp"

// Configuration
constexpr int SAMPLES_PER_ORIENTATION = 100;
// Note: Sample rate should be lower than or equal to the IMU's configured rate
// The IMU is configured to 380 Hz in lsm9ds0_config.hpp, so 100 Hz is safe
constexpr int SAMPLE_RATE_HZ = 100;
constexpr double GRAVITY = 9.80665; // m/s^2
constexpr const char* CONFIG_FILE = "configs/config.yaml";

struct AccelData {
    double x, y, z;
};

/**
 * @brief Collect accelerometer samples at current orientation
 */
std::vector<AccelData> collect_samples(int num_samples) {
    std::vector<AccelData> samples;
    samples.reserve(num_samples);
    
    const auto interval = std::chrono::microseconds(1000000 / SAMPLE_RATE_HZ);
    auto next_time = std::chrono::steady_clock::now() + interval;
    
    std::cout << "Collecting " << num_samples << " samples";
    for (int i = 0; i < num_samples; ++i) {
        int16_t ax, ay, az;
        if (lsm9ds0_device::read_accel(ax, ay, az)) {
            AccelData data;
            data.x = lsm9ds0_device::raw_to_g(ax);
            data.y = lsm9ds0_device::raw_to_g(ay);
            data.z = lsm9ds0_device::raw_to_g(az);
            samples.push_back(data);
        }
        
        if (i % 10 == 0) {
            std::cout << "." << std::flush;
        }
        
        std::this_thread::sleep_until(next_time);
        next_time += interval;
    }
    std::cout << " Done!\n";
    
    return samples;
}

/**
 * @brief Compute mean of accelerometer samples
 */
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

/**
 * @brief Prompt user and collect data for one orientation
 */
AccelData collect_orientation_data(const std::string& orientation_name, 
                                   const std::string& instruction) {
    std::cout << "\n===========================================\n";
    std::cout << "Orientation: " << orientation_name << "\n";
    std::cout << instruction << "\n";
    std::cout << "Press ENTER when ready...";
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    
    std::cout << "Stabilizing (2 seconds)...\n";
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    auto samples = collect_samples(SAMPLES_PER_ORIENTATION);
    auto mean = compute_mean(samples);
    
    std::cout << "Mean reading: [" << mean.x << ", " << mean.y << ", " << mean.z << "] g\n";
    
    return mean;
}

/**
 * @brief Compute accelerometer bias using least squares
 * 
 * We assume that in each orientation, one axis should measure ±1g (normalized gravity)
 * and the other two should measure 0g. The bias is what causes deviation from this ideal.
 * 
 * For each orientation k with gravity vector g_k and measurement m_k:
 *   m_k = a_true_k + bias
 *   a_true_k = g_k (since only gravity acts on accelerometer)
 * 
 * Therefore: bias = m_k - g_k for each measurement
 * We average across all 6 orientations to get the best estimate.
 */
std::array<double, 3> compute_bias(const std::vector<std::pair<AccelData, AccelData>>& measurements) {
    // measurements contains pairs of (measured, expected_gravity)
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

/**
 * @brief Update config.yaml file with new bias values
 */
bool update_config_file(const std::array<double, 3>& bias) {
    std::ifstream config_in(CONFIG_FILE);
    if (!config_in.is_open()) {
        std::cerr << "Error: Could not open config file: " << CONFIG_FILE << "\n";
        return false;
    }
    
    std::vector<std::string> lines;
    std::string line;
    bool found_accel_bias = false;
    
    while (std::getline(config_in, line)) {
        if (line.find("accel_bias:") != std::string::npos) {
            // Replace the accel_bias line
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
        std::cerr << "Error: Could not find 'accel_bias:' line in config file\n";
        return false;
    }
    
    // Write back to file
    std::ofstream config_out(CONFIG_FILE);
    if (!config_out.is_open()) {
        std::cerr << "Error: Could not write to config file: " << CONFIG_FILE << "\n";
        return false;
    }
    
    for (const auto& l : lines) {
        config_out << l << "\n";
    }
    config_out.close();
    
    return true;
}

int main() {
    std::cout << "==========================================\n";
    std::cout << "  Accelerometer Bias Calibration Tool\n";
    std::cout << "==========================================\n\n";
    
    std::cout << "This tool will guide you through calibrating the accelerometer bias.\n";
    std::cout << "You will need to place the IMU in 6 different orientations.\n";
    std::cout << "Make sure the IMU is on a level surface for each orientation.\n\n";
    
    try {
        // Verify device
        if (!lsm9ds0_device::verify_device_ids()) {
            std::cerr << "Error: LSM9DS0 device not found or incorrect WHO_AM_I response.\n";
            return 1;
        }
        
        // Configure IMU
        lsm9ds0_device::configure_imu();
        
        std::cout << "IMU configured successfully.\n";
        std::cout << "Press ENTER to start calibration...";
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        
        // Collect data for all 6 orientations
        std::vector<std::pair<AccelData, AccelData>> measurements;
        
        // +Z up (normal position)
        auto mean = collect_orientation_data(
            "+Z UP (Normal)", 
            "Place the IMU flat with Z-axis pointing UP."
        );
        measurements.push_back({mean, {0.0, 0.0, 1.0}});
        
        // -Z up (upside down)
        mean = collect_orientation_data(
            "-Z UP (Upside Down)", 
            "Flip the IMU so Z-axis points DOWN (upside down)."
        );
        measurements.push_back({mean, {0.0, 0.0, -1.0}});
        
        // +X up (right side up)
        mean = collect_orientation_data(
            "+X UP (Right Side)", 
            "Place the IMU on its right side with X-axis pointing UP."
        );
        measurements.push_back({mean, {1.0, 0.0, 0.0}});
        
        // -X up (left side up)
        mean = collect_orientation_data(
            "-X UP (Left Side)", 
            "Place the IMU on its left side with X-axis pointing DOWN."
        );
        measurements.push_back({mean, {-1.0, 0.0, 0.0}});
        
        // +Y up (front up)
        mean = collect_orientation_data(
            "+Y UP (Front Up)", 
            "Place the IMU on its back with Y-axis pointing UP."
        );
        measurements.push_back({mean, {0.0, 1.0, 0.0}});
        
        // -Y up (back up)
        mean = collect_orientation_data(
            "-Y UP (Back Up)", 
            "Place the IMU on its front with Y-axis pointing DOWN."
        );
        measurements.push_back({mean, {0.0, -1.0, 0.0}});
        
        // Compute bias
        std::cout << "\n===========================================\n";
        std::cout << "Computing bias...\n";
        auto bias = compute_bias(measurements);
        
        std::cout << "\nComputed accelerometer bias:\n";
        std::cout << "  X: " << bias[0] << " g\n";
        std::cout << "  Y: " << bias[1] << " g\n";
        std::cout << "  Z: " << bias[2] << " g\n";
        
        // Update config file
        std::cout << "\nUpdating config file: " << CONFIG_FILE << "\n";
        if (update_config_file(bias)) {
            std::cout << "SUCCESS! Config file updated with new bias values.\n";
            std::cout << "\nCalibration complete. The bias values will be applied on next run.\n";
            return 0;
        } else {
            std::cerr << "Failed to update config file.\n";
            return 1;
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
    
    return 0;
}
