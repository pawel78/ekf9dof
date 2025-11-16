#pragma once
#include <array>
#include <cstdint>

namespace imu::processing {
    // Magnetometer calibration functions
    
    // Load calibration parameters from config
    void load_mag_calibration(const std::array<float, 3>& bias, const std::array<float, 9>& matrix);
    
    // Read magnetometer with calibration applied
    bool read_mag_calibrated(float &x, float &y, float &z);

    // Get current calibration parameters
    void get_mag_calibration(std::array<float, 3>& bias, std::array<float, 9>& matrix);

    // Accelerometer calibration functions

    // Load calibration parameters from config
    void load_accel_calibration(const std::array<float, 3>& bias, const std::array<float, 9>& matrix);
    
    // Read magnetometer with calibration applied
    bool read_accel_calibrated(float &x, float &y, float &z);

    // Get current calibration parameters
    void get_accel_calibration(std::array<float, 3>& bias, std::array<float, 9>& matrix);
}