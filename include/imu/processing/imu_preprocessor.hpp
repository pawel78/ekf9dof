#pragma once
#include <array>
#include <cstdint>
#include <atomic>
#include <thread>


class IMUPreprocessor
{
private:
   // Member variables
    std::atomic<bool> running_;
    std::thread preprocessor_thread_;

    // Calibration state
    std::array<float, 3> ym_;
    std::array<float, 3> ya_;
    std::array<float, 3> yg_;

    // Magnetometer calibration
    std::array<float, 3> mag_bias_;
    std::array<float, 9> mag_matrix_;
    bool mag_calibration_loaded_;

    // Accelerometer calibration
    std::array<float, 3> accel_bias_;
    std::array<float, 9> accel_matrix_;
    bool accel_calibration_loaded_;

    // Gyroscope calibration
    std::array<float, 3> gyro_bias_;
    std::array<float, 9> gyro_matrix_;
    bool gyro_calibration_loaded_;

    // Overall calibration loaded flag
    bool calibration_loaded_;

    // Calibration application methods
    void apply_mag_calibration(float mx_raw, float my_raw, float mz_raw, float &mx_cal, float &my_cal, float &mz_cal);
    void apply_accel_calibration(float ax_raw, float ay_raw, float az_raw, float &ax_cal, float &ay_cal, float &az_cal);
    void apply_gyro_calibration(float gx_raw, float gy_raw, float gz_raw, float &gx_cal, float &gy_cal, float &gz_cal);

    // Static thread function (same pattern as driver)
    static void preprocessor_thread_func(IMUPreprocessor* preprocessor);

public:
    IMUPreprocessor(); // Constructor
    ~IMUPreprocessor(); // Destructor

    // Main processing loop
    void start();

    // Stop processing loop
    void stop();

    // Load magnetometer calibration parameters
    void get_mag_calibration(std::array<float, 3> &bias, std::array<float, 9> &matrix);

    // Get accelerometer calibration parameters
    void get_accel_calibration(std::array<float, 3> &bias, std::array<float, 9> &matrix);

    // Get gyroscope calibration parameters
    void get_gyro_calibration(std::array<float, 3> &bias, std::array<float, 9> &matrix);
    
    // Get gyroscope measurement with calibration applied
    void get_gyro_measurement(float &gx, float &gy, float &gz);

    // Get accelerometer measurement with calibration applied
    void get_accel_measurement(float &ax, float &ay, float &az);

    // Get magnetometer measurement with calibration applied
    void get_mag_measurement(float &mx, float &my, float &mz);

    // Load calibration parameters from config
    void load_mag_calibration(const std::array<float, 3>& bias, const std::array<float, 9>& matrix);
    
    // Read magnetometer with calibration applied
    bool read_mag_calibrated(float &x, float &y, float &z);

    // Load calibration parameters from config
    void load_accel_calibration(const std::array<float, 3>& bias, const std::array<float, 9>& matrix);
    
    // Read magnetometer with calibration applied
    bool read_accel_calibrated(float &x, float &y, float &z);
};