#pragma once
#include <array>
#include <cstdint>
#include <atomic>
#include <thread>
#include <memory>
#include "imu/messages/nng_imu_sockets.hpp"


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
    float mag_dec_;
    bool mag_calibration_loaded_;

    // Accelerometer calibration
    std::array<float, 3> accel_bias_;
    std::array<float, 9> accel_matrix_;
    bool accel_calibration_loaded_;

    // Gyroscope calibration
    std::array<float, 3> gyro_bias_;
    std::array<float, 9> gyro_matrix_;
    bool gyro_calibration_loaded_;
    
    // IMU-to-body frame rotation matrix
    std::array<float, 9> bdy_R_imu_;
    bool imu_rotation_loaded_;

    double sum_gx_;
    double sum_gy_;
    double sum_gz_;
    int sample_count_;   

    // Overall calibration loaded flag
    bool calibration_loaded_;
    bool stationary_gyro_cal_;

    // NNG subscriber sockets for inter-process communication
    std::unique_ptr<NngRawGyroSocket> nng_gyro_sub_;
    std::unique_ptr<NngRawAccelSocket> nng_accel_sub_;
    std::unique_ptr<NngRawMagSocket> nng_mag_sub_;
    std::unique_ptr<NngRawTempSocket> nng_temp_sub_;

    // NNG publisher sockets for inter-process communication
    std::unique_ptr<NngProcGyroSocket> nng_proc_gyro_pub_;
    std::unique_ptr<NngProcAccelSocket> nng_proc_accel_pub_;
    std::unique_ptr<NngProcMagSocket> nng_proc_mag_pub_;
    std::unique_ptr<NngProcTempSocket> nng_proc_temp_pub_;

    // Calibration application methods
    void estimate_gyro_bias();
    void apply_mag_calibration(const std::array<float, 3> &m_raw, std::array<float, 3> &m_cal);
    void apply_accel_calibration(const std::array<float, 3> &a_raw, std::array<float, 3> &a_cal);
    void apply_gyro_calibration(const std::array<float, 3> &g_raw, std::array<float, 3> &g_cal);
    
    // Rotate vector from IMU frame to body frame
    void rotate_imu_to_bdy(const std::array<float, 3> &v_imu, std::array<float, 3> &v_bdy);

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
    // Returns true if fresh data was received
    bool get_gyro_measurement(float &gx, float &gy, float &gz);

    // Get accelerometer measurement with calibration applied
    // Returns true if fresh data was received
    bool get_accel_measurement(float &ax, float &ay, float &az);

    // Get magnetometer measurement with calibration applied
    // Returns true if fresh data was received
    bool get_mag_measurement(float &mx, float &my, float &mz);

    // Load calibration parameters from config
    void load_mag_calibration(const std::array<float, 3>& bias, const std::array<float, 9>& matrix);
    
    // Read magnetometer with calibration applied
    bool read_mag_calibrated(float &x, float &y, float &z);

    // Load calibration parameters from config
    void load_accel_calibration(const std::array<float, 3>& bias, const std::array<float, 9>& matrix);
    
    // Read magnetometer with calibration applied
    bool read_accel_calibrated(float &x, float &y, float &z);
};