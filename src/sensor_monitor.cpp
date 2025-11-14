#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>
#include <iomanip>
#include <ctime>
#include <string>
#include <vector>
#include <array>
#include <cmath>
#include "lsm9ds0_device.hpp"
#include "config_loader.hpp"

/**
 * @brief Standalone LSM9DS0 Sensor Monitor
 * 
 * This application continuously reads sensor data and displays:
 * - Raw sensor values (uncalibrated)
 * - Calibrated magnetometer values (if calibration available)
 * - Temperature data
 * 
 * Also saves magnetometer calibration data for offline analysis.
 */

class SensorMonitor {
private:
    std::array<float, 3> mag_bias_ = {0.0f, 0.0f, 0.0f};
    std::array<float, 9> mag_matrix_ = {1.0f, 0.0f, 0.0f,
                                        0.0f, 1.0f, 0.0f,
                                        0.0f, 0.0f, 1.0f};
    bool calibration_loaded_ = false;
    std::ofstream sensor_data_log_;
    
public:
    bool initialize() {
        std::cout << "Initializing LSM9DS0 Sensor Monitor...\n";
        
        // Configure the IMU
        try {
            lsm9ds0_device::configure_imu();
            lsm9ds0_device::configure_temperature_sensor();
            std::cout << "✓ IMU configured successfully\n";
        } catch (const std::exception& e) {
            std::cerr << "✗ Failed to configure IMU: " << e.what() << std::endl;
            return false;
        }
        
        // Verify device IDs
        if (!lsm9ds0_device::verify_device_ids()) {
            std::cerr << "✗ Device ID verification failed\n";
            return false;
        }
        std::cout << "✓ Device IDs verified\n";
        
        // Try to load magnetometer calibration
        if (config_loader::load_mag_calibration("../configs/config.yaml", mag_bias_, mag_matrix_)) {
            lsm9ds0_device::load_mag_calibration(mag_bias_, mag_matrix_);
            calibration_loaded_ = true;
            std::cout << "✓ Magnetometer calibration loaded\n";
            std::cout << "  Bias: [" << mag_bias_[0] << ", " << mag_bias_[1] << ", " << mag_bias_[2] << "]\n";
        } else {
            std::cout << "⚠ No magnetometer calibration found (using identity)\n";
        }
        
        // Open calibration data log file
        auto now = std::chrono::system_clock::now();
        auto now_time_t = std::chrono::system_clock::to_time_t(now);
        std::tm tm_buf;
        localtime_r(&now_time_t, &tm_buf);
        
        char sensor_data_filename[64];
        std::strftime(sensor_data_filename, sizeof(sensor_data_filename), "sensor_data_%Y%m%d_%H%M%S.csv", &tm_buf);
        
        sensor_data_log_.open(sensor_data_filename);
        if (sensor_data_log_.is_open()) {
            sensor_data_log_ << "timestamp_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,mx_raw_gauss,my_raw_gauss,mz_raw_gauss\n";
            std::cout << "✓ Sensor data logging to: " << sensor_data_filename << "\n";
        }
        
        return true;
    }
    
    void display_header() {
        std::cout << "\n" << std::string(120, '=') << "\n";
        std::cout << "LSM9DS0 SENSOR MONITOR - Real-time Display\n";
        std::cout << std::string(120, '=') << "\n";
        std::cout << std::left;
        std::cout << std::setw(12) << "Time"
                  << std::setw(30) << "Accelerometer (g)"
                  << std::setw(30) << "Gyroscope (dps)"
                  << std::setw(35) << "Magnetometer Raw (gauss)" << "\n";
        std::cout << std::string(120, '-') << "\n";
    }
    
    void apply_mag_calibration(float mx_raw, float my_raw, float mz_raw,
                              float& mx_cal, float& my_cal, float& mz_cal) {
        if (!calibration_loaded_) {
            mx_cal = mx_raw;
            my_cal = my_raw;
            mz_cal = mz_raw;
            return;
        }
        
        // Apply hard iron correction (subtract bias)
        float mx_bias_corrected = mx_raw - mag_bias_[0];
        float my_bias_corrected = my_raw - mag_bias_[1];
        float mz_bias_corrected = mz_raw - mag_bias_[2];
        
        // Apply soft iron correction (matrix multiplication)
        mx_cal = mag_matrix_[0] * mx_bias_corrected + mag_matrix_[1] * my_bias_corrected + mag_matrix_[2] * mz_bias_corrected;
        my_cal = mag_matrix_[3] * mx_bias_corrected + mag_matrix_[4] * my_bias_corrected + mag_matrix_[5] * mz_bias_corrected;
        mz_cal = mag_matrix_[6] * mx_bias_corrected + mag_matrix_[7] * my_bias_corrected + mag_matrix_[8] * mz_bias_corrected;
    }
    
    void run_monitor() {
        display_header();
        
        auto start_time = std::chrono::steady_clock::now();
        const auto update_interval = std::chrono::milliseconds(200); // 5 Hz display update
        int sample_count = 0;
        
        while (true) {
            auto current_time = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time);
            
            // Read all sensors
            int16_t ax_raw, ay_raw, az_raw;
            int16_t gx_raw, gy_raw, gz_raw;
            int16_t mx_raw, my_raw, mz_raw;
            int16_t temp_raw;
            
            bool accel_ok = lsm9ds0_device::read_accel(ax_raw, ay_raw, az_raw);
            bool gyro_ok = lsm9ds0_device::read_gyro(gx_raw, gy_raw, gz_raw);
            bool mag_ok = lsm9ds0_device::read_mag(mx_raw, my_raw, mz_raw);
            bool temp_ok = lsm9ds0_device::read_temperature(temp_raw);
            
            if (!accel_ok || !gyro_ok || !mag_ok || !temp_ok) {
                std::cout << "⚠ Sensor read error - retrying...\n";
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
            
            // Convert to engineering units
            float ax_g = lsm9ds0_device::raw_to_g(ax_raw);
            float ay_g = lsm9ds0_device::raw_to_g(ay_raw);
            float az_g = lsm9ds0_device::raw_to_g(az_raw);
            
            float gx_dps = lsm9ds0_device::raw_to_dps(gx_raw);
            float gy_dps = lsm9ds0_device::raw_to_dps(gy_raw);
            float gz_dps = lsm9ds0_device::raw_to_dps(gz_raw);
            
            float mx_gauss = lsm9ds0_device::raw_to_gauss(mx_raw);
            float my_gauss = lsm9ds0_device::raw_to_gauss(my_raw);
            float mz_gauss = lsm9ds0_device::raw_to_gauss(mz_raw);
            
            float temp_c = lsm9ds0_device::raw_to_celsius(temp_raw);
            
            // Apply magnetometer calibration
            float mx_cal, my_cal, mz_cal;
            apply_mag_calibration(mx_gauss, my_gauss, mz_gauss, mx_cal, my_cal, mz_cal);
            
            // Display data
            std::cout << std::fixed << std::setprecision(3);
            std::cout << std::setw(12) << (elapsed.count() / 1000.0)
                        << "("
                        << std::setw(7) << std::fixed << std::setprecision(3) << ax_g << ", "
                        << std::setw(7) << ay_g << ", "
                        << std::setw(7) << az_g << ")   "
                        << "("
                        << std::setw(7) << std::fixed << std::setprecision(3) << gx_dps << ", "
                        << std::setw(7) << gy_dps << ", "
                        << std::setw(7) << gz_dps << ")   "
                        << "("
                        << std::setw(9) << std::fixed << std::setprecision(4) << mx_gauss << ", "
                        << std::setw(9) << my_gauss << ", "
                        << std::setw(9) << mz_gauss << ")"
                        << "\n";
            
            // Log sensor data for offline analysis
            if (sensor_data_log_.is_open()) {
                sensor_data_log_ << elapsed.count() << ","
                                << ax_g << "," << ay_g << "," << az_g << ","
                                << gx_dps << "," << gy_dps << "," << gz_dps << ","
                                << mx_gauss << "," << my_gauss << "," << mz_gauss << "\n";
                
                // Flush every 50 samples
                if (++sample_count % 50 == 0) {
                    sensor_data_log_.flush();
                }
            }
            
            // Redraw header every 50 lines for readability
            if (sample_count % 50 == 0) {
                display_header();
            }
            
            std::this_thread::sleep_for(update_interval);
        }
    }
    
    void display_calibration_info() {
        std::cout << "\n" << std::string(80, '=') << "\n";
        std::cout << "MAGNETOMETER CALIBRATION STATUS\n";
        std::cout << std::string(80, '=') << "\n";
        
        if (calibration_loaded_) {
            std::cout << "✓ Calibration ACTIVE\n\n";
            
            std::cout << "Hard Iron Bias (gauss):\n";
            std::cout << "  X: " << std::setw(10) << std::fixed << std::setprecision(6) << mag_bias_[0] << "\n";
            std::cout << "  Y: " << std::setw(10) << std::fixed << std::setprecision(6) << mag_bias_[1] << "\n";
            std::cout << "  Z: " << std::setw(10) << std::fixed << std::setprecision(6) << mag_bias_[2] << "\n\n";
            
            std::cout << "Soft Iron Correction Matrix:\n";
            std::cout << "  [" << std::setw(8) << std::fixed << std::setprecision(4) << mag_matrix_[0] 
                     << " " << std::setw(8) << mag_matrix_[1] 
                     << " " << std::setw(8) << mag_matrix_[2] << "]\n";
            std::cout << "  [" << std::setw(8) << std::fixed << std::setprecision(4) << mag_matrix_[3] 
                     << " " << std::setw(8) << mag_matrix_[4] 
                     << " " << std::setw(8) << mag_matrix_[5] << "]\n";
            std::cout << "  [" << std::setw(8) << std::fixed << std::setprecision(4) << mag_matrix_[6] 
                     << " " << std::setw(8) << mag_matrix_[7] 
                     << " " << std::setw(8) << mag_matrix_[8] << "]\n";
        } else {
            std::cout << "⚠ No calibration loaded - using raw values\n";
            std::cout << "\nTo calibrate magnetometer:\n";
            std::cout << "1. Run: ./calibrate_mag\n";
            std::cout << "2. Follow the guided calibration process\n";
            std::cout << "3. Restart this monitor\n";
        }
        std::cout << std::string(80, '=') << "\n\n";
    }
    
    ~SensorMonitor() {
        if (sensor_data_log_.is_open()) {
            sensor_data_log_.close();
            std::cout << "\n✓ Calibration data log closed\n";
        }
    }
};

void print_usage(const char* program_name) {
    std::cout << "LSM9DS0 Sensor Monitor\n";
    std::cout << "======================\n\n";
    std::cout << "Usage: " << program_name << " [options]\n\n";
    std::cout << "Options:\n";
    std::cout << "  -h, --help     Show this help message\n";
    std::cout << "  -i, --info     Show calibration status and exit\n\n";
    std::cout << "Description:\n";
    std::cout << "  Continuously reads LSM9DS0 sensor data and displays:\n";
    std::cout << "  - Raw accelerometer, gyroscope, magnetometer readings\n";
    std::cout << "  - Calibrated magnetometer readings (if calibration available)\n";
    std::cout << "  - Temperature data\n\n";
    std::cout << "  Magnetometer calibration data is logged to a CSV file\n";
    std::cout << "  for offline analysis and visualization.\n\n";
    std::cout << "  Press Ctrl+C to exit.\n\n";
}

int main(int argc, char* argv[]) {
    // Parse command line arguments
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            print_usage(argv[0]);
            return 0;
        } else if (arg == "-i" || arg == "--info") {
            SensorMonitor monitor;
            if (monitor.initialize()) {
                monitor.display_calibration_info();
            }
            return 0;
        }
    }
    
    SensorMonitor monitor;
    
    if (!monitor.initialize()) {
        std::cerr << "Failed to initialize sensor monitor\n";
        return 1;
    }
    
    std::cout << "\nStarting sensor monitor...\n";
    std::cout << "Press Ctrl+C to exit\n";
    
    try {
        monitor.run_monitor();
    } catch (const std::exception& e) {
        std::cerr << "\nError: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "\nUnknown error occurred\n";
        return 1;
    }
    
    return 0;
}