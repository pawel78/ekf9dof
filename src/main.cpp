#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>
#include <iomanip>
#include <ctime>
#include <limits>
#include "lsm9ds0_device.hpp"

// Poll sensor at 200 Hz and log data
void run_sensor_polling() {
    std::cout << "LSM9DS0 configured. Starting 200 Hz polling loop...\n";
    
    // Generate timestamped filename
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    std::tm tm_buf;
    localtime_r(&now_time_t, &tm_buf);
    
    char filename[64];
    std::strftime(filename, sizeof(filename), "sensor_data_%Y%m%d_%H%M%S.csv", &tm_buf);
    
    // Open log file
    std::ofstream log_file(filename);
    if (!log_file.is_open()) {
        throw std::runtime_error("Failed to open log file");
    }
    
    std::cout << "Logging to: " << filename << "\n";
    
    // Write CSV header
    log_file << "timestamp_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,mx_gauss,my_gauss,mz_gauss,temp_c\n";
    
    auto start_time = std::chrono::steady_clock::now();
    const auto interval = std::chrono::microseconds(5000); // 5ms = 200 Hz
    auto next_time = start_time + interval;
    
    int flush_counter = 0;
    const int FLUSH_INTERVAL = 200; // Flush every 200 samples (1 second at 200 Hz)
    
    while (true) {
        int16_t ax, ay, az;
        int16_t gx, gy, gz;
        int16_t mx, my, mz;
        int16_t temp;

        bool aok = lsm9ds0_device::read_accel(ax, ay, az);
        bool gok = lsm9ds0_device::read_gyro(gx, gy, gz);
        bool mok = lsm9ds0_device::read_mag(mx, my, mz);
        bool tok = lsm9ds0_device::read_temperature(temp);

        // Calculate timestamp in milliseconds since start
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time);
        
        // Log data to CSV file (use NaN for failed reads)
        const float nan_val = std::numeric_limits<float>::quiet_NaN();
        log_file << elapsed.count() << ","
                 << (aok ? lsm9ds0_device::raw_to_g(ax) : nan_val) << ","
                 << (aok ? lsm9ds0_device::raw_to_g(ay) : nan_val) << ","
                 << (aok ? lsm9ds0_device::raw_to_g(az) : nan_val) << ","
                 << (gok ? lsm9ds0_device::raw_to_dps(gx) : nan_val) << ","
                 << (gok ? lsm9ds0_device::raw_to_dps(gy) : nan_val) << ","
                 << (gok ? lsm9ds0_device::raw_to_dps(gz) : nan_val) << ","
                 << (mok ? lsm9ds0_device::raw_to_gauss(mx) : nan_val) << ","
                 << (mok ? lsm9ds0_device::raw_to_gauss(my) : nan_val) << ","
                 << (mok ? lsm9ds0_device::raw_to_gauss(mz) : nan_val) << ","
                 << (tok ? lsm9ds0_device::raw_to_celsius(temp) : nan_val) << "\n";
        
        // Flush periodically to ensure data is written (use counter instead of modulo)
        flush_counter++;
        if (flush_counter >= FLUSH_INTERVAL) {
            log_file.flush();
            flush_counter = 0;
        }
        
        // Sleep until next scheduled time
        std::this_thread::sleep_until(next_time);
        next_time += interval;
    }
}

int main() {
    try {
        // Verify device by reading WHO_AM_I registers
        if (!lsm9ds0_device::verify_device_ids()) {
            std::cerr << "Error: LSM9DS0 device not found or incorrect WHO_AM_I response.\n";
            return 1;
        }

        // Configure IMU
        lsm9ds0_device::configure_imu();

        // Configure temperature sensor
        lsm9ds0_device::configure_temperature_sensor();

        run_sensor_polling();

    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}