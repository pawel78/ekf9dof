#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>
#include <iomanip>
#include <ctime>
#include "lsm9ds0_device.hpp"

#ifdef ENABLE_SENSOR_TEST
// Test mode: Display sensor values to screen for checking if sensor is operational
void run_sensor_test() {
    std::cout << "LSM9DS0 configured. Running sensor test (displaying values)...\n";
    
    while (true) {
        int16_t ax, ay, az;
        int16_t gx, gy, gz;
        int16_t mx, my, mz;
        int16_t temp;

        bool aok = lsm9ds0_device::read_accel(ax, ay, az);
        bool gok = lsm9ds0_device::read_gyro(gx, gy, gz);
        bool mok = lsm9ds0_device::read_mag(mx, my, mz);
        bool tok = lsm9ds0_device::read_temperature(temp);

        if (aok) {
            std::cout << "ACC: " << lsm9ds0_device::raw_to_g(ax) << " g, " 
                      << lsm9ds0_device::raw_to_g(ay) << " g, " 
                      << lsm9ds0_device::raw_to_g(az) << " g\n";
        } else {
            std::cout << "ACC: read error\n";
        }

        if (gok) {
            std::cout << "GYR: " << lsm9ds0_device::raw_to_dps(gx) << " °/s, " 
                      << lsm9ds0_device::raw_to_dps(gy) << " °/s, " 
                      << lsm9ds0_device::raw_to_dps(gz) << " °/s\n";
        } else {
            std::cout << "GYR: read error\n";
        }

        if (mok) {
            std::cout << "MAG: " << lsm9ds0_device::raw_to_gauss(mx) << " gauss, " 
                      << lsm9ds0_device::raw_to_gauss(my) << " gauss, " 
                      << lsm9ds0_device::raw_to_gauss(mz) << " gauss\n";
        } else {
            std::cout << "MAG: read error\n";
        }

        if (tok) {
            std::cout << "TEMP: " << lsm9ds0_device::raw_to_celsius(temp) << " °C\n";
        } else {
            std::cout << "TEMP: read error\n";
        }

        std::cout << "----" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}
#else
// Normal mode: Poll sensor at 200 Hz and log data
void run_sensor_polling() {
    std::cout << "LSM9DS0 configured. Starting 200 Hz polling loop...\n";
    
    // Open log file
    std::ofstream log_file("sensor_data.csv");
    if (!log_file.is_open()) {
        throw std::runtime_error("Failed to open log file");
    }
    
    // Write CSV header
    log_file << "timestamp_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,mx_gauss,my_gauss,mz_gauss,temp_c\n";
    
    auto start_time = std::chrono::steady_clock::now();
    const auto interval = std::chrono::microseconds(5000); // 5ms = 200 Hz
    auto next_time = start_time + interval;
    
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
        
        // Log data to CSV file
        log_file << elapsed.count() << ","
                 << (aok ? lsm9ds0_device::raw_to_g(ax) : 0.0f) << ","
                 << (aok ? lsm9ds0_device::raw_to_g(ay) : 0.0f) << ","
                 << (aok ? lsm9ds0_device::raw_to_g(az) : 0.0f) << ","
                 << (gok ? lsm9ds0_device::raw_to_dps(gx) : 0.0f) << ","
                 << (gok ? lsm9ds0_device::raw_to_dps(gy) : 0.0f) << ","
                 << (gok ? lsm9ds0_device::raw_to_dps(gz) : 0.0f) << ","
                 << (mok ? lsm9ds0_device::raw_to_gauss(mx) : 0.0f) << ","
                 << (mok ? lsm9ds0_device::raw_to_gauss(my) : 0.0f) << ","
                 << (mok ? lsm9ds0_device::raw_to_gauss(mz) : 0.0f) << ","
                 << (tok ? lsm9ds0_device::raw_to_celsius(temp) : 0.0f) << "\n";
        
        // Flush periodically to ensure data is written
        if (elapsed.count() % 1000 == 0) {
            log_file.flush();
        }
        
        // Sleep until next scheduled time
        std::this_thread::sleep_until(next_time);
        next_time += interval;
    }
}
#endif

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

#ifdef ENABLE_SENSOR_TEST
        run_sensor_test();
#else
        run_sensor_polling();
#endif

    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}