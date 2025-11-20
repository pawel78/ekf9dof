#include "imu/drivers/lsm9ds0_driver.hpp"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <stdexcept>

LSM9DS0Driver::LSM9DS0Driver(const char* i2c_device_path)
    : running_(false)
    , debug_output_enabled_(false)
    , i2c_device_(std::make_unique<I2CDevice>(i2c_device_path))
{
    std::cout << "Initializing LSM9DS0 IMU...\n";
    
    // Verify device IDs
    if (!verify_device_ids()) {
        throw std::runtime_error("Failed to verify LSM9DS0 device IDs");
    }
    std::cout << "✓ Device IDs verified\n";
    
    // Configure IMU
    configure_imu();
    std::cout << "✓ IMU configured\n";
}

LSM9DS0Driver::~LSM9DS0Driver() {
    stop();
}

void LSM9DS0Driver::start() {
    if (running_.load()) {
        std::cerr << "WARNING: Driver already running\n";
        return;
    }

    std::cout << "Starting LSM9DS0 driver thread (200 Hz)...\n";
    running_.store(true);

    // Spawn driver thread using static member function
    driver_thread_ = std::thread(driver_thread_func, this);

    std::cout << "✓ Driver thread started\n";
}

void LSM9DS0Driver::stop() {
    if (!running_.load()) {
        return; // Already stopped
    }

    std::cout << "Stopping LSM9DS0 driver...\n";
    
    // Signal thread to stop
    running_.store(false);

    // Wait for thread to finish
    if (driver_thread_.joinable()) {
        driver_thread_.join();
    }

    std::cout << "✓ Driver stopped\n";
}

void LSM9DS0Driver::print_debug_data(uint64_t timestamp_ns,
                                      float ax, float ay, float az,
                                      float gx, float gy, float gz,
                                      float mx, float my, float mz,
                                      float temp) {
    // Convert nanoseconds to seconds with fractional part
    double timestamp_sec = static_cast<double>(timestamp_ns) / 1e9;
    
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "[" << timestamp_sec << "s] ";
    std::cout << "A(" << std::setw(6) << std::setprecision(3) << ax << "," 
              << std::setw(6) << std::setprecision(3) << ay << "," 
              << std::setw(6) << std::setprecision(3) << az << ") ";
    std::cout << "G(" << std::setw(7) << std::setprecision(2) << gx << "," 
              << std::setw(7) << std::setprecision(2) << gy << "," 
              << std::setw(7) << std::setprecision(2) << gz << ") ";
    std::cout << "M(" << std::setw(6) << std::setprecision(3) << mx << "," 
              << std::setw(6) << std::setprecision(3) << my << "," 
              << std::setw(6) << std::setprecision(3) << mz << ") ";
    std::cout << "T(" << std::setw(5) << std::setprecision(1) << temp << "°C)";
    std::cout << std::endl;
}
