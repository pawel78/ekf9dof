#include "imu/drivers/lsm9ds0_driver.hpp"
#include "imu/drivers/lsm9ds0_driver_internal.hpp"
#include <iostream>
#include <stdexcept>

LSM9DS0Driver::LSM9DS0Driver()
    : running_(false)
    , i2c_device_(std::make_unique<I2CDevice>("/dev/i2c-7"))
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
