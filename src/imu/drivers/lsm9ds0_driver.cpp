#include "imu/drivers/lsm9ds0_driver.hpp"
#include "imu/drivers/lsm9ds0_device.hpp"
#include <iostream>
#include <stdexcept>

namespace lsm9ds0 {

LSM9DS0Driver::LSM9DS0Driver()
    : running_(false)
{
    std::cout << "Initializing LSM9DS0 IMU...\n";
    
    // Verify device IDs
    if (!lsm9ds0_device::verify_device_ids()) {
        throw std::runtime_error("Failed to verify LSM9DS0 device IDs");
    }
    std::cout << "✓ Device IDs verified\n";
    
    // Configure IMU
    lsm9ds0_device::configure_imu();
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

    // Spawn driver thread
    driver_thread_ = std::thread(
        lsm9ds0_device::lsm9ds0_driver_thread,
        std::ref(running_),
        std::ref(raw_accel_chan_),
        std::ref(raw_gyro_chan_),
        std::ref(raw_mag_chan_),
        std::ref(raw_temp_chan_)
    );

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

} // namespace lsm9ds0
