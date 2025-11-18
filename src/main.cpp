#include <iostream>
#include <thread>
#include <chrono>
#include <iomanip>
#include <atomic>
#include <csignal>
#include "imu/messages/imu_data.hpp"  // Messages + channel types in imu namespace

// Note: Driver include only in main() for instantiation
#include "imu/drivers/lsm9ds0_driver.hpp"

// Global flag for signal handling
std::atomic<bool> g_running{true};

void signal_handler(int signal) {
    if (signal == SIGINT || signal == SIGTERM) {
        std::cout << "\nShutdown signal received. Stopping...\n";
        g_running.store(false);
    }
}

/**
 * @brief Example consumer - only needs channel reference, no driver knowledge
 * 
 * This function receives a channel reference and reads from it.
 * It has ZERO knowledge of drivers, hardware, or I2C.
 * 
 * @param gyro_channel Reference to gyro data channel
 */
void consume_gyro_data(imu::RawGyroChannel& gyro_channel) {
    std::cout << "Gyro consumer started (reading from channel)...\n";
    
    int count = 0;
    while (g_running.load() && count < 1000) {
        imu::messages::raw_gyro_msg_t gyro_msg;
        
        if (gyro_channel.try_receive(gyro_msg)) {
            if (count % 200 == 0) {  // Print every 200 samples
                std::cout << "Gyro: x=" << gyro_msg.x 
                         << " y=" << gyro_msg.y 
                         << " z=" << gyro_msg.z << " rad/s\n";
            }
            count++;
        }
        
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
}

/**
 * @brief Another consumer - only needs channel references
 * 
 * @param accel_channel Reference to accel data channel
 * @param gyro_channel Reference to gyro data channel  
 * @param mag_channel Reference to mag data channel
 */
void consume_all_sensors(imu::RawAccelChannel& accel_channel,
                         imu::RawGyroChannel& gyro_channel,
                         imu::RawMagChannel& mag_channel) {
    std::cout << "Multi-sensor consumer started...\n";
    
    int sample_count = 0;
    imu::messages::raw_accel_msg_t accel{0, 0, 0, 0};
    imu::messages::raw_gyro_msg_t gyro{0, 0, 0, 0};
    imu::messages::raw_mag_msg_t mag{0, 0, 0, 0};
    
    while (g_running.load()) {
        bool have_data = false;
        
        if (accel_channel.try_receive(accel)) { have_data = true; sample_count++; }
        if (gyro_channel.try_receive(gyro)) have_data = true;
        if (mag_channel.try_receive(mag)) have_data = true;
        
        if (have_data && sample_count % 200 == 0) {
            std::cout << std::fixed << std::setprecision(2)
                     << "A(" << accel.x << "," << accel.y << "," << accel.z << ") "
                     << "G(" << gyro.x << "," << gyro.y << "," << gyro.z << ") "
                     << "M(" << mag.x << "," << mag.y << "," << mag.z << ")\n";
        }
        
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
}

int main() {
    // Set up signal handlers for graceful shutdown
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    try {
        // Create driver (ONLY place that knows about LSM9DS0)
        LSM9DS0Driver imu_driver("/dev/i2c-7");
        
        // Start the driver
        imu_driver.start();
        
        // Pass channel references to consumers - they don't know about the driver!
        // Consumer only needs: #include "common/channel_types.hpp"
        
        // Option 1: Single sensor consumer
        // consume_gyro_data(imu_driver.get_gyro_channel());
        
        // Option 2: Multi-sensor consumer  
        consume_all_sensors(
            imu_driver.get_accel_channel(),
            imu_driver.get_gyro_channel(),
            imu_driver.get_mag_channel()
        );
        
        std::cout << "\nShutting down...\n";
        imu_driver.stop();
        std::cout << "Shutdown complete\n";
        
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << "\n";
        return 1;
    }

    return 0;
}