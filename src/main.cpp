#include <iostream>
#include <thread>
#include <chrono>
#include <iomanip>
#include <atomic>
#include <csignal>
#include "imu/drivers/lsm9ds0_driver.hpp"
#include "common/channel_types.hpp"

// Global flag for signal handling
std::atomic<bool> g_running{true};

void signal_handler(int signal) {
    if (signal == SIGINT || signal == SIGTERM) {
        std::cout << "\nShutdown signal received. Stopping...\n";
        g_running.store(false);
    }
}

int main() {
    // Set up signal handlers for graceful shutdown
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    try {
        // Create and initialize driver (owns channels, verifies device, configures registers)
        lsm9ds0::LSM9DS0Driver imu_driver;
        
        // Start the driver thread
        imu_driver.start();
        
        // TODO: Create and start processing thread
        // lsm9ds0::ImuProcessingThread processing(
        //     imu_driver.get_accel_channel(),
        //     imu_driver.get_gyro_channel(),
        //     imu_driver.get_mag_channel()
        // );
        // processing.start();
        
        // Simple consumer: read and print data for verification
        std::cout << "Reading sensor data (Ctrl+C to stop)...\n";
        std::cout << "Format: [timestamp_ns] accel(g) gyro(rad/s) mag(gauss) temp(°C)\n";
        std::cout << std::flush;
        
        int sample_count = 0;
        const int PRINT_INTERVAL = 200; // Print every 200 samples (1 second at 200 Hz)
        
        while (g_running.load()) {
            // Try to receive data from channels (non-blocking)
            imu::messages::raw_accel_msg_t accel_msg;
            imu::messages::raw_gyro_msg_t gyro_msg;
            imu::messages::raw_mag_msg_t mag_msg;
            imu::messages::raw_temp_msg_t temp_msg;
            
            bool got_accel = imu_driver.get_accel_channel().try_receive(accel_msg);
            bool got_gyro = imu_driver.get_gyro_channel().try_receive(gyro_msg);
            bool got_mag = imu_driver.get_mag_channel().try_receive(mag_msg);
            bool got_temp = imu_driver.get_temp_channel().try_receive(temp_msg);
            
            // Debug: print when we get any data
            if (got_accel || got_gyro || got_mag || got_temp) {
                sample_count++;
                if (sample_count == 1) {
                    std::cout << "First data received! A=" << got_accel 
                              << " G=" << got_gyro 
                              << " M=" << got_mag 
                              << " T=" << got_temp << "\n" << std::flush;
                }
            }
            
            if (got_accel && got_gyro && got_mag && got_temp) {
                // Print every PRINT_INTERVAL samples
                if (sample_count % PRINT_INTERVAL == 0) {
                    std::cout << std::fixed << std::setprecision(3);
                    std::cout << "[" << accel_msg.timestamp_ns << "] "
                              << "A(" << accel_msg.x << "," << accel_msg.y << "," << accel_msg.z << ") "
                              << "G(" << gyro_msg.x << "," << gyro_msg.y << "," << gyro_msg.z << ") "
                              << "M(" << mag_msg.x << "," << mag_msg.y << "," << mag_msg.z << ") "
                              << "T(" << temp_msg.temp_c << "°C)\n" << std::flush;
                }
            }
            
            // Small sleep to avoid busy-waiting
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        
        std::cout << "\nShutting down...\n";
        
        // Driver will stop automatically in destructor
        
        std::cout << "Total samples processed: " << sample_count << "\n";
        std::cout << "Shutdown complete\n";
        
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << "\n";
        return 1;
    }

    return 0;
}