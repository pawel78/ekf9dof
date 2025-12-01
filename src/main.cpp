#include <iostream>
#include <thread>
#include <chrono>
#include <iomanip>
#include <atomic>
#include <csignal>
#include "imu/messages/imu_data.hpp" // Messages + channel types in imu namespace
#include "imu/processing/imu_preprocessor.hpp"
#include "common/custom_time_methods.hpp"

// Note: Driver include only in main() for instantiation
#include "imu/drivers/lsm9ds0_driver.hpp"

// Global flag for signal handling
std::atomic<bool> g_running{true};

void signal_handler(int signal)
{
    if (signal == SIGINT || signal == SIGTERM)
    {
        std::cout << "\nShutdown signal received. Stopping...\n";
        g_running.store(false);
    }
}

int main(int argc, char* argv[])
{
    // Set up signal handlers for graceful shutdown
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    // Check for debug flag
    bool enable_debug = false;
    bool enable_logging = false;
    std::string log_file_timestamp = get_timestamp_filename();
    std::string log_filename = "imu_driver_data_" + log_file_timestamp + ".bin";
    
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--debug" || arg == "-d") {
            enable_debug = true;
        } else if (arg == "--log" || arg == "-l") {
            enable_logging = true;
            // Check if next argument is a filename
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                log_filename = argv[++i];
            }
        }
    }

    try
    {
        // Create driver FIRST (binds NNG publisher sockets)
        LSM9DS0Driver imu_driver("/dev/i2c-7");
        
        // Then create preprocessor (connects to NNG publisher sockets)
        IMUPreprocessor imu_preprocessor;

        // Start the IMU driver (spawns internal thread)
        imu_driver.start();
        
        // Enable debug output if requested
        if (enable_debug) {
            std::cout << "Debug mode enabled - sensor data will be displayed every 500ms\n";
            imu_driver.set_debug_output(true);
        }
        
        // Enable binary logging if requested
        if (enable_logging) {
            if (imu_driver.set_data_logging(true, log_filename)) {
                std::cout << "Binary logging enabled at 200 Hz to: " << log_filename << "\n";
            } else {
                std::cerr << "WARNING: Failed to enable binary logging\n";
            }
        }
       
        // Start the IMU preprocessor (spawns internal thread)
        imu_preprocessor.start();

        // Wait for shutdown signal
        while (g_running.load())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // Cleanup
        std::cout << "\nShutting down...\n";
        imu_preprocessor.stop();
        if (enable_logging) {
            imu_driver.set_data_logging(false);
        }
        imu_driver.stop();
        std::cout << "Shutdown complete\n";
    }
    catch (const std::exception &e)
    {
        std::cerr << "ERROR: " << e.what() << "\n";
        return 1;
    }

    return 0;
}