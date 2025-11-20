#include <iostream>
#include <thread>
#include <chrono>
#include <iomanip>
#include <atomic>
#include <csignal>
#include "imu/messages/imu_data.hpp" // Messages + channel types in imu namespace
#include "imu/processing/imu_preprocessor.hpp"

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

/**
 * @brief Example consumer - uses global channel directly, no driver knowledge
 *
 * This function reads from the global gyro channel.
 * It has ZERO knowledge of drivers, hardware, or I2C.
 */
void consume_gyro_data()
{
    std::cout << "Gyro consumer started (reading from global channel)...\n";

    int count = 0;
    while (g_running.load() && count < 1000)
    {
        imu::messages::raw_gyro_msg_t gyro_msg;

        if (imu::channels::raw_gyro.try_receive(gyro_msg))
        {
            if (count % 200 == 0)
            { // Print every 200 samples
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
 * @brief Another consumer - uses global channels directly
 */
void consume_all_sensors()
{
    std::cout << "Multi-sensor consumer started...\n";

    int sample_count = 0;
    imu::messages::raw_accel_msg_t accel{0, 0, 0, 0};
    imu::messages::raw_gyro_msg_t gyro{0, 0, 0, 0};
    imu::messages::raw_mag_msg_t mag{0, 0, 0, 0};

    while (g_running.load())
    {
        bool have_data = false;

        if (imu::channels::raw_accel.try_receive(accel))
        {
            have_data = true;
            sample_count++;
        }
        if (imu::channels::raw_gyro.try_receive(gyro))
            have_data = true;
        if (imu::channels::raw_mag.try_receive(mag))
            have_data = true;

        if (have_data && sample_count % 200 == 0)
        {
            std::cout << std::fixed << std::setprecision(2)
                      << "A(" << accel.x << "," << accel.y << "," << accel.z << ") "
                      << "G(" << gyro.x << "," << gyro.y << "," << gyro.z << ") "
                      << "M(" << mag.x << "," << mag.y << "," << mag.z << ")\n";
        }

        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
}

int main(int argc, char* argv[])
{
    // Set up signal handlers for graceful shutdown
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    // Check for debug flag
    bool enable_debug = false;
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--debug" || arg == "-d") {
            enable_debug = true;
        }
    }

    try
    {
        IMUPreprocessor imu_preprocessor;
        LSM9DS0Driver imu_driver("/dev/i2c-7");

        // Start the IMU driver (spawns internal thread)
        imu_driver.start();
        
        // Enable debug output if requested
        if (enable_debug) {
            std::cout << "Debug mode enabled - sensor data will be displayed every 500ms\n";
            imu_driver.set_debug_output(true);
        }
       
        // Start the IMU preprocessor (spawns internal thread)
        // imu_preprocessor.start();

        // Wait for shutdown signal
        while (g_running.load())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // Cleanup
        std::cout << "\nShutting down...\n";
        // imu_preprocessor.stop();
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