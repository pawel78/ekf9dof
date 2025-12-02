#include <thread>
#include <atomic>
#include <memory>
#include "common/quat.hpp"    
#include "common/nng_socket.hpp"
#include "imu/messages/nng_imu_sockets.hpp" 
#include "ahrs/ahrs.hpp"
#include <iostream>

Ahrs::Ahrs()
    : running_(false),
      nng_gyro_sub_(std::make_unique<NngRawGyroSocket>(nng_urls::RAW_GYRO, false)),
      nng_accel_sub_(std::make_unique<NngRawAccelSocket>(nng_urls::RAW_ACCEL, false)),
      nng_mag_sub_(std::make_unique<NngRawMagSocket>(nng_urls::RAW_MAG, false)),
      nng_temp_sub_(std::make_unique<NngRawTempSocket>(nng_urls::RAW_TEMP, false))
{
    // Initialization code if needed
}   
Ahrs::~Ahrs()
{
    stop();
    // Close NNG channels
    if (nng_gyro_sub_) nng_gyro_sub_->close();
    if (nng_accel_sub_) nng_accel_sub_->close();
    if (nng_mag_sub_) nng_mag_sub_->close();
    if (nng_temp_sub_) nng_temp_sub_->close();
}
void Ahrs::start()
{
    if (running_.load())
    {
        std::cerr << "WARNING: AHRS already running\n";
        return;
    }

    std::cout << "Starting AHRS processing thread...\n";
    running_.store(true);

    // Spawn AHRS thread using static member function
    ahrs_thread_ = std::thread(ahrs_thread_func, this);

    std::cout << "✓ AHRS thread started\n";
}
void Ahrs::stop()
{
    if (!running_.load())
    {
        return; // Already stopped
    }

    std::cout << "Stopping AHRS...\n";

    // Signal thread to stop
    running_.store(false);

    // Wait for thread to finish
    if (ahrs_thread_.joinable())
    {
        ahrs_thread_.join();
    }

    std::cout << "✓ AHRS stopped\n";
}
bool Ahrs::is_running() const
{
    return running_.load();
}   
void Ahrs::ahrs_thread_func(Ahrs* ahrs)
{
    std::cout << "AHRS thread running.\n";

    while (ahrs->running_.load())
    {
        // AHRS processing logic goes here
        // For example, read from NNG sockets, compute orientation, etc.

        // Simulate some processing delay
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "AHRS thread exiting.\n";
}   
