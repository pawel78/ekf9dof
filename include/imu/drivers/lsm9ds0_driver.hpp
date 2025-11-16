#pragma once

#include <thread>
#include <atomic>
#include <memory>
#include "common/channel_types.hpp"

namespace lsm9ds0 {

/**
 * @brief LSM9DS0 IMU Driver Class
 * 
 * Encapsulates hardware initialization, driver thread management, and output channels.
 * The driver reads sensors at 200 Hz, applies datasheet conversions,
 * timestamps samples, and publishes to separate channels per sensor type.
 */
class LSM9DS0Driver {
public:
    /**
     * @brief Construct and initialize the LSM9DS0 driver
     * 
     * Creates output channels, verifies hardware, and configures registers.
     * 
     * @throws std::runtime_error if device verification or configuration fails
     */
    LSM9DS0Driver();

    /**
     * @brief Destructor - stops the driver thread gracefully
     */
    ~LSM9DS0Driver();

    // Prevent copying
    LSM9DS0Driver(const LSM9DS0Driver&) = delete;
    LSM9DS0Driver& operator=(const LSM9DS0Driver&) = delete;

    /**
     * @brief Start the driver thread
     * 
     * Begins reading sensors at 200 Hz and publishing to channels
     */
    void start();

    /**
     * @brief Stop the driver thread gracefully
     * 
     * Signals the thread to stop and waits for it to finish
     */
    void stop();

    /**
     * @brief Check if the driver thread is running
     * 
     * @return true if running, false otherwise
     */
    bool is_running() const { return running_.load(); }

    /**
     * @brief Get reference to accelerometer channel
     */
    channels::RawAccelChannel& get_accel_channel() { return raw_accel_chan_; }

    /**
     * @brief Get reference to gyroscope channel
     */
    channels::RawGyroChannel& get_gyro_channel() { return raw_gyro_chan_; }

    /**
     * @brief Get reference to magnetometer channel
     */
    channels::RawMagChannel& get_mag_channel() { return raw_mag_chan_; }

    /**
     * @brief Get reference to temperature channel
     */
    channels::RawTempChannel& get_temp_channel() { return raw_temp_chan_; }

private:
    std::atomic<bool> running_;
    std::thread driver_thread_;

    // Channels owned by driver
    channels::RawAccelChannel raw_accel_chan_;
    channels::RawGyroChannel raw_gyro_chan_;
    channels::RawMagChannel raw_mag_chan_;
    channels::RawTempChannel raw_temp_chan_;
};

} // namespace lsm9ds0
