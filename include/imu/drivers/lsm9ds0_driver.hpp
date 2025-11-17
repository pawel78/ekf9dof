#pragma once

#include <thread>
#include <atomic>
#include <memory>
#include "common/channel_types.hpp"

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
     * 
     * Note: Must be explicitly declared here because of unique_ptr with forward-declared type
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
    // Forward declaration for nested I2C device class
    class I2CDevice;
    
    // Driver thread function (static to work with std::thread)
    static void driver_thread_func(LSM9DS0Driver* driver);
    
    // I2C operations
    void i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t value);
    uint8_t i2c_read_reg(uint8_t dev_addr, uint8_t reg_addr);
    bool i2c_read16_le(uint8_t dev_addr, uint8_t reg_low, int16_t& out);
    
    // Hardware verification and configuration
    bool verify_device_ids();
    void configure_imu();
    
    // Sensor read operations
    bool read_accel(int16_t& x, int16_t& y, int16_t& z);
    bool read_gyro(int16_t& x, int16_t& y, int16_t& z);
    bool read_mag(int16_t& x, int16_t& y, int16_t& z);
    bool read_temperature(int16_t& temp);
    
    // Datasheet conversion formulas
    static float raw_to_g(int16_t raw_accel);
    static float raw_to_dps(int16_t raw_gyro);
    static float raw_to_gauss(int16_t raw_mag);
    static float raw_to_celsius(int16_t raw_temp);
    
    // Timestamp helper
    static uint64_t get_timestamp_ns();
    
    // Member variables
    std::atomic<bool> running_;
    std::thread driver_thread_;
    std::unique_ptr<I2CDevice> i2c_device_;

    // Channels owned by driver
    channels::RawAccelChannel raw_accel_chan_;
    channels::RawGyroChannel raw_gyro_chan_;
    channels::RawMagChannel raw_mag_chan_;
    channels::RawTempChannel raw_temp_chan_;
};
