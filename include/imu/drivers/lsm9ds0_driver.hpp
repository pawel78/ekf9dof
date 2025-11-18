#pragma once

#include <thread>
#include <atomic>
#include <memory>
#include "common/channel_types.hpp"
#include "common/i2c_device.hpp"

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
     * @param i2c_device_path Path to I2C device (e.g., "/dev/i2c-7")
     * @throws std::runtime_error if device verification or configuration fails
     */
    explicit LSM9DS0Driver(const char* i2c_device_path);

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

private:
    // ========================================================================
    // LSM9DS0 Register Map - Hardware constants
    // ========================================================================
    
    // I2C addresses
    static constexpr uint8_t XM_ADDR = 0x1D; // Accelerometer/Magnetometer
    static constexpr uint8_t G_ADDR  = 0x6B; // Gyroscope

    // Gyroscope Registers
    static constexpr uint8_t WHO_AM_I_G        = 0x0F;
    static constexpr uint8_t CTRL_REG1_G       = 0x20;
    static constexpr uint8_t CTRL_REG2_G       = 0x21;
    static constexpr uint8_t CTRL_REG3_G       = 0x22;
    static constexpr uint8_t CTRL_REG4_G       = 0x23;
    static constexpr uint8_t CTRL_REG5_G       = 0x24;
    static constexpr uint8_t REFERENCE_G       = 0x25;
    static constexpr uint8_t OUT_TEMP_G        = 0x26;
    static constexpr uint8_t STATUS_REG_G      = 0x27;
    static constexpr uint8_t OUT_X_L_G         = 0x28;
    static constexpr uint8_t OUT_X_H_G         = 0x29;
    static constexpr uint8_t OUT_Y_L_G         = 0x2A;
    static constexpr uint8_t OUT_Y_H_G         = 0x2B;
    static constexpr uint8_t OUT_Z_L_G         = 0x2C;
    static constexpr uint8_t OUT_Z_H_G         = 0x2D;
    static constexpr uint8_t FIFO_CTRL_REG_G   = 0x2E;
    static constexpr uint8_t FIFO_SRC_REG_G    = 0x2F;
    static constexpr uint8_t INT1_CFG_G        = 0x30;
    static constexpr uint8_t INT1_SRC_G        = 0x31;
    static constexpr uint8_t INT1_THS_XH_G     = 0x32;
    static constexpr uint8_t INT1_THS_XL_G     = 0x33;
    static constexpr uint8_t INT1_THS_YH_G     = 0x34;
    static constexpr uint8_t INT1_THS_YL_G     = 0x35;
    static constexpr uint8_t INT1_THS_ZH_G     = 0x36;
    static constexpr uint8_t INT1_THS_ZL_G     = 0x37;
    static constexpr uint8_t INT1_DURATION_G   = 0x38;

    // Accelerometer/Magnetometer Registers
    static constexpr uint8_t WHO_AM_I_XM       = 0x0F;
    static constexpr uint8_t INT_CTRL_REG_M    = 0x12;
    static constexpr uint8_t OUT_TEMP_L_XM     = 0x05;
    static constexpr uint8_t OUT_TEMP_H_XM     = 0x06;
    static constexpr uint8_t STATUS_REG_M      = 0x07;
    static constexpr uint8_t OUT_X_L_M         = 0x08;
    static constexpr uint8_t OUT_X_H_M         = 0x09;
    static constexpr uint8_t OUT_Y_L_M         = 0x0A;
    static constexpr uint8_t OUT_Y_H_M         = 0x0B;
    static constexpr uint8_t OUT_Z_L_M         = 0x0C;
    static constexpr uint8_t OUT_Z_H_M         = 0x0D;
    static constexpr uint8_t CTRL_REG0_XM      = 0x1F;
    static constexpr uint8_t CTRL_REG1_XM      = 0x20;
    static constexpr uint8_t CTRL_REG2_XM      = 0x21;
    static constexpr uint8_t CTRL_REG3_XM      = 0x22;
    static constexpr uint8_t CTRL_REG4_XM      = 0x23;
    static constexpr uint8_t CTRL_REG5_XM      = 0x24;
    static constexpr uint8_t CTRL_REG6_XM      = 0x25;
    static constexpr uint8_t CTRL_REG7_XM      = 0x26;
    static constexpr uint8_t STATUS_REG_A      = 0x27;
    static constexpr uint8_t OUT_X_L_A         = 0x28;
    static constexpr uint8_t OUT_X_H_A         = 0x29;
    static constexpr uint8_t OUT_Y_L_A         = 0x2A;
    static constexpr uint8_t OUT_Y_H_A         = 0x2B;
    static constexpr uint8_t OUT_Z_L_A         = 0x2C;
    static constexpr uint8_t OUT_Z_H_A         = 0x2D;
    static constexpr uint8_t FIFO_CTRL_REG     = 0x2E;
    static constexpr uint8_t FIFO_SRC_REG      = 0x2F;
    static constexpr uint8_t INT_GEN_1_REG     = 0x30;
    static constexpr uint8_t INT_GEN_1_SRC     = 0x31;
    static constexpr uint8_t INT_GEN_1_THS     = 0x32;
    static constexpr uint8_t INT_GEN_1_DURATION= 0x33;
    static constexpr uint8_t INT_GEN_2_REG     = 0x34;
    static constexpr uint8_t INT_GEN_2_SRC     = 0x35;
    static constexpr uint8_t INT_GEN_2_THS     = 0x36;
    static constexpr uint8_t INT_GEN_2_DURATION= 0x37;
    static constexpr uint8_t CLICK_CFG         = 0x38;
    static constexpr uint8_t CLICK_SRC         = 0x39;
    static constexpr uint8_t CLICK_THS         = 0x3A;
    static constexpr uint8_t TIME_LIMIT        = 0x3B;
    static constexpr uint8_t TIME_LATENCY      = 0x3C;
    static constexpr uint8_t TIME_WINDOW       = 0x3D;
    static constexpr uint8_t ACT_THS           = 0x3E;
    static constexpr uint8_t ACT_DUR           = 0x3F;
    
    // ========================================================================
    // Private implementation
    // ========================================================================
    
    // Driver thread function (static to work with std::thread)
    static void driver_thread_func(LSM9DS0Driver* driver);
    
    // I2C operations
    void i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t value);
    uint8_t i2c_read_reg(uint8_t dev_addr, uint8_t reg_addr);
    bool i2c_read16_le(uint8_t dev_addr, uint8_t reg_low, int16_t& out);
    
    // Hardware verification and configuration
    bool verify_device_ids();
    void configure_imu();
    void configure_gyroscope();
    void configure_accelerometer();
    void configure_magnetometer();
    void configure_temperature_sensor();
    
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
};
