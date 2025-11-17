#pragma once

#include <cstdint>

/**
 * @brief Generic I2C Device Interface
 * 
 * Provides low-level I2C communication for any sensor or device on the I2C bus.
 * Handles platform-specific I2C operations (Linux ioctl-based access).
 * 
 * This class can be shared across multiple I2C sensors (IMUs, magnetometers, 
 * barometers, etc.) to avoid code duplication.
 */
class I2CDevice {
private:
#if defined(__linux__)
    int fd_; ///< File descriptor for I2C device (Linux only)
#endif

public:
    /**
     * @brief Open an I2C bus device
     * 
     * @param device Path to I2C device (e.g., "/dev/i2c-7")
     * @throws std::system_error if device cannot be opened
     * @throws std::runtime_error if platform doesn't support I2C
     */
    explicit I2CDevice(const char* device);
    
    /**
     * @brief Close the I2C device
     */
    ~I2CDevice();

    // Prevent copying (managing file descriptor)
    I2CDevice(const I2CDevice&) = delete;
    I2CDevice& operator=(const I2CDevice&) = delete;

    /**
     * @brief Write a single byte to a register
     * 
     * @param dev_addr I2C slave address (7-bit)
     * @param reg_addr Register address
     * @param value Byte value to write
     * @throws std::system_error on I2C communication failure
     */
    void write(uint8_t dev_addr, uint8_t reg_addr, uint8_t value);

    /**
     * @brief Read a single byte from a register
     * 
     * @param dev_addr I2C slave address (7-bit)
     * @param reg_addr Register address
     * @return The byte read from the register
     * @throws std::system_error on I2C communication failure
     */
    uint8_t read_reg(uint8_t dev_addr, uint8_t reg_addr);
};
