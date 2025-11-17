#pragma once

#include "lsm9ds0_driver.hpp"
#include <cstdint>

// Platform detection
#if defined(__linux__)
    #define PLATFORM_LINUX 1
    #include <fcntl.h>
    #include <unistd.h>
    #include <sys/ioctl.h>
    #include <linux/i2c-dev.h>
    #include <linux/i2c.h>
#else
    #define PLATFORM_LINUX 0
    #warning "Building for non-Linux platform - I2C hardware access will not be available"
#endif

/**
 * @brief Internal I2C Device Handler
 * 
 * Encapsulates low-level I2C communication with the LSM9DS0 sensor.
 * This is an implementation detail and should not be used outside of the driver.
 */
class LSM9DS0Driver::I2CDevice {
private:
#if PLATFORM_LINUX
    int fd_; // File descriptor for I2C device
#endif

public:
    explicit I2CDevice(const char* device = "/dev/i2c-7");
    ~I2CDevice();

    void write(uint8_t dev_addr, uint8_t reg_addr, uint8_t value);
    uint8_t read_reg(uint8_t dev_addr, uint8_t reg_addr);
};
