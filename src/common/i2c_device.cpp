#include "common/i2c_device.hpp"
#include <stdexcept>
#include <system_error>
#include <cstring>

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

I2CDevice::I2CDevice(const char* device) {
#if PLATFORM_LINUX
    fd_ = open(device, O_RDWR);
    if (fd_ < 0) {
        throw std::system_error(errno, std::system_category(), "Failed to open I2C device");
    }
#else
    (void)device;
    throw std::runtime_error("I2C device access not supported on this platform");
#endif
}

I2CDevice::~I2CDevice() {
#if PLATFORM_LINUX
    if (fd_ >= 0) {
        close(fd_);
    }
#endif
}

void I2CDevice::write(uint8_t dev_addr, uint8_t reg_addr, uint8_t value) {
#if PLATFORM_LINUX
    if (ioctl(fd_, I2C_SLAVE, dev_addr) < 0) {
        throw std::system_error(errno, std::system_category(), "Failed to set I2C slave address");
    }

    uint8_t buffer[2] = {reg_addr, value};
    if (::write(fd_, buffer, 2) != 2) {
        throw std::system_error(errno, std::system_category(), "Failed to write to I2C device");
    }
#else
    (void)dev_addr; (void)reg_addr; (void)value;
    throw std::runtime_error("I2C write not supported on this platform");
#endif
}

uint8_t I2CDevice::read_reg(uint8_t dev_addr, uint8_t reg_addr) {
#if PLATFORM_LINUX
    if (ioctl(fd_, I2C_SLAVE, dev_addr) < 0) {
        throw std::system_error(errno, std::system_category(), "Failed to set I2C slave address");
    }
    
    uint8_t r = reg_addr;
    if (::write(fd_, &r, 1) != 1) {
        throw std::system_error(errno, std::system_category(), "Failed to write register address");
    }
    
    uint8_t val = 0;
    if (::read(fd_, &val, 1) != 1) {
        throw std::system_error(errno, std::system_category(), "Failed to read register");
    }
    return val;
#else
    (void)dev_addr; (void)reg_addr;
    throw std::runtime_error("I2C read not supported on this platform");
#endif
}
