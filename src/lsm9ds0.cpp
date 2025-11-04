#include <cstdint>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <stdexcept>
#include <system_error>
#include <cstring>
#include "lsm9ds0_config.hpp"
#include "lsm9ds0.hpp"
#include "lsm9ds0_device.hpp"

class I2CDevice {
private:
    int fd; // File descriptor for I2C device

public:
    explicit I2CDevice(const char* device = "/dev/i2c-7") {
        // Open I2C device
        fd = open(device, O_RDWR);
        if (fd < 0) {
            throw std::system_error(errno, std::system_category(), "Failed to open I2C device");
        }
    }

    ~I2CDevice() {
        if (fd >= 0) {
            close(fd);
        }
    }

    // Implementation of i2c_write function
    void write(uint8_t dev_addr, uint8_t reg_addr, uint8_t value) {
        // Set I2C device address
        if (ioctl(fd, I2C_SLAVE, dev_addr) < 0) {
            throw std::system_error(errno, std::system_category(), "Failed to set I2C slave address");
        }

        // Write register address and value
        uint8_t buffer[2] = {reg_addr, value};
        if (::write(fd, buffer, 2) != 2) {
            throw std::system_error(errno, std::system_category(), "Failed to write to I2C device");
        }
    }

    // Read a single register (8-bit)
    uint8_t read_reg(uint8_t dev_addr, uint8_t reg_addr) {
        if (ioctl(fd, I2C_SLAVE, dev_addr) < 0) {
            throw std::system_error(errno, std::system_category(), "Failed to set I2C slave address");
        }
        // Write register address
        uint8_t r = reg_addr;
        if (::write(fd, &r, 1) != 1) {
            throw std::system_error(errno, std::system_category(), "Failed to write register address");
        }
        uint8_t val = 0;
        if (::read(fd, &val, 1) != 1) {
            throw std::system_error(errno, std::system_category(), "Failed to read register");
        }
        return val;
    }

    // Static wrapper that matches the function pointer signature
    static void i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t value) {
        // Note: This assumes a global/singleton instance. You might want to handle this differently.
        static I2CDevice device;
        device.write(dev_addr, reg_addr, value);
    }

    // Static singleton used by the exported functions below
    static I2CDevice &singleton() {
        static I2CDevice dev("/dev/i2c-7");
        return dev;
    }
};

// Example usage:
namespace lsm9ds0_device {

void i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t value) {
    I2CDevice::singleton().write(dev_addr, reg_addr, value);
}

void configure_imu() {
    // call the example config functions using the exported i2c_write
    lsm9ds0_config::configure_gyroscope(i2c_write);
    lsm9ds0_config::configure_accelerometer(i2c_write);
    lsm9ds0_config::configure_magnetometer(i2c_write);
}

void configure_temperature_sensor() {
    lsm9ds0_config::configure_temperature_sensor(i2c_write);
}   

bool read16_le(uint8_t dev_addr, uint8_t reg_low, int16_t &out) {
    try {
        uint8_t lo = I2CDevice::singleton().read_reg(dev_addr, reg_low);
        uint8_t hi = I2CDevice::singleton().read_reg(dev_addr, reg_low + 1);
        out = static_cast<int16_t>(static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8));
        return true;
    } catch (...) {
        return false;
    }
}

bool read_accel(int16_t &x, int16_t &y, int16_t &z) {
    return read16_le(lsm9ds0::XM_ADDR, lsm9ds0::OUT_X_L_A, x) &&
           read16_le(lsm9ds0::XM_ADDR, lsm9ds0::OUT_Y_L_A, y) &&
           read16_le(lsm9ds0::XM_ADDR, lsm9ds0::OUT_Z_L_A, z);
}

bool read_gyro(int16_t &x, int16_t &y, int16_t &z) {
    return read16_le(lsm9ds0::G_ADDR, lsm9ds0::OUT_X_L_G, x) &&
           read16_le(lsm9ds0::G_ADDR, lsm9ds0::OUT_Y_L_G, y) &&
           read16_le(lsm9ds0::G_ADDR, lsm9ds0::OUT_Z_L_G, z);
}

bool read_mag(int16_t &x, int16_t &y, int16_t &z) {
    return read16_le(lsm9ds0::XM_ADDR, lsm9ds0::OUT_X_L_M, x) &&
           read16_le(lsm9ds0::XM_ADDR, lsm9ds0::OUT_Y_L_M, y) &&
           read16_le(lsm9ds0::XM_ADDR, lsm9ds0::OUT_Z_L_M, z);
}

bool read_temperature(int16_t &temp) {
    return read16_le(lsm9ds0::XM_ADDR, lsm9ds0::OUT_TEMP_L_XM, temp);   
}

} // namespace lsm9ds0_device

