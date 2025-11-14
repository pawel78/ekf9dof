#include <cstdint>
#include <stdexcept>
#include <system_error>
#include <cstring>
#include <array>
#include "lsm9ds0_config.hpp"
#include "lsm9ds0.hpp"
#include "lsm9ds0_device.hpp"

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
    // Mock definitions for non-Linux platforms (e.g., macOS)
    // These allow the code to compile for build verification but won't function
    #warning "Building for non-Linux platform - I2C hardware access will not be available"
#endif

class I2CDevice {
private:
#if PLATFORM_LINUX
    int fd; // File descriptor for I2C device
#endif

public:
    explicit I2CDevice(const char* device = "/dev/i2c-7") {
#if PLATFORM_LINUX
        // Open I2C device
        fd = open(device, O_RDWR);
        if (fd < 0) {
            throw std::system_error(errno, std::system_category(), "Failed to open I2C device");
        }
#else
        (void)device; // Suppress unused parameter warning
        throw std::runtime_error("I2C device access not supported on this platform");
#endif
    }

    ~I2CDevice() {
#if PLATFORM_LINUX
        if (fd >= 0) {
            close(fd);
        }
#endif
    }

    // Implementation of i2c_write function
    void write(uint8_t dev_addr, uint8_t reg_addr, uint8_t value) {
#if PLATFORM_LINUX
        // Set I2C device address
        if (ioctl(fd, I2C_SLAVE, dev_addr) < 0) {
            throw std::system_error(errno, std::system_category(), "Failed to set I2C slave address");
        }

        // Write register address and value
        uint8_t buffer[2] = {reg_addr, value};
        if (::write(fd, buffer, 2) != 2) {
            throw std::system_error(errno, std::system_category(), "Failed to write to I2C device");
        }
#else
        (void)dev_addr; (void)reg_addr; (void)value; // Suppress unused parameter warnings
        throw std::runtime_error("I2C write not supported on this platform");
#endif
    }

    // Read a single register (8-bit)
    uint8_t read_reg(uint8_t dev_addr, uint8_t reg_addr) {
#if PLATFORM_LINUX
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
#else
        (void)dev_addr; (void)reg_addr; // Suppress unused parameter warnings
        throw std::runtime_error("I2C read not supported on this platform");
#endif
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
    } catch (const std::system_error& e) {
        // Optionally log: std::cerr << "I2C read error: " << e.what() << std::endl;
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
    try {
        // Read low and high temperature registers (little-endian)
        uint8_t lo = I2CDevice::singleton().read_reg(lsm9ds0::XM_ADDR, lsm9ds0::OUT_TEMP_L_XM);
        uint8_t hi = I2CDevice::singleton().read_reg(lsm9ds0::XM_ADDR, lsm9ds0::OUT_TEMP_H_XM);

        // Temperature is a 12-bit, two's-complement, right-justified value stored in
        // OUT_TEMP_L_XM (low) and OUT_TEMP_H_XM (high). Combine, mask to 12 bits and
        // sign-extend to a 16-bit signed integer.
        uint16_t raw = static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8);
        raw &= 0x0FFF; // keep only lower 12 bits

        int16_t signed12;
        if (raw & 0x0800) {
            // negative: set upper 4 bits to 1
            signed12 = static_cast<int16_t>(raw | 0xF000);
        } else {
            signed12 = static_cast<int16_t>(raw);
        }

        temp = signed12;
        return true;
    } catch (...) {
        return false;
    }
}

// verify who am i registers on the gyro and accel/mag devices
// Returns true if both match expected values, false otherwise
bool verify_device_ids() {
    try {
        uint8_t whoami_g = I2CDevice::singleton().read_reg(lsm9ds0::G_ADDR, lsm9ds0::WHO_AM_I_G);
        uint8_t whoami_xm = I2CDevice::singleton().read_reg(lsm9ds0::XM_ADDR, lsm9ds0::WHO_AM_I_XM);
        return (whoami_g == 0b11010100) && (whoami_xm == 0b01001001);
    } catch (...) {
        return false;
    }
}

// Convert raw 12-bit temperature value to degrees Celsius
// Formula from LSM9DS0 datasheet: T(°C) = raw/8 + 25
float raw_to_celsius(int16_t raw_temp) {
    return (raw_temp / 8.0f) + 25.0f;
}

// Convert raw gyroscope values to degrees per second (dps)
// Assumes ±245 dps full scale (sensitivity: 8.75 mdps/LSB)
// Formula: dps = raw * 0.00875
float raw_to_dps(int16_t raw_gyro) {
    return raw_gyro * 0.00875f;
}

// Convert raw accelerometer values to g (gravitational acceleration)
// Assumes ±2g full scale (sensitivity: 0.061 mg/LSB)
// Formula: g = raw * 0.061 / 1000 = raw * 0.000061
float raw_to_g(int16_t raw_accel) {
    return raw_accel * 0.000061f;
}

// Convert raw magnetometer values to gauss
// Assumes ±2 gauss full scale (sensitivity: 0.08 mgauss/LSB)
// Formula: gauss = raw * 0.08 / 1000 = raw * 0.00008
float raw_to_gauss(int16_t raw_mag) {
    return raw_mag * 0.00008f;
}

// Magnetometer calibration storage
static std::array<float, 3> mag_bias = {0.0f, 0.0f, 0.0f};
static std::array<float, 9> mag_matrix = {1.0f, 0.0f, 0.0f,
                                          0.0f, 1.0f, 0.0f,
                                          0.0f, 0.0f, 1.0f};
static bool mag_calibration_loaded = false;

// Load magnetometer calibration parameters
void load_mag_calibration(const std::array<float, 3>& bias, const std::array<float, 9>& matrix) {
    mag_bias = bias;
    mag_matrix = matrix;
    mag_calibration_loaded = true;
}

// Read magnetometer with calibration applied
bool read_mag_calibrated(float &x, float &y, float &z) {
    int16_t mx, my, mz;
    if (!read_mag(mx, my, mz)) {
        return false;
    }
    
    // Convert to gauss
    float raw_x = raw_to_gauss(mx);
    float raw_y = raw_to_gauss(my);
    float raw_z = raw_to_gauss(mz);
    
    if (!mag_calibration_loaded) {
        // No calibration loaded, return raw values
        x = raw_x;
        y = raw_y;
        z = raw_z;
        return true;
    }
    
    // Apply hard iron offset
    float temp_x = raw_x - mag_bias[0];
    float temp_y = raw_y - mag_bias[1];
    float temp_z = raw_z - mag_bias[2];
    
    // Apply soft iron correction matrix (3x3 matrix multiplication)
    x = mag_matrix[0] * temp_x + mag_matrix[1] * temp_y + mag_matrix[2] * temp_z;
    y = mag_matrix[3] * temp_x + mag_matrix[4] * temp_y + mag_matrix[5] * temp_z;
    z = mag_matrix[6] * temp_x + mag_matrix[7] * temp_y + mag_matrix[8] * temp_z;
    
    return true;
}

// Get current calibration parameters
void get_mag_calibration(std::array<float, 3>& bias, std::array<float, 9>& matrix) {
    bias = mag_bias;
    matrix = mag_matrix;
}

} // namespace lsm9ds0_device

