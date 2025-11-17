#include <cstdint>
#include <stdexcept>
#include <system_error>
#include <cstring>
#include <array>
#include <chrono>
#include <thread>
#include <atomic>
#include <cmath>
#include <iostream>
#include "imu/drivers/lsm9ds0_config.hpp"
#include "imu/drivers/lsm9ds0.hpp"
#include "imu/drivers/lsm9ds0_device.hpp"
#include "common/channel_types.hpp"
#include "imu/messages/imu_data.hpp"

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

namespace lsm9ds0_device {

void i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t value) {
    I2CDevice::singleton().write(dev_addr, reg_addr, value);
}

void configure_imu() {
    // call the example config functions using the exported i2c_write
    lsm9ds0_config::configure_gyroscope(i2c_write);
    lsm9ds0_config::configure_accelerometer(i2c_write);
    lsm9ds0_config::configure_magnetometer(i2c_write);
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

// Helper function to get high-resolution timestamp in nanoseconds
uint64_t get_timestamp_ns() {
    auto now = std::chrono::steady_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
}

// LSM9DS0 driver thread using channel architecture
// Reads sensors at 200 Hz, converts to SI units, and publishes to separate channels
void lsm9ds0_driver_thread(
    std::atomic<bool>& running,
    channels::RawAccelChannel& raw_accel_chan,
    channels::RawGyroChannel& raw_gyro_chan,
    channels::RawMagChannel& raw_mag_chan,
    channels::RawTempChannel& raw_temp_chan)
{
    constexpr auto sample_period = std::chrono::milliseconds(5); // 200 Hz
    
    int error_count = 0;
    int sample_count = 0;
    int accel_sent = 0, gyro_sent = 0, mag_sent = 0, temp_sent = 0;
    bool first_sample = true;
    
    std::cout << "Driver thread: Starting sensor reads...\n" << std::flush;
    
    while (running.load()) {
        // Capture timestamp as close to hardware read as possible
        uint64_t timestamp = get_timestamp_ns();
        
        bool all_ok = true;
        
        // Read accelerometer
        int16_t ax_raw, ay_raw, az_raw;
        if (read_accel(ax_raw, ay_raw, az_raw)) {
            // Convert to g using datasheet formula
            float ax = raw_to_g(ax_raw);
            float ay = raw_to_g(ay_raw);
            float az = raw_to_g(az_raw);
            
            imu::messages::raw_accel_msg_t accel_msg{timestamp, ax, ay, az};
            if (!raw_accel_chan.send(accel_msg)) {
                std::cerr << "ERROR: Accel channel closed\n" << std::flush;
                break; // Channel closed
            }
            accel_sent++;
            if (first_sample) {
                std::cout << "Driver: First accel sent: " << ax << ", " << ay << ", " << az << "\n" << std::flush;
            }
        } else {
            all_ok = false;
            if (first_sample || error_count < 5) {
                std::cerr << "ERROR: Failed to read accelerometer\n" << std::flush;
            }
        }
        
        // Read gyroscope
        int16_t gx_raw, gy_raw, gz_raw;
        if (read_gyro(gx_raw, gy_raw, gz_raw)) {
            // Convert to rad/s (dps → rad/s)
            float gx_dps = raw_to_dps(gx_raw);
            float gy_dps = raw_to_dps(gy_raw);
            float gz_dps = raw_to_dps(gz_raw);
            
            constexpr float deg_to_rad = M_PI / 180.0f;
            float gx = gx_dps * deg_to_rad;
            float gy = gy_dps * deg_to_rad;
            float gz = gz_dps * deg_to_rad;
            
            imu::messages::raw_gyro_msg_t gyro_msg{timestamp, gx, gy, gz};
            if (!raw_gyro_chan.send(gyro_msg)) {
                std::cerr << "ERROR: Gyro channel closed\n" << std::flush;
                break; // Channel closed
            }
            gyro_sent++;
            if (first_sample) {
                std::cout << "Driver: First gyro sent: " << gx << ", " << gy << ", " << gz << "\n" << std::flush;
            }
        } else {
            all_ok = false;
            if (first_sample || error_count < 5) {
                std::cerr << "ERROR: Failed to read gyroscope\n" << std::flush;
            }
        }
        
        // Read magnetometer
        int16_t mx_raw, my_raw, mz_raw;
        if (read_mag(mx_raw, my_raw, mz_raw)) {
            // Convert to gauss using datasheet formula
            float mx = raw_to_gauss(mx_raw);
            float my = raw_to_gauss(my_raw);
            float mz = raw_to_gauss(mz_raw);
            
            imu::messages::raw_mag_msg_t mag_msg{timestamp, mx, my, mz};
            if (!raw_mag_chan.send(mag_msg)) {
                std::cerr << "ERROR: Mag channel closed\n" << std::flush;
                break; // Channel closed
            }
            mag_sent++;
            if (first_sample) {
                std::cout << "Driver: First mag sent: " << mx << ", " << my << ", " << mz << "\n" << std::flush;
            }
        } else {
            all_ok = false;
            if (first_sample || error_count < 5) {
                std::cerr << "ERROR: Failed to read magnetometer\n" << std::flush;
            }
        }
        
        // Read temperature
        int16_t temp_raw;
        if (read_temperature(temp_raw)) {
            // Convert to Celsius using datasheet formula
            float temp_c = raw_to_celsius(temp_raw);
            
            imu::messages::raw_temp_msg_t temp_msg{timestamp, temp_c};
            if (!raw_temp_chan.send(temp_msg)) {
                std::cerr << "ERROR: Temp channel closed\n" << std::flush;
                break; // Channel closed
            }
            temp_sent++;
            if (first_sample) {
                std::cout << "Driver: First temp sent: " << temp_c << "°C\n" << std::flush;
            }
        } else {
            all_ok = false;
            if (first_sample || error_count < 5) {
                std::cerr << "ERROR: Failed to read temperature\n" << std::flush;
            }
        }
        
        if (!all_ok) {
            error_count++;
        }
        
        sample_count++;
        first_sample = false;
        
        // Print status every 1000 samples (~5 seconds at 200 Hz)
        if (sample_count % 1000 == 0) {
            std::cout << "Driver stats: samples=" << sample_count 
                      << " sent(A:" << accel_sent << " G:" << gyro_sent 
                      << " M:" << mag_sent << " T:" << temp_sent << ")"
                      << " errors=" << error_count << "\n" << std::flush;
        }
        
        // Sleep to maintain 200 Hz rate
        std::this_thread::sleep_for(sample_period);
    }
}

/*
// Old thread function (deprecated - kept for reference)
void lsm9ds0_thread_func(channels::unbounded_buffer<std::array<int16_t, 10>>& data_buffer) {
    while (true) {
        std::array<int16_t, 10> sample;
        bool aok = read_accel(sample[0], sample[1], sample[2]);
        bool gok = read_gyro(sample[3], sample[4], sample[5]);
        bool mok = read_mag(sample[6], sample[7], sample[8]);
        bool tok = read_temperature(sample[9]);

        if (aok && gok && mok && tok) {
            data_buffer.send(sample);
        } else {
            // Optionally log read error
        }

        // Sleep for 5 ms to achieve ~200 Hz sampling rate
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}
*/

} // namespace lsm9ds0_device

