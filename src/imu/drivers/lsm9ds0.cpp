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
#include "imu/drivers/lsm9ds0_driver.hpp"
#include "common/channel_types.hpp"
#include "imu/messages/imu_data.hpp"

// ============================================================================
// LSM9DS0Driver I2C operations (private methods)
// ============================================================================

void LSM9DS0Driver::i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t value) {
    i2c_device_->write(dev_addr, reg_addr, value);
}

uint8_t LSM9DS0Driver::i2c_read_reg(uint8_t dev_addr, uint8_t reg_addr) {
    return i2c_device_->read_reg(dev_addr, reg_addr);
}

bool LSM9DS0Driver::i2c_read16_le(uint8_t dev_addr, uint8_t reg_low, int16_t& out) {
    try {
        uint8_t lo = i2c_read_reg(dev_addr, reg_low);
        uint8_t hi = i2c_read_reg(dev_addr, reg_low + 1);
        out = static_cast<int16_t>(static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8));
        return true;
    } catch (const std::system_error& e) {
        return false;
    }
}

// ============================================================================
// Hardware verification and configuration (private methods)
// ============================================================================

bool LSM9DS0Driver::verify_device_ids() {
    try {
        uint8_t whoami_g = i2c_read_reg(G_ADDR, WHO_AM_I_G);
        uint8_t whoami_xm = i2c_read_reg(XM_ADDR, WHO_AM_I_XM);
        return (whoami_g == 0b11010100) && (whoami_xm == 0b01001001);
    } catch (...) {
        return false;
    }
}

void LSM9DS0Driver::configure_imu() {
    configure_gyroscope();
    configure_accelerometer();
    configure_magnetometer();
    configure_temperature_sensor();
}

void LSM9DS0Driver::configure_gyroscope() {
    // CTRL_REG1_G: ODR=95Hz, Cutoff=12.5Hz, all axes enabled, normal mode
    i2c_write(G_ADDR, CTRL_REG1_G, 0b00001111);
    // CTRL_REG4_G: 245 dps full scale
    i2c_write(G_ADDR, CTRL_REG4_G, 0b00000000);
}

void LSM9DS0Driver::configure_accelerometer() {
    // CTRL_REG1_XM: ODR=100Hz, all axes enabled
    i2c_write(XM_ADDR, CTRL_REG1_XM, 0b01100111);
    // CTRL_REG2_XM: ±2g full scale
    i2c_write(XM_ADDR, CTRL_REG2_XM, 0b00000000);
}

void LSM9DS0Driver::configure_magnetometer() {
    // CTRL_REG5_XM: mag resolution=high, ODR=50Hz
    i2c_write(XM_ADDR, CTRL_REG5_XM, 0b01110000);
    // CTRL_REG6_XM: ±2 gauss full scale
    i2c_write(XM_ADDR, CTRL_REG6_XM, 0b00000000);
    // CTRL_REG7_XM: continuous conversion mode
    i2c_write(XM_ADDR, CTRL_REG7_XM, 0b00000000);
}

void LSM9DS0Driver::configure_temperature_sensor() {
    // CTRL_REG5_XM: enable temperature sensor (bit 7) + mag settings
    i2c_write(XM_ADDR, CTRL_REG5_XM, 0b01110000 | 0b10000000);
}

// ============================================================================
// Sensor read operations (private methods)
// ============================================================================

bool LSM9DS0Driver::read_accel(int16_t& x, int16_t& y, int16_t& z) {
    return i2c_read16_le(XM_ADDR, OUT_X_L_A, x) &&
           i2c_read16_le(XM_ADDR, OUT_Y_L_A, y) &&
           i2c_read16_le(XM_ADDR, OUT_Z_L_A, z);
}

bool LSM9DS0Driver::read_gyro(int16_t& x, int16_t& y, int16_t& z) {
    return i2c_read16_le(G_ADDR, OUT_X_L_G, x) &&
           i2c_read16_le(G_ADDR, OUT_Y_L_G, y) &&
           i2c_read16_le(G_ADDR, OUT_Z_L_G, z);
}

bool LSM9DS0Driver::read_mag(int16_t& x, int16_t& y, int16_t& z) {
    return i2c_read16_le(XM_ADDR, OUT_X_L_M, x) &&
           i2c_read16_le(XM_ADDR, OUT_Y_L_M, y) &&
           i2c_read16_le(XM_ADDR, OUT_Z_L_M, z);
}

bool LSM9DS0Driver::read_temperature(int16_t& temp) {
    try {
        uint8_t lo = i2c_read_reg(XM_ADDR, OUT_TEMP_L_XM);
        uint8_t hi = i2c_read_reg(XM_ADDR, OUT_TEMP_H_XM);

        // Temperature is 12-bit, two's-complement, right-justified
        uint16_t raw = static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8);
        raw &= 0x0FFF; // Keep only lower 12 bits

        int16_t signed12;
        if (raw & 0x0800) {
            // Negative: set upper 4 bits to 1
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

// ============================================================================
// Datasheet conversion formulas (private static methods)
// ============================================================================

float LSM9DS0Driver::raw_to_celsius(int16_t raw_temp) {
    return (raw_temp / 8.0f) + 25.0f;
}

float LSM9DS0Driver::raw_to_dps(int16_t raw_gyro) {
    return raw_gyro * 0.00875f;
}

float LSM9DS0Driver::raw_to_g(int16_t raw_accel) {
    return raw_accel * 0.000061f;
}

float LSM9DS0Driver::raw_to_gauss(int16_t raw_mag) {
    return raw_mag * 0.00008f;
}

// ============================================================================
// Timestamp helper (private static method)
// ============================================================================

uint64_t LSM9DS0Driver::get_timestamp_ns() {
    auto now = std::chrono::steady_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
}

// ============================================================================
// Driver thread function (private static method)
// ============================================================================

void LSM9DS0Driver::driver_thread_func(LSM9DS0Driver* driver) {
    constexpr auto sample_period = std::chrono::milliseconds(5); // 200 Hz
    constexpr auto debug_period = std::chrono::milliseconds(500); // 2 Hz for debug output
    
    int error_count = 0;
    int sample_count = 0;
    int accel_sent = 0, gyro_sent = 0, mag_sent = 0, temp_sent = 0;
    bool first_sample = true;
    
    auto last_debug_time = std::chrono::steady_clock::now();
    
    // Variables to store latest sensor readings for debug output
    float latest_ax = 0, latest_ay = 0, latest_az = 0;
    float latest_gx = 0, latest_gy = 0, latest_gz = 0;
    float latest_mx = 0, latest_my = 0, latest_mz = 0;
    float latest_temp = 0;
    uint64_t latest_timestamp = 0;
    
    std::cout << "Driver thread: Starting sensor reads...\n" << std::flush;
    
    while (driver->running_.load()) {
        // Capture timestamp as close to hardware read as possible
        uint64_t timestamp = get_timestamp_ns();
        
        bool all_ok = true;
        
        // Read accelerometer
        int16_t ax_raw, ay_raw, az_raw;
        if (driver->read_accel(ax_raw, ay_raw, az_raw)) {
            float ax = raw_to_g(ax_raw);
            float ay = raw_to_g(ay_raw);
            float az = raw_to_g(az_raw);
            
            // Store for debug output
            latest_ax = ax;
            latest_ay = ay;
            latest_az = az;
            latest_timestamp = timestamp;
            
            imu::messages::raw_accel_msg_t accel_msg{timestamp, ax, ay, az};
            if (!imu::channels::raw_accel.send(accel_msg)) {
                std::cerr << "ERROR: Accel channel closed\n" << std::flush;
                break;
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
        if (driver->read_gyro(gx_raw, gy_raw, gz_raw)) {
            float gx_dps = raw_to_dps(gx_raw);
            float gy_dps = raw_to_dps(gy_raw);
            float gz_dps = raw_to_dps(gz_raw);
            
            constexpr float deg_to_rad = M_PI / 180.0f;
            float gx = gx_dps * deg_to_rad;
            float gy = gy_dps * deg_to_rad;
            float gz = gz_dps * deg_to_rad;
            
            // Store for debug output
            latest_gx = gx_dps;
            latest_gy = gy_dps;
            latest_gz = gz_dps;
            
            imu::messages::raw_gyro_msg_t gyro_msg{timestamp, gx, gy, gz};
            if (!imu::channels::raw_gyro.send(gyro_msg)) {
                std::cerr << "ERROR: Gyro channel closed\n" << std::flush;
                break;
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
        if (driver->read_mag(mx_raw, my_raw, mz_raw)) {
            float mx = raw_to_gauss(mx_raw);
            float my = raw_to_gauss(my_raw);
            float mz = raw_to_gauss(mz_raw);
            
            // Store for debug output
            latest_mx = mx;
            latest_my = my;
            latest_mz = mz;
            
            imu::messages::raw_mag_msg_t mag_msg{timestamp, mx, my, mz};
            if (!imu::channels::raw_mag.send(mag_msg)) {
                std::cerr << "ERROR: Mag channel closed\n" << std::flush;
                break;
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
        if (driver->read_temperature(temp_raw)) {
            float temp_c = raw_to_celsius(temp_raw);
            
            // Store for debug output
            latest_temp = temp_c;
            
            imu::messages::raw_temp_msg_t temp_msg{timestamp, temp_c};
            if (!imu::channels::raw_temp.send(temp_msg)) {
                std::cerr << "ERROR: Temp channel closed\n" << std::flush;
                break;
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
        
        first_sample = false;
        sample_count++;

        // Debug output at slower rate (500ms intervals)
        if (driver->debug_output_enabled_.load()) {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_debug_time);
            if (elapsed >= debug_period) {
                driver->print_debug_data(latest_timestamp, 
                                        latest_ax, latest_ay, latest_az,
                                        latest_gx, latest_gy, latest_gz,
                                        latest_mx, latest_my, latest_mz,
                                        latest_temp);
                last_debug_time = now;
            }
        }
             
        // Sleep to maintain 200 Hz rate
        std::this_thread::sleep_for(sample_period);
    }
    
    std::cout << "Driver thread: Exiting. Total samples: " << sample_count 
              << ", errors: " << error_count << "\n" << std::flush;
}


