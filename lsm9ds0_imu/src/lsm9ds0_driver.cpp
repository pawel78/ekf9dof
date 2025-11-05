/**
 * @file lsm9ds0_driver.cpp
 * @brief Implementation of LSM9DS0 I2C driver
 */

#include "lsm9ds0_imu/lsm9ds0_driver.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstring>
#include <cmath>

namespace lsm9ds0_imu {

// LSM9DS0 Register Map
namespace regs {
    // WHO_AM_I values
    constexpr uint8_t WHO_AM_I_G = 0x0F;
    constexpr uint8_t WHO_AM_I_XM = 0x0F;
    constexpr uint8_t WHO_AM_I_G_VALUE = 0xD4;
    constexpr uint8_t WHO_AM_I_XM_VALUE = 0x49;
    
    // Gyro registers
    constexpr uint8_t CTRL_REG1_G = 0x20;
    constexpr uint8_t CTRL_REG4_G = 0x23;
    constexpr uint8_t OUT_X_L_G = 0x28;
    
    // Accel/Mag registers
    constexpr uint8_t CTRL_REG1_XM = 0x20;
    constexpr uint8_t CTRL_REG2_XM = 0x21;
    constexpr uint8_t CTRL_REG5_XM = 0x24;
    constexpr uint8_t CTRL_REG6_XM = 0x25;
    constexpr uint8_t CTRL_REG7_XM = 0x26;
    constexpr uint8_t OUT_X_L_A = 0x28;
    constexpr uint8_t OUT_X_L_M = 0x08;
    constexpr uint8_t OUT_TEMP_L_XM = 0x05;
}

LSM9DS0Driver::LSM9DS0Driver(const Config& config)
    : config_(config), i2c_fd_(-1), initialized_(false) {
    // Calculate scale factors
    switch (config_.gyro_scale_dps) {
        case 245: gyro_scale_factor_ = 8.75e-3 * M_PI / 180.0; break;
        case 500: gyro_scale_factor_ = 17.50e-3 * M_PI / 180.0; break;
        case 2000: gyro_scale_factor_ = 70.0e-3 * M_PI / 180.0; break;
        default: gyro_scale_factor_ = 8.75e-3 * M_PI / 180.0;
    }
    
    switch (config_.accel_scale_g) {
        case 2: accel_scale_factor_ = 0.061e-3 * 9.81; break;
        case 4: accel_scale_factor_ = 0.122e-3 * 9.81; break;
        case 6: accel_scale_factor_ = 0.183e-3 * 9.81; break;
        case 8: accel_scale_factor_ = 0.244e-3 * 9.81; break;
        case 16: accel_scale_factor_ = 0.732e-3 * 9.81; break;
        default: accel_scale_factor_ = 0.061e-3 * 9.81;
    }
    
    switch (config_.mag_scale_gauss) {
        case 2: mag_scale_factor_ = 0.08e-3 * 1e-4; break;  // Convert to Tesla
        case 4: mag_scale_factor_ = 0.16e-3 * 1e-4; break;
        case 8: mag_scale_factor_ = 0.32e-3 * 1e-4; break;
        case 12: mag_scale_factor_ = 0.48e-3 * 1e-4; break;
        default: mag_scale_factor_ = 0.08e-3 * 1e-4;
    }
}

LSM9DS0Driver::~LSM9DS0Driver() {
    closeI2C();
}

bool LSM9DS0Driver::openI2C() {
    std::string i2c_device = "/dev/i2c-" + std::to_string(config_.i2c_bus);
    i2c_fd_ = open(i2c_device.c_str(), O_RDWR);
    return i2c_fd_ >= 0;
}

void LSM9DS0Driver::closeI2C() {
    if (i2c_fd_ >= 0) {
        close(i2c_fd_);
        i2c_fd_ = -1;
    }
}

bool LSM9DS0Driver::writeRegister(uint8_t dev_addr, uint8_t reg, uint8_t value) {
    if (ioctl(i2c_fd_, I2C_SLAVE, dev_addr) < 0) return false;
    uint8_t buffer[2] = {reg, value};
    return write(i2c_fd_, buffer, 2) == 2;
}

bool LSM9DS0Driver::readRegister(uint8_t dev_addr, uint8_t reg, uint8_t& value) {
    if (ioctl(i2c_fd_, I2C_SLAVE, dev_addr) < 0) return false;
    if (write(i2c_fd_, &reg, 1) != 1) return false;
    return read(i2c_fd_, &value, 1) == 1;
}

bool LSM9DS0Driver::readRegisters(uint8_t dev_addr, uint8_t reg, uint8_t* buffer, size_t len) {
    if (ioctl(i2c_fd_, I2C_SLAVE, dev_addr) < 0) return false;
    // Set MSB to enable auto-increment
    uint8_t reg_auto = reg | 0x80;
    if (write(i2c_fd_, &reg_auto, 1) != 1) return false;
    return read(i2c_fd_, buffer, len) == static_cast<ssize_t>(len);
}

int16_t LSM9DS0Driver::read16LE(uint8_t dev_addr, uint8_t reg_low) {
    uint8_t buffer[2];
    if (readRegisters(dev_addr, reg_low, buffer, 2)) {
        return static_cast<int16_t>((buffer[1] << 8) | buffer[0]);
    }
    return 0;
}

bool LSM9DS0Driver::initialize() {
    if (!openI2C()) {
        return false;
    }
    
    if (!verifyDeviceIds()) {
        closeI2C();
        return false;
    }
    
    // Configure gyroscope: ODR=190Hz, enable all axes
    if (!writeRegister(config_.gyro_accel_addr, regs::CTRL_REG1_G, 0x6F)) return false;
    // Set gyro scale
    uint8_t gyro_scale_bits = 0x00;  // 245 dps
    if (config_.gyro_scale_dps == 500) gyro_scale_bits = 0x10;
    else if (config_.gyro_scale_dps == 2000) gyro_scale_bits = 0x20;
    if (!writeRegister(config_.gyro_accel_addr, regs::CTRL_REG4_G, gyro_scale_bits)) return false;
    
    // Configure accelerometer: ODR=200Hz, enable all axes
    if (!writeRegister(config_.mag_addr, regs::CTRL_REG1_XM, 0x67)) return false;
    // Set accel scale
    uint8_t accel_scale_bits = 0x00;  // ±2g
    if (config_.accel_scale_g == 4) accel_scale_bits = 0x08;
    else if (config_.accel_scale_g == 6) accel_scale_bits = 0x10;
    else if (config_.accel_scale_g == 8) accel_scale_bits = 0x18;
    else if (config_.accel_scale_g == 16) accel_scale_bits = 0x20;
    if (!writeRegister(config_.mag_addr, regs::CTRL_REG2_XM, accel_scale_bits)) return false;
    
    // Configure magnetometer: continuous mode, ODR=100Hz
    if (!writeRegister(config_.mag_addr, regs::CTRL_REG5_XM, 0xF4)) return false;
    // Set mag scale
    uint8_t mag_scale_bits = 0x00;  // ±2 gauss
    if (config_.mag_scale_gauss == 4) mag_scale_bits = 0x20;
    else if (config_.mag_scale_gauss == 8) mag_scale_bits = 0x40;
    else if (config_.mag_scale_gauss == 12) mag_scale_bits = 0x60;
    if (!writeRegister(config_.mag_addr, regs::CTRL_REG6_XM, mag_scale_bits)) return false;
    if (!writeRegister(config_.mag_addr, regs::CTRL_REG7_XM, 0x00)) return false;  // Continuous mode
    
    initialized_ = true;
    return true;
}

bool LSM9DS0Driver::verifyDeviceIds() {
    uint8_t who_am_i_g, who_am_i_xm;
    if (!readRegister(config_.gyro_accel_addr, regs::WHO_AM_I_G, who_am_i_g)) return false;
    if (!readRegister(config_.mag_addr, regs::WHO_AM_I_XM, who_am_i_xm)) return false;
    return (who_am_i_g == regs::WHO_AM_I_G_VALUE) && (who_am_i_xm == regs::WHO_AM_I_XM_VALUE);
}

bool LSM9DS0Driver::readGyro(Eigen::Vector3d& gyro) {
    if (!initialized_) return false;
    
    int16_t raw_x = read16LE(config_.gyro_accel_addr, regs::OUT_X_L_G);
    int16_t raw_y = read16LE(config_.gyro_accel_addr, regs::OUT_X_L_G + 2);
    int16_t raw_z = read16LE(config_.gyro_accel_addr, regs::OUT_X_L_G + 4);
    
    gyro.x() = raw_x * gyro_scale_factor_;
    gyro.y() = raw_y * gyro_scale_factor_;
    gyro.z() = raw_z * gyro_scale_factor_;
    
    applyCalibration(gyro, config_.gyro_bias, Eigen::Vector3d::Ones());
    applyOrientation(gyro);
    
    return true;
}

bool LSM9DS0Driver::readAccel(Eigen::Vector3d& accel) {
    if (!initialized_) return false;
    
    int16_t raw_x = read16LE(config_.mag_addr, regs::OUT_X_L_A);
    int16_t raw_y = read16LE(config_.mag_addr, regs::OUT_X_L_A + 2);
    int16_t raw_z = read16LE(config_.mag_addr, regs::OUT_X_L_A + 4);
    
    accel.x() = raw_x * accel_scale_factor_;
    accel.y() = raw_y * accel_scale_factor_;
    accel.z() = raw_z * accel_scale_factor_;
    
    applyCalibration(accel, config_.accel_bias, config_.accel_scale);
    applyOrientation(accel);
    
    return true;
}

bool LSM9DS0Driver::readMag(Eigen::Vector3d& mag) {
    if (!initialized_) return false;
    
    int16_t raw_x = read16LE(config_.mag_addr, regs::OUT_X_L_M);
    int16_t raw_y = read16LE(config_.mag_addr, regs::OUT_X_L_M + 2);
    int16_t raw_z = read16LE(config_.mag_addr, regs::OUT_X_L_M + 4);
    
    mag.x() = raw_x * mag_scale_factor_;
    mag.y() = raw_y * mag_scale_factor_;
    mag.z() = raw_z * mag_scale_factor_;
    
    applyCalibration(mag, config_.mag_bias, Eigen::Vector3d::Ones());
    applyOrientation(mag);
    
    return true;
}

bool LSM9DS0Driver::readTemperature(float& temp) {
    if (!initialized_) return false;
    
    int16_t raw_temp = read16LE(config_.mag_addr, regs::OUT_TEMP_L_XM);
    temp = raw_temp / 8.0f + 25.0f;
    
    return true;
}

bool LSM9DS0Driver::readAll(Eigen::Vector3d& gyro, Eigen::Vector3d& accel,
                           Eigen::Vector3d& mag, float& temp) {
    return readGyro(gyro) && readAccel(accel) && readMag(mag) && readTemperature(temp);
}

void LSM9DS0Driver::applyCalibration(Eigen::Vector3d& data, const Eigen::Vector3d& bias,
                                    const Eigen::Vector3d& scale) {
    data = (data - bias).cwiseProduct(scale);
}

void LSM9DS0Driver::applyOrientation(Eigen::Vector3d& data) {
    if (config_.orientation_rpy.norm() < 1e-6) return;
    
    // Convert RPY to rotation matrix
    double roll = config_.orientation_rpy.x() * M_PI / 180.0;
    double pitch = config_.orientation_rpy.y() * M_PI / 180.0;
    double yaw = config_.orientation_rpy.z() * M_PI / 180.0;
    
    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    
    data = R * data;
}

} // namespace lsm9ds0_imu
