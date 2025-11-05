/**
 * @file lsm9ds0_driver.hpp
 * @brief LSM9DS0 I2C driver for ROS 2
 */

#pragma once

#include <cstdint>
#include <string>
#include <Eigen/Dense>

namespace lsm9ds0_imu {

/**
 * @brief Driver for LSM9DS0 9-DOF IMU on I2C
 */
class LSM9DS0Driver {
public:
    struct Config {
        int i2c_bus = 7;
        uint8_t gyro_accel_addr = 0x6B;
        uint8_t mag_addr = 0x1D;
        int sample_rate_hz = 200;
        
        // Full-scale ranges
        int gyro_scale_dps = 245;     // 245, 500, or 2000
        int accel_scale_g = 2;         // 2, 4, 6, 8, or 16
        int mag_scale_gauss = 2;       // 2, 4, 8, or 12
        
        // Mounting orientation (RPY in degrees)
        Eigen::Vector3d orientation_rpy = Eigen::Vector3d::Zero();
        
        // Calibration
        Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
        Eigen::Vector3d accel_bias = Eigen::Vector3d::Zero();
        Eigen::Vector3d mag_bias = Eigen::Vector3d::Zero();
        Eigen::Vector3d accel_scale = Eigen::Vector3d::Ones();
    };
    
    explicit LSM9DS0Driver(const Config& config);
    ~LSM9DS0Driver();
    
    /**
     * @brief Initialize the IMU
     * @return true if successful
     */
    bool initialize();
    
    /**
     * @brief Verify WHO_AM_I registers
     * @return true if both devices respond correctly
     */
    bool verifyDeviceIds();
    
    /**
     * @brief Read gyroscope data (rad/s)
     */
    bool readGyro(Eigen::Vector3d& gyro);
    
    /**
     * @brief Read accelerometer data (m/s^2)
     */
    bool readAccel(Eigen::Vector3d& accel);
    
    /**
     * @brief Read magnetometer data (Tesla)
     */
    bool readMag(Eigen::Vector3d& mag);
    
    /**
     * @brief Read temperature (Celsius)
     */
    bool readTemperature(float& temp);
    
    /**
     * @brief Read all sensors at once
     */
    bool readAll(Eigen::Vector3d& gyro, Eigen::Vector3d& accel,
                Eigen::Vector3d& mag, float& temp);

private:
    Config config_;
    int i2c_fd_;
    bool initialized_;
    
    // Helper functions
    bool openI2C();
    void closeI2C();
    bool writeRegister(uint8_t dev_addr, uint8_t reg, uint8_t value);
    bool readRegister(uint8_t dev_addr, uint8_t reg, uint8_t& value);
    bool readRegisters(uint8_t dev_addr, uint8_t reg, uint8_t* buffer, size_t len);
    int16_t read16LE(uint8_t dev_addr, uint8_t reg_low);
    
    void applyCalibration(Eigen::Vector3d& data, const Eigen::Vector3d& bias,
                         const Eigen::Vector3d& scale);
    void applyOrientation(Eigen::Vector3d& data);
    
    // Scale conversion factors
    double gyro_scale_factor_;
    double accel_scale_factor_;
    double mag_scale_factor_;
};

} // namespace lsm9ds0_imu
