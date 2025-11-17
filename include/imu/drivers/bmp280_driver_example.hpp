#pragma once

#include <memory>
#include "common/i2c_device.hpp"

/**
 * @brief Example: BMP280 Barometric Pressure Sensor Driver
 * 
 * Demonstrates how the standalone I2CDevice can be reused across
 * different I2C sensors without code duplication.
 */
class BMP280Driver {
public:
    /**
     * @brief Initialize BMP280 sensor
     * 
     * @param i2c_device Shared I2C device interface
     */
    explicit BMP280Driver(std::shared_ptr<I2CDevice> i2c_device);

    /**
     * @brief Read temperature and pressure
     * 
     * @param temperature_c Output temperature in Celsius
     * @param pressure_pa Output pressure in Pascals
     * @return true if successful
     */
    bool read(float& temperature_c, float& pressure_pa);

private:
    static constexpr uint8_t BMP280_ADDR = 0x76; // Default I2C address
    static constexpr uint8_t TEMP_XLSB = 0xFC;
    static constexpr uint8_t TEMP_LSB = 0xFB;
    static constexpr uint8_t TEMP_MSB = 0xFA;
    // ... more registers
    
    std::shared_ptr<I2CDevice> i2c_device_;
};

// Usage example:
// auto i2c_bus = std::make_shared<I2CDevice>("/dev/i2c-7");
// LSM9DS0Driver imu(i2c_bus);
// BMP280Driver baro(i2c_bus);  // Reuses same I2C bus!
