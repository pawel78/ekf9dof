#pragma once
#include <cstdint>
#include "lsm9ds0.hpp"

// Example functions for configuring LSM9DS0 registers
// Assumes you have I2C write function: void i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t value);

namespace lsm9ds0_config_example {

// Example: Configure gyroscope (normal mode, 95 Hz ODR, 12.5 Hz bandwidth, all axes enabled, 245 dps)
inline void configure_gyroscope(void (*i2c_write)(uint8_t, uint8_t, uint8_t)) {
    // CTRL_REG1_G: ODR=95Hz, Cutoff=12.5Hz, all axes enabled, normal mode
    i2c_write(lsm9ds0::G_ADDR, lsm9ds0::CTRL_REG1_G, 0b00001111);
    // CTRL_REG4_G: 245 dps full scale
    i2c_write(lsm9ds0::G_ADDR, lsm9ds0::CTRL_REG4_G, 0b00000000);
}

// Example: Configure accelerometer (normal mode, 100 Hz ODR, all axes enabled, ±2g)
inline void configure_accelerometer(void (*i2c_write)(uint8_t, uint8_t, uint8_t)) {
    // CTRL_REG1_XM: ODR=100Hz, all axes enabled
    i2c_write(lsm9ds0::XM_ADDR, lsm9ds0::CTRL_REG1_XM, 0b01100111);
    // CTRL_REG2_XM: ±2g full scale
    i2c_write(lsm9ds0::XM_ADDR, lsm9ds0::CTRL_REG2_XM, 0b00000000);
}

// Example: Configure magnetometer (continuous conversion mode, 50 Hz ODR, ±2 gauss)
inline void configure_magnetometer(void (*i2c_write)(uint8_t, uint8_t, uint8_t)) {
    // CTRL_REG5_XM: mag resolution=high, ODR=50Hz
    i2c_write(lsm9ds0::XM_ADDR, lsm9ds0::CTRL_REG5_XM, 0b11110000);
    // CTRL_REG6_XM: ±2 gauss full scale
    i2c_write(lsm9ds0::XM_ADDR, lsm9ds0::CTRL_REG6_XM, 0b00000000);
    // CTRL_REG7_XM: continuous conversion mode
    i2c_write(lsm9ds0::XM_ADDR, lsm9ds0::CTRL_REG7_XM, 0b00000000);
}

} // namespace lsm9ds0_config_example
