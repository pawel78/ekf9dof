#pragma once
#include <cstdint>

// High-level IMU device helpers (wrapper around Linux I2C implementation)
namespace lsm9ds0_device {

// Write a single byte to device register
void i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t value);

// Configure the IMU (calls the configuration examples)
void configure_imu();

// Configure the temperature sensor
void configure_temperature_sensor();

// Read a 16-bit little-endian register pair from device (low, high)
// Returns true on success, false on failure
bool read16_le(uint8_t dev_addr, uint8_t reg_low, int16_t &out);

// Read sensor convenience helpers
bool read_accel(int16_t &x, int16_t &y, int16_t &z);
bool read_gyro(int16_t &x, int16_t &y, int16_t &z);
bool read_mag(int16_t &x, int16_t &y, int16_t &z);
bool read_temperature(int16_t &temp);

// Convert raw 12-bit temperature value to degrees Celsius
// Formula: T(°C) = raw/8 + 25
float raw_to_celsius(int16_t raw_temp);

bool verify_device_ids();

} // namespace lsm9ds0_device
