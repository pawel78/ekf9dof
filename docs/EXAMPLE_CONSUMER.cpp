/**
 * @file EXAMPLE_CONSUMER.cpp
 * @brief Example demonstrating complete consumer-driver isolation
 * 
 * This file shows how a consumer can be written in complete isolation from
 * the driver. The consumer:
 * - Does NOT include any driver headers
 * - Does NOT know about I2C, hardware, or LSM9DS0
 * - Does NOT receive channel references as parameters
 * - ONLY includes imu/messages/imu_data.hpp for data types and global channels
 * 
 * The consumer can be compiled, tested, and developed independently.
 */

#include "imu/messages/imu_data.hpp"
#include <iostream>

/**
 * @brief Standalone gyro consumer - zero driver knowledge
 * 
 * This function could exist in a completely separate library or module.
 * It subscribes to the global gyro channel without any awareness of:
 * - Which driver is publishing (LSM9DS0, MPU6050, etc.)
 * - How data is acquired (I2C, SPI, simulated, etc.)
 * - Driver lifecycle or configuration
 */
void my_gyro_consumer() {
    // Access the global gyro channel directly
    imu::messages::raw_gyro_msg_t msg;
    
    while (imu::channels::raw_gyro.try_receive(msg)) {
        // Process gyro data
        std::cout << "Gyro: [" << msg.x << ", " << msg.y << ", " << msg.z << "] rad/s\n";
        
        // Do whatever processing you need...
        // - Feed into EKF
        // - Log to file
        // - Transmit over network
        // - Display on GUI
    }
}

/**
 * @brief Multi-sensor consumer - still zero driver knowledge
 * 
 * Even when consuming multiple sensor types, no driver awareness needed.
 */
void my_fusion_algorithm() {
    imu::messages::raw_accel_msg_t accel;
    imu::messages::raw_gyro_msg_t gyro;
    imu::messages::raw_mag_msg_t mag;
    
    // Read latest from all sensors
    bool have_accel = imu::channels::raw_accel.try_receive(accel);
    bool have_gyro = imu::channels::raw_gyro.try_receive(gyro);
    bool have_mag = imu::channels::raw_mag.try_receive(mag);
    
    if (have_accel && have_gyro && have_mag) {
        // Run sensor fusion algorithm
        // No idea where this data came from!
    }
}

/**
 * @brief Temperature monitor - demonstrates single-include simplicity
 */
void temperature_monitor() {
    imu::messages::raw_temp_msg_t temp;
    
    if (imu::channels::raw_temp.try_receive(temp)) {
        if (temp.temperature_c > 85.0f) {
            std::cerr << "WARNING: High temperature: " << temp.temperature_c << "°C\n";
        }
    }
}

// To use these functions, main.cpp only needs to:
// 1. Create and start the driver (driver publishes to globals)
// 2. Call the consumer functions (they subscribe to globals)
// 
// No channel passing, no driver references, complete isolation!
