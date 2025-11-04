/**
 * @file sensor_usage_example.cpp
 * @brief Example demonstrating usage of the Sensor class
 * 
 * This example shows how to use the Sensor class to handle measurements
 * from various sensors like gyroscope, accelerometer, magnetometer, and temperature.
 */

#include <iostream>
#include <thread>
#include <chrono>
#include "ekf9dof/sensor.hpp"

using namespace ekf9dof;

int main() {
    std::cout << "Sensor Class Usage Example\n";
    std::cout << "===========================\n\n";
    
    // Create sensors for different measurements
    Sensor3D gyro("gyroscope");
    Sensor3D accel("accelerometer");
    Sensor3D mag("magnetometer");
    Sensor1D temp("temperature");
    SensorGPS gps("gps");
    
    std::cout << "Created sensors:\n";
    std::cout << "  - " << gyro.get_name() << " (3D)\n";
    std::cout << "  - " << accel.get_name() << " (3D)\n";
    std::cout << "  - " << mag.get_name() << " (3D)\n";
    std::cout << "  - " << temp.get_name() << " (1D)\n";
    std::cout << "  - " << gps.get_name() << " (6D)\n\n";
    
    // Simulate sensor data acquisition
    std::cout << "Simulating sensor readings...\n\n";
    
    // Consumer keeps track of last seen flag to detect new data
    uint8_t last_gyro_flag = gyro.get_new_data_flag();
    uint8_t last_accel_flag = accel.get_new_data_flag();
    
    // Simulate 5 sensor readings
    for (int i = 0; i < 5; ++i) {
        auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()
        ).count();
        
        // Store gyroscope data (simulated)
        std::array<float, 3> gyro_data = {
            static_cast<float>(i) * 10.0f,
            static_cast<float>(i) * 20.0f,
            static_cast<float>(i) * 30.0f
        };
        gyro.store(gyro_data, timestamp);
        
        // Store accelerometer data (simulated)
        float accel_raw[3] = {
            1.0f + static_cast<float>(i) * 0.1f,
            0.5f + static_cast<float>(i) * 0.05f,
            9.8f + static_cast<float>(i) * 0.2f
        };
        accel.store(accel_raw, timestamp);
        
        // Store magnetometer data (simulated)
        std::array<float, 3> mag_data = {0.3f, -0.1f, 0.5f};
        mag.store(mag_data, timestamp);
        
        // Store temperature data (simulated)
        std::array<float, 1> temp_data = {25.0f + static_cast<float>(i) * 0.5f};
        temp.store(temp_data, timestamp);
        
        // Check if new gyro data is available
        if (gyro.get_new_data_flag() != last_gyro_flag) {
            const auto& gyro_measurements = gyro.get_measurements();
            std::cout << "Reading #" << i + 1 << " - Gyro (new data): "
                      << "x=" << gyro_measurements[0] 
                      << ", y=" << gyro_measurements[1] 
                      << ", z=" << gyro_measurements[2] << "\n";
            last_gyro_flag = gyro.get_new_data_flag();
        }
        
        // Check if new accel data is available
        if (accel.get_new_data_flag() != last_accel_flag) {
            std::cout << "Reading #" << i + 1 << " - Accel (new data): "
                      << "x=" << accel.get_measurement(0) 
                      << ", y=" << accel.get_measurement(1) 
                      << ", z=" << accel.get_measurement(2) << "\n";
            last_accel_flag = accel.get_new_data_flag();
        }
        
        // Display temperature
        std::cout << "Reading #" << i + 1 << " - Temperature: " 
                  << temp.get_measurement(0) << "°C\n\n";
        
        // Small delay between readings
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // Demonstrate GPS sensor usage
    std::cout << "\nGPS sensor example:\n";
    std::array<float, 6> gps_data = {
        37.7749f,   // latitude
        -122.4194f, // longitude
        50.0f,      // altitude (m)
        1.5f,       // velocity_x (m/s)
        0.5f,       // velocity_y (m/s)
        0.0f        // velocity_z (m/s)
    };
    
    auto gps_timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()
    ).count();
    
    gps.store(gps_data, gps_timestamp);
    
    const auto& gps_measurements = gps.get_measurements();
    std::cout << "GPS Position: lat=" << gps_measurements[0] 
              << ", lon=" << gps_measurements[1] 
              << ", alt=" << gps_measurements[2] << "m\n";
    std::cout << "GPS Velocity: vx=" << gps_measurements[3] 
              << ", vy=" << gps_measurements[4] 
              << ", vz=" << gps_measurements[5] << " m/s\n";
    
    std::cout << "\nExample completed successfully!\n";
    
    return 0;
}
