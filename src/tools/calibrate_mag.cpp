#include <iostream>
#include <string>
#include "imu/drivers/lsm9ds0_device.hpp"
#include "imu/processing/mag_calibration.hpp"

int main(int argc, char* argv[]) {
    // Default config path
    std::string config_path = "configs/config.yaml";
    
    // Allow user to specify config path
    if (argc > 1) {
        config_path = argv[1];
    }
    
    try {
        // Verify device by reading WHO_AM_I registers
        if (!lsm9ds0_device::verify_device_ids()) {
            std::cerr << "Error: LSM9DS0 device not found or incorrect WHO_AM_I response.\n";
            std::cerr << "Please ensure the sensor is connected and accessible.\n";
            return 1;
        }
        
        // Configure IMU
        lsm9ds0_device::configure_imu();
        
        std::cout << "LSM9DS0 sensor initialized successfully.\n";
        
        // Run calibration
        if (!mag_calibration::run_calibration(config_path)) {
            std::cerr << "Calibration failed.\n";
            return 1;
        }
        
        std::cout << "Calibration completed successfully!\n";
        std::cout << "You can now use the calibrated values in your application.\n";
        
        return 0;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}
