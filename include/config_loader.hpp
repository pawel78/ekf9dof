#pragma once

#include <array>
#include <string>

/**
 * @brief Simple configuration loader for YAML config files
 * 
 * Provides minimal YAML parsing functionality without external dependencies.
 * Supports reading float arrays for calibration parameters.
 */
namespace config_loader {

/**
 * @brief Load magnetometer calibration from config file
 * 
 * Reads mag_bias and mag_matrix from the calibration section of the config file.
 * 
 * @param config_path Path to config.yaml file
 * @param mag_bias Output: 3-element array for hard iron offset
 * @param mag_matrix Output: 9-element array for soft iron correction matrix
 * @return true if calibration was loaded successfully, false otherwise
 */
bool load_mag_calibration(const std::string& config_path,
                          std::array<float, 3>& mag_bias,
                          std::array<float, 9>& mag_matrix);

} // namespace config_loader
