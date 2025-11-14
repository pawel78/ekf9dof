#pragma once

#include <vector>
#include <array>
#include <string>

/**
 * @brief Magnetometer calibration utilities for soft and hard iron compensation
 * 
 * This module provides functions to:
 * - Collect magnetometer samples across various orientations
 * - Calculate hard iron offset (constant bias from nearby ferromagnetic materials)
 * - Calculate soft iron correction matrix (distortion from nearby materials)
 * - Apply calibration to raw magnetometer readings
 * - Run interactive calibration process with user guidance
 */
namespace mag_calibration {

/**
 * @brief Collect magnetometer samples at a specified rate
 * 
 * @param num_samples Number of samples to collect
 * @param sample_rate_ms Time between samples in milliseconds
 * @return Vector of 3D magnetometer readings in gauss
 */
std::vector<std::array<float, 3>> collect_samples(int num_samples, int sample_rate_ms);

/**
 * @brief Calculate hard iron offset from collected samples
 * 
 * Hard iron effects are caused by permanent magnets or ferromagnetic materials
 * near the sensor, creating a constant offset in the measurements.
 * 
 * @param samples Vector of magnetometer samples
 * @param offset Output: calculated hard iron offset for [x, y, z] in gauss
 */
void calculate_hard_iron_offset(const std::vector<std::array<float, 3>>& samples,
                                 std::array<float, 3>& offset);

/**
 * @brief Calculate soft iron correction matrix from collected samples
 * 
 * Soft iron effects are caused by materials that distort the magnetic field,
 * creating scaling and cross-axis coupling. This is represented as a 3x3 matrix.
 * 
 * @param samples Vector of magnetometer samples
 * @param hard_iron_offset Previously calculated hard iron offset
 * @param soft_iron_matrix Output: 3x3 correction matrix stored as 9-element array (row-major)
 */
void calculate_soft_iron_matrix(const std::vector<std::array<float, 3>>& samples,
                                 const std::array<float, 3>& hard_iron_offset,
                                 std::array<float, 9>& soft_iron_matrix);

/**
 * @brief Apply calibration to raw magnetometer reading
 * 
 * Applies both hard iron (offset) and soft iron (matrix) corrections:
 * calibrated = soft_iron_matrix * (raw - hard_iron_offset)
 * 
 * @param raw Raw magnetometer reading [x, y, z] in gauss
 * @param hard_iron_offset Hard iron offset [x, y, z] in gauss
 * @param soft_iron_matrix 3x3 soft iron correction matrix (row-major, 9 elements)
 * @param calibrated Output: calibrated magnetometer reading [x, y, z] in gauss
 */
void apply_calibration(const std::array<float, 3>& raw,
                       const std::array<float, 3>& hard_iron_offset,
                       const std::array<float, 9>& soft_iron_matrix,
                       std::array<float, 3>& calibrated);

/**
 * @brief Run interactive magnetometer calibration process
 * 
 * Guides the user through collecting samples at various orientations to
 * calculate calibration parameters. Updates the config file with results.
 * 
 * The calibration process involves:
 * 1. Static position for baseline
 * 2. Rotation around X-axis (roll)
 * 3. Rotation around Y-axis (pitch)
 * 4. Rotation around Z-axis (yaw)
 * 5. Random tumbling
 * 6. Figure-8 motion
 * 
 * @param config_path Path to config.yaml file to update
 * @return true if calibration succeeded and config was updated, false otherwise
 */
bool run_calibration(const std::string& config_path);

} // namespace mag_calibration
