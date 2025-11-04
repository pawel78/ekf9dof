#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace ekf9dof {

/**
 * @brief Generic sensor class for handling multi-dimensional sensor measurements
 * 
 * @tparam N Number of measurements (e.g., 3 for 3D sensors like gyro/accel/mag, 1 for temperature)
 * 
 * This class provides a flexible way to store sensor measurements with:
 * - Configurable measurement dimensions
 * - Sensor name identification
 * - New data flag that increments with each measurement (handles overflow)
 * - Timestamp support
 */
template<size_t N>
class Sensor {
public:
    /**
     * @brief Construct a new Sensor object
     * 
     * @param name Name of the sensor (e.g., "gyro", "accel", "mag", "temp")
     */
    explicit Sensor(const std::string& name) 
        : name_(name), new_data_flag_(0), timestamp_(0) {
        measurements_.fill(0.0f);
    }

    /**
     * @brief Store new measurement data
     * 
     * @param data Array of measurements
     * @param timestamp Optional timestamp for the measurement
     * 
     * Increments the new_data_flag_ to signal consumers that new data is available.
     * The flag is uint8_t and will naturally overflow after 255, which is acceptable
     * as consumers will check for changes.
     */
    void store(const std::array<float, N>& data, uint64_t timestamp = 0) {
        measurements_ = data;
        timestamp_ = timestamp;
        new_data_flag_++;  // Intentionally allows overflow
    }

    /**
     * @brief Store new measurement data from raw pointer
     * 
     * @param data Pointer to array of measurements
     * @param timestamp Optional timestamp for the measurement
     */
    void store(const float* data, uint64_t timestamp = 0) {
        for (size_t i = 0; i < N; ++i) {
            measurements_[i] = data[i];
        }
        timestamp_ = timestamp;
        new_data_flag_++;  // Intentionally allows overflow
    }

    /**
     * @brief Get the current measurements
     * 
     * @return const std::array<float, N>& Reference to measurements array
     */
    const std::array<float, N>& get_measurements() const {
        return measurements_;
    }

    /**
     * @brief Get a specific measurement by index
     * 
     * @param index Index of the measurement to retrieve
     * @return float The measurement value
     */
    float get_measurement(size_t index) const {
        return measurements_[index];
    }

    /**
     * @brief Get the timestamp of the last measurement
     * 
     * @return uint64_t Timestamp value
     */
    uint64_t get_timestamp() const {
        return timestamp_;
    }

    /**
     * @brief Get the new data flag
     * 
     * @return uint8_t Current value of the new data flag
     * 
     * Consumers should save the previous value and compare with the current value
     * to detect new measurements. The flag will overflow after 255, which is expected.
     */
    uint8_t get_new_data_flag() const {
        return new_data_flag_;
    }

    /**
     * @brief Get the sensor name
     * 
     * @return const std::string& Reference to the sensor name
     */
    const std::string& get_name() const {
        return name_;
    }

    /**
     * @brief Get the number of measurements this sensor stores
     * 
     * @return size_t Number of measurements
     */
    static constexpr size_t get_measurement_count() {
        return N;
    }

private:
    std::string name_;                  ///< Name of the sensor
    std::array<float, N> measurements_; ///< Array of sensor measurements
    uint8_t new_data_flag_;             ///< Flag that increments with each new measurement
    uint64_t timestamp_;                ///< Timestamp of the last measurement
};

// Convenience type aliases for common sensor types
using Sensor3D = Sensor<3>;    ///< 3D sensor (gyro, accel, mag)
using Sensor1D = Sensor<1>;    ///< 1D sensor (temperature)
using SensorGPS = Sensor<6>;   ///< GPS sensor (latitude, longitude, altitude, velocity_x, velocity_y, velocity_z)

} // namespace ekf9dof
