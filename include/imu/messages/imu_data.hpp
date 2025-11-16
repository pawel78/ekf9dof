#pragma once
#include <cstdint>

namespace imu {
    namespace messages {
        /**
         * @brief Raw accelerometer data from driver (uncalibrated, SI units)
         * 
         * Contains accelerometer readings in g units (uncalibrated).
         * Driver converts from hardware LSB to engineering units using datasheet formulas.
         * Processing thread applies calibration (ellipsoid correction).
         */
        struct raw_accel_msg_t {
            uint64_t timestamp_ns;      ///< Timestamp in nanoseconds
            float x;                    ///< Uncalibrated X-axis (g)
            float y;                    ///< Uncalibrated Y-axis (g)
            float z;                    ///< Uncalibrated Z-axis (g)
        };

        /**
         * @brief Raw gyroscope data from driver (uncalibrated, SI units)
         * 
         * Contains gyroscope readings in rad/s (uncalibrated).
         * Driver converts from hardware LSB to engineering units using datasheet formulas.
         * Processing thread applies calibration (bias correction).
         */
        struct raw_gyro_msg_t {
            uint64_t timestamp_ns;      ///< Timestamp in nanoseconds
            float x;                    ///< Uncalibrated X-axis (rad/s)
            float y;                    ///< Uncalibrated Y-axis (rad/s)
            float z;                    ///< Uncalibrated Z-axis (rad/s)
        };

        /**
         * @brief Raw magnetometer data from driver (uncalibrated, SI units)
         * 
         * Contains magnetometer readings in gauss (uncalibrated).
         * Driver converts from hardware LSB to engineering units using datasheet formulas.
         * Processing thread applies calibration (hard/soft iron correction).
         */
        struct raw_mag_msg_t {
            uint64_t timestamp_ns;      ///< Timestamp in nanoseconds
            float x;                    ///< Uncalibrated X-axis (gauss)
            float y;                    ///< Uncalibrated Y-axis (gauss)
            float z;                    ///< Uncalibrated Z-axis (gauss)
        };

        /**
         * @brief Raw temperature data from driver
         * 
         * Contains temperature in degrees Celsius.
         * Driver converts from hardware LSB using datasheet formula.
         */
        struct raw_temp_msg_t {
            uint64_t timestamp_ns;      ///< Timestamp in nanoseconds
            float temp_c;               ///< Temperature (°C)
        };

        /**
         * @brief Processed accelerometer data (calibrated, SI units)
         * 
         * Contains calibrated accelerometer readings in g.
         * Processing thread applies ellipsoid calibration to raw data.
         */
        struct proc_accel_msg_t {
            uint64_t timestamp_ns;      ///< Timestamp in nanoseconds (from raw)
            float x;                    ///< Calibrated X-axis (g)
            float y;                    ///< Calibrated Y-axis (g)
            float z;                    ///< Calibrated Z-axis (g)
        };

        /**
         * @brief Processed gyroscope data (calibrated, SI units)
         * 
         * Contains calibrated gyroscope readings in rad/s.
         * Processing thread applies bias correction to raw data.
         */
        struct proc_gyro_msg_t {
            uint64_t timestamp_ns;      ///< Timestamp in nanoseconds (from raw)
            float x;                    ///< Calibrated X-axis (rad/s)
            float y;                    ///< Calibrated Y-axis (rad/s)
            float z;                    ///< Calibrated Z-axis (rad/s)
        };

        /**
         * @brief Processed magnetometer data (calibrated, SI units)
         * 
         * Contains calibrated magnetometer readings in gauss.
         * Processing thread applies hard/soft iron correction to raw data.
         */
        struct proc_mag_msg_t {
            uint64_t timestamp_ns;      ///< Timestamp in nanoseconds (from raw)
            float x;                    ///< Calibrated X-axis (gauss)
            float y;                    ///< Calibrated Y-axis (gauss)
            float z;                    ///< Calibrated Z-axis (gauss)
        };

        /**
         * @brief Processed temperature data
         * 
         * Temperature in degrees Celsius (pass-through from driver).
         */
        struct proc_temp_msg_t {
            uint64_t timestamp_ns;      ///< Timestamp in nanoseconds (from raw)
            float temp_c;               ///< Temperature (°C)
        };

    } // namespace messages
} // namespace imu