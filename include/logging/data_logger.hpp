#pragma once

#include <string>
#include <fstream>
#include <memory>
#include <atomic>
#include <thread>
#include <vector>
#include "imu/messages/nng_imu_sockets.hpp"

/**
 * @brief Data logger that subscribes to multiple NNG publishers and logs to binary file
 * 
 * Subscribes to preprocessor output channels (calibrated gyro, accel, mag, temp)
 * and writes timestamped binary records to a file for post-processing.
 * 
 * File format:
 * - Each record: [msg_type:uint8_t][timestamp_ns:uint64_t][data:variable_size]
 * - Message types: 1=gyro, 2=accel, 3=mag, 4=temp
 */
class DataLogger {
public:
    /**
     * @brief Construct logger with output file path
     * @param filepath Path to output binary file
     * @param log_gyro Enable gyroscope logging
     * @param log_accel Enable accelerometer logging
     * @param log_mag Enable magnetometer logging
     * @param log_temp Enable temperature logging
     */
    DataLogger(const std::string& filepath,
               bool log_gyro = true,
               bool log_accel = true, 
               bool log_mag = true,
               bool log_temp = true);

    /**
     * @brief Destructor - stops logging and closes file
     */
    ~DataLogger();

    /**
     * @brief Start logging in background thread
     * @return true if started successfully
     */
    bool start();

    /**
     * @brief Stop logging and flush data to disk
     */
    void stop();

    /**
     * @brief Check if logger is currently running
     */
    bool is_running() const { return running_.load(); }

    /**
     * @brief Get total number of messages logged
     */
    uint64_t get_message_count() const { return message_count_.load(); }

    /**
     * @brief Get total bytes written to file
     */
    uint64_t get_bytes_written() const { return bytes_written_.load(); }

private:
    /**
     * @brief Main logging thread function
     */
    void logging_thread_func();

    /**
     * @brief Write gyro message to file
     */
    void log_gyro_message(const messages::proc_gyro_msg_t& msg);

    /**
     * @brief Write accel message to file
     */
    void log_accel_message(const messages::proc_accel_msg_t& msg);

    /**
     * @brief Write mag message to file
     */
    void log_mag_message(const messages::proc_mag_msg_t& msg);

    /**
     * @brief Write temp message to file
     */
    void log_temp_message(const messages::proc_temp_msg_t& msg);

    // Configuration
    std::string filepath_;
    bool log_gyro_;
    bool log_accel_;
    bool log_mag_;
    bool log_temp_;

    // File handle
    std::ofstream file_;

    // NNG subscriber sockets
    std::unique_ptr<NngProcGyroSocket> nng_gyro_sub_;
    std::unique_ptr<NngProcAccelSocket> nng_accel_sub_;
    std::unique_ptr<NngProcMagSocket> nng_mag_sub_;
    std::unique_ptr<NngProcTempSocket> nng_temp_sub_;

    // Thread control
    std::atomic<bool> running_;
    std::thread logging_thread_;

    // Statistics
    std::atomic<uint64_t> message_count_;
    std::atomic<uint64_t> bytes_written_;

    // Message type identifiers for binary format
    static constexpr uint8_t MSG_TYPE_GYRO = 1;
    static constexpr uint8_t MSG_TYPE_ACCEL = 2;
    static constexpr uint8_t MSG_TYPE_MAG = 3;
    static constexpr uint8_t MSG_TYPE_TEMP = 4;
};
