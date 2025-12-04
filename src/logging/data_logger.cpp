#include "logging/data_logger.hpp"
#include <iostream>
#include <cstring>
#include <nng/nng.h>

DataLogger::DataLogger(const std::string& filepath,
                       bool log_gyro,
                       bool log_accel,
                       bool log_mag,
                       bool log_temp)
    : filepath_(filepath),
      log_gyro_(log_gyro),
      log_accel_(log_accel),
      log_mag_(log_mag),
      log_temp_(log_temp),
      running_(false),
      message_count_(0),
      bytes_written_(0) {
}

DataLogger::~DataLogger() {
    stop();
}

bool DataLogger::start() {
    if (running_.load()) {
        std::cerr << "DataLogger: Already running\n";
        return false;
    }

    // Open output file
    file_.open(filepath_, std::ios::binary | std::ios::out | std::ios::trunc);
    if (!file_.is_open()) {
        std::cerr << "DataLogger: Failed to open file: " << filepath_ << "\n";
        return false;
    }

    // Write file header
    const char header[] = "EKF9DOF_LOG_V1";
    file_.write(header, sizeof(header));
    bytes_written_ += sizeof(header);

    // Initialize NNG subscriber sockets
    try {
        if (log_gyro_) {
            nng_gyro_sub_ = std::make_unique<NngProcGyroSocket>(
                nng_urls::PROC_GYRO, false);
            std::cout << "DataLogger: Subscribed to gyro at " << nng_urls::PROC_GYRO << "\n";
        }

        if (log_accel_) {
            nng_accel_sub_ = std::make_unique<NngProcAccelSocket>(
                nng_urls::PROC_ACCEL, false);
            std::cout << "DataLogger: Subscribed to accel at " << nng_urls::PROC_ACCEL << "\n";
        }

        if (log_mag_) {
            nng_mag_sub_ = std::make_unique<NngProcMagSocket>(
                nng_urls::PROC_MAG, false);
            std::cout << "DataLogger: Subscribed to mag at " << nng_urls::PROC_MAG << "\n";
        }

        if (log_temp_) {
            nng_temp_sub_ = std::make_unique<NngProcTempSocket>(
                nng_urls::PROC_TEMP, false);
            std::cout << "DataLogger: Subscribed to temp at " << nng_urls::PROC_TEMP << "\n";
        }
    } catch (const std::exception& e) {
        std::cerr << "DataLogger: Failed to create NNG sockets: " << e.what() << "\n";
        file_.close();
        return false;
    }

    // Start logging thread
    running_.store(true);
    logging_thread_ = std::thread(&DataLogger::logging_thread_func, this);

    std::cout << "DataLogger: Started logging to " << filepath_ << "\n";
    return true;
}

void DataLogger::stop() {
    if (!running_.load()) {
        return;
    }

    std::cout << "DataLogger: Stopping...\n";
    running_.store(false);

    if (logging_thread_.joinable()) {
        logging_thread_.join();
    }

    // Close sockets
    nng_gyro_sub_.reset();
    nng_accel_sub_.reset();
    nng_mag_sub_.reset();
    nng_temp_sub_.reset();

    // Close file
    if (file_.is_open()) {
        file_.flush();
        file_.close();
    }

    std::cout << "DataLogger: Stopped. Logged " << message_count_.load() 
              << " messages (" << bytes_written_.load() << " bytes)\n";
}

void DataLogger::logging_thread_func() {
    messages::proc_gyro_msg_t gyro_msg;
    messages::proc_accel_msg_t accel_msg;
    messages::proc_mag_msg_t mag_msg;
    messages::proc_temp_msg_t temp_msg;

    while (running_.load()) {
        bool received_any = false;

        // Try to receive gyro message (non-blocking)
        if (log_gyro_ && nng_gyro_sub_) {
            if (nng_gyro_sub_->try_receive(gyro_msg)) {
                log_gyro_message(gyro_msg);
                received_any = true;
            }
        }

        // Try to receive accel message (non-blocking)
        if (log_accel_ && nng_accel_sub_) {
            if (nng_accel_sub_->try_receive(accel_msg)) {
                log_accel_message(accel_msg);
                received_any = true;
            }
        }

        // Try to receive mag message (non-blocking)
        if (log_mag_ && nng_mag_sub_) {
            if (nng_mag_sub_->try_receive(mag_msg)) {
                log_mag_message(mag_msg);
                received_any = true;
            }
        }

        // Try to receive temp message (non-blocking)
        if (log_temp_ && nng_temp_sub_) {
            if (nng_temp_sub_->try_receive(temp_msg)) {
                log_temp_message(temp_msg);
                received_any = true;
            }
        }

        // Periodically flush to disk
        if (received_any && (message_count_.load() % 100 == 0)) {
            file_.flush();
        }

        // Small sleep if no messages received to avoid busy-waiting
        if (!received_any) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    // Final flush on exit
    file_.flush();
}

void DataLogger::log_gyro_message(const messages::proc_gyro_msg_t& msg) {
    // Write: [type:1][timestamp:8][x:4][y:4][z:4] = 21 bytes
    file_.write(reinterpret_cast<const char*>(&MSG_TYPE_GYRO), sizeof(uint8_t));
    file_.write(reinterpret_cast<const char*>(&msg.timestamp_ns), sizeof(uint64_t));
    file_.write(reinterpret_cast<const char*>(&msg.x), sizeof(float));
    file_.write(reinterpret_cast<const char*>(&msg.y), sizeof(float));
    file_.write(reinterpret_cast<const char*>(&msg.z), sizeof(float));

    message_count_++;
    bytes_written_ += (1 + 8 + 12);
}

void DataLogger::log_accel_message(const messages::proc_accel_msg_t& msg) {
    // Write: [type:1][timestamp:8][x:4][y:4][z:4] = 21 bytes
    file_.write(reinterpret_cast<const char*>(&MSG_TYPE_ACCEL), sizeof(uint8_t));
    file_.write(reinterpret_cast<const char*>(&msg.timestamp_ns), sizeof(uint64_t));
    file_.write(reinterpret_cast<const char*>(&msg.x), sizeof(float));
    file_.write(reinterpret_cast<const char*>(&msg.y), sizeof(float));
    file_.write(reinterpret_cast<const char*>(&msg.z), sizeof(float));

    message_count_++;
    bytes_written_ += (1 + 8 + 12);
}

void DataLogger::log_mag_message(const messages::proc_mag_msg_t& msg) {
    // Write: [type:1][timestamp:8][x:4][y:4][z:4] = 21 bytes
    file_.write(reinterpret_cast<const char*>(&MSG_TYPE_MAG), sizeof(uint8_t));
    file_.write(reinterpret_cast<const char*>(&msg.timestamp_ns), sizeof(uint64_t));
    file_.write(reinterpret_cast<const char*>(&msg.x), sizeof(float));
    file_.write(reinterpret_cast<const char*>(&msg.y), sizeof(float));
    file_.write(reinterpret_cast<const char*>(&msg.z), sizeof(float));

    message_count_++;
    bytes_written_ += (1 + 8 + 12);
}

void DataLogger::log_temp_message(const messages::proc_temp_msg_t& msg) {
    // Write: [type:1][timestamp:8][temp:4] = 13 bytes
    file_.write(reinterpret_cast<const char*>(&MSG_TYPE_TEMP), sizeof(uint8_t));
    file_.write(reinterpret_cast<const char*>(&msg.timestamp_ns), sizeof(uint64_t));
    file_.write(reinterpret_cast<const char*>(&msg.temp_c), sizeof(float));

    message_count_++;
    bytes_written_ += (1 + 8 + 4);
}
