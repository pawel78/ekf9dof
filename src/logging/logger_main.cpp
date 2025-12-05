#include "logging/data_logger.hpp"
#include <iostream>
#include <csignal>
#include <atomic>
#include <thread>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <sys/stat.h>

std::atomic<bool> keep_running(true);

void signal_handler(int signal) {
    std::cout << "\nCaught signal " << signal << ", stopping...\n";
    keep_running.store(false);
}

std::string generate_timestamped_filename(const std::string& base_dir = "data/logs") {
    // Get current time
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    
    // Format: YYYYMMDD_HHMMSS
    std::tm tm_now;
    localtime_r(&time_t_now, &tm_now);
    
    std::ostringstream oss;
    oss << base_dir << "/imu_log_"
        << std::setfill('0')
        << std::setw(4) << (tm_now.tm_year + 1900)
        << std::setw(2) << (tm_now.tm_mon + 1)
        << std::setw(2) << tm_now.tm_mday
        << "_"
        << std::setw(2) << tm_now.tm_hour
        << std::setw(2) << tm_now.tm_min
        << std::setw(2) << tm_now.tm_sec
        << ".bin";
    
    return oss.str();
}

bool ensure_directory_exists(const std::string& path) {
    struct stat info;
    if (stat(path.c_str(), &info) != 0) {
        // Directory doesn't exist, try to create it
        std::string mkdir_cmd = "mkdir -p " + path;
        return system(mkdir_cmd.c_str()) == 0;
    } else if (info.st_mode & S_IFDIR) {
        return true;
    }
    return false;
}

int main(int argc, char* argv[]) {
    // Setup signal handling for clean shutdown
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    // Parse command line arguments
    std::string output_file;
    std::string log_dir = "data/logs";
    bool log_gyro = true;
    bool log_accel = true;
    bool log_mag = true;
    bool log_temp = true;

    if (argc > 1) {
        // User specified output file
        output_file = argv[1];
    } else {
        // Generate timestamped filename
        if (argc > 2) {
            log_dir = argv[2];
        }
        if (!ensure_directory_exists(log_dir)) {
            std::cerr << "Failed to create log directory: " << log_dir << "\n";
            return 1;
        }
        output_file = generate_timestamped_filename(log_dir);
    }

    std::cout << "=== EKF9DOF Data Logger ===\n";
    std::cout << "Output file: " << output_file << "\n";
    std::cout << "Logging: ";
    if (log_gyro) std::cout << "gyro ";
    if (log_accel) std::cout << "accel ";
    if (log_mag) std::cout << "mag ";
    if (log_temp) std::cout << "temp ";
    std::cout << "\n\n";

    // Create and start logger
    DataLogger logger(output_file, log_gyro, log_accel, log_mag, log_temp);
    
    if (!logger.start()) {
        std::cerr << "Failed to start logger!\n";
        return 1;
    }

    std::cout << "Logger started. Press Ctrl+C to stop.\n\n";

    // Status update loop
    uint64_t last_count = 0;
    while (keep_running.load()) {
        std::this_thread::sleep_for(std::chrono::seconds(5));
        
        if (!logger.is_running()) {
            std::cerr << "Logger stopped unexpectedly!\n";
            break;
        }

        uint64_t current_count = logger.get_message_count();
        uint64_t bytes = logger.get_bytes_written();
        uint64_t messages_per_sec = (current_count - last_count) / 5;
        
        std::cout << "Status: " << current_count << " messages logged ("
                  << bytes << " bytes, " << messages_per_sec << " msg/s)\n";
        
        last_count = current_count;
    }

    // Stop logger
    std::cout << "\nStopping logger...\n";
    logger.stop();

    std::cout << "\nFinal statistics:\n";
    std::cout << "  Total messages: " << logger.get_message_count() << "\n";
    std::cout << "  Total bytes: " << logger.get_bytes_written() << "\n";
    std::cout << "  Output file: " << output_file << "\n";

    return 0;
}
