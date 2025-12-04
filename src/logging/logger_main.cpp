#include "logging/data_logger.hpp"
#include <iostream>
#include <csignal>
#include <atomic>
#include <thread>

std::atomic<bool> keep_running(true);

void signal_handler(int signal) {
    std::cout << "\nCaught signal " << signal << ", stopping...\n";
    keep_running.store(false);
}

int main(int argc, char* argv[]) {
    // Setup signal handling for clean shutdown
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    // Parse command line arguments
    std::string output_file = "data/imu_log.bin";
    bool log_gyro = true;
    bool log_accel = true;
    bool log_mag = true;
    bool log_temp = true;

    if (argc > 1) {
        output_file = argv[1];
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
