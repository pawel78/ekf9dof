/**
 * @file bin2csv.cpp
 * @brief Convert LSM9DS0 binary log files to CSV format
 * 
 * This tool reads binary log files created by the LSM9DS0 driver
 * and converts them to human-readable CSV format for offline analysis.
 * 
 * Binary format:
 * - Header (12 bytes):
 *   - Magic number (4 bytes): 0x494D5539 ("IMU9")
 *   - Version (4 bytes): 1
 *   - Sample size (4 bytes): 48 (8 + 40)
 * - Data records (48 bytes each):
 *   - Timestamp (8 bytes): uint64_t nanoseconds
 *   - Accel X (4 bytes): float (g)
 *   - Accel Y (4 bytes): float (g)
 *   - Accel Z (4 bytes): float (g)
 *   - Gyro X (4 bytes): float (deg/s)
 *   - Gyro Y (4 bytes): float (deg/s)
 *   - Gyro Z (4 bytes): float (deg/s)
 *   - Mag X (4 bytes): float (gauss)
 *   - Mag Y (4 bytes): float (gauss)
 *   - Mag Z (4 bytes): float (gauss)
 *   - Temperature (4 bytes): float (°C)
 * 
 * Usage:
 *   bin2csv input.bin output.csv
 *   bin2csv input.bin  (outputs to input.csv)
 */

#include <iostream>
#include <fstream>
#include <string>
#include <cstdint>
#include <iomanip>
#include <vector>

struct IMUSample {
    uint64_t timestamp_ns;
    float ax, ay, az;        // Acceleration (g)
    float gx, gy, gz;        // Gyroscope (deg/s)
    float mx, my, mz;        // Magnetometer (gauss)
    float temperature;       // Temperature (°C)
};

bool read_header(std::ifstream& file, uint32_t& version, uint32_t& sample_size) {
    uint32_t magic;
    file.read(reinterpret_cast<char*>(&magic), sizeof(magic));
    
    if (magic != 0x494D5539) {
        std::cerr << "ERROR: Invalid magic number. Not a valid IMU binary file.\n";
        std::cerr << "       Expected: 0x494D5539, Got: 0x" << std::hex << magic << std::dec << "\n";
        return false;
    }
    
    file.read(reinterpret_cast<char*>(&version), sizeof(version));
    file.read(reinterpret_cast<char*>(&sample_size), sizeof(sample_size));
    
    std::cout << "Binary file format version: " << version << "\n";
    std::cout << "Sample size: " << sample_size << " bytes\n";
    
    if (sample_size != 48) {
        std::cerr << "WARNING: Unexpected sample size. Expected 48, got " << sample_size << "\n";
    }
    
    return true;
}

bool read_sample(std::ifstream& file, IMUSample& sample) {
    file.read(reinterpret_cast<char*>(&sample.timestamp_ns), sizeof(sample.timestamp_ns));
    if (file.gcount() != sizeof(sample.timestamp_ns)) {
        return false; // End of file or read error
    }
    
    file.read(reinterpret_cast<char*>(&sample.ax), sizeof(float));
    file.read(reinterpret_cast<char*>(&sample.ay), sizeof(float));
    file.read(reinterpret_cast<char*>(&sample.az), sizeof(float));
    file.read(reinterpret_cast<char*>(&sample.gx), sizeof(float));
    file.read(reinterpret_cast<char*>(&sample.gy), sizeof(float));
    file.read(reinterpret_cast<char*>(&sample.gz), sizeof(float));
    file.read(reinterpret_cast<char*>(&sample.mx), sizeof(float));
    file.read(reinterpret_cast<char*>(&sample.my), sizeof(float));
    file.read(reinterpret_cast<char*>(&sample.mz), sizeof(float));
    file.read(reinterpret_cast<char*>(&sample.temperature), sizeof(float));
    
    return file.good();
}

void write_csv_header(std::ofstream& csv) {
    csv << "timestamp_ns,timestamp_sec,";
    csv << "accel_x_g,accel_y_g,accel_z_g,";
    csv << "gyro_x_dps,gyro_y_dps,gyro_z_dps,";
    csv << "mag_x_gauss,mag_y_gauss,mag_z_gauss,";
    csv << "temperature_c\n";
}

void write_csv_sample(std::ofstream& csv, const IMUSample& sample) {
    double timestamp_sec = static_cast<double>(sample.timestamp_ns) / 1e9;
    
    csv << std::fixed;
    csv << sample.timestamp_ns << ",";
    csv << std::setprecision(9) << timestamp_sec << ",";
    csv << std::setprecision(6) << sample.ax << ",";
    csv << std::setprecision(6) << sample.ay << ",";
    csv << std::setprecision(6) << sample.az << ",";
    csv << std::setprecision(3) << sample.gx << ",";
    csv << std::setprecision(3) << sample.gy << ",";
    csv << std::setprecision(3) << sample.gz << ",";
    csv << std::setprecision(6) << sample.mx << ",";
    csv << std::setprecision(6) << sample.my << ",";
    csv << std::setprecision(6) << sample.mz << ",";
    csv << std::setprecision(2) << sample.temperature << "\n";
}

int main(int argc, char* argv[]) {
    if (argc < 2 || argc > 3) {
        std::cerr << "Usage: " << argv[0] << " <input.bin> [output.csv]\n";
        std::cerr << "\n";
        std::cerr << "Converts LSM9DS0 binary log files to CSV format.\n";
        std::cerr << "If output.csv is not specified, uses input filename with .csv extension.\n";
        return 1;
    }
    
    std::string input_file = argv[1];
    std::string output_file;
    
    if (argc == 3) {
        output_file = argv[2];
    } else {
        // Auto-generate output filename
        size_t dot_pos = input_file.rfind('.');
        if (dot_pos != std::string::npos) {
            output_file = input_file.substr(0, dot_pos) + ".csv";
        } else {
            output_file = input_file + ".csv";
        }
    }
    
    std::cout << "Converting binary log to CSV...\n";
    std::cout << "Input:  " << input_file << "\n";
    std::cout << "Output: " << output_file << "\n\n";
    
    // Open input file
    std::ifstream bin_file(input_file, std::ios::binary);
    if (!bin_file.is_open()) {
        std::cerr << "ERROR: Failed to open input file: " << input_file << "\n";
        return 1;
    }
    
    // Read and validate header
    uint32_t version, sample_size;
    if (!read_header(bin_file, version, sample_size)) {
        return 1;
    }
    
    // Open output file
    std::ofstream csv_file(output_file);
    if (!csv_file.is_open()) {
        std::cerr << "ERROR: Failed to open output file: " << output_file << "\n";
        return 1;
    }
    
    // Write CSV header
    write_csv_header(csv_file);
    
    // Convert samples
    IMUSample sample;
    uint64_t sample_count = 0;
    uint64_t first_timestamp = 0;
    uint64_t last_timestamp = 0;
    
    while (read_sample(bin_file, sample)) {
        write_csv_sample(csv_file, sample);
        
        if (sample_count == 0) {
            first_timestamp = sample.timestamp_ns;
        }
        last_timestamp = sample.timestamp_ns;
        sample_count++;
        
        // Progress indicator every 1000 samples
        if (sample_count % 1000 == 0) {
            std::cout << "\rProcessed " << sample_count << " samples..." << std::flush;
        }
    }
    
    std::cout << "\rProcessed " << sample_count << " samples     \n";
    
    if (sample_count > 0) {
        double duration_sec = static_cast<double>(last_timestamp - first_timestamp) / 1e9;
        double rate_hz = sample_count / duration_sec;
        
        std::cout << "\nStatistics:\n";
        std::cout << "  Total samples: " << sample_count << "\n";
        std::cout << "  Duration: " << std::fixed << std::setprecision(3) << duration_sec << " seconds\n";
        std::cout << "  Average rate: " << std::fixed << std::setprecision(1) << rate_hz << " Hz\n";
        std::cout << "\n✓ Conversion complete!\n";
    } else {
        std::cout << "WARNING: No data samples found in file.\n";
    }
    
    bin_file.close();
    csv_file.close();
    
    return 0;
}
