/**
 * @file bin2csv.cpp
 * @brief Convert LSM9DS0 binary log files (v1 and v2) to CSV format
 * 
 * Supports two binary formats:
 * 
 * V1 Format (Magic: 0x494D5539 "IMU9"):
 *   - Fixed 48-byte records with all sensors
 *   - All sensors at same rate (200 Hz)
 * 
 * V2 Format (Magic: 0x494D5532 "IMU2"):
 *   - Variable-length records with type field
 *   - Per-sensor rates: Gyro 200 Hz, Accel/Mag/Temp ~67 Hz each
 *   - Record types:
 *     0x01: Gyro  (1 + 8 + 12 = 21 bytes)
 *     0x02: Accel (1 + 8 + 12 = 21 bytes)
 *     0x03: Mag   (1 + 8 + 12 = 21 bytes)
 *     0x04: Temp  (1 + 8 + 4  = 13 bytes)
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
#include <map>
#include <cstring>

// Record types for v2 format
enum RecordType : uint8_t {
    GYRO  = 0x01,
    ACCEL = 0x02,
    MAG   = 0x03,
    TEMP  = 0x04
};

struct SensorData {
    uint64_t timestamp_ns;
    float ax, ay, az;        // Acceleration (g)
    float gx, gy, gz;        // Gyroscope (deg/s)
    float mx, my, mz;        // Magnetometer (gauss)
    float temperature;       // Temperature (°C)
    
    // Flags to track which sensors have been updated
    bool has_accel;
    bool has_gyro;
    bool has_mag;
    bool has_temp;
    
    SensorData() : timestamp_ns(0), ax(0), ay(0), az(0), 
                   gx(0), gy(0), gz(0), mx(0), my(0), mz(0), 
                   temperature(0),
                   has_accel(false), has_gyro(false), 
                   has_mag(false), has_temp(false) {}
};

bool read_header(std::ifstream& file, uint32_t& magic, uint32_t& version, uint32_t& extra) {
    file.read(reinterpret_cast<char*>(&magic), sizeof(magic));
    file.read(reinterpret_cast<char*>(&version), sizeof(version));
    file.read(reinterpret_cast<char*>(&extra), sizeof(extra));
    
    if (!file.good()) {
        std::cerr << "ERROR: Failed to read file header\n";
        return false;
    }
    
    if (magic == 0x494D5539) {
        std::cout << "File format: V1 (IMU9)\n";
        std::cout << "Version: " << version << "\n";
        std::cout << "Sample size: " << extra << " bytes\n";
    } else if (magic == 0x494D5532) {
        std::cout << "File format: V2 (IMU2)\n";
        std::cout << "Version: " << version << "\n";
        std::cout << "Header size: " << extra << " bytes\n";
    } else {
        std::cerr << "ERROR: Invalid magic number: 0x" << std::hex << magic << std::dec << "\n";
        std::cerr << "       Expected: 0x494D5539 (v1) or 0x494D5532 (v2)\n";
        return false;
    }
    
    return true;
}

// Read V1 format (fixed 48-byte records)
bool read_v1_sample(std::ifstream& file, SensorData& data) {
    file.read(reinterpret_cast<char*>(&data.timestamp_ns), sizeof(data.timestamp_ns));
    if (file.gcount() != sizeof(data.timestamp_ns)) {
        return false;
    }
    
    file.read(reinterpret_cast<char*>(&data.ax), sizeof(float));
    file.read(reinterpret_cast<char*>(&data.ay), sizeof(float));
    file.read(reinterpret_cast<char*>(&data.az), sizeof(float));
    file.read(reinterpret_cast<char*>(&data.gx), sizeof(float));
    file.read(reinterpret_cast<char*>(&data.gy), sizeof(float));
    file.read(reinterpret_cast<char*>(&data.gz), sizeof(float));
    file.read(reinterpret_cast<char*>(&data.mx), sizeof(float));
    file.read(reinterpret_cast<char*>(&data.my), sizeof(float));
    file.read(reinterpret_cast<char*>(&data.mz), sizeof(float));
    file.read(reinterpret_cast<char*>(&data.temperature), sizeof(float));
    
    data.has_accel = data.has_gyro = data.has_mag = data.has_temp = true;
    return file.good();
}

// Read V2 format (variable-length records with type)
bool read_v2_record(std::ifstream& file, SensorData& data) {
    uint8_t record_type;
    uint64_t timestamp_ns;
    
    file.read(reinterpret_cast<char*>(&record_type), sizeof(record_type));
    if (file.gcount() != sizeof(record_type)) {
        return false; // EOF
    }
    
    file.read(reinterpret_cast<char*>(&timestamp_ns), sizeof(timestamp_ns));
    
    switch (record_type) {
        case GYRO:
            file.read(reinterpret_cast<char*>(&data.gx), sizeof(float));
            file.read(reinterpret_cast<char*>(&data.gy), sizeof(float));
            file.read(reinterpret_cast<char*>(&data.gz), sizeof(float));
            data.timestamp_ns = timestamp_ns;
            data.has_gyro = true;
            break;
            
        case ACCEL:
            file.read(reinterpret_cast<char*>(&data.ax), sizeof(float));
            file.read(reinterpret_cast<char*>(&data.ay), sizeof(float));
            file.read(reinterpret_cast<char*>(&data.az), sizeof(float));
            data.timestamp_ns = timestamp_ns;
            data.has_accel = true;
            break;
            
        case MAG:
            file.read(reinterpret_cast<char*>(&data.mx), sizeof(float));
            file.read(reinterpret_cast<char*>(&data.my), sizeof(float));
            file.read(reinterpret_cast<char*>(&data.mz), sizeof(float));
            data.timestamp_ns = timestamp_ns;
            data.has_mag = true;
            break;
            
        case TEMP:
            file.read(reinterpret_cast<char*>(&data.temperature), sizeof(float));
            data.timestamp_ns = timestamp_ns;
            data.has_temp = true;
            break;
            
        default:
            std::cerr << "ERROR: Unknown record type: 0x" << std::hex 
                      << static_cast<int>(record_type) << std::dec << "\n";
            return false;
    }
    
    return file.good();
}

void write_csv_header(std::ofstream& csv) {
    csv << "timestamp_ns,timestamp_sec,";
    csv << "accel_x_g,accel_y_g,accel_z_g,";
    csv << "gyro_x_dps,gyro_y_dps,gyro_z_dps,";
    csv << "mag_x_gauss,mag_y_gauss,mag_z_gauss,";
    csv << "temperature_c\n";
}

void write_csv_sample(std::ofstream& csv, const SensorData& data) {
    double timestamp_sec = static_cast<double>(data.timestamp_ns) / 1e9;
    
    csv << std::fixed;
    csv << data.timestamp_ns << ",";
    csv << std::setprecision(6) << timestamp_sec << ",";
    csv << std::setprecision(6) << data.ax << "," << data.ay << "," << data.az << ",";
    csv << std::setprecision(3) << data.gx << "," << data.gy << "," << data.gz << ",";
    csv << std::setprecision(6) << data.mx << "," << data.my << "," << data.mz << ",";
    csv << std::setprecision(2) << data.temperature << "\n";
}

int main(int argc, char* argv[]) {
    if (argc < 2 || argc > 3) {
        std::cerr << "Usage: " << argv[0] << " <input.bin> [output.csv]\n";
        std::cerr << "       If output.csv is not specified, uses input filename with .csv extension\n";
        return 1;
    }
    
    std::string input_file = argv[1];
    std::string output_file;
    
    if (argc == 3) {
        output_file = argv[2];
    } else {
        // Replace .bin extension with .csv
        size_t dot_pos = input_file.find_last_of('.');
        if (dot_pos != std::string::npos) {
            output_file = input_file.substr(0, dot_pos) + ".csv";
        } else {
            output_file = input_file + ".csv";
        }
    }
    
    std::cout << "Converting: " << input_file << " -> " << output_file << "\n";
    
    // Open binary input file
    std::ifstream bin_file(input_file, std::ios::binary);
    if (!bin_file.is_open()) {
        std::cerr << "ERROR: Cannot open input file: " << input_file << "\n";
        return 1;
    }
    
    // Read and validate header
    uint32_t magic, version, extra;
    if (!read_header(bin_file, magic, version, extra)) {
        return 1;
    }
    
    bool is_v1 = (magic == 0x494D5539);
    bool is_v2 = (magic == 0x494D5532);
    
    // Open CSV output file
    std::ofstream csv_file(output_file);
    if (!csv_file.is_open()) {
        std::cerr << "ERROR: Cannot create output file: " << output_file << "\n";
        return 1;
    }
    
    write_csv_header(csv_file);
    
    // Convert data
    uint64_t sample_count = 0;
    uint64_t first_timestamp = 0;
    uint64_t last_timestamp = 0;
    
    if (is_v1) {
        // V1: Read fixed-size records
        SensorData data;
        while (read_v1_sample(bin_file, data)) {
            if (sample_count == 0) {
                first_timestamp = data.timestamp_ns;
            }
            last_timestamp = data.timestamp_ns;
            
            write_csv_sample(csv_file, data);
            sample_count++;
            
            if (sample_count % 1000 == 0) {
                std::cout << "\rProcessed " << sample_count << " samples..." << std::flush;
            }
        }
    } else if (is_v2) {
        // V2: Read variable-length records, output when we have gyro
        // (gyro is at 200 Hz, others at ~67 Hz)
        SensorData data;
        while (read_v2_record(bin_file, data)) {
            // Output a CSV row whenever we read a gyro sample
            // This gives us 200 Hz output with the latest values from other sensors
            if (data.has_gyro) {
                if (sample_count == 0) {
                    first_timestamp = data.timestamp_ns;
                }
                last_timestamp = data.timestamp_ns;
                
                write_csv_sample(csv_file, data);
                sample_count++;
                
                if (sample_count % 1000 == 0) {
                    std::cout << "\rProcessed " << sample_count << " samples..." << std::flush;
                }
            }
        }
    }
    
    std::cout << "\rProcessed " << sample_count << " samples...     \n";
    
    // Print statistics
    if (sample_count > 0) {
        double duration_sec = static_cast<double>(last_timestamp - first_timestamp) / 1e9;
        double avg_rate = (sample_count - 1) / duration_sec;
        
        std::cout << "\n=== Conversion Complete ===\n";
        std::cout << "Total samples: " << sample_count << "\n";
        std::cout << "Duration: " << std::fixed << std::setprecision(3) << duration_sec << " seconds\n";
        std::cout << "Average rate: " << std::fixed << std::setprecision(1) << avg_rate << " Hz\n";
        std::cout << "Output file: " << output_file << "\n";
    } else {
        std::cerr << "WARNING: No samples found in input file\n";
    }
    
    bin_file.close();
    csv_file.close();
    
    return 0;
}
