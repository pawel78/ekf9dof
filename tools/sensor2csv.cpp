/**
 * @file sensor2csv.cpp
 * @brief Convert per-sensor binary log files to CSV format
 * 
 * Binary Format (per sensor file):
 *   Header: 
 *     - Magic: 0x494D5532 ("IMU2") - 4 bytes
 *     - Version: 2 - 2 bytes
 *   
 *   Records (no type byte needed - file is sensor-specific):
 *     Gyro/Accel/Mag:  timestamp (8) + x/y/z (12) = 20 bytes
 *     Temperature:     timestamp (8) + temp (4)   = 12 bytes
 * 
 * Usage:
 *   sensor2csv gyro base_gyro.bin base_gyro.csv
 *   sensor2csv accel base_accel.bin base_accel.csv
 *   sensor2csv mag base_mag.bin base_mag.csv
 *   sensor2csv temp base_temp.bin base_temp.csv
 */

#include <iostream>
#include <fstream>
#include <string>
#include <cstdint>
#include <iomanip>

enum class SensorType {
    GYRO,
    ACCEL,
    MAG,
    TEMP
};

bool read_header(std::ifstream& file) {
    uint32_t magic;
    uint16_t version;
    
    file.read(reinterpret_cast<char*>(&magic), sizeof(magic));
    file.read(reinterpret_cast<char*>(&version), sizeof(version));
    
    if (!file.good()) {
        std::cerr << "ERROR: Failed to read file header\n";
        return false;
    }
    
    if (magic != 0x494D5532) {
        std::cerr << "ERROR: Invalid magic number: 0x" << std::hex << magic << std::dec << "\n";
        std::cerr << "       Expected: 0x494D5532 (IMU2)\n";
        return false;
    }
    
    if (version != 2) {
        std::cerr << "WARNING: Unexpected version: " << version << " (expected 2)\n";
    }
    
    std::cout << "File format: IMU2 v" << version << "\n";
    return true;
}

bool convert_gyro(const std::string& input_file, const std::string& output_file) {
    std::ifstream in(input_file, std::ios::binary);
    if (!in.is_open()) {
        std::cerr << "ERROR: Cannot open input file: " << input_file << "\n";
        return false;
    }
    
    if (!read_header(in)) {
        return false;
    }
    
    std::ofstream out(output_file);
    if (!out.is_open()) {
        std::cerr << "ERROR: Cannot open output file: " << output_file << "\n";
        return false;
    }
    
    // Write CSV header
    out << "timestamp_ns,timestamp_sec,gyro_x_dps,gyro_y_dps,gyro_z_dps\n";
    out << std::fixed << std::setprecision(6);
    
    uint64_t timestamp_ns;
    float gx, gy, gz;
    int record_count = 0;
    
    while (in.read(reinterpret_cast<char*>(&timestamp_ns), sizeof(timestamp_ns))) {
        in.read(reinterpret_cast<char*>(&gx), sizeof(float));
        in.read(reinterpret_cast<char*>(&gy), sizeof(float));
        in.read(reinterpret_cast<char*>(&gz), sizeof(float));
        
        if (!in.good()) {
            std::cerr << "WARNING: Incomplete record at end of file\n";
            break;
        }
        
        double timestamp_sec = timestamp_ns / 1e9;
        out << timestamp_ns << "," << timestamp_sec << ","
            << gx << "," << gy << "," << gz << "\n";
        
        record_count++;
    }
    
    std::cout << "Converted " << record_count << " gyro records\n";
    return true;
}

bool convert_accel(const std::string& input_file, const std::string& output_file) {
    std::ifstream in(input_file, std::ios::binary);
    if (!in.is_open()) {
        std::cerr << "ERROR: Cannot open input file: " << input_file << "\n";
        return false;
    }
    
    if (!read_header(in)) {
        return false;
    }
    
    std::ofstream out(output_file);
    if (!out.is_open()) {
        std::cerr << "ERROR: Cannot open output file: " << output_file << "\n";
        return false;
    }
    
    // Write CSV header
    out << "timestamp_ns,timestamp_sec,accel_x_g,accel_y_g,accel_z_g\n";
    out << std::fixed << std::setprecision(6);
    
    uint64_t timestamp_ns;
    float ax, ay, az;
    int record_count = 0;
    
    while (in.read(reinterpret_cast<char*>(&timestamp_ns), sizeof(timestamp_ns))) {
        in.read(reinterpret_cast<char*>(&ax), sizeof(float));
        in.read(reinterpret_cast<char*>(&ay), sizeof(float));
        in.read(reinterpret_cast<char*>(&az), sizeof(float));
        
        if (!in.good()) {
            std::cerr << "WARNING: Incomplete record at end of file\n";
            break;
        }
        
        double timestamp_sec = timestamp_ns / 1e9;
        out << timestamp_ns << "," << timestamp_sec << ","
            << ax << "," << ay << "," << az << "\n";
        
        record_count++;
    }
    
    std::cout << "Converted " << record_count << " accel records\n";
    return true;
}

bool convert_mag(const std::string& input_file, const std::string& output_file) {
    std::ifstream in(input_file, std::ios::binary);
    if (!in.is_open()) {
        std::cerr << "ERROR: Cannot open input file: " << input_file << "\n";
        return false;
    }
    
    if (!read_header(in)) {
        return false;
    }
    
    std::ofstream out(output_file);
    if (!out.is_open()) {
        std::cerr << "ERROR: Cannot open output file: " << output_file << "\n";
        return false;
    }
    
    // Write CSV header
    out << "timestamp_ns,timestamp_sec,mag_x_gauss,mag_y_gauss,mag_z_gauss\n";
    out << std::fixed << std::setprecision(6);
    
    uint64_t timestamp_ns;
    float mx, my, mz;
    int record_count = 0;
    
    while (in.read(reinterpret_cast<char*>(&timestamp_ns), sizeof(timestamp_ns))) {
        in.read(reinterpret_cast<char*>(&mx), sizeof(float));
        in.read(reinterpret_cast<char*>(&my), sizeof(float));
        in.read(reinterpret_cast<char*>(&mz), sizeof(float));
        
        if (!in.good()) {
            std::cerr << "WARNING: Incomplete record at end of file\n";
            break;
        }
        
        double timestamp_sec = timestamp_ns / 1e9;
        out << timestamp_ns << "," << timestamp_sec << ","
            << mx << "," << my << "," << mz << "\n";
        
        record_count++;
    }
    
    std::cout << "Converted " << record_count << " mag records\n";
    return true;
}

bool convert_temp(const std::string& input_file, const std::string& output_file) {
    std::ifstream in(input_file, std::ios::binary);
    if (!in.is_open()) {
        std::cerr << "ERROR: Cannot open input file: " << input_file << "\n";
        return false;
    }
    
    if (!read_header(in)) {
        return false;
    }
    
    std::ofstream out(output_file);
    if (!out.is_open()) {
        std::cerr << "ERROR: Cannot open output file: " << output_file << "\n";
        return false;
    }
    
    // Write CSV header
    out << "timestamp_ns,timestamp_sec,temperature_c\n";
    out << std::fixed << std::setprecision(6);
    
    uint64_t timestamp_ns;
    float temp;
    int record_count = 0;
    
    while (in.read(reinterpret_cast<char*>(&timestamp_ns), sizeof(timestamp_ns))) {
        in.read(reinterpret_cast<char*>(&temp), sizeof(float));
        
        if (!in.good()) {
            std::cerr << "WARNING: Incomplete record at end of file\n";
            break;
        }
        
        double timestamp_sec = timestamp_ns / 1e9;
        out << timestamp_ns << "," << timestamp_sec << "," << temp << "\n";
        
        record_count++;
    }
    
    std::cout << "Converted " << record_count << " temp records\n";
    return true;
}

void print_usage(const char* program_name) {
    std::cerr << "Usage: " << program_name << " <sensor_type> <input.bin> <output.csv>\n";
    std::cerr << "\nSensor types:\n";
    std::cerr << "  gyro   - Convert gyroscope data\n";
    std::cerr << "  accel  - Convert accelerometer data\n";
    std::cerr << "  mag    - Convert magnetometer data\n";
    std::cerr << "  temp   - Convert temperature data\n";
    std::cerr << "\nExample:\n";
    std::cerr << "  " << program_name << " gyro data_gyro.bin data_gyro.csv\n";
}

int main(int argc, char* argv[]) {
    if (argc != 4) {
        print_usage(argv[0]);
        return 1;
    }
    
    std::string sensor_type = argv[1];
    std::string input_file = argv[2];
    std::string output_file = argv[3];
    
    std::cout << "Converting " << sensor_type << " data...\n";
    std::cout << "Input:  " << input_file << "\n";
    std::cout << "Output: " << output_file << "\n\n";
    
    bool success = false;
    
    if (sensor_type == "gyro") {
        success = convert_gyro(input_file, output_file);
    } else if (sensor_type == "accel") {
        success = convert_accel(input_file, output_file);
    } else if (sensor_type == "mag") {
        success = convert_mag(input_file, output_file);
    } else if (sensor_type == "temp") {
        success = convert_temp(input_file, output_file);
    } else {
        std::cerr << "ERROR: Unknown sensor type: " << sensor_type << "\n\n";
        print_usage(argv[0]);
        return 1;
    }
    
    if (success) {
        std::cout << "\nConversion completed successfully!\n";
        return 0;
    } else {
        std::cerr << "\nConversion failed!\n";
        return 1;
    }
}
