#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include "common/config_loader.hpp"

namespace config_loader {

// Helper function to parse a float array from a YAML-like string
// Expected format: [val1, val2, val3] or [val1,val2,val3, val4,val5,val6, val7,val8,val9]
static bool parse_float_array(const std::string& str, std::vector<float>& values) {
    values.clear();
    
    // Find the opening and closing brackets
    size_t start = str.find('[');
    size_t end = str.find(']');
    
    if (start == std::string::npos || end == std::string::npos) {
        return false;
    }
    
    // Extract the content between brackets
    std::string content = str.substr(start + 1, end - start - 1);
    
    // Parse comma-separated values
    std::istringstream ss(content);
    std::string token;
    
    while (std::getline(ss, token, ',')) {
        // Trim whitespace
        token.erase(0, token.find_first_not_of(" \t\n\r"));
        token.erase(token.find_last_not_of(" \t\n\r") + 1);
        
        if (!token.empty()) {
            try {
                values.push_back(std::stof(token));
            } catch (...) {
                return false;
            }
        }
    }
    
    return !values.empty();
}

bool load_mag_calibration(const std::string& config_path,
                          std::array<float, 3>& mag_bias,
                          std::array<float, 9>& mag_matrix) {
    std::ifstream file(config_path);
    if (!file.is_open()) {
        return false;
    }
    
    bool found_bias = false;
    bool found_matrix = false;
    bool in_calibration_section = false;
    
    std::string line;
    std::string matrix_content;
    
    while (std::getline(file, line)) {
        // Check if we're in the calibration section
        if (line.find("calibration:") != std::string::npos) {
            in_calibration_section = true;
            continue;
        }
        
        // Check if we've left the calibration section
        if (in_calibration_section && !line.empty() && line[0] != ' ' && line[0] != '\t') {
            in_calibration_section = false;
        }
        
        if (!in_calibration_section) {
            continue;
        }
        
        // Look for mag_bias
        if (line.find("mag_bias:") != std::string::npos) {
            std::vector<float> values;
            if (parse_float_array(line, values) && values.size() == 3) {
                for (size_t i = 0; i < 3; ++i) {
                    mag_bias[i] = values[i];
                }
                found_bias = true;
            }
        }
        
        // Look for mag_matrix (may span multiple lines)
        if (line.find("mag_matrix:") != std::string::npos) {
            matrix_content = line;
            
            // If the matrix doesn't close on the same line, read more lines
            if (matrix_content.find(']') == std::string::npos) {
                std::string next_line;
                while (std::getline(file, next_line)) {
                    matrix_content += next_line;
                    if (next_line.find(']') != std::string::npos) {
                        break;
                    }
                }
            }
            
            std::vector<float> values;
            if (parse_float_array(matrix_content, values) && values.size() == 9) {
                for (size_t i = 0; i < 9; ++i) {
                    mag_matrix[i] = values[i];
                }
                found_matrix = true;
            }
        }
    }
    
    file.close();
    return found_bias && found_matrix;
}

} // namespace config_loader
