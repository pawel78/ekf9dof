# LSM9DS0 Sensor Monitor

Real-time sensor data monitor with magnetometer calibration data logging.

## Quick Start

### 1. Build the Application

```bash
cd ekf9dof
mkdir -p build && cd build
cmake ..
make sensor_monitor
```

### 2. Run the Monitor

```bash
# Display real-time sensor data
./sensor_monitor

# Show help
./sensor_monitor --help

# Show calibration status
./sensor_monitor --info
```

## Features

### Real-time Display
- **Accelerometer**: Raw values in g (gravitational acceleration)
- **Gyroscope**: Raw values in degrees per second (dps)
- **Magnetometer**: Both raw and calibrated values in gauss
- **Temperature**: Sensor temperature in degrees Celsius

### Data Logging
- Automatically saves magnetometer calibration data to timestamped CSV files
- Format: `mag_cal_data_YYYYMMDD_HHMMSS.csv`
- Includes both raw and calibrated magnetometer readings for analysis

### Calibration Support
- Loads magnetometer calibration from `configs/config.yaml`
- Displays calibration status and parameters
- Shows difference between raw and calibrated readings in real-time

## Sample Output

```
================================================================================
LSM9DS0 SENSOR MONITOR - Real-time Display
================================================================================
Time        Accelerometer (g)             Gyroscope (dps)               Magnetometer Raw (gauss)           Magnetometer Cal (gauss)           Temp (°C)
--------------------------------------------------------------------------------
0.000       (-0.012, 0.043, 1.001)       (0.123, -0.087, 0.045)       (0.234, -0.123, 0.567)            (0.198, -0.089, 0.534)            23.45
0.200       (-0.008, 0.041, 0.998)       (0.098, -0.092, 0.038)       (0.237, -0.119, 0.571)            (0.201, -0.085, 0.538)            23.47
```

## Magnetometer Calibration

### Step 1: Check Current Status
```bash
./sensor_monitor --info
```

### Step 2: Calibrate (if needed)
```bash
./calibrate_mag
```

### Step 3: Verify Calibration
```bash
./sensor_monitor
```
Look for meaningful differences between "Raw" and "Cal" magnetometer columns.

## Data Logging

The sensor monitor automatically logs magnetometer data for calibration analysis:

### CSV Data Format
```csv
timestamp,mx_raw,my_raw,mz_raw,mx_cal,my_cal,mz_cal,mx_raw_gauss,my_raw_gauss,mz_raw_gauss,mx_cal_gauss,my_cal_gauss,mz_cal_gauss
2024-01-15_14:30:15.123,-234,145,567,-198,89,534,-0.234,0.145,0.567,-0.198,0.089,0.534
```

### Data Files
- Location: Current working directory
- Naming: `mag_cal_data_YYYYMMDD_HHMMSS.csv`
- Content: Raw and calibrated magnetometer data with timestamps
- Format: CSV with headers for easy import into analysis tools

### Using the Data
The logged data can be used for:
- Magnetometer calibration quality assessment
- Calibration parameter validation  
- Manual calibration algorithm development
- Data visualization and analysis

## Troubleshooting

### No Sensor Data
- Check I2C connections and permissions
- Verify LSM9DS0 is detected: `i2cdetect -y 1`
- Ensure user has access to I2C devices: `sudo usermod -a -G i2c $USER`

### No Calibration
- Run `./calibrate_mag` to create initial calibration
- Check that `configs/config.yaml` exists and has magnetometer calibration data
- Verify config file format matches expected YAML structure

### Poor Calibration Quality
- Re-run calibration in magnetically clean environment
- Ensure all 6 calibration movements were performed correctly
- Check analysis plots for ellipsoid shape distortion
- Consider environmental magnetic interference

## Configuration

## Configuration

Magnetometer calibration parameters in `configs/config.yaml`:

```yaml
calibration:
  mag_bias:  [0.123, -0.045, 0.234]      # Hard iron offset
  mag_matrix: [1.02, 0.01, 0.00,         # Soft iron correction matrix
               0.01, 0.98, 0.00,         # (3x3, row-major order)
               0.00, 0.00, 1.01]
```

## Files Created

- `mag_cal_data_*.csv` - Magnetometer calibration data with timestamps

## Integration

The sensor monitor can be integrated into larger systems:

```cpp
#include "lsm9ds0_device.hpp"
#include "config_loader.hpp"

// Load calibration
std::array<float, 3> mag_bias;
std::array<float, 9> mag_matrix;
config_loader::load_mag_calibration("config.yaml", mag_bias, mag_matrix);
lsm9ds0_device::load_mag_calibration(mag_bias, mag_matrix);

// Read calibrated data
float mx_cal, my_cal, mz_cal;
lsm9ds0_device::read_mag_calibrated(mx_cal, my_cal, mz_cal);
```