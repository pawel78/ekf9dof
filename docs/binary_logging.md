# LSM9DS0 Binary Data Logging

## Overview

The LSM9DS0 driver now supports high-speed binary logging at the full 200 Hz sampling rate. This feature is designed for capturing raw sensor data for offline analysis, debugging, and validation.

## Features

- **High-speed logging**: Records all sensor data at 200 Hz (5ms intervals)
- **Efficient binary format**: Compact storage, minimal CPU overhead
- **Thread-safe**: Uses mutex protection for file operations
- **Offline conversion tool**: `bin2csv` utility converts logs to CSV format

## Usage

### Enable Logging from Command Line

```bash
# Start with binary logging enabled (default filename: imu_data.bin)
./ekf9dof --log

# Specify custom log filename
./ekf9dof --log my_data.bin

# Combine with debug output
./ekf9dof --log --debug
./ekf9dof -l my_data.bin -d
```

### Enable Logging Programmatically

```cpp
LSM9DS0Driver imu_driver("/dev/i2c-7");
imu_driver.start();

// Enable logging
if (imu_driver.set_data_logging(true, "imu_data.bin")) {
    std::cout << "Logging enabled\n";
}

// ... run for some time ...

// Disable logging
imu_driver.set_data_logging(false);
```

## Binary File Format

### Header (12 bytes)
- **Magic Number** (4 bytes): `0x494D5539` ("IMU9" in ASCII)
- **Version** (4 bytes): `1`
- **Sample Size** (4 bytes): `48` (bytes per sample)

### Data Records (48 bytes each)
Each sample contains:
- **Timestamp** (8 bytes): `uint64_t` nanoseconds since epoch
- **Accel X** (4 bytes): `float` acceleration in g
- **Accel Y** (4 bytes): `float` acceleration in g
- **Accel Z** (4 bytes): `float` acceleration in g
- **Gyro X** (4 bytes): `float` angular velocity in deg/s
- **Gyro Y** (4 bytes): `float` angular velocity in deg/s
- **Gyro Z** (4 bytes): `float` angular velocity in deg/s
- **Mag X** (4 bytes): `float` magnetic field in gauss
- **Mag Y** (4 bytes): `float` magnetic field in gauss
- **Mag Z** (4 bytes): `float` magnetic field in gauss
- **Temperature** (4 bytes): `float` temperature in °C

**Total**: 48 bytes per sample

### File Size Estimation
- **Sample rate**: 200 Hz
- **Bytes per second**: 200 × 48 = 9,600 bytes ≈ 9.4 KB/s
- **1 minute**: ~563 KB
- **10 minutes**: ~5.5 MB
- **1 hour**: ~33 MB

## Converting to CSV

Use the `bin2csv` tool to convert binary logs to human-readable CSV format:

```bash
# Basic usage (output to same filename with .csv extension)
./bin2csv imu_data.bin

# Specify output filename
./bin2csv imu_data.bin output.csv
```

### CSV Output Format

The CSV file contains the following columns:

```
timestamp_ns,timestamp_sec,accel_x_g,accel_y_g,accel_z_g,gyro_x_dps,gyro_y_dps,gyro_z_dps,mag_x_gauss,mag_y_gauss,mag_z_gauss,temperature_c
```

Example output:
```
timestamp_ns,timestamp_sec,accel_x_g,accel_y_g,accel_z_g,gyro_x_dps,gyro_y_dps,gyro_z_dps,mag_x_gauss,mag_y_gauss,mag_z_gauss,temperature_c
1234567890123,1234.567890123,0.001234,-0.002345,1.008765,12.345,-23.456,34.567,0.234567,-0.345678,0.456789,28.50
```

The `bin2csv` tool also provides statistics:
```
Binary file format version: 1
Sample size: 48 bytes
Processed 12000 samples

Statistics:
  Total samples: 12000
  Duration: 60.000 seconds
  Average rate: 200.0 Hz

✓ Conversion complete!
```

## Offline Analysis Workflow

1. **Capture data** on your Jetson or target hardware:
   ```bash
   ./ekf9dof --log flight_data.bin
   # Run for desired duration, then Ctrl+C to stop
   ```

2. **Transfer** the binary file to your analysis machine (optional):
   ```bash
   scp jetson:/path/to/flight_data.bin ./
   ```

3. **Convert** to CSV:
   ```bash
   ./bin2csv flight_data.bin
   ```

4. **Analyze** with your preferred tools:
   - Python (pandas, numpy, matplotlib)
   - MATLAB
   - Excel
   - R
   - Julia

## Performance

- **CPU overhead**: Minimal - binary writes are highly optimized
- **Disk I/O**: ~9.4 KB/s sustained write rate
- **Memory**: Uses mutex-protected buffering for thread-safe writes
- **Thread safety**: Fully thread-safe, can be toggled at runtime

## Python Analysis Example

```python
import pandas as pd
import matplotlib.pyplot as plt

# Load CSV data
df = pd.read_csv('imu_data.csv')

# Plot accelerometer data
plt.figure(figsize=(12, 8))
plt.subplot(3, 1, 1)
plt.plot(df['timestamp_sec'], df['accel_x_g'], label='X')
plt.plot(df['timestamp_sec'], df['accel_y_g'], label='Y')
plt.plot(df['timestamp_sec'], df['accel_z_g'], label='Z')
plt.ylabel('Acceleration (g)')
plt.legend()
plt.grid(True)

# Plot gyroscope data
plt.subplot(3, 1, 2)
plt.plot(df['timestamp_sec'], df['gyro_x_dps'], label='X')
plt.plot(df['timestamp_sec'], df['gyro_y_dps'], label='Y')
plt.plot(df['timestamp_sec'], df['gyro_z_dps'], label='Z')
plt.ylabel('Angular Velocity (deg/s)')
plt.legend()
plt.grid(True)

# Plot magnetometer data
plt.subplot(3, 1, 3)
plt.plot(df['timestamp_sec'], df['mag_x_gauss'], label='X')
plt.plot(df['timestamp_sec'], df['mag_y_gauss'], label='Y')
plt.plot(df['timestamp_sec'], df['mag_z_gauss'], label='Z')
plt.xlabel('Time (seconds)')
plt.ylabel('Magnetic Field (gauss)')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.savefig('imu_analysis.png', dpi=300)
plt.show()
```

## Troubleshooting

### File won't open
- Check file permissions
- Ensure sufficient disk space
- Verify the path is writable

### High CPU usage
- Binary logging is optimized for performance
- If CPU is an issue, consider reducing sample rate in driver code

### File corruption
- Always stop logging gracefully (Ctrl+C or proper shutdown)
- The file header should start with magic number `0x494D5539`
- Use `hexdump -C imu_data.bin | head` to verify

### bin2csv errors
- Verify the input file has the correct magic number
- Check that file is not truncated (should be 12 + N×48 bytes)
- Ensure you have read permissions on the input file
