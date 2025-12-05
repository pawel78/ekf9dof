# EKF9DOF Data Logger

## Overview

The data logger subscribes to preprocessor output channels (calibrated IMU data) and records timestamped binary data to disk for offline analysis and debugging.

## Features

- **Multi-channel logging**: Subscribes to gyro, accel, mag, and temp channels simultaneously
- **Timestamped filenames**: Automatically generates unique filenames with timestamp (YYYYMMDD_HHMMSS_mmm)
- **Binary format**: Compact, efficient storage with precise timestamps
- **Non-blocking**: Uses non-blocking receives to handle multiple channels efficiently
- **Statistics tracking**: Reports message count, bytes written, and data rate
- **Selective logging**: Enable/disable individual sensor types
- **Clean shutdown**: Graceful handling of Ctrl+C with data flush

## Usage

### Basic Usage

```bash
# Log with auto-generated timestamped filename to data/logs/
# Creates: data/logs/imu_log_YYYYMMDD_HHMMSS.bin
./imu_logger

# Specify custom output file (disables auto-timestamp)
./imu_logger data/my_custom_log.bin

# Use auto-timestamp but different directory
./imu_logger "" custom_logs/
```

### Running with the System

1. Start the IMU driver (publishes raw data):
```bash
./ekf9dof
```

2. In another terminal, start the logger with timestamped filename:
```bash
./imu_logger
# Output: data/logs/imu_log_20251204_143022.bin
```

3. Press Ctrl+C to stop logging when done

### Converting to CSV

Use the provided Python tool to convert binary logs to CSV format:

```bash
# Convert to CSV files (creates separate file per sensor)
# Automatically preserves timestamp from input filename
python3 tools/log2csv.py data/logs/imu_log_20251204_143022.bin

# This creates:
#   imu_log_20251204_143022_gyro.csv
#   imu_log_20251204_143022_accel.csv
#   imu_log_20251204_143022_mag.csv
#   imu_log_20251204_143022_temp.csv

# Specify custom output prefix
python3 tools/log2csv.py data/flight_test_001.bin output/test_001
```

This creates:
- `output/test_001_gyro.csv` - Calibrated gyroscope data (rad/s)
- `output/test_001_accel.csv` - Calibrated accelerometer data (g)
- `output/test_001_mag.csv` - Calibrated magnetometer data (gauss)
- `output/test_001_temp.csv` - Temperature data (°C)

## Filename Format

### Auto-generated Filenames

When running without arguments, the logger creates timestamped filenames:

```
Format: imu_log_YYYYMMDD_HHMMSS.bin
Example: imu_log_20251204_143022.bin

Where:
  YYYY - Year (4 digits)
  MM   - Month (01-12)
  DD   - Day (01-31)
  HH   - Hour (00-23)
  MM   - Minute (00-59)
  SS   - Second (00-59)
```

### CSV Output Naming

The CSV converter preserves timestamps from input filenames:

```
Input:  imu_log_20251204_143022.bin
Output: imu_log_20251204_143022_gyro.csv
        imu_log_20251204_143022_accel.csv
        imu_log_20251204_143022_mag.csv
        imu_log_20251204_143022_temp.csv
```

This makes it easy to correlate binary logs with their CSV conversions and maintain chronological ordering.

## Binary File Format

### File Header
```
Bytes 0-14: "EKF9DOF_LOG_V1\0" (null-terminated string)
```

### Record Format
Each record consists of:
- **msg_type** (1 byte, uint8_t): Message type identifier
  - 1 = Gyroscope
  - 2 = Accelerometer
  - 3 = Magnetometer
  - 4 = Temperature
- **timestamp_ns** (8 bytes, uint64_t): Nanosecond timestamp
- **data** (variable): Sensor-specific data

#### Gyroscope Record (21 bytes total)
```
[type:1][timestamp:8][x:4][y:4][z:4]
- x, y, z: float32 (rad/s)
```

#### Accelerometer Record (21 bytes total)
```
[type:1][timestamp:8][x:4][y:4][z:4]
- x, y, z: float32 (g)
```

#### Magnetometer Record (21 bytes total)
```
[type:1][timestamp:8][x:4][y:4][z:4]
- x, y, z: float32 (gauss)
```

#### Temperature Record (13 bytes total)
```
[type:1][timestamp:8][temp:4]
- temp: float32 (°C)
```

## CSV Output Format

Each CSV file has the following columns:

### Gyro/Accel/Mag CSV
```
timestamp_ns, timestamp_s, x, y, z
```

### Temperature CSV
```
timestamp_ns, timestamp_s, temp_c
```

## Architecture

The logger uses the same NNG pub/sub architecture as the rest of the system:

```
┌──────────┐    raw     ┌──────────────┐   proc   ┌────────┐
│  Driver  │────────-──>│ Preprocessor │─────────>│  AHRS  │
└──────────┘            └──────────────┘          └────────┘
                               │
                               │ proc (gyro/accel/mag/temp)
                               ▼
                        ┌─────────────┐
                        │ Data Logger │
                        └─────────────┘
                               │
                               ▼
                          [Binary File]
```

### Subscribed Channels
- `ipc:///tmp/imu_proc_gyro.ipc` - Calibrated gyroscope
- `ipc:///tmp/imu_proc_accel.ipc` - Calibrated accelerometer
- `ipc:///tmp/imu_proc_mag.ipc` - Calibrated magnetometer
- `ipc:///tmp/imu_proc_temp.ipc` - Temperature

## Performance

- **Data rate**: ~200 Hz per sensor (gyro, accel, mag) + ~10 Hz (temp) = ~610 messages/sec
- **Storage rate**: ~12.9 KB/sec (gyro/accel/mag: 21 bytes × 200 Hz × 3, temp: 13 bytes × 10 Hz)
- **1 minute log**: ~775 KB
- **1 hour log**: ~46.4 MB

## Implementation Details

### Thread Model
- Main thread: Setup, signal handling, status updates
- Logging thread: Non-blocking receives from all channels, writes to file

### Error Handling
- Socket creation failures are reported and prevent startup
- File open failures prevent startup
- Receive errors are silently ignored (non-blocking mode)
- Graceful shutdown on signals (SIGINT, SIGTERM)

### Flush Strategy
- Automatic flush every 100 messages (~0.5 seconds at 200 Hz)
- Final flush on shutdown
- Balances data safety with performance

## Future Enhancements

- [ ] Add AHRS output logging (quaternion, Euler angles)
- [ ] Add EKF state logging
- [ ] Compression support (gzip, zstd)
- [ ] Time-based log rotation
- [ ] Real-time plotting mode
- [ ] Statistics per sensor type
- [ ] Configurable flush interval
- [ ] Memory-mapped file I/O for performance
