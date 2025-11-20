# Binary Logging Quick Reference

## Quick Start

### 1. Record Data
```bash
# On Jetson/target hardware
./ekf9dof --log my_flight.bin
# Press Ctrl+C when done
```

### 2. Convert to CSV
```bash
# Converts my_flight.bin to my_flight.csv
./bin2csv my_flight.bin
```

### 3. Analyze
```bash
# Open in your favorite tool
python analyze.py my_flight.csv
# or
open my_flight.csv  # Excel, LibreOffice, etc.
```

## Command Line Options

### ekf9dof
- `--log` or `-l` : Enable binary logging (default: imu_data.bin)
- `--log <filename>` : Enable logging with custom filename
- `--debug` or `-d` : Enable console debug output (500ms rate)

### bin2csv
- `./bin2csv input.bin` : Convert to input.csv
- `./bin2csv input.bin output.csv` : Convert to specific output file

## File Format Summary

### Binary (.bin)
- Compact, efficient storage
- 48 bytes per sample + 12 byte header
- ~9.4 KB/s at 200 Hz
- Not human-readable

### CSV (.csv)
- Human-readable text format
- Can be opened in Excel, MATLAB, Python, etc.
- Larger file size than binary
- Contains timestamp in both ns and seconds

## Example Session

```bash
# Terminal 1: Record data for 30 seconds
./ekf9dof --log test_data.bin
# ... wait 30 seconds ...
# Press Ctrl+C

# Terminal 2: Convert to CSV
./bin2csv test_data.bin
# Output:
# Converting binary log to CSV...
# Input:  test_data.bin
# Output: test_data.csv
# Binary file format version: 1
# Sample size: 48 bytes
# Processed 6000 samples
# Statistics:
#   Total samples: 6000
#   Duration: 30.000 seconds
#   Average rate: 200.0 Hz
# ✓ Conversion complete!

# Terminal 3: Quick preview
head test_data.csv
# timestamp_ns,timestamp_sec,accel_x_g,accel_y_g,accel_z_g,...

# Terminal 4: Analyze in Python
python3
>>> import pandas as pd
>>> df = pd.read_csv('test_data.csv')
>>> df.describe()
>>> df.plot(x='timestamp_sec', y=['accel_x_g', 'accel_y_g', 'accel_z_g'])
```

## Data Columns

| Column | Unit | Description |
|--------|------|-------------|
| `timestamp_ns` | nanoseconds | Raw timestamp |
| `timestamp_sec` | seconds | Converted timestamp (use for plotting) |
| `accel_x_g` | g | X-axis acceleration |
| `accel_y_g` | g | Y-axis acceleration |
| `accel_z_g` | g | Z-axis acceleration |
| `gyro_x_dps` | deg/s | X-axis angular velocity |
| `gyro_y_dps` | deg/s | Y-axis angular velocity |
| `gyro_z_dps` | deg/s | Z-axis angular velocity |
| `mag_x_gauss` | gauss | X-axis magnetic field |
| `mag_y_gauss` | gauss | Y-axis magnetic field |
| `mag_z_gauss` | gauss | Z-axis magnetic field |
| `temperature_c` | °C | IMU temperature |

## Storage Requirements

| Duration | Samples | Binary Size | CSV Size (approx) |
|----------|---------|-------------|-------------------|
| 10 sec   | 2,000   | ~94 KB      | ~350 KB           |
| 1 min    | 12,000  | ~563 KB     | ~2.1 MB           |
| 10 min   | 120,000 | ~5.5 MB     | ~21 MB            |
| 1 hour   | 720,000 | ~33 MB      | ~126 MB           |

## Tips

1. **Storage**: Binary files are ~3.5x smaller than CSV
2. **Transfer**: Transfer binary files, convert locally
3. **Backup**: Keep both .bin and .csv versions
4. **Validation**: Check sample count and rate after conversion
5. **Timestamps**: Use `timestamp_sec` for plotting (easier to read)
