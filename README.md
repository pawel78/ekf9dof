# EKF9DOF - Extended Kalman Filter for 9-DOF IMU

A high-performance C++ implementation of an Extended Kalman Filter for 9-axis inertial measurement units, specifically designed for the LSM9DS0 sensor.

## Features

- **9-DOF Sensor Fusion**: Combines gyroscope, accelerometer, and magnetometer data
- **Extended Kalman Filter**: State estimation with optimal sensor fusion
- **LSM9DS0 Support**: Direct I2C interface with the LSM9DS0 IMU
- **Magnetometer Calibration**: Built-in calibration tool for hard and soft iron compensation
- **High-Speed Logging**: 200 Hz sensor data collection
- **Cross-Platform**: Designed for embedded Linux systems (Jetson Nano, Raspberry Pi, etc.)

## Quick Start

### Build

```bash
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

**Note on Cross-Platform Support:** While this project is designed for embedded Linux systems with I2C hardware (Jetson Nano, Raspberry Pi, etc.), the code can be compiled on macOS for compile-time error checking and development. On non-Linux platforms, I2C hardware access will not be available, but building will help catch syntax and compilation errors before deploying to target hardware.

### Run Sensor Data Collection

```bash
./ekf9dof
```

This will:
- Initialize the LSM9DS0 sensor
- Load magnetometer calibration (if available)
- Collect sensor data at 200 Hz
- Save timestamped CSV file with all sensor readings

### Calibrate Magnetometer (Recommended)

For accurate magnetic heading, calibrate the magnetometer before first use:

```bash
./calibrate_mag
```

Follow the interactive prompts to move the sensor through various orientations. See [CALIBRATION_QUICK_START.md](CALIBRATION_QUICK_START.md) for details.

## Project Structure

```
ekf9dof/
├── include/              # Header files
│   ├── ekf9dof/         # Core EKF library headers
│   ├── lsm9ds0*.hpp     # LSM9DS0 sensor interfaces
│   ├── mag_calibration.hpp
│   └── config_loader.hpp
├── src/                 # Source files
│   ├── main.cpp         # Main sensor logger
│   ├── calibrate_mag.cpp # Calibration tool
│   ├── lsm9ds0.cpp      # Sensor I2C driver
│   ├── mag_calibration.cpp
│   └── config_loader.cpp
├── tests/               # Unit tests
├── examples/            # Example programs
├── configs/             # Configuration files
│   └── config.yaml      # Sensor calibration & parameters
├── scripts/             # Data analysis scripts
└── docs/                # Documentation
```

## Documentation

- [Channel Architecture](docs/CHANNEL_ARCHITECTURE.md) - Thread-safe communication design
- [Cross-Platform Build Support](docs/CROSS_PLATFORM_BUILD.md) - Building on macOS and other platforms
- [Magnetometer Calibration Guide](docs/MAGNETOMETER_CALIBRATION.md) - Comprehensive calibration documentation
- [Quick Start Guide](CALIBRATION_QUICK_START.md) - 5-minute calibration setup
- [Scripts README](scripts/README.md) - Data visualization tools

## Architecture

### Channel-Based Communication

The system uses a **simple channel-based architecture** for thread-safe data flow:

```cpp
// Consumer code only needs one include
#include "imu/messages/imu_data.hpp"

void my_consumer(imu::RawGyroChannel& gyro_channel) {
    imu::messages::raw_gyro_msg_t msg;
    if (gyro_channel.try_receive(msg)) {
        // Use gyro data - no driver knowledge needed!
    }
}
```

**Key Benefits:**
- Consumers only depend on message/channel types (in `imu` namespace)
- No driver includes in consumer code
- Easy to swap sensors - change one line in main()
- Thread-safe producer-consumer pattern

See [CHANNEL_ARCHITECTURE.md](docs/CHANNEL_ARCHITECTURE.md) for details.

## Hardware Requirements

- **Sensor**: LSM9DS0 9-axis IMU (or compatible)
- **Interface**: I2C connection
- **Platform**: Linux with I2C support (tested on Jetson Nano)
- **Permissions**: I2C device access (may require sudo)

## Configuration

Edit `configs/config.yaml` to adjust:
- Sensor I2C addresses
- Output data rates (ODR)
- Measurement ranges
- Calibration values
- EKF parameters

## Testing

Run the test suite:

```bash
cd build
ctest --output-on-failure
```

Current test coverage:
- Sensor class unit tests
- Magnetometer calibration tests
- Config loader tests

## Data Analysis

Python scripts are provided for visualizing collected sensor data:

```bash
cd scripts
python plot_sensor_data.py
```

See [scripts/README.md](scripts/README.md) for details on:
- Time series plotting
- Rolling averages
- Statistical analysis
- Jupyter notebook analysis

## Magnetometer Calibration

The magnetometer calibration tool compensates for:

### Hard Iron Effects
Constant offset from permanent magnets or ferromagnetic materials near the sensor.

### Soft Iron Effects  
Field distortion from materials that scale and couple magnetic field measurements.

**Benefits of calibration:**
- Accurate magnetic heading
- Improved orientation estimates
- Reduced drift in EKF
- Better sensor fusion results

## Development

### Building Tests

```bash
cd build
cmake ..
make test_sensor test_mag_calibration test_config_loader
```

### Adding Calibration Values Manually

If automatic config update fails, edit `configs/config.yaml`:

```yaml
calibration:
  mag_bias:  [x, y, z]
  mag_matrix: [m11,m12,m13,
               m21,m22,m23,
               m31,m32,m33]
```

## Contributing

Contributions are welcome! Areas of interest:
- Additional sensor support
- Enhanced EKF implementations
- Real-time visualization
- ROS integration
- Performance optimizations

## License

[Add your license information here]

## References

- LSM9DS0 Datasheet: STMicroelectronics
- Extended Kalman Filter theory
- Magnetometer calibration algorithms

## Troubleshooting

### I2C Permission Denied
```bash
sudo usermod -a -G i2c $USER
# Then log out and back in
```

### Sensor Not Found
- Check I2C wiring
- Verify I2C address in config.yaml
- Run `i2cdetect -y 1` to scan for devices

### Calibration Issues
- Ensure clean magnetic environment
- Move slowly through all orientations
- See [Calibration Guide](docs/MAGNETOMETER_CALIBRATION.md)

## Support

For issues, questions, or contributions, please open an issue on GitHub.
