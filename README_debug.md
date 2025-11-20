# LSM9DS0 Driver Debug Output Feature

## Overview
The LSM9DS0 driver now supports optional debug output that displays sensor data with timestamps at a slower rate (500ms intervals) than the channel publishing rate (200 Hz).

## Usage

### Enable Debug Output at Runtime

```bash
# Run with debug output enabled
./ekf9dof --debug
# or
./ekf9dof -d
```

### Enable Debug Output Programmatically

```cpp
LSM9DS0Driver imu_driver("/dev/i2c-7");
imu_driver.start();

// Enable debug output
imu_driver.set_debug_output(true);

// ... do work ...

// Disable debug output
imu_driver.set_debug_output(false);
```

## Debug Output Format

```
[timestamp] A(ax,ay,az) G(gx,gy,gz) M(mx,my,mz) T(temp°C)
```

Example:
```
[1234.567s] A( 0.123, -0.045,  1.008) G( 12.34, -45.67,  89.01) M( 0.234, -0.567,  0.890) T( 28.5°C)
```

Where:
- **timestamp**: Time in seconds with 3 decimal places (nanosecond resolution)
- **A(ax,ay,az)**: Accelerometer data in g (gravitational units)
- **G(gx,gy,gz)**: Gyroscope data in degrees per second
- **M(mx,my,mz)**: Magnetometer data in gauss
- **T(temp)**: Temperature in degrees Celsius

## Performance

- **Default**: Debug output is disabled for optimal performance
- **When enabled**: Display rate is 500ms (2 Hz), much slower than the 200 Hz sensor reading rate
- **Thread-safe**: Uses atomic flag, can be toggled at runtime without race conditions
- **Non-blocking**: Does not interfere with channel publishing or sensor reading

## Implementation Details

- Debug output displays every 500ms using time-based checking
- Sensor data continues publishing to channels at 200 Hz regardless of debug state
- Atomic flag ensures thread-safe enable/disable operations
- Compact one-line format for easy visual scanning
