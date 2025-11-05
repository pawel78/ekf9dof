# Magnetometer Calibration Guide

This guide explains how to calibrate the LSM9DS0 magnetometer for optimal performance by compensating for hard iron and soft iron effects.

## Background

### Hard Iron Effects
Hard iron distortion is caused by permanent magnets or ferromagnetic materials near the magnetometer. This creates a constant offset in the measurements, shifting the center of the measurement sphere away from the origin.

**Examples:**
- Permanent magnets in the device
- Magnetized screws or metal components
- Hard disk drives
- Speakers

### Soft Iron Effects
Soft iron distortion is caused by materials that distort the magnetic field, creating scaling and cross-axis coupling in the measurements. This transforms the measurement sphere into an ellipsoid.

**Examples:**
- Iron or steel chassis/enclosure
- PCB traces carrying current
- Batteries and power supplies

## Prerequisites

1. **Hardware:**
   - LSM9DS0 sensor properly connected and accessible via I2C
   - Device must be in an environment free from large magnetic disturbances

2. **Software:**
   - Built `calibrate_mag` executable (automatically built with the project)

## Calibration Process

### Step 1: Build the Calibration Tool

```bash
cd /path/to/ekf9dof
mkdir -p build && cd build
cmake ..
make calibrate_mag
```

### Step 2: Run the Calibration Tool

```bash
# From the build directory
./calibrate_mag

# Or specify a custom config path
./calibrate_mag /path/to/config.yaml
```

### Step 3: Follow the Interactive Prompts

The calibration tool will guide you through 6 positions to collect magnetometer data:

#### Position 1: Initial Baseline (1-2 seconds)
- Hold the sensor in any comfortable starting orientation
- Press Enter when ready
- Keep the sensor steady while samples are collected

#### Position 2: X-axis Rotation (10 seconds)
- Slowly rotate the sensor around the X-axis (roll motion)
- Imagine the sensor is a wheel rolling forward
- Complete at least one full rotation
- Press Enter to begin

#### Position 3: Y-axis Rotation (10 seconds)
- Slowly rotate the sensor around the Y-axis (pitch motion)
- Imagine tipping the sensor forward and backward
- Complete at least one full rotation
- Press Enter to begin

#### Position 4: Z-axis Rotation (10 seconds)
- Slowly rotate the sensor around the Z-axis (yaw motion)
- Imagine spinning the sensor like a compass
- Complete at least one full rotation
- Press Enter to begin

#### Position 5: Random Tumbling (10 seconds)
- Slowly tumble the sensor in various random orientations
- Try to cover as many different angles as possible
- Keep movements smooth and continuous
- Press Enter to begin

#### Position 6: Figure-8 Pattern (10 seconds)
- Move the sensor in a figure-8 or infinity pattern
- This helps capture data at various orientations
- Keep movements smooth and continuous
- Press Enter to begin

### Step 4: Review Calibration Results

After completing all 6 positions, the tool will:

1. Calculate the hard iron offset (bias)
2. Calculate the soft iron correction matrix
3. Display the calibration parameters
4. Automatically update `configs/config.yaml` with the new values

Example output:
```
========================================
  Calculating Calibration Parameters
========================================

Hard iron offset (bias): [0.0234, -0.0156, 0.0412]
Soft iron matrix:
  [1.0234, 0.0000, 0.0000]
  [0.0000, 0.9876, 0.0000]
  [0.0000, 0.0000, 1.0123]

✓ Calibration values saved to: configs/config.yaml
```

## Tips for Best Results

1. **Environment:**
   - Perform calibration away from large metal objects
   - Avoid areas with strong magnetic fields (motors, transformers, etc.)
   - Keep the sensor away from power cables

2. **Movement:**
   - Move slowly and smoothly
   - Avoid sudden jerks or movements
   - Try to cover all possible orientations
   - Multiple rotations are better than one

3. **Orientation Coverage:**
   - Aim to point the sensor in all directions (up, down, north, south, east, west)
   - The more orientations covered, the better the calibration

4. **Validation:**
   - After calibration, collect data and check that:
     - The magnetometer readings form a sphere (not ellipsoid) when plotted
     - The sphere is centered at the origin
     - The radius is approximately the expected local magnetic field strength

## Manual Configuration

If the automatic update fails, you can manually update `configs/config.yaml`:

```yaml
calibration:
  # ... other calibration values ...
  mag_bias:  [0.0234, -0.0156, 0.0412]
  mag_matrix: [1.0234,0.0000,0.0000,
               0.0000,0.9876,0.0000,
               0.0000,0.0000,1.0123]
```

## Using Calibrated Values

The main application automatically loads calibration values from `configs/config.yaml` on startup:

```bash
./ekf9dof
```

You should see:
```
✓ Magnetometer calibration loaded from config
```

If calibration is not found:
```
⚠ Warning: Could not load magnetometer calibration, using defaults
```

## Recalibration

Recalibrate the magnetometer when:

- The sensor is moved to a different physical location in the device
- Nearby magnetic materials are added or removed
- The device chassis or enclosure changes
- Measurements appear distorted or inaccurate
- After significant device modifications

## Troubleshooting

### Issue: Calibration tool cannot access sensor
**Solution:** Check that:
- The sensor is properly connected
- I2C permissions are correct (may need `sudo`)
- No other process is accessing the sensor

### Issue: Calibration values seem incorrect
**Solution:**
- Repeat the calibration in a better environment
- Ensure you covered all orientations during collection
- Move more slowly during data collection

### Issue: Config file not updated
**Solution:**
- Check file permissions on `configs/config.yaml`
- Manually copy the displayed values into the config file
- Verify the config file path is correct

## Technical Details

### Hard Iron Compensation
The hard iron offset is calculated as the midpoint between the minimum and maximum values on each axis:

```
offset[i] = (max[i] + min[i]) / 2
```

### Soft Iron Compensation
The soft iron matrix is a diagonal scaling matrix calculated to normalize the range on each axis:

```
scale[i] = avg_range / range[i]
```

Where `range[i]` is the difference between max and min values on axis i, and `avg_range` is the average range across all three axes.

### Calibration Application
Calibrated values are computed as:

```
calibrated = soft_iron_matrix × (raw - hard_iron_offset)
```

This is applied automatically when using `read_mag_calibrated()`.

## References

- LSM9DS0 Datasheet: STMicroelectronics
- "Calibrating an eCompass in the Presence of Hard and Soft-Iron Interference" - Freescale Semiconductor
- "Magnetometer Calibration" - Robot Operating System (ROS) Wiki
