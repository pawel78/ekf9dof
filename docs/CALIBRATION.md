# Accelerometer Calibration Guide

## Overview

The accelerometer bias calibration utility helps determine and correct systematic errors (biases) in the accelerometer measurements. These biases are assumed to be constant from power-on to power-off.

## Prerequisites

- LSM9DS0 IMU sensor properly connected
- A flat, level surface
- The ability to orient the IMU in 6 different positions

## Running the Calibration

1. Build the calibration tool:
   ```bash
   mkdir -p build
   cd build
   cmake ..
   make calibrate_accel
   ```

2. Run the calibration utility:
   ```bash
   ./calibrate_accel
   ```

3. Follow the on-screen prompts to place the IMU in each of the 6 orientations:
   - **+Z UP (Normal)**: IMU flat with Z-axis pointing up
   - **-Z UP (Upside Down)**: IMU flipped upside down with Z-axis pointing down
   - **+X UP (Right Side)**: IMU on its right side with X-axis pointing up
   - **-X UP (Left Side)**: IMU on its left side with X-axis pointing down
   - **+Y UP (Front Up)**: IMU on its back with Y-axis pointing up
   - **-Y UP (Back Up)**: IMU on its front with Y-axis pointing down

4. For each orientation:
   - Position the IMU as instructed
   - Press ENTER when the IMU is stable
   - Wait while samples are collected (about 1 second per orientation)

5. The tool will compute the optimal bias values and automatically update the `configs/config.yaml` file.

## How It Works

The calibration process:

1. **Data Collection**: For each orientation, collects 100 accelerometer samples at 100 Hz
2. **Bias Computation**: Uses least-squares method to compute bias for each axis:
   - In each orientation, one axis should measure ±1g (gravity) and the others should measure 0g
   - The bias is the average deviation from these expected values across all orientations
3. **Configuration Update**: Automatically updates the `accel_bias` field in `configs/config.yaml`

## Tips for Best Results

- Use a stable, level surface (e.g., table, desk)
- Ensure the IMU is completely stationary during sample collection
- Keep consistent orientation alignment across all measurements
- Perform calibration in the same environment where the IMU will be used

## Bias Values

After calibration, the computed bias values are stored in `configs/config.yaml`:

```yaml
calibration:
  accel_bias: [x_bias, y_bias, z_bias]
```

These values are in units of 'g' (gravitational acceleration) and represent the systematic error in each axis that should be subtracted from raw measurements.

## When to Recalibrate

Consider recalibrating when:
- The IMU is moved to a significantly different location
- Temperature conditions change dramatically
- The IMU mounting or orientation changes
- Accuracy degrades over extended periods (months to years)
