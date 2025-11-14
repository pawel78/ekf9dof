# Magnetometer Calibration - Quick Start Guide

This guide will get you up and running with magnetometer calibration in under 5 minutes.

## What is Magnetometer Calibration?

Magnetometer calibration compensates for:
- **Hard Iron Effects**: Constant magnetic interference from nearby permanent magnets
- **Soft Iron Effects**: Magnetic field distortion from nearby materials

Without calibration, your magnetometer readings may be offset or distorted, leading to inaccurate heading/orientation estimates.

## Quick Start (3 Steps)

### 1. Build the Calibration Tool

```bash
cd ekf9dof
mkdir -p build && cd build
cmake ..
make calibrate_mag
```

### 2. Run Calibration

```bash
./calibrate_mag
```

The tool will prompt you through 6 movements:
1. **Hold still** - Press Enter, wait 2 seconds
2. **Roll** (X-axis) - Press Enter, rotate slowly for 10 seconds
3. **Pitch** (Y-axis) - Press Enter, tilt forward/back for 10 seconds
4. **Yaw** (Z-axis) - Press Enter, spin like a compass for 10 seconds
5. **Tumble** - Press Enter, random orientations for 10 seconds
6. **Figure-8** - Press Enter, draw figure-8 for 10 seconds

### 3. Use Calibrated Data

The calibration values are automatically saved to `configs/config.yaml`. Your main application will load them on startup:

```bash
./ekf9dof
```

You should see: `✓ Magnetometer calibration loaded from config`

## Tips for Best Results

- Perform calibration in a magnetically clean environment (away from motors, transformers, large metal objects)
- Move **slowly and smoothly** - no sudden jerks
- Try to cover **all possible orientations** during each movement
- If unsure, repeat the calibration

## Troubleshooting

**Can't access sensor?**
- Check I2C connections
- May need `sudo ./calibrate_mag` for I2C permissions

**Calibration seems wrong?**
- Ensure you moved through all orientations
- Try again in a cleaner magnetic environment
- Move more slowly during data collection

**Config not updating?**
- Check file permissions on `configs/config.yaml`
- Manually copy the displayed values if needed

## What Next?

For detailed information, see [docs/MAGNETOMETER_CALIBRATION.md](docs/MAGNETOMETER_CALIBRATION.md)

## Example Session

```
$ ./calibrate_mag

========================================
  Magnetometer Calibration Utility
========================================

[1/6] Position 1: Hold sensor in any starting orientation
Press Enter when ready...
  Collecting 50 samples.......... Done!

[2/6] Slowly rotate sensor around X-axis (roll) for ~10 seconds
Press Enter when ready...
  Collecting 100 samples.................... Done!

... (continues for all 6 positions) ...

========================================
  Calculating Calibration Parameters
========================================

Hard iron offset (bias): [0.0234, -0.0156, 0.0412]
Soft iron matrix:
  [1.0234, 0.0000, 0.0000]
  [0.0000, 0.9876, 0.0000]
  [0.0000, 0.0000, 1.0123]

✓ Calibration values saved to: configs/config.yaml

========================================
  Calibration Complete!
========================================
```

That's it! Your magnetometer is now calibrated and ready to use.
