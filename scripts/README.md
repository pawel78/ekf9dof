# Scripts

This folder contains analysis and visualization scripts for the ekf9dof project.

## Sensor Data Visualization

Two options are available for visualizing sensor data:

### 1. plot_sensor_data.ipynb (Recommended for Interactive Analysis)

Jupyter notebook for visualizing LSM9DS0 sensor data collected by the main program.

### Features

- **4 Time Series Plots:**
  1. **Gyroscope**: Angular rates (x, y, z) in degrees/second
  2. **Accelerometer**: Acceleration (x, y, z) in g's
  3. **Magnetometer**: Magnetic field (x, y, z) in gauss
  4. **Temperature**: Temperature readings in Celsius

- **Rolling Average Analysis**: Optional visualization with smoothed rolling averages to identify trends
- **Combined View**: All sensors in a single figure for easy comparison
- **Statistics**: Automatic calculation of mean, std, min, max, and range for all sensors

### Requirements

```bash
pip install pandas matplotlib numpy jupyter
```

### Usage

1. Run the main program to collect sensor data (generates `sensor_data_YYYYMMDD_HHMMSS.csv`)
2. Launch Jupyter notebook:
   ```bash
   cd scripts
   jupyter notebook plot_sensor_data.ipynb
   ```
3. The notebook will automatically find the most recent CSV file, or you can specify a file path
4. Run all cells to generate plots and statistics

### 2. plot_sensor_data.py (Standalone Python Script)

Simple Python script for quick visualization without Jupyter.

#### Requirements

```bash
pip install pandas matplotlib numpy
```

#### Usage

```bash
# Plot the most recent sensor data file
python plot_sensor_data.py

# Specify a specific file
python plot_sensor_data.py path/to/sensor_data.csv

# Automatically save plots as PNG files
python plot_sensor_data.py --save-plots

# Search in a specific directory for data files
python plot_sensor_data.py --search-dir /path/to/data
```

The script will:
- Display all 4 plots (gyroscope, accelerometer, magnetometer, temperature)
- Print statistics for each sensor
- Optionally save plots as PNG files (use `--save-plots` flag or interactive prompt)

### Data Format

Both tools expect CSV files with the following columns:
- `timestamp_ms`: Time in milliseconds since start
- `ax_g, ay_g, az_g`: Accelerometer (g's)
- `gx_dps, gy_dps, gz_dps`: Gyroscope (degrees/second)
- `mx_gauss, my_gauss, mz_gauss`: Magnetometer (gauss)
- `temp_c`: Temperature (Celsius)

## Other Scripts

- `plot_attitude.py`: Python script for attitude plotting
- `lsm9ds0_check.sh`: Shell script for checking LSM9DS0 sensor
- `setup_jetson.sh`: Setup script for Jetson devices
