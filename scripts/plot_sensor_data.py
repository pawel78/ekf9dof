#!/usr/bin/env python3
"""
LSM9DS0 Sensor Data Visualization Script

This script provides visualization tools for the 9-DOF sensor data collected 
from the LSM9DS0. It generates 4 time series plots for gyroscope, accelerometer,
magnetometer, and temperature data.

Usage:
    python plot_sensor_data.py <path_to_csv_file> [--save-plots]
    
    Or to use the most recent sensor data file:
    python plot_sensor_data.py [--save-plots]
    
Options:
    --save-plots    Save plots as PNG files without prompting
"""

import sys
import os
import glob
import argparse
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np


def find_latest_sensor_file(search_dir='.'):
    """Find the most recent sensor data CSV file.
    
    Args:
        search_dir: Directory to search for sensor data files (default: current directory)
    """
    # Try parent directory first (common when running from scripts/)
    sensor_files = glob.glob(os.path.join(search_dir, '..', 'sensor_data_*.csv'))
    if not sensor_files:
        # Try current directory
        sensor_files = glob.glob(os.path.join(search_dir, 'sensor_data_*.csv'))
    
    if sensor_files:
        return max(sensor_files, key=os.path.getctime)
    return None


def load_sensor_data(filename):
    """Load sensor data from CSV file."""
    try:
        df = pd.read_csv(filename)
        # Convert timestamp from milliseconds to seconds
        df['time_s'] = df['timestamp_ms'] / 1000.0
        
        print(f"Data loaded successfully from: {filename}")
        print(f"Total samples: {len(df)}")
        print(f"Duration: {df['time_s'].max():.2f} seconds")
        print(f"Sampling rate: ~{len(df) / df['time_s'].max():.0f} Hz\n")
        
        return df
    except FileNotFoundError:
        print(f"Error: File '{filename}' not found.")
        sys.exit(1)
    except Exception as e:
        print(f"Error loading data: {e}")
        sys.exit(1)


def plot_gyroscope(df, save_fig=False):
    """Plot gyroscope data."""
    fig, ax = plt.subplots(figsize=(14, 6))
    
    ax.plot(df['time_s'], df['gx_dps'], label='Gyro X', alpha=0.7, linewidth=1)
    ax.plot(df['time_s'], df['gy_dps'], label='Gyro Y', alpha=0.7, linewidth=1)
    ax.plot(df['time_s'], df['gz_dps'], label='Gyro Z', alpha=0.7, linewidth=1)
    
    ax.set_xlabel('Time (seconds)', fontsize=12)
    ax.set_ylabel('Angular Rate (degrees/second)', fontsize=12)
    ax.set_title('Gyroscope Data: Angular Rates vs Time', fontsize=14, fontweight='bold')
    ax.legend(loc='upper right', fontsize=11)
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    if save_fig:
        plt.savefig('gyroscope_plot.png', dpi=150)
        print("Saved gyroscope_plot.png")
    
    # Print statistics
    print("Gyroscope Statistics:")
    print(f"  X-axis: mean={df['gx_dps'].mean():.2f}, std={df['gx_dps'].std():.2f}, "
          f"range=[{df['gx_dps'].min():.2f}, {df['gx_dps'].max():.2f}]")
    print(f"  Y-axis: mean={df['gy_dps'].mean():.2f}, std={df['gy_dps'].std():.2f}, "
          f"range=[{df['gy_dps'].min():.2f}, {df['gy_dps'].max():.2f}]")
    print(f"  Z-axis: mean={df['gz_dps'].mean():.2f}, std={df['gz_dps'].std():.2f}, "
          f"range=[{df['gz_dps'].min():.2f}, {df['gz_dps'].max():.2f}]\n")


def plot_accelerometer(df, save_fig=False):
    """Plot accelerometer data."""
    fig, ax = plt.subplots(figsize=(14, 6))
    
    ax.plot(df['time_s'], df['ax_g'], label='Accel X', alpha=0.7, linewidth=1)
    ax.plot(df['time_s'], df['ay_g'], label='Accel Y', alpha=0.7, linewidth=1)
    ax.plot(df['time_s'], df['az_g'], label='Accel Z', alpha=0.7, linewidth=1)
    
    # Add reference lines at ±1g
    ax.axhline(y=1.0, color='gray', linestyle='--', alpha=0.5, label='1g reference')
    ax.axhline(y=-1.0, color='gray', linestyle='--', alpha=0.5)
    
    ax.set_xlabel('Time (seconds)', fontsize=12)
    ax.set_ylabel('Acceleration (g)', fontsize=12)
    ax.set_title('Accelerometer Data: Acceleration vs Time', fontsize=14, fontweight='bold')
    ax.legend(loc='upper right', fontsize=11)
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    if save_fig:
        plt.savefig('accelerometer_plot.png', dpi=150)
        print("Saved accelerometer_plot.png")
    
    # Print statistics
    print("Accelerometer Statistics:")
    print(f"  X-axis: mean={df['ax_g'].mean():.3f}, std={df['ax_g'].std():.3f}, "
          f"range=[{df['ax_g'].min():.3f}, {df['ax_g'].max():.3f}]")
    print(f"  Y-axis: mean={df['ay_g'].mean():.3f}, std={df['ay_g'].std():.3f}, "
          f"range=[{df['ay_g'].min():.3f}, {df['ay_g'].max():.3f}]")
    print(f"  Z-axis: mean={df['az_g'].mean():.3f}, std={df['az_g'].std():.3f}, "
          f"range=[{df['az_g'].min():.3f}, {df['az_g'].max():.3f}]\n")


def plot_magnetometer(df, save_fig=False):
    """Plot magnetometer data."""
    fig, ax = plt.subplots(figsize=(14, 6))
    
    ax.plot(df['time_s'], df['mx_gauss'], label='Mag X', alpha=0.7, linewidth=1)
    ax.plot(df['time_s'], df['my_gauss'], label='Mag Y', alpha=0.7, linewidth=1)
    ax.plot(df['time_s'], df['mz_gauss'], label='Mag Z', alpha=0.7, linewidth=1)
    
    ax.set_xlabel('Time (seconds)', fontsize=12)
    ax.set_ylabel('Magnetic Field (gauss)', fontsize=12)
    ax.set_title('Magnetometer Data: Magnetic Field vs Time', fontsize=14, fontweight='bold')
    ax.legend(loc='upper right', fontsize=11)
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    if save_fig:
        plt.savefig('magnetometer_plot.png', dpi=150)
        print("Saved magnetometer_plot.png")
    
    # Calculate and print statistics
    print("Magnetometer Statistics:")
    print(f"  X-axis: mean={df['mx_gauss'].mean():.3f}, std={df['mx_gauss'].std():.3f}, "
          f"range=[{df['mx_gauss'].min():.3f}, {df['mx_gauss'].max():.3f}]")
    print(f"  Y-axis: mean={df['my_gauss'].mean():.3f}, std={df['my_gauss'].std():.3f}, "
          f"range=[{df['my_gauss'].min():.3f}, {df['my_gauss'].max():.3f}]")
    print(f"  Z-axis: mean={df['mz_gauss'].mean():.3f}, std={df['mz_gauss'].std():.3f}, "
          f"range=[{df['mz_gauss'].min():.3f}, {df['mz_gauss'].max():.3f}]")
    
    # Calculate total magnetic field magnitude
    mag_magnitude = np.sqrt(df['mx_gauss']**2 + df['my_gauss']**2 + df['mz_gauss']**2)
    print(f"  Total magnitude: mean={mag_magnitude.mean():.3f}, std={mag_magnitude.std():.3f}\n")


def plot_temperature(df, save_fig=False):
    """Plot temperature data."""
    fig, ax = plt.subplots(figsize=(14, 6))
    
    ax.plot(df['time_s'], df['temp_c'], alpha=0.8, linewidth=1.5, color='red')
    
    ax.set_xlabel('Time (seconds)', fontsize=12)
    ax.set_ylabel('Temperature (°C)', fontsize=12)
    ax.set_title('Temperature Sensor Data: Temperature vs Time', fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    if save_fig:
        plt.savefig('temperature_plot.png', dpi=150)
        print("Saved temperature_plot.png")
    
    # Print statistics
    print("Temperature Statistics:")
    print(f"  Mean: {df['temp_c'].mean():.2f}°C")
    print(f"  Std:  {df['temp_c'].std():.2f}°C")
    print(f"  Min:  {df['temp_c'].min():.2f}°C")
    print(f"  Max:  {df['temp_c'].max():.2f}°C")
    print(f"  Range: {df['temp_c'].max() - df['temp_c'].min():.2f}°C\n")


def plot_combined(df, save_fig=False):
    """Plot all sensors in a combined view."""
    fig, axes = plt.subplots(4, 1, figsize=(14, 12))
    
    # Gyroscope
    axes[0].plot(df['time_s'], df['gx_dps'], label='X', alpha=0.7, linewidth=1)
    axes[0].plot(df['time_s'], df['gy_dps'], label='Y', alpha=0.7, linewidth=1)
    axes[0].plot(df['time_s'], df['gz_dps'], label='Z', alpha=0.7, linewidth=1)
    axes[0].set_ylabel('Gyro (°/s)', fontsize=10)
    axes[0].set_title('Gyroscope', fontsize=11, fontweight='bold')
    axes[0].legend(loc='upper right', fontsize=9)
    axes[0].grid(True, alpha=0.3)
    
    # Accelerometer
    axes[1].plot(df['time_s'], df['ax_g'], label='X', alpha=0.7, linewidth=1)
    axes[1].plot(df['time_s'], df['ay_g'], label='Y', alpha=0.7, linewidth=1)
    axes[1].plot(df['time_s'], df['az_g'], label='Z', alpha=0.7, linewidth=1)
    axes[1].axhline(y=1.0, color='gray', linestyle='--', alpha=0.3)
    axes[1].axhline(y=-1.0, color='gray', linestyle='--', alpha=0.3)
    axes[1].set_ylabel('Accel (g)', fontsize=10)
    axes[1].set_title('Accelerometer', fontsize=11, fontweight='bold')
    axes[1].legend(loc='upper right', fontsize=9)
    axes[1].grid(True, alpha=0.3)
    
    # Magnetometer
    axes[2].plot(df['time_s'], df['mx_gauss'], label='X', alpha=0.7, linewidth=1)
    axes[2].plot(df['time_s'], df['my_gauss'], label='Y', alpha=0.7, linewidth=1)
    axes[2].plot(df['time_s'], df['mz_gauss'], label='Z', alpha=0.7, linewidth=1)
    axes[2].set_ylabel('Mag (gauss)', fontsize=10)
    axes[2].set_title('Magnetometer', fontsize=11, fontweight='bold')
    axes[2].legend(loc='upper right', fontsize=9)
    axes[2].grid(True, alpha=0.3)
    
    # Temperature
    axes[3].plot(df['time_s'], df['temp_c'], alpha=0.8, linewidth=1.5, color='red')
    axes[3].set_ylabel('Temp (°C)', fontsize=10)
    axes[3].set_xlabel('Time (seconds)', fontsize=11)
    axes[3].set_title('Temperature', fontsize=11, fontweight='bold')
    axes[3].grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    if save_fig:
        plt.savefig('combined_plot.png', dpi=150)
        print("Saved combined_plot.png")


def main():
    """Main function."""
    # Parse command-line arguments
    parser = argparse.ArgumentParser(
        description='Visualize LSM9DS0 sensor data from CSV files',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        'filename',
        nargs='?',
        help='Path to sensor data CSV file (default: auto-detect most recent file)'
    )
    parser.add_argument(
        '--save-plots',
        action='store_true',
        help='Save plots as PNG files without prompting'
    )
    parser.add_argument(
        '--search-dir',
        default='.',
        help='Directory to search for sensor data files (default: current directory)'
    )
    
    args = parser.parse_args()
    
    # Configure matplotlib
    plt.style.use('seaborn-v0_8-darkgrid')
    
    # Determine which file to use
    if args.filename:
        filename = args.filename
    else:
        filename = find_latest_sensor_file(args.search_dir)
        if filename is None:
            print("Error: No sensor data CSV file found.")
            print(f"Searched in: {args.search_dir} and parent directory")
            print("Usage: python plot_sensor_data.py <path_to_csv_file>")
            sys.exit(1)
    
    # Load data
    df = load_sensor_data(filename)
    
    # Determine if we should save figures
    if args.save_plots:
        save_fig = True
    else:
        # Ask user if they want to save figures (only in interactive mode)
        try:
            response = input("\nSave plots as PNG files? (y/n, default=n): ").strip().lower()
            save_fig = response == 'y'
        except (EOFError, KeyboardInterrupt):
            save_fig = False
            print()
    
    # Generate all plots
    print("\nGenerating plots...\n")
    plot_gyroscope(df, save_fig)
    plot_accelerometer(df, save_fig)
    plot_magnetometer(df, save_fig)
    plot_temperature(df, save_fig)
    plot_combined(df, save_fig)
    
    # Show all plots
    print("Displaying plots. Close all plot windows to exit.")
    plt.show()


if __name__ == "__main__":
    main()
