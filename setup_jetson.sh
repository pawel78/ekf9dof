#!/bin/bash
# Setup script for Jetson Orin Nano

set -e

echo "=== Stereo VO Setup for Jetson Orin Nano ==="
echo ""

# Check if running on Jetson
if [ ! -f /etc/nv_tegra_release ]; then
    echo "Warning: This doesn't appear to be a Jetson device"
    echo "Continue anyway? (y/n)"
    read -r response
    if [ "$response" != "y" ]; then
        exit 1
    fi
fi

echo "Installing system dependencies..."
sudo apt update

# ROS 2 dependencies
sudo apt install -y \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-camera-info-manager \
    ros-humble-tf2-ros \
    ros-humble-message-filters \
    ros-humble-image-proc

# Build dependencies
sudo apt install -y \
    libeigen3-dev \
    libopencv-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    python3-opencv \
    python3-yaml \
    python3-numpy

# I2C tools
sudo apt install -y \
    i2c-tools \
    libi2c-dev

# GTSAM (optional but recommended)
echo ""
echo "Install GTSAM for bundle adjustment? (y/n)"
read -r install_gtsam
if [ "$install_gtsam" = "y" ]; then
    sudo apt install -y libgtsam-dev || {
        echo "GTSAM not available via apt, will build without it"
    }
fi

# Enable I2C permissions
echo ""
echo "Setting up I2C permissions..."
sudo usermod -a -G i2c $USER
sudo chmod a+rw /dev/i2c-* || true

echo ""
echo "=== Setup Complete ==="
echo ""
echo "Next steps:"
echo "1. Log out and back in for I2C permissions to take effect"
echo "2. Build the workspace:"
echo "   cd ~/stereo_vo_ws"
echo "   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"
echo "3. Source the workspace:"
echo "   source install/setup.bash"
echo "4. Run camera calibration"
echo "5. Launch the system:"
echo "   ros2 launch stereo_vo vo_stereo.launch.py"
echo ""
