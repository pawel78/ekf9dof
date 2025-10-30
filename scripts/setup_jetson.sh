#!/bin/bash

# Exit on any error
set -e

echo "Setting up dependencies for ekf9dof on Jetson Orin Nano..."

# Update package lists
echo "Updating package lists..."
apt-get update

# Install basic development tools
echo "Installing basic development tools..."
apt-get install -y \
    build-essential \
    cmake \
    git \
    pkg-config \
    i2c-tools

# Create build directory
echo "Creating build directory..."
mkdir -p /root/ekf9dof/build

echo "Setup complete!"
echo "To build and run the project (as root):"
echo "1. Copy your project files to /root/ekf9dof/"
echo "2. cd /root/ekf9dof/build"
echo "3. cmake .."
echo "4. make"
echo "5. ./ekf9dof"