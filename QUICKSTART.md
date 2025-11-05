# Quick Start Guide

Get your stereo VO system running in minutes!

## Prerequisites

- Jetson Orin Nano with Ubuntu 22.04 + L4T
- Arducam Camarray Stereo HAT with two IMX477 cameras
- LSM9DS0 9-DOF IMU on I²C bus 7
- ROS 2 Humble installed

## 1. Setup (5 minutes)

```bash
# Clone repository
cd ~
git clone https://github.com/pawel78/ekf9dof.git
cd ekf9dof

# Install dependencies
./setup_jetson.sh

# Log out and back in for I2C permissions
```

## 2. Build (2-3 minutes)

```bash
# Create workspace
mkdir -p ~/stereo_vo_ws/src
cd ~/stereo_vo_ws/src
ln -s ~/ekf9dof .

# Build
cd ~/stereo_vo_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source
source install/setup.bash
echo "source ~/stereo_vo_ws/install/setup.bash" >> ~/.bashrc
```

## 3. Test Hardware

### Test Cameras

```bash
# Test Argus (Jetson native)
gst-launch-1.0 nvarguscamerasrc sensor-id=0 ! nvoverlaysink

# Or test V4L2
ls /dev/video*
v4l2-ctl --list-devices
```

### Test IMU

```bash
# Check I2C
sudo i2cdetect -y 7
# Should show devices at 0x1D and 0x6B

# Run IMU node
ros2 run lsm9ds0_imu lsm9ds0_node

# In another terminal, test self-test
ros2 service call /imu/self_test std_srvs/srv/Trigger
# Should return: success: True, message: "LSM9DS0 self-test passed (WHO_AM_I: 0xD4, 0x49)"

# Check IMU data
ros2 topic echo /imu/data
```

## 4. Calibrate Cameras (10-15 minutes)

```bash
# Print a 9x6 chessboard (25mm squares)
# https://github.com/opencv/opencv/blob/master/doc/pattern.png

# Launch cameras
ros2 launch stereo_camera_bringup stereo_camera.launch.py

# In another terminal, capture ~20 image pairs by moving the chessboard
# Save to a directory, then run calibration:
cd ~/stereo_vo_ws/src/ekf9dof/stereo_vo/scripts
python3 calibrate_stereo.py \
    --left "calib/left_*.png" \
    --right "calib/right_*.png" \
    --pattern-cols 9 \
    --pattern-rows 6 \
    --square-size 0.025 \
    --output ~/stereo_vo_ws/install/stereo_vo/share/stereo_vo/config/

# Copy calibration files
cp calibration/*.yaml ~/stereo_vo_ws/install/stereo_vo/share/stereo_vo/config/
```

## 5. Run VO System

```bash
# Full system with Argus cameras and IMU
ros2 launch stereo_vo vo_stereo.launch.py \
    camera_driver:=argus \
    use_imu:=true \
    publish_debug_images:=true

# Or with V4L2 cameras (fallback)
ros2 launch stereo_vo vo_stereo.launch.py \
    camera_driver:=v4l2 \
    use_imu:=false
```

## 6. Monitor & Visualize

```bash
# Check topics
ros2 topic list
ros2 topic hz /vo/odom
ros2 topic hz /imu/data

# Echo odometry
ros2 topic echo /vo/odom

# Visualize in RViz
ros2 run rviz2 rviz2
# Add displays for:
# - /vo/odom (Odometry)
# - /tf (TF)
# - /vo/tracks (Image, if debug enabled)
# - /stereo/left/image_raw (Image)
```

## 7. Record Data

```bash
cd ~/stereo_vo_ws/src/ekf9dof/stereo_vo/scripts
./record_bag.sh

# Drive/move the robot around
# Press Ctrl+C to stop

# Bags saved to ~/stereo_vo_bags/
```

## 8. Evaluate (Optional)

```bash
cd ~/stereo_vo_ws/src/ekf9dof/stereo_vo/scripts
python3 eval_kitti_like.py \
    --estimated ~/stereo_vo_bags/stereo_vo_20240101_120000/ \
    --ground-truth ~/ground_truth.csv \
    --output ./evaluation/

# View results
cat ./evaluation/metrics.yaml
xdg-open ./evaluation/trajectory.png
```

## Troubleshooting

### Cameras not detected
```bash
# Check CSI connection
dmesg | grep -i csi

# Try V4L2 instead
ros2 launch stereo_vo vo_stereo.launch.py camera_driver:=v4l2
```

### IMU not responding
```bash
# Check I2C
sudo i2cdetect -y 7

# Check permissions
ls -l /dev/i2c-7
sudo chmod a+rw /dev/i2c-7
```

### Low frame rate
```bash
# Reduce resolution in config
nano ~/stereo_vo_ws/install/stereo_vo/share/stereo_vo/config/params.yaml
# Set: resolution: [640, 480]

# Or reduce features
# Set: max_features: 800
```

### Build errors
```bash
# Install missing dependencies
cd ~/stereo_vo_ws
rosdep install --from-paths src --ignore-src -r -y

# Clean and rebuild
rm -rf build install log
colcon build --symlink-install
```

## Performance Tips

1. **Maximize throughput**: Lower resolution (640×480) runs at 30+ Hz
2. **Better accuracy**: Higher resolution (1280×720) with more features
3. **Enable IMU**: Improves robustness in low-texture or fast motion
4. **Tune parameters**: Adjust `max_features`, `ba_window_size` in params.yaml

## Next Steps

- Fine-tune parameters in `config/params.yaml`
- Calibrate IMU biases for better VIO
- Integrate with your robot's navigation stack
- Add loop closure for SLAM

## Need Help?

- Check main [README.md](README.md) for detailed documentation
- Review troubleshooting section
- Open an issue on GitHub

Happy mapping! 🚀🗺️
