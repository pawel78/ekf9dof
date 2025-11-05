# Stereo Visual Odometry for ROS 2 Humble on Jetson Orin Nano

Production-ready stereo visual odometry (VO) stack for robotics applications on **Jetson Orin Nano** with **Arducam Camarray stereo HAT** (Sony IMX477) and **LSM9DS0 9-DOF IMU**.

## Features

- **Stereo Camera Support**:
  - Argus/GStreamer (native Jetson, hardware-accelerated)
  - V4L2 fallback for standard USB cameras
  - Synchronized stereo capture with hardware timestamps
  
- **Visual Odometry**:
  - ORB feature detection
  - Pyramidal Lucas-Kanade optical flow tracking
  - SGBM stereo matching with optional CUDA acceleration
  - PnP pose estimation with RANSAC
  - Sliding window bundle adjustment (GTSAM)
  - Optional IMU preintegration for VIO
  
- **IMU Support**:
  - LSM9DS0 9-DOF (gyro/accel/mag) on I²C
  - Configurable sampling rate (up to 200 Hz)
  - Calibration and orientation correction
  - Self-test service

- **ROS 2 Integration**:
  - Standard sensor_msgs (Image, CameraInfo, Imu)
  - Odometry publishing with covariance
  - TF2 transforms (odom → base_link)
  - Debug visualization topics

## Hardware Setup

### Cameras
- **Arducam Camarray Stereo HAT** on CSI0
- Two **Sony IMX477** (12.3MP) modules with synchronized exposure
- Appears as sensor_id 0 (left) and 1 (right) via Argus

### IMU
- **LSM9DS0** 9-DOF on I²C bus 7
- Gyro/Accel address: `0x6B`
- Magnetometer address: `0x1D`

## Installation

### Prerequisites (Jetson)

```bash
# Install ROS 2 Humble (if not already installed)
sudo apt update
sudo apt install ros-humble-desktop

# Install dependencies
sudo apt install \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-camera-info-manager \
    ros-humble-tf2-ros \
    libeigen3-dev \
    libopencv-dev \
    python3-opencv \
    python3-yaml

# Optional: Install GTSAM for bundle adjustment
sudo apt install libgtsam-dev

# Install Jetson multimedia API (for Argus)
# Usually pre-installed on L4T
```

### Build

```bash
# Create workspace
mkdir -p ~/stereo_vo_ws/src
cd ~/stereo_vo_ws/src

# Clone repository
git clone https://github.com/pawel78/ekf9dof.git

# Build packages
cd ~/stereo_vo_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source workspace
source install/setup.bash
```

## Configuration

### Camera Calibration

1. Print a chessboard pattern (9x6 squares, 25mm each)
2. Capture calibration images:

```bash
# Start camera node
ros2 launch stereo_camera_bringup stereo_camera.launch.py

# Save images (manual or script)
ros2 run image_view image_saver --ros-args -r image:=/stereo/left/image_raw -p filename_format:="left_%04d.png"
ros2 run image_view image_saver --ros-args -r image:=/stereo/right/image_raw -p filename_format:="right_%04d.png"
```

3. Run calibration:

```bash
cd ~/stereo_vo_ws/src/ekf9dof/stereo_vo/scripts
python3 calibrate_stereo.py \
    --left "left_*.png" \
    --right "right_*.png" \
    --pattern-cols 9 \
    --pattern-rows 6 \
    --square-size 0.025 \
    --output ~/stereo_vo_ws/install/stereo_vo/share/stereo_vo/config/
```

### IMU Calibration

Edit `~/stereo_vo_ws/install/lsm9ds0_imu/share/lsm9ds0_imu/config/imu.yaml`:

```yaml
/**:
  ros__parameters:
    # Update biases after calibration
    gyro_bias: [0.01, -0.02, 0.005]
    accel_bias: [0.0, 0.0, 0.1]
    mag_bias: [0.0, 0.0, 0.0]
```

## Usage

### Launch Complete System

```bash
# With Argus cameras and IMU
ros2 launch stereo_vo vo_stereo.launch.py \
    camera_driver:=argus \
    use_imu:=true \
    publish_debug_images:=true
```

### Launch Components Separately

```bash
# Terminal 1: Stereo cameras
ros2 launch stereo_camera_bringup stereo_camera.launch.py

# Terminal 2: IMU
ros2 run lsm9ds0_imu lsm9ds0_node --ros-args -p i2c_bus:=7

# Terminal 3: VO
ros2 run stereo_vo vo_node --ros-args --params-file install/stereo_vo/share/stereo_vo/config/params.yaml
```

### Verify System

```bash
# Check IMU
ros2 service call /imu/self_test std_srvs/srv/Trigger

# Monitor topics
ros2 topic hz /stereo/left/image_raw
ros2 topic hz /imu/data
ros2 topic echo /vo/odom

# Visualize in RViz
ros2 run rviz2 rviz2
```

### Record Data

```bash
cd ~/stereo_vo_ws/src/ekf9dof/stereo_vo/scripts
./record_bag.sh
```

## Topics

### Published

- `/stereo/left/image_raw` - Left camera image (sensor_msgs/Image)
- `/stereo/right/image_raw` - Right camera image (sensor_msgs/Image)
- `/stereo/left/camera_info` - Left camera parameters (sensor_msgs/CameraInfo)
- `/stereo/right/camera_info` - Right camera parameters (sensor_msgs/CameraInfo)
- `/imu/data` - IMU data at 100-200 Hz (sensor_msgs/Imu)
- `/imu/mag` - Magnetometer data (sensor_msgs/MagneticField)
- `/vo/odom` - Visual odometry (nav_msgs/Odometry)
- `/vo/quality` - VO quality metric 0-1 (std_msgs/Float32)
- `/vo/tracks` - Debug visualization (sensor_msgs/Image)
- `/tf` - Transforms (odom → base_link)

### Services

- `/imu/self_test` - IMU WHO_AM_I verification (std_srvs/Trigger)

## Parameters

See `stereo_vo/config/params.yaml` for all configurable parameters:

```yaml
# Key parameters
use_imu: false                    # Enable IMU integration
camera_driver: "argus"            # "argus" or "v4l2"
sensor_id_left: 0                 # Argus sensor ID
sensor_id_right: 1
resolution: [1280, 720]
fps: 30

max_features: 1500                # Feature tracking
lk_levels: 3
ransac_reproj_thresh_px: 2.0      # Pose estimation
ba_window_size: 7                 # Bundle adjustment
```

## Performance

### Expected Performance on Jetson Orin Nano

- **640×480 @ 30 FPS**: ≥30 Hz odometry (real-time)
- **1280×720 @ 30 FPS**: ≥20 Hz odometry
- **Feature count**: 500-1500 tracked features
- **Latency**: <50ms end-to-end

### Accuracy (Indoor, 20m trajectory)

- **ATE** (Absolute Trajectory Error): <0.20 m
- **Drift**: <1°/m rotation, <1% translation

## Troubleshooting

### Camera Issues

```bash
# Check Argus cameras
ls /dev/video*
v4l2-ctl --list-devices

# Test Argus pipeline
gst-launch-1.0 nvarguscamerasrc sensor-id=0 ! nvoverlaysink

# Fallback to V4L2
ros2 launch stereo_camera_bringup stereo_camera.launch.py camera_driver:=v4l2
```

### IMU Issues

```bash
# Check I2C bus
sudo i2cdetect -y 7

# Should see devices at 0x1D and 0x6B

# Test self-test
ros2 service call /imu/self_test std_srvs/srv/Trigger
```

### Build Issues

```bash
# Missing dependencies
rosdep install --from-paths src --ignore-src -r -y

# Clean build
rm -rf build install log
colcon build --symlink-install
```

## Package Structure

```
ekf9dof/
├── stereo_vo/                    # Main VO package
│   ├── src/core/                 # Core algorithms
│   │   ├── feature_tracker.cpp
│   │   ├── stereo_matcher.cpp
│   │   ├── pose_estimator.cpp
│   │   ├── sliding_window.cpp
│   │   └── imu_preintegrator.cpp
│   ├── src/ros/                  # ROS nodes
│   │   └── vo_node.cpp
│   ├── launch/
│   ├── config/
│   ├── scripts/
│   └── test/
├── stereo_camera_bringup/        # Camera drivers
│   ├── src/
│   │   ├── argus_stereo_node.cpp
│   │   └── v4l2_stereo_node.cpp
│   └── launch/
└── lsm9ds0_imu/                  # IMU driver
    ├── src/
    │   ├── lsm9ds0_driver.cpp
    │   └── lsm9ds0_node.cpp
    └── config/
```

## Development

### Run Tests

```bash
colcon test --packages-select stereo_vo
colcon test-result --verbose
```

### Code Style

- C++20 with `-Wall -Wextra -Wpedantic`
- RAII, no raw new/delete
- Doxygen documentation
- Namespaces: `stereo_vo::core`, `stereo_vo::ros`

## References

- [OpenCV Stereo Calibration](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html)
- [GTSAM](https://gtsam.org/)
- [ROS 2 Humble](https://docs.ros.org/en/humble/)
- [Jetson Multimedia API](https://docs.nvidia.com/jetson/l4t-multimedia/index.html)

## License

MIT

## Contributing

Issues and pull requests welcome!
