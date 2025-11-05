#!/bin/bash
# Record stereo camera and IMU data to a ROS 2 bag

# Get timestamp for filename
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
BAG_DIR="${HOME}/stereo_vo_bags"
BAG_NAME="stereo_vo_${TIMESTAMP}"

# Create bag directory if it doesn't exist
mkdir -p "${BAG_DIR}"

echo "Recording to: ${BAG_DIR}/${BAG_NAME}"
echo "Topics: /stereo/left/image_raw, /stereo/right/image_raw, /stereo/*/camera_info, /imu/data"
echo "Press Ctrl+C to stop recording"

# Record with ZSTD compression
ros2 bag record \
    -o "${BAG_DIR}/${BAG_NAME}" \
    --compression-mode file \
    --compression-format zstd \
    /stereo/left/image_raw \
    /stereo/right/image_raw \
    /stereo/left/camera_info \
    /stereo/right/camera_info \
    /imu/data \
    /vo/odom \
    /tf
