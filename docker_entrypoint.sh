#!/bin/bash
set -e

# Source ROS 2
source /opt/ros/${ROS_DISTRO}/setup.bash
source /root/stereo_vo_ws/install/setup.bash

# Execute command
exec "$@"
