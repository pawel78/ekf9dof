# Dockerfile for Jetson Orin Nano (L4T base)
# Based on NVIDIA L4T with ROS 2 Humble

FROM dustynv/ros:humble-pytorch-l4t-r35.2.1

# Set environment
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Install dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    libeigen3-dev \
    libopencv-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    gstreamer1.0-tools \
    i2c-tools \
    libi2c-dev \
    python3-opencv \
    python3-yaml \
    python3-numpy \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-camera-info-manager \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-message-filters \
    && rm -rf /var/lib/apt/lists/*

# Optional: Install GTSAM
RUN apt-get update && apt-get install -y \
    libgtsam-dev \
    || echo "GTSAM not available, continuing without it" \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
RUN mkdir -p /root/stereo_vo_ws/src
WORKDIR /root/stereo_vo_ws

# Copy source code
COPY . /root/stereo_vo_ws/src/ekf9dof/

# Build workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --symlink-install \
    --cmake-args \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_CXX_FLAGS="-O3 -DNDEBUG -march=native"

# Source workspace in bashrc
RUN echo "source /root/stereo_vo_ws/install/setup.bash" >> /root/.bashrc

# Set entrypoint
COPY docker_entrypoint.sh /root/
RUN chmod +x /root/docker_entrypoint.sh
ENTRYPOINT ["/root/docker_entrypoint.sh"]
CMD ["bash"]
