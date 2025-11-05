cmake_minimum_required(VERSION 3.8)
project(stereo_camera_bringup)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(include)

# Argus stereo node (Jetson-specific)
add_executable(argus_stereo_node src/argus_stereo_node.cpp)
ament_target_dependencies(argus_stereo_node
  rclcpp
  sensor_msgs
  camera_info_manager
  image_transport
  cv_bridge
)
target_link_libraries(argus_stereo_node ${OpenCV_LIBS})

# V4L2 stereo node
add_executable(v4l2_stereo_node src/v4l2_stereo_node.cpp)
ament_target_dependencies(v4l2_stereo_node
  rclcpp
  sensor_msgs
  camera_info_manager
  image_transport
  cv_bridge
)
target_link_libraries(v4l2_stereo_node ${OpenCV_LIBS})

# Install targets
install(TARGETS
  argus_stereo_node
  v4l2_stereo_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
