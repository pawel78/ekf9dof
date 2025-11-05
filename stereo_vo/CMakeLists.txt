cmake_minimum_required(VERSION 3.8)
project(stereo_vo)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3 -DNDEBUG)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

# Check for GTSAM (optional but recommended)
find_package(GTSAM QUIET)
if(GTSAM_FOUND)
  message(STATUS "GTSAM found - enabling bundle adjustment")
  add_compile_definitions(USE_GTSAM)
else()
  message(WARNING "GTSAM not found - bundle adjustment will be limited")
endif()

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Core library sources
set(CORE_SOURCES
  src/core/feature_tracker.cpp
  src/core/stereo_matcher.cpp
  src/core/pose_estimator.cpp
  src/core/sliding_window.cpp
  src/core/imu_preintegrator.cpp
)

# Create core library
add_library(${PROJECT_NAME}_core SHARED ${CORE_SOURCES})
ament_target_dependencies(${PROJECT_NAME}_core
  rclcpp
  sensor_msgs
  Eigen3
  OpenCV
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
if(GTSAM_FOUND)
  target_link_libraries(${PROJECT_NAME}_core gtsam)
endif()

# VO node executable
add_executable(vo_node src/ros/vo_node.cpp)
ament_target_dependencies(vo_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  image_transport
  cv_bridge
  message_filters
)
target_link_libraries(vo_node ${PROJECT_NAME}_core ${OpenCV_LIBS})

# Install targets
install(TARGETS
  ${PROJECT_NAME}_core
  vo_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install scripts
install(DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
  USE_SOURCE_PERMISSIONS
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  
  # Linting
  ament_lint_auto_find_test_dependencies()
  
  # Unit tests
  ament_add_gtest(test_feature_tracker test/test_feature_tracker.cpp)
  target_link_libraries(test_feature_tracker ${PROJECT_NAME}_core)
  
  ament_add_gtest(test_pose_estimator test/test_pose_estimator.cpp)
  target_link_libraries(test_pose_estimator ${PROJECT_NAME}_core)
endif()

ament_package()
