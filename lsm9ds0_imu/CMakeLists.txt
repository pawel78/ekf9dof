cmake_minimum_required(VERSION 3.8)
project(lsm9ds0_imu)

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
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(include)

# LSM9DS0 node
add_executable(lsm9ds0_node src/lsm9ds0_node.cpp src/lsm9ds0_driver.cpp)
ament_target_dependencies(lsm9ds0_node
  rclcpp
  sensor_msgs
  std_srvs
)

# Install targets
install(TARGETS
  lsm9ds0_node
  DESTINATION lib/${PROJECT_NAME}
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
