cmake_minimum_required(VERSION 3.18)
project(ekf9dof VERSION 1.0.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Platform detection
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    message(STATUS "Building for Linux - I2C hardware access enabled")
elseif(APPLE)
    message(WARNING "Building for macOS - I2C hardware access NOT available. Build for compile-time error checking only.")
else()
    message(WARNING "Building for ${CMAKE_SYSTEM_NAME} - I2C hardware access NOT available. Build for compile-time error checking only.")
endif()

# Enable optimization flags for Release builds
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

# Common library source files (excluding main executables)
set(LIB_SOURCES
    src/common/i2c_device.cpp
    src/imu/drivers/lsm9ds0.cpp
    src/imu/drivers/lsm9ds0_driver.cpp
    src/common/config_loader.cpp
    src/imu/processing/imu_preprocessor.cpp
)

# Create the main executable
add_executable(${PROJECT_NAME} 
    ${LIB_SOURCES}
    src/main.cpp
)

# Add include directories
target_include_directories(${PROJECT_NAME} 
    PUBLIC 
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Enable testing
enable_testing()
add_subdirectory(tests)

# Add examples (if directory exists)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/examples)
    add_subdirectory(examples)
endif()

# Installation rules
install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)