cmake_minimum_required(VERSION 3.18)
project(ekf9dof VERSION 1.0.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Enable optimization flags for Release builds
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

# Common library source files (excluding main executables)
set(LIB_SOURCES
    src/lsm9ds0.cpp
    src/attitude_ekf.cpp
    src/mag_calibration.cpp
    src/config_loader.cpp
    include/ekf9dof/attitude_ekf.cpp
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

# Create the calibration tool executable
add_executable(calibrate_mag
    ${LIB_SOURCES}
    src/calibrate_mag.cpp
)

target_include_directories(calibrate_mag
    PUBLIC 
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Enable testing
enable_testing()
add_subdirectory(tests)

# Add examples
add_subdirectory(examples)

# Installation rules
install(TARGETS ${PROJECT_NAME} calibrate_mag
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)