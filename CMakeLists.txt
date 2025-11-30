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

# ============================================================================
# Optional NNG + Protobuf support for inter-process communication
# ============================================================================
option(USE_NNG_CHANNEL "Enable NNG-based pub/sub channels with protobuf" OFF)

if(USE_NNG_CHANNEL)
    include(FetchContent)

    # Fetch nanomsg-next-gen (nng)
    FetchContent_Declare(
        nng
        GIT_REPOSITORY https://github.com/nanomsg/nng.git
        GIT_TAG        v1.8.0
    )
    set(NNG_TESTS OFF CACHE BOOL "" FORCE)
    set(NNG_TOOLS OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(nng)

    # Find protobuf
    find_package(Protobuf REQUIRED)

    # Generate protobuf C++ sources
    set(PROTO_FILES ${CMAKE_CURRENT_SOURCE_DIR}/proto/imu_messages.proto)
    protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})

    # Create a library for protobuf-generated code
    add_library(imu_proto ${PROTO_SRCS} ${PROTO_HDRS})
    target_include_directories(imu_proto PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
    target_link_libraries(imu_proto PUBLIC protobuf::libprotobuf)

    message(STATUS "NNG + Protobuf support enabled")
endif()

# Common library source files (excluding main executables)
set(LIB_SOURCES
    src/common/i2c_device.cpp
    src/common/custom_time_methods.cpp
    src/common/Quat.cpp
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

# ============================================================================
# Offline Tools
# ============================================================================

# bin2csv - Convert binary IMU logs to CSV format
add_executable(bin2csv tools/bin2csv.cpp)

# sensor2csv - Convert per-sensor binary logs to CSV format
add_executable(sensor2csv tools/sensor2csv.cpp)

# Enable testing
enable_testing()
add_subdirectory(tests)

# Add examples (if directory exists)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/examples)
    add_subdirectory(examples)
endif()

# Installation rules
install(TARGETS ${PROJECT_NAME} bin2csv sensor2csv
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)