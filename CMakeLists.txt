cmake_minimum_required(VERSION 3.18)
project(ekf9dof VERSION 1.0.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Enable optimization flags for Release builds
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

# Option to enable sensor test mode (displays sensor values to screen)
option(ENABLE_SENSOR_TEST "Enable sensor test mode (displays values instead of logging)" OFF)

# Collect all source files
file(GLOB_RECURSE SOURCES 
    "src/*.cpp"
    "include/ekf9dof/*.cpp"
)

# Create the main executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Add include directories
target_include_directories(${PROJECT_NAME} 
    PUBLIC 
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Add preprocessor definition if ENABLE_SENSOR_TEST is ON
if(ENABLE_SENSOR_TEST)
    target_compile_definitions(${PROJECT_NAME} PRIVATE ENABLE_SENSOR_TEST=1)
endif()

# Enable testing
enable_testing()
add_subdirectory(tests)

# Installation rules
install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)