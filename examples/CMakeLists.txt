# Example executables demonstrating sensor class usage

# Sensor usage example
add_executable(sensor_usage_example sensor_usage_example.cpp)
target_include_directories(sensor_usage_example PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_compile_features(sensor_usage_example PRIVATE cxx_std_17)
