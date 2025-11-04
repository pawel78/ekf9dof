# Test executable for sensor class
add_executable(test_sensor test_sensor.cpp)
target_include_directories(test_sensor PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_compile_features(test_sensor PRIVATE cxx_std_17)

# Add test
add_test(NAME SensorTest COMMAND test_sensor)
