# Test executable for sensor class
add_executable(test_sensor test_sensor.cpp)
target_include_directories(test_sensor PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_compile_features(test_sensor PRIVATE cxx_std_17)

# Test executable for calibration logic
add_executable(test_calibration test_calibration.cpp)
target_include_directories(test_calibration PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_compile_features(test_calibration PRIVATE cxx_std_17)

# Add tests
add_test(NAME SensorTest COMMAND test_sensor)
add_test(NAME CalibrationTest COMMAND test_calibration)
