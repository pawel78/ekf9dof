# Test executable for sensor class
add_executable(test_sensor test_sensor.cpp)
target_include_directories(test_sensor PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_compile_features(test_sensor PRIVATE cxx_std_17)

# Add test
add_test(NAME SensorTest COMMAND test_sensor)

# Test executable for magnetometer calibration
add_executable(test_mag_calibration 
    test_mag_calibration.cpp
    ${CMAKE_SOURCE_DIR}/src/imu/drivers/lsm9ds0.cpp
    ${CMAKE_SOURCE_DIR}/src/imu/processing/mag_calibration.cpp
)
target_include_directories(test_mag_calibration PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_compile_features(test_mag_calibration PRIVATE cxx_std_17)

# Add test
add_test(NAME MagCalibrationTest COMMAND test_mag_calibration)

# Test executable for config loader
add_executable(test_config_loader 
    test_config_loader.cpp
    ${CMAKE_SOURCE_DIR}/src/common/config_loader.cpp
)
target_include_directories(test_config_loader PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_compile_features(test_config_loader PRIVATE cxx_std_17)

# Add test
add_test(NAME ConfigLoaderTest COMMAND test_config_loader)
