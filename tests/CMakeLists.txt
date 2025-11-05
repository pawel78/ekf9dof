# Test executable for sensor class
add_executable(test_sensor test_sensor.cpp)
target_include_directories(test_sensor PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_compile_features(test_sensor PRIVATE cxx_std_17)

# Add test
add_test(NAME SensorTest COMMAND test_sensor)

# Test executable for SO3 library
add_executable(test_so3 test_so3.cpp)
target_include_directories(test_so3 PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_compile_features(test_so3 PRIVATE cxx_std_17)
target_link_libraries(test_so3 Eigen3::Eigen)

# Add test
add_test(NAME SO3Test COMMAND test_so3)
