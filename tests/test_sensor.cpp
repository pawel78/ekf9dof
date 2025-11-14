#include <iostream>
#include <cassert>
#include <cmath>
#include "imu/processing/sensor.hpp"

using namespace ekf9dof;

// Helper function to compare floats
bool float_equal(float a, float b, float epsilon = 1e-6f) {
    return std::fabs(a - b) < epsilon;
}

void test_sensor_3d_creation() {
    std::cout << "Testing 3D sensor creation..." << std::endl;
    
    Sensor3D gyro("gyro");
    assert(gyro.get_name() == "gyro");
    assert(gyro.get_measurement_count() == 3);
    assert(gyro.get_new_data_flag() == 0);
    assert(gyro.get_timestamp() == 0);
    
    std::cout << "  ✓ 3D sensor creation test passed" << std::endl;
}

void test_sensor_1d_creation() {
    std::cout << "Testing 1D sensor creation..." << std::endl;
    
    Sensor1D temp("temperature");
    assert(temp.get_name() == "temperature");
    assert(temp.get_measurement_count() == 1);
    assert(temp.get_new_data_flag() == 0);
    
    std::cout << "  ✓ 1D sensor creation test passed" << std::endl;
}

void test_sensor_gps_creation() {
    std::cout << "Testing GPS sensor creation..." << std::endl;
    
    SensorGPS gps("gps");
    assert(gps.get_name() == "gps");
    assert(gps.get_measurement_count() == 6);
    assert(gps.get_new_data_flag() == 0);
    
    std::cout << "  ✓ GPS sensor creation test passed" << std::endl;
}

void test_store_3d_data_array() {
    std::cout << "Testing 3D sensor data storage with array..." << std::endl;
    
    Sensor3D accel("accel");
    std::array<float, 3> data = {1.0f, 2.0f, 3.0f};
    uint64_t timestamp = 1000;
    
    uint8_t initial_flag = accel.get_new_data_flag();
    accel.store(data, timestamp);
    
    assert(accel.get_new_data_flag() == initial_flag + 1);
    assert(accel.get_timestamp() == timestamp);
    
    const auto& measurements = accel.get_measurements();
    assert(float_equal(measurements[0], 1.0f));
    assert(float_equal(measurements[1], 2.0f));
    assert(float_equal(measurements[2], 3.0f));
    
    std::cout << "  ✓ 3D sensor data storage test passed" << std::endl;
}

void test_store_3d_data_pointer() {
    std::cout << "Testing 3D sensor data storage with pointer..." << std::endl;
    
    Sensor3D mag("mag");
    float data[3] = {-0.5f, 0.3f, 1.2f};
    uint64_t timestamp = 2000;
    
    uint8_t initial_flag = mag.get_new_data_flag();
    mag.store(data, timestamp);
    
    assert(mag.get_new_data_flag() == initial_flag + 1);
    assert(mag.get_timestamp() == timestamp);
    
    assert(float_equal(mag.get_measurement(0), -0.5f));
    assert(float_equal(mag.get_measurement(1), 0.3f));
    assert(float_equal(mag.get_measurement(2), 1.2f));
    
    std::cout << "  ✓ 3D sensor data storage with pointer test passed" << std::endl;
}

void test_store_1d_data() {
    std::cout << "Testing 1D sensor data storage..." << std::endl;
    
    Sensor1D temp("temperature");
    std::array<float, 1> data = {25.5f};
    uint64_t timestamp = 3000;
    
    uint8_t initial_flag = temp.get_new_data_flag();
    temp.store(data, timestamp);
    
    assert(temp.get_new_data_flag() == initial_flag + 1);
    assert(temp.get_timestamp() == timestamp);
    assert(float_equal(temp.get_measurement(0), 25.5f));
    
    std::cout << "  ✓ 1D sensor data storage test passed" << std::endl;
}

void test_multiple_stores() {
    std::cout << "Testing multiple data stores..." << std::endl;
    
    Sensor3D gyro("gyro");
    
    std::array<float, 3> data1 = {1.0f, 2.0f, 3.0f};
    std::array<float, 3> data2 = {4.0f, 5.0f, 6.0f};
    std::array<float, 3> data3 = {7.0f, 8.0f, 9.0f};
    
    gyro.store(data1, 1000);
    assert(gyro.get_new_data_flag() == 1);
    
    gyro.store(data2, 2000);
    assert(gyro.get_new_data_flag() == 2);
    
    gyro.store(data3, 3000);
    assert(gyro.get_new_data_flag() == 3);
    
    // Verify latest data
    const auto& measurements = gyro.get_measurements();
    assert(float_equal(measurements[0], 7.0f));
    assert(float_equal(measurements[1], 8.0f));
    assert(float_equal(measurements[2], 9.0f));
    assert(gyro.get_timestamp() == 3000);
    
    std::cout << "  ✓ Multiple data stores test passed" << std::endl;
}

void test_new_data_flag_overflow() {
    std::cout << "Testing new data flag overflow..." << std::endl;
    
    Sensor1D temp("temperature");
    std::array<float, 1> data = {20.0f};
    
    // Test boundary condition: store 254 times, should be at 254
    for (int i = 0; i < 254; ++i) {
        temp.store(data);
    }
    assert(temp.get_new_data_flag() == 254);
    
    // Store once more, should be at 255
    temp.store(data);
    assert(temp.get_new_data_flag() == 255);
    
    // Store once more, should overflow to 0
    temp.store(data);
    assert(temp.get_new_data_flag() == 0);
    
    // Store once more, should be at 1
    temp.store(data);
    assert(temp.get_new_data_flag() == 1);
    
    std::cout << "  ✓ New data flag overflow test passed" << std::endl;
}

void test_consumer_pattern() {
    std::cout << "Testing consumer pattern with flag checking..." << std::endl;
    
    Sensor3D accel("accel");
    
    // Consumer stores the last seen flag
    uint8_t last_flag = accel.get_new_data_flag();
    
    // No new data yet
    assert(accel.get_new_data_flag() == last_flag);
    
    // Store new data
    std::array<float, 3> data1 = {1.0f, 2.0f, 3.0f};
    accel.store(data1, 1000);
    
    // Consumer detects new data
    assert(accel.get_new_data_flag() != last_flag);
    last_flag = accel.get_new_data_flag();
    
    // Store another measurement
    std::array<float, 3> data2 = {4.0f, 5.0f, 6.0f};
    accel.store(data2, 2000);
    
    // Consumer detects new data again
    assert(accel.get_new_data_flag() != last_flag);
    
    std::cout << "  ✓ Consumer pattern test passed" << std::endl;
}

void test_gps_sensor_storage() {
    std::cout << "Testing GPS sensor with multiple inputs..." << std::endl;
    
    SensorGPS gps("gps");
    
    // GPS data: lat, lon, alt, vel_x, vel_y, vel_z
    std::array<float, 6> gps_data = {
        37.7749f,  // latitude
        -122.4194f, // longitude
        50.0f,      // altitude
        1.5f,       // velocity_x
        0.5f,       // velocity_y
        0.0f        // velocity_z
    };
    
    gps.store(gps_data, 5000);
    
    assert(gps.get_new_data_flag() == 1);
    assert(gps.get_timestamp() == 5000);
    
    const auto& measurements = gps.get_measurements();
    assert(float_equal(measurements[0], 37.7749f));
    assert(float_equal(measurements[1], -122.4194f));
    assert(float_equal(measurements[2], 50.0f));
    assert(float_equal(measurements[3], 1.5f));
    assert(float_equal(measurements[4], 0.5f));
    assert(float_equal(measurements[5], 0.0f));
    
    std::cout << "  ✓ GPS sensor storage test passed" << std::endl;
}

void test_null_pointer_protection() {
    std::cout << "Testing null pointer protection..." << std::endl;
    
    Sensor3D accel("accel");
    float* null_data = nullptr;
    
    bool exception_thrown = false;
    try {
        accel.store(null_data);
    } catch (const std::invalid_argument&) {
        // Correct exception type was thrown
        exception_thrown = true;
    }
    
    assert(exception_thrown);
    std::cout << "  ✓ Null pointer protection test passed" << std::endl;
}

void test_bounds_checking() {
    std::cout << "Testing bounds checking..." << std::endl;
    
    Sensor3D gyro("gyro");
    std::array<float, 3> data = {1.0f, 2.0f, 3.0f};
    gyro.store(data);
    
    // Valid access - test all valid indices
    float val = gyro.get_measurement(0);
    assert(float_equal(val, 1.0f));
    
    val = gyro.get_measurement(1);
    assert(float_equal(val, 2.0f));
    
    val = gyro.get_measurement(2);
    assert(float_equal(val, 3.0f));
    
    // Invalid access - should throw
    bool exception_thrown = false;
    try {
        gyro.get_measurement(3);  // Out of bounds
    } catch (const std::out_of_range&) {
        // Correct exception type was thrown
        exception_thrown = true;
    }
    
    assert(exception_thrown);
    std::cout << "  ✓ Bounds checking test passed" << std::endl;
}

int main() {
    std::cout << "Running Sensor class tests..." << std::endl;
    std::cout << "==============================" << std::endl;
    
    try {
        test_sensor_3d_creation();
        test_sensor_1d_creation();
        test_sensor_gps_creation();
        test_store_3d_data_array();
        test_store_3d_data_pointer();
        test_store_1d_data();
        test_multiple_stores();
        test_new_data_flag_overflow();
        test_consumer_pattern();
        test_gps_sensor_storage();
        test_null_pointer_protection();
        test_bounds_checking();
        
        std::cout << "==============================" << std::endl;
        std::cout << "All tests passed! ✓" << std::endl;
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Test failed with exception: " << e.what() << std::endl;
        return 1;
    }
}
