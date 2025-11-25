#include "imu/processing/imu_preprocessor.hpp"
#include "common/config_loader.hpp"
#include "imu/messages/imu_data.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <iomanip>

void IMUPreprocessor::estimate_gyro_bias()
{   
    if (stationary_gyro_cal_) {
        sum_gx_ += yg_[0];
        sum_gy_ += yg_[1];
        sum_gz_ += yg_[2];
        sample_count_++;
        
        if (sample_count_ >= 1000) { // Collect 1000 samples
            gyro_bias_[0] = sum_gx_ / sample_count_;
            gyro_bias_[1] = sum_gy_ / sample_count_;
            gyro_bias_[2] = sum_gz_ / sample_count_;
            stationary_gyro_cal_ = false;
            std::cout << "Gyro bias estimated: "
                      << "gx_bias=" << gyro_bias_[0]
                      << ", gy_bias=" << gyro_bias_[1]
                      << ", gz_bias=" << gyro_bias_[2] << "\n";
        }   
    }
}
void IMUPreprocessor::apply_mag_calibration(float mx_raw, float my_raw, float mz_raw, float &mx_cal, float &my_cal, float &mz_cal)
{
    if (!mag_calibration_loaded_)
    {
        mx_cal = mx_raw;
        my_cal = my_raw;
        mz_cal = mz_raw;
        return;
    }

    // Apply hard iron correction (subtract bias)
    float mx_bias_corrected = mx_raw - mag_bias_[0];
    float my_bias_corrected = my_raw - mag_bias_[1];
    float mz_bias_corrected = mz_raw - mag_bias_[2];

    // Apply soft iron correction (matrix multiplication)
    mx_cal = mag_matrix_[0] * mx_bias_corrected + mag_matrix_[1] * my_bias_corrected + mag_matrix_[2] * mz_bias_corrected;
    my_cal = mag_matrix_[3] * mx_bias_corrected + mag_matrix_[4] * my_bias_corrected + mag_matrix_[5] * mz_bias_corrected;
    mz_cal = mag_matrix_[6] * mx_bias_corrected + mag_matrix_[7] * my_bias_corrected + mag_matrix_[8] * mz_bias_corrected;
}

void IMUPreprocessor::apply_accel_calibration(float ax_raw, float ay_raw, float az_raw, float &ax_cal, float &ay_cal, float &az_cal)
{
    if (!accel_calibration_loaded_)
    {
        ax_cal = ax_raw;
        ay_cal = ay_raw;
        az_cal = az_raw;
        return;
    }

    // Apply hard iron correction (subtract bias)
    float ax_bias_corrected = ax_raw - accel_bias_[0];
    float ay_bias_corrected = ay_raw - accel_bias_[1];
    float az_bias_corrected = az_raw - accel_bias_[2];

    // Apply soft iron correction (matrix multiplication)
    ax_cal = accel_matrix_[0] * ax_bias_corrected + accel_matrix_[1] * ay_bias_corrected + accel_matrix_[2] * az_bias_corrected;
    ay_cal = accel_matrix_[3] * ax_bias_corrected + accel_matrix_[4] * ay_bias_corrected + accel_matrix_[5] * az_bias_corrected;
    az_cal = accel_matrix_[6] * ax_bias_corrected + accel_matrix_[7] * ay_bias_corrected + accel_matrix_[8] * az_bias_corrected;
}

void IMUPreprocessor::apply_gyro_calibration(float gx_raw, float gy_raw, float gz_raw,
                                             float &gx_cal, float &gy_cal, float &gz_cal)
{
    if (!gyro_calibration_loaded_)
    {
        gx_cal = gx_raw;
        gy_cal = gy_raw;
        gz_cal = gz_raw;
        return;
    }

    // Apply hard iron correction (subtract bias)
    float gx_bias_corrected = gx_raw - gyro_bias_[0];
    float gy_bias_corrected = gy_raw - gyro_bias_[1];
    float gz_bias_corrected = gz_raw - gyro_bias_[2];

    // Apply soft iron correction (matrix multiplication)
    gx_cal = gyro_matrix_[0] * gx_bias_corrected + gyro_matrix_[1] * gy_bias_corrected + gyro_matrix_[2] * gz_bias_corrected;
    gy_cal = gyro_matrix_[3] * gx_bias_corrected + gyro_matrix_[4] * gy_bias_corrected + gyro_matrix_[5] * gz_bias_corrected;
    gz_cal = gyro_matrix_[6] * gx_bias_corrected + gyro_matrix_[7] * gy_bias_corrected + gyro_matrix_[8] * gz_bias_corrected;
}

IMUPreprocessor::IMUPreprocessor()
    : running_(false)
{

    ym_ = {0.0f, 0.0f, 0.0f};
    ya_ = {0.0f, 0.0f, 0.0f};
    yg_ = {0.0f, 0.0f, 0.0f};

    mag_bias_ = {0.0f, 0.0f, 0.0f};
    mag_matrix_ = {1.0f, 0.0f, 0.0f,
                                        0.0f, 1.0f, 0.0f,
                                        0.0f, 0.0f, 1.0f};
    mag_calibration_loaded_ = false;

    accel_bias_ = {0.0f, 0.0f, 0.0f};
    accel_matrix_ = {1.0f, 0.0f, 0.0f,
                                          0.0f, 1.0f, 0.0f,
                                          0.0f, 0.0f, 1.0f};
    accel_calibration_loaded_ = false;

    gyro_bias_ = {0.0f, 0.0f, 0.0f};
    gyro_matrix_ = {1.0f, 0.0f, 0.0f,
                                         0.0f, 1.0f, 0.0f,
                                         0.0f, 0.0f, 1.0f};
    gyro_calibration_loaded_ = false;
    calibration_loaded_ = false;

    stationary_gyro_cal_ = true;
    sum_gx_ = 0.0;
    sum_gy_ = 0.0;
    sum_gz_ = 0.0;
    sample_count_ = 0;  

    // Try to load magnetometer calibration
    if (config_loader::load_mag_calibration("../configs/config.yaml", mag_bias_, mag_matrix_))
    {
        mag_calibration_loaded_ = true;
        std::cout << "✓ Magnetometer calibration loaded\n";
        std::cout << "  Bias: [" << mag_bias_[0] << ", " << mag_bias_[1] << ", " << mag_bias_[2] << "]\n";
    }
    else
    {
        std::cout << "⚠ No magnetometer calibration found (using identity)\n";
    }

    // Try to load accelerometer calibration
    if (config_loader::load_accel_calibration("../configs/config.yaml", accel_bias_, accel_matrix_))
    {
        accel_calibration_loaded_ = true;
        std::cout << "✓ Accelerometer calibration loaded\n";
        std::cout << "  Bias: [" << accel_bias_[0] << ", " << accel_bias_[1] << ", " << accel_bias_[2] << "]\n";
    }
    else
    {
        std::cout << "⚠ No accelerometer calibration found (using identity)\n";
    }

    // Try to load magnetometer calibration
    if (config_loader::load_gyro_calibration("../configs/config.yaml", gyro_bias_, gyro_matrix_))
    {
        gyro_calibration_loaded_ = true;
        std::cout << "✓ Gyroscope calibration loaded\n";
        std::cout << "  Bias: [" << gyro_bias_[0] << ", " << gyro_bias_[1] << ", " << gyro_bias_[2] << "]\n";
    }
    else
    {
        std::cout << "⚠ No gyroscope calibration found (using identity)\n";
    }

    // Overall calibration status
    if (mag_calibration_loaded_ || accel_calibration_loaded_ || gyro_calibration_loaded_)
    {
        calibration_loaded_ = true;
    }
    else
    {
        calibration_loaded_ = false;
    }
}

IMUPreprocessor::~IMUPreprocessor()
{
    stop();
}

void IMUPreprocessor::get_mag_calibration(std::array<float, 3> &bias, std::array<float, 9> &matrix)
{
    bias = mag_bias_;
    matrix = mag_matrix_;
}
void IMUPreprocessor::get_accel_calibration(std::array<float, 3> &bias, std::array<float, 9> &matrix)
{
    bias = accel_bias_;
    matrix = accel_matrix_;
}
void IMUPreprocessor::get_gyro_calibration(std::array<float, 3> &bias, std::array<float, 9> &matrix)
{
    bias = gyro_bias_;
    matrix = gyro_matrix_;
}
void IMUPreprocessor::get_gyro_measurement(float &gx, float &gy, float &gz)
{
    // Try to read gyro from channel
    bool have_data = false;
    imu::messages::raw_gyro_msg_t gyro{0, 0, 0, 0};
    if (imu::channels::raw_gyro.try_receive(gyro))
        have_data = true;

    apply_gyro_calibration(gyro.x, gyro.y, gyro.z, yg_[0], yg_[1], yg_[2]);
    gx = yg_[0];
    gy = yg_[1];
    gz = yg_[2];
}
void IMUPreprocessor::get_accel_measurement(float &ax, float &ay, float &az)
{
    // Try to read gyro from channel
    bool have_data = false;
    imu::messages::raw_accel_msg_t accel{0, 0, 0, 0};
    if (imu::channels::raw_accel.try_receive(accel))
        have_data = true;

    apply_accel_calibration(accel.x, accel.y, accel.z, ya_[0], ya_[1], ya_[2]);
    ax = ya_[0];
    ay = ya_[1];
    az = ya_[2];
}

void IMUPreprocessor::get_mag_measurement(float &mx, float &my, float &mz)
{
    // Try to read gyro from channel
    bool have_data = false;
    imu::messages::raw_mag_msg_t mag{0, 0, 0, 0};
    if (imu::channels::raw_mag.try_receive(mag))
        have_data = true;
    apply_mag_calibration(mag.x, mag.y, mag.z, ym_[0], ym_[1], ym_[2]);
    mx = ym_[0];
    my = ym_[1];
    mz = ym_[2];
}

void IMUPreprocessor::start()
{
    if (running_.load())
    {
        std::cerr << "WARNING: Preprocessor already running\n";
        return;
    }

    std::cout << "Starting IMU preprocessor thread...\n";
    running_.store(true);

    // Spawn preprocessor thread using static member function
    preprocessor_thread_ = std::thread(preprocessor_thread_func, this);

    std::cout << "✓ Preprocessor thread started\n";
}

void IMUPreprocessor::stop()
{
    if (!running_.load())
    {
        return; // Already stopped
    }

    std::cout << "Stopping IMU preprocessor...\n";

    // Signal thread to stop
    running_.store(false);

    // Wait for thread to finish
    if (preprocessor_thread_.joinable())
    {
        preprocessor_thread_.join();
    }

    std::cout << "✓ Preprocessor stopped\n";
}

void IMUPreprocessor::preprocessor_thread_func(IMUPreprocessor *preprocessor)
{
    std::cout << "IMU Preprocessor thread running.\n";
    
    if (preprocessor->stationary_gyro_cal_) {
        std::cout << "Gyro bias estimation enabled. Ensure the IMU is stationary.\n";
    }   

    while (preprocessor->running_.load())
    {
        // get gyro, accel, mag measurements with calibration applied
        float gx, gy, gz;
        preprocessor->get_gyro_measurement(gx, gy, gz);
        float ax, ay, az;
        preprocessor->get_accel_measurement(ax, ay, az);
        float mx, my, mz;
        preprocessor->get_mag_measurement(mx, my, mz);

        if (preprocessor->stationary_gyro_cal_) {
            preprocessor->estimate_gyro_bias();
        }

        // Just sleep, processing is done on-demand in get_measurement functions
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}