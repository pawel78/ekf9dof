#include "imu/processing/imu_preprocessor.hpp"
#include "common/config_loader.hpp"
#include "imu/messages/imu_data.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <iomanip>

void IMUPreprocessor::estimate_gyro_bias()
{
    if (!stationary_gyro_cal_)
    {
        return; // Already estimated
    }

    // Accumulate raw gyro readings (yg_ contains raw data during estimation)
    sum_gx_ += yg_[0];
    sum_gy_ += yg_[1];
    sum_gz_ += yg_[2];
    sample_count_++;

    // Print progress every 200 samples
    if (sample_count_ % 200 == 0)
    {
        std::cout << "  Gyro bias estimation: " << sample_count_ << "/1000 samples...\n";
    }

    if (sample_count_ >= 1000)
    { // Collect 1000 samples
        gyro_bias_[0] = sum_gx_ / sample_count_;
        gyro_bias_[1] = sum_gy_ / sample_count_;
        gyro_bias_[2] = sum_gz_ / sample_count_;
        stationary_gyro_cal_ = false;
        gyro_calibration_loaded_ = true; // Enable bias correction
        std::cout << "✓ Gyro bias estimated from " << sample_count_ << " samples:\n";
        std::cout << "  Bias: [" << gyro_bias_[0] << ", " << gyro_bias_[1] << ", " << gyro_bias_[2] << "]\n";
    }
}
void IMUPreprocessor::apply_mag_calibration(const std::array<float, 3> &m_raw, std::array<float, 3> &m_cal)
{
    if (!mag_calibration_loaded_)
    {
        m_cal = m_raw;
        return;
    }

    // Apply hard iron correction (subtract bias)
    float mx_bias_corrected = m_raw[0] - mag_bias_[0];
    float my_bias_corrected = m_raw[1] - mag_bias_[1];
    float mz_bias_corrected = m_raw[2] - mag_bias_[2];

    // Apply soft iron correction (matrix multiplication)
    m_cal[0] = mag_matrix_[0] * mx_bias_corrected + mag_matrix_[1] * my_bias_corrected + mag_matrix_[2] * mz_bias_corrected;
    m_cal[1] = mag_matrix_[3] * mx_bias_corrected + mag_matrix_[4] * my_bias_corrected + mag_matrix_[5] * mz_bias_corrected;
    m_cal[2] = mag_matrix_[6] * mx_bias_corrected + mag_matrix_[7] * my_bias_corrected + mag_matrix_[8] * mz_bias_corrected;
}

void IMUPreprocessor::apply_accel_calibration(const std::array<float, 3> &a_raw, std::array<float, 3> &a_cal)
{
    if (!accel_calibration_loaded_)
    {
        a_cal = a_raw;
        return;
    }

    // Apply hard iron correction (subtract bias)
    float ax_bias_corrected = a_raw[0] - accel_bias_[0];
    float ay_bias_corrected = a_raw[1] - accel_bias_[1];
    float az_bias_corrected = a_raw[2] - accel_bias_[2];

    // Apply soft iron correction (matrix multiplication)
    a_cal[0] = accel_matrix_[0] * ax_bias_corrected + accel_matrix_[1] * ay_bias_corrected + accel_matrix_[2] * az_bias_corrected;
    a_cal[1] = accel_matrix_[3] * ax_bias_corrected + accel_matrix_[4] * ay_bias_corrected + accel_matrix_[5] * az_bias_corrected;
    a_cal[2] = accel_matrix_[6] * ax_bias_corrected + accel_matrix_[7] * ay_bias_corrected + accel_matrix_[8] * az_bias_corrected;
}

void IMUPreprocessor::apply_gyro_calibration(const std::array<float, 3> &g_raw, std::array<float, 3> &g_cal)
{
    // Apply bias correction and matrix multiplication
    float gx_bias_corrected = g_raw[0] - gyro_bias_[0];
    float gy_bias_corrected = g_raw[1] - gyro_bias_[1];
    float gz_bias_corrected = g_raw[2] - gyro_bias_[2];
    
    g_cal[0] = gyro_matrix_[0] * gx_bias_corrected + gyro_matrix_[1] * gy_bias_corrected + gyro_matrix_[2] * gz_bias_corrected;
    g_cal[1] = gyro_matrix_[3] * gx_bias_corrected + gyro_matrix_[4] * gy_bias_corrected + gyro_matrix_[5] * gz_bias_corrected;
    g_cal[2] = gyro_matrix_[6] * gx_bias_corrected + gyro_matrix_[7] * gy_bias_corrected + gyro_matrix_[8] * gz_bias_corrected;
}

void IMUPreprocessor::rotate_imu_to_bdy(const std::array<float, 3> &v_imu, std::array<float, 3> &v_bdy)
{
    // Apply rotation matrix: v_bdy = bdy_R_imu * v_imu
    v_bdy[0] = bdy_R_imu_[0] * v_imu[0] + bdy_R_imu_[1] * v_imu[1] + bdy_R_imu_[2] * v_imu[2];
    v_bdy[1] = bdy_R_imu_[3] * v_imu[0] + bdy_R_imu_[4] * v_imu[1] + bdy_R_imu_[5] * v_imu[2];
    v_bdy[2] = bdy_R_imu_[6] * v_imu[0] + bdy_R_imu_[7] * v_imu[1] + bdy_R_imu_[8] * v_imu[2];
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
    mag_dec_ = 0.0f;
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
    
    // Initialize IMU-to-body rotation to identity
    bdy_R_imu_ = {1.0f, 0.0f, 0.0f,
                  0.0f, 1.0f, 0.0f,
                  0.0f, 0.0f, 1.0f};
    imu_rotation_loaded_ = false;

    stationary_gyro_cal_ = true;
    sum_gx_ = 0.0;
    sum_gy_ = 0.0;
    sum_gz_ = 0.0;
    sample_count_ = 0;

    // Try to load magnetometer calibration
    if (config_loader::load_mag_calibration("../configs/config.yaml", mag_bias_, mag_matrix_, mag_dec_))
    {
        mag_calibration_loaded_ = true;
        std::cout << "✓ Magnetometer calibration loaded\n";
        std::cout << "  Bias: [" << mag_bias_[0] << ", " << mag_bias_[1] << ", " << mag_bias_[2] << "]\n";
        std::cout << "  Declination: " << mag_dec_ * (180.0f / 3.14159265358979323846f) << " converted to deg. for display only\n";
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

    // Try to load gyroscope calibration
    if (config_loader::load_gyro_calibration("../configs/config.yaml", gyro_bias_, gyro_matrix_))
    {
        gyro_calibration_loaded_ = true;
        std::cout << "✓ Gyroscope calibration loaded\n";
        std::cout << "  Bias: [" << gyro_bias_[0] << ", " << gyro_bias_[1] << ", " << gyro_bias_[2] << "]\n";
        std::cout << "  → Will also estimate gyro bias on startup (keep IMU stationary!)\n";
    }
    else
    {
        std::cout << "⚠ No gyroscope calibration found\n";
        std::cout << "  → Will estimate gyro bias on startup (keep IMU stationary!)\n";
    }
    
    // Try to load IMU-to-body frame rotation
    if (config_loader::load_imu_to_body_rotation("../configs/config.yaml", bdy_R_imu_))
    {
        imu_rotation_loaded_ = true;
        std::cout << "✓ IMU-to-body rotation loaded\n";
        std::cout << "  [" << bdy_R_imu_[0] << ", " << bdy_R_imu_[1] << ", " << bdy_R_imu_[2] << "]\n";
        std::cout << "  [" << bdy_R_imu_[3] << ", " << bdy_R_imu_[4] << ", " << bdy_R_imu_[5] << "]\n";
        std::cout << "  [" << bdy_R_imu_[6] << ", " << bdy_R_imu_[7] << ", " << bdy_R_imu_[8] << "]\n";
    }
    else
    {
        std::cout << "⚠ No IMU-to-body rotation found (using identity)\n";
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

    // Close NNG channels
    if (nng_gyro_sub_)
        nng_gyro_sub_->close();
    if (nng_accel_sub_)
        nng_accel_sub_->close();
    if (nng_mag_sub_)
        nng_mag_sub_->close();
    if (nng_temp_sub_)
        nng_temp_sub_->close();
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
bool IMUPreprocessor::get_gyro_measurement(float &gx, float &gy, float &gz)
{
    // Try to read gyro from NNG channel
    messages::raw_gyro_msg_t gyro{0, 0, 0, 0};
    bool have_data = nng_gyro_sub_->try_receive(gyro);

    // Only apply calibration if we received new data
    if (have_data)
    {
        // First rotate from IMU frame to body frame
        std::array<float, 3> yg_imu = {gyro.x, gyro.y, gyro.z};
        std::array<float, 3> yg_bdy;
        rotate_imu_to_bdy(yg_imu, yg_bdy);
        
        // Then apply calibration
        std::array<float, 3> yg_cal;
        apply_gyro_calibration(yg_bdy, yg_cal);
        yg_[0] = yg_cal[0];
        yg_[1] = yg_cal[1];
        yg_[2] = yg_cal[2];
    }

    gx = yg_[0];
    gy = yg_[1];
    gz = yg_[2];
    
    return have_data;
}
bool IMUPreprocessor::get_accel_measurement(float &ax, float &ay, float &az)
{
    // Try to read accel from NNG channel
    messages::raw_accel_msg_t accel{0, 0, 0, 0};
    bool have_data = nng_accel_sub_->try_receive(accel);

    // Only apply calibration if we received new data
    if (have_data)
    {
        // First rotate from IMU frame to body frame
        std::array<float, 3> ya_imu = {accel.x, accel.y, accel.z};
        std::array<float, 3> ya_bdy;
        rotate_imu_to_bdy(ya_imu, ya_bdy);
        
        // Then apply calibration
        std::array<float, 3> ya_cal;
        apply_accel_calibration(ya_bdy, ya_cal);
        ya_[0] = ya_cal[0];
        ya_[1] = ya_cal[1];
        ya_[2] = ya_cal[2];
    }

    ax = ya_[0];
    ay = ya_[1];
    az = ya_[2];
    
    return have_data;
}

bool IMUPreprocessor::get_mag_measurement(float &mx, float &my, float &mz)
{
    // Try to read mag from NNG channel
    messages::raw_mag_msg_t mag{0, 0, 0, 0};
    bool have_data = nng_mag_sub_->try_receive(mag);

    // Only apply calibration if we received new data
    if (have_data)
    {
        // First rotate from IMU frame to body frame
        std::array<float, 3> ym_imu = {mag.x, mag.y, mag.z};
        std::array<float, 3> ym_bdy;
        rotate_imu_to_bdy(ym_imu, ym_bdy);
        
        // Then apply calibration
        std::array<float, 3> ym_cal;
        apply_mag_calibration(ym_bdy, ym_cal);
        ym_[0] = ym_cal[0];
        ym_[1] = ym_cal[1];
        ym_[2] = ym_cal[2];
    }

    mx = ym_[0];
    my = ym_[1];
    mz = ym_[2];
    
    return have_data;
}

void IMUPreprocessor::start()
{
    if (running_.load())
    {
        std::cerr << "WARNING: Preprocessor already running\n";
        return;
    }

    // Initialize NNG subscriber sockets (do this in start() to ensure publishers are ready)
    std::cout << "Initializing NNG subscriber sockets...\n";
    nng_gyro_sub_ = std::make_unique<NngRawGyroSocket>(nng_urls::RAW_GYRO, false);
    nng_accel_sub_ = std::make_unique<NngRawAccelSocket>(nng_urls::RAW_ACCEL, false);
    nng_mag_sub_ = std::make_unique<NngRawMagSocket>(nng_urls::RAW_MAG, false);
    nng_temp_sub_ = std::make_unique<NngRawTempSocket>(nng_urls::RAW_TEMP, false);
    std::cout << "✓ NNG subscriber sockets initialized\n";

    // Initialize NNG publisher sockets
    std::cout << "Initializing NNG publisher sockets...\n";
    nng_proc_gyro_pub_ = std::make_unique<NngProcGyroSocket>(nng_urls::PROC_GYRO, true);
    nng_proc_accel_pub_ = std::make_unique<NngProcAccelSocket>(nng_urls::PROC_ACCEL, true);
    nng_proc_mag_pub_ = std::make_unique<NngProcMagSocket>(nng_urls::PROC_MAG, true);
    nng_proc_temp_pub_ = std::make_unique<NngProcTempSocket>(nng_urls::PROC_TEMP, true);
    std::cout << "✓ NNG publisher sockets initialized\n";

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
    float gx, gy, gz, ax, ay, az, mx, my, mz;
    uint64_t timestamp = 0;
    bool gyro_first_sample = true;
    bool accel_first_sample = true;
    bool mag_first_sample = true;
    size_t gyro_sent = 0;
    size_t accel_sent = 0;
    size_t mag_sent = 0;
    std::cout << "IMU Preprocessor thread running.\n";

    if (preprocessor->stationary_gyro_cal_)
    {
        std::cout << "Gyro bias estimation enabled. Ensure the IMU is stationary.\n";
    }
    
    // Wait for first valid data from driver before publishing
    std::cout << "Waiting for first sensor data from driver...\n";
    messages::raw_gyro_msg_t gyro_init{0, 0, 0, 0};
    messages::raw_accel_msg_t accel_init{0, 0, 0, 0};
    messages::raw_mag_msg_t mag_init{0, 0, 0, 0};
    
    while (preprocessor->running_.load())
    {
        if (preprocessor->nng_gyro_sub_->try_receive(gyro_init) &&
            preprocessor->nng_accel_sub_->try_receive(accel_init) &&
            preprocessor->nng_mag_sub_->try_receive(mag_init))
        {
            // Apply IMU-to-body rotation before storing initial values
            std::array<float, 3> yg_imu = {gyro_init.x, gyro_init.y, gyro_init.z};
            std::array<float, 3> yg_bdy;
            preprocessor->rotate_imu_to_bdy(yg_imu, yg_bdy);
            preprocessor->yg_[0] = yg_bdy[0];
            preprocessor->yg_[1] = yg_bdy[1];
            preprocessor->yg_[2] = yg_bdy[2];
            
            std::array<float, 3> ya_imu = {accel_init.x, accel_init.y, accel_init.z};
            std::array<float, 3> ya_bdy;
            preprocessor->rotate_imu_to_bdy(ya_imu, ya_bdy);
            preprocessor->ya_[0] = ya_bdy[0];
            preprocessor->ya_[1] = ya_bdy[1];
            preprocessor->ya_[2] = ya_bdy[2];
            
            std::array<float, 3> ym_imu = {mag_init.x, mag_init.y, mag_init.z};
            std::array<float, 3> ym_bdy;
            preprocessor->rotate_imu_to_bdy(ym_imu, ym_bdy);
            preprocessor->ym_[0] = ym_bdy[0];
            preprocessor->ym_[1] = ym_bdy[1];
            preprocessor->ym_[2] = ym_bdy[2];
            
            std::cout << "✓ First sensor data received from driver\n";
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    while (preprocessor->running_.load())
    {
        // get gyro, accel, mag measurements with calibration applied
        bool have_gyro = preprocessor->get_gyro_measurement(gx, gy, gz);
        bool have_accel = preprocessor->get_accel_measurement(ax, ay, az);
        bool have_mag = preprocessor->get_mag_measurement(mx, my, mz);

        // Only publish fresh gyro data
        if (have_gyro)
        {
            messages::proc_gyro_msg_t gyro_msg{timestamp, gx, gy, gz};
            if (!preprocessor->nng_proc_gyro_pub_->send(gyro_msg))
            {
                std::cerr << "ERROR: Gyro channel closed\n"
                          << std::flush;
                break;
            }
            gyro_sent++;
            if (gyro_first_sample)
            {
                std::cout << "Preprocessor: First gyro sent: " << gyro_msg.x << ", " << gyro_msg.y << ", " << gyro_msg.z << "\n"
                          << std::flush;
                gyro_first_sample = false;
            }
        }

        // Only publish fresh accel data
        if (have_accel)
        {
            messages::proc_accel_msg_t accel_msg{timestamp, ax, ay, az};
            if (!preprocessor->nng_proc_accel_pub_->send(accel_msg))
            {
                std::cerr << "ERROR: Accel channel closed\n"
                          << std::flush;
                break;
            }
            accel_sent++;
            if (accel_first_sample)
            {
                std::cout << "Preprocessor: First accel sent: " << accel_msg.x << ", " << accel_msg.y << ", " << accel_msg.z << "\n"
                          << std::flush;
                accel_first_sample = false;
            }
        }

        // Only publish fresh mag data
        if (have_mag)
        {
            messages::proc_mag_msg_t mag_msg{timestamp, mx, my, mz};
            if (!preprocessor->nng_proc_mag_pub_->send(mag_msg))
            {
                std::cerr << "ERROR: Mag channel closed\n"
                          << std::flush;
                break;
            }
            mag_sent++;
            if (mag_first_sample)
            {
                std::cout << "Preprocessor: First mag sent: " << mag_msg.x << ", " << mag_msg.y << ", " << mag_msg.z << "\n"
                          << std::flush;
                mag_first_sample = false;
            }
        }

        // Sleep 10ms for ~100Hz sampling during bias estimation, slower after
        int sleep_ms = preprocessor->stationary_gyro_cal_ ? 10 : 100;
        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
    }
}