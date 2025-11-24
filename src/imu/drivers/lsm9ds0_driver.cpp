#include <cstdint>
#include <stdexcept>
#include <system_error>
#include <cstring>
#include <array>
#include <chrono>
#include <thread>
#include <atomic>
#include <cmath>
#include <iostream>
#include "imu/drivers/lsm9ds0_driver.hpp"
#include "common/channel_types.hpp"
#include "imu/messages/imu_data.hpp"

// ============================================================================
// LSM9DS0Driver I2C operations (private methods)
// ============================================================================

void LSM9DS0Driver::i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t value)
{
    i2c_device_->write(dev_addr, reg_addr, value);
}

uint8_t LSM9DS0Driver::i2c_read_reg(uint8_t dev_addr, uint8_t reg_addr)
{
    return i2c_device_->read_reg(dev_addr, reg_addr);
}

bool LSM9DS0Driver::i2c_read16_le(uint8_t dev_addr, uint8_t reg_low, int16_t &out)
{
    try
    {
        uint8_t lo = i2c_read_reg(dev_addr, reg_low);
        uint8_t hi = i2c_read_reg(dev_addr, reg_low + 1);
        out = static_cast<int16_t>(static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8));
        return true;
    }
    catch (const std::system_error &e)
    {
        return false;
    }
}

// ============================================================================
// Hardware verification and configuration (private methods)
// ============================================================================

bool LSM9DS0Driver::verify_device_ids()
{
    try
    {
        uint8_t whoami_g = i2c_read_reg(G_ADDR, WHO_AM_I_G);
        uint8_t whoami_xm = i2c_read_reg(XM_ADDR, WHO_AM_I_XM);
        return (whoami_g == 0b11010100) && (whoami_xm == 0b01001001);
    }
    catch (...)
    {
        return false;
    }
}

void LSM9DS0Driver::configure_imu()
{
    configure_gyroscope();
    configure_accelerometer();
    configure_magnetometer();
    configure_temperature_sensor();
}

void LSM9DS0Driver::configure_gyroscope()
{
    // CTRL_REG1_G: ODR=95Hz, Cutoff=12.5Hz, all axes enabled, normal mode
    i2c_write(G_ADDR, CTRL_REG1_G, 0b00001111);
    // CTRL_REG4_G: 245 dps full scale
    i2c_write(G_ADDR, CTRL_REG4_G, 0b00000000);
}

void LSM9DS0Driver::configure_accelerometer()
{
    // CTRL_REG1_XM: ODR=100Hz, all axes enabled
    i2c_write(XM_ADDR, CTRL_REG1_XM, 0b01100111);
    // CTRL_REG2_XM: ±2g full scale
    i2c_write(XM_ADDR, CTRL_REG2_XM, 0b00000000);
}

void LSM9DS0Driver::configure_magnetometer()
{
    // CTRL_REG5_XM: mag resolution=high, ODR=50Hz
    i2c_write(XM_ADDR, CTRL_REG5_XM, 0b01110000);
    // CTRL_REG6_XM: ±2 gauss full scale
    i2c_write(XM_ADDR, CTRL_REG6_XM, 0b00000000);
    // CTRL_REG7_XM: continuous conversion mode
    i2c_write(XM_ADDR, CTRL_REG7_XM, 0b00000000);
}

void LSM9DS0Driver::configure_temperature_sensor()
{
    // CTRL_REG5_XM: enable temperature sensor (bit 7) + mag settings
    i2c_write(XM_ADDR, CTRL_REG5_XM, 0b01110000 | 0b10000000);
}

// ============================================================================
// Sensor read operations (private methods)
// ============================================================================

bool LSM9DS0Driver::read_accel(int16_t &x, int16_t &y, int16_t &z)
{
    return i2c_read16_le(XM_ADDR, OUT_X_L_A, x) &&
           i2c_read16_le(XM_ADDR, OUT_Y_L_A, y) &&
           i2c_read16_le(XM_ADDR, OUT_Z_L_A, z);
}

bool LSM9DS0Driver::read_gyro(int16_t &x, int16_t &y, int16_t &z)
{
    return i2c_read16_le(G_ADDR, OUT_X_L_G, x) &&
           i2c_read16_le(G_ADDR, OUT_Y_L_G, y) &&
           i2c_read16_le(G_ADDR, OUT_Z_L_G, z);
}

bool LSM9DS0Driver::read_mag(int16_t &x, int16_t &y, int16_t &z)
{
    return i2c_read16_le(XM_ADDR, OUT_X_L_M, x) &&
           i2c_read16_le(XM_ADDR, OUT_Y_L_M, y) &&
           i2c_read16_le(XM_ADDR, OUT_Z_L_M, z);
}

bool LSM9DS0Driver::read_temperature(int16_t &temp)
{
    try
    {
        uint8_t lo = i2c_read_reg(XM_ADDR, OUT_TEMP_L_XM);
        uint8_t hi = i2c_read_reg(XM_ADDR, OUT_TEMP_H_XM);

        // Temperature is 12-bit, two's-complement, right-justified
        uint16_t raw = static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8);
        raw &= 0x0FFF; // Keep only lower 12 bits

        int16_t signed12;
        if (raw & 0x0800)
        {
            // Negative: set upper 4 bits to 1
            signed12 = static_cast<int16_t>(raw | 0xF000);
        }
        else
        {
            signed12 = static_cast<int16_t>(raw);
        }

        temp = signed12;
        return true;
    }
    catch (...)
    {
        return false;
    }
}

// ============================================================================
// Datasheet conversion formulas (private static methods)
// ============================================================================

float LSM9DS0Driver::raw_to_celsius(int16_t raw_temp)
{
    return (raw_temp / 8.0f) + 25.0f;
}

float LSM9DS0Driver::raw_to_dps(int16_t raw_gyro)
{
    return raw_gyro * 0.00875f;
}

float LSM9DS0Driver::raw_to_g(int16_t raw_accel)
{
    return raw_accel * 0.000061f;
}

float LSM9DS0Driver::raw_to_gauss(int16_t raw_mag)
{
    return raw_mag * 0.00008f;
}

// ============================================================================
// Timestamp helper (private static method)
// ============================================================================

uint64_t LSM9DS0Driver::get_timestamp_ns()
{
    auto now = std::chrono::steady_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
}

// ============================================================================
// Driver thread function (private static method)
// ============================================================================

void LSM9DS0Driver::driver_thread_func(LSM9DS0Driver *driver)
{
    constexpr auto sample_period = std::chrono::milliseconds(5);  // 200 Hz
    constexpr auto debug_period = std::chrono::milliseconds(500); // 2 Hz for debug output

    int error_count = 0;
    int sample_count = 0;
    int accel_sent = 0, gyro_sent = 0, mag_sent = 0, temp_sent = 0;
    bool first_sample = true;

    auto last_debug_time = std::chrono::steady_clock::now();

    // Variables to store latest sensor readings for debug output
    float latest_ax = 0, latest_ay = 0, latest_az = 0;
    float latest_gx = 0, latest_gy = 0, latest_gz = 0;
    float latest_mx = 0, latest_my = 0, latest_mz = 0;
    float latest_temp = 0;
    uint64_t latest_timestamp = 0;

    std::cout << "Driver thread: Starting sensor reads...\n"
              << std::flush;

    while (driver->running_.load())
    {
        // Capture timestamp as close to hardware read as possible
        uint64_t timestamp = get_timestamp_ns();

        bool all_ok = true;

        // Read gyroscope data every sample
        int16_t gx_raw, gy_raw, gz_raw;
        if (driver->read_gyro(gx_raw, gy_raw, gz_raw))
        {
            float gx_dps = raw_to_dps(gx_raw);
            float gy_dps = raw_to_dps(gy_raw);
            float gz_dps = raw_to_dps(gz_raw);
            
            constexpr float deg_to_rad = M_PI / 180.0f;
            float gx = gx_dps * deg_to_rad;
            float gy = gy_dps * deg_to_rad;
            float gz = gz_dps * deg_to_rad;

            if (driver->logging_enabled_.load()) {
                driver->write_gyro_log(get_timestamp_ns(), gx_dps, gy_dps, gz_dps);
            }

            // Store for debug output
            latest_gx = gx_dps;
            latest_gy = gy_dps;
            latest_gz = gz_dps;

            imu::messages::raw_gyro_msg_t gyro_msg{timestamp, gx, gy, gz};
            if (!imu::channels::raw_gyro.send(gyro_msg))
            {
                std::cerr << "ERROR: Gyro channel closed\n"
                          << std::flush;
                break;
            }
            gyro_sent++;
            if (first_sample)
            {
                std::cout << "Driver: First gyro sent: " << gx << ", " << gy << ", " << gz << "\n"
                          << std::flush;
            }
        }
        else
        {
            all_ok = false;
            if (first_sample || error_count < 5)
            {
                std::cerr << "ERROR: Failed to read gyroscope\n"
                          << std::flush;
            }
        }

        // Interleave reading accelerometer data
        switch (sample_count % 3)
        {
        case 0: // Even samples: read accelerometer
            int16_t ax_raw, ay_raw, az_raw;
            if (driver->read_accel(ax_raw, ay_raw, az_raw))
            {
                float ax = raw_to_g(ax_raw);
                float ay = raw_to_g(ay_raw);
                float az = raw_to_g(az_raw);

                // Store for debug output
                latest_ax = ax;
                latest_ay = ay;
                latest_az = az;
                latest_timestamp = timestamp;

                if (driver->logging_enabled_.load()) {
                    driver->write_accel_log(get_timestamp_ns(), ax, ay, az);
                }

                imu::messages::raw_accel_msg_t accel_msg{timestamp, ax, ay, az};
                if (!imu::channels::raw_accel.send(accel_msg))
                {
                    std::cerr << "ERROR: Accel channel closed\n"
                              << std::flush;
                    break;
                }
                accel_sent++;
                if (first_sample)
                {
                    std::cout << "Driver: First accel sent: " << ax << ", " << ay << ", " << az << "\n"
                              << std::flush;
                }
            }
            else
            {
                all_ok = false;
                if (first_sample || error_count < 5)
                {
                    std::cerr << "ERROR: Failed to read accelerometer\n"
                              << std::flush;
                }
            }
            break;
        case 1: // Odd samples mod 3 == 1: read magnetometer
            // Read magnetometer
            int16_t mx_raw, my_raw, mz_raw;

            if (driver->read_mag(mx_raw, my_raw, mz_raw))
            {
                float mx = raw_to_gauss(mx_raw);
                float my = raw_to_gauss(my_raw);
                float mz = raw_to_gauss(mz_raw);

                if (driver->logging_enabled_.load()) {
                    driver->write_mag_log(get_timestamp_ns(), mx, my, mz);
                }

                // Store for debug output
                latest_mx = mx;
                latest_my = my;
                latest_mz = mz;

                imu::messages::raw_mag_msg_t mag_msg{timestamp, mx, my, mz};
                if (!imu::channels::raw_mag.send(mag_msg))
                {
                    std::cerr << "ERROR: Mag channel closed\n"
                              << std::flush;
                    break;
                }
                mag_sent++;
                if (first_sample)
                {
                    std::cout << "Driver: First mag sent: " << mx << ", " << my << ", " << mz << "\n"
                              << std::flush;
                }
            }
            else
            {
                all_ok = false;
                if (first_sample || error_count < 5)
                {
                    std::cerr << "ERROR: Failed to read magnetometer\n"
                              << std::flush;
                }
            }
            break;
        case 2:
            // Read temperature
            int16_t temp_raw;
            if (driver->read_temperature(temp_raw))
            {
                float temp_c = raw_to_celsius(temp_raw);

                if (driver->logging_enabled_.load()) {
                    driver->write_temp_log(get_timestamp_ns(), temp_c);
                }

                // Store for debug output
                latest_temp = temp_c;

                imu::messages::raw_temp_msg_t temp_msg{timestamp, temp_c};
                if (!imu::channels::raw_temp.send(temp_msg))
                {
                    std::cerr << "ERROR: Temp channel closed\n"
                              << std::flush;
                    break;
                }
                temp_sent++;
                if (first_sample)
                {
                    std::cout << "Driver: First temp sent: " << temp_c << "°C\n"
                              << std::flush;
                }
            }
            else
            {
                all_ok = false;
                if (first_sample || error_count < 5)
                {
                    std::cerr << "ERROR: Failed to read temperature\n"
                              << std::flush;
                }
            }
            break;
        default:
            break;
        }
        if (!all_ok)
        {
            error_count++;
        }

        first_sample = false;
        sample_count++;

        // Debug output at slower rate (500ms intervals)
        if (driver->debug_output_enabled_.load())
        {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_debug_time);
            if (elapsed >= debug_period)
            {
                driver->print_debug_data(latest_timestamp,
                                         latest_ax, latest_ay, latest_az,
                                         latest_gx, latest_gy, latest_gz,
                                         latest_mx, latest_my, latest_mz,
                                         latest_temp);
                last_debug_time = now;
            }
        }

        // Binary logging at full rate (200 Hz)
        /*
        if (driver->logging_enabled_.load())
        {
            driver->write_binary_log(latest_timestamp,
                                     latest_ax, latest_ay, latest_az,
                                     latest_gx, latest_gy, latest_gz,
                                     latest_mx, latest_my, latest_mz,
                                     latest_temp);
        }
        */
        // Sleep to maintain 200 Hz rate
        std::this_thread::sleep_for(sample_period);
    }

    std::cout << "Driver thread: Exiting. Total samples: " << sample_count
              << ", errors: " << error_count << "\n"
              << std::flush;
}

#include "imu/drivers/lsm9ds0_driver.hpp"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <stdexcept>

LSM9DS0Driver::LSM9DS0Driver(const char *i2c_device_path)
    : running_(false), debug_output_enabled_(false), logging_enabled_(false), i2c_device_(std::make_unique<I2CDevice>(i2c_device_path))
{
    std::cout << "Initializing LSM9DS0 IMU...\n";

    // Verify device IDs
    if (!verify_device_ids())
    {
        throw std::runtime_error("Failed to verify LSM9DS0 device IDs");
    }
    std::cout << "✓ Device IDs verified\n";

    // Configure IMU
    configure_imu();
    std::cout << "✓ IMU configured\n";
}

LSM9DS0Driver::~LSM9DS0Driver()
{
    stop();
    if (log_file_gyro_.is_open()) log_file_gyro_.close();
    if (log_file_accel_.is_open()) log_file_accel_.close();
    if (log_file_mag_.is_open()) log_file_mag_.close();
    if (log_file_temp_.is_open()) log_file_temp_.close();
}

void LSM9DS0Driver::start()
{
    if (running_.load())
    {
        std::cerr << "WARNING: Driver already running\n";
        return;
    }

    std::cout << "Starting LSM9DS0 driver thread (200 Hz)...\n";
    running_.store(true);

    // Spawn driver thread using static member function
    driver_thread_ = std::thread(driver_thread_func, this);

    std::cout << "✓ Driver thread started\n";
}

void LSM9DS0Driver::stop()
{
    if (!running_.load())
    {
        return; // Already stopped
    }

    std::cout << "Stopping LSM9DS0 driver...\n";

    // Signal thread to stop
    running_.store(false);

    // Wait for thread to finish
    if (driver_thread_.joinable())
    {
        driver_thread_.join();
    }

    std::cout << "✓ Driver stopped\n";
}

void LSM9DS0Driver::print_debug_data(uint64_t timestamp_ns,
                                     float ax, float ay, float az,
                                     float gx, float gy, float gz,
                                     float mx, float my, float mz,
                                     float temp)
{
    // Convert nanoseconds to seconds with fractional part
    double timestamp_sec = static_cast<double>(timestamp_ns) / 1e9;

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "[" << timestamp_sec << "s] ";
    std::cout << "A(" << std::setw(6) << std::setprecision(3) << ax << ","
              << std::setw(6) << std::setprecision(3) << ay << ","
              << std::setw(6) << std::setprecision(3) << az << ") ";
    std::cout << "G(" << std::setw(7) << std::setprecision(2) << gx << ","
              << std::setw(7) << std::setprecision(2) << gy << ","
              << std::setw(7) << std::setprecision(2) << gz << ") ";
    std::cout << "M(" << std::setw(6) << std::setprecision(3) << mx << ","
              << std::setw(6) << std::setprecision(3) << my << ","
              << std::setw(6) << std::setprecision(3) << mz << ") ";
    std::cout << "T(" << std::setw(5) << std::setprecision(1) << temp << "°C)";
    std::cout << std::endl;
}

// ============================================================================
// Binary Logging
// ============================================================================

bool LSM9DS0Driver::set_data_logging(bool enable, const std::string& filename)
{
    std::lock_guard<std::mutex> lock(log_mutex_);
    
    if (enable)
    {
        if (!log_file_gyro_.is_open())
        {
            // Extract base name and create per-sensor files
            size_t dot_pos = filename.find_last_of('.');
            std::string base = (dot_pos == std::string::npos) ? filename : filename.substr(0, dot_pos);
            std::string ext = (dot_pos == std::string::npos) ? ".bin" : filename.substr(dot_pos);
            
            std::string gyro_file = base + "_gyro" + ext;
            std::string accel_file = base + "_accel" + ext;
            std::string mag_file = base + "_mag" + ext;
            std::string temp_file = base + "_temp" + ext;
            
            log_file_gyro_.open(gyro_file, std::ios::binary);
            log_file_accel_.open(accel_file, std::ios::binary);
            log_file_mag_.open(mag_file, std::ios::binary);
            log_file_temp_.open(temp_file, std::ios::binary);
            
            if (!log_file_gyro_.is_open() || !log_file_accel_.is_open() || 
                !log_file_mag_.is_open() || !log_file_temp_.is_open())
            {
                // Close any that opened
                if (log_file_gyro_.is_open()) log_file_gyro_.close();
                if (log_file_accel_.is_open()) log_file_accel_.close();
                if (log_file_mag_.is_open()) log_file_mag_.close();
                if (log_file_temp_.is_open()) log_file_temp_.close();
                logging_enabled_.store(false);
                return false;
            }
            
            // Write magic and version to each file
            uint32_t magic = 0x494D5532; // "IMU2"
            uint16_t version = 2;
            
            log_file_gyro_.write(reinterpret_cast<const char*>(&magic), sizeof(magic));
            log_file_gyro_.write(reinterpret_cast<const char*>(&version), sizeof(version));
            
            log_file_accel_.write(reinterpret_cast<const char*>(&magic), sizeof(magic));
            log_file_accel_.write(reinterpret_cast<const char*>(&version), sizeof(version));
            
            log_file_mag_.write(reinterpret_cast<const char*>(&magic), sizeof(magic));
            log_file_mag_.write(reinterpret_cast<const char*>(&version), sizeof(version));
            
            log_file_temp_.write(reinterpret_cast<const char*>(&magic), sizeof(magic));
            log_file_temp_.write(reinterpret_cast<const char*>(&version), sizeof(version));
            
            logging_enabled_.store(true);
            return true;
        }
        return true; // Already enabled
    }
    else
    {
        if (log_file_gyro_.is_open()) log_file_gyro_.close();
        if (log_file_accel_.is_open()) log_file_accel_.close();
        if (log_file_mag_.is_open()) log_file_mag_.close();
        if (log_file_temp_.is_open()) log_file_temp_.close();
        logging_enabled_.store(false);
        return true;
    }
}

// ============================================================================
// Per-Sensor Binary Logging (V2 Format)
// ============================================================================

void LSM9DS0Driver::write_gyro_log(uint64_t timestamp_ns, float gx, float gy, float gz)
{
    std::lock_guard<std::mutex> lock(log_mutex_);
    
    if (!log_file_gyro_.is_open())
    {
        return;
    }
    
    // No record type byte needed - file is gyro-specific
    log_file_gyro_.write(reinterpret_cast<const char *>(&timestamp_ns), sizeof(timestamp_ns));
    log_file_gyro_.write(reinterpret_cast<const char *>(&gx), sizeof(float));
    log_file_gyro_.write(reinterpret_cast<const char *>(&gy), sizeof(float));
    log_file_gyro_.write(reinterpret_cast<const char *>(&gz), sizeof(float));
}

void LSM9DS0Driver::write_accel_log(uint64_t timestamp_ns, float ax, float ay, float az)
{
    std::lock_guard<std::mutex> lock(log_mutex_);
    
    if (!log_file_accel_.is_open())
    {
        return;
    }
    
    // No record type byte needed - file is accel-specific
    log_file_accel_.write(reinterpret_cast<const char *>(&timestamp_ns), sizeof(timestamp_ns));
    log_file_accel_.write(reinterpret_cast<const char *>(&ax), sizeof(float));
    log_file_accel_.write(reinterpret_cast<const char *>(&ay), sizeof(float));
    log_file_accel_.write(reinterpret_cast<const char *>(&az), sizeof(float));
}

void LSM9DS0Driver::write_mag_log(uint64_t timestamp_ns, float mx, float my, float mz)
{
    std::lock_guard<std::mutex> lock(log_mutex_);
    
    if (!log_file_mag_.is_open())
    {
        return;
    }
    
    // No record type byte needed - file is mag-specific
    log_file_mag_.write(reinterpret_cast<const char *>(&timestamp_ns), sizeof(timestamp_ns));
    log_file_mag_.write(reinterpret_cast<const char *>(&mx), sizeof(float));
    log_file_mag_.write(reinterpret_cast<const char *>(&my), sizeof(float));
    log_file_mag_.write(reinterpret_cast<const char *>(&mz), sizeof(float));
}

void LSM9DS0Driver::write_temp_log(uint64_t timestamp_ns, float temp)
{
    std::lock_guard<std::mutex> lock(log_mutex_);
    
    if (!log_file_temp_.is_open())
    {
        return;
    }
    
    // No record type byte needed - file is temp-specific
    log_file_temp_.write(reinterpret_cast<const char *>(&timestamp_ns), sizeof(timestamp_ns));
    log_file_temp_.write(reinterpret_cast<const char *>(&temp), sizeof(float));
}