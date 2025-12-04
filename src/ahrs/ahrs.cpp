#include <thread>
#include <atomic>
#include <memory>
#include "common/quat.hpp"
#include "common/nng_socket.hpp"
#include "imu/messages/nng_imu_sockets.hpp"
#include "ahrs/ahrs.hpp"
#include <iostream>

Ahrs::Ahrs()
    : running_(false)
{
    // Constructor implementation (if any)
}
Ahrs::~Ahrs()
{
    stop();
    // Close NNG channels
    if (nng_proc_gyro_sub_)
        nng_proc_gyro_sub_->close();
    if (nng_proc_accel_sub_)
        nng_proc_accel_sub_->close();
    if (nng_proc_mag_sub_)
        nng_proc_mag_sub_->close();
    if (nng_proc_temp_sub_)
        nng_proc_temp_sub_->close();
}
void Ahrs::start()
{
    if (running_.load())
    {
        std::cerr << "WARNING: AHRS already running\n";
        return;
    }

    // Initialize NNG subscriber sockets (do this in start() to ensure publishers are ready)
    std::cout << "Initializing NNG subscriber sockets...\n";
    nng_proc_gyro_sub_ = std::make_unique<NngProcGyroSocket>(nng_urls::PROC_GYRO, false);
    nng_proc_accel_sub_ = std::make_unique<NngProcAccelSocket>(nng_urls::PROC_ACCEL, false);
    nng_proc_mag_sub_ = std::make_unique<NngProcMagSocket>(nng_urls::PROC_MAG, false);
    nng_proc_temp_sub_ = std::make_unique<NngProcTempSocket>(nng_urls::PROC_TEMP, false);
    std::cout << "✓ NNG subscriber sockets initialized\n";

    std::cout << "Starting AHRS processing thread...\n";
    running_.store(true);

    // Spawn AHRS thread using static member function
    ahrs_thread_ = std::thread(ahrs_thread_func, this);

    std::cout << "✓ AHRS thread started\n";
}
void Ahrs::stop()
{
    if (!running_.load())
    {
        return; // Already stopped
    }

    std::cout << "Stopping AHRS...\n";

    // Signal thread to stop
    running_.store(false);

    // Wait for thread to finish
    if (ahrs_thread_.joinable())
    {
        ahrs_thread_.join();
    }

    std::cout << "✓ AHRS stopped\n";
}
bool Ahrs::is_running() const
{
    return running_.load();
}
void Ahrs::ahrs_thread_func(Ahrs *ahrs)
{
    std::cout << "AHRS thread running.\n";
    messages::proc_gyro_msg_t gyro{0, 0, 0, 0};
    messages::proc_accel_msg_t accel{0, 0, 0, 0};
    messages::proc_mag_msg_t mag{0, 0, 0, 0};

    int no_gyro_count = 0;
    int no_accel_count = 0;
    int no_mag_count = 0;
    const int MAX_NO_DATA_BEFORE_WARN = 10; // Warn every 100 iterations (~1 second)

    while (ahrs->running_.load())
    {
        bool have_data = false;
        
        // Try to read gyro from NNG socket
        if (ahrs->nng_proc_gyro_sub_->try_receive(gyro))
        {
            ahrs->imu_omega_imu_nav_[0] = gyro.x;
            ahrs->imu_omega_imu_nav_[1] = gyro.y;
            ahrs->imu_omega_imu_nav_[2] = gyro.z;
            no_gyro_count = 0;
            have_data = true;
        }
        else
        {
            no_gyro_count++;
            if (no_gyro_count >= MAX_NO_DATA_BEFORE_WARN)
            {
                std::cout << "Warning: No gyro data received (waiting for preprocessor...)\n";
                no_gyro_count = 0;
            }
        }

        // Try to read accel from NNG socket (don't block if not available)
        if (ahrs->nng_proc_accel_sub_->try_receive(accel))
        {
            ahrs->imu_accel_[0] = accel.x;
            ahrs->imu_accel_[1] = accel.y;
            ahrs->imu_accel_[2] = accel.z;
            no_accel_count = 0;
            have_data = true;
        }
        else
        {
            no_accel_count++;
            if (no_accel_count >= MAX_NO_DATA_BEFORE_WARN)
            {
                std::cout << "Warning: No accel data received (waiting for preprocessor...)\n";
                no_accel_count = 0;
            }
        }

        // Try to read mag from NNG socket (don't block if not available)
        if (ahrs->nng_proc_mag_sub_->try_receive(mag))
        {
            ahrs->imu_mag_[0] = mag.x;
            ahrs->imu_mag_[1] = mag.y;
            ahrs->imu_mag_[2] = mag.z;
            no_mag_count = 0;
            have_data = true;
        }
        else
        {
            no_mag_count++;
            if (no_mag_count >= MAX_NO_DATA_BEFORE_WARN)
            {
                std::cout << "Warning: No mag data received (waiting for preprocessor...)\n";
                no_mag_count = 0;
            }
        }
        
        // If no data at all this iteration, sleep and continue
        if (!have_data)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        // Perform stationary initial alignment if needed
        if (ahrs->is_stationary_ && !ahrs->stationary_alignment_done_)
        {
            ahrs->stationary_alignment_done_ = ahrs->stationary_initial_alignment();
        }
        else if (!ahrs->stationary_alignment_done_)
        {
            // Skip processing until stationary alignment is done
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        // TODO: AHRS update logic here

        // Sleep to control update rate (~100Hz)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "AHRS thread exiting.\n";
}
void Ahrs::state_init(std::array<double, 3> &m_avg, std::array<double, 3> &g_avg)
{
    // Initialize AHRS state from averaged sensor data during stationary alignment
    // Build rotation matrix to express body frame in navigation (NED) frame
    
    // NOTE: Accelerometer measures specific force (reaction to gravity), which points UP at rest
    // We need gravity vector pointing DOWN for NED frame, so negate the accelerometer reading
    std::array<double, 3> gravity = {-g_avg[0], -g_avg[1], -g_avg[2]};
    
    double g_norm = std::sqrt(gravity[0] * gravity[0] + gravity[1] * gravity[1] + gravity[2] * gravity[2]);
    double m_norm = std::sqrt(m_avg[0] * m_avg[0] + m_avg[1] * m_avg[1] + m_avg[2] * m_avg[2]);
    
    // Check for zero vectors
    if (g_norm < 1e-6 || m_norm < 1e-6) {
        std::cerr << "ERROR: Zero vector in state_init! g_norm=" << g_norm << " m_norm=" << m_norm << "\n";
        return;
    }
    
    // Normalize sensor measurements
    std::array<double, 3> g_unit = {gravity[0] / g_norm, gravity[1] / g_norm, gravity[2] / g_norm};
    std::array<double, 3> m_unit = {m_avg[0] / m_norm, m_avg[1] / m_norm, m_avg[2] / m_norm};
    
    // Build rotation matrix: bdy_R_nav (body frame w.r.t. navigation NED frame)
    // For NED: Down = gravity direction, East = (Down × mag_horizontal), North = (East × Down)
    std::array<std::array<double, 3>, 3> bdy_R_nav;
    
    // Column 2 (Down): Aligned with gravity
    bdy_R_nav[0][2] = g_unit[0];
    bdy_R_nav[1][2] = g_unit[1];
    bdy_R_nav[2][2] = g_unit[2];
    
    // Column 1 (East): Perpendicular to Down and magnetic field (Down × mag)
    std::array<double, 3> east;
    east[0] = g_unit[1] * m_unit[2] - g_unit[2] * m_unit[1];
    east[1] = g_unit[2] * m_unit[0] - g_unit[0] * m_unit[2];
    east[2] = g_unit[0] * m_unit[1] - g_unit[1] * m_unit[0];
    
    double east_norm = std::sqrt(east[0] * east[0] + east[1] * east[1] + east[2] * east[2]);
    
    if (east_norm < 1e-6) {
        std::cerr << "ERROR: Gravity and mag are parallel in state_init!\n";
        return;
    }
    
    bdy_R_nav[0][1] = east[0] / east_norm;
    bdy_R_nav[1][1] = east[1] / east_norm;
    bdy_R_nav[2][1] = east[2] / east_norm;
    
    // Column 0 (North): Complete right-handed system (East × Down)
    bdy_R_nav[0][0] = bdy_R_nav[1][1] * bdy_R_nav[2][2] - bdy_R_nav[2][1] * bdy_R_nav[1][2];
    bdy_R_nav[1][0] = bdy_R_nav[2][1] * bdy_R_nav[0][2] - bdy_R_nav[0][1] * bdy_R_nav[2][2];
    bdy_R_nav[2][0] = bdy_R_nav[0][1] * bdy_R_nav[1][2] - bdy_R_nav[1][1] * bdy_R_nav[0][2];

    // Convert rotation matrix to quaternion
    bdy_q_nav_ = Quat::from_rotation_matrix(bdy_R_nav);
       
    // For debugging: show roll/pitch from accel only
    std::array<double, 3> rp_accel;
    rp_accel[0] = std::atan2(gravity[1], gravity[2]);
    rp_accel[1] = std::atan2(-gravity[0], std::sqrt(gravity[1] * gravity[1] + gravity[2] * gravity[2]));
    std::cout << "Accel-only RPY (deg): Roll: " << rp_accel[0] * 180.0 / M_PI
              << ", Pitch: " << rp_accel[1] * 180.0 / M_PI << "\n";

    // Compute magnetic heading in local level frame for debugging
    // Rotate mag to level frame (remove tilt)
    Quat i_q_bdy = Quat::from_euler(rp_accel);
    std::array<double, 3> m_level = i_q_bdy * m_avg;
    double mag_heading = std::atan2(m_level[1], m_level[0]); // East, North components
    std::cout << "Magnetic heading (before declination): " << mag_heading * 180.0 / M_PI << " deg\n";
    
}
std::array<double, 3> Ahrs::extract_rpy() const
{
    return bdy_q_nav_.to_euler();
}

bool Ahrs::stationary_initial_alignment()
{
    // Estimate average accelerometer and magnetometer readings
    if (stationary_alignment_done_)
    {
        std::cout << "Stationary initial alignment already done.\n";
        return true; // Already done
    }
    
    // Make these static so they persist across function calls
    static std::array<double, 3> ya_sum = {0.0, 0.0, 0.0};
    static std::array<double, 3> ym_sum = {0.0, 0.0, 0.0};
    static std::array<double, 3> yg_sum = {0.0, 0.0, 0.0};
    static size_t n = 0;
    const size_t NUM_SAMPLES = 100;

    if (n < NUM_SAMPLES)
    {
        for (size_t i = 0; i < 3; ++i)
        {
            ya_sum[i] += imu_accel_[i];
            ym_sum[i] += imu_mag_[i];
            yg_sum[i] += imu_omega_imu_nav_[i];
        }
        ++n;
        
        if (n % 20 == 0) {
            std::cout << "Collecting alignment samples: " << n << "/" << NUM_SAMPLES << "\n";
        }
    }
    else
    {
        // Compute averages
        for (size_t i = 0; i < 3; ++i)
        {
            imu_g_avg_[i] = ya_sum[i] / NUM_SAMPLES;
            imu_m_avg_[i] = ym_sum[i] / NUM_SAMPLES;
            imu_yg_b_[i] = yg_sum[i] / NUM_SAMPLES;
        }
        
        std::cout << "Accel avg: [" << imu_g_avg_[0] << ", " << imu_g_avg_[1] << ", " << imu_g_avg_[2] << "]\n";
        std::cout << "Mag avg: [" << imu_m_avg_[0] << ", " << imu_m_avg_[1] << ", " << imu_m_avg_[2] << "]\n";

        // Use averaged sensor data to compute initial orientation
        state_init(imu_m_avg_, imu_g_avg_);
        std::array<double, 3> rpy = extract_rpy();
        std::cout << "Initial alignment RPY (deg): Roll: " << rpy[0] * 180.0 / M_PI
                  << ", Pitch: " << rpy[1] * 180.0 / M_PI
                  << ", Yaw: " << rpy[2] * 180.0 / M_PI << "\n";
        std::cout << "✓ Stationary initial alignment complete\n";
        return true;
    }
    return false;
}
