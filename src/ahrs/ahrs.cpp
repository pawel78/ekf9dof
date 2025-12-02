#include <thread>
#include <atomic>
#include <memory>
#include "common/quat.hpp"
#include "common/nng_socket.hpp"
#include "imu/messages/nng_imu_sockets.hpp"
#include "ahrs/ahrs.hpp"
#include <iostream>

Ahrs::Ahrs()
    : running_(false),
      nng_proc_gyro_sub_(std::make_unique<NngProcGyroSocket>(nng_urls::PROC_GYRO, false)),
      nng_proc_accel_sub_(std::make_unique<NngProcAccelSocket>(nng_urls::PROC_ACCEL, false)),
      nng_proc_mag_sub_(std::make_unique<NngProcMagSocket>(nng_urls::PROC_MAG, false)),
      nng_proc_temp_sub_(std::make_unique<NngProcTempSocket>(nng_urls::PROC_TEMP, false))
{
    // Initialization code if needed
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

    while (ahrs->running_.load())
    {
        // AHRS processing logic goes here
        // For example, read from NNG sockets, compute orientation, etc.
            // Try to read gyro from NNG channel
        
        if (ahrs->nng_proc_gyro_sub_->try_receive(gyro)) {
            // Successfully received gyro data
            ahrs->imu_omega_imu_nav_[0] = gyro.x;
            ahrs->imu_omega_imu_nav_[1] = gyro.y;
            ahrs->imu_omega_imu_nav_[2] = gyro.z;
        } else {
            // No new gyro data available
            // Handle accordingly (e.g., skip processing or use last known value)
            std::cout << "No new gyro data available.\n";
            continue; // Skip this iteration
        }   
        
        if (ahrs->nng_proc_accel_sub_->try_receive(accel)) {
            // Successfully received accel data
            ahrs->imu_accel_[0] = accel.x;
            ahrs->imu_accel_[1] = accel.y;
            ahrs->imu_accel_[2] = accel.z;
        } else {
            // No new accel data available
            std::cout << "No new accel data available.\n";
            continue; // Skip this iteration
        }

        if (ahrs->nng_proc_mag_sub_->try_receive(mag)) {
            // Successfully received mag data
            ahrs->imu_mag_[0] = mag.x;
            ahrs->imu_mag_[1] = mag.y;
            ahrs->imu_mag_[2] = mag.z;
        } else {
            // No new mag data available
            std::cout << "No new mag data available.\n";
            continue; // Skip this iteration
        }
    
        // Perform stationary initial alignment if needed
        if (ahrs->is_stationary_ && !ahrs->stationary_alignment_done_)
        {
            ahrs->stationary_alignment_done_ = ahrs->stationary_initial_alignment();
        }
        else    
        {
            continue; // Skip this iteration, wait until stationary alignment is done
        }
        
        // Simulate some processing delay
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "AHRS thread exiting.\n";
}
void Ahrs::state_init(std::array<double, 3> &m_avg, std::array<double, 3> &g_avg)
{
    // Initialize AHRS state
    // Implementation of state initialization goes here
    // build rotation matrix from accelerometer and magnetometer averages
    std::array<std::array<double, 3>, 3> imu_R_nav;
    double g_norm = std::sqrt(g_avg[0]*g_avg[0] + g_avg[1]*g_avg[1] + g_avg[2]*g_avg[2]);
    double m_norm = std::sqrt(m_avg[0]*m_avg[0] + m_avg[1]*m_avg[1] + m_avg[2]*m_avg[2]);
    for (size_t i = 0; i < 3; ++i)
    {
        imu_R_nav[i][0] = g_avg[i] / g_norm; // normalize gravity vector
        imu_R_nav[i][2] = m_avg[i] / m_norm; // normalize mag vector
    }
    // cross product of gravity and mag (g_avg x m_avg) to get 2nd column
    std::array<double, 3> r;
    r[0] = g_avg[1]*m_avg[2] - g_avg[2]*m_avg[1];
    r[1] = g_avg[2]*m_avg[0] - g_avg[0]*m_avg[2];
    r[2] = g_avg[0]*m_avg[1] - g_avg[1]*m_avg[0];

    double r_norm = std::sqrt(r[0]*r[0] + r[1]*r[1] + r[2]*r[2]);
    for (size_t i = 0; i < 3; ++i)
    {
        imu_R_nav[i][1] = r[i] / r_norm; // normalize
    }

    // Convert rotation matrix to quaternion
    imu_q_nav_ = Quat::from_rotation_matrix(imu_R_nav);
    nav_q_bdy_ = bdy_q_imu_ * imu_q_nav_;
    bdy_q_nav_ = nav_q_bdy_.conjugate();
    
    // Extract Euler angles (roll, pitch, yaw) from quaternion
    bdy_rpy_nav_ = nav_q_bdy_.to_euler();

}
std::array<double, 3> Ahrs::extract_rpy() const {
    return nav_q_bdy_.to_euler();
}

bool Ahrs::stationary_initial_alignment()
{
    // Estimate avergage accelerometer and magnetometer readings
    if (stationary_alignment_done_)
    {
        std::cout << "Stationary initial alignment already done.\n";
        return true; // Already done
    }
    std::array<double, 3> ya_sum = {0.0, 0.0, 0.0};
    std::array<double, 3> ym_sum = {0.0, 0.0, 0.0};
    std::array<double, 3> yg_sum = {0.0, 0.0, 0.0};
    const size_t NUM_SAMPLES = 100;
    static size_t n = 0;

    if (n <= NUM_SAMPLES)
    {   
        for (size_t i = 0; i < 3; ++i)
        {
            ya_sum[i] += imu_accel_[i];
            ym_sum[i] += imu_mag_[i];
            yg_sum[i] += imu_omega_imu_nav_[i];
        }
        ++n;
    }
    else 
    {
        // Compute averages
        for (size_t i = 0; i < 3; ++i)
        {
            imu_g_avg_[i] =  ya_sum[i] / NUM_SAMPLES;
            imu_m_avg_[i] =  ym_sum[i] / NUM_SAMPLES;
            imu_yg_b_[i]  =  yg_sum[i] / NUM_SAMPLES;
        }

        // Use averaged sensor data to compute initial orientation
        state_init(imu_m_avg_, imu_g_avg_);
        std::array<double, 3> rpy = extract_rpy();
        std::cout << "Initial alignment RPY (deg): Roll: " << rpy[0] * 180.0/M_PI
                  << ", Pitch: " << rpy[1] * 180.0/M_PI
                  << ", Yaw: " << rpy[2] * 180.0/M_PI << "\n";   
        std::cout << "✓ Stationary initial alignment complete\n";
        return true;
    }
    return false;
}
