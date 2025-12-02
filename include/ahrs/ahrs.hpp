#pragma once
#include <thread>
#include <atomic>
#include <memory>
#include "common/quat.hpp"    
#include "common/nng_socket.hpp"
#include "imu/messages/nng_imu_sockets.hpp"

class Ahrs {
public:
    /**
     * @brief Construct and initialize the AHRS
     * 
     * @throws std::runtime_error if initialization fails
     */
    Ahrs();
    /**
     * @brief Destructor
     */
    ~Ahrs();
    // Prevent copying
    Ahrs(const Ahrs&) = delete;
    Ahrs& operator=(const Ahrs&) = delete;

    /**
     * @brief Start the AHRS processing thread
     */
    void start();
    /**
     * @brief Stop the AHRS processing thread
     */
    void stop();
    /**
     * @brief Check if the AHRS is running
     * 
     * @return true if running, false otherwise
     */
    bool is_running() const;
    /**
     * @brief Extract roll, pitch, yaw from current orientation
     * 
     * @return Array of [roll, pitch, yaw] in radians
     */
    std::array<double, 3> extract_rpy() const;

private:
    std::atomic<bool> running_;
    std::thread ahrs_thread_;
    
    // NNG subscriber sockets for IMU data
    std::unique_ptr<NngProcGyroSocket> nng_proc_gyro_sub_;
    std::unique_ptr<NngProcAccelSocket> nng_proc_accel_sub_;
    std::unique_ptr<NngProcMagSocket> nng_proc_mag_sub_;
    std::unique_ptr<NngProcTempSocket> nng_proc_temp_sub_;

    std::array<double, 3> imu_accel_{0.0, 0.0, 0.0};
    std::array<double, 3> imu_omega_imu_nav_{0.0, 0.0, 0.0};
    std::array<double, 3> imu_mag_{0.0, 0.0, 0.0};
    double imu_temp_{0.0};

    // AHRS state initialization and processing methods
    void state_init(std::array<double, 3> &m_avg, std::array<double, 3> &g_avg);

    // AHRS algorithm methods for stationary initial alignment
    bool is_stationary_{true};                          // assume stationary at startup
    bool state_initialized_{false};                     // state init flag
    bool stationary_alignment_done_{false};             // stationary alignment done flag
    std::array<double, 3> imu_g_avg_{0.0, 0.0, 0.0};    // avg accel in imu frame during stationary
    std::array<double, 3> imu_m_avg_{0.0, 0.0, 0.0};    // avg mag in imu frame during stationary
    std::array<double, 3> imu_yg_b_{0.0, 0.0, 0.0};     // avg gyro in imu frame during stationary
    bool stationary_initial_alignment();
    
    /** 
    * @brief AHRS processing thread function
    *
    * @param ahrs Pointer to the Ahrs instance
    * @note This function is static and requires the Ahrs instance to operate on.
    *  It is passed the 'this' pointer when the thread is created.
    */
    static void ahrs_thread_func(Ahrs* ahrs);

    Quat nav_q_bdy_{0,0,0,1};  // body to navigation quaternion
    Quat bdy_q_nav_{0,0,0,1};  // navigation to body quaternion
    Quat bdy_q_imu_{0,0,0,1};  // IMU to body quaternion
    Quat imu_q_nav_{0,0,0,1};    // IMU to navigation quaternion
    std::array<double, 3> bdy_rpy_nav_{0.0, 0.0, 0.0}; // Roll, pitch, yaw

};  