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

private:
    std::atomic<bool> running_;
    std::thread ahrs_thread_;
    
    // NNG subscriber sockets for IMU data
    std::unique_ptr<NngRawGyroSocket> nng_gyro_sub_;
    std::unique_ptr<NngRawAccelSocket> nng_accel_sub_;
    std::unique_ptr<NngRawMagSocket> nng_mag_sub_;
    std::unique_ptr<NngRawTempSocket> nng_temp_sub_;
    
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
    std::array<double, 3> bdy_rpy_nav_{0.0, 0.0, 0.0}; // Roll, pitch, yaw

};  