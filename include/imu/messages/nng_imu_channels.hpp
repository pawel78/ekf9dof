#pragma once

#include "common/nng_channel.hpp"
#include "imu/messages/imu_data.hpp"

namespace imu {

// NNG Channel type aliases using raw C++ struct serialization
using NngRawAccelChannel = channel::NngChannel<messages::raw_accel_msg_t>;
using NngRawGyroChannel = channel::NngChannel<messages::raw_gyro_msg_t>;
using NngRawMagChannel = channel::NngChannel<messages::raw_mag_msg_t>;
using NngRawTempChannel = channel::NngChannel<messages::raw_temp_msg_t>;

using NngProcAccelChannel = channel::NngChannel<messages::proc_accel_msg_t>;
using NngProcGyroChannel = channel::NngChannel<messages::proc_gyro_msg_t>;
using NngProcMagChannel = channel::NngChannel<messages::proc_mag_msg_t>;
using NngProcTempChannel = channel::NngChannel<messages::proc_temp_msg_t>;

/**
 * @brief IPC URL definitions for nng channels
 *
 * Default paths use /tmp which is standard for embedded Linux robotics platforms
 * like Jetson where filesystem access is typically restricted.
 *
 * Security note: For production deployments with multiple users, consider
 * overriding IMU_NNG_IPC_DIR to a more restricted directory (e.g., /run/imu).
 *
 * Override at compile time: cmake -DIMU_NNG_IPC_DIR=/run/imu ..
 */
namespace nng_urls {
#ifndef IMU_NNG_IPC_DIR
#define IMU_NNG_IPC_DIR "/tmp"
#endif
    constexpr const char* RAW_ACCEL = "ipc://" IMU_NNG_IPC_DIR "/imu_raw_accel.ipc";
    constexpr const char* RAW_GYRO = "ipc://" IMU_NNG_IPC_DIR "/imu_raw_gyro.ipc";
    constexpr const char* RAW_MAG = "ipc://" IMU_NNG_IPC_DIR "/imu_raw_mag.ipc";
    constexpr const char* RAW_TEMP = "ipc://" IMU_NNG_IPC_DIR "/imu_raw_temp.ipc";
    constexpr const char* PROC_ACCEL = "ipc://" IMU_NNG_IPC_DIR "/imu_proc_accel.ipc";
    constexpr const char* PROC_GYRO = "ipc://" IMU_NNG_IPC_DIR "/imu_proc_gyro.ipc";
    constexpr const char* PROC_MAG = "ipc://" IMU_NNG_IPC_DIR "/imu_proc_mag.ipc";
    constexpr const char* PROC_TEMP = "ipc://" IMU_NNG_IPC_DIR "/imu_proc_temp.ipc";
}

} // namespace imu
