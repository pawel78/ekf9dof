#pragma once

#include "common/nng_socket.hpp"
#include "imu/messages/imu_data.hpp"

// NNG Socket type aliases
using NngRawAccelSocket = NngSocket<messages::raw_accel_msg_t>;
using NngRawGyroSocket = NngSocket<messages::raw_gyro_msg_t>;
using NngRawMagSocket = NngSocket<messages::raw_mag_msg_t>;
using NngRawTempSocket = NngSocket<messages::raw_temp_msg_t>;

using NngProcAccelSocket = NngSocket<messages::proc_accel_msg_t>;
using NngProcGyroSocket = NngSocket<messages::proc_gyro_msg_t>;
using NngProcMagSocket = NngSocket<messages::proc_mag_msg_t>;
using NngProcTempSocket = NngSocket<messages::proc_temp_msg_t>;

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