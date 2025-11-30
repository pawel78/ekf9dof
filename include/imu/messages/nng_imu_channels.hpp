#pragma once

#include "common/nng_channel.hpp"
#include "imu/messages/imu_data.hpp"
#include "imu_messages.pb.h"

namespace channel {

// Template specializations for converting between C++ structs and protobuf messages

// ============================================================================
// Raw Accelerometer
// ============================================================================
template <>
inline void NngChannel<imu::messages::raw_accel_msg_t, imu::proto::RawAccelMsg>::to_proto(
    const imu::messages::raw_accel_msg_t& src, imu::proto::RawAccelMsg& dst) {
    dst.set_timestamp_ns(src.timestamp_ns);
    dst.set_x(src.x);
    dst.set_y(src.y);
    dst.set_z(src.z);
}

template <>
inline void NngChannel<imu::messages::raw_accel_msg_t, imu::proto::RawAccelMsg>::from_proto(
    const imu::proto::RawAccelMsg& src, imu::messages::raw_accel_msg_t& dst) {
    dst.timestamp_ns = src.timestamp_ns();
    dst.x = src.x();
    dst.y = src.y();
    dst.z = src.z();
}

// ============================================================================
// Raw Gyroscope
// ============================================================================
template <>
inline void NngChannel<imu::messages::raw_gyro_msg_t, imu::proto::RawGyroMsg>::to_proto(
    const imu::messages::raw_gyro_msg_t& src, imu::proto::RawGyroMsg& dst) {
    dst.set_timestamp_ns(src.timestamp_ns);
    dst.set_x(src.x);
    dst.set_y(src.y);
    dst.set_z(src.z);
}

template <>
inline void NngChannel<imu::messages::raw_gyro_msg_t, imu::proto::RawGyroMsg>::from_proto(
    const imu::proto::RawGyroMsg& src, imu::messages::raw_gyro_msg_t& dst) {
    dst.timestamp_ns = src.timestamp_ns();
    dst.x = src.x();
    dst.y = src.y();
    dst.z = src.z();
}

// ============================================================================
// Raw Magnetometer
// ============================================================================
template <>
inline void NngChannel<imu::messages::raw_mag_msg_t, imu::proto::RawMagMsg>::to_proto(
    const imu::messages::raw_mag_msg_t& src, imu::proto::RawMagMsg& dst) {
    dst.set_timestamp_ns(src.timestamp_ns);
    dst.set_x(src.x);
    dst.set_y(src.y);
    dst.set_z(src.z);
}

template <>
inline void NngChannel<imu::messages::raw_mag_msg_t, imu::proto::RawMagMsg>::from_proto(
    const imu::proto::RawMagMsg& src, imu::messages::raw_mag_msg_t& dst) {
    dst.timestamp_ns = src.timestamp_ns();
    dst.x = src.x();
    dst.y = src.y();
    dst.z = src.z();
}

// ============================================================================
// Raw Temperature
// ============================================================================
template <>
inline void NngChannel<imu::messages::raw_temp_msg_t, imu::proto::RawTempMsg>::to_proto(
    const imu::messages::raw_temp_msg_t& src, imu::proto::RawTempMsg& dst) {
    dst.set_timestamp_ns(src.timestamp_ns);
    dst.set_temp_c(src.temp_c);
}

template <>
inline void NngChannel<imu::messages::raw_temp_msg_t, imu::proto::RawTempMsg>::from_proto(
    const imu::proto::RawTempMsg& src, imu::messages::raw_temp_msg_t& dst) {
    dst.timestamp_ns = src.timestamp_ns();
    dst.temp_c = src.temp_c();
}

// ============================================================================
// Processed Accelerometer
// ============================================================================
template <>
inline void NngChannel<imu::messages::proc_accel_msg_t, imu::proto::ProcAccelMsg>::to_proto(
    const imu::messages::proc_accel_msg_t& src, imu::proto::ProcAccelMsg& dst) {
    dst.set_timestamp_ns(src.timestamp_ns);
    dst.set_x(src.x);
    dst.set_y(src.y);
    dst.set_z(src.z);
}

template <>
inline void NngChannel<imu::messages::proc_accel_msg_t, imu::proto::ProcAccelMsg>::from_proto(
    const imu::proto::ProcAccelMsg& src, imu::messages::proc_accel_msg_t& dst) {
    dst.timestamp_ns = src.timestamp_ns();
    dst.x = src.x();
    dst.y = src.y();
    dst.z = src.z();
}

// ============================================================================
// Processed Gyroscope
// ============================================================================
template <>
inline void NngChannel<imu::messages::proc_gyro_msg_t, imu::proto::ProcGyroMsg>::to_proto(
    const imu::messages::proc_gyro_msg_t& src, imu::proto::ProcGyroMsg& dst) {
    dst.set_timestamp_ns(src.timestamp_ns);
    dst.set_x(src.x);
    dst.set_y(src.y);
    dst.set_z(src.z);
}

template <>
inline void NngChannel<imu::messages::proc_gyro_msg_t, imu::proto::ProcGyroMsg>::from_proto(
    const imu::proto::ProcGyroMsg& src, imu::messages::proc_gyro_msg_t& dst) {
    dst.timestamp_ns = src.timestamp_ns();
    dst.x = src.x();
    dst.y = src.y();
    dst.z = src.z();
}

// ============================================================================
// Processed Magnetometer
// ============================================================================
template <>
inline void NngChannel<imu::messages::proc_mag_msg_t, imu::proto::ProcMagMsg>::to_proto(
    const imu::messages::proc_mag_msg_t& src, imu::proto::ProcMagMsg& dst) {
    dst.set_timestamp_ns(src.timestamp_ns);
    dst.set_x(src.x);
    dst.set_y(src.y);
    dst.set_z(src.z);
}

template <>
inline void NngChannel<imu::messages::proc_mag_msg_t, imu::proto::ProcMagMsg>::from_proto(
    const imu::proto::ProcMagMsg& src, imu::messages::proc_mag_msg_t& dst) {
    dst.timestamp_ns = src.timestamp_ns();
    dst.x = src.x();
    dst.y = src.y();
    dst.z = src.z();
}

// ============================================================================
// Processed Temperature
// ============================================================================
template <>
inline void NngChannel<imu::messages::proc_temp_msg_t, imu::proto::ProcTempMsg>::to_proto(
    const imu::messages::proc_temp_msg_t& src, imu::proto::ProcTempMsg& dst) {
    dst.set_timestamp_ns(src.timestamp_ns);
    dst.set_temp_c(src.temp_c);
}

template <>
inline void NngChannel<imu::messages::proc_temp_msg_t, imu::proto::ProcTempMsg>::from_proto(
    const imu::proto::ProcTempMsg& src, imu::messages::proc_temp_msg_t& dst) {
    dst.timestamp_ns = src.timestamp_ns();
    dst.temp_c = src.temp_c();
}

} // namespace channel

namespace imu {

// NNG Channel type aliases
using NngRawAccelChannel = channel::NngChannel<messages::raw_accel_msg_t, proto::RawAccelMsg>;
using NngRawGyroChannel = channel::NngChannel<messages::raw_gyro_msg_t, proto::RawGyroMsg>;
using NngRawMagChannel = channel::NngChannel<messages::raw_mag_msg_t, proto::RawMagMsg>;
using NngRawTempChannel = channel::NngChannel<messages::raw_temp_msg_t, proto::RawTempMsg>;

using NngProcAccelChannel = channel::NngChannel<messages::proc_accel_msg_t, proto::ProcAccelMsg>;
using NngProcGyroChannel = channel::NngChannel<messages::proc_gyro_msg_t, proto::ProcGyroMsg>;
using NngProcMagChannel = channel::NngChannel<messages::proc_mag_msg_t, proto::ProcMagMsg>;
using NngProcTempChannel = channel::NngChannel<messages::proc_temp_msg_t, proto::ProcTempMsg>;

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
