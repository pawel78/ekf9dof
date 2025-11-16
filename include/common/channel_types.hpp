#pragma once

#include "common/channel.hpp"
#include "imu/messages/imu_data.hpp"

namespace channels {
    // Raw sensor channels (float, uncalibrated SI units from driver)
    using RawAccelChannel = channel::Channel<imu::messages::raw_accel_msg_t>;
    using RawGyroChannel = channel::Channel<imu::messages::raw_gyro_msg_t>;
    using RawMagChannel = channel::Channel<imu::messages::raw_mag_msg_t>;
    using RawTempChannel = channel::Channel<imu::messages::raw_temp_msg_t>;
    
    // Processed sensor channels (float, calibrated SI units)
    using ProcAccelChannel = channel::Channel<imu::messages::proc_accel_msg_t>;
    using ProcGyroChannel = channel::Channel<imu::messages::proc_gyro_msg_t>;
    using ProcMagChannel = channel::Channel<imu::messages::proc_mag_msg_t>;
    using ProcTempChannel = channel::Channel<imu::messages::proc_temp_msg_t>;
} 
