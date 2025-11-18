#pragma once

#include "imu/messages/imu_data.hpp"

/**
 * @brief Channel type aliases
 * 
 * For convenience, channel types are defined in imu namespace.
 * This file provides backward compatibility by re-exporting them in channels namespace.
 * 
 * Preferred usage: imu::RawGyroChannel
 * Legacy usage: channels::RawGyroChannel (forwards to imu::)
 */
namespace channels {
    // Raw sensor channels - forward to imu namespace
    using RawAccelChannel = imu::RawAccelChannel;
    using RawGyroChannel = imu::RawGyroChannel;
    using RawMagChannel = imu::RawMagChannel;
    using RawTempChannel = imu::RawTempChannel;
    
    // Processed sensor channels - forward to imu namespace
    using ProcAccelChannel = imu::ProcAccelChannel;
    using ProcGyroChannel = imu::ProcGyroChannel;
    using ProcMagChannel = imu::ProcMagChannel;
    using ProcTempChannel = imu::ProcTempChannel;
}