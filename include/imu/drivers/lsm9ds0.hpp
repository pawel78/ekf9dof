#pragma once
#include <cstdint>
// LSM9DS0 Register Map
// Reference: STMicroelectronics LSM9DS0 datasheet

namespace lsm9ds0 {

// I2C addresses (7-bit, shifted left by 1 for 8-bit addressing if needed)
constexpr uint8_t XM_ADDR = 0x1D; // Accelerometer/Magnetometer
constexpr uint8_t G_ADDR  = 0x6B; // Gyroscope

// Gyroscope Registers
constexpr uint8_t WHO_AM_I_G        = 0x0F;
constexpr uint8_t CTRL_REG1_G       = 0x20;
constexpr uint8_t CTRL_REG2_G       = 0x21;
constexpr uint8_t CTRL_REG3_G       = 0x22;
constexpr uint8_t CTRL_REG4_G       = 0x23;
constexpr uint8_t CTRL_REG5_G       = 0x24;
constexpr uint8_t REFERENCE_G       = 0x25;
constexpr uint8_t OUT_TEMP_G        = 0x26;
constexpr uint8_t STATUS_REG_G      = 0x27;
constexpr uint8_t OUT_X_L_G         = 0x28;
constexpr uint8_t OUT_X_H_G         = 0x29;
constexpr uint8_t OUT_Y_L_G         = 0x2A;
constexpr uint8_t OUT_Y_H_G         = 0x2B;
constexpr uint8_t OUT_Z_L_G         = 0x2C;
constexpr uint8_t OUT_Z_H_G         = 0x2D;
constexpr uint8_t FIFO_CTRL_REG_G   = 0x2E;
constexpr uint8_t FIFO_SRC_REG_G    = 0x2F;
constexpr uint8_t INT1_CFG_G        = 0x30;
constexpr uint8_t INT1_SRC_G        = 0x31;
constexpr uint8_t INT1_THS_XH_G     = 0x32;
constexpr uint8_t INT1_THS_XL_G     = 0x33;
constexpr uint8_t INT1_THS_YH_G     = 0x34;
constexpr uint8_t INT1_THS_YL_G     = 0x35;
constexpr uint8_t INT1_THS_ZH_G     = 0x36;
constexpr uint8_t INT1_THS_ZL_G     = 0x37;
constexpr uint8_t INT1_DURATION_G   = 0x38;

// Accelerometer/Magnetometer Registers
constexpr uint8_t WHO_AM_I_XM       = 0x0F;
constexpr uint8_t INT_CTRL_REG_M    = 0x12;
constexpr uint8_t OUT_TEMP_L_XM     = 0x05;
constexpr uint8_t OUT_TEMP_H_XM     = 0x06;
constexpr uint8_t STATUS_REG_M      = 0x07;
constexpr uint8_t OUT_X_L_M         = 0x08;
constexpr uint8_t OUT_X_H_M         = 0x09;
constexpr uint8_t OUT_Y_L_M         = 0x0A;
constexpr uint8_t OUT_Y_H_M         = 0x0B;
constexpr uint8_t OUT_Z_L_M         = 0x0C;
constexpr uint8_t OUT_Z_H_M         = 0x0D;
constexpr uint8_t CTRL_REG0_XM      = 0x1F;
constexpr uint8_t CTRL_REG1_XM      = 0x20;
constexpr uint8_t CTRL_REG2_XM      = 0x21;
constexpr uint8_t CTRL_REG3_XM      = 0x22;
constexpr uint8_t CTRL_REG4_XM      = 0x23;
constexpr uint8_t CTRL_REG5_XM      = 0x24;
constexpr uint8_t CTRL_REG6_XM      = 0x25;
constexpr uint8_t CTRL_REG7_XM      = 0x26;
constexpr uint8_t STATUS_REG_A      = 0x27;
constexpr uint8_t OUT_X_L_A         = 0x28;
constexpr uint8_t OUT_X_H_A         = 0x29;
constexpr uint8_t OUT_Y_L_A         = 0x2A;
constexpr uint8_t OUT_Y_H_A         = 0x2B;
constexpr uint8_t OUT_Z_L_A         = 0x2C;
constexpr uint8_t OUT_Z_H_A         = 0x2D;
constexpr uint8_t FIFO_CTRL_REG     = 0x2E;
constexpr uint8_t FIFO_SRC_REG      = 0x2F;
constexpr uint8_t INT_GEN_1_REG     = 0x30;
constexpr uint8_t INT_GEN_1_SRC     = 0x31;
constexpr uint8_t INT_GEN_1_THS     = 0x32;
constexpr uint8_t INT_GEN_1_DURATION= 0x33;
constexpr uint8_t INT_GEN_2_REG     = 0x34;
constexpr uint8_t INT_GEN_2_SRC     = 0x35;
constexpr uint8_t INT_GEN_2_THS     = 0x36;
constexpr uint8_t INT_GEN_2_DURATION= 0x37;
constexpr uint8_t CLICK_CFG         = 0x38;
constexpr uint8_t CLICK_SRC         = 0x39;
constexpr uint8_t CLICK_THS         = 0x3A;
constexpr uint8_t TIME_LIMIT        = 0x3B;
constexpr uint8_t TIME_LATENCY      = 0x3C;
constexpr uint8_t TIME_WINDOW       = 0x3D;
constexpr uint8_t ACT_THS           = 0x3E;
constexpr uint8_t ACT_DUR           = 0x3F;

} // namespace lsm9ds0
