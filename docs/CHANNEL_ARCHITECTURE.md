# Channel Architecture Design

## Overview

The system uses **independent channels per sensor type** for maximum flexibility:

- **3 Raw Channels**: Accelerometer, Gyroscope, Magnetometer (uncalibrated SI units)
- **3 Processed Channels**: Accelerometer, Gyroscope, Magnetometer (calibrated SI units)
- **Separate threads per stage**: Driver → Processing → Consumers

### Terminology

- **Raw** = Uncalibrated SI units from driver (e.g., g, rad/s, gauss)
- **Processed** = Calibrated SI units after correction (e.g., bias, ellipsoid)

### Channel Types Location

All channel types and message definitions are in the **`imu` namespace**:
- Include: `#include "imu/messages/imu_data.hpp"`
- Channel types: `imu::RawGyroChannel`, `imu::ProcAccelChannel`, etc.
- Message types: `imu::messages::raw_gyro_msg_t`, etc.

**Consumer code only needs:**
```cpp
#include "imu/messages/imu_data.hpp"  // Gets messages + channel types

void my_consumer(imu::RawGyroChannel& gyro_channel) {
    imu::messages::raw_gyro_msg_t msg;
    if (gyro_channel.try_receive(msg)) {
        // Use data - no driver knowledge needed!
    }
}
```

### Driver Responsibilities

The driver **encapsulates all hardware details**:
- ✅ Timestamps sensor readings (closest to hardware)
- ✅ Applies datasheet conversion formulas (int16_t → float SI units)
- ✅ Provides hardware abstraction layer

## Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        LSM9DS0 Driver Thread                             │
│                           (200 Hz main loop)                             │
│                                                                          │
│  ┌────────────────────────────────────────────────────────────────────┐ │
│  │  while (running) {                                                 │ │
│  │    timestamp = get_time();                                         │ │
│  │                                                                    │ │
│  │    // Read accelerometer (int16_t from I2C)                       │ │
│  │    int16_t ax_raw, ay_raw, az_raw;                                │ │
│  │    if (read_accel(ax_raw, ay_raw, az_raw)) {                      │ │
│  │      // Convert to g using datasheet formula                      │ │
│  │      float ax = ax_raw * 0.000061f;  // ±2g scale                │ │
│  │      float ay = ay_raw * 0.000061f;                               │ │
│  │      float az = az_raw * 0.000061f;                               │ │
│  │      raw_accel_msg_t msg{timestamp, ax, ay, az};                 │ │
│  │      raw_accel_chan.send(msg);                                    │ │
│  │    }                                                               │ │
│  │                                                                    │ │
│  │    // Read gyroscope (int16_t from I2C)                           │ │
│  │    int16_t gx_raw, gy_raw, gz_raw;                                │ │
│  │    if (read_gyro(gx_raw, gy_raw, gz_raw)) {                       │ │
│  │      // Convert to rad/s using datasheet formula                  │ │
│  │      float gx_dps = gx_raw * 0.00875f;  // ±245 dps scale        │ │
│  │      float gx = gx_dps * (M_PI / 180.0f);  // → rad/s            │ │
│  │      float gy = gy_raw * 0.00875f * (M_PI / 180.0f);             │ │
│  │      float gz = gz_raw * 0.00875f * (M_PI / 180.0f);             │ │
│  │      raw_gyro_msg_t msg{timestamp, gx, gy, gz};                  │ │
│  │      raw_gyro_chan.send(msg);                                     │ │
│  │    }                                                               │ │
│  │                                                                    │ │
│  │    // Read magnetometer (int16_t from I2C)                        │ │
│  │    int16_t mx_raw, my_raw, mz_raw;                                │ │
│  │    if (read_mag(mx_raw, my_raw, mz_raw)) {                        │ │
│  │      // Convert to gauss using datasheet formula                  │ │
│  │      float mx = mx_raw * 0.00008f;  // ±2 gauss scale            │ │
│  │      float my = my_raw * 0.00008f;                                │ │
│  │      float mz = mz_raw * 0.00008f;                                │ │
│  │      raw_mag_msg_t msg{timestamp, mx, my, mz};                   │ │
│  │      raw_mag_chan.send(msg);                                      │ │
│  │    }                                                               │ │
│  │                                                                    │ │
│  │    sleep(5ms);  // 200 Hz                                         │ │
│  │  }                                                                 │ │
│  └────────────────────────────────────────────────────────────────────┘ │
└───────────────┬────────────┬────────────┬────────────────────────────────┘
                │            │            │
                │            │            │
        ┌───────▼─────┐ ┌───▼──────┐ ┌──▼───────┐
        │RawAccelChan │ │RawGyroChan│ │RawMagChan│
        │             │ │           │ │          │
        │ float x,y,z │ │float x,y,z│ │float x,y,z│
        │   (g, raw)  │ │ (rad/s)   │ │ (gauss)  │
        │ timestamp   │ │timestamp  │ │timestamp │
        └───────┬─────┘ └────┬──────┘ └────┬─────┘
                │            │              │
                │            │              │
┌───────────────▼────────────▼──────────────▼───────────────────────────┐
│                    IMU Processing Thread                               │
│                                                                        │
│  ┌──────────────────────────────────────────────────────────────────┐ │
│  │  while (running) {                                               │ │
│  │    // Receive uncalibrated data (already in SI units)           │ │
│  │                                                                  │ │
│  │    if (auto accel = raw_accel_chan.receive()) {                 │ │
│  │      // Apply ellipsoid calibration                             │ │
│  │      proc_accel_msg_t proc = calibrate_accel(*accel);           │ │
│  │      proc_accel_chan.send(proc);                                │ │
│  │    }                                                             │ │
│  │                                                                  │ │
│  │    if (auto gyro = raw_gyro_chan.receive()) {                   │ │
│  │      // Apply bias correction                                   │ │
│  │      proc_gyro_msg_t proc = calibrate_gyro(*gyro);              │ │
│  │      proc_gyro_chan.send(proc);                                 │ │
│  │    }                                                             │ │
│  │                                                                  │ │
│  │    if (auto mag = raw_mag_chan.receive()) {                     │ │
│  │      // Apply hard/soft iron calibration                        │ │
│  │      proc_mag_msg_t proc = calibrate_mag(*mag);                 │ │
│  │      proc_mag_chan.send(proc);                                  │ │
│  │    }                                                             │ │
│  │  }                                                               │ │
│  └──────────────────────────────────────────────────────────────────┘ │
└────────────────┬────────────┬────────────┬───────────────────────────┘
                 │            │            │
         ┌───────▼─────┐ ┌───▼──────┐ ┌──▼───────┐
         │ProcAccelChan│ │ProcGyroChan│ │ProcMagChan│
         │             │ │            │ │           │
         │float x,y,z  │ │float x,y,z │ │float x,y,z│
         │(g, calib'd) │ │(rad/s,cal) │ │(gauss,cal)│
         │ timestamp   │ │ timestamp  │ │ timestamp │
         └───────┬─────┘ └────┬───────┘ └────┬──────┘
                 │            │               │
                 └────────────▼───────────────┘
                              │
                    ┌─────────▼──────────┐
                    │   EKF / Fusion     │
                    │      Thread        │
                    └────────────────────┘
```

## Layered Abstraction

```
┌─────────────────────────────────────────────────────────────┐
│                    Hardware Layer                            │
│  - I2C registers contain int16_t values                      │
│  - Datasheet provides conversion formulas                    │
│  - Example: ±2g scale = 0.000061 g/LSB                       │
└──────────────────────┬──────────────────────────────────────┘
                       │ Driver reads & converts
                       ▼
┌─────────────────────────────────────────────────────────────┐
│                  Raw Messages (Driver)                       │
│  - float values in SI units (g, rad/s, gauss)                │
│  - Uncalibrated (factory sensor characteristics)             │
│  - Hardware-agnostic (could swap LSM9DS0 for other sensor)   │
└──────────────────────┬──────────────────────────────────────┘
                       │ Processing applies calibration
                       ▼
┌─────────────────────────────────────────────────────────────┐
│              Processed Messages (Processing)                 │
│  - float values in SI units (g, rad/s, gauss)                │
│  - Calibrated (bias, ellipsoid corrections applied)          │
│  - Ready for sensor fusion / algorithms                      │
└─────────────────────────────────────────────────────────────┘
```

## Message Structures

### Raw Messages (Driver → Processing)

```cpp
struct raw_accel_msg_t {
    uint64_t timestamp_ns;  // Nanosecond timestamp from driver
    float x, y, z;          // Uncalibrated, in g units
};

struct raw_gyro_msg_t {
    uint64_t timestamp_ns;
    float x, y, z;          // Uncalibrated, in rad/s
};

struct raw_mag_msg_t {
    uint64_t timestamp_ns;
    float x, y, z;          // Uncalibrated, in gauss
};
```

**Size:** 20 bytes each  
**Frequency:** 200 Hz (or sensor-dependent)  
**Producer:** Driver thread (includes datasheet conversion)  
**Consumer:** Processing thread

### Processed Messages (Processing → EKF/Logger)

```cpp
struct proc_accel_msg_t {
    uint64_t timestamp_ns;  // Original timestamp preserved
    float x, y, z;          // Calibrated in g units
};

struct proc_gyro_msg_t {
    uint64_t timestamp_ns;
    float x, y, z;          // Calibrated in rad/s
};

struct proc_mag_msg_t {
    uint64_t timestamp_ns;
    float x, y, z;          // Calibrated in gauss
};
```

**Size:** 20 bytes each  
**Frequency:** 200 Hz (matches input rate)  
**Producer:** Processing thread  
**Consumer:** EKF thread, logger, etc.

## Processing Pipeline Per Sensor

### Accelerometer Processing

```cpp
proc_accel_msg_t process_accel(const raw_accel_msg_t& raw) {
    // 1. Convert to engineering units (g)
    float ax = raw.x * ACCEL_SCALE;  // 0.000061 for ±2g
    float ay = raw.y * ACCEL_SCALE;
    float az = raw.z * ACCEL_SCALE;
    
    // 2. Apply calibration (hard iron + soft iron)
    float ax_centered = ax - accel_bias_[0];
    float ay_centered = ay - accel_bias_[1];
    float az_centered = az - accel_bias_[2];
    
    float ax_cal = accel_matrix_[0] * ax_centered +
                   accel_matrix_[1] * ay_centered +
                   accel_matrix_[2] * az_centered;
    float ay_cal = accel_matrix_[3] * ax_centered +
                   accel_matrix_[4] * ay_centered +
                   accel_matrix_[5] * az_centered;
    float az_cal = accel_matrix_[6] * ax_centered +
                   accel_matrix_[7] * ay_centered +
                   accel_matrix_[8] * az_centered;
    
    return {raw.timestamp_ns, ax_cal, ay_cal, az_cal};
}
```

### Gyroscope Processing

```cpp
proc_gyro_msg_t process_gyro(const raw_gyro_msg_t& raw) {
    // 1. Convert to dps
    float gx_dps = raw.x * GYRO_SCALE;  // 0.00875 for ±245 dps
    float gy_dps = raw.y * GYRO_SCALE;
    float gz_dps = raw.z * GYRO_SCALE;
    
    // 2. Convert to rad/s
    const float DEG_TO_RAD = M_PI / 180.0;
    float gx = gx_dps * DEG_TO_RAD;
    float gy = gy_dps * DEG_TO_RAD;
    float gz = gz_dps * DEG_TO_RAD;
    
    // 3. Apply calibration (bias only for gyro)
    float gx_cal = gx - gyro_bias_[0];
    float gy_cal = gy - gyro_bias_[1];
    float gz_cal = gz - gyro_bias_[2];
    
    return {raw.timestamp_ns, gx_cal, gy_cal, gz_cal};
}
```

### Magnetometer Processing

```cpp
proc_mag_msg_t process_mag(const raw_mag_msg_t& raw) {
    // 1. Convert to gauss
    float mx = raw.x * MAG_SCALE;  // 0.00008 for ±2 gauss
    float my = raw.y * MAG_SCALE;
    float mz = raw.z * MAG_SCALE;
    
    // 2. Apply calibration (hard iron + soft iron)
    float mx_centered = mx - mag_bias_[0];
    float my_centered = my - mag_bias_[1];
    float mz_centered = mz - mag_bias_[2];
    
    float mx_cal = mag_matrix_[0] * mx_centered +
                   mag_matrix_[1] * my_centered +
                   mag_matrix_[2] * mz_centered;
    float my_cal = mag_matrix_[3] * mx_centered +
                   mag_matrix_[4] * my_centered +
                   mag_matrix_[5] * mz_centered;
    float mz_cal = mag_matrix_[6] * mx_centered +
                   mag_matrix_[7] * my_centered +
                   mag_matrix_[8] * mz_centered;
    
    return {raw.timestamp_ns, mx_cal, my_cal, mz_cal};
}
```

## Channel Usage Patterns

### Driver Thread (Producer)

```cpp
void lsm9ds0_driver_thread(
    imu::RawAccelChannel& raw_accel_chan,
    imu::RawGyroChannel& raw_gyro_chan,
    imu::RawMagChannel& raw_mag_chan,
    std::atomic<bool>& running)
{
    while (running) {
        uint64_t timestamp = get_timestamp_ns();
        
        // Read and publish accelerometer
        int16_t ax, ay, az;
        if (read_accel(ax, ay, az)) {
            raw_accel_chan.send({timestamp, ax, ay, az});
        }
        
        // Read and publish gyroscope
        int16_t gx, gy, gz;
        if (read_gyro(gx, gy, gz)) {
            raw_gyro_chan.send({timestamp, gx, gy, gz});
        }
        
        // Read and publish magnetometer
        int16_t mx, my, mz;
        if (read_mag(mx, my, mz)) {
            raw_mag_chan.send({timestamp, mx, my, mz});
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(5));  // 200 Hz
    }
}
```

### Processing Thread (Consumer → Producer)

```cpp
void imu_processing_thread(
    imu::RawAccelChannel& raw_accel_chan,
    imu::RawGyroChannel& raw_gyro_chan,
    imu::RawMagChannel& raw_mag_chan,
    imu::ProcAccelChannel& proc_accel_chan,
    imu::ProcGyroChannel& proc_gyro_chan,
    imu::ProcMagChannel& proc_mag_chan,
    std::atomic<bool>& running)
{
    // Load calibration parameters
    load_calibration();
    
    while (running) {
        bool processed_any = false;
        
        // Process accelerometer (non-blocking)
        auto accel_opt = raw_accel_chan.receive();
        if (accel_opt) {
            proc_accel_msg_t proc = process_accel(*accel_opt);
            proc_accel_chan.send(proc);
            processed_any = true;
        }
        
        // Process gyroscope (non-blocking)
        auto gyro_opt = raw_gyro_chan.receive();
        if (gyro_opt) {
            proc_gyro_msg_t proc = process_gyro(*gyro_opt);
            proc_gyro_chan.send(proc);
            processed_any = true;
        }
        
        // Process magnetometer (non-blocking)
        auto mag_opt = raw_mag_chan.receive();
        if (mag_opt) {
            proc_mag_msg_t proc = process_mag(*mag_opt);
            proc_mag_chan.send(proc);
            processed_any = true;
        }
        
        // If all channels empty and closed, exit
        if (!processed_any && 
            raw_accel_chan.is_closed() && 
            raw_gyro_chan.is_closed() && 
            raw_mag_chan.is_closed()) {
            break;
        }
        
        // Small sleep to avoid busy-wait when no data
        if (!processed_any) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}
```

### Consumer Thread (EKF/Fusion)

```cpp
### EKF Thread (Consumer)

```cpp
void ekf_thread(
    imu::ProcAccelChannel& accel_chan,
    imu::ProcGyroChannel& gyro_chan,
    imu::ProcMagChannel& mag_chan)
{
    while (true) {
        // Can handle async sensor updates
        // EKF decides which sensor to process based on timestamps
        
        auto accel = accel_chan.try_receive();
        if (accel) {
            ekf.update_accel(accel->timestamp_ns, 
                           accel->x, accel->y, accel->z);
        }
        
        auto gyro = gyro_chan.try_receive();
        if (gyro) {
            ekf.propagate(gyro->timestamp_ns,
                        gyro->x, gyro->y, gyro->z);
        }
        
        auto mag = mag_chan.try_receive();
        if (mag) {
            ekf.update_mag(mag->timestamp_ns,
                         mag->x, mag->y, mag->z);
        }
        
        // ... fusion logic ...
    }
}
```

## Main Thread Coordination

```cpp
int main() {
    // Create channels in imu namespace
    imu::RawAccelChannel raw_accel_chan;
    imu::RawGyroChannel raw_gyro_chan;
    imu::RawMagChannel raw_mag_chan;
    
    imu::ProcAccelChannel proc_accel_chan;
    imu::ProcGyroChannel proc_gyro_chan;
    imu::ProcMagChannel proc_mag_chan;
    
    std::atomic<bool> running{true};
    
    // Spawn threads
    std::thread driver(lsm9ds0_driver_thread,
                      std::ref(raw_accel_chan),
                      std::ref(raw_gyro_chan),
                      std::ref(raw_mag_chan),
                      std::ref(running));
    
    std::thread processor(imu_processing_thread,
                         std::ref(raw_accel_chan),
                         std::ref(raw_gyro_chan),
                         std::ref(raw_mag_chan),
                         std::ref(proc_accel_chan),
                         std::ref(proc_gyro_chan),
                         std::ref(proc_mag_chan),
                         std::ref(running));
    
    // ... run application ...
    
    // Graceful shutdown
    running = false;
    driver.join();
    
    raw_accel_chan.close();
    raw_gyro_chan.close();
    raw_mag_chan.close();
    processor.join();
    
    proc_accel_chan.close();
    proc_gyro_chan.close();
    proc_mag_chan.close();
    
    return 0;
}
```

## Future Extensibility Examples

### Different Sampling Rates

```cpp
// Example: High-rate gyro (400 Hz), normal accel (200 Hz), low-rate mag (50 Hz)
void multi_rate_driver_thread() {
    while (running) {
        // Gyro: every 2.5ms (400 Hz)
        if (should_sample_gyro()) {
            read_and_send_gyro();
        }
        
        // Accel: every 5ms (200 Hz)
        if (should_sample_accel()) {
            read_and_send_accel();
        }
        
        // Mag: every 20ms (50 Hz)
        if (should_sample_mag()) {
            read_and_send_mag();
        }
        
        std::this_thread::sleep_for(std::chrono::microseconds(2500));
    }
}
```

### Multiple Sensor Sources

```cpp
// Example: Mix LSM9DS0 accel + external high-end gyro
void hybrid_driver_threads() {
    // Thread 1: LSM9DS0 accel + mag
    std::thread lsm9ds0([&]() {
        while (running) {
            read_and_send_accel();  // LSM9DS0
            read_and_send_mag();    // LSM9DS0
            sleep(5ms);
        }
    });
    
    // Thread 2: High-end gyro
    std::thread gyro([&]() {
        while (running) {
            read_and_send_gyro();  // Different sensor!
            sleep(2ms);  // Higher rate
        }
    });
}
```

### Sensor Hot-Swap

```cpp
// Can change sensor source without recompiling
if (config.use_external_mag) {
    // External magnetometer driver
    std::thread mag_driver(external_mag_thread, std::ref(raw_mag_chan));
} else {
    // Built-in LSM9DS0 magnetometer
    std::thread mag_driver(lsm9ds0_mag_thread, std::ref(raw_mag_chan));
}
// Processing thread doesn't care which sensor!
```

## Performance Characteristics

**Memory per channel:**
- Raw message: 14 bytes
- Processed message: 20 bytes
- Queue node overhead: ~48 bytes
- Total: ~82 bytes per message

**At 200 Hz with 100-deep queue:**
- Memory per channel pair (raw + proc): ~16.4 KB
- Total for 3 sensors: ~49 KB
- Throughput per sensor: ~6.8 KB/s

**Latency:**
- Driver → Processing: <1ms (queue overhead)
- Processing time: <0.1ms per sample
- End-to-end: <2ms typical

## Next Steps

1. ✅ Channel architecture finalized
2. ⬜ Implement driver thread
3. ⬜ Implement processing thread
4. ⬜ Update main.cpp
5. ⬜ Add calibration loading
6. ⬜ Add monitoring/logging

## Message Flow

```
┌─────────────────────────────────────────────────────────────────────┐
│                          Driver Thread (200 Hz)                      │
│  ┌────────────────────────────────────────────────────────────────┐ │
│  │ 1. Read I2C Sensors                                            │ │
│  │    - read_accel()  → int16_t ax, ay, az                       │ │
│  │    - read_gyro()   → int16_t gx, gy, gz                       │ │
│  │    - read_mag()    → int16_t mx, my, mz                       │ │
│  │    - read_temp()   → int16_t temp                             │ │
│  │                                                                │ │
│  │ 2. Create raw_imu_msg_t                                        │ │
│  │    - timestamp_ns = current_time()                            │ │
│  │    - Copy all int16_t values                                  │ │
│  │                                                                │ │
│  │ 3. Send to channel                                             │ │
│  │    - raw_imu_channel.send(msg)                                │ │
│  └────────────────────────────────────────────────────────────────┘ │
└────────────────────────────┬────────────────────────────────────────┘
                              │
                              ▼
              ┌───────────────────────────────┐
              │      RawImuChannel            │
              │  Channel<raw_imu_msg_t>       │
              │  - Thread-safe queue          │
              │  - Blocking receive()         │
              │  - Supports close()           │
              └───────────────┬───────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────────┐
│                       Processing Thread                              │
│  ┌────────────────────────────────────────────────────────────────┐ │
│  │ 1. Receive raw data                                            │ │
│  │    - auto msg = raw_imu_channel.receive()                     │ │
│  │    - if (!msg) break; // Channel closed                       │ │
│  │                                                                │ │
│  │ 2. Convert to engineering units                                │ │
│  │    - accel: raw * 0.000061 → g                                │ │
│  │    - gyro:  raw * 0.00875  → dps → rad/s                      │ │
│  │    - mag:   raw * 0.00008  → gauss                            │ │
│  │    - temp:  raw / 8 + 25   → °C                               │ │
│  │                                                                │ │
│  │ 3. Apply calibration                                           │ │
│  │    - Load bias & matrix from config                           │ │
│  │    - Apply hard iron (bias subtraction)                       │ │
│  │    - Apply soft iron (matrix multiplication)                  │ │
│  │                                                                │ │
│  │ 4. Create proc_imu_msg_t                                       │ │
│  │    - timestamp_ns (copied from raw)                           │ │
│  │    - Copy all float values                                    │ │
│  │                                                                │ │
│  │ 5. Send to processed channel                                   │ │
│  │    - proc_imu_channel.send(msg)                               │ │
│  └────────────────────────────────────────────────────────────────┘ │
└────────────────────────────┬────────────────────────────────────────┘
                              │
                              ▼
              ┌───────────────────────────────┐
              │     ProcImuChannel            │
              │  Channel<proc_imu_msg_t>      │
              │  - Ready for EKF/fusion       │
              │  - Calibrated SI units        │
              └───────────────────────────────┘
```

## Message Structures

### raw_imu_msg_t (Driver → Processing)

```cpp
struct raw_imu_msg_t {
    uint64_t timestamp_ns;      // Nanosecond timestamp
    
    // Raw sensor values (int16_t from hardware)
    int16_t accel_x, accel_y, accel_z;  // Accelerometer
    int16_t gyro_x, gyro_y, gyro_z;      // Gyroscope
    int16_t mag_x, mag_y, mag_z;         // Magnetometer
    int16_t temp;                        // Temperature
};
```

**Size:** ~28 bytes  
**Frequency:** 200 Hz  
**Producer:** `lsm9ds0_driver_thread()`  
**Consumer:** `imu_processing_thread()`

### proc_imu_msg_t (Processing → EKF/Logger)

```cpp
struct proc_imu_msg_t {
    uint64_t timestamp_ns;      // Nanosecond timestamp (copied)
    
    // Calibrated values in SI units (float)
    float accel_x, accel_y, accel_z;  // g
    float gyro_x, gyro_y, gyro_z;      // rad/s
    float mag_x, mag_y, mag_z;         // gauss
    float temp_c;                      // °C
};
```

**Size:** ~44 bytes  
**Frequency:** 200 Hz  
**Producer:** `imu_processing_thread()`  
**Consumer:** EKF thread, logger thread, etc.

## Channel Implementation

### Features

```cpp
template <typename T>
class Channel {
    bool send(const T& data);           // Returns false if closed
    std::optional<T> receive();         // Blocking, returns nullopt if closed
    bool try_receive(T& data);          // Non-blocking
    void close();                       // Signal end of data
    bool is_closed() const;
    size_t size() const;                // Queue depth (monitoring)
};
```

### Thread Safety

- **Mutex-protected queue** - All operations are thread-safe
- **Condition variable** - Efficient blocking on `receive()`
- **Close semantics** - Clean shutdown support
- **No busy-waiting** - Threads block until data available

### Typical Usage

**Producer (Driver Thread):**
```cpp
void driver_thread(imu::RawImuChannel& raw_channel) {
    while (running) {
        raw_imu_msg_t msg = read_sensors();
        if (!raw_channel.send(msg)) {
            break; // Channel closed
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}
```

**Consumer (Processing Thread):**
```cpp
void processing_thread(imu::RawImuChannel& raw_channel,
                      imu::ProcImuChannel& proc_channel) {
    while (true) {
        auto msg_opt = raw_channel.receive();
        if (!msg_opt) {
            break; // Channel closed and empty
        }
        
        proc_imu_msg_t proc_msg = process(*msg_opt);
        proc_channel.send(proc_msg);
    }
}
```

**Main Thread (Coordination):**
```cpp
int main() {
    imu::RawImuChannel raw_channel;
    imu::ProcImuChannel proc_channel;
    
    std::thread driver(driver_thread, std::ref(raw_channel));
    std::thread processor(processing_thread, 
                         std::ref(raw_channel), 
                         std::ref(proc_channel));
    
    // ... run for some time ...
    
    // Graceful shutdown
    raw_channel.close();
    driver.join();
    processor.join();  // Will exit when raw_channel is closed and empty
    proc_channel.close();
}
```

## Design Decisions

### Why Combined Messages?

✅ **Atomic sampling** - All sensors read at same time  
✅ **Timestamp consistency** - One timestamp for all sensors  
✅ **Simpler code** - One send/receive per sample  
✅ **Better for fusion** - Algorithms need synchronized data  

❌ Alternative (separate channels per sensor):
- 3x send/receive operations
- Synchronization complexity
- Timestamp alignment issues

### Why int16_t in Raw Messages?

✅ **Hardware native** - Direct from I2C registers  
✅ **Zero conversion cost** in driver  
✅ **Smaller messages** - 2 bytes vs 4 bytes per value  
✅ **Processing isolation** - Calibration in dedicated thread  

### Why std::optional<T> for receive()?

✅ **Clean shutdown** - Distinguishes "closed" from "data"  
✅ **Modern C++17** - Standard way to represent optional values  
✅ **Type safe** - No special sentinel values needed  

### Timestamp Generation

**Driver thread generates timestamp** when reading I2C:
- Most accurate representation of sample time
- Processing thread preserves timestamp
- EKF uses original sample timestamp

## Channel Types Reference

```cpp
// Primary (recommended)
using RawImuChannel = Channel<raw_imu_msg_t>;
using ProcImuChannel = Channel<proc_imu_msg_t>;

// Legacy (deprecated - kept for compatibility)
using RawAccelChannel = Channel<raw_accel_msg_t>;
using RawGyroChannel = Channel<raw_gyro_msg_t>;
using RawMagChannel = Channel<raw_mag_msg_t>;
using ProcAccelChannel = Channel<proc_accel_msg_t>;
using ProcGyroChannel = Channel<proc_gyro_msg_t>;
using ProcMagChannel = Channel<proc_mag_msg_t>;
```

## Performance Characteristics

**Memory:**
- Raw message: ~28 bytes
- Processed message: ~44 bytes
- Queue overhead: ~48 bytes per message (std::queue node)
- Total per sample: ~120 bytes

**At 200 Hz with 100 message queue:**
- Memory: ~12 KB per channel
- Latency: Max 500ms buffering (100 msgs / 200 Hz)
- Throughput: ~17 KB/s per channel

**Typical queue depth:**
- Under normal load: 0-5 messages
- Processing faster than sampling: usually empty
- Backpressure: grows if processing falls behind

## Next Steps

1. ✅ Channel architecture designed
2. ⬜ Implement driver thread function
3. ⬜ Implement processing thread function
4. ⬜ Update main.cpp to spawn threads
5. ⬜ Add calibration loading
6. ⬜ Add logging/monitoring
