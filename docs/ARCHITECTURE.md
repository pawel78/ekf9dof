# ARCHITECTURE.md

# IMU + GPS EKF System Architecture

This document describes the high-level architecture for a modular sensor-fusion system built around:

- **IMU drivers**  
- **GPS drivers**  
- **Message-based data flow**  
- **Sensor processing (calibration + unit conversion)**  
- **Real-time EKF estimator**  
- **Logging & replay**  
- **Threaded runtime pipeline**

It is intentionally simple, modular, and scalable to SLAM in the future.

---

## 1. Overview

The system centers around a set of **threads** that communicate using **typed messages** sent through **thread-safe channels**.

At runtime:

```
 ┌─────────────┐   ┌─────────────┐   ┌─────────────┐   ┌─────────────┐
 │ IMU Driver  │   │ GPS Driver  │   │AHRS/EKF/GNSS│   │ Logger / Viz│
 │   Thread    │   │   Thread    │   │   Thread    │   │   Thread(s) │
 └──────┬──────┘   └──────┬──────┘   └──────┬──────┘   └──────┬──────┘
        │                 │                 │                 │
        ▼                 ▼                 ▲                 ▲
   ImuChannel        GpsChannel        StateChannel      LogChannel
```

Each module is isolated and interacts **only** through message channels.

---

## 2. Module Breakdown

The project is divided into modules, each with its own namespace and directory.

```
imu/
  messages/
  drivers/
  processing/
  logging/

gps/
  messages/
  drivers/
  processing/
  logging/

ahrs/
  messages/
  estimator/
  logging/

common/
  channel/
  time/
  config/
```

Below is what each module does.

---

## 3. Messages (Shared Data Types)

Canonical message structs used between different modules:

### IMU messages
- `imu::messages::ImuSampleRaw`  
  Raw sensor counts from drivers.

- `imu::messages::ImuSample`  
  Calibrated SI units.

- `imu::messages::ImuCalibration`  
  Biases, scaling matrices, mag ellipsoid, etc.

### GPS messages
- `gps::messages::GpsFix`  
  Lat/Lon/Alt, accuracy, validity flag.

### State messages
- `nav::messages::StateEstimate`  
  Position, orientation, velocity, covariance.

### Design principles
- Pure data (POD-like structures)
- Hardware agnostic
- No file I/O or OS dependencies
- Shared between drivers, processing, estimator, logging

These serve as the **common language** between modules.

---

## 4. Drivers

Drivers interact with **hardware** or **log replay sources** and emit **messages**.

### IMU drivers (`imu::drivers`)
- `ImuDriver` (base)
- `lsm9ds0::Lsm9ds0Driver`
- `ReplayDriver`

### GPS drivers (`gps::drivers`)
- `GpsDriver` (base)
- `NmeaDriver`
- `UbxDriver`

Driver responsibilities:

- Low-level hardware communication (SPI / I2C / UART)
- Sensor register configuration
- Timestamping
- Output **ImuSampleRaw** or **GpsFix**

Drivers produce **messages**, but do not process or store them.

---

## 5. Processing (Calibration + Unit Conversion)

Sensor processing module transforms **raw sensor messages** into **engineering units** and apply offline computed calibration coefficients

### IMU processing (`imu::processing`)
- `ImuCalibrator`
- Optional: low-pass filtering or resampling utilities

Responsibilities:

- Convert raw LSB counts → SI units  
- Apply biases and scale matrices  
- Apply magnetometer ellipsoid calibration  
- Produce `imu::messages::ImuSample`

Processing modules contain **no hardware I/O** and interface only through messages.

---

## 6. Estimator (e.g., AHRS, INS, SLAM and VIO)

Implements a variety of different navigation solution, AHRS, INS (loosely and tightly coupled), SLAM (batch and slidinig window) and VIO.

### Estimator (`ahrs::estimator`)
- `ImuEkf`
- `EkfParams`

AHRS estimator responsibilities:

- `handle_imu(calibrated imu)` → propagation  
- `handle_gps(gps_fix)` → measurement update  
- Fuse gyro with accel and magnetometer data in real-time  
- Output `ahrs::messages::state_estimate`

This is the mathematical core of the system, independent from hardware and logging.

---

## 7. Logging

Logging stores raw and calibrated data to disk, and can replay it.

### IMU logging (`imu::logging`)
- `ImuRawWriter`
- `ImuRawReader`
- `ImuCalibratedWriter`
- `ImuCalibratedReader`

### GPS logging (`gps::logging`)
- `GpsLogger`

### State logging (`nav::logging`)
- `StateLogger`

Logging responsibilities:

- Write messages to disk (HDF5/CSV/MCAP/etc.)
- Read messages for replay or testing
- Never talk to hardware directly

This allows offline processing, calibration, or playback.

---

## 8. Threading Model

### Threads

- **IMU Driver Thread**  
  Reads IMU → pushes `ImuSampleRaw`

- **GPS Driver Thread**  
  Reads GPS → pushes `GpsFix`

- **EKF Thread**  
  - Pops IMU (high rate)  
  - Pops GPS (low rate)  
  - Runs EKF propagate/update  
  - Publishes `StateEstimate`

- **Logger Thread(s)** (optional)  
  Consume any channel to write logs.

### Channels

All threads communicate using:

```
common::channel::Channel<T>
```

Operations:

- `push(T value)`
- `pop(T& out)` (blocking)
- `stop()`

Channels ensure clean separation between producers and consumers.

---

## 9. Dataflow Diagram

```
                IMU Driver                         GPS Driver
             (imu::drivers)                     (gps::drivers)
                    │                                 │
       ImuSampleRaw │                                 │ GpsFix
                    ▼                                 ▼
             ┌─────────────────┐             ┌─────────────────┐
             │   ImuChannel    │             │   GpsChannel    │
             └─────────────────┘             └─────────────────┘
                    │                                 │
                    ▼                                 ▼
        ┌──────────────────────────────────────────────────────────┐
        │                       EKF Thread                         │
        │  imu::processing::ImuCalibrator → ImuSample              │
        │  nav::estimator::ImuGpsEkf                               │
        │     - handle_imu()                                       │
        │     - handle_gps()                                       │
        │     - current_state() → StateEstimate                    │
        └──────────────────────────┬───────────────────────────────┘
                                   │
                                   ▼
                             StateChannel
                                   │
                    ┌──────────────┴──────────────┐
                    ▼                             ▼
           StateLogger (logging)            Visualization
```

---

## 10. Directory Layout

```
imu-ekf-system/
├── include/
│   ├── imu/
│   │   ├── messages/
│   │   │   ├── imu_sample_raw.hpp
│   │   │   ├── imu_sample.hpp
│   │   │   └── imu_calibration.hpp
│   │   ├── drivers/
│   │   │   ├── imu_driver.hpp
│   │   │   ├── lsm9ds0_driver.hpp
│   │   │   └── replay_driver.hpp
│   │   ├── processing/
│   │   │   └── imu_calibrator.hpp
│   │   └── logging/
│   │       ├── imu_raw_writer.hpp
│   │       ├── imu_raw_reader.hpp
│   │       ├── imu_calibrated_writer.hpp
│   │       └── imu_calibrated_reader.hpp
│   │
│   ├── gps/
│   │   ├── messages/gps_fix.hpp
│   │   ├── drivers/nmea_driver.hpp
│   │   └── logging/gps_logger.hpp
│   │
│   ├── nav/
│   │   ├── messages/state_estimate.hpp
│   │   ├── estimator/imu_gps_ekf.hpp
│   │   └── logging/state_logger.hpp
│   │
│   ├── common/
│   │   ├── channel.hpp
│   │   ├── time.hpp
│   │   └── config.hpp
│   │
│   └── viz/ (optional)
│       └── state_publisher.hpp
│
└── src/
    ├── imu/
    ├── gps/
    ├── nav/
    ├── common/
    └── tools/
```

---

## 11. Philosophy

This architecture aims for:

- **Low coupling**  
  Modules interact only via messages.

- **High cohesion**  
  Drivers only talk to hardware; estimator only does math.

- **Replayability**  
  Logging + replay drivers allow offline testing and calibration.

- **Deterministic real-time behavior**  
  Clear thread priorities and channels.

- **Extensibility**  
  Easy to add:
  - Barometer  
  - Magnetometer  
  - Wheel odometry  
  - Vision measurements  
  - Full SLAM backend  

---

# End of Document
