# Quaternion Math Library

This document describes the quaternion math implementation in the ekf9dof library, following the 321 rotation sequence and NED (North-East-Down) navigation frame of reference.

## Table of Contents
- [Overview](#overview)
- [Quaternion Representation](#quaternion-representation)
- [Coordinate Frames](#coordinate-frames)
- [Rotation Sequence (321)](#rotation-sequence-321)
- [API Reference](#api-reference)
- [Usage Examples](#usage-examples)
- [Frame Transformations](#frame-transformations)

## Overview

The quaternion math library provides robust and efficient 3D rotation operations for navigation and attitude estimation. The implementation is designed for use with 9-DOF IMU sensors and follows aerospace conventions.

**Key Features:**
- Hamilton convention quaternions (scalar-last: [x, y, z, w])
- 321 Euler angle sequence (yaw-pitch-roll / ZYX)
- NED navigation frame and FRD body frame support
- Automatic normalization for numerical stability
- Optimized vector rotation operations
- Complete conversion between quaternions, Euler angles, and rotation matrices

## Quaternion Representation

### Convention

Quaternions are represented as 4-element vectors:
```
q = [x, y, z, w]
```

Where:
- `x, y, z` are the **imaginary/vector** components
- `w` is the **real/scalar** component (last position)

This follows the **Hamilton convention** with the scalar component in the **last** position.

### Unit Quaternions

All quaternions in this library are **unit quaternions** with norm = 1:
```
||q|| = sqrt(x² + y² + z² + w²) = 1
```

Unit quaternions represent pure rotations in SO(3) (Special Orthogonal group in 3D).

### Quaternion Naming

Format: `q_to_from`

A quaternion `q_B_A` represents:
- The rotation **from frame A to frame B**
- The attitude of frame B **relative to** frame A
- The transformation that rotates vectors from A coordinates to B coordinates

**Examples:**
- `q_n_b`: Rotation from body frame to navigation frame
- `q_b_n`: Rotation from navigation frame to body frame (inverse of `q_n_b`)

## Coordinate Frames

### 1. Body Frame (b) - FRD Convention

The body frame is fixed to the vehicle/sensor platform:

- **X-axis (Forward)**: Points toward the front of the vehicle
- **Y-axis (Right)**: Points to the right side of the vehicle  
- **Z-axis (Down)**: Points downward through the vehicle

**IMU Sensor Frame:**
- Origin at the IMU sensor location
- Axes aligned with FRD convention
- Raw sensor measurements (gyro, accel, mag) are in this frame

**Camera Frame(s):**
- Multiple camera frames may exist
- Each camera frame transformation relative to body frame is defined
- Typical camera frame: X-right, Y-down, Z-forward (different from body!)

### 2. Navigation Frame (n) - NED Convention

The navigation frame uses North-East-Down coordinates:

- **X-axis (North)**: Points toward true north
- **Y-axis (East)**: Points east
- **Z-axis (Down)**: Points toward Earth's center (positive down)

**Properties:**
- Locally level, non-rotating reference frame
- Origin at local tangent plane (vehicle starting position)
- Used for position, velocity, and attitude representation

### 3. GPS Antenna Frame

For GPS integration:
- Offset from body frame origin (IMU location)
- Position vector from IMU to GPS antenna in body frame: `r_gps_b`
- Used to compensate for lever arm effects in navigation

### 4. Wheel Odometry Frame

For wheeled vehicles with odometry:
- Typically aligned with body frame but at wheel axle height
- Offset from IMU location: `r_wheel_b`
- Used for velocity measurements and dead reckoning

## Rotation Sequence (321)

### Definition

The 321 Euler angle sequence is also known as:
- **Yaw-Pitch-Roll** sequence
- **ZYX** Tait-Bryan angles
- **Aerospace sequence**

### Rotation Order

Rotations are applied in the following order (from reference to body):

1. **First**: Yaw (ψ) - Rotation about Z-axis (vertical)
2. **Second**: Pitch (θ) - Rotation about Y-axis (lateral)
3. **Third**: Roll (φ) - Rotation about X-axis (longitudinal)

### Matrix Representation

The combined rotation matrix is:
```
R = Rx(φ) * Ry(θ) * Rz(ψ)
```

Where:
```
Rz(ψ) = | cos(ψ)  -sin(ψ)   0     |
        | sin(ψ)   cos(ψ)   0     |
        | 0        0         1     |

Ry(θ) = | cos(θ)   0     sin(θ)   |
        | 0        1     0         |
        |-sin(θ)   0     cos(θ)   |

Rx(φ) = | 1        0         0     |
        | 0     cos(φ)  -sin(φ)   |
        | 0     sin(φ)   cos(φ)   |
```

### Angle Ranges

- **Roll (φ)**: [-π, π] (-180° to +180°)
- **Pitch (θ)**: [-π/2, π/2] (-90° to +90°)
- **Yaw (ψ)**: [-π, π] (-180° to +180°)

### Physical Interpretation

For an aircraft/vehicle:
- **Roll**: Banking left (negative) or right (positive)
- **Pitch**: Nose down (negative) or up (positive)  
- **Yaw**: Heading angle relative to north

### Gimbal Lock

At pitch = ±90°, roll and yaw become coupled (gimbal lock). The library handles this case gracefully, but for applications requiring full 3D orientation at all attitudes, use quaternions directly rather than Euler angles.

## API Reference

### Class: `Quaternion`

Located in: `include/common/quaternion.hpp` (implementation) and `include/common/so3.hpp` (Eigen wrapper)

#### Constructors

```cpp
// Default constructor - identity quaternion [0, 0, 0, 1]
Quaternion();

// Construct from components
Quaternion(double x, double y, double z, double w);

// Construct from Eigen::Vector4d
explicit Quaternion(const Eigen::Vector4d& vec);
```

#### Accessors

```cpp
double x() const;                    // Get x component
double y() const;                    // Get y component
double z() const;                    // Get z component
double w() const;                    // Get w component
double operator()(int i) const;      // Access by index (0-3)
const Eigen::Vector4d& coeffs() const; // Get all coefficients
```

#### Core Operations

```cpp
double norm() const;                 // Compute quaternion norm
void normalize();                     // Normalize to unit length
Quaternion normalized() const;        // Return normalized copy
Quaternion conjugate() const;         // Get conjugate (inverse rotation)
Quaternion inverse() const;           // Get inverse (same as conjugate for unit q)
```

#### Vector Rotation

```cpp
// Rotate vector: v' = q * v * q^(-1)
Eigen::Vector3d rotate(const Eigen::Vector3d& v) const;

// Rotate by inverse: v' = q^(-1) * v * q
Eigen::Vector3d rotate_inverse(const Eigen::Vector3d& v) const;
```

#### Quaternion Composition

```cpp
// Multiply quaternions (compose rotations)
// q1 * q2 means: first rotate by q2, then by q1
Quaternion operator*(const Quaternion& other) const;
```

#### Conversions

```cpp
// Convert to/from Euler angles (321 sequence)
static Quaternion from_euler(const Eigen::Vector3d& euler);
Eigen::Vector3d to_euler() const;

// Convert to/from rotation matrix
static Quaternion from_rotation_matrix(const Eigen::Matrix3d& R);
Eigen::Matrix3d to_rotation_matrix() const;
```

## Usage Examples

### Example 1: Create Quaternion from Euler Angles

```cpp
#include "common/so3.hpp"

using namespace ekf9dof;

// Define Euler angles: [roll, pitch, yaw] in radians
Eigen::Vector3d euler(0.1, 0.2, 0.3);  // 5.7°, 11.5°, 17.2°

// Convert to quaternion
Quaternion q_n_b = Quaternion::from_euler(euler);

// Access components
std::cout << "Quaternion: [" << q_n_b.x() << ", " 
          << q_n_b.y() << ", " << q_n_b.z() << ", " 
          << q_n_b.w() << "]" << std::endl;
```

### Example 2: Rotate Vector from Body to Navigation Frame

```cpp
// Accelerometer reading in body frame (1g downward)
Eigen::Vector3d accel_b(0.0, 0.0, 9.81);

// Current attitude (30° pitch up)
Eigen::Vector3d euler(0.0, M_PI/6.0, 0.0);
Quaternion q_n_b = Quaternion::from_euler(euler);

// Transform to navigation frame
Eigen::Vector3d accel_n = q_n_b.rotate(accel_b);

// With 30° pitch, gravity appears tilted in nav frame
std::cout << "Accel in nav frame: " << accel_n.transpose() << std::endl;
```

### Example 3: Rotate Vector from Navigation to Body Frame

```cpp
// Gravity vector in navigation frame (NED: positive down)
Eigen::Vector3d gravity_n(0.0, 0.0, 9.81);

// Rotate to body frame
Eigen::Vector3d gravity_b = q_n_b.rotate_inverse(gravity_n);
// Or equivalently:
// Eigen::Vector3d gravity_b = q_n_b.conjugate().rotate(gravity_n);

std::cout << "Gravity in body frame: " << gravity_b.transpose() << std::endl;
```

### Example 4: Remove Gravity from Accelerometer

```cpp
// Raw accelerometer (includes gravity)
Eigen::Vector3d accel_measured_b = imu.get_acceleration();

// Current attitude
Quaternion q_n_b = get_current_attitude();

// Gravity in body frame
Eigen::Vector3d gravity_n(0.0, 0.0, 9.81);
Eigen::Vector3d gravity_b = q_n_b.rotate_inverse(gravity_n);

// True acceleration (without gravity)
Eigen::Vector3d accel_true_b = accel_measured_b - gravity_b;
```

### Example 5: Compose Rotations

```cpp
// Rotation from body to intermediate frame
Quaternion q_i_b = Quaternion::from_euler(Eigen::Vector3d(0.1, 0.0, 0.0));

// Rotation from intermediate to navigation frame
Quaternion q_n_i = Quaternion::from_euler(Eigen::Vector3d(0.0, 0.2, 0.0));

// Combined rotation: body to navigation
// Order matters! Right-to-left application
Quaternion q_n_b = q_n_i * q_i_b;
```

### Example 6: Integrate Gyroscope for Attitude Update

```cpp
// Angular rate from gyroscope (rad/s in body frame)
Eigen::Vector3d omega_b = imu.get_gyro();

// Time step
double dt = 0.01;  // 10 ms

// Rotation angle and axis
double theta = omega_b.norm() * dt;
Eigen::Vector3d axis = omega_b.normalized();

// Create incremental rotation quaternion
double half_theta = theta * 0.5;
Quaternion dq(
    axis(0) * std::sin(half_theta),
    axis(1) * std::sin(half_theta),
    axis(2) * std::sin(half_theta),
    std::cos(half_theta)
);

// Update attitude (right multiplication for body-frame rates)
Quaternion q_n_b_new = q_n_b_old * dq;

// Normalize periodically to prevent drift
q_n_b_new.normalize();
```

### Example 7: Convert to Rotation Matrix

```cpp
Quaternion q = Quaternion::from_euler(Eigen::Vector3d(0.1, 0.2, 0.3));

// Get rotation matrix
Eigen::Matrix3d R_n_b = q.to_rotation_matrix();

// Rotate vector using matrix (equivalent to q.rotate())
Eigen::Vector3d v_b(1.0, 2.0, 3.0);
Eigen::Vector3d v_n = R_n_b * v_b;

// Verify: R should be orthogonal
Eigen::Matrix3d I = R_n_b.transpose() * R_n_b;
assert((I - Eigen::Matrix3d::Identity()).norm() < 1e-6);
```

### Example 8: Extract Heading from Magnetometer

```cpp
// Magnetometer reading (body frame)
Eigen::Vector3d mag_b = imu.get_magnetometer();

// Current attitude
Quaternion q_n_b = get_current_attitude();

// Rotate to navigation frame
Eigen::Vector3d mag_n = q_n_b.rotate(mag_b);

// Extract heading (yaw) from horizontal components
double heading = std::atan2(mag_n(1), mag_n(0));  // atan2(East, North)

std::cout << "Magnetic heading: " << heading * 180.0 / M_PI << " degrees" << std::endl;
```

## Frame Transformations

### IMU to Body Frame

The IMU sensor frame is typically aligned with the body frame:
```cpp
// If IMU is aligned with body frame:
Eigen::Vector3d measurement_b = imu.get_measurement();

// If IMU has a fixed rotation relative to body:
Quaternion q_b_imu = Quaternion::from_euler(imu_rotation_offset);
Eigen::Vector3d measurement_b = q_b_imu.rotate(measurement_imu);
```

### Camera to Body Frame

Cameras typically have different frame conventions:
```cpp
// Camera frame: X-right, Y-down, Z-forward
// Body frame: X-forward, Y-right, Z-down
// Rotation: 90° about body-Y, then 90° about new-Z

Eigen::Vector3d camera_rotation(0.0, M_PI/2.0, M_PI/2.0);
Quaternion q_b_cam = Quaternion::from_euler(camera_rotation);

// Transform feature point from camera to body frame
Eigen::Vector3d point_cam = detect_feature();
Eigen::Vector3d point_b = q_b_cam.rotate(point_cam);
```

### GPS Antenna Position Compensation

```cpp
// GPS antenna offset from IMU (in body frame)
Eigen::Vector3d r_gps_b(0.1, 0.0, -0.05);  // 10cm forward, 5cm up

// Vehicle angular rate
Eigen::Vector3d omega_b = imu.get_gyro();

// GPS velocity includes rotation effect
Eigen::Vector3d v_nav = gps.get_velocity();

// Remove lever arm effect to get velocity at IMU
Quaternion q_n_b = get_current_attitude();
Eigen::Vector3d omega_n = q_n_b.rotate(omega_b);

// Cross product in navigation frame
Eigen::Vector3d r_gps_n = q_n_b.rotate(r_gps_b);
Eigen::Vector3d v_imu_n = v_nav - omega_n.cross(r_gps_n);
```

### Wheel Odometry Integration

```cpp
// Wheel encoder gives velocity in odometry frame (aligned with body)
double wheel_speed = encoder.get_speed();  // m/s forward

// Convert to body frame velocity
Eigen::Vector3d v_body(wheel_speed, 0.0, 0.0);  // Forward motion only

// Transform to navigation frame for integration
Quaternion q_n_b = get_current_attitude();
Eigen::Vector3d v_nav = q_n_b.rotate(v_body);

// Integrate for position
position_n += v_nav * dt;
```

## Best Practices

1. **Always normalize**: Quaternions should remain normalized. Call `normalize()` periodically to prevent numerical drift.

2. **Frame consistency**: Always track which frame each vector is expressed in. Use clear variable naming.

3. **Rotation order**: Remember that `q1 * q2` means "first rotate by q2, then by q1" (right-to-left).

4. **Gimbal lock**: At pitch = ±90°, use quaternions directly rather than converting to Euler angles.

5. **Sign ambiguity**: Quaternions q and -q represent the same rotation. Use the representation closest to identity for smooth interpolation.

6. **Numerical stability**: The library uses Shepperd's method for rotation matrix to quaternion conversion to avoid division by small numbers.

## References

- **Quaternions**: Hamilton convention, scalar-last [x, y, z, w]
- **Euler Angles**: 321 sequence (yaw-pitch-roll, ZYX Tait-Bryan angles)
- **Frames**: NED navigation, FRD body
- **Rotation Convention**: Active rotations (vector rotation, not frame rotation)

## See Also

- [COORDINATE_FRAMES.md](COORDINATE_FRAMES.md) - Detailed frame conventions
- [include/common/quaternion.hpp](../include/common/quaternion.hpp) - Implementation
- [include/common/so3.hpp](../include/common/so3.hpp) - Eigen-compatible wrapper
- [tests/test_so3.cpp](../tests/test_so3.cpp) - Test suite and examples
