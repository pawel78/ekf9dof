# Coordinate Frames and Rotation Conventions

This document describes the coordinate frame conventions and naming standards used throughout the ekf9dof library.

## Table of Contents
- [Coordinate Frame Definitions](#coordinate-frame-definitions)
- [Quaternion Convention](#quaternion-convention)
- [Rotation Sequence](#rotation-sequence)
- [Naming Conventions](#naming-conventions)
- [Usage Examples](#usage-examples)
- [Common Transformations](#common-transformations)

## Coordinate Frame Definitions

### 1. Body Frame (b)
The **body frame** is fixed to the vehicle/sensor platform.

- **Origin**: Located at the IMU sensor
- **X-axis (Forward)**: Points toward the front of the vehicle
- **Y-axis (Right)**: Points to the right side of the vehicle
- **Z-axis (Down)**: Points downward through the vehicle

This is a right-handed coordinate system following the aerospace convention (Forward-Right-Down).

**Use cases:**
- Raw sensor measurements (gyroscope, accelerometer, magnetometer)
- Vehicle-relative vectors
- Control inputs

### 2. Navigation Frame (n) / Earth Frame (e)
The **navigation frame** uses the North-East-Down (NED) convention.

- **Origin**: Local tangent plane at the vehicle's starting position
- **X-axis (North)**: Points toward true north
- **Y-axis (East)**: Points east
- **Z-axis (Down)**: Points toward the Earth's center

This is a locally level, non-rotating reference frame commonly used in navigation and control.

**Use cases:**
- Position and velocity in local coordinates
- Gravity vector reference
- Waypoint navigation
- Attitude representation relative to Earth

### 3. Inertial Frame (i)
The **inertial frame** uses the Earth-Centered Earth-Fixed (ECEF) convention.

- **Origin**: Earth's center of mass
- **X-axis**: Points from Earth center to intersection of equator and prime meridian
- **Y-axis**: Points from Earth center to intersection of equator and 90°E longitude
- **Z-axis**: Points from Earth center through the North Pole

**Use cases:**
- GPS integration
- Long-distance navigation
- Multi-vehicle coordination

*Note: The attitude-only EKF primarily uses body and navigation frames. The inertial frame is included for future GPS integration.*

## Quaternion Convention

### Representation
Quaternions are represented as a 4-element vector:
```
q = [x, y, z, w]
```
where:
- `x, y, z` are the **imaginary** components (vector part)
- `w` is the **real/scalar** component

This follows the **Hamilton convention** with the scalar component **last**.

### Quaternion Interpretation
A quaternion `q_B_A` represents:
- The **rotation** from frame A to frame B
- The **attitude** of frame B relative to frame A
- The **orientation** that transforms vectors from A coordinates to B coordinates

**Example:**
- `q_n_b`: Rotation from body frame to navigation frame
- `q_b_n`: Rotation from navigation frame to body frame (inverse of `q_n_b`)

### Unit Quaternions
All quaternions in this library are **unit quaternions** (norm = 1), which represent pure rotations in SO(3).

## Rotation Sequence

### 3-2-1 Euler Angle Sequence
The library uses the **3-2-1 (yaw-pitch-roll)** rotation sequence, also known as **ZYX** or **Tait-Bryan angles**.

#### Sequence Order
Applied in the following order:
1. **Yaw (ψ)**: Rotation about Z-axis (vertical)
2. **Pitch (θ)**: Rotation about Y-axis (lateral)
3. **Roll (φ)**: Rotation about X-axis (longitudinal)

#### Mathematical Representation
```
R = Rx(φ) * Ry(θ) * Rz(ψ)
```

Where:
- `Rz(ψ)` = Yaw rotation matrix
- `Ry(θ)` = Pitch rotation matrix  
- `Rx(φ)` = Roll rotation matrix

#### Angle Ranges
- **Roll (φ)**: [-π, π] (-180° to +180°)
- **Pitch (θ)**: [-π/2, π/2] (-90° to +90°)
- **Yaw (ψ)**: [-π, π] (-180° to +180°)

#### Physical Interpretation
For an aircraft:
- **Roll**: Banking left/right
- **Pitch**: Nose up/down
- **Yaw**: Heading change

## Naming Conventions

### Vectors
Format: `v_frame`

**Examples:**
- `v_b`: Vector expressed in body frame
- `v_n`: Vector expressed in navigation frame
- `accel_b`: Acceleration vector in body frame
- `gyro_b`: Angular rate vector in body frame

### Quaternions
Format: `q_to_from`

**Examples:**
- `q_n_b`: Quaternion rotating from body to navigation frame
- `q_b_n`: Quaternion rotating from navigation to body frame
- `q_i_n`: Quaternion rotating from navigation to inertial frame

### Rotation Matrices
Format: `R_to_from`

**Examples:**
- `R_n_b`: Rotation matrix from body to navigation frame
- `R_b_n`: Rotation matrix from navigation to body frame

### Euler Angles
Format: `[roll, pitch, yaw]` in radians

**Example:**
```cpp
Eigen::Vector3d euler(0.1, 0.2, 0.3);  // [roll=0.1, pitch=0.2, yaw=0.3] rad
```

## Usage Examples

### Example 1: Converting Euler Angles to Quaternion
```cpp
#include "ekf9dof/so3.hpp"

using namespace ekf9dof;

// Define Euler angles [roll, pitch, yaw] in radians
Eigen::Vector3d euler(0.1, 0.2, 0.3);  // 5.7°, 11.5°, 17.2°

// Convert to quaternion
Quaternion q_n_b = Quaternion::from_euler(euler);

// Access quaternion components
double qx = q_n_b.x();
double qy = q_n_b.y();
double qz = q_n_b.z();
double qw = q_n_b.w();
```

### Example 2: Rotating a Vector Between Frames
```cpp
// Vector in body frame (e.g., accelerometer reading)
Eigen::Vector3d accel_b(0.0, 0.0, 9.81);  // 1g downward in body frame

// Current attitude
Eigen::Vector3d euler(0.0, M_PI/6.0, 0.0);  // 30° pitch
Quaternion q_n_b = Quaternion::from_euler(euler);

// Transform to navigation frame
Eigen::Vector3d accel_n = q_n_b.rotate(accel_b);

// accel_n now represents the acceleration in navigation coordinates
```

### Example 3: Inverse Transformation
```cpp
// Transform from navigation frame back to body frame
Quaternion q_b_n = q_n_b.inverse();  // or q_n_b.conjugate()
Eigen::Vector3d accel_b_again = q_b_n.rotate(accel_n);

// Alternatively, use rotate_inverse
Eigen::Vector3d accel_b_direct = q_n_b.rotate_inverse(accel_n);
```

### Example 4: Composing Rotations
```cpp
// First rotation: body to intermediate frame
Quaternion q_i_b = Quaternion::from_euler(Eigen::Vector3d(0.1, 0.0, 0.0));

// Second rotation: intermediate to navigation frame
Quaternion q_n_i = Quaternion::from_euler(Eigen::Vector3d(0.0, 0.2, 0.0));

// Combined rotation: body to navigation
Quaternion q_n_b = q_n_i * q_i_b;
```

### Example 5: Converting Back to Euler Angles
```cpp
Quaternion q = Quaternion::from_euler(Eigen::Vector3d(0.1, 0.2, 0.3));

// Convert back to Euler angles
Eigen::Vector3d euler_out = q.to_euler();

// euler_out == [0.1, 0.2, 0.3]
```

### Example 6: Using Rotation Matrices
```cpp
// Convert quaternion to rotation matrix
Eigen::Matrix3d R_n_b = q_n_b.to_rotation_matrix();

// Rotate vector using matrix
Eigen::Vector3d v_n = R_n_b * v_b;

// Convert rotation matrix back to quaternion
Quaternion q_recovered = Quaternion::from_rotation_matrix(R_n_b);
```

## Common Transformations

### Gravity Vector
The gravity vector in different frames:

```cpp
// In navigation frame (NED): gravity points down (positive Z)
Eigen::Vector3d gravity_n(0.0, 0.0, 9.81);

// Transform to body frame
Eigen::Vector3d gravity_b = q_n_b.rotate_inverse(gravity_n);
// or equivalently:
Eigen::Vector3d gravity_b = q_n_b.conjugate().rotate(gravity_n);
```

### Accelerometer Compensation
Remove gravity from accelerometer measurements:

```cpp
// Raw accelerometer measurement (body frame)
Eigen::Vector3d accel_measured_b = sensor.get_measurement();

// Current attitude
Quaternion q_n_b = get_current_attitude();

// Gravity in body frame
Eigen::Vector3d gravity_n(0.0, 0.0, 9.81);
Eigen::Vector3d gravity_b = q_n_b.rotate_inverse(gravity_n);

// True acceleration (without gravity)
Eigen::Vector3d accel_true_b = accel_measured_b - gravity_b;
```

### Gyroscope Integration
Update attitude using gyroscope measurements:

```cpp
// Angular rate from gyroscope (body frame)
Eigen::Vector3d omega_b = gyro.get_measurement();

// Time step
double dt = 0.01;  // 10ms

// Small angle approximation for quaternion update
double theta = omega_b.norm() * dt;
Eigen::Vector3d axis = omega_b.normalized();

// Create rotation quaternion
double half_theta = theta * 0.5;
Quaternion dq(
    axis(0) * std::sin(half_theta),
    axis(1) * std::sin(half_theta),
    axis(2) * std::sin(half_theta),
    std::cos(half_theta)
);

// Update attitude
Quaternion q_n_b_new = q_n_b_old * dq;
```

### Magnetometer Heading
Extract yaw from magnetometer:

```cpp
// Magnetometer reading (body frame)
Eigen::Vector3d mag_b = magnetometer.get_measurement();

// Rotate to navigation frame
Eigen::Vector3d mag_n = q_n_b.rotate(mag_b);

// Extract yaw (heading)
double yaw = std::atan2(mag_n(1), mag_n(0));  // atan2(East, North)
```

## Best Practices

1. **Consistency**: Always use the naming convention `q_to_from` to avoid confusion about rotation direction.

2. **Frame Awareness**: When performing calculations, explicitly track which frame each vector is expressed in.

3. **Normalization**: Quaternions should remain normalized. Periodically call `normalize()` to prevent numerical drift.

4. **Gimbal Lock**: Be aware that at pitch = ±90°, roll and yaw become coupled (gimbal lock). The library handles this gracefully, but consider using quaternions directly for these cases.

5. **Rotation Order**: The 321 sequence is standard in aerospace, but verify this matches your application's needs.

6. **Right-Hand Rule**: All coordinate frames follow the right-hand rule. Verify sensor orientations match this convention.

## References

- **Euler Angles**: Tait-Bryan angles, ZYX sequence (3-2-1)
- **Quaternions**: Hamilton convention, scalar-last representation
- **Coordinate Systems**: NED navigation frame, FRD body frame
- **Rotation Matrices**: Active rotations (vector rotation, not frame rotation)

## Additional Resources

- [Quaternion Tutorial](https://www.3dgep.com/understanding-quaternions/)
- [Coordinate Frame Conventions in Aerospace](https://www.nasa.gov/centers/dryden/pdf/88104main_H-2465.pdf)
- [Euler Angle Sequences](https://en.wikipedia.org/wiki/Euler_angles)
