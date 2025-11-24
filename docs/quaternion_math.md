# Quaternion Math Library

This document describes the quaternion math implementation following the 321 rotation sequence and NED (North-East-Down) navigation frame of reference.

## Table of Contents
- [Overview](#overview)
- [Class Structure](#class-structure)
- [Quaternion Representation](#quaternion-representation)
- [Coordinate Frames](#coordinate-frames)
- [Frame Transformation vs Rotation](#frame-transformation-vs-rotation)
- [Frame-Explicit Operations](#frame-explicit-operations)
- [Rotation Sequence (321)](#rotation-sequence-321)
- [API Reference](#api-reference)
- [Usage Examples](#usage-examples)

## Overview

The quaternion math library provides robust and efficient frame transformation operations for navigation and attitude estimation. The implementation is designed for use with 9-DOF IMU sensors and follows aerospace conventions.

**Key Features:**
- Hamilton convention quaternions (scalar-last: [x, y, z, w])
- 321 Euler angle sequence (yaw-pitch-roll / ZYX)
- NED navigation frame and FRD body frame support
- Frame-explicit naming for navigation clarity
- Automatic normalization for numerical stability
- Optimized frame transformation operations
- Complete conversion between quaternions, Euler angles, and rotation matrices

## Class Structure

The quaternion library uses a simple `Quat` class without namespace encapsulation:

```cpp
class Quat {
    // Quaternion implementation
};
```

**Usage:**
```cpp
#include "common/so3.hpp"

// Simple, direct usage
Quat my_quaternion;
Quat n_q_b = Quat::from_euler(euler_angles);
```

The class is self-contained and doesn't require a namespace since it's already well-encapsulated.

## Quaternion Representation

### Convention

Quaternions are represented as 4-element arrays:
```
q = [x, y, z, w]
```

Where:
- `x, y, z` are the **imaginary/vector** components
- `w` is the **real/scalar** component (**LAST** position)

This follows the **Hamilton convention** with the scalar component in the **last** position, which is standard in robotics and navigation applications.

### Unit Quaternions

All quaternions in this library are **unit quaternions** with norm = 1:
```
||q|| = sqrt(x² + y² + z² + w²) = 1
```

Unit quaternions represent pure rotations in SO(3) (Special Orthogonal group in 3D).

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

### 2. Navigation Frame (n) - NED Convention

The navigation frame uses North-East-Down coordinates:

- **X-axis (North)**: Points toward true north
- **Y-axis (East)**: Points east
- **Z-axis (Down)**: Points toward Earth's center (positive down)

**Properties:**
- Locally level, non-rotating reference frame
- Origin at local tangent plane (vehicle starting position)
- Used for position, velocity, and attitude representation

### 3. Camera Frame(s)

For visual sensors:
- Multiple camera frames may exist
- Each camera frame transformation relative to body frame is defined
- Typical camera frame: X-right, Y-down, Z-forward (different from body!)

### 4. GPS Antenna Frame

For GPS integration:
- Offset from body frame origin (IMU location)
- Position vector from IMU to GPS antenna in body frame: `r_gps_b`
- Used to compensate for lever arm effects in navigation

### 5. Wheel Odometry Frame

For wheeled vehicles with odometry:
- Typically aligned with body frame but at wheel axle height
- Offset from IMU location: `r_wheel_b`
- Used for velocity measurements and dead reckoning

## Frame Transformation vs Rotation

**Critical distinction:**

### Frame Transformation (what this library does)
- **Changes coordinate representation, NOT the physical vector**
- The same physical vector is expressed in different coordinate systems
- Example: A vector pointing North is [1,0,0] in NED frame but may be [0,1,0] in body frame
- Method: `transform()` and `transform_inverse()`
- Think: "Express vector from frame A in frame B coordinates"

### Rotation (different operation)
- **Changes the physical vector's direction while keeping the same frame**
- Actually rotates the vector in physical space
- Example: Rotating a pointing-North vector by 90° makes it point East
- This library does NOT provide rotation operations by design
- Think: "Turn the vector itself"

**For navigation:**
We use frame transformations because we want to express sensor measurements (which are in body frame) in navigation frame coordinates. The physical measurement doesn't change; we're just representing it differently.

## Frame-Explicit Operations

### Naming Convention

For a quaternion `b_q_a`:
- Expresses vectors **from frame 'a' in frame 'b' coordinates**
- Changes coordinate representation, not the physical vector
- Usage: `v_b = b_q_a.transform(v_a)`

**Examples:**
- `n_q_b`: Expresses body frame vectors in navigation frame coordinates
- `b_q_n`: Expresses navigation frame vectors in body frame coordinates
- `n_q_b = b_q_n.inverse()` or `n_q_b = b_q_n.conjugate()`

### Quaternion Composition

Frame composition follows the rule: `c_q_a = c_q_b * b_q_a`

This reads: "to go from frame 'a' to frame 'c', first transform from 'a' to 'b', then from 'b' to 'c'"

**Example:**
```cpp
Quat i_q_b;  // intermediate from body
Quat n_q_i;  // navigation from intermediate
Quat n_q_b = n_q_i * i_q_b;  // navigation from body (composed)

// Apply transformation
std::array<double, 3> v_b = {1.0, 0.0, 0.0};  // vector in body frame
std::array<double, 3> v_n = n_q_b.transform(v_b);  // vector in nav frame
```

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

### Angle Ranges

- **Roll (φ)**: [-π, π] (-180° to +180°)
- **Pitch (θ)**: [-π/2, π/2] (-90° to +90°)
- **Yaw (ψ)**: [-π, π] (-180° to +180°)

### Physical Interpretation

For an aircraft/vehicle:
- **Roll**: Banking left (negative) or right (positive)
- **Pitch**: Nose down (negative) or up (positive)  
- **Yaw**: Heading angle relative to north

## API Reference

### Class: `Quat`

Located in: `include/common/Quat.hpp` and `include/common/so3.hpp`

#### Constructors

```cpp
// Default constructor - identity quaternion [0, 0, 0, 1]
Quat();

// Construct from components (scalar LAST)
Quat(double x, double y, double z, double w);

// Construct from std::array<double, 4>
explicit Quat(const std::array<double, 4>& vec);
```

#### Accessors

```cpp
double x() const;                    // Get x component
double y() const;                    // Get y component
double z() const;                    // Get z component
double w() const;                    // Get w component (scalar, last)
double operator()(int i) const;      // Access by index (0-3)
const std::array<double, 4>& coeffs() const; // Get all coefficients [x,y,z,w]
```

#### Core Operations

```cpp
double norm() const;                 // Compute quaternion norm
void normalize();                    // Normalize to unit length
Quat normalized() const;             // Return normalized copy
Quat conjugate() const;              // Get conjugate (inverse transformation)
Quat inverse() const;                // Get inverse (same as conjugate for unit q)
```

#### Frame Transformation

```cpp
// Express vector in different frame coordinates
// For b_q_a: v_b = b_q_a.transform(v_a)
std::array<double, 3> transform(const std::array<double, 3>& v) const;

// Express by inverse transformation
// For b_q_a: v_a = b_q_a.transform_inverse(v_b)
std::array<double, 3> transform_inverse(const std::array<double, 3>& v) const;
```

#### Quaternion Composition

```cpp
// Compose frame transformations
// c_q_a = c_q_b * b_q_a
Quat operator*(const Quat& other) const;
```

#### Conversions

```cpp
// Convert to/from Euler angles (321 sequence)
static Quat from_euler(const std::array<double, 3>& euler);
std::array<double, 3> to_euler() const;

// Convert to/from rotation matrix
static Quat from_rotation_matrix(const std::array<std::array<double, 3>, 3>& R);
std::array<std::array<double, 3>, 3> to_rotation_matrix() const;
```

## Usage Examples

### Example 1: Create Quaternion from Euler Angles

```cpp
#include "common/so3.hpp"

// Define Euler angles: [roll, pitch, yaw] in radians
std::array<double, 3> euler{0.1, 0.2, 0.3};  // 5.7°, 11.5°, 17.2°

// Convert to quaternion (navigation from body)
Quat n_q_b = Quat::from_euler(euler);

// Access components (scalar is LAST: w at index 3)
std::cout << "Quaternion: [" << n_q_b.x() << ", " 
          << n_q_b.y() << ", " << n_q_b.z() << ", " 
          << n_q_b.w() << "]" << std::endl;
```

### Example 2: Transform Vector from Body to Navigation Frame

```cpp
// Accelerometer reading in body frame (1g downward in FRD)
std::array<double, 3> accel_b{0.0, 0.0, 9.81};

// Current attitude (30° pitch up)
std::array<double, 3> euler{0.0, M_PI/6.0, 0.0};
Quat n_q_b = Quat::from_euler(euler);

// Transform to navigation frame: v_n = n_q_b * v_b
std::array<double, 3> accel_n = n_q_b.transform(accel_b);

// With 30° pitch, gravity appears tilted in nav frame
std::cout << "Accel in nav frame: [" << accel_n[0] << ", " 
          << accel_n[1] << ", " << accel_n[2] << "]" << std::endl;
```

### Example 3: Transform Vector from Navigation to Body Frame

```cpp
// Gravity vector in navigation frame (NED: positive down)
std::array<double, 3> gravity_n{0.0, 0.0, 9.81};

// Current attitude quaternion (nav from body)
Quat n_q_b = Quat::from_euler(euler);

// Transform to body frame: v_b = n_q_b.transform_inverse(v_n)
std::array<double, 3> gravity_b = n_q_b.transform_inverse(gravity_n);

std::cout << "Gravity in body frame: [" << gravity_b[0] << ", " 
          << gravity_b[1] << ", " << gravity_b[2] << "]" << std::endl;
```

### Example 4: Remove Gravity from Accelerometer

```cpp
// Raw accelerometer (includes gravity)
std::array<double, 3> accel_measured_b = imu.get_acceleration();

// Current attitude (nav from body)
Quat n_q_b = get_current_attitude();

// Gravity in body frame
std::array<double, 3> gravity_n{0.0, 0.0, 9.81};
std::array<double, 3> gravity_b = n_q_b.transform_inverse(gravity_n);

// True acceleration (without gravity)
std::array<double, 3> accel_true_b{
    accel_measured_b[0] - gravity_b[0],
    accel_measured_b[1] - gravity_b[1],
    accel_measured_b[2] - gravity_b[2]
};
```

### Example 5: Compose Transformations (Frame-Explicit)

```cpp
// Rotation from body to intermediate frame
Quat i_q_b = Quat::from_euler(std::array<double, 3>{0.1, 0.0, 0.0});

// Rotation from intermediate to navigation frame
Quat n_q_i = Quat::from_euler(std::array<double, 3>{0.0, 0.2, 0.0});

// Combined rotation: navigation from body
// Formula: c_q_a = c_q_b * b_q_a
Quat n_q_b = n_q_i * i_q_b;

// Apply transformation
std::array<double, 3> v_b{1.0, 2.0, 3.0};
std::array<double, 3> v_n = n_q_b.transform(v_b);
```

### Example 6: Integrate Gyroscope for Attitude Update

```cpp
// Angular rate from gyroscope (rad/s in body frame)
std::array<double, 3> omega_b = imu.get_gyro();

// Time step
double dt = 0.01;  // 10 ms

// Compute rotation angle and axis
double omega_norm = std::sqrt(omega_b[0]*omega_b[0] + 
                              omega_b[1]*omega_b[1] + 
                              omega_b[2]*omega_b[2]);
double theta = omega_norm * dt;

if (theta > 1e-8) {  // Avoid division by zero
    std::array<double, 3> axis{
        omega_b[0] / omega_norm,
        omega_b[1] / omega_norm,
        omega_b[2] / omega_norm
    };
    
    // Create incremental rotation quaternion
    double half_theta = theta * 0.5;
    Quat dq(
        axis[0] * std::sin(half_theta),
        axis[1] * std::sin(half_theta),
        axis[2] * std::sin(half_theta),
        std::cos(half_theta)
    );
    
    // Update attitude (right multiplication for body-frame rates)
    Quat n_q_b_new = n_q_b_old * dq;
    
    // Normalize periodically to prevent drift
    n_q_b_new.normalize();
}
```

### Example 7: Extract Heading from Magnetometer

```cpp
// Magnetometer reading (body frame)
std::array<double, 3> mag_b = imu.get_magnetometer();

// Current attitude (nav from body)
Quat n_q_b = get_current_attitude();

// Rotate to navigation frame
std::array<double, 3> mag_n = n_q_b.transform(mag_b);

// Extract heading (yaw) from horizontal components (NED frame)
double heading = std::atan2(mag_n[1], mag_n[0]);  // atan2(East, North)

std::cout << "Magnetic heading: " << heading * 180.0 / M_PI << " degrees" << std::endl;
```

### Example 8: Camera to Body Frame Transformation

```cpp
// Camera frame: X-right, Y-down, Z-forward
// Body frame: X-forward, Y-right, Z-down
// Need rotation to align frames

std::array<double, 3> camera_euler{0.0, M_PI/2.0, M_PI/2.0};
Quat b_q_cam = Quat::from_euler(camera_euler);

// Transform feature point from camera to body frame
std::array<double, 3> point_cam = detect_feature();
std::array<double, 3> point_b = b_q_cam.transform(point_cam);
```

## Best Practices

1. **Frame-explicit naming**: Always use the `to_from` convention (e.g., `n_q_b` for navigation from body) to avoid confusion.

2. **Scalar last**: Remember that the quaternion representation is [x, y, z, w] with the scalar component **last**.

3. **Composition order**: For `c_q_a = c_q_b * b_q_a`, the transformation is applied right-to-left (first b_q_a, then c_q_b).

4. **Always normalize**: Quaternions should remain normalized. Call `normalize()` periodically to prevent numerical drift.

5. **Gimbal lock**: At pitch = ±90°, use quaternions directly rather than converting to Euler angles.

6. **Use std::array**: The library uses `std::array<double, 3>` and `std::array<double, 4>` instead of Eigen types for simplicity and portability.

7. **Namespace qualification**: Always use `Quat` for the class name to avoid ambiguity with the namespace.

## References

- **Quaternions**: Hamilton convention, scalar-last [x, y, z, w]
- **Euler Angles**: 321 sequence (yaw-pitch-roll, ZYX Tait-Bryan angles)
- **Frames**: NED navigation, FRD body
- **Rotation Convention**: Active rotations (vector rotation, not frame rotation)

## See Also

- [COORDINATE_FRAMES.md](COORDINATE_FRAMES.md) - Detailed frame conventions
- [include/common/Quat.hpp](../include/common/Quat.hpp) - Core implementation
- [include/common/so3.hpp](../include/common/so3.hpp) - Header with documentation
- [tests/test_so3.cpp](../tests/test_so3.cpp) - Test suite and examples
