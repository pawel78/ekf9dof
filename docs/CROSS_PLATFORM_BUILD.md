# Cross-Platform Build Support

## Overview

This project is designed to run on embedded Linux systems with I2C hardware support (e.g., Jetson Nano, Raspberry Pi). However, the code can be compiled on **macOS** and other non-Linux platforms for development purposes, allowing developers to catch compile-time errors before deploying to target hardware.

## Platform Support

| Platform | Compilation | Runtime I2C Access | Use Case |
|----------|-------------|-------------------|----------|
| Linux (x86_64/ARM) | ✅ Full support | ✅ Available | Production use |
| macOS | ✅ Compiles with warnings | ❌ Not available | Development/testing |
| Other POSIX | ✅ Likely compiles | ❌ Not available | Development/testing |

## Implementation Details

### Platform Detection

The code uses preprocessor macros to detect the platform at compile time:

```cpp
#if defined(__linux__)
    #define PLATFORM_LINUX 1
    // Linux-specific includes
    #include <linux/i2c-dev.h>
    #include <linux/i2c.h>
#else
    #define PLATFORM_LINUX 0
    #warning "Building for non-Linux platform - I2C hardware access will not be available"
#endif
```

### Conditional Compilation

Platform-specific code is wrapped in `#if PLATFORM_LINUX` blocks:

```cpp
#if PLATFORM_LINUX
    // Linux I2C implementation
    int fd = open(device, O_RDWR);
    // ... actual I2C operations
#else
    // Stub for non-Linux platforms
    throw std::runtime_error("I2C device access not supported on this platform");
#endif
```

### Files with Platform-Specific Code

- **src/lsm9ds0.cpp**: Contains I2CDevice class with platform-specific I2C operations
  - Linux: Full I2C hardware access via `/dev/i2c-*`
  - Non-Linux: Throws runtime error when I2C operations are attempted

### CMake Configuration

CMake detects the platform and provides appropriate messages:

```
-- Building for Linux - I2C hardware access enabled
```

or

```
CMake Warning:
  Building for macOS - I2C hardware access NOT available.
  Build for compile-time error checking only.
```

## Building on macOS

### Prerequisites

- CMake 3.18 or higher
- C++17 compatible compiler (Clang, GCC)

### Build Steps

```bash
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

### Expected Behavior

1. **Compilation**: Code compiles successfully with a compiler warning about non-Linux platform
2. **Tests**: Unit tests that don't require hardware should pass
3. **Runtime**: Any attempt to access I2C hardware will throw a runtime exception

### Example Compiler Warning

```
warning: "Building for non-Linux platform - I2C hardware access will not be available" [-W#warnings]
```

## Benefits for Development

1. **Early Error Detection**: Catch syntax errors, type mismatches, and other compile-time issues on macOS before deploying to target hardware
2. **Faster Iteration**: Develop and test non-hardware-dependent code on development machines
3. **Code Quality**: Ensure code portability and proper use of platform abstractions
4. **CI/CD Integration**: Enable continuous integration on macOS runners (e.g., GitHub Actions)

## Limitations on Non-Linux Platforms

The following features will **not work** on non-Linux platforms:

- ❌ Actual sensor data collection (requires I2C hardware)
- ❌ Magnetometer calibration (requires sensor access)
- ❌ Real-time sensor monitoring
- ❌ Any operation that calls `lsm9ds0_device::*` functions

However, these **will work**:

- ✅ Code compilation and syntax checking
- ✅ Unit tests for non-hardware components
- ✅ Configuration file parsing
- ✅ Data structure manipulation
- ✅ Mathematical algorithms (EKF, coordinate transformations)

## Future Enhancements

Potential improvements for cross-platform support:

1. **Mock Hardware Layer**: Implement a mock I2C device that can replay recorded sensor data
2. **Simulation Mode**: Add a `--simulate` flag to run with synthetic sensor data
3. **Unit Test Improvements**: Separate hardware-dependent and hardware-independent tests
4. **Windows Support**: Add conditional compilation for Windows platforms
5. **Hardware Abstraction Layer**: Create a proper HAL interface for better platform separation

## Troubleshooting

### "I2C device access not supported on this platform"

**Cause**: Attempting to run sensor-dependent code on a non-Linux platform.

**Solution**: This is expected behavior. The code compiled successfully for development purposes but cannot access hardware on non-Linux systems.

### Compilation Errors on macOS

**Cause**: Missing platform guards or incorrect conditional compilation.

**Solution**: Ensure all Linux-specific includes and code are wrapped in `#if PLATFORM_LINUX` blocks.

## Contributing

When adding new platform-specific code:

1. Always use `#if PLATFORM_LINUX` guards for Linux-specific operations
2. Provide appropriate stubs or throw runtime errors for non-Linux platforms
3. Add compile-time warnings when appropriate
4. Update this documentation with any new platform-specific features
5. Test on both Linux and macOS (if possible) before submitting PRs

## References

- [CMake Platform Detection](https://cmake.org/cmake/help/latest/variable/CMAKE_SYSTEM_NAME.html)
- [C++ Preprocessor Macros](https://gcc.gnu.org/onlinedocs/cpp/Predefined-Macros.html)
- [Linux I2C Documentation](https://www.kernel.org/doc/Documentation/i2c/)
