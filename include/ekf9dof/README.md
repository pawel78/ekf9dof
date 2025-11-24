# ekf9dof Include Directory

This directory contains compatibility headers for backward compatibility with existing code that includes files using the `ekf9dof/` prefix.

## Files

- `so3.hpp` - Symbolic link to `../common/so3.hpp`

The actual implementation files are in the `common/` directory. This symlink ensures that both `#include "common/so3.hpp"` and `#include "ekf9dof/so3.hpp"` work correctly.

## Note for Developers

When adding new common headers, prefer placing them in the `common/` directory and creating symlinks here only if needed for backward compatibility.
