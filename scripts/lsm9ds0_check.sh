#!/usr/bin/env bash
# lsm9ds0_check.sh — verify LSM9DS0 presence on a given I2C bus.
# Usage: sudo ./lsm9ds0_check.sh [BUS]
# Example: sudo ./lsm9ds0_check.sh 7
# Returns: exit 0 if both devices (gyro + accel/mag) are identified; non-zero otherwise.

BUS="${1:-7}"     # default to bus 7 if not provided

# Require i2c-tools
if ! command -v i2cget >/dev/null 2>&1; then
  echo "ERROR: i2cget not found. Install with: sudo apt update && sudo apt install -y i2c-tools"
  exit 2
fi

# Helper: try multiple addresses for a device and validate WHO_AM_I
# args: <name> <who_reg_hex> <expected_csv_hex> <addr1> <addr2> ...
check_device() {
  local NAME="$1"; shift
  local WHO_REG="$1"; shift
  local EXPECTED_CSV="$1"; shift
  local ADDRS=("$@")

  local EXPECTED=()
  IFS=',' read -r -a EXPECTED <<< "$EXPECTED_CSV"

  for A in "${ADDRS[@]}"; do
    # Attempt to read WHO_AM_I; suppress stderr, capture value
    VAL=$(i2cget -y "$BUS" "$A" "$WHO_REG" 2>/dev/null)
    RC=$?
    if [ $RC -ne 0 ]; then
      continue
    fi

    # Normalize (e.g., 0xD4 or 0xd4)
    VAL_LC=$(echo "$VAL" | tr 'A-F' 'a-f')
    for E in "${EXPECTED[@]}"; do
      E_LC=$(echo "$E" | tr 'A-F' 'a-f')
      if [ "$VAL_LC" = "$E_LC" ]; then
        printf "%s OK: addr 0x%02x WHO_AM_I=%s\n" "$NAME" "0x$A" "$VAL"
        return 0
      fi
    done
    printf "%s at 0x%02x responded, but WHO_AM_I=%s (expected %s)\n" \
      "$NAME" "0x$A" "$VAL" "$EXPECTED_CSV"
  done

  printf "%s NOT FOUND/IDENTIFIED on bus %s (tried addrs: %s)\n" \
    "$NAME" "$BUS" "${ADDRS[*]}"
  return 1
}

# LSM9DS0 specifics:
# - Gyro at 0x6A or 0x6B, WHO_AM_I=0xD4 (some revs 0xD7)
# - Accel/Mag (XM) at 0x1D or 0x1E, WHO_AM_I=0x49
GYRO_OK=0
XM_OK=0

check_device "GYRO" "0x0f" "0xD4,0xD7" 6a 6b && GYRO_OK=1
check_device "ACCEL/MAG" "0x0f" "0x49" 1d 1e && XM_OK=1

if [ $GYRO_OK -eq 1 ] && [ $XM_OK -eq 1 ]; then
  echo "SUCCESS: LSM9DS0 (gyro + accel/mag) identified on I2C bus ${BUS}."
  exit 0
else
  echo "FAIL: Could not positively identify both LSM9DS0 devices on bus ${BUS}."
  exit 1
fi

