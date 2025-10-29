#!/usr/bin/env bash
# I2C identity checks for LSM9DS0 on Jetson (bus 7)
# - Gyro    @ 0x6B, WHO_AM_I 0x0F -> expect 0xD4
# - Acc/Mag @ 0x1D, WHO_AM_I 0x0F -> expect 0x49

set -u -o pipefail

BUS=7
WHO_REG=0x0F

_read_byte() {
  # args: bus addr reg
  sudo i2cget -y "$1" "$2" "$3" b 2>/dev/null
}

check_gyro() {
  local out
  if out=$(_read_byte "$BUS" 0x6b "$WHO_REG"); then
    if [[ "${out,,}" == "0xd4" ]]; then
      echo "PASS: gyro @0x6B WHO_AM_I=0xD4"
      return 0
    else
      echo "FAIL: gyro @0x6B WHO_AM_I=$out (expected 0xD4)"
      return 1
    fi
  else
    echo "FAIL: gyro @0x6B not responding"
    return 1
  fi
}

check_accelmag() {
  local out
  if out=$(_read_byte "$BUS" 0x1d "$WHO_REG"); then
    if [[ "${out,,}" == "0x49" ]]; then
      echo "PASS: accel/mag @0x1D WHO_AM_I=0x49"
      return 0
    else
      echo "FAIL: accel/mag @0x1D WHO_AM_I=$out (expected 0x49)"
      return 1
    fi
  else
    echo "FAIL: accel/mag @0x1D not responding"
    return 1
  fi
}

# If executed directly, run both and report overall status.
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  gyro_ok=1
  acc_ok=1

  check_gyro;     gyro_ok=$?
  check_accelmag; acc_ok=$?

  if (( gyro_ok == 0 && acc_ok == 0 )); then
    echo "OVERALL: PASS"
    exit 0
  else
    echo "OVERALL: FAIL"
    exit 1
  fi
fi
