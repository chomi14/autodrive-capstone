#!/usr/bin/env bash
set -u

echo '=== serial devices ==='
ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || true

echo
echo '=== stable serial by-id links ==='
ls -l /dev/serial/by-id/ 2>/dev/null || true

echo
echo '=== cameras ==='
if command -v v4l2-ctl >/dev/null 2>&1; then
  v4l2-ctl --list-devices || true
else
  echo 'v4l2-ctl not installed: sudo apt install v4l-utils'
fi

echo
echo '=== stable camera by-id links ==='
ls -l /dev/v4l/by-id/ 2>/dev/null || true

echo
echo '=== USB list ==='
lsusb || true

echo
echo 'For a serial device, inspect attributes with:'
echo '  udevadm info --query=property --name=/dev/ttyACM0 | grep -E "ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL_SHORT|ID_PATH"'
echo '  udevadm info --query=property --name=/dev/ttyUSB0 | grep -E "ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL_SHORT|ID_PATH"'
