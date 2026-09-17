#!/usr/bin/env bash
# Usage:
#   ./tools/create_serial_alias.sh /dev/ttyACM0 arduino
#   ./tools/create_serial_alias.sh /dev/ttyUSB0 lidar
# Creates /dev/<alias> using vendor/product and, if available, USB serial number.
set -euo pipefail

if [ "$#" -ne 2 ]; then
  echo "Usage: $0 /dev/ttyACM0 arduino"
  exit 2
fi

DEV="$1"
ALIAS="$2"
RULE_FILE="/etc/udev/rules.d/99-skku-vehicle.rules"

if [ ! -e "$DEV" ]; then
  echo "Device not found: $DEV"
  exit 1
fi
if [[ ! "$ALIAS" =~ ^[A-Za-z0-9_-]+$ ]]; then
  echo "Alias must contain only letters, numbers, _ or -"
  exit 1
fi

PROPS="$(udevadm info --query=property --name="$DEV")"
VID="$(printf '%s\n' "$PROPS" | awk -F= '$1=="ID_VENDOR_ID"{print $2; exit}')"
PID="$(printf '%s\n' "$PROPS" | awk -F= '$1=="ID_MODEL_ID"{print $2; exit}')"
SER="$(printf '%s\n' "$PROPS" | awk -F= '$1=="ID_SERIAL_SHORT"{print $2; exit}')"

if [ -z "$VID" ] || [ -z "$PID" ]; then
  echo "Could not determine USB VID/PID for $DEV"
  echo "$PROPS" | grep -E 'ID_VENDOR|ID_MODEL|ID_SERIAL|ID_PATH' || true
  exit 1
fi

if [ -n "$SER" ]; then
  RULE="SUBSYSTEM==\"tty\", ENV{ID_VENDOR_ID}==\"$VID\", ENV{ID_MODEL_ID}==\"$PID\", ENV{ID_SERIAL_SHORT}==\"$SER\", SYMLINK+=\"$ALIAS\", GROUP=\"dialout\", MODE=\"0660\""
else
  echo "WARNING: no serial number found; rule will match every device with VID:PID $VID:$PID"
  RULE="SUBSYSTEM==\"tty\", ENV{ID_VENDOR_ID}==\"$VID\", ENV{ID_MODEL_ID}==\"$PID\", SYMLINK+=\"$ALIAS\", GROUP=\"dialout\", MODE=\"0660\""
fi

echo "Will append this rule to $RULE_FILE:"
echo "$RULE"
echo "$RULE" | sudo tee -a "$RULE_FILE" >/dev/null
sudo udevadm control --reload-rules
sudo udevadm trigger

echo
printf 'Created rule. Unplug/replug the device, then check:\n  ls -l /dev/%s\n' "$ALIAS"
