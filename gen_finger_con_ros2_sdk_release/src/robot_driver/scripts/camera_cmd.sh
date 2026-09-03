#!/usr/bin/env bash
set -euo pipefail

# Usage examples (after colcon build + source install/setup.bash):
#   Single device:
#     bash camera_cmd.sh camerarc
#     bash camera_cmd.sh MCUID
#   Dual device (left/right):
#     bash camera_cmd.sh left camerarc
#     bash camera_cmd.sh right MCUID
# Optional: SERIAL_PORT=/dev/ttyFingerLeft

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

usage() {
  echo "Usage:"
  echo "  Single: bash ${BASH_SOURCE[0]} {1234|camerarc|MCUID}"
  echo "  Dual:   bash ${BASH_SOURCE[0]} {left|right} {1234|camerarc|MCUID}"
  echo "Optional env: SERIAL_PORT=/dev/ttyFingerLeft"
  exit 1
}

if [[ $# -eq 1 ]]; then
  SIDE=""
  RECORD_VALUE="$1"
elif [[ $# -eq 2 ]]; then
  SIDE="$1"
  RECORD_VALUE="$2"
  if [[ "${SIDE}" != "left" && "${SIDE}" != "right" ]]; then
    echo "Error: first argument must be 'left' or 'right'"
    usage
  fi
else
  usage
fi

case "${RECORD_VALUE}" in
  1234|camerarc|MCUID)
    ;;
  *)
    echo "Error: command must be one of 1234/camerarc/MCUID"
    usage
    ;;
esac

CMD=(ros2 run robot_driver camera_calib_cmd "${RECORD_VALUE}")

if [[ -n "${SIDE}" ]]; then
  CMD+=(--side "${SIDE}")
fi

if [[ -n "${SERIAL_PORT:-}" ]]; then
  CMD+=(--serial-port "${SERIAL_PORT}")
fi

echo "Command: ${RECORD_VALUE}, device: ${SIDE:-single}, serial: ${SERIAL_PORT:-configured udev device}"
exec "${CMD[@]}"
