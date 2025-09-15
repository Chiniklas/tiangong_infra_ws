#!/usr/bin/env bash
set -euo pipefail

# Defaults
SIDE="left_hand"
RATIO=""
FINGER=""

# Parse args:
# - If first arg looks numeric, treat as RATIO and second as FINGER.
# - Else treat first as SIDE, second as RATIO, third as FINGER.
if [[ $# -ge 1 && "$1" =~ ^[0-9.]+$ ]]; then
  # Usage: ./finger_one.sh <ratio> <finger>
  RATIO="${1}"
  FINGER="${2:-}"
else
  # Usage: ./finger_one.sh <side> <ratio> <finger>
  SIDE="${1:-left_hand}"
  RATIO="${2:-}"
  FINGER="${3:-}"
fi

if [[ -z "${RATIO}" || -z "${FINGER}" ]]; then
  echo "Usage:"
  echo "  $0 <ratio> <finger>                  # side defaults to left_hand"
  echo "  $0 <side> <ratio> <finger>"
  echo "Examples:"
  echo "  $0 1.0 4"
  echo "  $0 right_hand 0.5 2"
  exit 2
fi

SRV="/inspire_hand/set_angle_flexible/${SIDE}"
PAYLOAD="{name: ['${FINGER}'], angleRatio: [${RATIO}]}"

echo "Calling: ${SRV}  ${PAYLOAD}"
rosservice call "${SRV}" "${PAYLOAD}"
