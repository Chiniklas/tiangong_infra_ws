#!/usr/bin/env bash
set -euo pipefail

# Usage: ./multi_finger_control.sh <ratio> <fingers...> --
RATIO="${1:-}"; shift || true
[[ -z "${RATIO}" ]] && { echo "Error: ratio missing"; exit 2; }

FINGERS=()
for a in "$@"; do
  [[ "$a" == "--" ]] && break
  FINGERS+=("$a")
done

# default if none provided
[[ ${#FINGERS[@]} -eq 0 ]] && FINGERS=(1 2 3 4 5 6)

# validate
for f in "${FINGERS[@]}"; do
  [[ "$f" =~ ^[0-6]$ ]] || { echo "Error: finger id '$f' must be in 0..6."; exit 2; }
done

SRV="/inspire_hand/set_angle_flexible/left_hand"
NAMES="$(printf " '%s'," "${FINGERS[@]}")"; NAMES="[${NAMES%,}]"
RATIOS=""
for _ in "${FINGERS[@]}"; do RATIOS+=" ${RATIO},"; done; RATIOS="[${RATIOS%,}]"

PAYLOAD="{name: ${NAMES}, angleRatio: ${RATIOS}}"
echo "Calling: ${SRV}  ${PAYLOAD}"
rosservice call "${SRV}" "${PAYLOAD}"
