#!/usr/bin/env bash
# Bootstrap a ROS Noetic catkin workspace at /tiangong_infra_ws
# - Sources /opt/ros/noetic
# - rosdep init/update (safe if already initialized)
# - Installs system deps for packages in src/
# - Configures & builds with catkin tools
# - Ensures future shells auto-source the overlay

set -euo pipefail

WS="${WS:-/tiangong_infra_ws}"
SRC="${WS}/src"
ROS_SETUP="/opt/ros/noetic/setup.bash"
OVERLAY_SETUP="${WS}/devel/setup.bash"

say() { printf "\033[1;36m[bootstrap]\033[0m %s\n" "$*"; }
warn(){ printf "\033[1;33m[warning]\033[0m %s\n" "$*"; }

# 0) Source ROS Noetic
if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "ROS Noetic not found at ${ROS_SETUP}. Is this the Noetic image?"; exit 1
fi
# shellcheck disable=SC1090
source "${ROS_SETUP}"
say "ROS_DISTRO = ${ROS_DISTRO:-<unset>} (expect: noetic)"

# 1) rosdep init/update (idempotent)
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  say "rosdep init…"
  rosdep init
else
  say "rosdep already initialized; skipping init."
fi
say "rosdep update…"
rosdep update

# 2) Install system deps for packages in src/
if [[ -d "${SRC}" ]] && [[ -n "$(ls -A "${SRC}" 2>/dev/null || true)" ]]; then
  say "Installing system dependencies for packages in ${SRC}…"
  rosdep install --from-paths "${SRC}" --ignore-src -r -y
else
  warn "No packages found in ${SRC}; skipping rosdep install."
fi

# 3) Configure & build with catkin tools
say "Configuring catkin workspace…"
catkin config --workspace "${WS}" --extend /opt/ros/noetic \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
say "Building workspace…"
catkin build -w "${WS}"

# 4) Auto-source overlay for future shells
if ! grep -qsF "${OVERLAY_SETUP}" ~/.bashrc; then
  say "Adding overlay sourcing to ~/.bashrc"
  echo "if [ -f ${OVERLAY_SETUP} ]; then source ${OVERLAY_SETUP}; fi" >> ~/.bashrc
fi

# 5) Source overlay for this shell (if running interactively)
if [[ -f "${OVERLAY_SETUP}" ]]; then
  # shellcheck disable=SC1090
  source "${OVERLAY_SETUP}"
  say "Overlay sourced: ${OVERLAY_SETUP}"
fi

say "Done. You can now run: roscore  (in one tab) and rviz (in another)."
