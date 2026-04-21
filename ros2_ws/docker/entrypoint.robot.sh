#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
# entrypoint.robot.sh — Container entrypoint for the ROS2 robot workspace
#
# Builds the ROS2 workspace on startup, then leaves the container idle so users
# can start nodes manually with `docker compose exec`.
# Build artifacts are cached in named Docker volumes (build/ and install/),
# so only the first startup is slow.
# ─────────────────────────────────────────────────────────────────────────────
set -e

source /opt/ros/jazzy/setup.bash
cd /ros2_ws

packages=( ${NUEVO_ROS_PACKAGES:-robot sensors bridge bridge_interfaces vision rplidar_ros} )

echo "[entrypoint] Building ROS2 packages: ${packages[*]}"
colcon build \
    --symlink-install \
    --packages-select "${packages[@]}" \
    --cmake-args -DBUILD_TESTING=OFF

source /ros2_ws/install/setup.bash

echo "[entrypoint] ROS_DISTRO=${ROS_DISTRO}"
echo "[entrypoint] Launching: $*"
exec "$@"
