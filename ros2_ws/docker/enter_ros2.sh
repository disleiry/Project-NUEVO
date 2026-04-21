#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/../.." && pwd)"

usage() {
    cat <<'EOF'
Usage:
  ros2_ws/docker/enter_ros2.sh [rpi|vm|jetson|/path/to/docker-compose.yml]

Behavior:
  - If COMPOSE is set, it takes precedence.
  - Otherwise:
      rpi    -> ros2_ws/docker/docker-compose.rpi.yml    (service: ros2_runtime)
      vm     -> ros2_ws/docker/docker-compose.vm.yml     (service: ros2_runtime)
      jetson -> ros2_ws/docker/docker-compose.jetson.yml (service: global_gps)
  - Default is rpi.
  - The script runs `docker compose up -d --build --wait` for the selected
    service before opening a shell, so the container is fully started and
    healthy before you enter it.

Examples:
  ./ros2_ws/docker/enter_ros2.sh
  ./ros2_ws/docker/enter_ros2.sh rpi
  ./ros2_ws/docker/enter_ros2.sh jetson
  COMPOSE=ros2_ws/docker/docker-compose.rpi.yml ./ros2_ws/docker/enter_ros2.sh
EOF
}

target="${1:-rpi}"

if [[ "${target}" == "-h" || "${target}" == "--help" ]]; then
    usage
    exit 0
fi

service="ros2_runtime"

if [[ -n "${COMPOSE:-}" ]]; then
    compose_file="${COMPOSE}"
else
    case "${target}" in
        rpi)
            compose_file="${repo_root}/ros2_ws/docker/docker-compose.rpi.yml"
            ;;
        vm)
            compose_file="${repo_root}/ros2_ws/docker/docker-compose.vm.yml"
            ;;
        jetson)
            compose_file="${repo_root}/ros2_ws/docker/docker-compose.jetson.yml"
            service="global_gps"
            ;;
        *.yml|*.yaml)
            compose_file="${target}"
            ;;
        *)
            echo "Unknown target: ${target}" >&2
            usage >&2
            exit 1
            ;;
    esac
fi

echo "[enter_ros2] Starting ${service} with docker compose up -d --build --wait..."
docker compose -f "${compose_file}" up -d --build --wait "${service}"

exec docker compose -f "${compose_file}" exec "${service}" bash -lc '
source /opt/ros/jazzy/setup.bash
cd /ros2_ws

if [[ -f /ros2_ws/install/setup.bash ]]; then
    source /ros2_ws/install/setup.bash
fi

exec bash -i
'
