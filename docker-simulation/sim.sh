#!/usr/bin/env bash
# Friendly wrapper around `docker compose` for the OpenMower simulation stack.
# Run `./sim.sh help` (or just `./sim.sh` with no args) for usage.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"

usage() {
    cat <<'EOF'
Usage: ./sim.sh <command> [args]

Commands:
  up              Start the simulation stack in the background.
                   (First run builds mower_simulation_gui locally - can take a
                   few minutes. Does NOT rebuild on later runs, even if you
                   changed source - use `rebuild` for that.)
  down            Stop the stack and remove its containers.
  restart         Shortcut for `down` followed by `up`.
  rebuild         Force-rebuild mower_simulation_gui (and open_mower_ros, if
                   BASE_IMAGE points at a local build) from source, then start
                   the stack. Use this after editing code/config that's baked
                   into the image, or after switching VERSION/BASE_IMAGE.
  build-from-source
                   Build open_mower_ros from your local checkout instead of a
                   published tag. Run this once, then set
                   BASE_IMAGE=local/open_mower_ros:local in .env and
                   `./sim.sh rebuild`.
  logs [service]  Follow logs. With no service, follows everything.
                   Common services: mower_simulation_gui, open_mower_ros
  ps              Show container status (check mower_simulation_gui is
                   "healthy" if the stack seems stuck).
  reset           Discard local map/param edits, restoring the checked-in
                   starter map and config.
  clean           DESTRUCTIVE: wipe all simulation state (ROS home, logs,
                   recordings, map, params) back to an empty ./data/. Asks
                   for confirmation first.
  down-volumes    Stop the stack and remove containers + named volumes
                   (bind-mounted ./data/ is untouched either way).
  help            Show this message.

Examples:
  ./sim.sh up                       # first run / normal start
  ./sim.sh logs open_mower_ros      # tail one service's logs
  ./sim.sh rebuild                  # picked up a code change, rebuild + restart
EOF
}

require_docker() {
    if ! command -v docker >/dev/null 2>&1; then
        echo "error: 'docker' is not installed or not on PATH. Install Docker first: https://docs.docker.com/get-docker/" >&2
        exit 1
    fi
    if ! docker compose version >/dev/null 2>&1; then
        echo "error: 'docker compose' (the v2 plugin) isn't available. Update Docker Desktop/Engine, or install the compose plugin." >&2
        exit 1
    fi
    if ! docker info >/dev/null 2>&1; then
        echo "error: could not talk to the Docker daemon. Is Docker running? Do you need 'sudo' or to be in the 'docker' group?" >&2
        exit 1
    fi
}

confirm() {
    read -r -p "$1 [y/N] " reply
    case "$reply" in
        [yY]|[yY][eE][sS]) return 0 ;;
        *) return 1 ;;
    esac
}

cmd="${1:-help}"
shift || true

case "$cmd" in
    up)
        require_docker
        echo "Starting the simulation stack..."
        docker compose up -d "$@"
        echo
        echo "Up. Give mower_simulation_gui a minute to report healthy (./sim.sh ps), then open:"
        echo "  http://localhost:${NOVNC_PORT:-6080}  - simulation view (noVNC)"
        echo "  http://localhost:8080                 - OpenMowerApp"
        ;;
    down)
        require_docker
        docker compose down "$@"
        ;;
    down-volumes)
        require_docker
        docker compose down -v "$@"
        ;;
    restart)
        require_docker
        docker compose down
        docker compose up -d "$@"
        ;;
    rebuild)
        require_docker
        echo "Rebuilding images from source (this can take a few minutes)..."
        docker compose build "$@"
        docker compose up -d
        ;;
    build-from-source)
        require_docker
        docker compose --profile build-from-source build build_from_source "$@"
        echo
        echo "Built local/open_mower_ros:local. Now set BASE_IMAGE=local/open_mower_ros:local in .env,"
        echo "then run: ./sim.sh rebuild"
        ;;
    logs)
        require_docker
        docker compose logs -f "$@"
        ;;
    ps|status)
        require_docker
        docker compose ps "$@"
        ;;
    reset)
        git checkout -- data/ros/map.json data/params/custom_params.yaml
        echo "Restored data/ros/map.json and data/params/custom_params.yaml to their checked-in state."
        ;;
    clean)
        if confirm "This deletes everything under ./data/ (map, params, ROS home, recordings). Continue?"; then
            git clean -fdx data/
            echo "./data/ wiped. Run './sim.sh up' to recreate it."
        else
            echo "Cancelled."
        fi
        ;;
    help|-h|--help)
        usage
        ;;
    *)
        echo "error: unknown command '$cmd'"
        echo
        usage
        exit 1
        ;;
esac
