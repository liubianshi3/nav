#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${A2_WORKSPACE:-$HOME/ws/device-navigation}"
DLIO_REPO="${A2_DLIO_REPO:-https://github.com/vectr-ucla/direct_lidar_inertial_odometry.git}"
DLIO_BRANCH="${A2_DLIO_BRANCH:-feature/ros2}"
DLIO_DIR="${WORKSPACE}/src/third_party/direct_lidar_inertial_odometry"
SKIP_APT="${A2_SKIP_APT:-0}"
SKIP_BUILD=0

usage() {
  cat <<EOF
Usage:
  $(basename "$0") [--skip-apt] [--no-build]

Installs the ROS2 feature branch of VECTR-UCLA DLIO into:
  ${DLIO_DIR}

Environment overrides:
  A2_WORKSPACE
  A2_DLIO_REPO
  A2_DLIO_BRANCH
  A2_SKIP_APT
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --skip-apt)
      SKIP_APT=1
      shift
      ;;
    --no-build)
      SKIP_BUILD=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[ERROR] unknown argument: $1" >&2
      usage
      exit 2
      ;;
  esac
done

log() {
  printf '[INFO] %s\n' "$*"
}

die() {
  printf '[ERROR] %s\n' "$*" >&2
  exit 1
}

command -v git >/dev/null 2>&1 || die "git is required"
command -v colcon >/dev/null 2>&1 || die "colcon is required"
[[ -f /opt/ros/humble/setup.bash ]] || die "ROS2 Humble is required at /opt/ros/humble/setup.bash"
[[ -d "$WORKSPACE/src" ]] || die "workspace src directory not found: $WORKSPACE/src"

log "Installing ROS/PCL dependencies when apt is available"
if [[ "$SKIP_APT" == "1" ]]; then
  log "Skipping apt dependency installation"
elif command -v apt-get >/dev/null 2>&1; then
  sudo apt-get update
  sudo apt-get install -y \
    libeigen3-dev \
    libomp-dev \
    libpcl-dev \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    ros-humble-tf2-ros \
    ros-humble-nav-msgs \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs
else
  log "apt-get not found; skipping system dependency installation"
fi

if [[ -d "$DLIO_DIR/.git" ]]; then
  log "DLIO repository already exists at $DLIO_DIR"
  git -C "$DLIO_DIR" fetch origin "$DLIO_BRANCH"
  git -C "$DLIO_DIR" checkout "$DLIO_BRANCH"
  git -C "$DLIO_DIR" pull --ff-only origin "$DLIO_BRANCH"
elif [[ -e "$DLIO_DIR" ]]; then
  die "DLIO target exists but is not a git repository: $DLIO_DIR"
else
  mkdir -p "$(dirname "$DLIO_DIR")"
  git clone --branch "$DLIO_BRANCH" "$DLIO_REPO" "$DLIO_DIR"
fi

log "DLIO commit: $(git -C "$DLIO_DIR" rev-parse --short HEAD)"
if [[ "$SKIP_BUILD" -eq 1 ]]; then
  log "Skipping DLIO build"
  exit 0
fi
cd "$WORKSPACE"
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select direct_lidar_inertial_odometry
log "DLIO installed. Source: source ${WORKSPACE}/install/setup.bash"
