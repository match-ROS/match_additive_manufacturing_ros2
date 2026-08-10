#!/usr/bin/env bash
# Bootstrap a ROS 2 Jazzy workspace for one MuR620 (MiR + two UR10 arms).
#
# This intentionally does not install Robotnik/Bunker sources and does not
# launch hardware. Review the resolved robot profile and commissioning manual
# before enabling any physical device.
set -Eeuo pipefail

ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-jazzy}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT_AM_REPO="$(cd "${SCRIPT_DIR}/.." && pwd)"
# Expected script location: <workspace>/src/match_additive_manufacturing_ros2/scripts.
# Use that workspace by default so no second workspace is created unexpectedly.
DEFAULT_WORKSPACE="$(cd "${SCRIPT_AM_REPO}/../.." && pwd)"
WORKSPACE="${MUR620_WORKSPACE:-${DEFAULT_WORKSPACE}}"
INSTALL_SYSTEM=false
USE_APT_UR_DRIVER=false
SKIP_ROSDEP=false
BUILD=true
HOST_CHECK=false

usage() {
  cat <<'EOF'
Usage: setup_mur620_workspace.sh [options]

Create or update the ROS 2 Jazzy workspace containing this AM repository (or
the explicit --workspace), with MuR620 sources but no Robotnik/Bunker source.

Options:
  --workspace PATH       Workspace root (default: workspace containing this script)
  --install-system        Install required apt packages and initialize rosdep.
  --use-apt-ur-driver     Use ros-jazzy-ur-robot-driver instead of UR source submodules.
  --skip-rosdep           Do not run rosdep install.
  --no-build              Fetch and resolve only; do not run colcon build.
  --host-check            Run the MuR host checker after a successful build.
  -h, --help              Show this help.

Examples:
  ./scripts/setup_mur620_workspace.sh --install-system
  ./scripts/setup_mur620_workspace.sh --workspace /opt/mur620_ws --host-check

The script never launches a robot. Follow docs/mur620_install.md for staged
hardware commissioning after setup completes.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --workspace)
      shift
      WORKSPACE="${1:?missing workspace path}"
      ;;
    --install-system) INSTALL_SYSTEM=true ;;
    --use-apt-ur-driver) USE_APT_UR_DRIVER=true ;;
    --skip-rosdep) SKIP_ROSDEP=true ;;
    --no-build) BUILD=false ;;
    --host-check) HOST_CHECK=true ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

if [[ ! -f /etc/os-release ]]; then
  echo 'Cannot determine operating system.' >&2
  exit 1
fi
. /etc/os-release
if [[ "${UBUNTU_CODENAME:-${VERSION_CODENAME:-}}" != 'noble' ]]; then
  echo 'This setup is supported only on Ubuntu 24.04 (noble) with ROS 2 Jazzy.' >&2
  exit 1
fi

install_system_packages() {
  sudo apt update
  sudo apt install -y \
    git python3-rosdep python3-vcstool python3-colcon-common-extensions \
    "ros-${ROS_DISTRO_NAME}-desktop" \
    "ros-${ROS_DISTRO_NAME}-ros2-control" \
    "ros-${ROS_DISTRO_NAME}-ros2-controllers" \
    "ros-${ROS_DISTRO_NAME}-ros2controlcli" \
    "ros-${ROS_DISTRO_NAME}-xacro" \
    "ros-${ROS_DISTRO_NAME}-robot-state-publisher" \
    "ros-${ROS_DISTRO_NAME}-navigation2" \
    "ros-${ROS_DISTRO_NAME}-nav2-bringup" \
    "ros-${ROS_DISTRO_NAME}-robot-localization" \
    "ros-${ROS_DISTRO_NAME}-moveit" \
    "ros-${ROS_DISTRO_NAME}-tf-transformations" \
    "ros-${ROS_DISTRO_NAME}-pcl-ros" \
    "ros-${ROS_DISTRO_NAME}-pcl-conversions" \
    "ros-${ROS_DISTRO_NAME}-pcl-msgs" \
    python3-can python3-serial python3-websocket
  if [[ "${USE_APT_UR_DRIVER}" == true ]]; then
    sudo apt install -y "ros-${ROS_DISTRO_NAME}-ur-robot-driver"
  fi
  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo rosdep init
  fi
  rosdep update
}

if [[ "${INSTALL_SYSTEM}" == true ]]; then
  install_system_packages
fi

if ! command -v colcon >/dev/null || ! command -v vcs >/dev/null || ! command -v rosdep >/dev/null; then
  cat >&2 <<'EOF'
Missing ROS development tools. Re-run with --install-system after ROS 2 Jazzy
apt sources are configured, or install colcon, vcstool, and rosdep manually.
EOF
  exit 1
fi

mkdir -p "${WORKSPACE}/src"
AM_REPO="${WORKSPACE}/src/match_additive_manufacturing_ros2"
MOBILE_REPO="${WORKSPACE}/src/match_mobile_robotics_jazzy"

clone_if_missing() {
  local url="$1"
  local destination="$2"
  if [[ -d "${destination}/.git" ]]; then
    echo "Using existing repository: ${destination}"
  elif [[ -e "${destination}" ]]; then
    echo "Refusing to overwrite non-git path: ${destination}" >&2
    exit 1
  else
    git clone "$url" "$destination"
  fi
}

if [[ "${SCRIPT_AM_REPO}" == "${AM_REPO}" ]]; then
  echo "Using AM repository containing this script: ${AM_REPO}"
else
  clone_if_missing https://github.com/match-ROS/match_additive_manufacturing_ros2.git "${AM_REPO}"
fi
clone_if_missing https://github.com/match-ROS/match_mobile_robotics_jazzy.git "${MOBILE_REPO}"

if [[ "${USE_APT_UR_DRIVER}" == false ]]; then
  git -C "${MOBILE_REPO}" submodule update --init --recursive \
    ur_robot/Universal_Robots_ROS2_Driver \
    ur_robot/Universal_Robots_Client_Library
fi
git -C "${MOBILE_REPO}" submodule update --init --recursive \
  ewellix/ewellix_lift ewellix/ewellix_lift_common ewellix/serial

vcs import "${WORKSPACE}/src" < "${MOBILE_REPO}/ros2.repos"

# ROS setup scripts intentionally probe optional variables such as
# AMENT_TRACE_SETUP_FILES.  They are not nounset-safe, while this bootstrap is.
# Temporarily disable nounset only for that third-party setup script.
set +u
source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
set -u
if [[ "${SKIP_ROSDEP}" == false ]]; then
  rosdep install --from-paths "${WORKSPACE}/src" --ignore-src -r -y \
    --rosdistro "${ROS_DISTRO_NAME}" \
    --skip-keys 'controllers_ros2 robotnik_rbvogui_tum'
fi

if [[ "${BUILD}" == true ]]; then
  BASE_PATHS=(
    "${AM_REPO}"
    "${MOBILE_REPO}/mir_robot"
    "${MOBILE_REPO}/mur_robot"
    "${MOBILE_REPO}/ewellix/ewellix_lift"
    "${MOBILE_REPO}/ewellix/ewellix_lift_common"
    "${MOBILE_REPO}/ewellix/serial"
    "${WORKSPACE}/src/ira_laser_tools"
  )
  if [[ "${USE_APT_UR_DRIVER}" == false ]]; then
    BASE_PATHS+=(
      "${MOBILE_REPO}/ur_robot/Universal_Robots_Client_Library"
      "${MOBILE_REPO}/ur_robot/Universal_Robots_ROS2_Driver"
    )
  fi
  cd "${WORKSPACE}"
  for required_package in am_operator_gui mur_launch_hardware; do
    if ! colcon list --base-paths "${BASE_PATHS[@]}" --names-only | grep -qx "${required_package}"; then
      echo "Required package ${required_package} was not discovered in the configured source roots." >&2
      exit 1
    fi
  done
  colcon build --symlink-install \
    --base-paths "${BASE_PATHS[@]}" \
    --packages-up-to mur_launch_hardware am_operator_gui \
    --cmake-args -DCMAKE_POSITION_INDEPENDENT_CODE=ON
  if [[ ! -f "${WORKSPACE}/install/setup.bash" ]]; then
    cat >&2 <<EOF
colcon returned without creating ${WORKSPACE}/install/setup.bash.
Inspect ${WORKSPACE}/log/latest_build for the first package error.
EOF
    exit 1
  fi
  # Colcon-generated setup scripts, like the ROS underlay, probe optional trace
  # variables and therefore cannot be sourced with nounset enabled.
  set +u
  source "${WORKSPACE}/install/setup.bash"
  set -u
fi

if [[ "${HOST_CHECK}" == true ]]; then
  WS="${WORKSPACE}" REPO="${MOBILE_REPO}" \
    bash "${MOBILE_REPO}/setup_mur_hardware_host.sh" --check
fi

cat <<EOF

Setup complete.

Next safe step:
  source ${WORKSPACE}/install/setup.bash
  WS=${WORKSPACE} REPO=${MOBILE_REPO} bash ${MOBILE_REPO}/setup_mur_hardware_host.sh --check

Do not launch AM following on physical hardware yet. Continue with the staged
checks in ${AM_REPO}/docs/mur620_install.md.
EOF
