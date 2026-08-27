#!/usr/bin/env bash
# ROS 2 course environment installer for Ubuntu 24.04 (Noble) and ROS 2 Jazzy.

set -Eeuo pipefail
IFS=$'\n\t'

TARGET_UBUNTU_CODENAME="noble"
TARGET_ROS_DISTRO="jazzy"
CARLA_VERSION="0.9.16"
CARLA_BRIDGE_COMMIT="e9063d97ff5a724f76adbb1b852dc71da1dcfeec"

COURSE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COURSE_WS="${ROS2_COURSE_WS:-${HOME}/ros2_course_ws}"
COURSE_SRC="${COURSE_ROOT}/src"
LAB_SRC="${COURSE_ROOT}/src/lab_code"
ML_VENV="${ROS2_COURSE_ML_VENV:-${HOME}/.venvs/ros2-course-ml}"
CARLA_VENV="${ROS2_COURSE_CARLA_VENV:-${HOME}/.venvs/carla-${CARLA_VERSION}}"
CARLA_DIR="${CARLA_ROOT:-${HOME}/carla}"
CARLA_WS="${CARLA_BRIDGE_WS:-${HOME}/carla_ws}"
ENV_DIR="${XDG_CONFIG_HOME:-${HOME}/.config}/ros2-course"
ENV_FILE="${ENV_DIR}/env.bash"

WITH_ML=false
WITH_HARDWARE=false
WITH_CARLA=false
RUN_TESTS=false
VERIFY_ONLY=false
DRY_RUN=false
CARLA_ONLY=false

CURRENT_STEP="startup"
LOCK_FD=9

readonly BASHRC_BEGIN="# >>> ROS2 course environment >>>"
readonly BASHRC_END="# <<< ROS2 course environment <<<"

BASE_APT_PACKAGES=(
  build-essential
  ca-certificates
  cmake
  curl
  flake8
  git
  gnupg
  libomp5
  locales
  lsb-release
  mesa-utils
  pkg-config
  python3-colcon-common-extensions
  python3-dev
  python3-matplotlib
  python3-numpy
  python3-opencv
  python3-pip
  python3-pytest
  python3-pytest-cov
  python3-requests
  python3-rosdep
  python3-scipy
  python3-setuptools
  python3-sklearn
  python3-vcstool
  python3-venv
  python3-wheel
  python3-yaml
  rsync
  shellcheck
  software-properties-common
  tar
  tmux
  unzip
  wget
  xz-utils
)

BASE_ROS_PACKAGES=(
  "ros-${TARGET_ROS_DISTRO}-demo-nodes-py"
  "ros-${TARGET_ROS_DISTRO}-image-tools"
  "ros-${TARGET_ROS_DISTRO}-launch-testing"
  "ros-${TARGET_ROS_DISTRO}-launch-testing-ament-cmake"
  "ros-${TARGET_ROS_DISTRO}-nav2-bringup"
  "ros-${TARGET_ROS_DISTRO}-navigation2"
  "ros-${TARGET_ROS_DISTRO}-robot-localization"
  "ros-${TARGET_ROS_DISTRO}-ros-testing"
  "ros-${TARGET_ROS_DISTRO}-rosbag2"
  "ros-${TARGET_ROS_DISTRO}-rqt-image-view"
  "ros-${TARGET_ROS_DISTRO}-teleop-twist-keyboard"
  "ros-${TARGET_ROS_DISTRO}-tf-transformations"
)

HARDWARE_APT_PACKAGES=(
  python3-serial
  "ros-${TARGET_ROS_DISTRO}-aruco-opencv"
  "ros-${TARGET_ROS_DISTRO}-aruco-opencv-msgs"
  "ros-${TARGET_ROS_DISTRO}-aruco-ros"
  "ros-${TARGET_ROS_DISTRO}-find-object-2d"
  "ros-${TARGET_ROS_DISTRO}-realsense2-camera"
  "ros-${TARGET_ROS_DISTRO}-usb-cam"
)

HARDWARE_ROSDEP_KEYS=(
  aruco_opencv
  aruco_opencv_msgs
  aruco_ros
  find_object_2d
  realsense2_camera
  usb_cam
)

BASE_ROSDEP_SKIP_KEYS=(
  ament_python
)

ML_PIP_PACKAGES=(
  "evo==1.31.1"
  "filterpy==1.4.5"
  "openai>=1.0,<3"
  "ultralytics>=8.3,<9"
)

usage() {
  cat <<'EOF'
Usage: bash setup_course.sh [options]

Default action:
  Install ROS 2 Jazzy and base dependencies, synchronize the course into
  ~/ros2_course_ws, build all ROS packages, configure ~/.bashrc, and verify.

Options:
  --with-ml             Install ML dependencies in an isolated venv
  --with-hardware       Install camera, RealSense, and ArUco dependencies
  --with-carla          Install CARLA 0.9.16 and the pinned ROS bridge
  --all-profiles        Enable ML, hardware, and CARLA profiles
  --workspace PATH      Use a managed workspace other than ~/ros2_course_ws
  --run-tests           Run colcon tests after a successful build
  --verify              Verify an existing installation without changing it
  --dry-run             Print mutating commands without executing them
  --help                Show this help

Legacy aliases:
  --ros2-only           Same as the default profile
  --skip-carla          Accepted; CARLA is already opt-in
  --carla-only          Install only ROS prerequisites and the CARLA profile

Environment overrides:
  ROS2_COURSE_WS, ROS2_COURSE_ML_VENV, ROS2_COURSE_CARLA_VENV,
  CARLA_ROOT, CARLA_BRIDGE_WS, CARLA_SHA256, CARLA_ARCHIVE_URL
EOF
}

log_info() {
  printf '[INFO] %s\n' "$*"
}

log_ok() {
  printf '[ OK ] %s\n' "$*"
}

log_warn() {
  printf '[WARN] %s\n' "$*" >&2
}

die() {
  printf '[FAIL] %s\n' "$*" >&2
  exit 1
}

on_error() {
  local exit_code=$?
  local line_no=$1
  local command=$2
  printf '[FAIL] Step "%s" failed at line %s (exit=%s): %s\n' \
    "${CURRENT_STEP}" "${line_no}" "${exit_code}" "${command}" >&2
  exit "${exit_code}"
}

trap 'on_error "${LINENO}" "${BASH_COMMAND}"' ERR

print_command() {
  printf '[DRY-RUN]'
  printf ' %q' "$@"
  printf '\n'
}

run() {
  if [[ "${DRY_RUN}" == true ]]; then
    print_command "$@"
    return 0
  fi
  "$@"
}

parse_args() {
  while (($# > 0)); do
    case "$1" in
      --with-ml)
        WITH_ML=true
        ;;
      --with-hardware)
        WITH_HARDWARE=true
        ;;
      --with-carla)
        WITH_CARLA=true
        ;;
      --all-profiles)
        WITH_ML=true
        WITH_HARDWARE=true
        WITH_CARLA=true
        ;;
      --workspace)
        (($# >= 2)) || die "--workspace requires a path"
        COURSE_WS="$2"
        shift
        ;;
      --run-tests)
        RUN_TESTS=true
        ;;
      --verify)
        VERIFY_ONLY=true
        ;;
      --dry-run)
        DRY_RUN=true
        ;;
      --ros2-only)
        log_warn "--ros2-only is deprecated; the default profile has the same behavior"
        ;;
      --skip-carla)
        log_warn "--skip-carla is deprecated; CARLA is installed only with --with-carla"
        ;;
      --carla-only)
        CARLA_ONLY=true
        WITH_CARLA=true
        ;;
      --help|-h)
        usage
        exit 0
        ;;
      *)
        die "Unknown option: $1"
        ;;
    esac
    shift
  done

  COURSE_WS="${COURSE_WS/#\~/${HOME}}"
  [[ "${COURSE_WS}" == /* ]] || die "Workspace path must be absolute: ${COURSE_WS}"
}

preflight() {
  CURRENT_STEP="environment preflight"

  [[ "$(uname -s)" == "Linux" ]] || die "This installer requires Ubuntu Linux or WSL 2"
  [[ -r /etc/os-release ]] || die "Cannot read /etc/os-release"

  # shellcheck disable=SC1091
  source /etc/os-release
  [[ "${ID:-}" == "ubuntu" ]] || die "Unsupported operating system: ${ID:-unknown}"
  if [[ "${VERSION_CODENAME:-}" != "${TARGET_UBUNTU_CODENAME}" ]]; then
    if [[ "${DRY_RUN}" == true ]]; then
      log_warn "Dry-run platform differs from target: ${VERSION_ID:-unknown} (${VERSION_CODENAME:-unknown})"
    else
      die "Ubuntu 24.04 Noble is required; detected ${VERSION_ID:-unknown} (${VERSION_CODENAME:-unknown})"
    fi
  fi

  [[ "${EUID}" -ne 0 ]] || die "Run this script as a normal user; sudo is invoked only when needed"
  [[ -d "${COURSE_SRC}" ]] || die "Course src directory not found: ${COURSE_SRC}"
  [[ -d "${LAB_SRC}" ]] || die "Course lab_code directory not found: ${LAB_SRC}"

  if [[ -n "${ROS_DISTRO:-}" && "${ROS_DISTRO}" != "${TARGET_ROS_DISTRO}" ]]; then
    die "Another ROS distribution is active (${ROS_DISTRO}); start a clean shell"
  fi

  if grep -qi microsoft /proc/version 2>/dev/null; then
    log_info "WSL detected; the managed workspace will stay on the Linux filesystem"
  fi

  local available_kb
  available_kb="$(df -Pk "${HOME}" | awk 'NR == 2 {print $4}')"
  if ((available_kb < 15728640)); then
    log_warn "Less than 15 GiB is available under ${HOME}"
  fi
  if [[ "${WITH_CARLA}" == true && "${available_kb}" -lt 31457280 ]]; then
    log_warn "The CARLA profile should have at least 30 GiB free"
  fi

  log_ok "Platform: Ubuntu Noble, target ROS 2 ${TARGET_ROS_DISTRO}"
  log_info "Course root: ${COURSE_ROOT}"
  log_info "Managed workspace: ${COURSE_WS}"
}

acquire_lock() {
  [[ "${DRY_RUN}" == true || "${VERIFY_ONLY}" == true ]] && return 0
  command -v flock >/dev/null 2>&1 || die "flock is required (package: util-linux)"
  local lock_file="${XDG_RUNTIME_DIR:-/tmp}/ros2-course-setup-${UID}.lock"
  exec 9>"${lock_file}"
  flock -n "${LOCK_FD}" || die "Another setup_course.sh process is running"
}

apt_install() {
  (($# > 0)) || return 0
  run sudo env DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends "$@"
}

install_base_tools() {
  CURRENT_STEP="base system dependencies"
  log_info "Installing base build and Python dependencies"
  run sudo apt-get update
  apt_install "${BASE_APT_PACKAGES[@]}"
  run sudo locale-gen en_US en_US.UTF-8
  run sudo update-locale LANG=en_US.UTF-8
  export LANG=en_US.UTF-8
}

configure_ros_repository() {
  local source_file="/etc/apt/sources.list.d/ros2.list"
  local keyring="/usr/share/keyrings/ros-archive-keyring.gpg"

  if [[ -s "${source_file}" && -s "${keyring}" ]]; then
    log_ok "ROS 2 apt repository is already configured"
    return 0
  fi

  if [[ "${DRY_RUN}" == true ]]; then
    log_info "Would configure the official ROS 2 apt repository for Noble"
    return 0
  fi

  local key_tmp source_tmp
  key_tmp="$(mktemp)"
  source_tmp="$(mktemp)"
  curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o "${key_tmp}"
  sudo gpg --dearmor --yes -o "${keyring}" "${key_tmp}"
  printf 'deb [arch=%s signed-by=%s] http://packages.ros.org/ros2/ubuntu %s main\n' \
    "$(dpkg --print-architecture)" "${keyring}" "${TARGET_UBUNTU_CODENAME}" > "${source_tmp}"
  sudo install -m 0644 "${source_tmp}" "${source_file}"
  rm -f "${key_tmp}" "${source_tmp}"
}

install_ros() {
  CURRENT_STEP="ROS 2 Jazzy installation"
  configure_ros_repository
  run sudo apt-get update
  apt_install \
    "ros-${TARGET_ROS_DISTRO}-desktop" \
    "ros-${TARGET_ROS_DISTRO}-rmw-cyclonedds-cpp" \
    ros-dev-tools

  if [[ "${DRY_RUN}" == false ]]; then
    [[ -f "/opt/ros/${TARGET_ROS_DISTRO}/setup.bash" ]] || \
      die "ROS installation completed without /opt/ros/${TARGET_ROS_DISTRO}/setup.bash"
  fi
}

source_ros() {
  local setup_file="/opt/ros/${TARGET_ROS_DISTRO}/setup.bash"
  if [[ ! -f "${setup_file}" ]]; then
    [[ "${DRY_RUN}" == true ]] && return 0
    die "ROS setup file not found: ${setup_file}"
  fi

  set +u
  # shellcheck disable=SC1090
  source "${setup_file}"
  set -u
  [[ "${ROS_DISTRO:-}" == "${TARGET_ROS_DISTRO}" ]] || die "Failed to activate ROS 2 Jazzy"
}

initialize_rosdep() {
  CURRENT_STEP="rosdep initialization"
  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    run sudo rosdep init
  fi
  run rosdep update --rosdistro "${TARGET_ROS_DISTRO}"
}

discover_package_names() {
  colcon list --base-paths "$@" | awk '{print $1}' | LC_ALL=C sort
}

course_package_base_paths() {
  find "${COURSE_SRC}" -mindepth 1 -maxdepth 1 -type d \
    ! -path "${LAB_SRC}" -print | LC_ALL=C sort
}

discover_source_package_names() {
  local course_base_paths=()
  mapfile -t course_base_paths < <(course_package_base_paths)
  discover_package_names "${course_base_paths[@]}" "${LAB_SRC}"
}

assert_unique_packages() {
  local duplicate_names
  local course_base_paths=()
  mapfile -t course_base_paths < <(course_package_base_paths)
  duplicate_names="$(colcon list --base-paths "${course_base_paths[@]}" "${LAB_SRC}" | \
    awk '{print $1}' | LC_ALL=C sort | uniq -d)"
  [[ -z "${duplicate_names}" ]] || die "Duplicate ROS package names detected: ${duplicate_names}"
}

sync_workspace() {
  CURRENT_STEP="managed workspace synchronization"
  local marker="${COURSE_WS}/.ros2-course-managed"
  local excludes=(
    --exclude=.git/
    --exclude=.venv/
    --exclude=__pycache__/
    --exclude='*.pyc'
    --exclude=build/
    --exclude=install/
    --exclude=log/
    --exclude=lab_code/
  )

  if [[ "${DRY_RUN}" == true ]]; then
    print_command mkdir -p "${COURSE_WS}/src/course" "${COURSE_WS}/src/labs"
    print_command rsync -a --delete "${excludes[@]}" "${COURSE_SRC}/" "${COURSE_WS}/src/course/"
    print_command rsync -a --delete "${excludes[@]}" "${LAB_SRC}/" "${COURSE_WS}/src/labs/"
    return 0
  fi

  assert_unique_packages

  if [[ -d "${COURSE_WS}" && ! -f "${marker}" ]]; then
    if [[ -n "$(find "${COURSE_WS}" -mindepth 1 -maxdepth 1 -print -quit)" ]]; then
      die "Refusing to modify an unmanaged non-empty workspace: ${COURSE_WS}"
    fi
  fi

  mkdir -p "${COURSE_WS}/src/course" "${COURSE_WS}/src/labs"
  touch "${marker}"

  rsync -a --delete "${excludes[@]}" "${COURSE_SRC}/" "${COURSE_WS}/src/course/"
  rsync -a --delete "${excludes[@]}" "${LAB_SRC}/" "${COURSE_WS}/src/labs/"

  mapfile -t source_packages < <(discover_source_package_names)
  mapfile -t workspace_packages < <(discover_package_names "${COURSE_WS}/src")
  if [[ "${source_packages[*]}" != "${workspace_packages[*]}" ]]; then
    die "Workspace package discovery differs from the course source"
  fi
  log_ok "Synchronized ${#workspace_packages[@]} ROS packages"
}

install_workspace_dependencies() {
  CURRENT_STEP="course and lab dependencies"
  log_info "Installing reviewed dependencies for package and script-based labs"
  apt_install "${BASE_ROS_PACKAGES[@]}"

  local rosdep_args=(
    --from-paths "${COURSE_WS}/src"
    --ignore-src
    --rosdistro "${TARGET_ROS_DISTRO}"
    -r
    -y
  )
  local rosdep_check_args=(
    --from-paths "${COURSE_WS}/src"
    --ignore-src
    --rosdistro "${TARGET_ROS_DISTRO}"
  )

  local rosdep_skip_keys=("${BASE_ROSDEP_SKIP_KEYS[@]}")
  if [[ "${WITH_HARDWARE}" == false ]]; then
    rosdep_skip_keys+=("${HARDWARE_ROSDEP_KEYS[@]}")
  fi
  local skip_keys
  skip_keys="$(printf '%s ' "${rosdep_skip_keys[@]}")"
  skip_keys="${skip_keys% }"
  rosdep_args+=(--skip-keys "${skip_keys}")
  rosdep_check_args+=(--skip-keys "${skip_keys}")

  run rosdep install "${rosdep_args[@]}"
  run rosdep check "${rosdep_check_args[@]}"
}

install_ml_profile() {
  [[ "${WITH_ML}" == true ]] || return 0
  CURRENT_STEP="ML profile"
  log_info "Installing ML packages into ${ML_VENV}"
  run mkdir -p "$(dirname "${ML_VENV}")"
  if [[ ! -x "${ML_VENV}/bin/python" ]]; then
    run python3 -m venv --system-site-packages "${ML_VENV}"
  fi
  run "${ML_VENV}/bin/python" -m pip install "${ML_PIP_PACKAGES[@]}"
}

install_hardware_profile() {
  [[ "${WITH_HARDWARE}" == true ]] || return 0
  CURRENT_STEP="hardware profile"
  log_info "Installing camera, RealSense, and fiducial dependencies"
  apt_install "${HARDWARE_APT_PACKAGES[@]}"
  run sudo usermod -aG dialout "${USER:-$(id -un)}"
  log_warn "The dialout group change takes effect after signing out and back in"
  log_warn "The course's broad FTDI udev rule is intentionally not installed automatically"
}

install_carla_server() {
  local marker="${CARLA_DIR}/.ros2-course-carla-${CARLA_VERSION}"
  local archive_dir="${HOME}/.cache/ros2-course"
  local archive="${archive_dir}/CARLA_${CARLA_VERSION}.tar.gz"
  local archive_url="${CARLA_ARCHIVE_URL:-https://tiny.carla.org/carla-0-9-16-linux}"

  if [[ -f "${marker}" && -x "${CARLA_DIR}/CarlaUE4.sh" ]]; then
    log_ok "CARLA ${CARLA_VERSION} is already installed"
  elif [[ "${DRY_RUN}" == true ]]; then
    print_command curl -fL --retry 3 -o "${archive}" "${archive_url}"
    log_info "Would extract CARLA ${CARLA_VERSION} into ${CARLA_DIR}"
  else
    if [[ -d "${CARLA_DIR}" && -n "$(find "${CARLA_DIR}" -mindepth 1 -maxdepth 1 -print -quit)" ]]; then
      die "Refusing to replace unmanaged CARLA directory: ${CARLA_DIR}"
    fi

    mkdir -p "${archive_dir}"
    if [[ ! -s "${archive}" ]]; then
      curl -fL --retry 3 --continue-at - -o "${archive}" "${archive_url}"
    fi
    tar -tzf "${archive}" >/dev/null

    if [[ -n "${CARLA_SHA256:-}" ]]; then
      printf '%s  %s\n' "${CARLA_SHA256}" "${archive}" | sha256sum --check --status || \
        die "CARLA archive checksum mismatch"
    else
      log_warn "CARLA_SHA256 is unset; archive structure was checked but provenance was not"
    fi

    local staging
    staging="$(mktemp -d "${HOME}/.carla-${CARLA_VERSION}.XXXXXX")"
    tar -xzf "${archive}" -C "${staging}"
    [[ -x "${staging}/CarlaUE4.sh" ]] || die "CARLA archive does not contain CarlaUE4.sh"
    if [[ -d "${CARLA_DIR}" ]]; then
      rmdir "${CARLA_DIR}"
    fi
    mv "${staging}" "${CARLA_DIR}"
    touch "${marker}"
  fi

  run mkdir -p "$(dirname "${CARLA_VENV}")"
  if [[ ! -x "${CARLA_VENV}/bin/python" ]]; then
    run python3 -m venv --system-site-packages "${CARLA_VENV}"
  fi
  run "${CARLA_VENV}/bin/python" -m pip install "carla==${CARLA_VERSION}"
}

carla_python_path() {
  "${CARLA_VENV}/bin/python" -c \
    'import site; print(next(path for path in site.getsitepackages() if "site-packages" in path))'
}

install_carla_bridge() {
  local marker="${CARLA_WS}/.ros2-course-managed"
  local repo="${CARLA_WS}/src/ros-bridge"

  if [[ "${DRY_RUN}" == true ]]; then
    log_info "Would clone CARLA ROS bridge commit ${CARLA_BRIDGE_COMMIT} into ${repo}"
    log_info "Would apply the ROS2 Jazzy tf2_eigen compatibility patch to pcl_recorder"
    print_command colcon build --base-paths "${CARLA_WS}/src" --symlink-install
    return 0
  fi

  if [[ -d "${CARLA_WS}" && ! -f "${marker}" && \
        -n "$(find "${CARLA_WS}" -mindepth 1 -maxdepth 1 -print -quit)" ]]; then
    die "Refusing to modify an unmanaged CARLA bridge workspace: ${CARLA_WS}"
  fi

  mkdir -p "${CARLA_WS}/src"
  touch "${marker}"
  if [[ ! -d "${repo}/.git" ]]; then
    git clone --recurse-submodules https://github.com/carla-simulator/ros-bridge.git "${repo}"
  fi
  git -C "${repo}" fetch origin "${CARLA_BRIDGE_COMMIT}"
  git -C "${repo}" checkout --detach "${CARLA_BRIDGE_COMMIT}"
  git -C "${repo}" submodule update --init --recursive

  local bridge_version_file
  bridge_version_file="${repo}/carla_ros_bridge/src/carla_ros_bridge/CARLA_VERSION"
  [[ -f "${bridge_version_file}" ]] || die "Pinned bridge is missing its CARLA_VERSION contract"
  if [[ "$(<"${bridge_version_file}")" != "${CARLA_VERSION}" ]]; then
    log_warn "Pinned bridge declares CARLA $(<"${bridge_version_file}"); applying the managed 0.9.16 API compatibility override"
    printf '%s\n' "${CARLA_VERSION}" > "${bridge_version_file}"
  fi

  local pcl_recorder_cmake pcl_recorder_header pcl_recorder_manifest tf2_eigen_depend
  pcl_recorder_cmake="${repo}/pcl_recorder/CMakeLists.txt"
  pcl_recorder_header="${repo}/pcl_recorder/include/PclRecorderROS2.h"
  pcl_recorder_manifest="${repo}/pcl_recorder/package.xml"
  tf2_eigen_depend="<build_depend condition=\"\$ROS_VERSION == 2\">tf2_eigen</build_depend>"
  [[ -f "${pcl_recorder_cmake}" && -f "${pcl_recorder_header}" && \
        -f "${pcl_recorder_manifest}" ]] || die "Pinned bridge has an unexpected pcl_recorder layout"
  if grep -Fq '<tf2_eigen/tf2_eigen.h>' "${pcl_recorder_header}"; then
    log_warn "Applying the ROS2 Jazzy tf2_eigen compatibility patch to pcl_recorder"
    sed -i 's|<tf2_eigen/tf2_eigen.h>|<tf2_eigen/tf2_eigen.hpp>|' "${pcl_recorder_header}"
  elif ! grep -Fq '<tf2_eigen/tf2_eigen.hpp>' "${pcl_recorder_header}"; then
    die "Pinned bridge has an unexpected pcl_recorder tf2_eigen include"
  fi
  if grep -Fq 'pcl_conversions tf2 tf2_ros)' "${pcl_recorder_cmake}"; then
    sed -i 's|pcl_conversions tf2 tf2_ros)|pcl_conversions tf2 tf2_eigen tf2_ros)|' \
      "${pcl_recorder_cmake}"
  elif ! grep -Fq 'pcl_conversions tf2 tf2_eigen tf2_ros)' "${pcl_recorder_cmake}"; then
    die "Pinned bridge has unexpected pcl_recorder target dependencies"
  fi
  if ! grep -Fq "${tf2_eigen_depend}" "${pcl_recorder_manifest}"; then
    sed -i "/>ament_cmake<\\/buildtool_depend>/a\\  ${tf2_eigen_depend}" \
      "${pcl_recorder_manifest}"
  fi

  local carla_site_packages
  carla_site_packages="$(carla_python_path)"
  export PYTHONPATH="${carla_site_packages}:${PYTHONPATH:-}"
  rosdep install \
    --from-paths "${CARLA_WS}/src" \
    --ignore-src \
    --rosdistro "${TARGET_ROS_DISTRO}" \
    --skip-keys "ament_python" \
    -r -y

  (
    cd "${CARLA_WS}"
    colcon build --symlink-install --event-handlers console_cohesion+
  )
  [[ -f "${CARLA_WS}/install/setup.bash" ]] || die "CARLA bridge did not produce an install overlay"
}

install_carla_profile() {
  [[ "${WITH_CARLA}" == true ]] || return 0
  CURRENT_STEP="CARLA profile"
  apt_install libomp5
  install_carla_server
  install_carla_bridge
}

build_workspace() {
  CURRENT_STEP="course workspace build"
  if [[ "${DRY_RUN}" == true ]]; then
    print_command colcon build \
      --base-paths "${COURSE_WS}/src" \
      --symlink-install \
      --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
    return 0
  fi

  (
    cd "${COURSE_WS}"
    colcon build \
      --base-paths "${COURSE_WS}/src" \
      --symlink-install \
      --event-handlers console_cohesion+ \
      --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
  )
  [[ -f "${COURSE_WS}/install/setup.bash" ]] || die "Course build did not produce install/setup.bash"

  set +u
  # shellcheck disable=SC1091
  source "${COURSE_WS}/install/setup.bash"
  set -u
  log_ok "Course workspace built successfully"
}

test_workspace() {
  [[ "${RUN_TESTS}" == true ]] || return 0
  CURRENT_STEP="course workspace tests"
  if [[ "${DRY_RUN}" == true ]]; then
    print_command colcon test --base-paths "${COURSE_WS}/src" --executor sequential
    print_command colcon test-result --test-result-base "${COURSE_WS}/build" --verbose
    return 0
  fi

  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((20 + $$ % 200))}"
  (
    cd "${COURSE_WS}"
    colcon test \
      --base-paths "${COURSE_WS}/src" \
      --executor sequential \
      --event-handlers console_cohesion+ \
      --return-code-on-test-failure
    colcon test-result --test-result-base "${COURSE_WS}/build" --verbose
  )
}

write_environment_file() {
  CURRENT_STEP="shell environment configuration"
  if [[ "${DRY_RUN}" == true ]]; then
    log_info "Would generate ${ENV_FILE} and update the managed ~/.bashrc block"
    return 0
  fi

  mkdir -p "${ENV_DIR}"
  local env_tmp bashrc_tmp bashrc_file
  env_tmp="$(mktemp)"
  bashrc_tmp="$(mktemp)"
  bashrc_file="${HOME}/.bashrc"

  {
    printf '# Generated by %q. Manual edits will be replaced.\n' "${COURSE_ROOT}/setup_course.sh"
    printf 'if [[ -f %q ]]; then\n' "/opt/ros/${TARGET_ROS_DISTRO}/setup.bash"
    printf '  source %q\n' "/opt/ros/${TARGET_ROS_DISTRO}/setup.bash"
    printf 'fi\n'
    printf 'if [[ -f %q ]]; then\n' "${COURSE_WS}/install/setup.bash"
    printf '  source %q\n' "${COURSE_WS}/install/setup.bash"
    printf 'fi\n'
    if [[ -f "${CARLA_WS}/install/setup.bash" ]]; then
      printf 'if [[ -f %q ]]; then\n' "${CARLA_WS}/install/setup.bash"
      printf '  source %q\n' "${CARLA_WS}/install/setup.bash"
      printf 'fi\n'
    fi
    printf 'export ROS2_COURSE_ROOT=%q\n' "${COURSE_ROOT}"
    printf 'export ROS2_COURSE_WS=%q\n' "${COURSE_WS}"
    printf 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp\n'
    printf 'export RCUTILS_COLORIZED_OUTPUT=1\n'
    printf 'export RCUTILS_LOGGING_USE_STDOUT=1\n'
    if [[ -x "${ML_VENV}/bin/python" ]]; then
      printf 'export ROS2_COURSE_ML_PYTHON=%q\n' "${ML_VENV}/bin/python"
    fi
    if [[ -x "${CARLA_VENV}/bin/python" ]]; then
      printf 'export CARLA_ROOT=%q\n' "${CARLA_DIR}"
      # PYTHONPATH must be expanded when the generated file is sourced.
      # shellcheck disable=SC2016
      printf 'export PYTHONPATH=%q:${PYTHONPATH:-}\n' "$(carla_python_path)"
      printf "alias carla-server='cd \"\${CARLA_ROOT}\" && ./CarlaUE4.sh -quality-level=Low'\n"
    fi
    if [[ -f "${CARLA_WS}/install/setup.bash" ]]; then
      printf "alias carla-bridge='source %q && ros2 launch carla_ros_bridge carla_ros_bridge.launch.py'\n" \
        "${CARLA_WS}/install/setup.bash"
    fi
    printf "alias cw='cd \"\${ROS2_COURSE_WS}\"'\n"
    printf "alias cs='source \"\${ROS2_COURSE_WS}/install/setup.bash\"'\n"
    printf "alias cb='cd \"\${ROS2_COURSE_WS}\" && colcon build --symlink-install'\n"
  } > "${env_tmp}"
  install -m 0644 "${env_tmp}" "${ENV_FILE}"
  rm -f "${env_tmp}"

  touch "${bashrc_file}"
  awk -v begin="${BASHRC_BEGIN}" -v end="${BASHRC_END}" '
    $0 == begin {skip = 1; next}
    $0 == end {skip = 0; next}
    !skip {print}
  ' "${bashrc_file}" > "${bashrc_tmp}"
  {
    cat "${bashrc_tmp}"
    printf '\n%s\n' "${BASHRC_BEGIN}"
    printf 'source %q\n' "${ENV_FILE}"
    printf '%s\n' "${BASHRC_END}"
  } > "${bashrc_file}"
  rm -f "${bashrc_tmp}"
  log_ok "Managed shell environment written to ${ENV_FILE}"
}

check() {
  local description=$1
  shift
  if "$@"; then
    log_ok "${description}"
    return 0
  fi
  log_warn "${description}"
  return 1
}

verify_installation() {
  CURRENT_STEP="installation verification"
  [[ "${DRY_RUN}" == true ]] && {
    log_info "Dry-run complete; verification requires installed artifacts"
    return 0
  }

  local failures=0
  source_ros
  if [[ -f "${COURSE_WS}/install/setup.bash" ]]; then
    set +u
    # shellcheck disable=SC1091
    source "${COURSE_WS}/install/setup.bash"
    set -u
  fi

  check "ROS 2 Jazzy setup exists" test -f "/opt/ros/${TARGET_ROS_DISTRO}/setup.bash" || ((failures += 1))
  if [[ "${CARLA_ONLY}" == false ]]; then
    check "Course workspace overlay exists" test -f "${COURSE_WS}/install/setup.bash" || ((failures += 1))
  fi
  check "Managed shell environment exists" test -f "${ENV_FILE}" || ((failures += 1))
  check "CycloneDDS RMW is installed" ros2 pkg prefix rmw_cyclonedds_cpp >/dev/null || ((failures += 1))
  check "Required Python modules import" python3 -c \
    'import cv2, matplotlib, numpy, scipy, sklearn, yaml' || ((failures += 1))

  if [[ "${CARLA_ONLY}" == false && -f "${COURSE_WS}/install/setup.bash" ]]; then
    check "Representative course package is discoverable" \
      ros2 pkg prefix course_lab_utils >/dev/null || ((failures += 1))
    mapfile -t source_packages < <(discover_source_package_names)
    mapfile -t workspace_packages < <(discover_package_names "${COURSE_WS}/src")
    if [[ "${source_packages[*]}" == "${workspace_packages[*]}" ]]; then
      log_ok "All ${#workspace_packages[@]} source packages are present in the workspace"
    else
      log_warn "Workspace package list differs from the source tree"
      ((failures += 1))
    fi
  fi

  if [[ "${WITH_ML}" == true ]]; then
    check "ML profile imports" "${ML_VENV}/bin/python" -c \
      'import filterpy, openai, ultralytics' || ((failures += 1))
  fi
  if [[ "${WITH_HARDWARE}" == true ]]; then
    check "RealSense ROS package is installed" \
      ros2 pkg prefix realsense2_camera >/dev/null || ((failures += 1))
  fi
  if [[ "${WITH_CARLA}" == true ]]; then
    check "CARLA server is installed" test -x "${CARLA_DIR}/CarlaUE4.sh" || ((failures += 1))
    check "CARLA Python API imports" "${CARLA_VENV}/bin/python" -c \
      'import carla; from importlib.metadata import version; assert version("carla") == "0.9.16"' || \
      ((failures += 1))
    check "CARLA bridge overlay exists" test -f "${CARLA_WS}/install/setup.bash" || ((failures += 1))
  fi

  check "Installer shell syntax" bash -n "${COURSE_ROOT}/setup_course.sh" || ((failures += 1))
  if command -v shellcheck >/dev/null 2>&1; then
    check "Installer ShellCheck" shellcheck "${COURSE_ROOT}/setup_course.sh" || ((failures += 1))
  fi
  if [[ -f "${COURSE_ROOT}/tools/verify_course.py" ]]; then
    check "Course static verification" python3 "${COURSE_ROOT}/tools/verify_course.py" \
      --root "${COURSE_ROOT}" || ((failures += 1))
  fi

  ((failures == 0)) || die "Verification failed with ${failures} error(s)"
  log_ok "Installation verification passed"
}

main() {
  parse_args "$@"
  preflight
  acquire_lock

  if [[ "${VERIFY_ONLY}" == true ]]; then
    verify_installation
    return 0
  fi

  if [[ "${DRY_RUN}" == false ]]; then
    sudo -v
  fi

  install_base_tools
  install_ros
  source_ros
  initialize_rosdep

  if [[ "${CARLA_ONLY}" == false ]]; then
    sync_workspace
    install_workspace_dependencies
    install_hardware_profile
    install_ml_profile
    build_workspace
    test_workspace
  fi

  install_carla_profile
  write_environment_file
  verify_installation

  log_ok "Setup completed"
  log_info "Open a new terminal or run: source ${ENV_FILE}"
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  main "$@"
fi
