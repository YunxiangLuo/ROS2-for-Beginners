#!/usr/bin/env bash
# bridge_launch.sh - start CARLA, ROS 2 Bridge, and a bounded Ego Vehicle run.
# Usage: bash bridge_launch.sh [carla_path] [spawn_point_index]

set -Eeuo pipefail

SPAWN_POINT="${2:-10}"
ROS_WS="${CARLA_BRIDGE_WS:-${HOME}/carla_ws}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COURSE_ENV_FILE="${XDG_CONFIG_HOME:-${HOME}/.config}/ros2-course/env.bash"
BRIDGE_PID=""
EGO_PID=""

if [[ -f "${COURSE_ENV_FILE}" ]]; then
    # shellcheck disable=SC1090
    source "${COURSE_ENV_FILE}"
fi

CARLA_PATH="${1:-${CARLA_ROOT:-}}"
if [[ -d "${CARLA_PATH}" && -x "${CARLA_PATH}/CarlaUE4.sh" ]]; then
    CARLA_PATH="${CARLA_PATH}/CarlaUE4.sh"
fi

if [[ -z "${CARLA_HOST:-}" ]]; then
    if grep -qi microsoft /proc/version 2>/dev/null && command -v ip >/dev/null 2>&1; then
        CARLA_HOST="$(ip route show default | awk '/default via/ {print $3; exit}')"
    fi
    CARLA_HOST="${CARLA_HOST:-127.0.0.1}"
fi
CARLA_PORT="${CARLA_PORT:-2000}"
CARLA_MAP="${CARLA_MAP:-Carla/Maps/Town10HD_Opt}"
CARLA_BRIDGE_TIMEOUT="${CARLA_BRIDGE_TIMEOUT:-30}"
CARLA_STARTUP_TIMEOUT="${CARLA_STARTUP_TIMEOUT:-60}"
BRIDGE_STARTUP_TIMEOUT="${BRIDGE_STARTUP_TIMEOUT:-30}"
CARLA_DURATION="${CARLA_DURATION:-30}"
CARLA_KEEP_RUNNING="${CARLA_KEEP_RUNNING:-false}"

die() {
    echo "[ERROR] $*" >&2
    exit 1
}

cleanup() {
    if [[ -n "${EGO_PID}" ]] && kill -0 "${EGO_PID}" 2>/dev/null; then
        kill -INT "${EGO_PID}" 2>/dev/null || true
        wait "${EGO_PID}" 2>/dev/null || true
    fi
    if [[ "${CARLA_KEEP_RUNNING}" != true && -n "${BRIDGE_PID}" ]] && \
        kill -0 "${BRIDGE_PID}" 2>/dev/null; then
        kill -INT "${BRIDGE_PID}" 2>/dev/null || true
        wait "${BRIDGE_PID}" 2>/dev/null || true
    fi
}

trap 'exit 130' INT TERM
trap cleanup EXIT

[[ "${CARLA_STARTUP_TIMEOUT}" =~ ^[0-9]+$ ]] || \
    die "CARLA_STARTUP_TIMEOUT must be a non-negative integer"
[[ "${BRIDGE_STARTUP_TIMEOUT}" =~ ^[0-9]+$ ]] || \
    die "BRIDGE_STARTUP_TIMEOUT must be a non-negative integer"
[[ "${CARLA_DURATION}" =~ ^[0-9]+([.][0-9]+)?$ ]] || \
    die "CARLA_DURATION must be zero or a positive number of seconds"

carla_ready() {
    python3 -c \
        'import carla, sys; c=carla.Client(sys.argv[1], int(sys.argv[2])); c.set_timeout(1.0); c.get_world()' \
        "${CARLA_HOST}" "${CARLA_PORT}" >/dev/null 2>&1
}

echo "============================================"
echo " CARLA-ROS2 Bridge 一键启动脚本"
echo "============================================"
echo "CARLA地址:     ${CARLA_HOST}:${CARLA_PORT}"
echo "CARLA地图:     ${CARLA_MAP}"
echo "CARLA路径:     ${CARLA_PATH:-<external server>}"
echo "生成点索引:     ${SPAWN_POINT}"
echo "运行时长:       ${CARLA_DURATION}s"
echo "ROS工作空间:    ${ROS_WS}"
echo "脚本目录:       ${SCRIPT_DIR}"
echo "============================================"

if carla_ready; then
    echo "[INFO] CARLA 仿真器已就绪"
else
    [[ -n "${CARLA_PATH}" ]] || die \
        "CARLA is not reachable at ${CARLA_HOST}:${CARLA_PORT}; set CARLA_HOST or pass carla_path"
    echo "[INFO] 正在启动 CARLA 仿真器..."
    CARLA_LAUNCH_MAP="${CARLA_MAP##*/}"
    if [[ "${CARLA_PATH}" == *.exe ]]; then
        "${CARLA_PATH}" -quality-level=Low -carla-map="${CARLA_LAUNCH_MAP}" \
            -carla-rpc-port="${CARLA_PORT}" -carla-streaming-port=2001 &
    elif grep -qi microsoft /proc/version 2>/dev/null; then
        GALLIUM_DRIVER="${GALLIUM_DRIVER:-d3d12}" bash "${CARLA_PATH}" \
            -quality-level=Low -carla-map="${CARLA_LAUNCH_MAP}" \
            -carla-rpc-port="${CARLA_PORT}" -carla-streaming-port=2001 &
    else
        bash "${CARLA_PATH}" -quality-level=Low -carla-map="${CARLA_LAUNCH_MAP}" \
            -carla-rpc-port="${CARLA_PORT}" -carla-streaming-port=2001 &
    fi

    echo "[INFO] 等待 CARLA 就绪，最多 ${CARLA_STARTUP_TIMEOUT} 秒..."
    carla_started=false
    for ((second = 0; second <= CARLA_STARTUP_TIMEOUT; second++)); do
        if carla_ready; then
            carla_started=true
            break
        fi
        sleep 1
    done
    [[ "${carla_started}" == true ]] || die \
        "CARLA did not become ready at ${CARLA_HOST}:${CARLA_PORT}"
fi

if [[ -f "/opt/ros/jazzy/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "/opt/ros/jazzy/setup.bash"
fi
if [[ -f "${ROS_WS}/install/setup.bash" ]]; then
    echo "[INFO] 加载 ROS2 工作空间: ${ROS_WS}"
    # shellcheck disable=SC1090
    source "${ROS_WS}/install/setup.bash"
fi
command -v ros2 >/dev/null 2>&1 || die "ROS 2 is not available; source the Jazzy environment"

echo "[INFO] 正在启动 CARLA ROS2 Bridge..."
ros2 launch carla_ros_bridge carla_ros_bridge.launch.py \
    host:="${CARLA_HOST}" \
    port:="${CARLA_PORT}" \
    timeout:="${CARLA_BRIDGE_TIMEOUT}" \
    town:="${CARLA_MAP}" \
    synchronous_mode:=False \
    register_all_sensors:=True &
BRIDGE_PID=$!
echo "[INFO] Bridge PID: ${BRIDGE_PID}"

echo "[INFO] 等待 Bridge 就绪，最多 ${BRIDGE_STARTUP_TIMEOUT} 秒..."
bridge_started=false
for ((second = 0; second <= BRIDGE_STARTUP_TIMEOUT; second++)); do
    if ros2 node list 2>/dev/null | grep -qx '/carla_ros_bridge'; then
        bridge_started=true
        break
    fi
    if ! kill -0 "${BRIDGE_PID}" 2>/dev/null; then
        break
    fi
    sleep 1
done
[[ "${bridge_started}" == true ]] || die "CARLA ROS2 Bridge did not become ready"

echo "[INFO] 正在生成 Ego Vehicle (spawn point ${SPAWN_POINT})..."
python3 "${SCRIPT_DIR}/spawn_ego.py" \
    --host "${CARLA_HOST}" \
    --port "${CARLA_PORT}" \
    --spawn-point "${SPAWN_POINT}" \
    --role-name "ego_vehicle" \
    --duration "${CARLA_DURATION}" &
EGO_PID=$!

sleep 2
echo "[INFO] 正在验证话题..."
python3 "${SCRIPT_DIR}/check_topics.py" --role-name ego_vehicle --verbose || true
wait "${EGO_PID}"
EGO_PID=""

echo ""
echo "============================================"
echo " ${CARLA_DURATION} 秒运行完成"
echo "============================================"
