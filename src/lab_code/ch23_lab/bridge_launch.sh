#!/bin/bash
# bridge_launch.sh — 一键启动CARLA + ROS2 Bridge + Ego Vehicle
# 用法: bash bridge_launch.sh [carla_path] [spawn_point_index]

set -e

CARLA_PATH="${1:-./CarlaUE4.sh}"
SPAWN_POINT="${2:-10}"
ROS_WS="${HOME}/carla_ws"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "============================================"
echo " CARLA-ROS2 Bridge 一键启动脚本"
echo "============================================"
echo "CARLA路径:    ${CARLA_PATH}"
echo "生成点索引:    ${SPAWN_POINT}"
echo "ROS工作空间:  ${ROS_WS}"
echo "脚本目录:     ${SCRIPT_DIR}"
echo "============================================"

# 步骤1: 检查CARLA是否已在运行
if pgrep -x "CarlaUE4" > /dev/null 2>&1 || \
   pgrep -x "CarlaUE4.sh" > /dev/null 2>&1; then
    echo "[INFO] CARLA 仿真器已在运行"
else
    echo "[INFO] 正在启动 CARLA 仿真器..."
    if [[ "$CARLA_PATH" == *.exe ]] || [[ "$OSTYPE" == "msys" ]] || \
       [[ "$OSTYPE" == "cygwin" ]]; then
        # Windows
        "${CARLA_PATH}" -quality-level=Low &
    else
        # Linux
        bash "${CARLA_PATH}" -quality-level=Low &
    fi
    echo "[INFO] 等待 CARLA 启动 (15秒)..."
    sleep 15
fi

# 步骤2: 检查并source ROS2环境
if [ -f "${ROS_WS}/install/setup.bash" ]; then
    echo "[INFO] 加载 ROS2 工作空间: ${ROS_WS}"
    source "${ROS_WS}/install/setup.bash"
elif [ -f "install/setup.bash" ]; then
    echo "[INFO] 加载本地 ROS2 工作空间"
    source "install/setup.bash"
else
    echo "[WARN] 未找到 ROS2 工作空间，尝试系统级 source"
    if [ -f "/opt/ros/jazzy/setup.bash" ]; then
        source "/opt/ros/jazzy/setup.bash"
    else
        echo "[ERROR] 无法找到 ROS2 环境配置"
        exit 1
    fi
fi

# 步骤3: 启动 ROS2 Bridge
echo "[INFO] 正在启动 CARLA ROS2 Bridge..."
ros2 launch carla_ros_bridge carla_ros_bridge.launch.py \
    synchronous_mode:=False \
    register_all_sensors:=True &
BRIDGE_PID=$!
echo "[INFO] Bridge PID: ${BRIDGE_PID}"

# 等待Bridge就绪
echo "[INFO] 等待 Bridge 就绪 (5秒)..."
sleep 5

# 步骤4: 生成 Ego Vehicle
echo "[INFO] 正在生成 Ego Vehicle (spawn point ${SPAWN_POINT})..."
python3 "${SCRIPT_DIR}/spawn_ego.py" \
    --spawn-point "${SPAWN_POINT}" \
    --role-name "ego_vehicle"

# 步骤5: 验证
echo "[INFO] 正在验证话题..."
python3 "${SCRIPT_DIR}/check_topics.py" || true

echo ""
echo "============================================"
echo " 启动完成！"
echo ""
echo " RViz2 可视化:    rviz2"
echo " 话题列表:        ros2 topic list | grep carla"
echo " 发布控制指令:"
echo "   ros2 topic pub /carla/ego_vehicle/vehicle_control_cmd \\"
echo "     carla_msgs/msg/CarlaEgoVehicleControl \\"
echo '     "{throttle: 0.3, steer: 0.0, brake: 0.0}" --rate 10'
echo " 自动驾驶模式:"
echo "   ros2 service call /carla/ego_vehicle/enable_autopilot \\"
echo '     std_srvs/srv/SetBool "{data: true}"'
echo "============================================"

# 等待Bridge进程
wait "${BRIDGE_PID}"
