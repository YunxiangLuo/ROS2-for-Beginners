#!/bin/bash
# =============================================================================
# 第31章 综合项目：城区自动驾驶 - 一键启动脚本
#
# 用法:
#   ./town_demo.sh                    # 标准启动
#   ./town_demo.sh --weather rainy    # 雨天场景
#   ./town_demo.sh --traffic dense    # 密集交通（30+ NPC）
#   ./town_demo.sh --record           # 启动并录制ROS bag
#   ./town_demo.sh --help             # 查看帮助
# =============================================================================

set -euo pipefail

# ── 颜色定义 ──
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# ── 默认配置 ──
CARLA_PATH="${CARLA_PATH:-/opt/carla}"
ROS_WS="${ROS_WS:-$HOME/ros2_course_ws}"
TOWN="Town03"
WEATHER="default"
TRAFFIC_DENSITY="medium"
RECORD_BAG=false
AUTO_START=false
SCREEN_MODE="normal"
PIPELINE_AVAILABLE=false

# ── 帮助函数 ──
usage() {
    echo -e "${CYAN}用法:${NC} $0 [选项]"
    echo ""
    echo "选项:"
    echo "  --carla-path PATH    CARLA安装路径 (默认: /opt/carla)"
    echo "  --ros-ws PATH        ROS2工作空间路径 (默认: ~/ros2_course_ws)"
    echo "  --town NAME          CARLA城镇地图 (默认: Town03)"
    echo "  --weather TYPE       天气: default|rainy|sunset|night (默认: default)"
    echo "  --traffic DENSITY    交通密度: light|medium|dense (默认: medium)"
    echo "  --record             录制ROS bag到 ./demo_bag/"
    echo "  --auto               启动后自动开始自动驾驶"
    echo "  --screen-fullscreen  全屏模式启动CARLA"
    echo "  --screen-quality-low CARLA低画质模式"
    echo "  --help               显示此帮助"
    exit 0
}

# ── 参数解析 ──
while [[ $# -gt 0 ]]; do
    case "$1" in
        --carla-path)       CARLA_PATH="$2"; shift 2 ;;
        --ros-ws)           ROS_WS="$2"; shift 2 ;;
        --town)             TOWN="$2"; shift 2 ;;
        --weather)          WEATHER="$2"; shift 2 ;;
        --traffic)          TRAFFIC_DENSITY="$2"; shift 2 ;;
        --record)           RECORD_BAG=true; shift ;;
        --auto)             AUTO_START=true; shift ;;
        --screen-fullscreen) SCREEN_MODE="-fullscreen"; shift ;;
        --screen-quality-low) SCREEN_MODE="-quality-level=Low"; shift ;;
        --help|-h)          usage ;;
        *)                  echo -e "${RED}未知参数: $1${NC}"; usage ;;
    esac
done

# ── 检查依赖 ──
check_dependencies() {
    echo -e "${BLUE}[CHECK] 检查依赖...${NC}"

    # 检查CARLA
    if [ ! -f "$CARLA_PATH/CarlaUE4.sh" ]; then
        echo -e "${RED}[ERROR] CARLA未找到: $CARLA_PATH/CarlaUE4.sh${NC}"
        echo "请设置 CARLA_PATH 环境变量或使用 --carla-path 指定"
        exit 1
    fi
    echo -e "${GREEN}[OK] CARLA: $CARLA_PATH${NC}"

    # 检查ROS2
    if [ -z "${ROS_DISTRO:-}" ]; then
        echo -e "${YELLOW}[WARN] ROS_DISTRO未设置，尝试加载...${NC}"
        if [ -f /opt/ros/jazzy/setup.bash ]; then
            source /opt/ros/jazzy/setup.bash
        else
            echo -e "${RED}[ERROR] 无法找到ROS2安装${NC}"
            exit 1
        fi
    fi
    echo -e "${GREEN}[OK] ROS2: ${ROS_DISTRO}${NC}"

    # 检查工作空间
    if [ ! -f "$ROS_WS/install/setup.bash" ]; then
        echo -e "${YELLOW}[WARN] 工作空间未编译: $ROS_WS${NC}"
        echo "执行编译: cd $ROS_WS && colcon build --symlink-install"
        read -p "是否现在编译? [y/N] " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            cd "$ROS_WS"
            colcon build --symlink-install
        else
            exit 1
        fi
    fi
    echo -e "${GREEN}[OK] 工作空间: $ROS_WS${NC}"
}

# ── 启动CARLA服务器 ──
start_carla_server() {
    echo -e "${BLUE}[CARLA] 启动CARLA服务器...${NC}"
    cd "$CARLA_PATH"

    # 检查是否已在运行
    if pgrep -x "CarlaUE4" > /dev/null; then
        echo -e "${YELLOW}[WARN] CARLA服务器已在运行${NC}"
        read -p "是否重启? [y/N] " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            pkill CarlaUE4 2>/dev/null || true
            sleep 3
        else
            return 0
        fi
    fi

    CARLA_CMD="./CarlaUE4.sh $SCREEN_MODE -carla-rpc-port=2000 -carla-streaming-port=2100"

    # 根据天气添加参数
    case "$WEATHER" in
        rainy)  CARLA_CMD="$CARLA_CMD -quality-level=Low -weather=Rainy" ;;
        sunset) CARLA_CMD="$CARLA_CMD -quality-level=Low -weather=Sunset" ;;
        night)  CARLA_CMD="$CARLA_CMD -quality-level=Low -weather=Night" ;;
        *)      CARLA_CMD="$CARLA_CMD -quality-level=Low" ;;
    esac

    # 在后台启动CARLA
    echo -e "${CYAN}执行: $CARLA_CMD${NC}"
    $CARLA_CMD &
    CARLA_PID=$!
    echo -e "${GREEN}[CARLA] 服务器已启动 (PID: $CARLA_PID)${NC}"

    # 等待CARLA就绪
    echo -e "${YELLOW}[CARLA] 等待CARLA就绪...${NC}"
    for i in $(seq 1 30); do
        if python3 -c "import carla; client = carla.Client('localhost', 2000); client.get_world()" 2>/dev/null; then
            echo -e "${GREEN}[CARLA] 就绪!${NC}"
            return 0
        fi
        sleep 1
        echo -n "."
    done
    echo
    echo -e "${RED}[ERROR] CARLA启动超时${NC}"
    exit 1
}

# ── 加载ROS2环境 ──
source_ros() {
    source /opt/ros/${ROS_DISTRO}/setup.bash
    source "$ROS_WS/install/setup.bash"
}

# ── 启动CARLA ROS2 Bridge ──
start_bridge() {
    echo -e "${BLUE}[BRIDGE] 启动CARLA ROS2 Bridge...${NC}"
    source_ros

    BRIDGE_CMD="ros2 launch carla_ros_bridge carla_ros_bridge.launch.py town:=$TOWN"
    if [ "$WEATHER" != "default" ]; then
        BRIDGE_CMD="$BRIDGE_CMD weather:=$WEATHER"
    fi

    echo -e "${CYAN}执行: $BRIDGE_CMD${NC}"
    gnome-terminal --title="CARLA Bridge" -- bash -c "$BRIDGE_CMD; exec bash" 2>/dev/null || \
    tmux new-window -t "autonomous_demo" -n "bridge" "$BRIDGE_CMD" 2>/dev/null || \
    ($BRIDGE_CMD &)

    sleep 5
    echo -e "${GREEN}[BRIDGE] 启动完成${NC}"
}

# ── 启动自动驾驶各节点 ──
start_autonomous_driving() {
    echo -e "${BLUE}[PIPELINE] 启动自动驾驶管线...${NC}"
    source_ros

    local pipeline_dir="$ROS_WS/src/labs/ch31_lab"
    if [ ! -d "$pipeline_dir" ]; then
        pipeline_dir="$ROS_WS/src/lab_code/ch31_lab"
    fi
    local required_modules=(
        carla_sensor_driver
        perception_node
        localization_node
        planning_node
        control_node
        safety_monitor_node
    )
    local missing_modules=()
    for module in "${required_modules[@]}"; do
        if [ ! -d "$pipeline_dir/$module" ]; then
            missing_modules+=("$module")
        fi
    done

    if [ ${#missing_modules[@]} -gt 0 ]; then
        echo -e "${YELLOW}[WARN] 未找到完整管线组件，跳过主管线启动: ${missing_modules[*]}${NC}"
        echo "请按 ch31_lab/README.md 配置目录外的运行组件后再启动主管线。"
        return 0
    fi

    PIPELINE_AVAILABLE=true
    local launch_cmd="PYTHONPATH=\"$(dirname "$pipeline_dir"):\${PYTHONPATH:-}\" python3 -m ch31_lab.main_pipeline"

    echo -e "${CYAN}执行: $launch_cmd${NC}"
    gnome-terminal --title="Autonomous Driving" -- bash -c "$launch_cmd; exec bash" 2>/dev/null || \
    tmux new-window -t "autonomous_demo" -n "driving" "$launch_cmd" 2>/dev/null || \
    (eval "$launch_cmd" &)

    sleep 3
    echo -e "${GREEN}[PIPELINE] 启动完成${NC}"
}

# ── 启动可视化 ──
start_visualization() {
    echo -e "${BLUE}[VIZ] 启动RViz2可视化...${NC}"
    source_ros

    if [ -z "${RVIZ_CONFIG:-}" ]; then
        RVIZ_CONFIG="$ROS_WS/src/course/navigation_sim_demo_ros2/rviz/navigation.rviz"
        if [ ! -f "$RVIZ_CONFIG" ]; then
            RVIZ_CONFIG="$ROS_WS/src/navigation_sim_demo_ros2/rviz/navigation.rviz"
        fi
    fi
    if [ -f "$RVIZ_CONFIG" ]; then
        rviz2 -d "$RVIZ_CONFIG" &
    else
        rviz2 &
    fi

    echo -e "${GREEN}[VIZ] RViz2已启动${NC}"
}

# ── 启动ROS Bag录制 ──
start_recording() {
    if [ "$RECORD_BAG" = true ]; then
        echo -e "${BLUE}[BAG] 开始录制ROS bag...${NC}"
        source_ros

        BAG_DIR="${BAG_DIR:-./demo_bag}"
        mkdir -p "$BAG_DIR"

        ros2 bag record \
            -o "$BAG_DIR/autonomous_demo_$(date +%Y%m%d_%H%M%S)" \
            /sensor/camera/rgb/image \
            /sensor/lidar/pointcloud \
            /sensor/gnss/data \
            /perception/obstacles \
            /perception/traffic_lights \
            /perception/lane_markers \
            /localization/ego_pose \
            /localization/ego_twist \
            /planning/trajectory \
            /planning/behavior \
            /control/throttle \
            /control/steer \
            /control/brake \
            /safety/emergency_stop \
            /safety/status \
            /system/pipeline_status &

        echo -e "${GREEN}[BAG] 录制已开始, 保存到 $BAG_DIR${NC}"
    fi
}

# ── 等待自动驾驶完成 ──
wait_for_completion() {
    local timeout=$((10 * 60))  # 10分钟超时
    local elapsed=0
    local check_interval=5

    echo -e "${BLUE}[MONITOR] 监控行驶状态...${NC}"

    while [ $elapsed -lt $timeout ]; do
        sleep $check_interval
        elapsed=$((elapsed + check_interval))

        source_ros 2>/dev/null

        # 检查管线状态
        if ros2 topic echo /system/pipeline_status --once --field message 2>/dev/null | grep -q "COMPLETED"; then
            echo -e "${GREEN}[SUCCESS] 自动驾驶完成!${NC}"
            return 0
        fi

        # 检查故障
        if ros2 topic echo /system/pipeline_status --once --field level 2>/dev/null | grep -q "3"; then
            echo -e "${RED}[ERROR] 系统故障${NC}"
            return 1
        fi

        # 打印状态
        local behavior=$(ros2 topic echo /planning/behavior --once --field data 2>/dev/null)
        echo -e "${CYAN}[$(date +%H:%M:%S)] 行为: ${behavior:-N/A} | 已运行: ${elapsed}s${NC}"
    done

    echo -e "${YELLOW}[TIMEOUT] 运行超时${NC}"
    return 2
}

# ── 停止所有进程 ──
cleanup() {
    echo -e "\n${YELLOW}[CLEANUP] 停止所有进程...${NC}"

    # 停止录制
    pkill -f "ros2 bag record" 2>/dev/null || true

    # 停止自动驾驶管线
    source_ros 2>/dev/null
    ros2 service call /pipeline/enable std_srvs/srv/SetBool "{data: false}" 2>/dev/null || true
    sleep 1

    # 停止RViz
    pkill rviz2 2>/dev/null || true

    # 停止ROS2节点
    pkill -f "ros2 launch" 2>/dev/null || true
    pkill -f "ros2 run" 2>/dev/null || true

    # 停止CARLA
    if [ -n "${CARLA_PID:-}" ]; then
        kill $CARLA_PID 2>/dev/null || true
    fi
    pkill CarlaUE4 2>/dev/null || true

    echo -e "${GREEN}[CLEANUP] 所有进程已停止${NC}"
}

# ── 设置退出钩子 ──
trap cleanup EXIT INT TERM

# ── 主流程 ──
main() {
    echo -e "${CYAN}╔═══════════════════════════════════════════╗${NC}"
    echo -e "${CYAN}║   第31章 城区自动驾驶综合项目 Demo       ║${NC}"
    echo -e "${CYAN}║   Town: $TOWN  Weather: $WEATHER  Traffic: $TRAFFIC_DENSITY ║${NC}"
    echo -e "${CYAN}╚═══════════════════════════════════════════╝${NC}"
    echo

    # 使用tmux if available
    if command -v tmux &>/dev/null; then
        echo -e "${GREEN}[INFO] tmux可用，将使用tmux管理终端${NC}"
    fi

    check_dependencies
    start_carla_server
    start_bridge
    start_autonomous_driving
    start_visualization
    start_recording

    echo
    echo -e "${GREEN}╔═══════════════════════════════════════════╗${NC}"
    echo -e "${GREEN}║    CARLA、Bridge 和可视化已启动           ║${NC}"
    if [ "$PIPELINE_AVAILABLE" = true ]; then
        echo -e "${GREEN}║    自动驾驶主管线已启动                  ║${NC}"
    else
        echo -e "${YELLOW}║    自动驾驶主管线等待外部组件            ║${NC}"
    fi
    echo -e "${GREEN}╚═══════════════════════════════════════════╝${NC}"
    echo
    echo -e "查看话题:     ${CYAN}ros2 topic list${NC}"
    if [ "$PIPELINE_AVAILABLE" = true ]; then
        echo -e "启动自动驾驶: ${CYAN}ros2 service call /pipeline/enable std_srvs/srv/SetBool \"{data: true}\"${NC}"
        echo -e "设置目标点:   ${CYAN}ros2 topic pub /planning/set_goal ...${NC}"
        echo -e "查看状态:     ${CYAN}ros2 topic echo /system/pipeline_status${NC}"
    else
        echo -e "${YELLOW}请先配置目录外的自动驾驶管线组件。${NC}"
    fi
    echo

    if [ "$AUTO_START" = true ] && [ "$PIPELINE_AVAILABLE" = true ]; then
        echo -e "${BLUE}[AUTO] 自动启动自动驾驶 (5秒后)...${NC}"
        sleep 5
        source_ros
        ros2 service call /pipeline/enable std_srvs/srv/SetBool "{data: true}"
        wait_for_completion
    else
        # 保持前台运行，等待Ctrl+C
        echo "按 Ctrl+C 停止所有进程..."
        wait
    fi
}

main
