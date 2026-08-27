#!/bin/bash
# ============================================================
# SLAM 建图启动脚本
# 用于 ROS 2 Jazzy + slam_toolbox 在线建图
# 使用说明: chmod +x slam_bringup.sh && ./slam_bringup.sh
# ============================================================

set -e

echo "=============================================="
echo "  SLAM 建图启动脚本 - ROS 2 Jazzy"
echo "=============================================="

# 步骤1: 检查环境
echo ""
echo "[Step 1/4] 检查 ROS2 环境..."
if ! command -v ros2 &>/dev/null; then
    echo "错误: 未找到 ros2 命令，请先 source ROS2 环境"
    exit 1
fi
echo "ROS2 环境正常: $(ros2 --version)"

# 步骤2: 启动 Gazebo 仿真
echo ""
echo "[Step 2/4] 启动 Gazebo 仿真世界..."
ros2 launch robot_sim_demo gazebo2.launch.py gui:=true rviz:=false drive:=false &
GAZEBO_PID=$!
echo "Gazebo PID: $GAZEBO_PID"

# 等待 Gazebo 完全启动
echo "等待 Gazebo 初始化 (10秒)..."
sleep 10

# 步骤3: 启动 slam_toolbox 建图
echo ""
echo "[Step 3/4] 启动 slam_toolbox 在线建图..."
ros2 launch slam_sim_demo_ros2 slam_demo.launch.py &
SLAM_PID=$!
echo "SLAM PID: $SLAM_PID"

# 等待 SLAM 初始化
echo "等待 SLAM 初始化 (5秒)..."
sleep 5

# 步骤4: 提示用户操作
echo ""
echo "[Step 4/4] 启动完成!"
echo "=============================================="
echo ""
echo "可用操作:"
echo "  1. 启动键盘遥控建图:"
echo "     ros2 run teleop_twist_keyboard teleop_twist_keyboard"
echo ""
echo "  2. 可视化建图过程:"
echo "     rviz2 -d src/slam_sim_demo_ros2/rviz/slam.rviz"
echo ""
echo "  3. 保存地图:"
echo "     ros2 run nav2_map_server map_saver_cli -f ~/maps/office_map"
echo ""
echo "  4. 查看地图话题:"
echo "     ros2 topic echo /map --field info --once"
echo ""
echo "  5. 查看 TF 树:"
echo "     ros2 run tf2_tools view_frames"
echo ""
echo "=============================================="
echo "按 Ctrl+C 停止所有进程"

# 等待子进程
trap "echo '正在停止...'; kill $GAZEBO_PID $SLAM_PID 2>/dev/null; exit" INT TERM
wait
