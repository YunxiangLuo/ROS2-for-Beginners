#!/bin/bash
# ============================================================
# Nav2 导航启动脚本
# 用于 ROS 2 Jazzy + Nav2 自主导航
# 使用说明: chmod +x nav2_bringup.sh && ./nav2_bringup.sh
# ============================================================

set -e

echo "=============================================="
echo "  Nav2 导航启动脚本 - ROS 2 Jazzy"
echo "=============================================="

# 检查环境
echo ""
echo "[Step 1/5] 检查 ROS2 环境..."
if ! command -v ros2 &>/dev/null; then
    echo "错误: 未找到 ros2 命令"
    exit 1
fi

# 检查地图文件
MAP_FILE=~/maps/office_map.yaml
if [ ! -f "$MAP_FILE" ]; then
    echo "警告: 地图文件 $MAP_FILE 不存在"
    echo "将尝试使用默认地图..."
    MAP_FLAG=""
else
    echo "使用地图: $MAP_FILE"
    MAP_FLAG="map:=$MAP_FILE"
fi

# 启动 Gazebo 仿真
echo ""
echo "[Step 2/5] 启动 Gazebo 仿真世界..."
ros2 launch robot_sim_demo gazebo2.launch.py gui:=true rviz:=false drive:=false &
GAZEBO_PID=$!
echo "Gazebo PID: $GAZEBO_PID"
sleep 8

# 启动 Nav2 导航
echo ""
echo "[Step 3/5] 启动 Nav2 导航系统..."
ros2 launch navigation_sim_demo_ros2 nav2_demo.launch.py $MAP_FLAG &
NAV2_PID=$!
echo "Nav2 PID: $NAV2_PID"
sleep 5

# 检查 Nav2 节点状态
echo ""
echo "[Step 4/5] 检查 Nav2 节点状态..."
for node in /amcl /bt_navigator /planner_server /controller_server; do
    status=$(ros2 lifecycle get $node 2>/dev/null || echo "未知")
    echo "  $node: $status"
done

# 完成
echo ""
echo "[Step 5/5] 启动完成!"
echo "=============================================="
echo ""
echo "可用操作:"
echo "  1. RViz2 可视化导航:"
echo "     rviz2 -d src/navigation_sim_demo_ros2/rviz/navigation.rviz"
echo "     使用 \"Nav2 Goal\" 按钮设置目标点"
echo ""
echo "  2. 命令行发送导航目标:"
echo "     ros2 topic pub /goal_pose geometry_msgs/PoseStamped \"{header: {frame_id: 'map'}, pose: {position: {x: 3.0, y: -1.0, z: 0.0}, orientation: {w: 1.0}}}\" --once"
echo ""
echo "  3. Python 脚本导航:"
echo "     ros2 run navigation_sim_demo_ros2 nav_goal_runner"
echo ""
echo "  4. 查看导航动作:"
echo "     ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \"{pose: {header: {frame_id: 'map'}, pose: {position: {x: 3.0, y: -1.0}, orientation: {w: 1.0}}}}\""
echo ""
echo "  5. 查看代价地图:"
echo "     ros2 topic echo /global_costmap/costmap --once --field info"
echo "     ros2 topic echo /local_costmap/costmap --once --field info"
echo ""
echo "  6. 查看 AMCL 定位:"
echo "     ros2 topic echo /amcl_pose --once"
echo "     ros2 topic echo /particlecloud --once | head -5"
echo ""
echo "=============================================="
echo "按 Ctrl+C 停止所有进程"

trap "echo '正在停止...'; kill $GAZEBO_PID $NAV2_PID 2>/dev/null; wait; exit" INT TERM
wait
