#!/bin/bash
# ============================================================
# RealSense D415/D435 相机 ROS2 启动脚本
# 用于 ROS 2 Jazzy + realsense2_camera
# 使用说明: chmod +x realsense_launch.sh && ./realsense_launch.sh
# ============================================================

set -e

echo "=============================================="
echo "  RealSense 相机启动脚本 - ROS 2 Jazzy"
echo "=============================================="

# 步骤1: 检查环境
echo ""
echo "[Step 1/5] 检查 ROS2 环境..."
if ! command -v ros2 &>/dev/null; then
    echo "错误: 未找到 ros2 命令"
    exit 1
fi

# 检查 realsense2_camera 是否安装
if ! ros2 package list 2>/dev/null | grep -q realsense2_camera; then
    echo "警告: realsense2_camera 未安装"
    echo "安装命令: sudo apt install ros-jazzy-realsense2-camera"
    echo "继续尝试启动..."
fi

# 检查相机设备
echo ""
echo "[Step 2/5] 检查 RealSense 设备..."
if command -v rs-enumerate-devices &>/dev/null; then
    rs-enumerate-devices | grep -E "Device Name|Product Line|Serial Number|USB Type" || true
else
    echo "rs-enumerate-devices 未安装，跳过设备检查"
fi

# 启动 RealSense 相机驱动
echo ""
echo "[Step 3/5] 启动 RealSense 相机..."

# 启动模式选择
MODE="rgbd"
if [ "$1" = "pointcloud" ]; then
    echo "模式: RGB-D + 点云"
    ros2 launch realsense2_camera rs_launch.py \
        depth_module.depth_profile:=640x480x30 \
        rgb_camera.color_profile:=640x480x30 \
        pointcloud.enable:=true \
        align_depth.enable:=true \
        enable_sync:=true &
elif [ "$1" = "highres" ]; then
    echo "模式: 高分辨率"
    ros2 launch realsense2_camera rs_launch.py \
        depth_module.depth_profile:=1280x720x15 \
        rgb_camera.color_profile:=1280x720x15 &
else
    echo "模式: 标准 RGB-D (默认)"
    echo "可选模式: $0 pointcloud (带点云) / $0 highres (高分辨率)"
    ros2 launch realsense2_camera rs_launch.py \
        depth_module.depth_profile:=640x480x30 \
        rgb_camera.color_profile:=640x480x30 &
fi

CAMERA_PID=$!
echo "相机 PID: $CAMERA_PID"
sleep 3

# 检查话题
echo ""
echo "[Step 4/5] 检查相机话题..."
echo "等待话题发布..."
sleep 2

expected_topics=(
    "/camera/color/image_raw"
    "/camera/depth/image_rect_raw"
    "/camera/color/camera_info"
)

for topic in "${expected_topics[@]}"; do
    if ros2 topic list 2>/dev/null | grep -q "$topic"; then
        echo "  ✓ $topic"
    else
        echo "  ✗ $topic (未找到)"
    fi
done

# 完成
echo ""
echo "[Step 5/5] 启动完成!"
echo "=============================================="
echo ""
echo "可用操作:"
echo "  1. 查看 RGB 图像:"
echo "     rqt_image_view /camera/color/image_raw"
echo ""
echo "  2. 查看深度图像:"
echo "     rqt_image_view /camera/depth/image_rect_raw"
echo ""
echo "  3. 查看相机内参:"
echo "     ros2 topic echo /camera/color/camera_info --once"
echo ""
echo "  4. 录制数据集:"
echo "     ros2 bag record /camera/color/image_raw /camera/depth/image_rect_raw \\"
echo "       /camera/color/camera_info /camera/depth/camera_info \\"
echo "       -o realsense_dataset"
echo ""
echo "  5. 查看图像话题频率:"
echo "     ros2 topic hz /camera/color/image_raw"
echo ""
echo "  6. 点云可视化 (需要点云模式):"
echo "     ros2 run rviz2 rviz2"
echo "     添加 PointCloud2 话题: /camera/color/points"
echo ""
echo "=============================================="
echo "按 Ctrl+C 停止相机"

trap "echo '正在停止...'; kill $CAMERA_PID 2>/dev/null; wait; exit" INT TERM
wait
