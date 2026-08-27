#!/bin/bash
# record_bag.sh — 多传感器数据录制脚本
#
# 用法:
#   bash record_bag.sh                  # 默认录制120秒
#   bash record_bag.sh -d 60            # 录制60秒
#   bash record_bag.sh -o my_dataset    # 自定义输出文件名
#   bash record_bag.sh --compress       # 启用zstd压缩
#
# 录制话题:
#   - /camera/rgb/image_raw        RGB图像
#   - /camera/depth/image_raw      深度图
#   - /camera/semseg/image_raw     语义分割图
#   - /lidar/points                LiDAR点云
#   - /radar/detections            RADAR检测
#   - /gnss/data                   GNSS定位
#   - /imu/data                    IMU六轴数据
#   - /tf /tf_static               坐标变换

DURATION=120
OUTPUT_DIR="./carla_datasets"
OUTPUT_NAME=""
COMPRESS=""

# 解析命令行参数
while [[ $# -gt 0 ]]; do
    case "$1" in
        -d|--duration)
            DURATION="$2"
            shift 2
            ;;
        -o|--output)
            OUTPUT_NAME="$2"
            shift 2
            ;;
        --compress)
            COMPRESS="--compression-mode file --compression-format zstd"
            shift
            ;;
        -h|--help)
            echo "用法: bash record_bag.sh [选项]"
            echo "  -d, --duration SEC    录制时长(秒), 默认120"
            echo "  -o, --output NAME     输出文件名"
            echo "  --compress            启用zstd压缩"
            echo "  -h, --help            显示帮助"
            exit 0
            ;;
        *)
            echo "未知选项: $1"
            exit 1
            ;;
    esac
done

# 创建输出目录
mkdir -p "$OUTPUT_DIR"

# 生成输出文件名
DATETIME=$(date +"%Y%m%d_%H%M%S")
if [ -z "$OUTPUT_NAME" ]; then
    OUTPUT_NAME="carla_dataset_${DATETIME}"
fi
OUTPUT_PATH="${OUTPUT_DIR}/${OUTPUT_NAME}"

echo "============================================"
echo "  多传感器数据录制"
echo "============================================"
echo "输出目录:  $OUTPUT_PATH"
echo "录制时长:  ${DURATION}秒"
echo "压缩:      $([ -n "$COMPRESS" ] && echo '启用(zstd)' || echo '未启用')"
echo "============================================"

# 检查是否有话题发布者
echo "检查传感器话题..."
TOPICS_TO_CHECK=(
    "/camera/rgb/image_raw"
    "/lidar/points"
    "/imu/data"
)

for topic in "${TOPICS_TO_CHECK[@]}"; do
    if ros2 topic info "$topic" 2>/dev/null | grep -q "Publisher count: 0"; then
        echo "  [警告] $topic 无发布者, 请确认传感器已启动"
    else
        echo "  [OK] $topic 正常"
    fi
done

echo ""
echo "开始录制 ${DURATION} 秒..."
echo "按 Ctrl+C 提前停止录制"

# 执行录制
ros2 bag record $COMPRESS \
    /camera/rgb/image_raw \
    /camera/depth/image_raw \
    /camera/semseg/image_raw \
    /lidar/points \
    /radar/detections \
    /gnss/data \
    /imu/data \
    /tf \
    /tf_static \
    -o "$OUTPUT_PATH" \
    --duration "$DURATION"

# 录制完成
if [ $? -eq 0 ]; then
    echo ""
    echo "============================================"
    echo "  录制完成"
    echo "============================================"
    ros2 bag info "${OUTPUT_PATH}" 2>/dev/null | head -n 20
    echo ""
    echo "回放命令: ros2 bag play ${OUTPUT_PATH}"
    echo "============================================"
else
    echo "录制被中断或出错"
fi
