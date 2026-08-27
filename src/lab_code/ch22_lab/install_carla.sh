#!/bin/bash
# CARLA 0.9.16 安装脚本
# 适用于 Ubuntu 24.04 + ROS 2 Jazzy

set -e

CARLA_VERSION="0.9.16"
CARLA_DIR="${HOME}/carla"

echo "===== CARLA ${CARLA_VERSION} 安装脚本 ====="
echo ""

# 检查系统依赖
echo "[1/5] 安装系统依赖..."
sudo apt-get update
sudo apt-get install -y libomp5 wget python3-pip python3-dev

# 创建安装目录
echo "[2/5] 创建安装目录 ${CARLA_DIR}..."
mkdir -p "${CARLA_DIR}"
cd "${CARLA_DIR}"

# 下载CARLA
echo "[3/5] 下载 CARLA ${CARLA_VERSION}..."
if [ ! -f "CARLA_${CARLA_VERSION}.tar.gz" ]; then
    wget "https://tiny.carla.org/carla-0-9-16-linux" -O "CARLA_${CARLA_VERSION}.tar.gz"
else
    echo "CARLA_${CARLA_VERSION}.tar.gz 已存在，跳过下载"
fi

# 解压
echo "[4/5] 解压 CARLA..."
tar -xzf "CARLA_${CARLA_VERSION}.tar.gz"
echo "解压完成"

# 下载附加地图
echo "[4.1/5] 下载附加地图..."
if [ ! -f "AdditionalMaps_${CARLA_VERSION}.tar.gz" ]; then
    wget "https://carla-releases.b-cdn.net/Linux/AdditionalMaps_${CARLA_VERSION}.tar.gz"
fi
tar -xzf "AdditionalMaps_${CARLA_VERSION}.tar.gz" -C "${CARLA_DIR}"
echo "附加地图解压完成"

# 安装Python API
echo "[5/5] 安装 CARLA Python API..."
pip install --upgrade pip
pip install pygame numpy

cd "${CARLA_DIR}/PythonAPI/carla"
pip install -e .

echo ""
echo "===== CARLA ${CARLA_VERSION} 安装完成 ====="
echo ""
echo "启动CARLA服务器:"
echo "  cd ${CARLA_DIR} && ./CarlaUE4.sh -quality-level=Low"
echo ""
echo "验证Python连接:"
echo "  python3 -c \"import carla; client = carla.Client('localhost', 2000); client.set_timeout(5); print(client.get_server_version())\""
