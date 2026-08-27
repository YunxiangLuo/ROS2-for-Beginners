# 第22章 实验代码：自动驾驶概述与 CARLA 基础

本章学习使用CARLA仿真平台和Python API进行自动驾驶仿真实验。

## 文件说明

| 文件 | 用途 | 运行方式 |
|------|------|----------|
| `install_carla.sh` | CARLA 0.9.16 一键安装脚本 | `bash install_carla.sh` |
| `explore_carla.py` | 连接CARLA并打印世界信息（地图、蓝图、天气等） | `python3 explore_carla.py` |
| `spawn_vehicles.py` | 在CARLA中生成多辆车辆并设置自动驾驶 | `python3 spawn_vehicles.py --num-vehicles 10` |

## 运行说明

### 步骤1：启动CARLA服务器

```bash
cd ~/carla
./CarlaUE4.sh -quality-level=Low
```

### 步骤2：运行Python脚本

```bash
# 终端2：探索CARLA世界信息
cd src/lab_code/ch22_lab/
python3 explore_carla.py

# 终端2（或新终端）：生成车辆
cd src/lab_code/ch22_lab/
python3 spawn_vehicles.py --num-vehicles 20
```

### 注意事项

- CARLA服务器至少需要4GB显存和8GB系统内存
- 若使用 `-quality-level=Low` 仍卡顿，可尝试 `-opengl` 参数
- 默认连接地址为 `localhost:2000`，可通过 `--host` 和 `--port` 参数修改
- 首次运行 `explore_carla.py` 前请确保已安装carla Python API

---

## 安装与编译

```bash
# CARLA 0.9.16 与 Python 3.12 环境
pip install numpy pyyaml
# 或使用本目录脚本自动安装 CARLA egg:
bash install_carla.sh
```

## 运行方法

```bash
# 先启动 CARLA 服务器: ./CarlaUE4.sh -quality-level=Low
python explore_carla.py --host localhost --port 2000   # 地图/生成点/天气探索
python spawn_vehicles.py --count 10                    # 生成背景交通车辆
```

## 运行结果

在已启动的 CARLA 环境中，`explore_carla.py` 输出地图、蓝图和天气信息，`spawn_vehicles.py` 输出生成的背景车辆数量。本目录未提供独立离线测试套件。
