# 第31章 实验代码：综合项目：城区自动驾驶

基于ROS2与CARLA的城区自动驾驶综合项目。

## 项目概述

在CARLA Town03城区环境中，自车从起点A自动驾驶至终点B，全程约1.8km，需完成6个红绿灯路口、3处人行横道的安全通行，并避让10~20个动态交通参与者。

## 环境要求

- **操作系统**：Ubuntu 24.04
- **ROS 2**：Jazzy
- **CARLA**：0.9.16
- **Python**：3.12+
- **依赖**：CARLA Python API、ROS 2 Jazzy 与项目脚本所需的 Python 包

## 快速开始

### 1. 安装依赖

```bash
pip install carla numpy pyyaml
```

### 2. 启动CARLA

```bash
./CarlaUE4.sh -quality-level=Low -prefernvidia
```

### 3. 一键启动演示

```bash
cd src/lab_code/ch31_lab/
bash town_demo.sh
```

### 4. 运行测试

```bash
bash run_all_tests.sh
```

## 项目结构

```text
src/lab_code/ch31_lab/
├── README.md           # 本文件
├── main_pipeline.py    # 主自动驾驶管线节点
├── town_demo.sh        # 一键启动脚本
└── run_all_tests.sh    # 测试脚本
```

## 模块描述

| 脚本 | 职责 |
|------|------|
| `main_pipeline.py` | 主自动驾驶管线节点 |
| `town_demo.sh` | 启动城区自动驾驶演示 |
| `run_all_tests.sh` | 在已配置的运行环境中执行项目测试 |

## 四阶段开发

1. **路径跟踪**：Pure Pursuit + PID，实现A→B基本行驶
2. **避障**：LiDAR聚类、视觉车道线、Frenet轨迹规划
3. **交通灯**：视觉检测、停车/通行决策、精确停车
4. **完整闭环**：集成安全监控、系统诊断、多场景验证

## 验收标准

| 指标 | 阈值 |
|------|------|
| 碰撞次数 | = 0 |
| 红灯停车率 | 100% |
| 路线完成率 | ≥ 95% |
| 平均速度 | ≥ 15 km/h |
| 完成任务时间 | ≤ 10 min |
| 横向跟踪误差 | < 0.5 m |
| 停车位置精度 | < 1.0 m |

---

## 命令汇总

```bash
# 在课程根目录执行
cd src/lab_code/ch31_lab/

# 一键启动（CARLA + ros-bridge；检测到外部模块时再启动管线）
bash town_demo.sh
# 测试脚本（需 ROS 2 Jazzy/CARLA）
bash run_all_tests.sh
# 主管线单节点（需先提供管线模块目录）
cd ..
PYTHONPATH=. python3 -m ch31_lab.main_pipeline
```

## 验证

本章目录仅提供上述脚本。完整验证需要 CARLA、ROS 2 Jazzy 及其运行依赖；`main_pipeline.py` 还依赖目录外的运行组件，缺少这些组件时无法完成全链路演示。执行 `bash run_all_tests.sh` 前请先完成环境配置。
