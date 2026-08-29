# 第13章 实验代码：SLAM 一键建图综合实训

本章学习使用 ROS2 SLAM 工具包进行地图构建。

## 文件说明

| 文件 | 用途 | 运行方式 |
|------|------|----------|
| `slam_bringup.sh` | SLAM 建图启动脚本。启动机器人驱动、激光雷达、SLAM Toolbox 和 RViz 等节点 | `bash slam_bringup.sh` |
| `slam_map_runner.py` | 自动建图遍历节点。发布目标点驱动机器人自主移动，完成环境遍历和地图构建 | `python3 slam_map_runner.py` |

## 运行说明

### 步骤1：启动 SLAM 建图

```bash

cd src/lab_code/ch13_lab/

bash slam_bringup.sh
```

该脚本会启动：
- 机器人底盘驱动
- 激光雷达驱动
- SLAM Toolbox（同步模式）
- RViz2 显示界面

### 步骤2：自动遍历建图

在启动 SLAM 后，新开终端运行自动遍历节点：

```bash
cd src/lab_code/ch13_lab/
python3 slam_map_runner.py
```

### 步骤3：保存地图

建图完成后，保存地图：

```bash

ros2 run nav2_map_server map_saver_cli -f ~/map
```
