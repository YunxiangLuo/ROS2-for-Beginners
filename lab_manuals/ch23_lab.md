# 第23章 实验: CARLA-ROS2 桥接与车辆部署

> **对应理论章节**：第37章《CARLA-ROS2桥接与车辆部署》  
> **实验课时**：3 课时  
> **实验代码**：`src/lab_code/ch23_lab/`

## 实验目标
- 掌握CARLA仿真器与ROS2 Bridge的启动流程
- 学会通过Python脚本生成Ego Vehicle
- 能够在RViz2中可视化CARLA传感器数据
- 实现键盘控制车辆移动

## 实验环境
- ROS 2 Jazzy
- CARLA 0.9.16
- carla_msgs / carla_ros_bridge (源码安装)
- Python3 + keyboard库
- RViz2

## 实验准备

### 安装依赖
```bash
# 安装CARLA ROS2 Bridge（由课程安装脚本固定版本和提交）
cd /path/to/Technologies-of-ROS2-Programming-master
bash setup_course.sh --with-carla
source ~/.config/ros2-course/env.bash
cd ~/carla_ws
rosdep install --from-paths src --ignore-src -r -y
# 可选：使用 gezp 维护版（兼容性更好）
# ROS 2 Jazzy 下建议使用课程根目录 setup_course.sh 管理的固定 bridge commit

# 编译前确保PYTHONPATH包含CARLA的.egg文件
export CARLA_ROOT=~/carla
export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla:$CARLA_ROOT/PythonAPI/carla/dist

colcon build
source install/setup.bash
```

### 启动CARLA仿真器
```bash
# Linux
./CarlaUE4.sh -quality-level=Low

# Windows (CARLA安装在C:/CARLA)
C:/CARLA/CarlaUE4.exe -quality-level=Low
```

### 验证环境就绪
在开始实验前，执行以下预检查确认所有组件正常：

```bash
# 1. 检查CARLA服务器运行状态
pgrep -x CarlaUE4 && echo "CARLA服务器运行中" || echo "CARLA未启动"

# 2. 验证ROS2环境
ros2 node list 2>/dev/null | head -5 || echo "ROS2环境未加载，请先 source install/setup.bash"

# 3. 验证CARLA Python API
python3 -c "import carla; c=carla.Client('localhost',2000); c.set_timeout(5); print(f'CARLA {c.get_server_version()}')" 2>&1

# 4. 验证Bridge可启动
ros2 launch carla_ros_bridge carla_ros_bridge.launch.py --show-args 2>&1 | head -3
```

## 练习 23.1: 启动CARLA+ROS2 Bridge

### 步骤
1. 确保CARLA仿真器已在运行（观察Town地图窗口）
2. 启动ROS2 Bridge：

```bash
source ~/carla_ws/install/setup.bash
ros2 launch carla_ros_bridge carla_ros_bridge.launch.py \
  synchronous_mode:=False
```

3. 在新终端中验证Bridge节点已启动：

```bash
ros2 node list
# 应看到: /carla_ros_bridge
```

4. 查看Bridge发布的话题：

```bash
ros2 topic list | grep carla
```

### 预期输出
```
/carla/status
/carla/weather_control
/carla/traffic_lights
/carla/objects
...
```

### 检查要点
- [ ] CARLA仿真器窗口正常显示
- [ ] Bridge节点 `/carla_ros_bridge` 存在
- [ ] 至少能看到 `/carla/status` 话题

---

## 练习 23.2: 生成Ego Vehicle并在RViz中可视化

### 步骤
1. 运行 `spawn_ego.py` 生成Ego Vehicle：

```bash
cd src/lab_code/ch23_lab/
python3 spawn_ego.py --spawn-point 10
```

2. 验证新话题出现：

```bash
ros2 topic list | grep ego_vehicle
```

3. 启动RViz2并可视化传感器数据：

```bash
rviz2
```

4. 在RViz2中添加以下显示：
   - 添加 Image 显示，Topic 选择 `/carla/ego_vehicle/rgb_front/image`
   - 添加 PointCloud2 显示，Topic 选择 `/carla/ego_vehicle/lidar`
   - 添加 TF 显示，查看坐标树

### spawn_ego.py 参数说明
| 参数 | 默认值 | 说明 |
|:----:|:------:|------|
| `--host` | `localhost` | CARLA服务器地址 |
| `--port` | `2000` | CARLA服务器端口 |
| `--spawn-point` | `0` | 生成点索引 |
| `--role-name` | `ego_vehicle` | 角色名 |
| `--blueprint` | `vehicle.tesla.model3` | 车型Blueprint |

### 预期输出
```
Ego Vehicle spawned at: (x=36.28, y=52.38, z=0.30)
Role name: ego_vehicle
Blueprint: vehicle.tesla.model3
```

### 检查要点
- [ ] Ego Vehicle出现在CARLA仿真窗口中
- [ ] `/carla/ego_vehicle/*` 话题组出现
- [ ] RViz2中能看到RGB图像
- [ ] RViz2中能看到激光雷达点云
- [ ] TF树显示完整的坐标关系

---

## 练习 23.3: 键盘控制车辆移动

### 步骤
1. 确保CARLA + Bridge + Ego Vehicle 均在运行
2. 使用 `check_topics.py` 验证所有必要话题：

```bash
python3 check_topics.py
```

3. 启动键盘控制节点（使用ROS2内置工具）：

```bash
# 方法一：使用 teleop_twist_keyboard (需转换消息)
sudo apt install ros-jazzy-teleop-twist-keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/carla/ego_vehicle/vehicle_control_cmd
```

4. 或者直接发布控制指令测试：

```bash
# 前进 (油门 0.5)
ros2 topic pub /carla/ego_vehicle/vehicle_control_cmd \
  carla_msgs/msg/CarlaEgoVehicleControl \
  "{throttle: 0.5, brake: 0.0, steer: 0.0}" --once

# 右转 (油门 0.3 + 转向 0.5)
ros2 topic pub /carla/ego_vehicle/vehicle_control_cmd \
  carla_msgs/msg/CarlaEgoVehicleControl \
  "{throttle: 0.3, brake: 0.0, steer: 0.5}" --once

# 刹车急停
ros2 topic pub /carla/ego_vehicle/vehicle_control_cmd \
  carla_msgs/msg/CarlaEgoVehicleControl \
  "{throttle: 0.0, brake: 1.0, steer: 0.0}" --once
```

5. 测试自动驾驶模式切换：

```bash
# 启用自动驾驶
ros2 service call /carla/ego_vehicle/enable_autopilot \
  std_srvs/srv/SetBool "{data: true}"

# 观察车辆自动行驶 → 切回手动
ros2 service call /carla/ego_vehicle/enable_autopilot \
  std_srvs/srv/SetBool "{data: false}"
```

### 预期行为
- 发送throttle指令后车辆向前移动
- 发送steer指令后车辆转向
- 启用自动驾驶后车辆沿车道自动行驶
- 禁用自动驾驶后车辆停止自主控制

### 检查要点
- [ ] `check_topics.py` 报告所有话题正常
- [ ] 发布throttle指令后车辆移动
- [ ] 发布steer指令后车辆转向
- [ ] 自动驾驶模式下车辆自主行驶
- [ ] 模式切换功能正常

---

## 练习 23.4: 端到端验证与排障

### 目标
使用 `check_topics.py` 和 `bridge_launch.sh` 对CARLA-ROS2 Bridge进行完整的端到端验证，并掌握常见问题的诊断方法。

### 步骤

**步骤1：一键启动全栈**
```bash
cd src/lab_code/ch23_lab/
bash bridge_launch.sh ~/carla/CarlaUE4.sh 10
```
该脚本会自动：检查CARLA进程 → 启动CARLA（如未运行）→ Source ROS2工作空间 → 启动Bridge → 生成Ego Vehicle → 验证话题。

**步骤2：运行话题检查**
```bash
# 检查所有CARLA话题
python3 check_topics.py --verbose

# 检查特定role_name的话题
python3 check_topics.py --role-name ego_vehicle --verbose
```

**步骤3：诊断常用工具**
```bash
# 查看节点通信图
rqt_graph

# 查看话题带宽和频率
ros2 topic bw /carla/ego_vehicle/rgb_front/image
ros2 topic hz /carla/ego_vehicle/odometry

# 查看TF树
ros2 run tf2_tools view_frames.py

# 查看消息内容
ros2 topic echo /carla/ego_vehicle/vehicle_status --once
```

**步骤4：保存诊断报告**
```bash
# 保存环境快照
{
  echo "=== CARLA Version ==="
  python3 -c "import carla; c=carla.Client('localhost',2000); c.set_timeout(5); print(c.get_server_version())"
  echo "=== ROS2 Topics ==="
  ros2 topic list | grep carla
  echo "=== ROS2 Nodes ==="
  ros2 node list
  echo "=== TF Tree ==="
  ros2 run tf2_tools view_frames.py
} > carla_diagnostic_report.txt 2>&1
echo "诊断报告已保存至 carla_diagnostic_report.txt"
```

### 检查清单
- [ ] `bridge_launch.sh` 一键启动成功
- [ ] `check_topics.py` 报告所有关键话题正常
- [ ] `rqt_graph` 显示完整的话题通信图
- [ ] TF树文件 `frames.pdf` 生成成功
- [ ] 诊断报告包含所有组件信息

### 常见问题诊断表

| 症状 | 可能原因 | 诊断命令 | 解决方案 |
|------|---------|---------|---------|
| `ros2 node list` 无 `/carla_ros_bridge` | Bridge未启动或卡死 | `ps aux \| grep carla` | 重新启动Bridge，检查终端错误输出 |
| `ros2 topic list` 无 `/carla/` 话题 | Bridge未连接CARLA | `ros2 service call /carla/get_available_maps` | 确认CARLA服务器运行中，端口2000 |
| `/carla/ego_vehicle/*` 话题缺失 | Ego Vehicle未生成 | `ros2 service list \| grep carla` | 运行 `spawn_ego.py` |
| 图像话题无数据 | 相机传感器未附着 | `ros2 topic hz /carla/ego_vehicle/rgb_front/image` | 检查 `spawn_ego.py` 中传感器配置 |
| `ModuleNotFoundError: No module named 'carla'` | PYTHONPATH缺失 | `echo $PYTHONPATH` | `export PYTHONPATH=$PYTHONPATH:~/carla/PythonAPI/carla` |
| Bridge启动报 `carla_msgs` 错误 | 消息包未构建 | `ros2 pkg list \| grep carla` | `colcon build --packages-select carla_msgs` |
| `colcon build` 失败 | Python版本不匹配 | `python3 --version` | 确认 `.egg` 文件Python版本与系统一致 |

---

## 实验报告要求
1. 记录Bridge启动后的完整话题列表（`ros2 topic list`）
2. 截取RViz2中RGB图像和激光雷达点云的可视化效果图
3. 描述键盘控制车辆的实验过程与观察结果
4. 对比手动控制与自动驾驶模式下车辆行为差异
5. 记录遇到的问题及解决方法

## 思考题
1. 同步模式和异步模式对传感器数据时序有什么影响？
2. 如果同时生成两辆Ego Vehicle（role_name不同），Bridge如何处理话题命名？
3. CarlaEgoVehicleControl中的steer值为正时车辆往哪个方向转弯？
