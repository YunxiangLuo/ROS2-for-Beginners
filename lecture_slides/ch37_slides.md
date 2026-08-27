# 第37章 CARLA-ROS2 桥接与车辆部署

---

## 学习目标
- 理解CARLA-ROS2 Bridge架构与通信原理
- 掌握Ego Vehicle生成流程与配置
- 学会在RViz2中可视化传感器数据
- 掌握车辆控制接口与模式切换

---

## 37.1 CARLA-ROS2 桥接架构
- Bridge作为ROS2节点运行
- 通过CARLA Python API与仿真器通信
- 将传感器数据发布为ROS2话题
- 将ROS2控制指令下发到仿真器

```
ROS2节点 ←→ carla_ros_bridge ←→ CARLA Server
```

- 支持多Ego Vehicle管理
- 通过role_name区分不同车辆

---

## Bridge 数据流

```
┌──────────────┐    ROS2 Topics    ┌──────────────┐
│  ROS2 Nodes  │◄────────────────►│     Bridge   │
│  (RViz, Ctrl)│                   │  (ros_bridge)│
└──────────────┘                   └──────┬───────┘
                                          │ API
                                   ┌──────▼───────┐
                                   │CARLA Simulator│
                                   │  World/Vehicles│
                                   └──────────────┘
```

- 传感器数据: Camera → Image, Lidar → PointCloud2
- 控制指令: CarlaEgoVehicleControl → 车辆

---

## 话题映射 — 传感器
| CARLA组件 | ROS2话题 | 消息类型 |
|:---------:|:--------:|:--------:|
| RGB Camera | `/carla/.../rgb_front/image` | `sensor_msgs/Image` |
| Depth Camera | `/carla/.../depth_front/image` | `sensor_msgs/Image` |
| Lidar | `/carla/.../lidar` | `sensor_msgs/PointCloud2` |
| GNSS | `/carla/.../gnss` | `nav_msgs/Odometry` |
| IMU | `/carla/.../imu` | `sensor_msgs/Imu` |
| Radar | `/carla/.../radar_front` | `RadarDetectionArray` |

---

## 话题映射 — 车辆状态与控制
| 功能 | ROS2话题 | 消息类型 |
|:----:|:--------:|:--------:|
| 车辆状态 | `/carla/.../vehicle_status` | `CarlaEgoVehicleStatus` |
| 车辆控制 | `/carla/.../vehicle_control_cmd` | `CarlaEgoVehicleControl` |
| 里程计 | `/carla/.../odometry` | `nav_msgs/Odometry` |
| 碰撞检测 | `/carla/.../collision` | `CarlaCollisionEvent` |
| 车道入侵 | `/carla/.../lane_invasion` | `CarlaLaneInvasionEvent` |
| 自动驾驶 | `/carla/.../enable_autopilot` | `std_srvs/SetBool` |

---

## 同步模式 vs 异步模式
| 特性 | 同步模式 | 异步模式 |
|:----:|:--------:|:--------:|
| 步进控制 | ROS2控制仿真步进 | CARLA自主推进 |
| 时序一致性 | 严格保证 | 可能有延迟 |
| 传感器同步 | 所有传感器同时触发 | 各自独立触发 |
| 适用场景 | 强化学习、高精度控制 | 可视化、数据采集 |
| 启动参数 | `synchronous_mode:=True` | `synchronous_mode:=False` |

---

## 37.2 Ego Vehicle 部署
### Blueprint 选取流程
```python
client = carla.Client('localhost', 2000)
world = client.get_world()
blueprints = world.get_blueprint_library()
vehicle_bp = blueprints.find('vehicle.tesla.model3')
```

### 常用Ego车型
- `vehicle.tesla.model3` — Tesla Model 3
- `vehicle.toyota.prius` — Toyota Prius
- `vehicle.audi.etron` — Audi e-tron
- `vehicle.lincoln.mkz2017` — Lincoln MKZ

---

## 车辆生成流程

```
连接CARLA → 获取World → 选Blueprint
                              │
                      设置role_name = "ego_vehicle"
                              │
                        选Spawn Point
                              │
                       spawn_actor()
                              │
                      附加传感器 → 完成
```

- `role_name` 必须与Bridge配置一致
- `spawn_point` 从地图生成点列表中选取
- 生成后可通过灯光、颜色等属性定制

---

## role_name 与多车辆管理
| role_name | 话题前缀 | 说明 |
|:---------:|:--------:|:----:|
| `ego_vehicle` | `/carla/ego_vehicle/*` | 主控车辆 |
| `hero` | `/carla/hero/*` | 第二车辆 |
| `vehicle_1` | `/carla/vehicle_1/*` | 其他受控车辆 |

一个Bridge实例可管理多辆车，通过前缀区分话题。

---

## 37.3 RViz2 可视化
### 常用可视化配置
| RViz显示类型 | 话题 |
|:-----------:|:----:|
| Image | `/carla/.../rgb_front/image` |
| PointCloud2 | `/carla/.../lidar` |
| Odometry | `/carla/.../odometry` |
| TF | `/tf` |

### 启动RViz2
```bash
rviz2 -d src/carla_ros_bridge/rviz/carla_bridge.rviz
```

---

## TF 树结构
```
map
 └── odom
      └── ego_vehicle
           ├── ego_vehicle/rgb_front
           ├── ego_vehicle/lidar
           ├── ego_vehicle/imu
           └── ego_vehicle/gnss
```

查看TF树：
```bash
ros2 run tf2_tools view_frames.py
```

---

## 37.4 车辆控制接口
### CarlaEgoVehicleControl 消息
```
float32 throttle   # 油门 [0.0, 1.0]
float32 brake      # 刹车 [0.0, 1.0]
float32 steer      # 转向 [-1.0, 1.0]
bool   hand_brake  # 手刹
bool   reverse     # 倒车
bool   manual_gear_shift
int32  gear
```

### 发布控制指令
```python
cmd = CarlaEgoVehicleControl()
cmd.throttle = 0.5; cmd.steer = 0.0; cmd.brake = 0.0
pub.publish(cmd)
```

---

## 自动/手动模式切换
### 启用自动驾驶
```bash
ros2 service call /carla/ego_vehicle/enable_autopilot \
  std_srvs/srv/SetBool "{data: true}"
```

### 禁用自动驾驶
```bash
ros2 service call /carla/ego_vehicle/enable_autopilot \
  std_srvs/srv/SetBool "{data: false}"
```

### 编程切换
```python
switcher.set_autopilot(True)   # 启用
switcher.set_autopilot(False)  # 禁用
```

---

## 键盘控制映射
| 按键 | 动作 | 效果 |
|:----:|:----:|:----:|
| W | 加速 | throttle +0.05 |
| S | 刹车 | brake +0.05 |
| A | 左转 | steer -0.05 |
| D | 右转 | steer +0.05 |
| Space | 急停 | brake = 1.0 |
| R | 倒车切换 | reverse toggle |
| P | 自动驾驶切换 | autopilot toggle |

---

## 本章总结
### 关键知识点
1. CARLA-ROS2 Bridge 实现仿真 ⇄ ROS2 双向通信
2. Ego Vehicle 部署：Blueprint → role_name → Spawn
3. RViz2 可视化传感器话题和TF树
4. 车辆控制支持手动/自动驾驶模式切换

### 课后实验
- 启动CARLA+ROS2 Bridge
- 生成Ego Vehicle并在RViz中可视化
- 键盘控制车辆移动
