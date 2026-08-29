# av_sensor_kit — CARLA 传感器套件

传感器配置(预设/读写 YAML)与传感器健康管理节点。

## 目录结构

```
av_sensor_kit/
├── setup.py / package.xml
├── config/default_sensors.yaml              # 默认传感器配置(修复后为合法 YAML)
├── resource/av_sensor_kit                   # ament 资源标记
├── av_sensor_kit/
│   ├── sensor_config.py     # SensorConfig 数据类 + load/save + 预设
│   └── sensor_manager.py    # 传感器状态监控节点
└── test/
    ├── test_sensor_config.py
    └── test_sensor_manager.py
```

## 安装与编译

```bash
# ROS2 环境
cd <工作空间根目录>

colcon build --packages-select av_sensor_kit

source install/setup.bash
```

## 运行方法

```bash
# 传感器配置查看/校验 (纯 Python, 无需 ROS2)
python av_sensor_kit/sensor_config.py config/default_sensors.yaml

# 传感器管理节点 (ROS2)
ros2 run av_sensor_kit sensor_manager
# 服务: ~/reconfigure_sensors (std_srvs/Trigger)
# 话题: ~/sensor_status (std_msgs/String, 1Hz 状态摘要)
```

## 测试方法

```bash

cd src/av_sensor_kit

python -m pytest test -q
```

## 运行结果

```text
$ python av_sensor_kit/sensor_config.py config/default_sensors.yaml
SensorConfig(type='sensor.camera.rgb', name='front_rgb', x=1.5, y=0.0, z=1.8, ..., width=1280, height=720, fov=90, ...)
SensorConfig(type='sensor.camera.depth', name='front_depth', x=1.5, y=0.0, z=1.8, ..., width=1280, height=720, fov=90, ...)
SensorConfig(type='sensor.lidar.ray_cast', name='lidar_top', x=0.0, y=0.0, z=2.2, ..., channels=64, range=120.0, points_per_second=1300000, rotation_frequency=10.0)
SensorConfig(type='sensor.other.gnss', name='gnss', x=0.0, y=0.0, z=0.0, ...)
SensorConfig(type='sensor.other.imu', name='imu', x=0.0, y=0.0, z=0.0, ...)

$ cd src/av_sensor_kit && python -m pytest test -q
.........                                                               [100%]
9 passed in 0.13s
```

> 说明: 本机(Windows)未安装 ROS2/CARLA, 无法截取仿真运行画面,
> 运行结果以**真实终端输出**代替截图; 全部输出均可按上述命令复现。

## 本次修复记录

1. `config/default_sensors.yaml` 原为分号分隔的非法 YAML(`x: 1.5; y: 0.0; ...`,
   `yaml.safe_load` 直接抛 ScannerError) → 重写为标准 YAML;
2. `sensor_manager.py` 的 `~/sensor_status` 话题误用 `sensor_msgs/Image` 类型
   → 改为 `std_msgs/String`, 并新增 1Hz 状态摘要发布;
3. `package.xml` 为 ament_python 包却声明 `buildtool_depend ament_cmake` → 删除。
