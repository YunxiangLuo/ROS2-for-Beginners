# 第36章 自动驾驶概述与CARLA基础

> **课程**：ROS2 Python 编程  
> **章节**：第36章  
> **课时**：2 课时（90 分钟）  
> **教学方式**：讲授 + 演示  

---

## 学习目标

本章将带领读者系统认识自动驾驶技术：先了解自动驾驶技术的发展历程与 SAE J3016 分级标准，在此基础上掌握感知—规划—控制三层系统架构；随后进入开源自动驾驶仿真平台 CARLA，理解其以 Unreal Engine 为基础的核心概念与架构，学会搭建 CARLA 仿真环境并开展基本操作，最终掌握 World、Map、Actor、Sensor 等核心概念的实际用法。

## 36.1 自动驾驶技术发展背景

### 36.1.1 自动驾驶分级标准

SAE International（国际自动机工程师学会）于2014年发布了J3016标准，将自动驾驶分为L0-L5六个等级，该标准已被全球广泛采用。

| 等级 | 名称 | 定义 | 驾驶操作 | 环境监控 | 动态驾驶任务接管 | 典型场景 |
|:----:|------|------|:--------:|:--------:|:---------------:|:--------:|
| **L0** | 无自动化 | 人类驾驶员完成全部驾驶任务 | 人类 | 人类 | 人类 | — |
| **L1** | 驾驶辅助 | 系统对方向盘或加减速单项提供支持 | 人类+系统 | 人类 | 人类 | 自适应巡航(ACC) |
| **L2** | 部分自动化 | 系统同时控制方向盘和加减速 | 系统 | 人类 | 人类 | 特斯拉Autopilot |
| **L3** | 有条件自动化 | 系统完成全部驾驶操作，人类需在请求时接管 | 系统 | 系统 | 人类(接管请求) | 奥迪A8 Traffic Jam Pilot |
| **L4** | 高度自动化 | 系统完成全部驾驶操作，无需人类接管（限定场景） | 系统 | 系统 | 系统 | Robotaxi（限定区域） |
| **L5** | 完全自动化 | 系统在任何条件下完成全部驾驶操作 | 系统 | 系统 | 系统 | 全场景无人驾驶 |

目前业界量产车主要集中在L2-L2+级别，Robotaxi（如Waymo、百度Apollo）在限定区域内达到L4水平。

### 36.1.2 官方定义再阐释：ODD、DDT 与 L2/L3 分水岭

SAE J3016（2021 年修订版）删除了旧版的 L0-L2 vs L3-L5 两段式表述，改用三个工程概念精确切分：DDT（动态驾驶任务，含横向/纵向控制、OBD 与对事件的响应）、ODD（运行设计域，如道路类型、车速区间、天气）与 DDT 后备（fallback）。L2 与 L3 的分水岭不在「谁握着方向盘」，而在「系统承担 DDT 的哪一部分、事故责任如何转移」：L2 由驾驶员持续执行 OBD 与后备，L3 条件激活时系统执行全部 DDT、驾驶员只做紧急后备——这正是练习题第 1 题（L2/L3 核心区别）的官方答案来源。工程上 L3 被普遍认为是分水岭，是因为它首次把「责任」从人转移到系统，直接决定了传感器冗余、功能安全与数采方案的设计基准。

### 36.1.3 自动驾驶系统架构

自动驾驶系统通常采用**感知→规划→控制**三层架构：

```
┌──────────────────────────────────────────────────┐
│                    感知层 (Perception)              │
│  ┌──────────┐ ┌──────────┐ ┌──────────────────┐  │
│  │ 相机     │ │ 激光雷达  │ │ 毫米波雷达 / 超声波│  │
│  │ (目标检测)│ │ (点云)   │ │ (距离/速度)      │  │
│  └────┬─────┘ └────┬─────┘ └───────┬──────────┘  │
│       └────────────┼───────────────┘              │
│                    ▼                              │
│          ┌─────────────────┐                      │
│          │ 多传感器融合     │                      │
│          │ (障碍物/车道/信号灯)│                    │
│          └────────┬────────┘                      │
├───────────────────┼──────────────────────────────┤
│                    规划层 (Planning)               │
│          ┌────────┴────────┐                      │
│          │   行为决策       │                      │
│          │ (换道/跟车/停车)  │                     │
│          └────────┬────────┘                      │
│          ┌────────┴────────┐                      │
│          │   运动规划       │                      │
│          │ (路径/速度/轨迹)  │                     │
│          └────────┬────────┘                      │
├───────────────────┼──────────────────────────────┤
│                    控制层 (Control)                │
│          ┌────────┴────────┐                      │
│          │   PID / MPC     │                      │
│          │  (油门/刹车/转向) │                     │
│          └────────┬────────┘                      │
├───────────────────┼──────────────────────────────┤
│                    ▼                              │
│              执行机构 (底盘/线控)                   │
└──────────────────────────────────────────────────┘
```

**感知层**：通过摄像头、激光雷达、毫米波雷达、超声波等传感器采集环境数据，进行目标检测、语义分割、深度估计等任务。

**规划层**：根据感知结果做出驾驶决策（行为选择）并生成可执行的轨迹（路径+速度曲线）。

**控制层**：将规划好的轨迹转化为油门、刹车、转向等底层控制信号，通过PID或MPC等控制器执行。

## 36.2 CARLA仿真平台

### 36.2.1 CARLA概述

CARLA（Car Learning to Act）是一个基于Unreal Engine 4的开源自动驾驶仿真器，由巴塞罗那计算机视觉中心（CVC）研发并维护。

作为开源项目，CARLA 的完整代码托管于 GitHub，社区活跃；其基于 Unreal Engine 4 的高保真渲染能够提供逼真的光照、反射和物理效果。在传感器方面，CARLA 配备灵活的传感器套件，支持 RGB 相机、深度相机、语义分割相机、LIDAR、雷达、GNSS、IMU 等多种类型；内置交通管理器（Traffic Manager）可控制 NPC 车辆行为，模拟真实交通流；地图方面则提供 Town01–Town12 共 12 种不同风格的场景。开发者可通过完整的 Python API 实现车辆控制、传感器配置与场景编辑，并可借助 carla_ros_bridge 实现 CARLA 与 ROS 2 的无缝对接。

### 36.2.2 CARLA环境搭建与版本选型

**版本选型建议**：

| 版本 | 发布时间 | UE版本 | Python API | 推荐用途 |
|:----:|:--------:|:------:|:----------:|:---------|
| 0.9.13 | 2022.06 | UE4.26 | 0.9.13 | 历史版本，仅供兼容性对比 |
| **0.9.16** | — | — | **0.9.16** | **本课程基线** |
| 0.9.14 | 2022.12 | UE4.27 | 0.9.14 | 历史版本，不用于本课程 |

本课程采用 **CARLA 0.9.16**。

**安装步骤（Ubuntu 24.04 / WSL2）**：

推荐使用仓库根目录安装器，它会同时安装 CARLA 运行库、Python API、图形/无头依赖和固定版本的 ROS 2 Bridge：

```bash
cd /path/to/Technologies-of-ROS2-Programming-master
bash setup_course.sh --with-carla
source ~/.config/ros2-course/env.bash
```

如果只需要 CARLA 与 Bridge，可以使用 `bash setup_course.sh --carla-only`。

如果 CARLA 服务端运行在 Windows 主机上，WSL2 中不需要再下载 Linux 服务端，使用：

```bash
bash setup_course.sh --carla-bridge-only
source ~/.config/ros2-course/env.bash
```

手动安装时，至少需要以下系统依赖：

```bash
# 1. 安装依赖
sudo apt-get update
sudo apt-get install -y libomp5 libegl1 libgl1 libgl1-mesa-dri \
  libglx-mesa0 libvulkan1 mesa-vulkan-drivers vulkan-tools xauth xvfb

# 2. 下载 CARLA 0.9.16
mkdir -p ~/carla
cd ~/carla
wget https://tiny.carla.org/carla-0-9-16-linux -O CARLA_0.9.16.tar.gz
tar -xzf CARLA_0.9.16.tar.gz

# 3. 下载Additional Maps（可选，推荐）
wget https://carla-releases.b-cdn.net/Linux/AdditionalMaps_0.9.16.tar.gz
tar -xzf AdditionalMaps_0.9.16.tar.gz -C ~/carla

# 4. 在隔离环境中安装Python API
python3 -m venv --system-site-packages ~/.venvs/carla-0.9.16
~/.venvs/carla-0.9.16/bin/python -m pip install pygame numpy carla==0.9.16

# 5. 验证安装
cd ~/carla
# WSL2 下 Intel/AMD GPU 通过 WSLg 的 D3D12 后端运行
if grep -qi microsoft /proc/version; then
  export GALLIUM_DRIVER="${GALLIUM_DRIVER:-d3d12}"
fi
./CarlaUE4.sh -quality-level=Low  # 启动CARLA服务器
```

**安装验证清单**：

| 检查项 | 验证命令 | 预期结果 |
|--------|---------|---------|
| GPU驱动/OpenGL | `glxinfo -B` | 显示物理GPU或 WSLg D3D12 后端，非llvmpipe |
| CARLA服务器 | `./CarlaUE4.sh -quality-level=Low` | UE4窗口正常显示 |
| Python API | `python3 -c "from importlib.metadata import version; print(version('carla'))"` | 打印 `0.9.16` |
| 客户端连接 | `python3 -c "import carla; c=carla.Client('localhost',2000); c.set_timeout(5); print(c.get_server_version())"` | 返回服务器版本元组 |
| 磁盘空间 | `df -h ~/carla` | 可用空间 >10GB |

**常见问题与排障**：

| 问题 | 可能原因 | 解决方案 |
|------|---------|---------|
| `CarlaUE4.sh` 启动后黑屏退出 | GPU不支持或驱动版本过低 | 使用 `./CarlaUE4.sh -vulkan` 或升级显卡驱动 |
| WSL2 中 OpenGL 显示 `llvmpipe` | WSLg 未选择 D3D12 后端 | 先执行 `export GALLIUM_DRIVER=d3d12`，再运行 `glxinfo -B` |
| `import carla` 失败 | Python包未正确安装 | `~/.venvs/carla-0.9.16/bin/python -m pip install carla==0.9.16` |
| `Client.__init__()` 连接超时 | CARLA服务器未启动 | 确认 `./CarlaUE4.sh` 已运行，检查端口2000 |
| UE4窗口卡死无响应 | 显存不足 | 使用 `-quality-level=Low` 降低画质 |
| `AdditionalMaps` 不显示 | 附加地图未解压到正确目录 | 确保解压到 CARLA 根目录（与 CarlaUE4.sh 同级） |
| `libomp5` 缺失 | 系统缺少OpenMP库 | `sudo apt-get install libomp5` |

### 36.2.3 官方安装文档要点：三种安装方式与版本匹配纪律

CARLA 官方文档《Installation》页明确三种安装方式：预编译版本（binary）、源码编译（source）与 Docker 镜像，并强调**版本匹配纪律**——`carla==0.9.x` 的 Python API 与模拟器版本必须完全一致（`pip install carla` 的版本号要和下载包一致），否则脚本报 `client is not running` 或 API 不可用；这与本章 36.2.2 的版本选型建议完全对应。文档同时给出显卡要求（≥6 GB 显存推荐 RTX 20 系以上）与 Linux/Windows 双平台支持说明。官方《First steps》教程强调：首次运行必须确认 `CarlaUE4` 正常启动渲染窗口后再运行 Python 脚本，因为客户端连不上时最常见的错误是模拟器未就绪而非代码问题。

### 36.2.4 CARLA核心概念

CARLA的软件架构围绕以下核心概念构建：

```
┌─────────────────────────────────────────────────┐
│                    CARLA Server                    │
│  ┌─────────┐  ┌─────────────────────────────┐    │
│  │  World   │──│     Actor 管理              │    │
│  │─────────│  │  ┌─────────┐ ┌──────────┐   │    │
│  │  Map     │  │  │ Vehicle │ │ Walker   │   │    │
│  │  Weather │  │  ├─────────┤ ├──────────┤   │    │
│  │  Traffic │  │  │ Sensor  │ │ 其他Actor│   │    │
│  │  Manager │  │  └─────────┘ └──────────┘   │    │
│  └─────────┘  └─────────────────────────────┘    │
│                         │                         │
│                    ┌────┴────┐                    │
│                    │Blueprint│                    │
│                    │ (工厂模式)│                   │
│                    └─────────┘                    │
└──────────────────────┬──────────────────────────┘
                       │ Python API (TCP连接)
┌──────────────────────┴──────────────────────────┐
│                   CARLA Client                     │
│  ┌─────────────────────────────────────────────┐  │
│  │  carla.Client → carla.World → Actor控制     │  │
│  └─────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────┘
```

| 核心概念 | 说明 | Python类 |
|----------|------|----------|
| **World** | 仿真世界的实例，包含地图、天气、Actor等所有元素 | `carla.World` |
| **Map** | 仿真地图，包含道路网络、交通标志、地形等 | `carla.Map` |
| **Actor** | 世界中的动态实体，包括车辆、行人、传感器 | `carla.Actor` |
| **Blueprint** | Actor的模板/工厂，定义了Actor的属性 | `carla.ActorBlueprint` |
| **Sensor** | 特殊Actor，用于收集环境数据（相机、LIDAR等） | `carla.Sensor` |
| **Traffic Manager** | 交通流控制器，管理NPC车辆的行为 | `carla.TrafficManager` |

**基本Python API示例**：

```python
import carla

# 连接CARLA服务器
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)

# 获取世界
world = client.get_world()

# 获取蓝图库
blueprint_library = world.get_blueprint_library()

# 生成车辆
vehicle_bp = blueprint_library.find('vehicle.tesla.model3')
spawn_point = world.get_map().get_spawn_points()[0]
vehicle = world.spawn_actor(vehicle_bp, spawn_point)

# 生成RGB相机
camera_bp = blueprint_library.find('sensor.camera.rgb')
camera_bp.set_attribute('image_size_x', '800')
camera_bp.set_attribute('image_size_y', '600')
camera_bp.set_attribute('fov', '90')
camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)

# 传感器回调
def process_image(image):
    image.save_to_disk(f'output/{image.frame:06d}.png')

camera.listen(lambda data: process_image(data))

# 控制车辆
vehicle.apply_control(carla.VehicleControl(throttle=0.5, steer=0.0))
```

### 36.2.5 官方核心概念与 API 用法：Blueprint、Town 与天气

CARLA 官方文档把《Core concepts》拆为 Blueprint 库（Actor 的「工厂模板」，对应练习题第 3 题的工厂模式）、世界（World/Actor/Episode）与交通系统三块。官方示例对天气的设置方式是 `world.set_weather(carla.WeatherParameters.ClearNoon)` 或逐字段构造 `carla.WeatherParameters(sun_altitude=...)`——雨天黄昏场景只需组合 `cloudiness`、`precipitation`、`sun_altitude`（黄昏约 5°~15°）与 `sun_azimuth`，即可复现练习题第 4 题的全部要素。Town 地图方面，官方文档推荐 Town01/Town05 做基础驾驶测试（路网规整、红绿灯齐备）、Town03 做城市场景（环岛与立交）、Town04 偏高速与变道测试；`carla.get_available_maps()` 可列出全部地图，官方还支持 `OpenDRIVE` 高精地图导入——与 36.3.1 的 Town 地图类型一节对应。

## 36.3 CARLA世界与地图

### 36.3.1 Town地图类型

CARLA 0.9.16提供12个官方地图（Town01-Town12），各具特色：

| 地图 | 风格 | 特点 | 推荐用途 |
|:----:|:----:|------|:--------:|
| Town01 | 简单小镇 | 直线道路，基础交叉口 | 入门练习 |
| Town02 | 小型城市 | 小规模街区，T型路口 | 基础测试 |
| Town03 | 中型城市 | 环岛、立交桥，车道多 | 综合测试 |
| Town04 | 大型城市 | 大型环岛，多车道高速 | 高速路测试 |
| Town05 | 城市+高速 | 多种道路类型混合 | 完整场景 |
| Town06 | 长距离 | 高速公路长廊，多出入口 | 长距测试 |
| Town07 | 乡村道路 | 狭窄道路，无车道线 | 乡村场景 |
| Town08 | 秘密小镇 | 住宅区，邻里道路 | 小区测试 |
| Town09 | 市中心 | 密集建筑，复杂路口 | 城市挑战 |
| Town10 | 大城市 | 天际线，多种建筑 | 视觉测试 |
| Town11 | 欧洲风格 | 欧式小镇，石板路 | 风格测试 |
| Town12 | 港口区域 | 码头、集装箱、货轮 | 特殊场景 |

通过以下方式切换地图：

```python
client.load_world('Town03')
# 或
world = client.get_world()
world.unload_map_layer(carla.MapLayer.All)  # 控制地图图层
```

### 36.3.2 天气与环境系统

CARLA提供丰富的动态天气系统，通过`carla.WeatherParameters`控制：

```python
# 预设天气
weather_presets = [
    carla.WeatherParameters.ClearNoon,       # 晴天正午
    carla.WeatherParameters.CloudyNoon,       # 多云正午
    carla.WeatherParameters.WetNoon,          # 雨后正午
    carla.WeatherParameters.WetCloudyNoon,    # 雨后多云
    carla.WeatherParameters.MidRainyNoon,     # 中雨
    carla.WeatherParameters.HardRainNoon,     # 大雨
    carla.WeatherParameters.SoftRainNoon,     # 小雨
]

# 自定义天气参数
weather = carla.WeatherParameters(
    cloudiness=50.0,          # 云量 0-100
    precipitation=30.0,       # 降水 0-100
    precipitation_deposits=20.0,  # 积水 0-100
    wind_intensity=10.0,      # 风速 0-100
    sun_azimuth_angle=180.0,  # 太阳方位角
    sun_altitude_angle=60.0,  # 太阳高度角
    fog_density=5.0,          # 雾浓度 0-100
    fog_distance=30.0,        # 雾距离
    wetness=50.0,             # 路面湿润度 0-100
)
world.set_weather(weather)
```

**天气参数详解**：

| 参数 | 范围 | 效果 |
|------|:----:|------|
| `cloudiness` | 0-100 | 控制云层覆盖，0=晴天，100=阴天 |
| `precipitation` | 0-100 | 降雨强度，0=无雨，100=暴雨 |
| `precipitation_deposits` | 0-100 | 路面积水，与实际降水相关 |
| `wind_intensity` | 0-100 | 影响树木摆动和粒子效果 |
| `fog_density` | 0-100 | 雾的浓度，影响能见度 |
| `fog_distance` | 0-∞ | 雾的可见距离 |
| `wetness` | 0-100 | 路面反光湿润效果 |
| `sun_azimuth_angle` | 0-360 | 太阳水平方向角 |
| `sun_altitude_angle` | -90-90 | 太阳高度角，负值=夜晚 |

**天气渐变效果**：

```python
# 天气平滑过渡
weather = carla.WeatherParameters(
    cloudiness=0.0, precipitation=0.0, precipitation_deposits=0.0
)
world.set_weather(weather)

# 20秒内渐变到雨天
target_weather = carla.WeatherParameters(
    cloudiness=80.0, precipitation=60.0, precipitation_deposits=40.0
)
for i in range(200):
    t = i / 200.0
    current = carla.WeatherParameters(
        cloudiness=weather.cloudiness * (1-t) + target_weather.cloudiness * t,
        precipitation=weather.precipitation * (1-t) + target_weather.precipitation * t,
        precipitation_deposits=weather.precipitation_deposits * (1-t) + target_weather.precipitation_deposits * t,
    )
    world.set_weather(current)
    time.sleep(0.1)
```

### 36.3.3 国际课程中的 CARLA 教学实践

The Construct 的「Self-Driving Cars with ROS 2 and CARLA」课程把 CARLA 当作 ROS 2 的传感器世界：先用官方 Python API 生成车辆与相机，再经桥接把 `/carla/ego_vehicle` 与图像话题接进 ROS 2，后续感知与规划全部用 ROS 2 工具链完成。课程教学设计先「用手动键盘控制车辆建数据集」、再「实现自动跟随车道」，与本章 Part 4 各章的推进节奏一致。Udacity 的自驾车工程师课程则采用「仿真 + 数据集」双轨：用其自研模拟器练感知（车道线、车辆检测）与控制（PID 纵向），再用真实公开数据集评估模型鲁棒性。两门课共同的教学结论是：仿真平台（CARLA/Udacity 模拟器）的价值在于「可复现、可切片、可注入故障」——降雨、夜间、传感器失效都可以一键注入，这正对应 36.2.4 世界与天气的可编程性。建议读者完成章节练习后，按官方《First steps》教程重新走一遍「生成车辆 → 生成传感器 → 设定天气」的最小闭环，建立对官方 API 的第一手感。

## 36.4 本章小结

本章围绕自动驾驶技术基础与 CARLA 仿真平台展开。自动驾驶分级标准（SAE J3016 L0-L5）定义了从纯人工驾驶到完全自动化的六个等级，而典型自动驾驶系统遵循感知→规划→控制的三层架构。CARLA 是基于 UE4 的开源自动驾驶仿真器，提供高保真的仿真环境，其核心概念包括 World、Map、Actor、Blueprint、Sensor 与 Traffic Manager；平台提供 12 种不同风格的地图（Town01-Town12）和丰富的动态天气系统，并支持通过 Python API 进行车辆生成、传感器配置和天气控制等完整的仿真控制。

## 36.5 练习题

1. 简述SAE J3016标准中L2和L3的核心区别。为什么L3被认为是自动驾驶技术的重要分水岭？

2. 自动驾驶系统的感知—规划—控制三层架构中，每一层的主要职责是什么？请举例说明各层使用的典型算法。

3. 什么是CARLA中的Blueprint？它与面向对象编程中的"工厂模式"有何关联？请结合代码说明如何使用Blueprint生成一辆车辆和一台相机传感器。

4. 假设你需要在CARLA的Town03地图中搭建一个雨天黄昏的测试场景，请编写相应的Python代码实现（包括天气参数设置、地图加载和车辆生成）。

---

> 本章扩展内容综合翻译自 SAE International 官方 J3016《驾驶自动化分级》标准文件、CARLA 官方文档（carla.readthedocs.io，含核心概念、Python API 与教程）、The Construct 的「Self-Driving Cars with ROS 2 and CARLA」课程与 Udacity 的自动驾驶工程师纳米学位课程。原文均为英文，此处为中文编译，供课后巩固与进阶阅读。

> 参考来源：
> - SAE International 官方 J3016 标准文件：https://www.sae.org/standards/content/j3016_202104/
> - CARLA 官方文档（安装、核心概念、Python API 与教程）：https://carla.readthedocs.io/
> - CARLA 官方代码库：https://github.com/carla-simulator/carla
> - The Construct —— Self-Driving Cars with ROS 2 and CARLA 课程：https://www.theconstructsim.com/
> - Udacity —— 自动驾驶工程师纳米学位：https://www.udacity.com/
