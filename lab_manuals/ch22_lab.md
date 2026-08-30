# 第22章 实验指导书：自动驾驶概述与CARLA基础

> **对应理论章节**：第36章《自动驾驶概述与CARLA基础》  
> **实验课时**：2 课时  
> **实验代码**：`src/lab_code/ch22_lab/`  

---

本实验使用 Ubuntu 24.04 + ROS 2 Jazzy + CARLA 0.9.16 环境。CARLA 服务端和 ROS 2 Bridge 必须先完成安装。

推荐在课程仓库根目录统一安装：

```bash
cd /path/to/Technologies-of-ROS2-Programming-master
bash setup_course.sh --with-carla
source ~/.config/ros2-course/env.bash
```

安装器会将 CARLA 放在 `~/carla`，将固定版本 Bridge 编译到 `~/carla_ws`，并安装 `libomp5`、Vulkan/OpenGL、Xvfb 等运行依赖。

## 实验目标

- 学会安装并验证CARLA仿真环境
- 掌握CARLA Python API的基本使用方法
- 能够连接CARLA服务器并获取世界信息
- 学会在CARLA中生成车辆并控制其运动

---

## 实验准备

### 硬件环境
- PC with NVIDIA GPU（推荐4GB以上显存）
- 至少 8GB RAM
- 可用磁盘空间 30GB+

### 软件环境
- Ubuntu 24.04
- ROS 2 Jazzy
- Python 3.12+
- CARLA 0.9.16

---

## 练习 22.1：环境预检查

### 目标
在安装CARLA之前，检查系统环境是否满足要求，避免安装过程中出现问题。

### 步骤

**步骤1：检查GPU与驱动**
```bash
# 检查NVIDIA GPU
nvidia-smi
# 期望输出: GPU型号、驱动版本（推荐≥470）、CUDA版本
# 注意: 如输出 "command not found"，需安装NVIDIA驱动

# 检查OpenGL渲染器
sudo apt-get install mesa-utils
glxinfo -B | grep "OpenGL renderer"
# 期望: 显示物理GPU；WSL2 可显示 D3D12 后端，不应使用 llvmpipe
if grep -qi microsoft /proc/version; then
  export GALLIUM_DRIVER="${GALLIUM_DRIVER:-d3d12}"
fi
```

**步骤2：检查系统依赖与磁盘空间**
```bash
# 检查系统依赖
dpkg -l | grep libomp5 || echo "需要安装 libomp5"

# 检查磁盘空间（CARLA需要约20GB）
df -h ~ | awk 'NR==2 {print "可用空间: " $4}'
```

**步骤3：检查Python环境**
```bash
python3 --version   # 需 ≥ 3.8
pip3 --version      # 确认pip已安装
pip3 list 2>/dev/null | grep -E "numpy|pygame" || echo "需要安装 numpy/pygame"
```

**步骤4：检查网络与端口**
```bash
# 检查端口2000是否被占用（CARLA默认端口）
ss -tlnp | grep :2000 || echo "端口2000可用"

# 检查能否访问CARLA下载服务器
curl -sI https://tiny.carla.org/carla-0-9-16-linux | head -1
# 期望: HTTP/2 200 或 403（服务器可达）
```

### 检查清单
- [ ] `nvidia-smi` 显示GPU信息
- [ ] OpenGL渲染器为物理GPU
- [ ] 磁盘可用空间 ≥ 20GB
- [ ] Python ≥ 3.8
- [ ] pip 已安装
- [ ] 端口2000未被占用
- [ ] CARLA下载服务器可达

---

## 练习 22.2：CARLA安装验证

### 目标
完成 CARLA 0.9.16 的安装，验证服务器能否正常启动，并通过 Python 客户端确认连接。

### 步骤

**步骤1：下载并解压CARLA**

```bash
cd ~
mkdir -p carla && cd carla

# 下载CARLA本体
wget https://tiny.carla.org/carla-0-9-16-linux -O CARLA_0.9.16.tar.gz
tar -xzf CARLA_0.9.16.tar.gz

# 下载附加地图（可选）
wget https://carla-releases.b-cdn.net/Linux/AdditionalMaps_0.9.16.tar.gz
tar -xzf AdditionalMaps_0.9.16.tar.gz -C ~/carla
```

**步骤2：安装Python依赖**

```bash
python3 -m venv --system-site-packages ~/.venvs/carla-0.9.16
~/.venvs/carla-0.9.16/bin/python -m pip install pygame numpy carla==0.9.16
```

**步骤3：启动CARLA服务器**

```bash
cd ~/carla
./CarlaUE4.sh -quality-level=Low
```

> 首次启动可能需要较长时间，`-quality-level=Low` 可降低GPU负载。
> 服务器启动后会在 2000 端口监听客户端连接。

**步骤4：验证连接**

新建终端，执行：

```bash
cd src/lab_code/ch22_lab/
python3 explore_carla.py
```

预期输出（类似）：

```
CARLA 版本: 0.9.16
当前地图: /Game/Carla/Maps/Town03
可生成点数量: 76
Actor 数量: 1
天气: ClearNoon
```

---

## 练习 22.3：探索CARLA世界

### 目标
在CARLA世界中遍历不同的地图和天气，理解World、Map、Weather等核心概念。

### 步骤

**步骤1：切换地图**

```python
import carla

client = carla.Client('localhost', 2000)
client.set_timeout(10.0)

# 加载不同地图
world = client.load_world('Town01')
print(f'当前地图: {world.get_map().name}')

world = client.load_world('Town05')
print(f'当前地图: {world.get_map().name}')
```

**步骤2：尝试不同天气**

```python
import time

presets = [
    carla.WeatherParameters.ClearNoon,
    carla.WeatherParameters.CloudyNoon,
    carla.WeatherParameters.WetNoon,
    carla.WeatherParameters.MidRainyNoon,
    carla.WeatherParameters.HardRainNoon,
    carla.WeatherParameters.WetCloudySunset,
]

for i, preset in enumerate(presets):
    world.set_weather(preset)
    print(f'天气 {i+1}/{len(presets)} 已设置')
    time.sleep(2.0)
```

**步骤3：获取地图路网信息**

```python
map = world.get_map()
spawn_points = map.get_spawn_points()
topology = map.get_topology()

print(f'可生成点数量: {len(spawn_points)}')
print(f'路网段数量: {len(topology)}')

# 打印前5个生成点坐标
for i, pt in enumerate(spawn_points[:5]):
    loc = pt.location
    print(f'点 {i}: ({loc.x:.1f}, {loc.y:.1f}, {loc.z:.1f})')
```

**步骤4：查看所有蓝图**

```python
bp_lib = world.get_blueprint_library()

# 筛选车辆蓝图
vehicle_bps = bp_lib.filter('vehicle.*')
print(f'可用车辆蓝图: {len(vehicle_bps)}')
for bp in vehicle_bps[:5]:
    print(f'  - {bp.id}')

# 筛选传感器蓝图
sensor_bps = bp_lib.filter('sensor.*')
print(f'可用传感器蓝图: {len(sensor_bps)}')
for bp in sensor_bps:
    print(f'  - {bp.id}')
```

---

## 练习 22.4：Python API基础

### 目标
使用CARLA Python API在仿真世界中生成车辆、配置传感器并实现基础控制。

### 步骤

**步骤1：生成车辆**

使用 `spawn_vehicles.py` 脚本在Town03中随机生成多辆车辆：

```bash
cd src/lab_code/ch22_lab/
python3 spawn_vehicles.py --num-vehicles 10
```

该脚本的功能：
1. 连接CARLA服务器
2. 获取Town03地图的生成点
3. 随机选择车辆蓝图
4. 在随机生成点生成车辆
5. 打印每辆车的位置信息

**步骤2：设置Traffic Manager**

```python
# 获取Traffic Manager
tm = client.get_trafficmanager(8000)
tm.set_global_distance_to_leading_vehicle(2.0)  # 跟车距离
tm.set_synchronous_mode(False)                    # 异步模式

# 让生成的车辆自动驾驶
for vehicle in vehicles:
    vehicle.set_autopilot(True, tm.get_port())
```

**步骤3：附加RGB相机传感器**

```python
# 配置相机蓝图
camera_bp = bp_lib.find('sensor.camera.rgb')
camera_bp.set_attribute('image_size_x', '1280')
camera_bp.set_attribute('image_size_y', '720')
camera_bp.set_attribute('fov', '110')
camera_bp.set_attribute('sensor_tick', '0.05')  # 20FPS

# 相机安装位置（前挡风玻璃处）
camera_transform = carla.Transform(
    carla.Location(x=1.5, z=2.4),
    carla.Rotation(pitch=0, yaw=0, roll=0)
)

# 生成并附着到车辆
camera = world.spawn_actor(
    camera_bp, camera_transform, attach_to=vehicle
)

# 定义回调保存图像
def save_image(image):
    image.save_to_disk(f'_out/{image.frame:06d}.png')

camera.listen(lambda image: save_image(image))
```

**步骤4：手动控制车辆**

```python
import pygame

# 使用键盘控制车辆移动
def control_vehicle(vehicle):
    control = carla.VehicleControl()
    control.throttle = 0.5
    control.steer = 0.0
    control.brake = 0.0

    # 根据按键更新控制
    keys = pygame.key.get_pressed()
    if keys[pygame.K_UP]:
        control.throttle = 0.7
    if keys[pygame.K_DOWN]:
        control.brake = 0.7
    if keys[pygame.K_LEFT]:
        control.steer = -0.3
    if keys[pygame.K_RIGHT]:
        control.steer = 0.3

    vehicle.apply_control(control)
```

## 实验总结

- 掌握了CARLA服务器的启动与客户端连接方法
- 理解了World、Map、Blueprint、Actor等核心概念
- 学会了使用Python API生成车辆和控制车辆
- 掌握了传感器（相机）的配置与数据采集
- 了解了Traffic Manager的基本用法

## 补充：Docker 部署方案（备用）

对于无法直接运行 CARLA 的环境（如 GPU 性能不足、非 Ubuntu 系统），可使用 Docker 运行 CARLA：

```bash
# 1. 拉取 CARLA Docker 镜像
docker pull carlasim/carla:0.9.16

# 2. 运行 CARLA 服务器（需 NVIDIA Container Toolkit）
docker run --name carla \
  --privileged \
  --gpus all \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -e SDL_VIDEODRIVER=x11 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  carlasim/carla:0.9.16 \
  /bin/bash ./CarlaUE4.sh -vulkan -quality-level=Low

# 3. Python 客户端直接在宿主机运行（连接 localhost:2000）
  pip install carla==0.9.16  # 客户端库必须与服务端版本一致
python3 -c "import carla; c=carla.Client('localhost',2000); c.set_timeout(10); print(c.get_server_version())"
```

> 注意：Docker 方式性能开销较大，推荐仅用于开发和功能验证。正式实验建议使用原生安装。

## 思考题

1. CARLA采用C/S架构设计，这种设计有什么优点？在仿真实验中如果网络延迟较大，会有什么影响？

2. 在同一场景中生成大量车辆（如50辆以上）时，可能会遇到什么性能问题？你有什么优化建议？

3. 对比CARLA中的RGB相机和语义分割相机，它们在数据格式和应用场景上有何不同？
