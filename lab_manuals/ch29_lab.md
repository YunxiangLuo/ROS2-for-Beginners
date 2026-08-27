# 第29章 实验手册：行为决策与交通规则

> **对应理论章节**：第43章《行为决策与交通规则》  
> **实验课时**：2 课时  
> **实验代码**：`src/lab_code/ch29_lab/`

---

## 实验目标

1. 掌握 CARLA 交通灯 API 的使用方法，实现交通灯检测与停止
2. 实现基于有限状态机 (FSM) 的行为决策节点
3. 掌握 Traffic Manager 的配置方法，实现多车交通流

## 前置条件

- CARLA 0.9.16 已安装运行
- ROS 2 Jazzy 已配置
- `carla-ros-bridge` 已编译
- Python 3.8+ with `carla` pip package

---

## 练习29.1: 交通灯检测与停止

### 目标

编写一个 ROS2 节点，订阅 CARLA 的车辆状态和交通灯信息，当检测到红灯时控制车辆在停止线前停车。

### 实现步骤

#### 步骤1: 创建 ROS2 包

```bash
cd ~/ros2_course_ws/src
ros2 pkg create ch29_traffic_light \
    --build-type ament_python \
    --dependencies rclpy std_msgs geometry_msgs carla_msgs
```

#### 步骤2: 编写交通灯检测节点

创建 `ch29_traffic_light/ch29_traffic_light/traffic_light_detector.py`：

```python
import rclpy
from rclpy.node import Node
from carla_msgs.msg import CarlaTrafficLightStatusList, CarlaEgoVehicleControl
from geometry_msgs.msg import Point
import math

class TrafficLightDetector(Node):
    def __init__(self):
        super().__init__('traffic_light_detector')

        # 订阅交通灯状态
        self.tl_sub = self.create_subscription(
            CarlaTrafficLightStatusList,
            '/carla/traffic_light_status',
            self.traffic_light_callback,
            10
        )

        # 订阅自车位置（通过CARLA的odometry）
        self.ego_sub = self.create_subscription(
            Point,  # 实际使用 Odometry
            '/carla/ego_vehicle/odometry',
            self.ego_position_callback,
            10
        )

        # 发布控制指令
        self.cmd_pub = self.create_publisher(
            CarlaEgoVehicleControl,
            '/carla/ego_vehicle/vehicle_control_cmd',
            10
        )

        self.current_pose = None
        self.traffic_lights = []
        self.stop_signals = []

        # 控制定时器 (20Hz)
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('TrafficLightDetector 初始化完成')

    def traffic_light_callback(self, msg):
        """处理交通灯状态"""
        self.traffic_lights = msg.traffic_lights

    def ego_position_callback(self, msg):
        """更新自车位置"""
        self.current_pose = msg

    def get_distance_to_stop_line(self, tl_status):
        """计算车辆到停止线的距离"""
        # 实际实现需要根据交通灯位置和自车位置计算
        # 这里简化为读取交通灯关联的停止线位置
        if self.current_pose is None:
            return float('inf')

        # 假设 tl_status 包含 stop_line 位置信息
        stop_x = tl_status.stop_line_location.x
        stop_y = tl_status.stop_line_location.y
        dx = self.current_pose.x - stop_x
        dy = self.current_pose.y - stop_y
        return math.sqrt(dx*dx + dy*dy)

    def should_stop(self, tl_status):
        """判断是否需要停止"""
        STOP_DISTANCE = 12.0
        MARGIN = 3.0

        # 红灯或黄灯
        if tl_status.state not in [1, 2]:  # 1=Red, 2=Yellow
            return False, 0.0

        distance = self.get_distance_to_stop_line(tl_status)
        if distance < MARGIN:
            return True, distance
        if distance < STOP_DISTANCE:
            return True, distance
        return False, distance

    def control_loop(self):
        """控制主循环：判断交通灯并输出控制指令"""
        cmd = CarlaEgoVehicleControl()
        cmd.throttle = 0.3
        cmd.brake = 0.0
        cmd.steer = 0.0
        cmd.hand_brake = False
        cmd.reverse = False
        cmd.gear = 1
        cmd.manual_gear_shift = False

        for tl in self.traffic_lights:
            stop, distance = self.should_stop(tl)
            if stop:
                self.get_logger().warn(
                    f'红灯! 距离停止线: {distance:.2f}m'
                )
                cmd.throttle = 0.0
                # 根据距离线性刹车
                if distance < 5.0:
                    cmd.brake = 0.8
                else:
                    cmd.brake = 0.3
                break

        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### 步骤3: 配置 setup.py

```python
entry_points={
    'console_scripts': [
        'traffic_light_detector = ch29_traffic_light.traffic_light_detector:main',
    ],
},
```

#### 步骤4: 编译与测试

```bash
cd ~/ros2_course_ws
colcon build --packages-select ch29_traffic_light
source install/setup.bash

# 启动 CARLA 和 ros-bridge
ros2 launch carla_ros_bridge carla_ros_bridge.launch.py

# 运行节点
ros2 run ch29_traffic_light traffic_light_detector
```

### 预期结果

- 车辆在红灯前自动减速停止
- 绿灯亮起后自动恢复行驶
- 终端输出"红灯! 距离停止线: X.XXm"日志

### 思考题

1. 如何区分本车道和相邻车道的红绿灯？
2. 黄灯闪烁时应如何处理？
3. 多车道大路口可能有多个交通灯，如何选择相关的那个？

---

## 练习29.2: 设计 FSM 行为决策节点

### 目标

实现一个基于有限状态机的行为决策节点，包含巡航、跟车、停止三种基本状态。

### 实现步骤

#### 步骤1: 创建包

```bash
cd ~/ros2_course_ws/src
ros2 pkg create ch29_fsm_decision \
    --build-type ament_python \
    --dependencies rclpy std_msgs geometry_msgs carla_msgs
```

#### 步骤2: 编写 FSM 决策节点

创建 `ch29_fsm_decision/ch29_fsm_decision/fsm_decision.py`：

```python
import rclpy
from rclpy.node import Node
from enum import Enum
from carla_msgs.msg import (
    CarlaEgoVehicleControl,
    CarlaEgoVehicleStatus,
    CarlaTrafficLightStatusList
)
from std_msgs.msg import Float32
import math

class FsmState(Enum):
    INIT = 0
    CRUISE = 1
    FOLLOW = 2
    STOP = 3

class FSMDecisionNode(Node):
    def __init__(self):
        super().__init__('fsm_decision_node')

        # 状态定义
        self.state = FsmState.INIT
        self.state_names = {
            FsmState.INIT: 'INIT',
            FsmState.CRUISE: 'CRUISE',
            FsmState.FOLLOW: 'FOLLOW',
            FsmState.STOP: 'STOP',
        }

        # 参数
        self.SAFE_DISTANCE = 15.0  # 安全跟车距离 (m)
        self.CRUISE_SPEED = 10.0   # 巡航速度 (m/s)
        self.MIN_SPEED = 0.0

        # 订阅
        self.status_sub = self.create_subscription(
            CarlaEgoVehicleStatus,
            '/carla/ego_vehicle/vehicle_status',
            self.status_callback,
            10
        )
        self.lead_dist_sub = self.create_subscription(
            Float32,
            '/carla/ego_vehicle/lead_distance',
            self.lead_distance_callback,
            10
        )
        self.tl_sub = self.create_subscription(
            CarlaTrafficLightStatusList,
            '/carla/traffic_light_status',
            self.traffic_light_callback,
            10
        )

        # 发布
        self.cmd_pub = self.create_publisher(
            CarlaEgoVehicleControl,
            '/carla/ego_vehicle/vehicle_control_cmd',
            10
        )
        self.state_pub = self.create_publisher(
            Int32, '/carla/ego_vehicle/fsm_state', 10
        )

        # 状态
        self.current_speed = 0.0
        self.lead_distance = float('inf')
        self.have_red_light = False

        # 控制定时器
        self.timer = self.create_timer(0.1, self.update)

        self.get_logger().info('FSM决策节点启动，初始状态: CRUISE')
        self.state = FsmState.CRUISE

    def status_callback(self, msg):
        self.current_speed = math.sqrt(
            msg.velocity.x**2 + msg.velocity.y**2
        )

    def lead_distance_callback(self, msg):
        self.lead_distance = msg.data

    def traffic_light_callback(self, msg):
        self.have_red_light = any(
            tl.state == 1 for tl in msg.traffic_lights
        )

    def update(self):
        """定时执行状态机转移"""
        self.get_logger().debug(
            f'状态: {self.state_names[self.state]}, '
            f'速度: {self.current_speed:.1f}, '
            f'前车距离: {self.lead_distance:.1f}'
        )

        if self.state == FsmState.CRUISE:
            cmd = self._handle_cruise()
        elif self.state == FsmState.FOLLOW:
            cmd = self._handle_follow()
        elif self.state == FsmState.STOP:
            cmd = self._handle_stop()
        else:
            cmd = CarlaEgoVehicleControl()

        self.cmd_pub.publish(cmd)
        self.state_pub.publish(Int32(data=self.state.value))

    def _handle_cruise(self):
        cmd = CarlaEgoVehicleControl()
        cmd.throttle = 0.4
        cmd.brake = 0.0

        # 检测前车 → 切换到 FOLLOW
        if self.lead_distance < self.SAFE_DISTANCE:
            self._transition(FsmState.FOLLOW)
            return self._handle_follow()

        # 检测红灯 → 切换到 STOP
        if self.have_red_light and self.lead_distance < 30.0:
            self._transition(FsmState.STOP)
            return self._handle_stop()

        # 速度控制
        if self.current_speed < self.CRUISE_SPEED:
            cmd.throttle = 0.5
        else:
            cmd.throttle = 0.2

        return cmd

    def _handle_follow(self):
        cmd = CarlaEgoVehicleControl()

        # 前车消失 → CRUISE
        if self.lead_distance > self.SAFE_DISTANCE * 1.5:
            self._transition(FsmState.CRUISE)
            return self._handle_cruise()

        # 前车距离太近 → 刹车
        if self.lead_distance < 5.0:
            cmd.throttle = 0.0
            cmd.brake = 0.6
        elif self.lead_distance < self.SAFE_DISTANCE:
            cmd.throttle = 0.2
            cmd.brake = 0.1
        else:
            cmd.throttle = 0.3
            cmd.brake = 0.0

        # 目标速度不超过前车速度
        target_speed = min(self.CRUISE_SPEED,
                          self.lead_distance / 2.0)
        if self.current_speed > target_speed:
            cmd.throttle = 0.0
            cmd.brake = 0.3

        return cmd

    def _handle_stop(self):
        cmd = CarlaEgoVehicleControl()
        cmd.throttle = 0.0

        # 红灯消失 → CRUISE
        if not self.have_red_light:
            self._transition(FsmState.CRUISE)
            return self._handle_cruise()

        # 停车
        if self.current_speed > 1.0:
            cmd.brake = 0.7
        else:
            cmd.brake = 1.0  # 保持刹车

        return cmd

    def _transition(self, new_state):
        old_state = self.state
        self.state = new_state
        self.get_logger().info(
            f'状态转移: {self.state_names[old_state]} → '
            f'{self.state_names[new_state]}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = FSMDecisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### 步骤3: 前车距离计算工具节点

创建 `ch29_fsm_decision/ch29_fsm_decision/lead_distance_calc.py`：

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from carla_msgs.msg import CarlaEgoVehicleStatus
from derived_object_msgs.msg import ObjectArray
import math

class LeadDistanceCalculator(Node):
    def __init__(self):
        super().__init__('lead_distance_calculator')
        self.pub = self.create_publisher(
            Float32, '/carla/ego_vehicle/lead_distance', 10
        )
        self.ego_sub = self.create_subscription(
            CarlaEgoVehicleStatus, '/carla/ego_vehicle/vehicle_status',
            self.status_cb, 10
        )
        self.obj_sub = self.create_subscription(
            ObjectArray, '/carla/objects',
            self.objects_cb, 10
        )
        self.ego_x = self.ego_y = 0.0
        self.ego_yaw = 0.0
        self.timer = self.create_timer(0.1, self.publish_distance)

    def status_cb(self, msg):
        # 从 CarlaEgoVehicleStatus 提取位置和朝向
        pass  # 实际实现需要提取自车位置

    def objects_cb(self, msg):
        self.objects = msg.objects

    def publish_distance(self):
        # 计算最近的前车距离
        min_dist = float('inf')
        for obj in self.objects:
            if obj.classification == 'car':
                dx = obj.pose.position.x - self.ego_x
                dy = obj.pose.position.y - self.ego_y
                dist = math.sqrt(dx*dx + dy*dy)
                # 判断是否在同一车道、在前方
                if dist < min_dist:
                    min_dist = dist
        if min_dist < 100.0:
            self.pub.publish(Float32(data=min_dist))
```

#### 步骤4: 编译与测试

```bash
cd ~/ros2_course_ws
colcon build --packages-select ch29_fsm_decision
source install/setup.bash

# 启动
ros2 run ch29_fsm_decision fsm_decision_node
```

### 预期结果

- 无前车时保持巡航速度行驶
- 检测到前车进入跟车模式，保持安全距离
- 红灯前减速停止
- 终端显示状态转移日志

### 思考题

1. 如何处理多条车道的前车选择问题？
2. 如何增加变道状态？变道的触发条件是什么？
3. FSM 状态数量膨胀时如何管理？

---

## 练习29.3: 配置 Traffic Manager 实现交通流

### 目标

通过 CARLA Traffic Manager 控制多辆 AI 车辆，配置不同的驾驶风格，并与自车协调形成交通流。

### 实现步骤

#### 步骤1: 创建 Python 脚本

创建 `ch29_traffic_flow.py`（独立脚本，非 ROS2 节点）：

```python
#!/usr/bin/env python3

import carla
import random
import time
import argparse

def spawn_vehicles(world, blueprint_library, spawn_points, num):
    """在随机出生点生成车辆"""
    vehicles = []
    vehicle_bps = blueprint_library.filter('vehicle.*')
    spawn_points = random.sample(spawn_points, num)

    for i, sp in enumerate(spawn_points):
        bp = random.choice(vehicle_bps)
        bp.set_attribute('role_name', f'background_{i}')

        vehicle = world.spawn_actor(bp, sp)
        if vehicle:
            vehicles.append(vehicle)
            print(f'生成车辆 {i+1}/{num}: {bp.id}')

    return vehicles


def configure_traffic_manager(client, vehicles, mode='normal'):
    """配置 Traffic Manager 参数"""
    tm = client.get_trafficmanager(8000)

    # 全局设置
    tm.set_global_distance_to_leading_vehicle(3.0)
    tm.set_random_device_seed(42)
    tm.set_synchronous_mode(True)

    # 每辆车单独配置
    for i, vehicle in enumerate(vehicles):
        vehicle.set_autopilot(True, tm.get_port())

        if mode == 'aggressive':
            # 激进模式
            tm.set_desired_speed(vehicle, random.uniform(60, 90))
            tm.set_distance_to_leading_vehicle(
                vehicle, random.uniform(1.0, 2.5))
            tm.set_ignore_traffic_light_percentage(vehicle, 20)
            tm.set_ignore_signs_percentage(vehicle, 15)
            tm.set_lane_change_behavior(
                vehicle, random.choice([0, 1, 2]))
            tm.set_force_lane_change(vehicle, True)

        elif mode == 'conservative':
            # 保守模式
            tm.set_desired_speed(vehicle, random.uniform(30, 50))
            tm.set_distance_to_leading_vehicle(
                vehicle, random.uniform(5.0, 10.0))
            tm.set_ignore_traffic_light_percentage(vehicle, 0)
            tm.set_ignore_signs_percentage(vehicle, 0)
            tm.set_lane_change_behavior(vehicle, 0)

        else:  # normal
            # 普通模式
            tm.set_desired_speed(vehicle, random.uniform(40, 70))
            tm.set_distance_to_leading_vehicle(
                vehicle, random.uniform(3.0, 6.0))
            tm.set_ignore_traffic_light_percentage(vehicle, 5)
            tm.set_ignore_signs_percentage(vehicle, 2)
            tm.set_lane_change_behavior(
                vehicle, random.choice([0, 1]))

        print(f'已配置车辆 {i+1} ({mode})')

    return tm


def main():
    parser = argparse.ArgumentParser(
        description='CARLA Traffic Manager Demo')
    parser.add_argument('--host', type=str, default='localhost')
    parser.add_argument('--port', type=int, default=2000)
    parser.add_argument('--tm-port', type=int, default=8000)
    parser.add_argument('--vehicles', type=int, default=20)
    parser.add_argument('--mode', type=str, default='normal',
                       choices=['normal', 'aggressive', 'conservative',
                                'mixed'])
    parser.add_argument('--sync', action='store_true',
                       default=True)
    args = parser.parse_args()

    # 连接 CARLA
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    world = client.get_world()

    # 获取出生点
    spawn_points = world.get_map().get_spawn_points()
    if len(spawn_points) < args.vehicles:
        print(f'警告: 只有 {len(spawn_points)} 个出生点')
        args.vehicles = len(spawn_points)

    # 清理现有车辆
    for actor in world.get_actors().filter('vehicle.*'):
        actor.destroy()

    # 生成车辆
    blueprint_library = world.get_blueprint_library()
    vehicles = spawn_vehicles(
        world, blueprint_library, spawn_points, args.vehicles
    )
    print(f'成功生成 {len(vehicles)} 辆车')

    # 配置 Traffic Manager
    if args.mode == 'mixed':
        # 混合模式：随机分配驾驶风格
        modes = ['aggressive', 'normal', 'conservative']
        for vehicle in vehicles:
            mode = random.choice(modes)
            configure_traffic_manager(
                client, [vehicle], mode=mode)
    else:
        configure_traffic_manager(
            client, vehicles, mode=args.mode)

    # 主循环
    try:
        if args.sync:
            settings = world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05
            world.apply_settings(settings)

        tm = client.get_trafficmanager(args.tm_port)
        frame = 0

        while True:
            if args.sync:
                world.tick()
            else:
                time.sleep(0.05)

            frame += 1
            if frame % 200 == 0:  # 每10秒输出统计
                speeds = []
                for v in vehicles:
                    vels = v.get_velocity()
                    speed = 3.6 * math.sqrt(
                        vels.x**2 + vels.y**2 + vels.z**2
                    )
                    speeds.append(speed)
                avg_speed = sum(speeds) / len(speeds)
                max_speed = max(speeds)
                min_speed = min(speeds)
                print(
                    f'交通流统计 | 平均: {avg_speed:.1f} km/h '
                    f'| 最大: {max_speed:.1f} km/h '
                    f'| 最小: {min_speed:.1f} km/h'
                )

    except KeyboardInterrupt:
        print('用户中断')
    finally:
        print('清理车辆...')
        for vehicle in vehicles:
            vehicle.destroy()
        print('完成')


if __name__ == '__main__':
    import math
    main()
```

#### 步骤2: 编写 ROS2 封装节点（可选）

如果需要将 Traffic Manager 集成到 ROS2 中：

```python
# ch29_traffic_flow_ros.py
import rclpy
from rclpy.node import Node
import carla

class TrafficFlowManager(Node):
    def __init__(self):
        super().__init__('traffic_flow_manager')

        self.declare_parameter('host', 'localhost')
        self.declare_parameter('port', 2000)
        self.declare_parameter('num_vehicles', 20)
        self.declare_parameter('mode', 'normal')

        host = self.get_parameter('host').value
        port = self.get_parameter('port').value
        num = self.get_parameter('num_vehicles').value
        mode = self.get_parameter('mode').value

        self.client = carla.Client(host, port)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.tm = self.client.get_trafficmanager(8000)

        self.vehicles = []
        self._spawn_and_configure(num, mode)

        self.timer = self.create_timer(0.1, self.update_stats)

    def _spawn_and_configure(self, num, mode):
        # ... 生成和配置逻辑同上 ...
        pass

    def update_stats(self):
        # 发布交通流统计信息
        pass
```

#### 步骤3: 运行

```bash
# 基本运行 - 20辆车，普通模式
python ch29_traffic_flow.py --vehicles 20 --mode normal

# 激进模式 - 30辆车，闯灯概率20%
python ch29_traffic_flow.py --vehicles 30 --mode aggressive

# 混合模式 - 不同车辆不同驾驶风格
python ch29_traffic_flow.py --vehicles 50 --mode mixed

# 同步模式
python ch29_traffic_flow.py --sync --vehicles 20 --mode normal
```

### 预期结果

- 多辆 AI 车辆在交通流中自主行驶
- 不同类型车辆表现出不同的驾驶风格
- 终端每 10 秒输出交通流统计

### 思考题

1. Traffic Manager 和自定义 FSM 决策的区别与适用场景？
2. 如何通过 TM 参数模拟高峰期的拥堵路况？
3. 混合模式中，激进车辆和保守车辆相遇时会发生什么？

---

## 附加挑战

### 挑战 1: FSM + 交通灯集成

将练习 29.1 和 29.2 合并，在 FSM 中增加 `CHANGE_LEFT` 和 `CHANGE_RIGHT` 状态。实现自动变道超车逻辑。

### 挑战 2: 行人避让

在 Scenario Runner 中定义"行人横穿"场景，让自车在感知到行人时从 CRUISE 切换到 AVOID 状态。

### 挑战 3: 交通流 + 自车

在练习 29.3 的 Traffic Manager 交通流中，加入 ROS2 FSM 控制的自车。评估自车在不同交通密度和驾驶风格下的表现。

---

## 提交要求

| 项目 | 要求 |
|------|------|
| 代码 | 注释完整，符合 ROS2 编码规范 |
| 实验报告 | 包含代码结构、实验结果截图、问题分析 |
| 视频(可选) | 录制车辆行为决策过程演示 |

## 参考资源

- [CARLA Traffic Manager 文档](https://carla.readthedocs.io/en/latest/ts_traffic_manager/)
- [Scenario Runner 文档](https://carla-scenariorunner.readthedocs.io/)
- [ROS2 CARLA Bridge 源码](https://github.com/carla-simulator/ros-bridge)
