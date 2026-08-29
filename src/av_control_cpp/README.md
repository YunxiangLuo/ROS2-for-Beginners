# av_control_cpp — 车辆控制器 (C++)

纵向 PID 控制器、横向 Pure Pursuit 控制器与一体化车辆控制器(ament_cmake, C++)。

## 目录结构

```
av_control_cpp/
├── CMakeLists.txt
├── package.xml
├── include/av_control_cpp/pid_controller.hpp   # PIDController + PurePursuitController (header-only)
└── src/
    ├── longitudinal_controller.cpp   # 纵向速度PID -> /throttle_cmd /brake_cmd
    ├── lateral_controller.cpp        # 横向Pure Pursuit -> /steering_cmd
    └── vehicle_controller.cpp        # 纵横向一体 -> /carla/ego_vehicle/vehicle_control_cmd
```

## 安装与编译 (需 ROS2 + 编译器)

```bash

cd <工作空间根目录>

colcon build --packages-select av_carla_interfaces av_control_cpp

source install/setup.bash
```

## 运行方法

```bash
# 纵向控制 (目标速度由参数 target_speed 指定, m/s)
ros2 run av_control_cpp longitudinal_controller --ros-args -p target_speed:=10.0

# 横向控制 (订阅 /waypoints 与 /ego_odom)
ros2 run av_control_cpp lateral_controller --ros-args -p lookahead_distance:=3.0

# 一体化控制器 (订阅 /waypoints 与 /ego_state)
ros2 run av_control_cpp vehicle_controller --ros-args -p target_speed:=10.0
```

话题接口:

| 节点 | 订阅 | 发布 |
|---|---|---|
| longitudinal_controller | `/carla/ego_vehicle/speed` (TwistStamped), `/ego_state` (EgoState) | `/throttle_cmd`, `/brake_cmd` (Float64) |
| lateral_controller | `/waypoints` (WaypointArray), `/ego_odom` (Odometry) | `/steering_cmd` (Twist) |
| vehicle_controller | `/waypoints` (WaypointArray), `/ego_state` (EgoState) | `/carla/ego_vehicle/vehicle_control_cmd` (ControlCmd) |

## 测试方法

本机无 C++ 编译器/ROS2, 通过接口包的静态一致性测试验证(检查源码中

`msg->field` 访问与 `.msg` 定义一致):

```bash

cd src/av_carla_interfaces

python -m pytest test -q
```

## 运行结果

```text
$ cd src/av_carla_interfaces && python -m pytest test -q
......                                                                   [100%]
6 passed in 0.05s
```

其中 `test_cpp_referenced_fields_exist` 逐函数校验了本包三个节点的消息字段访问,

修复前的 `EgoState.target_speed`、`VehicleControl`、`wp.position` 等

编译期错误均已消除(该测试曾失败, 修复后通过)。

> 说明: 本机(Windows)未安装 ROS2/CARLA, 无法截取仿真运行画面,
> 运行结果以**真实终端输出**代替截图; 全部输出均可按上述命令复现。

## 本次修复记录

1. `vehicle_controller.cpp` 引用了不存在的 `VehicleControl` 消息 → 改用接口包实际定义的 `ControlCmd`;
2. `EgoState.msg` 无 `target_speed`/`velocity` 字段 → 纵向目标速度改由 ROS 参数 `target_speed` 提供, 当前速度取 `EgoState.speed`;
3. `Waypoint.msg` 为平面字段(x/y/z), 源码误用 `wp.position.x` → 全部改为 `wp.x/y/z`;
4. 补充缺失的 `tf2/LinearMath/{Quaternion,Matrix3x3}.h` 头文件包含;
5. `vehicle_controller` 订阅话题 `/plan`(类型不匹配) → 改为 `/waypoints`(WaypointArray)。
