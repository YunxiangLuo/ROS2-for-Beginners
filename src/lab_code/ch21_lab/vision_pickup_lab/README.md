# vision_pickup_lab — 第 21 章视觉引导抓取实验

- 包类型：`ament_python`
- ROS 2 Jazzy + xArm6（`xarm_ros2_arm_only`）+ AR 码检测

## 简介

本章将视觉检测与机械臂抓取结合：`tf2_camera_broadcaster` 发布相机外参
TF，AR 码检测与抓取服务由 `course_lab_utils` 提供，`vision_pickup_pipeline`
编排「检测→定位→抓取→放置」全流程。

| 程序 | 内容 |
|------|------|
| `tf2_camera_broadcaster` | 相机-底盘静态 TF（参数/标定文件） |
| `aruco_pick_server` | AR 码引导抓取 Action 服务 |
| `vision_pickup_pipeline` | 视觉抓取全流程 |

## 构建

```bash
cd <robot_sim_demo 工作区>
source /opt/ros/jazzy/setup.bash
# 需先构建并 source xarm_description 底层与 course_lab_utils
colcon build --symlink-install --packages-select vision_pickup_lab
source install/setup.bash
```

## 运行

```bash
# 1. 启动 xArm 仿真
ros2 launch xarm_ros2_arm_only arm_only.launch.py

# 2. 发布相机 TF（另开终端）
ros2 run vision_pickup_lab tf2_camera_broadcaster --ros-args \
  -p x:=0.15 -p z:=0.25 -p child_frame:=camera_link

# 3. 启动抓取服务与流水线
ros2 run vision_pickup_lab aruco_pick_server
ros2 run vision_pickup_lab vision_pickup_pipeline
```

## 测试

```bash
colcon test --packages-select vision_pickup_lab
colcon test-result --all
```

## 运行结果

流水线检测到 AR 码后广播其 3D 位姿，机械臂规划并执行抓取，终端输出
`Pick succeeded`。截图保存至 `docs/images/vision_pickup.png`。
