# vision_detection_lab — 第 19 章视觉检测实验

- 包类型：`ament_python`
- ROS 2 Jazzy + OpenCV + cv_bridge

## 简介

本章练习 ROS 2 视觉基础：USB 相机接入、cv_bridge 图像转换、颜色检测与
AR 码检测。颜色/AR 检测的规范实现由 `course_lab_utils` 提供，本包转发；
`usb_cam_node` 与 `cv_bridge_demo` 为独立教学实现。

| 程序 | 内容 |
|------|------|
| `usb_cam_node` | 发布 USB 相机图像话题 |
| `cv_bridge_demo` | cv_bridge 图像转换演示 |
| `color_detection_node` | HSV 颜色检测 |
| `ar_tag_detection_node` | ArUco 码检测 |

## 构建

```bash
cd <robot_sim_demo 工作区>
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select vision_detection_lab
source install/setup.bash
```

## 运行

```bash
# USB 相机（真实硬件）
ros2 run vision_detection_lab usb_cam_node

# 或使用仿真相机（robot_sim_demo 提供 /camera/image_raw）
ros2 launch robot_sim_demo gazebo2.launch.py

# 检测节点
ros2 run vision_detection_lab cv_bridge_demo
ros2 run vision_detection_lab color_detection_node
ros2 run vision_detection_lab ar_tag_detection_node
```

验证：

```bash
ros2 topic echo /camera/image_raw --field header --once
rqt_image_view
```

## 测试

```bash
colcon test --packages-select vision_detection_lab
colcon test-result --all
```

## 运行结果

`rqt_image_view` 中可见原图与检测结果叠加（颜色掩膜/AR 码框）。
截图保存至 `docs/images/vision_detection.png`。
