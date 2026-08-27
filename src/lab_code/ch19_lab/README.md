# 第19章 实验代码：相机驱动、标定与视觉检测

本章学习使用 OpenCV 和 ROS2 进行图像处理、颜色检测和 AR 标签识别。

## 文件说明

| 文件 | 用途 | 运行方式 |
|------|------|----------|
| `usb_cam_node.py` | USB 摄像头图像查看器。订阅 `/image_raw` 话题，使用 OpenCV 显示图像，按 ESC 退出 | `ros2 run vision_detection_lab usb_cam_node` |
| `color_detection_node.py` | 颜色检测节点。在图像中检测指定颜色的物体区域 | `ros2 run vision_detection_lab color_detection_node` |
| `ar_tag_detection_node.py` | AR 标签检测节点。检测 ArUco 标签并估计位姿 | `ros2 run vision_detection_lab ar_tag_detection_node` |
| `cv_bridge_demo.py` | cv_bridge 图像转换演示。将 ROS 图像转为 OpenCV 格式，在图像上画矩形，再转回 ROS 图像发布 | `ros2 run vision_detection_lab cv_bridge_demo` |

## 运行说明

```bash
# 终端1：启动摄像头图像节点
ros2 run vision_detection_lab usb_cam_node

# 终端2：运行视觉脚本
cd src/lab_code/ch19_lab/
ros2 run vision_detection_lab cv_bridge_demo
```

或者使用测试图像循环播放：

```bash
ros2 run image_tools cam2image
ros2 run vision_detection_lab cv_bridge_demo --ros-args -p image_topic:=/image
```

### cv_bridge_demo.py

功能：
1. 订阅 `/camera/color/image_raw` 图像话题
2. 使用 `cv_bridge` 将 ROS 图像转为 OpenCV 格式
3. 在图像上绘制橙色矩形
4. 显示处理后的图像
5. 将 OpenCV 图像转回 ROS 格式发布到 `/image_show`
