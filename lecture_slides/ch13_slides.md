# 第13章 YOLO + ROS 2 目标检测

## 第1页: 章节目录
- 13.1 目标检测概述
- 13.2 YOLOv8 模型架构
- 13.3 cv_bridge 图像转换
- 13.4 YOLO ONNX 推理节点
- 13.5 检测结果 3D 定位
- 13.6 目标跟随控制
- 13.7 自定义检测消息
- 13.8 练习与总结

---

## 第2页: YOLO 核心思想

```
YOLO = 将检测视为回归问题, 单次前向即可输出边界框

输入 (640×640):           输出:
┌────────────┐            ┌─────────────────┐
│            │            │ [batch, 84, 8400]│
│            │  YOLOv8   │  ├ 4: bbox坐标    │
│            │  ──────>  │  ├ 1: obj置信度   │
│            │  one-pass │  └ 80: 类别分数   │
│            │            │ 每个grid cell 预测│
└────────────┘            └─────────────────┘

网格划分:
  ┌─┬─┬─┬─┬─┬─┬─┐
  │ │ │ │ │ │ │ │  每个 cell 预测 3 个 anchor box
  ├─┼─┼─┼─┼─┼─┼─┤
  │ │ │▓│▓│ │ │ │  ▓ = 有目标, □ = 无目标
  ├─┼─┼─┼─┼─┼─┼─┤
  │ │ │▓│▓│ │ │ │
  ├─┼─┼─┼─┼─┼─┼─┤
  │ │ │ │ │ │ │ │
  └─┴─┴─┴─┴─┴─┴─┘
```

**优势:** 速度快 (nano 模型 30+ FPS), 端到端训练, 全局上下文理解

---

## 第3页: YOLOv8 vs YOLOv5 改进

| 特性 | YOLOv5 | YOLOv8 |
|------|--------|--------|
| Backbone | CSPDarknet53 | C2f 模块 |
| Head | 耦合检测头 | 解耦分类/回归头 |
| Anchor | 基于锚点 | Anchor-Free |
| 标签分配 | 静态 | Task Aligned |
| 损失 | CIoU | CIoU + DFL |
| 数据增强 | Mosaic | Mosaic (最后10轮关闭) |

---

## 第4页: cv_bridge 图像转换

```
ROS 2 图像管道:

  Camera Driver              用户节点
  ┌────────────┐            ┌─────────┐
  │ sensor_msgs│  cv_bridge │ OpenCV  │
  │ /Image     │ ─────────> │ cv::Mat │
  │ RGB8/BGR8  │ <───────── │ ndarray │
  └────────────┘  cv_bridge └─────────┘
         │                        │
         ▼                        ▼
      /camera/                YOLO推理
      image_raw               图像处理

编码转换:
  rgb8 ↔ bgr8 ↔ mono8 ↔ bayer_rggb8 ↔ 16UC1 (深度)
```

**关键 API:**
```python
bridge = CvBridge()
cv_img = bridge.imgmsg_to_cv2(img_msg, 'bgr8')  # ROS → CV
img_msg = bridge.cv2_to_imgmsg(cv_img, 'bgr8')  # CV → ROS
```

---

## 第5页: YOLO ONNX 推理节点架构

```
┌─────────────────────────────────────────┐
│         YoloDetectorNode                │
│                                          │
│  /camera/image_raw ──> image_callback    │
│                            │             │
│                     CvBridge decode      │
│                            │             │
│                     preprocess()         │
│                       resize→normalize   │
│                            │             │
│                     ONNX inference       │
│                            │             │
│                     postprocess()        │
│                       NMS→box decode     │
│                            │             │
│         ┌──────────────────┼───────┐     │
│         ▼                  ▼       │     │
│   /detections         /detections/ │     │
│   (Detection2DArray)   annotated   │     │
│                        (Image)      │     │
└──────────────────────────────────────┘    │
└──────────────────────────────────────────┘
```

---

## 第6页: 检测结果消息格式

**标准视觉消息 (vision_msgs/Detection2DArray):**
```
Detection2DArray
├── Header header
└── Detection2D[] detections
    ├── BoundingBox2D bbox
    │   ├── Pose2D center      # 中心点
    │   └── float64 size_x, size_y
    └── ObjectHypothesisWithPose[] results
        ├── int32 class_id
        └── float64 score      # 置信度
```

**自定义消息 (YoloDetection):**
```
uint16 class_id
string class_name
float32 confidence
# bbox 坐标 (像素)
uint16 x1, y1, x2, y2
# bbox 中心 + 尺寸 (像素)
float32 center_x, center_y
float32 width, height
```

---

## 第7页: 2D → 3D 投影

```
针孔相机投影模型:

        世界坐标系
         │z
         │  /y
         │ /
         └──── x

  ┌───────────┐
  │  相机光心  │  ───── 光轴 ─────>  物体 P(x,y,z)
  │           │                    (真实3D位置)
  └─────┬─────┘
        │
        │ 焦距 f
        ▼
  ┌──────────────┐
  │   像平面      │
  │   p(u,v)     │  ← 2D 像素坐标
  │   (bbox中心) │
  └──────────────┘

投影公式:
  u = (fx * X + cx * Z) / Z
  v = (fy * Y + cy * Z) / Z

反投影 (逆公式):
  X = (u - cx) * Z / fx
  Y = (v - cy) * Z / fy
```

---

## 第8页: ckpt → ONNX → TensorRT 部署链

```
训练阶段:
  PyTorch (yolov8n.pt) ──> train + validate
                    │
                    ▼ export (ultralytics)
              ┌─ ONNX ──────┐
              │  yolov8n.onnx│
              └──────┬───────┘
                     │
         ┌───────────┼───────────┐
         ▼           ▼           ▼
    ONNX Runtime  OpenVINO   TensorRT
    (通用推理)    (Intel)    (NVIDIA)
     CPU/GPU       CPU/GPU    GPU only
```

**导出命令:**
```bash
# Python
from ultralytics import YOLO
model = YOLO('yolov8n.pt')
model.export(format='onnx', opset=12, simplify=True)

# CLI
yolo export model=yolov8n.pt format=onnx simplify
```

---

## 第9页: 目标跟随控制流程

```
  YOLO 检测          PID 控制器          机器人底盘
┌──────────┐      ┌──────────────┐      ┌──────────┐
│ detection│      │ angular_error│      │  cmd_vel │
│ (person) │─────>│ (从bbox偏移) │─────>│ (v, ω)   │
│ bbox中心 │      │              │      │          │
│ u=320+dx │      │ distance_err │      │ robot    │
└──────────┘      │ (从bbox大小) │      └──────────┘
                  └──────────────┘

控制律:
  ω = -Kp_angular * (u - u_center) / u_center
  v =  Kp_linear * (depth - target_distance)

期望: 保持在目标后方 1.5m, 图像中心对齐
```

---

## 第10页: 自定义检测消息与接口

**自定义 ROS 2 消息包结构:**
```
yolo_detector_interfaces/
├── CMakeLists.txt
├── package.xml
├── msg/
│   └── YoloDetection.msg       # 自定义检测消息
├── srv/
│   ├── SetClasses.srv           # 设置检测类别
│   └── SaveModel.srv            # 切换模型
└── action/
    └── TrackObject.action       # 目标跟踪 Action
```

**Python 发布者示例:**
```python
from yolo_detector_interfaces.msg import YoloDetection

det = YoloDetection()
det.class_id = class_id
det.class_name = self.class_names[class_id]
det.confidence = conf
det.x1, det.y1, det.x2, det.y2 = x1, y1, x2, y2
det.center_x = (x1 + x2) / 2.0
det.center_y = (y1 + y2) / 2.0
det_pub.publish(det)
```

---

## 第11页: RViz 检测可视化

```
  rviz2 ─ rviz 中显示 YOLO 检测:

  ┌─────────────────────────────────────┐
  │  Display                            │
  │   ├── TF ✓                          │
  │   ├── RobotModel ✓                  │
  │   ├── Camera/Image ✓               │
  │   │    └── topic: /detections/annotated
  │   ├── Map ✓                        │
  │   └── MarkerArray ✓                │
  │        └── topic: /detection_markers│
  └─────────────────────────────────────┘

  发布 Marker 可视化 (可选):
  # 在 YOLO 节点中额外发布 Marker 到 /detection_markers
  # 用于在 3D 视图中显示检测框位置
```

---

## 第12页: 本章总结

**核心要点回顾:**
1. YOLO 单阶段检测, 速度快, 部署灵活
2. cv_bridge 桥接 ROS Image ↔ OpenCV
3. ONNX Runtime 实现跨平台高效推理
4. 深度图 + bbox → 3D 目标定位
5. PID 控制实现基于视觉的目标跟随
6. 自定义消息接口便于系统集成

**关键命令:**
```bash
# 导出模型
python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt').export(format='onnx')"

# 运行推理节点
python3 yolo_detector.py --ros-args -p model_path:=yolov8n.onnx

# 查看检测结果
ros2 topic echo /detections
rqt_image_view /detections/annotated
```

**下一步: 第14章 视觉大模型 + ROS 2**