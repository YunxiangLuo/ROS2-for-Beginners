# 第42章 交通参与者感知

## 幻灯片 1: 封面
**第42章 交通参与者感知**
- 目标检测概述
- YOLO目标检测
- LiDAR点云聚类
- 多目标跟踪
- 实验：CARLA感知系统

---

## 幻灯片 2: 目标检测概述

**2D检测 vs 3D检测**

| | 2D检测 | 3D检测 |
|---|---|---|
| 输入 | RGB图像 | LiDAR/双目 |
| 输出 | 2D bbox + 类别 | 3D bbox + 类别 |
| 典型算法 | YOLO, Faster R-CNN | PointPillars, VoxelNet |
| 优点 | 速度快，语义丰富 | 空间信息完整 |

**核心指标**
```
  IoU = Area(交) / Area(并)
  mAP = 所有类别AP平均值
```

---

## 幻灯片 3: 目标检测流程

```
输入图像 → Backbone → Neck → Head → NMS → 检测框
              │         │       │       │
           ResNet     FPN    分类+回归  过滤重叠
```

**常见Backbone**:
- ResNet: 残差学习，梯度直达
- CSPNet: 跨阶段局部连接
- EfficientNet: 复合缩放

---

## 幻灯片 4: YOLO检测原理

**YOLO核心思想**：单个神经网络直接预测边界框和类别概率

- 图像划分为S×S网格
- 每个网格预测B个边界框
- 每个框预测(x,y,w,h) + confidence + C类概率

**单阶段 vs 双阶段**：
```
YOLO: 图像 → CNN → 检测结果  (一步到位)
Faster R-CNN: 图像 → CNN → RPN(候选框) → ROI Pool → 分类/回归
```

---

## 幻灯片 5: YOLOv8网络结构

```
YOLOv8 Architecture:

Input(640×640×3)
    ↓
[CSPDarknet Backbone]
    ├── C2f × 3 → P3 (1/8)
    ├── C2f × 6 → P4 (1/16)
    └── C2f × 3 → P5 (1/32) + SPPF
    ↓
[PAN-FPN Neck]
    P3 ←── P4 ←── P5 (Top-down FPN)
    P3 ──→ P4 ──→ P5 (Bottom-up PAN)
    ↓
[Decoupled Head]
    ├── 分类分支: Conv → Conv → Classify
    └── 回归分支: Conv → Conv → Regress + DFL
```

---

## 幻灯片 6: YOLO ROS2集成

```python
class YOLODetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        self.model = YOLO('yolov8n.pt')
        self.pub = self.create_publisher(
            Detection2DArray, '/yolo/detections', 10)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model(frame)[0]

        det_msg = Detection2DArray()
        for box in results.boxes:
            det = Detection2D()
            x1, y1, x2, y2 = box.xyxy[0]
            det.bbox.center.x = (x1 + x2) / 2
            det.bbox.center.y = (y1 + y2) / 2
            det.bbox.size_x = x2 - x1
            det.bbox.size_y = y2 - y1
            det_msg.detections.append(det)

        self.pub.publish(det_msg)
```

---

## 幻灯片 7: LiDAR点云预处理

**原始点云问题**：稀疏、噪声、数据量大

**体素滤波**：
```
┌────┬────┬────┐    每个体素(0.1m)内
│ ·· │ ·  │    │    取重心作为代表点
├────┼────┼────┤    N个点 → 1个点
│    │ ·· │ ·  │
├────┼────┼────┤    点数减少90%+
│ ·  │    │    │
└────┴────┴────┘
```

**直通滤波**：保留指定范围(x:0-50m, y:±20m, z:-2~3m)

**半径滤波**：统计R内邻点，少于阈值则移除

---

## 幻灯片 8: DBSCAN聚类算法

**基于密度的聚类，不需要指定K值**

```
参数:
  eps: 邻域半径 (例: 0.5m)
  minPts: 核心点最小邻点数 (例: 10)

三点类型:
  ● 核心点:  邻域内 ≥ minPts
  ◎ 边界点:  邻域内 < minPts，但在核心点邻域内
  ○ 噪声点:  其他

聚类结果:
  ○ ○  ●●●●●●  ○ ○    ← 3个聚类
     ○ ●●◎◎●●  ○
     ○ ●●●●●●  ○ ○
       ○ ○  ○ ○
```

---

## 幻灯片 9: 欧式聚类代码

```python
def euclidean_clustering(points, eps=0.5, min_samples=10):
    from sklearn.cluster import DBSCAN

    clustering = DBSCAN(
        eps=eps,
        min_samples=min_samples,
        metric='euclidean'
    ).fit(points)

    labels = clustering.labels_
    n_clusters = len(set(labels)) - (1 if -1 in labels else 0)

    clusters = []
    for label in range(n_clusters):
        mask = labels == label
        clusters.append(points[mask])

    return clusters
```

**聚类后障碍物表示**：
- 中心点(centroid)
- 3D边界框(min/max)
- 点云数量
- 运动状态

---

## 幻灯片 10: 多目标跟踪流水线

```
帧 t ──→ 检测结果 ──→ 预测(卡尔曼) ──→ 匈牙利匹配 ──→ 更新跟踪器
            │                              │
            │                     ┌────────┴────────┐
            │              ┌──────┴──────┐  ┌──────┴──────┐
            │              匹配成功        匹配失败
            │                 │              │
            │             更新状态      新目标 → 创建跟踪器
            │                                        │
            │                                   分配新ID
            │
            │ ←──────────── 丢失目标检测 ──────────→ │
                                                     │
                                              丢失 > 阈值?
                                                  │
                                              删除跟踪器(ID回收)
```

---

## 幻灯片 11: 卡尔曼滤波

**状态预测 → 观测更新 循环**

```
预测(时间更新):
  x̂ₖ = F·x̂ₖ₋₁ + B·uₖ
  Pₖ = F·Pₖ₋₁·Fᵀ + Q

更新(观测更新):
  Kₖ = Pₖ·Hᵀ·(H·Pₖ·Hᵀ + R)⁻¹
  x̂ₖ = x̂ₖ + Kₖ·(zₖ - H·x̂ₖ)
  Pₖ = (I - Kₖ·H)·Pₖ

状态向量 x = [cx, cy, cz, vx, vy, vz, w, h, l]ᵀ
观测向量 z = [cx, cy, cz, w, h, l]ᵀ
```

---

## 幻灯片 12: 匈牙利匹配与ID管理

**代价矩阵**: IoU作为匹配度量

```
         D₁    D₂    D₃    D₄
  T₁   [0.9   0.1   0.05  0.0 ]  ← 高IoU = 低代价
  T₂   [0.1   0.8   0.1   0.05]
  T₃   [0.0   0.1   0.7   0.2 ]
  T₄   [0.0   0.0   0.1   0.6 ]

匹配结果: T₁→D₁, T₂→D₂, T₃→D₃, T₄→D₄
```

**ID规则**:
- 新目标 → ID自增 (1, 2, 3, ...)
- 3帧确认 → 稳定跟踪标记
- 丢失10帧 → 删除，ID不立即重用

---

## 幻灯片 13: 检测方法对比

| 方法 | 传感器 | 输出 | FPS | 精度 |
|:----:|:------:|:----:|:---:|:----:|
| YOLOv8 | Camera | 2D bbox | ~100 | mAP@50: 53% |
| PointPillars | LiDAR | 3D bbox | ~60 | AP@70: 75% |
| DBSCAN | LiDAR | 点云簇 | 100+ | 参数依赖 |
| BEVFormer | Multi | BEV | ~25 | NDS: 55% |

**推荐组合**:
- Camera → YOLO → 2D检测
- LiDAR → DBSCAN → 障碍物位置
- 多传感器 → 卡尔曼滤波 → 稳定跟踪

---

## 幻灯片 14: 本章实验

**实验42.1**: CARLA相机YOLO目标检测
- 订阅RGB图像话题，运行YOLOv8推理
- 发布Detection2DArray消息
- 可视化检测结果

**实验42.2**: LiDAR点云聚类检测障碍物
- 订阅LiDAR点云话题
- 体素滤波 + 直通滤波预处理
- DBSCAN聚类 → 发布障碍物列表

**实验42.3**: 多目标跟踪与可视化
- 结合前两节检测结果
- 卡尔曼滤波跟踪
- 匈牙利匹配 + ID分配
- RViz可视化
