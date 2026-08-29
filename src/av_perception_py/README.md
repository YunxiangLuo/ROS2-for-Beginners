# av_perception_py — 感知节点

相机目标检测(YOLO/颜色兜底)、LiDAR 障碍物聚类(DBSCAN)与相机-LiDAR 前融合。

## 目录结构

```
av_perception_py/
├── setup.py / package.xml
├── config/perception_params.yaml
├── resource/av_perception_py
├── av_perception_py/
│   ├── object_detector.py    # YOLO 检测, 无 ultralytics 时退化为 HSV 颜色检测
│   ├── lidar_detector.py     # 点云解析 + 体素滤波 + DBSCAN 聚类
│   └── fusion_node.py        # 3D->2D 投影匹配融合
└── test/
    ├── test_lidar_detector.py
    └── test_fusion_node.py
```

## 安装与编译

```bash

pip install numpy opencv-python            # 基础依赖

pip install ultralytics                    # 可选: YOLO 检测

cd <工作空间根目录>

colcon build --packages-select av_carla_interfaces av_perception_py

source install/setup.bash
```

## 运行方法

```bash
ros2 run av_perception_py object_detector --ros-args -p confidence_threshold:=0.5
ros2 run av_perception_py lidar_detector  --ros-args -p cluster_tolerance:=0.5
ros2 run av_perception_py fusion_node
```

话题: 订阅 `/carla/ego_vehicle/front_rgb/image`、`/carla/ego_vehicle/lidar_top/pointcloud`、

`/detections`、`/clusters`; 发布 `/detections`(Detection2DArray)、

`/lidar_obstacle_markers`、`/perception_objects`(PerceptionObjectArray)。

## 测试方法

```bash

cd src/av_perception_py

python -m pytest test -q
```

## 运行结果

```text
$ cd src/av_perception_py && python -m pytest test -q
................                                                         [100%]
16 passed in 0.09s
```

覆盖: 点云字段 offset/point_step 解析、体素降采样、DBSCAN 边界点吸收、

双簇分离、相机投影(中心/偏轴/出界/相机后方)。

> 说明: 本机(Windows)未安装 ROS2/CARLA, 无法截取仿真运行画面,
> 运行结果以**真实终端输出**代替截图; 全部输出均可按上述命令复现。

## 本次修复记录

1. `fusion_node.py` 向 `PerceptionObject` 写入不存在的 `class_id` 字段
   (运行时 AttributeError) → 改用 `id` 字段;
2. `project_to_image` 未拒绝相机后方(z<=0)的点, 车后障碍物会被镜像投影到
   图像内造成误匹配 → 增加 z<=0 早退;
3. `pointcloud2_to_numpy` 忽略字段 `offset` 且不使用 `point_step`, 含 padding
   的真实点云会解析错位 → 改为按 offset 构造 structured dtype, 优先使用

   `msg.point_step` 作为步长;
4. `_dbscan_clustering` 将邻域不足的点提前标记 visited, 边界点被永久丢弃
   (少聚/漏检) → 重写为标准标签式 DBSCAN;
5. 新增 16 个单元测试锁定上述行为(含边界点吸收回归测试)。
