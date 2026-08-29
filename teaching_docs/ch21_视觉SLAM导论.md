# 第21章 视觉SLAM导论

## 仿真结合实例（当前仓库）：用 Gazebo 相机检查视觉 SLAM 输入

### 目标与知识点对应

视觉 SLAM 需要连续图像、相机内参和坐标变换。本仓库没有 ORB-SLAM 节点，因此使用 `robot_sim_demo` 验证 RGB 图像、`CameraInfo` 和 TF 输入的格式与时间戳。

### 运行步骤

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_sim_demo gazebo2.launch.py \
  gui:=true rviz:=false drive:=true
```

```bash
ros2 topic echo /camera/image_raw --once
ros2 topic echo /camera/camera_info --once
ros2 run tf2_ros tf2_echo base_link camera_link
```

### 观察结果

相机图像应持续更新，`CameraInfo` 应包含 320x180 的内参；TF 查询用于把相机观测关联到机器人坐标系。

### 源码与边界

- 相机桥与 Gazebo：`src/robot_sim_demo/launch/gazebo2.launch.py`
- 内参节点：`src/robot_sim_demo/robot_sim_demo/camera_info_publisher.py`
- 模型 TF：`src/robot_sim_demo/models/wheeltec_robot/model.sdf`

没有 ORB-SLAM/DSO 实现，不能由此实例宣称完成视觉里程计或地图构建。

## 学习目标
- 理解视觉SLAM的基本原理和经典框架
- 掌握ORB-SLAM2/3的特征提取与跟踪方法
- 了解DSO直接法和RGB-D SLAM的原理
- 熟悉视觉特征提取与匹配技术
- 掌握视觉与激光融合SLAM的策略

## 21.1 视觉SLAM概述

### 21.1.1 视觉SLAM vs 激光SLAM

| 特性 | 视觉SLAM | 激光SLAM |
|------|----------|----------|
| 传感器成本 | 低（百元级相机） | 高（千元-万元级） |
| 信息丰富度 | 高（纹理、颜色、语义） | 低（距离信息） |
| 定位精度 | 亚米级 | 毫米级 |
| 计算量 | 大（图像处理） | 小 |
| 低纹理环境 | 容易丢失 | 可正常工作 |
| 长走廊场景 | 可正常工作 | 容易漂移 |
| 地图用途 | 主要用于定位 | 可直接用于导航 |
| 光照变化 | 敏感 | 不敏感 |
| 动态物体 | 需要处理 | 相对容易过滤 |

### 21.1.2 相机类型

**单目相机 (Monocular)：**
- 成本最低，结构简单
- 尺度不确定性（无法直接获取深度）
- 需要运动初始化
- 适合小规模AR/VR场景

**双目相机 (Stereo)：**
- 基线提供尺度信息
- 计算量大（视差匹配）
- 深度范围受基线限制
- 代表：ZED, MYNT EYE

**RGB-D相机：**
- 直接获取深度信息
- 室内场景效果好
- 室外受环境光影响
- 代表：Intel RealSense D415/D435, Microsoft Kinect

### 21.1.3 经典视觉SLAM框架

```
传感器 (相机图像)
    ↓
┌──────────────────┐
│ 前端视觉里程计    │ ← 帧间位姿估计
│ (Visual Odometry) │
└────────┬─────────┘
         ↓
┌──────────────────┐
│ 后端非线性优化    │ ← 局部/全局BA
│ (Optimization)    │
└────────┬─────────┘
         ↓
┌──────────────────┐
│ 建图             │ ← 稀疏/稠密地图
│ (Mapping)        │
└────────┬─────────┘
         ↓
┌──────────────────┐
│ 闭环检测          │ ← 消除累积误差
│ (Loop Closing)    │
└──────────────────┘
```

**ORB-SLAM的三个并行线程：**
1. **Tracking（跟踪线程）：** 实时跟踪当前帧位姿
2. **Local Mapping（局部建图线程）：** 局部BA优化和地图点管理
3. **Loop Closing（闭环检测线程）：** 回环检测和全局优化

## 21.2 相机模型与ROS2视觉接口

### 21.2.1 针孔相机模型

**投影方程：**
```
z · [u, v, 1]^T = K · [R|t] · [X, Y, Z, 1]^T
```

**内参矩阵K：**
```
K = [fx   0   cx]
    [0   fy   cy]
    [0    0    1]
```

其中：
- fx, fy: 焦距（像素单位）
- cx, cy: 光心坐标

```python
import numpy as np

class PinholeCamera:
    """针孔相机模型"""
    def __init__(self, fx: float, fy: float, 
                 cx: float, cy: float,
                 width: int = 640, height: int = 480):
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        self.width = width
        self.height = height
        
        self.K = np.array([
            [fx, 0, cx],
            [0, fy, cy],
            [0, 0, 1]
        ])
        
    def project(self, points_3d: np.ndarray) -> np.ndarray:
        """3D点投影到2D图像平面"""
        # points_3d: (N, 3) 在世界坐标系
        points_h = np.hstack([points_3d, np.ones((len(points_3d), 1))])
        
        # 投影
        pixels = self.K @ points_h[:, :3].T
        pixels = pixels.T
        
        # 归一化
        pixels[:, 0] /= pixels[:, 2]
        pixels[:, 1] /= pixels[:, 2]
        
        return pixels[:, :2]
    
    def back_project(self, pixels: np.ndarray, depth: np.ndarray) -> np.ndarray:
        """2D像素+深度反投影到3D"""
        # 像素坐标
        u, v = pixels[:, 0], pixels[:, 1]
        z = depth
        
        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy
        
        return np.column_stack([x, y, z])
```

### 21.2.2 相机畸变

**径向畸变（Radial Distortion）：**
```
x_corrected = x * (1 + k1*r² + k2*r⁴ + k3*r⁶)
y_corrected = y * (1 + k1*r² + k2*r⁴ + k3*r⁶)
```

**切向畸变（Tangential Distortion）：**
```
x_corrected = x + (2*p1*x*y + p2*(r² + 2*x²))
y_corrected = y + (p1*(r² + 2*y²) + 2*p2*x*y)
```

### 21.2.3 ROS2相机接口

```bash
# 查看相机内参
ros2 topic echo /camera/camera_info --once

# 查看原始图像
ros2 topic echo /camera/image_raw

# 查看深度图像 (RGB-D)
ros2 topic echo /camera/depth/image_raw

# 图像显示
ros2 run image_view image_view image:=/camera/image_raw
ros2 run rqt_image_view rqt_image_view
```

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraProcessor(Node):
    """ROS2相机数据处理节点"""
    def __init__(self):
        super().__init__('camera_processor')
        
        self.bridge = CvBridge()
        
        # 彩色图像订阅
        self.image_sub = self.create_subscription(
            Image, '/camera/color/image_raw', 
            self.image_callback, 10)
        
        # 深度图像订阅    
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw',
            self.depth_callback, 10)
        
        # 相机内参订阅
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/color/camera_info',
            self.info_callback, 10)
        
        self.latest_image = None
        self.latest_depth = None
        self.camera_matrix = None
        
    def image_callback(self, msg: Image):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
    def depth_callback(self, msg: Image):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, '32FC1')
        
    def info_callback(self, msg: CameraInfo):
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.get_logger().info(f'相机内参已获取:\n{self.camera_matrix}')
        
    def get_rgbd_pointcloud(self):
        """获取RGB-D点云"""
        if self.latest_image is None or self.latest_depth is None:
            return None
            
        h, w = self.latest_depth.shape
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]
        
        points = []
        colors = []
        
        for v in range(0, h, 5):  # 步长5降采样
            for u in range(0, w, 5):
                z = self.latest_depth[v, u]
                if z <= 0 or z > 5.0:  # 有效深度范围
                    continue
                    
                x = (u - cx) * z / fx
                y = (v - cy) * z / fy
                
                points.append([x, y, z])
                colors.append(self.latest_image[v, u])
                
        return np.array(points), np.array(colors)
```

### 21.2.4 image_transport图像传输

```bash
# Raw格式（无损但带宽大）
ros2 run image_transport republish raw in:=/camera/image_raw raw out:=/image_raw

# Compressed格式（JPEG压缩）
ros2 run image_transport republish compressed in:=/camera/image_raw raw out:=/image_compressed

# Theora格式（视频流编码）
ros2 run image_transport republish theora in:=/camera/image_raw raw out:=/image_theora
```

## 21.3 ORB-SLAM

### 21.3.1 ORB特征

ORB (Oriented FAST and Rotated BRIEF) 是视觉SLAM中最常用的特征：

**FAST关键点检测：**
- 比较像素与其周围16个像素的亮度
- 若有连续N个像素亮度超过阈值，则认为是角点
- 计算速度快，适合实时SLAM

**BRIEF描述子：**
- 在关键点周围随机选取点对比较亮度
- 生成二进制字符串描述子
- 使用Hamming距离快速匹配

**ORB改进：**
- 方向计算：使用灰度质心法（旋转不变性）
- 尺度不变性：使用图像金字塔
- 旋转BRIEF：根据关键点方向旋转描述子

```python
import cv2
import numpy as np

class ORBFeatureExtractor:
    """ORB特征提取器"""
    def __init__(self, num_features: int = 1000,
                 scale_factor: float = 1.2,
                 num_levels: int = 8):
        self.orb = cv2.ORB_create(
            nfeatures=num_features,
            scaleFactor=scale_factor,
            nlevels=num_levels
        )
        
    def extract(self, image: np.ndarray) -> tuple:
        """
        提取ORB特征
        
        返回:
            keypoints: 关键点列表
            descriptors: (N, 32) 描述子
        """
        keypoints, descriptors = self.orb.detectAndCompute(image, None)
        return keypoints, descriptors
    
    def match_features(self, desc1: np.ndarray, 
                        desc2: np.ndarray,
                        ratio_threshold: float = 0.75) -> list:
        """
        特征匹配（使用FLANN或暴力匹配）
        
        参数:
            desc1: 第一帧描述子
            desc2: 第二帧描述子
            ratio_threshold: Lowe比率测试阈值
            
        返回:
            good_matches: 好的匹配对
        """
        # 暴力匹配
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
        matches = bf.knnMatch(desc1, desc2, k=2)
        
        # Lowe比率测试
        good_matches = []
        for pair in matches:
            if len(pair) == 2:
                m, n = pair
                if m.distance < ratio_threshold * n.distance:
                    good_matches.append(m)
                    
        return good_matches
    
    def visualize_matches(self, img1: np.ndarray, img2: np.ndarray,
                           kp1: list, kp2: list, 
                           matches: list) -> np.ndarray:
        """可视化特征匹配结果"""
        return cv2.drawMatches(
            img1, kp1, img2, kp2, matches, None,
            flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS
        )
```

### 21.3.2 ORB-SLAM算法流程

**跟踪线程 (Tracking Thread)：**
```
1. 预处理输入
   - 提取ORB特征
   - 如果是RGB-D，获取对应深度

2. 位姿预测 / 重定位
   - 匀速模型预测
   - 特征匹配 + EPnP求解位姿

3. 局部地图跟踪
   - 匹配更多地图点
   - 优化当前帧位姿

4. 关键帧决策
   - 判断是否插入新关键帧
   - 条件：跟踪质量、时间间隔、距离间隔
```

**建图线程 (Local Mapping Thread)：**
```
1. 插入关键帧
   - 更新共视图(Covisibility Graph)

2. 地图点管理
   - 剔除质量差的地图点
   - 三角化新的地图点

3. 局部BA优化
   - 优化当前关键帧及其邻居

4. 冗余关键帧剔除
   - 删除观测冗余的关键帧
```

**闭环检测线程 (Loop Closing Thread)：**
```
1. DBoW词袋检测
   - 使用视觉词汇进行闭环候选搜索

2. 闭环验证
   - 计算闭环帧间SE3变换

3. 闭环融合
   - 合并闭环处的地图点和关键帧

4. 本质图优化
   - 全局位姿图优化消除累积误差
```

### 21.3.3 重投影误差

视觉SLAM的核心是最小化重投影误差：

```
E = Σ || z_ij - h(T_i, P_j) ||²_Σ
```

其中：
- z_ij: 第j个地图点在第i帧图像上的观测像素坐标
- T_i: 第i帧相机位姿
- P_j: 第j个地图点的3D位置
- h(T_i, P_j): 重投影函数（3D→2D投影）

```python
import numpy as np

class BundleAdjustment:
    """光束平差法(BA)优化"""
    def __init__(self, camera_matrix: np.ndarray):
        self.K = camera_matrix
        
    def compute_reprojection_error(self, pose: np.ndarray,
                                      point_3d: np.ndarray,
                                      observation: np.ndarray) -> float:
        """
        计算单个重投影误差
        
        参数:
            pose: [rx, ry, rz, tx, ty, tz] 相机位姿
            point_3d: [X, Y, Z] 地图点位置
            observation: [u, v] 观测像素坐标
        """
        # 将地图点变换到相机坐标系
        R = self._rodrigues(pose[:3])  # 旋转向量→旋转矩阵
        t = pose[3:]
        
        P_c = R @ point_3d + t
        
        # 投影到图像平面
        u = self.K[0, 0] * P_c[0] / P_c[2] + self.K[0, 2]
        v = self.K[1, 1] * P_c[1] / P_c[2] + self.K[1, 2]
        
        # 误差
        error = np.array([u - observation[0], v - observation[1]])
        return np.linalg.norm(error)
    
    def _rodrigues(self, r: np.ndarray) -> np.ndarray:
        """罗德里格斯公式：旋转向量→旋转矩阵"""
        theta = np.linalg.norm(r)
        if theta < 1e-6:
            return np.eye(3)
        n = r / theta
        K = np.array([[0, -n[2], n[1]],
                      [n[2], 0, -n[0]],
                      [-n[1], n[0], 0]])
        R = np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * (K @ K)
        return R
```

### 21.3.4 ORB-SLAM3 ROS2配置

```yaml
# RGB-D相机配置 (D415.yaml)
Camera.type: "PinHole"
Camera.fx: 615.0
Camera.fy: 615.0
Camera.cx: 320.0
Camera.cy: 240.0
Camera.k1: 0.0
Camera.k2: 0.0
Camera.p1: 0.0
Camera.p2: 0.0
Camera.width: 640
Camera.height: 480
Camera.fps: 30

# RGB-D参数
RGBD.ThDepth: 5.0           # 深度阈值(m)
Depth.map_factor: 1000.0    # 深度缩放因子

# ORB特征参数
ORBextractor.nFeatures: 1200
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8

# 跟踪参数
Tracker.nFeaturesPerLevel: 200
Tracker.nonFeaturesPerLevel: 300

# 查看器参数
Viewer.keyFrameSize: 0.05
Viewer.keyFrameLineWidth: 1.0
Viewer.graphLineWidth: 0.9
Viewer.pointSize: 2.0
```

```bash
# 启动ORB-SLAM3 (需要编译ORB-SLAM3源码)
ros2 run orbslam3 rgbd \
  /path/to/Vocabulary/ORBvoc.txt \
  /path/to/config/D415.yaml

# 查看ORB-SLAM3输出
ros2 topic echo /orb_slam3/camera_pose
ros2 topic echo /orb_slam3/trajectory
ros2 topic echo /orb_slam3/map_points
```

## 21.4 DSO直接法

### 21.4.1 直接法 vs 特征法

| 特征法 (ORB-SLAM) | 直接法 (DSO) |
|--------------------|--------------|
| 提取特征点 | 使用原始像素亮度 |
| 构建描述子 | 直接使用光度误差 |
| 适合弱纹理 | 不适合 | 不适合 |
| 适合模糊图像 | 不适合 | 不适合 |
| 计算量 | 特征提取+匹配 | 像素采样+优化 |
| 信息利用 | 稀疏（仅特征点） | 半稠密（梯度像素） |

### 21.4.2 DSO算法原理

**核心思想：** 最小化光度误差 (Photometric Error)

```
E_photo = Σ || I_i(p') - I_j(p) ||²
```

其中 I_i(p') 是前一帧图像在 p' 处的亮度，I_j(p) 是当前帧图像在 p 处的亮度。

**关键模块：**
1. **光度标定：** 校正曝光时间、伽马校正、渐晕
2. **直接图像对齐：** 使用像素亮度差异优化位姿
3. **滑窗优化：** 维护最近关键帧窗口进行BA

```python
import numpy as np

class DirectImageAlignment:
    """直接图像对齐（DSO核心）"""
    def __init__(self, num_levels: int = 3):
        self.num_levels = num_levels
        
    def align(self, img_ref: np.ndarray, img_cur: np.ndarray,
              initial_pose: np.ndarray,
              inv_depth: np.ndarray) -> np.ndarray:
        """
        直接图像对齐
        
        参数:
            img_ref: 参考帧图像
            img_cur: 当前帧图像
            initial_pose: 初始位姿估计 [tx, ty, tz, rx, ry, rz]
            inv_depth: 逆深度值
            
        返回:
            optimized_pose: 优化后的位姿
        """
        pose = initial_pose.copy()
        
        # 构建图像金字塔
        pyramid_ref = self._build_pyramid(img_ref)
        pyramid_cur = self._build_pyramid(img_cur)
        
        # 由粗到精优化
        for level in range(self.num_levels - 1, -1, -1):
            scale = 2**level
            H, W = pyramid_ref[level].shape
            
            for iteration in range(10):
                # 计算光度残差
                residuals, J = self._compute_photometric_error(
                    pyramid_ref[level], pyramid_cur[level],
                    pose, inv_depth / scale
                )
                
                # Gauss-Newton优化
                H_gn = J.T @ J
                b = J.T @ residuals
                
                try:
                    delta = np.linalg.solve(
                        H_gn + np.eye(6) * 0.001, -b
                    )
                except np.linalg.LinAlgError:
                    break
                    
                pose = self._se3_exp(delta) @ pose
                
                if np.linalg.norm(delta) < 0.001:
                    break
                    
        return pose
    
    def _compute_photometric_error(self, img_ref: np.ndarray,
                                     img_cur: np.ndarray,
                                     pose: np.ndarray,
                                     inv_depth: float) -> tuple:
        """计算光度误差"""
        h, w = img_ref.shape
        
        # 在参考帧上采样像素（取梯度大的区域）
        grad_x = np.gradient(img_ref, axis=1)
        grad_y = np.gradient(img_ref, axis=0)
        grad_mag = np.sqrt(grad_x**2 + grad_y**2)
        
        # 选择高梯度像素
        mask = grad_mag > 10.0
        ys, xs = np.where(mask)
        
        # 随机采样
        if len(ys) > 1000:
            indices = np.random.choice(len(ys), 1000, replace=False)
            ys, xs = ys[indices], xs[indices]
            
        residuals = []
        J = []
        
        for y, x in zip(ys, xs):
            # 参考帧像素坐标→归一化坐标→3D点
            # (简化实现)
            z = 1.0 / inv_depth
            p_3d = np.array([(x - w/2) * z / 500, 
                             (y - h/2) * z / 500, z])
            
            # 变换到当前帧
            p_cur = pose[:3, :3] @ p_3d + pose[:3, 3]
            u = p_cur[0] / p_cur[2] * 500 + w/2
            v = p_cur[1] / p_cur[2] * 500 + h/2
            
            if not (0 <= u < w-1 and 0 <= v < h-1):
                continue
                
            # 亮度误差
            residual = img_ref[y, x] - img_cur[int(v), int(u)]
            residuals.append(residual)
            
        return np.array(residuals), np.array(J)
    
    def _build_pyramid(self, image: np.ndarray) -> list:
        """构建图像金字塔"""
        pyramid = [image]
        for _ in range(1, self.num_levels):
            h, w = pyramid[-1].shape
            smaller = cv2.resize(
                pyramid[-1], (w//2, h//2), 
                interpolation=cv2.INTER_LINEAR
            )
            pyramid.append(smaller)
        return pyramid
    
    def _se3_exp(self, xi: np.ndarray) -> np.ndarray:
        """SE3指数映射"""
        w = xi[:3]
        v = xi[3:]
        
        theta = np.linalg.norm(w)
        if theta < 1e-6:
            R = np.eye(3)
            t = v
        else:
            wx = np.array([[0, -w[2], w[1]],
                          [w[2], 0, -w[0]],
                          [-w[1], w[0], 0]])
            R = (np.eye(3) + np.sin(theta)/theta * wx + 
                 (1 - np.cos(theta))/theta**2 * wx @ wx)
            A = (np.eye(3) + (1-np.cos(theta))/theta**2 * wx + 
                 (theta - np.sin(theta))/theta**3 * wx @ wx)
            t = A @ v
            
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = t
        return T
```

## 21.5 RGB-D SLAM

### 21.5.1 RGB-D SLAM特点

RGB-D相机直接提供深度信息，简化了SLAM问题：

**优势：**
- 直接获取3D信息，无需三角化
- 尺度已知，无尺度漂移
- 可构建稠密地图
- 初始化简单

**挑战：**
- 深度范围有限（通常0.3-5m）
- 受环境光影响大
- 多路径干扰
- 运动模糊

```python
import numpy as np
import open3d as o3d

class RGBDSLAM:
    """简化版RGB-D SLAM"""
    def __init__(self, camera_matrix: np.ndarray):
        self.K = camera_matrix
        self.point_cloud = o3d.geometry.PointCloud()
        self.keyframes = []
        self.poses = []
        
    def process_frame(self, rgb: np.ndarray, depth: np.ndarray,
                       initial_pose: np.ndarray = None) -> np.ndarray:
        """处理RGB-D帧"""
        # 1. RGB-D生成点云
        pcd = self._rgbd_to_pointcloud(rgb, depth)
        
        # 2. 位姿估计
        if len(self.keyframes) == 0:
            # 首帧
            pose = np.eye(4)
            if initial_pose is not None:
                pose[:3, 3] = initial_pose
        else:
            # 与上一关键帧配准
            pose = self._register_pointcloud(
                pcd, self.keyframes[-1]['cloud'], initial_pose
            )
            
        self.poses.append(pose)
        
        # 3. 判断是否插入关键帧
        if self._is_keyframe(pose):
            self.keyframes.append({
                'pose': pose,
                'cloud': pcd,
                'rgb': rgb,
                'depth': depth
            })
            
            # 4. 增量式地图融合
            self._fusion_map()
            
        return pose
    
    def _rgbd_to_pointcloud(self, rgb: np.ndarray, 
                              depth: np.ndarray) -> o3d.geometry.PointCloud:
        """RGB-D转点云"""
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d.geometry.Image(rgb),
            o3d.geometry.Image(depth),
            depth_trunc=5.0,
            convert_rgb_to_intensity=False
        )
        
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
            rgbd, self.K
        )
        
        return pcd.voxel_down_sample(voxel_size=0.01)
    
    def _register_pointcloud(self, source: o3d.geometry.PointCloud,
                               target: o3d.geometry.PointCloud,
                               init_pose: np.ndarray = None) -> np.ndarray:
        """点云配准"""
        if init_pose is None:
            init_pose = np.eye(4)
            
        # ICP配准
        reg_p2p = o3d.pipelines.registration.registration_icp(
            source, target, 0.1, init_pose,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(
                max_iteration=50)
        )
        
        return reg_p2p.transformation
    
    def _is_keyframe(self, pose: np.ndarray) -> bool:
        """判断是否为关键帧"""
        if len(self.poses) < 2:
            return True
            
        # 检查与上一关键帧的位移和旋转
        last_pose = self.keyframes[-1]['pose']
        delta = np.linalg.inv(last_pose) @ pose
        
        translation = np.linalg.norm(delta[:3, 3])
        rotation = np.arccos(
            (np.trace(delta[:3, :3]) - 1) / 2
        )
        
        return translation > 0.1 or abs(rotation) > 0.3
    
    def _fusion_map(self):
        """融合地图（TSDF融合）"""
        # 使用所有关键帧进行TSDF融合
        volume = o3d.pipelines.integration.ScalableTSDFVolume(
            voxel_length=0.01,
            sdf_trunc=0.03,
            color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8
        )
        
        for kf in self.keyframes:
            rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
                o3d.geometry.Image(kf['rgb']),
                o3d.geometry.Image(kf['depth']),
                depth_trunc=4.0,
                convert_rgb_to_intensity=False
            )
            volume.integrate(rgbd, self.K, np.linalg.inv(kf['pose']))
            
        self.point_cloud = volume.extract_point_cloud()
```

## 21.6 视觉与激光融合SLAM

### 21.6.1 融合策略

视觉与激光SLAM各有优势，融合可以取长补短：

| 融合方式 | 描述 | 代表系统 |
|---------|------|---------|
| 松耦合 | 独立运行后融合结果 | LIC-Fusion |
| 紧耦合 | 统一优化视觉+激光因子 | LVI-SAM, R3LIVE |
| 视觉辅助激光 | 视觉提供回环检测 | Cartographer+视觉 |
| 激光辅助视觉 | 激光提供深度 | RGB-D+激光扩展 |

**融合框架示意：**
```
┌────────────────────────────────────────────┐
│              因子图优化                      │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐ │
│  │ 预积分IMU │  │ 激光因子  │  │ 视觉因子  │ │
│  │ 约束      │  │ 扫描匹配  │  │ 重投影    │ │
│  └──────────┘  └──────────┘  └──────────┘ │
└────────────────────────────────────────────┘
```

### 21.6.2 视觉-惯性融合 (VIO)

VINS-Mono是经典的视觉惯性融合算法：

```python
import numpy as np

class VINSMono:
    """视觉惯性导航系统(VINS)简化实现"""
    def __init__(self):
        self.imu_buffer = []      # IMU数据缓冲
        self.image_buffer = []    # 图像数据缓冲
        self.keyframe_poses = []  # 关键帧位姿
        self.landmarks = {}       # 地图点
        
        # IMU参数
        self.gyro_noise = 0.01
        self.acc_noise = 0.01
        self.gyro_bias_noise = 0.0001
        self.acc_bias_noise = 0.0001
        
        # 状态
        self.state = {
            'position': np.zeros(3),
            'velocity': np.zeros(3),
            'orientation': np.eye(3),
            'gyro_bias': np.zeros(3),
            'acc_bias': np.zeros(3)
        }
        
    def process_imu(self, acc: np.ndarray, gyro: np.ndarray, dt: float):
        """IMU预积分"""
        # 去除偏置
        acc_corrected = acc - self.state['acc_bias']
        gyro_corrected = gyro - self.state['gyro_bias']
        
        # 状态传播
        self.state['position'] += (self.state['velocity'] * dt + 
                                   0.5 * acc_corrected * dt**2)
        self.state['velocity'] += acc_corrected * dt
        
        # 姿态更新
        omega = np.array([[0, -gyro_corrected[2], gyro_corrected[1]],
                          [gyro_corrected[2], 0, -gyro_corrected[0]],
                          [-gyro_corrected[1], gyro_corrected[0], 0]])
        self.state['orientation'] = (self.state['orientation'] @ 
                                     (np.eye(3) + omega * dt))
        
        # 保存到缓冲
        self.imu_buffer.append({
            'acc': acc, 'gyro': gyro, 'dt': dt,
            'state': self.state.copy()
        })
        
    def process_image(self, image: np.ndarray, timestamp: float):
        """处理图像帧"""
        # 特征提取和匹配
        # ...
        
        # 视觉-惯性对齐
        # ...
        
        # 滑窗优化
        # ...
```

### 21.6.3 视觉-激光融合实践

```bash
# LVI-SAM (需要编译安装)
ros2 launch lvi_sam run.launch.py

# 数据融合可视化
rviz2
# 添加: PointCloud2 (/lvi_sam/mapping/registered_cloud)
# 添加: Path (/lvi_sam/odometry/odometry)
# 添加: Image (/camera/image_raw)
```

### 21.6.4 工程建议

```python
def suggest_fusion_strategy(
    environment: str,
    has_lidar: bool,
    has_camera: bool,
    has_imu: bool,
    budget: str
) -> str:
    """推荐融合策略"""
    if budget == 'low':
        if has_camera and has_imu:
            return 'VINS-Mono (仅视觉+IMU)'
        else:
            return 'ORB-SLAM2 (仅视觉)'
            
    elif budget == 'medium':
        if has_lidar and has_camera:
            return 'LVI-SAM / Cartographer+视觉回环'
        else:
            return 'Cartographer (激光+IMU)'
            
    elif budget == 'high':
        if has_lidar and has_camera and has_imu:
            return 'R3LIVE / LVI-SAM (全传感器融合)'
        else:
            return 'FAST-LIO2 (激光+IMU)'
            
    return '推荐使用激光SLAM建图 + 视觉SLAM提供语义信息'
```

## 课后练习

1. **原理题:** 比较特征法视觉SLAM和直接法视觉SLAM的异同，分析各自优缺点和适用场景。

2. **编程题:** 使用OpenCV提取ORB特征并实现特征匹配，要求包含Lowe比率测试剔除误匹配。

3. **分析题:** 分析ORB-SLAM三个并行线程（跟踪、建图、闭环）的协作机制，说明各自的作用和交互方式。

4. **配置题:** 配置ORB-SLAM3的ROS2接口，使用RealSense D415相机进行实时RGB-D SLAM建图。

5. **操作题:** 录制视觉SLAM的rosbag数据（包含图像和IMU话题），使用EVO工具评估SLAM轨迹精度。

6. **设计题:** 设计一个视觉+激光+IMU多传感器融合SLAM方案，用于大型室内停车场（存在光照变化和重复纹理），包括传感器配置、融合策略和地图表示方法。
