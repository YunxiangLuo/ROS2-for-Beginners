# 第12章 Hector SLAM

> **课程**：ROS2 Python 编程  
> **章节**：第12章  
> **课时**：2 课时（90 分钟）  
> **教学方式**：讲授 + 演示  

---

## 学习目标

本章学习目标包括：理解Hector-SLAM的基本原理和算法架构；掌握基于优化的激光SLAM方法；理解双栅格地图和多分辨率匹配策略；熟悉Gauss-Newton地图匹配的数学推导；能够分析Hector-SLAM的优缺点和适用场景。

## 12.1 Hector-SLAM算法概述

### 12.1.1 算法背景

Hector-SLAM是一种基于优化的2D SLAM方法，由德国达姆施塔特理工大学提出。与传统的基于滤波的SLAM方法不同，Hector-SLAM采用扫描-地图匹配（Scan-to-Map）策略，不需要里程计信息。

**核心思想：** 最小化当前激光扫描与已有栅格地图之间的灰度差异，通过Gauss-Newton迭代优化求解机器人位姿。

**主要特点：** Hector-SLAM不依赖里程计，可在手持设备或无轮式编码器的机器人上运行；使用多分辨率栅格地图加速匹配；基于双线性插值的连续地图表示。

### 12.1.2 Hector-SLAM的架构

Hector-SLAM的系统架构分为三个主要模块：

```
激光扫描数据输入
    ↓
┌─────────────────────────┐
│    扫描-地图匹配        │
│  (Gauss-Newton优化)     │
│  输入: 当前扫描+已有地图 │
│  输出: 优化后位姿       │
└─────────┬───────────────┘
          ↓
┌─────────────────────────┐
│    地图更新             │
│  (双栅格分辨率)          │
│  输入: 优化后的位姿+扫描 │
│  输出: 更新后的栅格地图  │
└─────────┬───────────────┘
          ↓
┌─────────────────────────┐
│    地图发布             │
│  /map (OccupancyGrid)   │
│  /slam_out_pose (Pose)  │
└─────────────────────────┘
```

### 12.1.3 Hector-SLAM的优缺点

**优点：** 无需里程计，适用性广泛；计算效率高，可实时运行；在结构化环境中精度高；实现相对简单。

**缺点：** 依赖高精度激光雷达，低速率的激光会导致漂移；速度过快会导致建图漂移；回环检测能力弱；对初值敏感，重定位能力差。

| 对比维度 | Hector-SLAM | gmapping | Cartographer |
|---------|-------------|----------|-------------|
| 里程计依赖 | 不需要 | 需要 | 可选 |
| 回环检测 | 不支持 | 不支持 | 支持 |
| 建图质量 | 依赖激光频率 | 依赖里程计精度 | 高（图优化） |
| 计算效率 | 高 | 中 | 中 |
| 适用场景 | 手持/无里程计 | 有里程计 | 复杂大场景 |

## 12.2 扫描-地图匹配算法

### 12.2.1 目标函数定义

Hector-SLAM的扫描-地图匹配通过最小化以下误差函数来优化位姿：

```
T* = argmin Σ [1 - M(S_i(T))]²
```

其中，T = (x, y, θ) 为待优化的机器人位姿；S_i(T) 为第i个激光点在变换T下的地图坐标；M(S_i(T)) 为地图在坐标S_i(T)处的占据概率值（0=空闲，1=占据）。

**目标函数的直观理解：** 当激光点落在占据区域时M≈1、误差≈0；当激光点落在空闲区域时M≈0、误差≈1；优化目标是所有激光点都匹配到地图的占据区域。

### 12.2.2 Gauss-Newton优化求解

由于目标函数是非线性的，使用Gauss-Newton法迭代求解：

```
ΔT = H⁻¹ · ∇M · Σ (∂S_i/∂T)ᵀ · [1 - M(S_i(T))]
```

其中，∇M 为地图M在S_i处的梯度；∂S_i/∂T 为激光点对位姿参数的雅可比；H 为Hessian矩阵近似：H = Σ J_iᵀ · J_i。

```python
import numpy as np

class HectorScanMatcher:
    """Hector-SLAM扫描匹配器实现"""
    def __init__(self, max_iterations: int = 15, 
                 convergence_delta: float = 1e-6):
        self.max_iterations = max_iterations
        self.convergence_delta = convergence_delta
        
    def compute_coords(self, scan_points: np.ndarray, pose: np.ndarray) -> np.ndarray:
        """
        将激光点转换到地图坐标系
        
        参数:
            scan_points: (N, 2) 激光点在激光坐标系下的坐标
            pose: (3,) 机器人位姿 (x, y, theta)
            
        返回:
            world_coords: (N, 2) 地图坐标系下的坐标
        """
        theta = pose[2]
        c, s = np.cos(theta), np.sin(theta)
        R = np.array([[c, -s], [s, c]])
        
        # 转换为世界坐标
        world_coords = scan_points @ R.T + pose[:2]
        
        # 转换为栅格坐标
        grid_coords = world_coords / self.map_resolution
        return grid_coords
    
    def match(self, scan_points: np.ndarray, map_data: np.ndarray,
              map_resolution: float, initial_pose: np.ndarray) -> np.ndarray:
        """
        执行扫描-地图匹配
        
        参数:
            scan_points: (N, 2) 激光点坐标
            map_data: (H, W) 栅格地图占据概率
            map_resolution: 地图分辨率 (m/像素)
            initial_pose: 初始位姿估计
            
        返回:
            optimized_pose: 优化后的位姿
        """
        self.map_resolution = map_resolution
        pose = initial_pose.copy()
        
        # 预计算地图梯度
        grad_x = np.gradient(map_data, axis=1) / map_resolution
        grad_y = np.gradient(map_data, axis=0) / map_resolution
        
        for iteration in range(self.max_iterations):
            # 计算变换后的激光点坐标
            theta = pose[2]
            c, s = np.cos(theta), np.sin(theta)
            R = np.array([[c, -s], [s, c]])
            R_theta = np.array([[-s, -c], [c, -s]])  # dR/dθ
            
            world_coords = scan_points @ R.T + pose[:2]
            grid_coords = world_coords / map_resolution
            
            J = np.zeros((len(scan_points), 3))
            residuals = np.zeros(len(scan_points))
            
            for i, (wc, q) in enumerate(zip(world_coords, scan_points)):
                # 双线性插值获取地图值和梯度
                M, dM_dx, dM_dy = self.bilinear_interpolate_with_gradient(
                    map_data, grid_coords[i], grad_x, grad_y
                )
                
                residuals[i] = 1.0 - M
                
                # 雅可比矩阵
                J[i, 0] = -dM_dx
                J[i, 1] = -dM_dy
                J[i, 2] = -(dM_dx * (R_theta[0, 0] * q[0] + R_theta[0, 1] * q[1]) +
                            dM_dy * (R_theta[1, 0] * q[0] + R_theta[1, 1] * q[1]))
            
            # 构建正规方程并求解
            H = J.T @ J
            b = J.T @ residuals
            
            try:
                delta = np.linalg.solve(H + np.eye(3) * 1e-6, -b)
            except np.linalg.LinAlgError:
                break
            
            # 更新位姿
            pose[:2] += delta[:2]
            pose[2] += delta[2]
            
            # 收敛检查
            if np.linalg.norm(delta) < self.convergence_delta:
                break
                
        return pose
    
    def bilinear_interpolate_with_gradient(self, map_data: np.ndarray,
                                             grid_coord: np.ndarray,
                                             grad_x: np.ndarray,
                                             grad_y: np.ndarray) -> tuple:
        """
        双线性插值：同时返回地图值和梯度
        """
        x, y = grid_coord
        x0, y0 = int(np.floor(x)), int(np.floor(y))
        x1, y1 = x0 + 1, y0 + 1
        
        # 边界检查
        H, W = map_data.shape
        if not (0 <= x0 < W-1 and 0 <= y0 < H-1):
            return 0.5, 0.0, 0.0
            
        dx = x - x0
        dy = y - y0
        
        # 四角的地图值
        M00 = map_data[y0, x0]
        M10 = map_data[y0, x1]
        M01 = map_data[y1, x0]
        M11 = map_data[y1, x1]
        
        # 双线性插值地图值
        M = (M00 * (1-dx) * (1-dy) +
             M10 * dx * (1-dy) +
             M01 * (1-dx) * dy +
             M11 * dx * dy)
        
        # 双线性插值梯度
        dM_dx = (grad_x[y0, x0] * (1-dx) * (1-dy) +
                 grad_x[y0, x1] * dx * (1-dy) +
                 grad_x[y1, x0] * (1-dx) * dy +
                 grad_x[y1, x1] * dx * dy)
        
        dM_dy = (grad_y[y0, x0] * (1-dx) * (1-dy) +
                 grad_y[y0, x1] * dx * (1-dy) +
                 grad_y[y1, x0] * (1-dx) * dy +
                 grad_y[y1, x1] * dx * dy)
        
        return M, dM_dx, dM_dy
```

### 12.2.3 双线性地图插值

Hector-SLAM的一个关键创新是使用**双线性插值**将离散栅格地图转换为连续可微函数：

```
M(P) = Σ w_ij · M_ij
```

其中 w_ij 为双线性插值权重，使得地图函数可导，支持Gauss-Newton优化。

**双线性插值图示：**
```
M00 ─── dx ─── M10
  │             │
 dy    M(P)     dy
  │             │
M01 ─── dx ─── M11

M(P) = M00·(1-dx)·(1-dy) + M10·dx·(1-dy) + M01·(1-dx)·dy + M11·dx·dy
```

### 12.2.4 扫描匹配性能分析

```python
def analyze_scan_matching(map_data: np.ndarray, 
                           scan_points: np.ndarray):
    """分析扫描匹配的性能特征"""
    matcher = HectorScanMatcher()
    
    # 测试不同初始偏移下的收敛行为
    test_offsets = [
        np.array([0.0, 0.0, 0.0]),
        np.array([0.1, 0.1, 0.05]),
        np.array([0.5, 0.5, 0.2]),
        np.array([1.0, 1.0, 0.5]),
    ]
    
    results = []
    for offset in test_offsets:
        # 初始化位姿（偏移）
        initial_pose = np.array([5.0, 5.0, 0.0]) + offset
        optimized = matcher.match(
            scan_points, map_data, 0.05, initial_pose
        )
        
        error = np.linalg.norm(optimized - np.array([5.0, 5.0, 0.0]))
        results.append({
            'offset': offset,
            'final_error': error,
            'converged': error < 0.1
        })
        
    return results
```

## 12.3 双栅格地图与多分辨率匹配

### 12.3.1 多分辨率地图策略

Hector-SLAM使用两级（或多级）分辨率栅格地图来提高匹配效率和鲁棒性：

```
粗分辨率地图 (0.2m/像素):
┌──────────────────────┐
│  快速粗匹配           │
│  获取初始位姿估计      │
│  收敛域大，精度低     │
└──────────────────────┘
         ↓ (结果作为初值)
┌──────────────────────┐
│  高分辨率地图 (0.05m/像素) │
│  精确匹配             │
│  收敛域小，精度高     │
└──────────────────────┘
         ↓
    最终精确位姿
```

```python
class MultiResolutionMap:
    """多分辨率栅格地图管理"""
    def __init__(self, resolutions: list = None):
        if resolutions is None:
            resolutions = [0.20, 0.10, 0.05]
        self.resolutions = sorted(resolutions, reverse=True)
        self.maps = {r: None for r in self.resolutions}
        
    def build_pyramid(self, base_map: np.ndarray):
        """构建多分辨率地图金字塔"""
        base_resolution = self.resolutions[-1]
        
        for i, resolution in enumerate(self.resolutions):
            if resolution == base_resolution:
                self.maps[resolution] = base_map
            else:
                # 降采样
                scale = int(resolution / base_resolution)
                h, w = base_map.shape
                h_new, w_new = h // scale, w // scale
                downsampled = np.zeros((h_new, w_new))
                
                for r in range(h_new):
                    for c in range(w_new):
                        patch = base_map[r*scale:(r+1)*scale,
                                         c*scale:(c+1)*scale]
                        downsampled[r, c] = np.mean(patch)
                        
                self.maps[resolution] = downsampled
                
    def get_map(self, resolution: float) -> np.ndarray:
        """获取指定分辨率的地图"""
        if resolution in self.maps:
            return self.maps[resolution]
        # 找到最接近的分辨率
        closest = min(self.resolutions, 
                       key=lambda r: abs(r - resolution))
        return self.maps[closest]
```

### 12.3.2 由粗到精的匹配策略

```python
class CoarseToFineMatcher:
    """由粗到精的多分辨率扫描匹配"""
    def __init__(self):
        self.matcher = HectorScanMatcher()
        self.resolutions = [0.20, 0.10, 0.05]
        
    def match(self, scan_points: np.ndarray, 
              map_pyramid: dict,
              initial_pose: np.ndarray) -> np.ndarray:
        """
        由粗到精的多分辨率匹配
        """
        pose = initial_pose.copy()
        
        for resolution in self.resolutions:
            map_data = map_pyramid[resolution]
            
            # 在当前分辨率上进行匹配
            pose = self.matcher.match(
                scan_points, map_data, resolution, pose
            )
            
            print(f"分辨率 {resolution:.2f}m: 位姿=({pose[0]:.3f}, "
                  f"{pose[1]:.3f}, {np.degrees(pose[2]):.2f}°)")
            
        return pose
```

### 12.3.3 地图更新策略

Hector-SLAM的地图更新是一个增量过程，每当获得新位姿后，就将激光扫描数据融合到地图中：

```python
import numpy as np

class OccupancyGridMap:
    """栅格占据地图"""
    def __init__(self, width: float, height: float, resolution: float):
        self.resolution = resolution
        self.width_pixels = int(width / resolution)
        self.height_pixels = int(height / resolution)
        self.log_odds = np.zeros((self.height_pixels, self.width_pixels))
        self.origin_x = -width / 2
        self.origin_y = -height / 2
        
    def world_to_grid(self, x: float, y: float) -> tuple:
        """世界坐标转栅格坐标"""
        gx = int((x - self.origin_x) / self.resolution)
        gy = int((y - self.origin_y) / self.resolution)
        return gx, gy
        
    def grid_to_world(self, gx: int, gy: int) -> tuple:
        """栅格坐标转世界坐标"""
        x = gx * self.resolution + self.origin_x
        y = gy * self.resolution + self.origin_y
        return x, y
        
    def update_with_scan(self, scan_points: np.ndarray, 
                          robot_pose: np.ndarray,
                          laser_pose: np.ndarray = None):
        """
        使用激光扫描更新占据地图
        
        参数:
            scan_points: (N, 2) 激光点在激光坐标系下的坐标
            robot_pose: (3,) 机器人位姿
            laser_pose: (3,) 激光雷达相对机器人位姿
        """
        # 激光坐标系到世界坐标的变换
        theta = robot_pose[2]
        c, s = np.cos(theta), np.sin(theta)
        R = np.array([[c, -s], [s, c]])
        
        world_robot = robot_pose[:2]
        
        for point in scan_points:
            # 激光点在世界坐标系下的坐标
            world_point = R @ point + world_robot
            
            # 栅格坐标
            gx, gy = self.world_to_grid(*world_point)
            
            # 边界检查
            if not (0 <= gx < self.width_pixels and 
                    0 <= gy < self.height_pixels):
                continue
                
            # Bresenham线段扫描：更新激光点到机器人之间的栅格
            rx, ry = self.world_to_grid(*world_robot)
            cells = self.bresenham_line(rx, ry, gx, gy)
            
            # 线上各栅格（除终点外）更新为空闲
            for cx, cy in cells[:-1]:
                if 0 <= cx < self.width_pixels and 0 <= cy < self.height_pixels:
                    self.log_odds[cy, cx] -= 0.3  # 空闲更新
                    
            # 终点更新为占据
            if cells:
                cx, cy = cells[-1]
                if 0 <= cx < self.width_pixels and 0 <= cy < self.height_pixels:
                    self.log_odds[cy, cx] += 1.0  # 占据更新
                    
        # 限制log-odds范围
        np.clip(self.log_odds, -5.0, 5.0, out=self.log_odds)
        
    def bresenham_line(self, x0: int, y0: int, 
                        x1: int, y1: int) -> list:
        """Bresenham直线算法"""
        points = []
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        
        while True:
            points.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x0 += sx
            if e2 <= dx:
                err += dx
                y0 += sy
                
        return points
    
    def get_probability_map(self) -> np.ndarray:
        """获取占据概率地图 (0~1)"""
        prob = 1 - 1 / (1 + np.exp(self.log_odds))
        return prob
    
    def get_occupancy_grid(self) -> np.ndarray:
        """获取占据栅格地图 (ROS OccupancyGrid格式: 0-100)"""
        prob = self.get_probability_map()
        occ = (prob * 100).astype(np.int8)
        # 阈值化：<0.2空闲，>0.8占据，其余未知
        occ[prob < 0.2] = 0
        occ[prob > 0.8] = 100
        occ[(prob >= 0.2) & (prob <= 0.8)] = -1
        return occ
```

### 12.3.4 官方要点——多分辨率匹配与双线性插值的工程价值

原论文（Kohlbrecher 等，IROS 2011，官方仓库 README 引用）给出了两个被后续系统广泛借鉴的设计：用双线性插值把占据栅格变成连续可微的地图函数（本章 12.2.3），以及用多分辨率地图做由粗到精匹配（本章 12.3）。第二点与 Cartographer 文档中的相关性粗匹配异曲同工：先在大收敛域上把初值拉进正确「盆地」，再在细分辨率上精化——理解了 12.3.1 的金字塔策略，就同时理解了工业级 SLAM 里「粗配准+精配准」的通用套路。

## 12.4 Hector-SLAM的ROS2实现

### 12.4.1 ROS2节点设计

在ROS2中，Hector-SLAM可以作为独立节点运行：

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros
import numpy as np

class HectorSLAMNode(Node):
    """Hector-SLAM ROS2节点"""
    def __init__(self):
        super().__init__('hector_slam')
        
        # 参数声明
        self.declare_parameter('map_resolution', 0.05)
        self.declare_parameter('map_size', 20.0)
        self.declare_parameter('max_iterations', 15)
        self.declare_parameter('update_factor_free', 0.4)
        self.declare_parameter('update_factor_occupied', 0.9)
        
        map_resolution = self.get_parameter('map_resolution').value
        map_size = self.get_parameter('map_size').value
        
        # 地图初始化
        self.map = OccupancyGridMap(map_size, map_size, map_resolution)
        
        # 扫描匹配器
        self.matcher = HectorScanMatcher(
            max_iterations=self.get_parameter('max_iterations').value
        )
        
        # 机器人位姿
        self.pose = np.zeros(3)
        self.last_scan = None
        self.first_scan = True
        
        # 发布器
        self.map_pub = self.create_publisher(
            OccupancyGrid, '/map', 10)
        self.pose_pub = self.create_publisher(
            PoseStamped, '/slam_out_pose', 10)
        
        # 订阅器
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        
        # TF广播
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # 定时器
        self.timer = self.create_timer(1.0, self.publish_map)
        
        self.get_logger().info('Hector-SLAM节点已启动')
        
    def scan_callback(self, msg: LaserScan):
        """处理激光扫描数据"""
        # 将LaserScan转换为点云
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        
        valid = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < msg.range_max)
        x = ranges[valid] * np.cos(angles[valid])
        y = ranges[valid] * np.sin(angles[valid])
        scan_points = np.column_stack((x, y))
        
        if self.first_scan:
            # 第一帧：仅更新地图，不匹配
            self.map.update_with_scan(scan_points, self.pose)
            self.first_scan = False
            self.last_scan = scan_points
            self.get_logger().info('首帧扫描已处理')
            return
            
        if len(scan_points) < 10:
            return
            
        # 执行扫描-地图匹配
        map_data = self.map.get_probability_map()
        try:
            new_pose = self.matcher.match(
                scan_points, map_data, self.map.resolution, self.pose
            )
            self.pose = new_pose
        except Exception as e:
            self.get_logger().warn(f'扫描匹配失败: {e}')
            return
            
        # 更新地图
        self.map.update_with_scan(scan_points, self.pose)
        
        # 发布位姿
        self.publish_pose()
        
        # 广播TF变换
        self.broadcast_tf()
        
        self.last_scan = scan_points
        
    def publish_pose(self):
        """发布SLAM估计的位姿"""
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = self.pose[0]
        msg.pose.position.y = self.pose[1]
        
        qz = np.sin(self.pose[2] / 2.0)
        qw = np.cos(self.pose[2] / 2.0)
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        
        self.pose_pub.publish(msg)
        
    def broadcast_tf(self):
        """广播 map→odom 变换"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = self.pose[0]
        t.transform.translation.y = self.pose[1]
        qz = np.sin(self.pose[2] / 2.0)
        qw = np.cos(self.pose[2] / 2.0)
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)
        
    def publish_map(self):
        """发布占据栅格地图"""
        grid = self.map.get_occupancy_grid()
        
        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = self.map.resolution
        msg.info.width = self.map.width_pixels
        msg.info.height = self.map.height_pixels
        msg.info.origin.position.x = self.map.origin_x
        msg.info.origin.position.y = self.map.origin_y
        msg.data = grid.flatten().tolist()
        
        self.map_pub.publish(msg)
```

### 12.4.2 启动配置

```bash
# Hector-SLAM 在 Jazzy 下通常需从源码或兼容分支构建，先检查是否有二进制包
apt-cache policy ros-jazzy-hector-slam
# 若已安装兼容的 hector_slam 包，再运行：
# sudo apt install ros-jazzy-hector-slam

# 启动Hector-SLAM
ros2 launch hector_slam hector_slam.launch.py

# 或者在自定义launch文件中配置
```

```python
# hector_slam_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hector_slam',
            executable='hector_mapping',
            name='hector_mapping',
            output='screen',
            parameters=[{
                'map_resolution': 0.05,
                'map_size': 20.0,
                'map_start_x': 0.5,
                'map_start_y': 0.5,
                'base_frame': 'base_footprint',
                'odom_frame': 'odom',
                'map_frame': 'map',
                'scan_topic': '/scan',
                'use_tf_scan_transformation': True,
                'pub_map_odom_transform': True,
                'pub_map_scanmatch_transform': False,
            }]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', 'config/hector_slam.rviz']
        ),
    ])
```

### 12.4.3 参数调优指南

Hector-SLAM的关键参数及其调优建议：

| 参数 | 默认值 | 说明 | 调优建议 |
|------|--------|------|---------|
| map_resolution | 0.05 | 地图分辨率(m) | 大场景增大到0.1，精细场景减小到0.025 |
| map_size | 20.0 | 地图尺寸(m) | 根据环境大小调整 |
| max_iterations | 15 | 优化最大迭代次数 | 激光频率高可减少 |
| update_factor_free | 0.4 | 空闲栅格更新因子 | 减小提高平滑度 |
| update_factor_occupied | 0.9 | 占据栅格更新因子 | 增大可加快收敛 |

### 12.4.4 官方要点——hector_slam 官方包：节点构成与 frame 约定

hector_slam 由德国达姆施塔特理工大学（TU Darmstadt）团队以 `tu-darmstadt-ros-pkg` 生态发布，其 ROS 1 Wiki 页面至今仍是事实上的官方教程。核心包 `hector_mapping` 提供本章 12.4 所示的节点接口：订阅 `scan_topic`（默认 `/scan`），发布 `/map` 与 `/slam_out_pose`。frame 约定与本章 12.4.2 启动参数一一对应：`base_frame`（如 `base_footprint`）、`odom_frame` 与 `map_frame` 三个参数决定 TF 树的挂接方式；`pub_map_odom_transform` 打开时由建图节点直接广播 map→odom（本章 `broadcast_tf` 的行为），关闭时则需要外部定位节点补齐该变换。

在 ROS 2 发行版中，hector_slam 长期停留在社区移植或源码构建阶段，官方 Wiki 也没有 ROS 2 专属教程。因此本章 12.4.2 先用 `apt-cache policy` 检查二进制包存在性的做法，正是遵循官方文档对可用性的提示；若不可用，社区普遍建议改用 slam_toolbox 或 Cartographer，它们覆盖同样的扫描-地图匹配思想且维护活跃。

### 12.4.5 官方要点——参数调优与建图质量评估

官方 Wiki 的 hector_mapping 参数表对应本章 12.4.3：`map_resolution`（0.025~0.1 m 之间权衡内存与精度）、`map_size`（按环境外扩）、`update_factor_free/occupied`（对数几率增减幅度，本章 12.3.3 中 0.3/1.0 的演示值即其来源）、`max_iterations`（高帧率雷达可适当减小以控延迟）。调参纪律与第 10 章扩展一致：先用 `ros2 bag record` 录制 `/scan` 与 `/tf`，离线重放试参，每次只改一个变量。

质量评估可复用 ROS 2 官方工具链：建图完成后用 `nav2_map_server` 的 `map_saver_cli` 保存（本章 12.5.1），再与已知平面图对比占据边界偏移；对漂移的直观检查是把 `map→odom` TF 在 RViz 中长期观察其缓慢累积的旋转量。建议读者按本章练习第 5 题在仿真中系统复现这些影响。

## 12.5 仿真与实践

### 12.5.1 Gazebo仿真建图

```bash
# 1. 启动Gazebo仿真环境
ros2 launch robot_sim_demo gazebo2.launch.py drive:=false

# 2. 启动Hector-SLAM
ros2 launch hector_slam hector_slam.launch.py

# 3. 控制机器人移动
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 4. 保存地图
ros2 run nav2_map_server map_saver_cli -f hector_map
```

### 12.5.2 在已知地图上定位

```python
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np

class HectorLocalization(Node):
    """使用Hector-SLAM在已知地图上定位"""
    def __init__(self):
        super().__init__('hector_localization')
        
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
            
        self.known_map = None
        self.map_resolution = None
        self.map_origin = None
        
        self.localization_mode = True
        
    def map_callback(self, msg: OccupancyGrid):
        """加载已知地图"""
        if self.known_map is not None:
            return
            
        self.map_resolution = msg.info.resolution
        self.map_origin = msg.info.origin
        width = msg.info.width
        height = msg.info.height
        
        # 转换回概率地图
        data = np.array(msg.data).reshape(height, width)
        prob = data.astype(float) / 100.0
        prob[data < 0] = 0.5  # 未知区域
        
        self.known_map = prob
        self.get_logger().info(f'已知地图已加载: {width}x{height}')
        
    def scan_callback(self, msg):
        if self.known_map is None:
            return
            
        # 在已知地图上定位
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        valid = np.isfinite(ranges)
        scan_points = np.column_stack(
            (ranges[valid] * np.cos(angles[valid]),
             ranges[valid] * np.sin(angles[valid]))
        )
        
        matcher = HectorScanMatcher()
        pose = matcher.match(
            scan_points, self.known_map, self.map_resolution, 
            self.current_pose
        )
        self.current_pose = pose
```

### 12.5.3 常见问题与解决方案

**问题1：建图漂移**：原因是机器人移动过快或激光帧率不足，解决方法是降低移动速度、增加激光频率。

**问题2：地图失真**：原因是初始位姿误差过大导致匹配失败，解决方法是确保初始位姿估计准确，或增加粗分辨率层。

**问题3：重定位失败**：Hector-SLAM缺乏全局定位能力，解决方法是配合AMCL使用，或在启动时手动给定位姿：

```bash
# 手动设置初始位姿
ros2 topic pub /initialpose geometry_msgs/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}}}"
```

### 12.5.4 官方要点——无里程计建图：适用与失效的边界

Hector SLAM 最大的卖点是不依赖里程计，官方 Wiki 明确给出其设计场景：手持建图设备、无编码器的平台（如部分无人机、扫地原型机）。但这也带来两个边界条件，与本章 12.5.3 的问题清单完全吻合：其一，初值完全靠「上一帧结果」外推，要求激光帧率高（官方建议 10 Hz 以上）且角速度受限，否则快速转身会直接导致建图漂移；其二，缺乏全局重定位能力，一旦丢失只能重启或配合 AMCL 恢复，这正是 12.5.3「问题3」给出 `/initialpose` 手动注入方案的原因。

对比理解：slam_toolbox 的 Ceres 匹配器与 Hector 同属「扫描-地图 + Gauss-Newton 派」（见本书第 11 章扩展），但多了里程计先验与回环后端；Cartographer 则用实时相关性匹配（粗）+ 非线性优化（精）的双层结构显式扩大初值容错窗。三者可视为本章 12.1.3 对比表在实现层面的注脚。

## 课后练习

1. **原理题:** 说明Hector-SLAM为什么不需要里程计信息？它的扫描-地图匹配策略相比帧-帧匹配有什么优势？

2. **推导题:** 推导Hector-SLAM中Gauss-Newton优化的雅可比矩阵，包括旋转参数θ的导数和地图梯度∇M的计算。

3. **编程题:** 实现一个简化版Hector-SLAM系统的扫描匹配模块，能够对模拟激光数据进行扫描-地图匹配并输出优化位姿。

4. **分析题:** 分析Hector-SLAM中双栅格地图的作用，说明由粗到精匹配策略如何提高匹配的鲁棒性和效率。

5. **配置题:** 在Gazebo仿真中启动Hector-SLAM，调整参数（地图分辨率、更新因子等），观察不同参数对建图质量的影响。

6. **设计题:** 某手持建图设备（无里程计）需要在2000m²的办公环境中快速建图。设计方案包括激光雷达选型、Hector-SLAM参数配置、建图路径规划策略和地图质量评估方法。

---

## 仿真结合实例（当前仓库）：为 Hector SLAM 准备二维激光输入

### 目标与知识点对应

Hector SLAM 依赖高频激光和 TF。本仓库未包含 Hector SLAM 节点，实例使用 `robot_sim_demo` 验证其所需的 `/scan`、`/tf` 和仿真时钟接口，为外部 Hector 节点接入做准备。

### 运行步骤

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_sim_demo gazebo2.launch.py \
  gui:=true rviz:=true drive:=true
```

```bash
ros2 topic hz /scan
ros2 topic echo /scan --once
ros2 run tf2_ros tf2_echo base_link laser_link
```

### 观察结果

RViz 中可同时显示 LaserScan 和 TF；外部 Hector 节点应将激光 frame 与机器人基座 frame 配置一致，并使用 `use_sim_time`。

### 源码与边界

仿真模型与桥接配置位于 `src/robot_sim_demo/models/wheeltec_robot/model.sdf` 与 `src/robot_sim_demo/config/gazebo2_bridge.yaml`；可替代的在线建图示例见 `src/slam_sim_demo_ros2/`。

当前仓库没有 Hector SLAM 实现，不能把 `slam_toolbox` 输出标为 Hector 结果。

学习材料：
- ROS Wiki —— hector_slam（Tutorials 与 hector_mapping 参数说明）：https://wiki.ros.org/hector_slam
- GitHub（tu-darmstadt-ros-pkg）—— hector_slam 官方仓库与 README：https://github.com/tu-darmstadt-ros-pkg/hector_slam
- docs.ros.org —— nav2_map_server map_saver_cli 用法：https://docs.ros.org/en/jazzy/p/nav2_map_server/
- The Construct —— ROS 2 SLAM 相关课程（Hector 与 slam_toolbox 对比）：https://www.theconstructsim.com/
- Robotics Back-End —— 激光 SLAM 建图教程与实测对比：https://roboticsbackend.com/