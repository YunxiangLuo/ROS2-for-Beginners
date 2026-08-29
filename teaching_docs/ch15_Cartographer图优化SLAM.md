# 第15章 Cartographer图优化SLAM

## 仿真结合实例（当前仓库）：记录 Cartographer 图优化所需传感器数据

### 目标与知识点对应

Cartographer 需要连续的激光、里程计和 TF 数据。本仓库没有 Cartographer 配置的可运行实现，因此先用 `robot_sim_demo` 采集一组可复现输入，理解局部子图和后端优化所依赖的数据关系。

### 运行步骤

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_sim_demo gazebo2.launch.py \
  gui:=true rviz:=true drive:=true
```

```bash
ros2 bag record -o /tmp/cartographer_input /scan /odom /tf /tf_static /clock
# 运行约 10 秒后 Ctrl+C
ros2 bag info /tmp/cartographer_input
```

### 观察结果

RViz 中可看到机器人运动和激光扫描；bag 的时间序列可用于后续 Cartographer 离线配置与回环实验。

### 源码与边界

- 仿真入口：`src/robot_sim_demo/launch/gazebo2.launch.py`
- 传感器桥：`src/robot_sim_demo/config/gazebo2_bridge.yaml`
- RViz：`src/robot_sim_demo/rviz/museum.rviz`

当前仓库没有 Cartographer 后端，不能把 `slam_toolbox` 的在线地图称为 Cartographer 图优化结果。

## 学习目标
- 理解图优化SLAM的基本原理和数学框架
- 掌握Cartographer系统的整体架构和设计思想
- 熟悉局部SLAM（前端匹配）算法流程
- 理解全局SLAM（后端优化）和回环检测机制
- 能够在ROS2中配置和使用Cartographer进行多传感器融合建图
- 掌握Cartographer参数调优方法

## 15.1 图优化SLAM基本原理

### 15.1.1 从滤波到图优化

传统滤波SLAM（EKF-SLAM, FastSLAM）存在两个根本性问题：
1. **累积误差：** 只维护当前状态估计，过去的误差无法修正
2. **线性化误差：** 系统线性化只发生在当前估计点，长期误差大

图优化SLAM（Graph-based SLAM）通过记录所有位姿和约束，构建全局优化问题来消除累积误差。

**核心思想：**
- 将SLAM问题建模为图（Graph）
- 图的节点（Vertex）= 机器人位姿 + 路标点
- 图的边（Edge）= 位姿间的约束（里程计、观测、回环）
- 优化目标 = 最小化所有约束误差的平方和

```
图结构示意:
   x1 ──── x2 ──── x3 ──── x4 ──── x5
    │       │       │       │       │
    │   ┌───┘       │       └───┐   │
    │   │           │           │   │
   lm1  lm2        lm3         lm4 lm5

x: 机器人位姿节点
lm: 路标点节点
─: 里程计边
│: 观测边
```

### 15.1.2 图优化的数学形式

**优化目标函数：**

```
X* = argmin Σ e_ij(X)ᵀ · Ω_ij · e_ij(X)
```

其中：
- X = {x₁, x₂, ..., xₙ, l₁, l₂, ..., lₘ} 为所有待优化变量
- e_ij(X) 为约束ij的误差向量
- Ω_ij 为约束ij的信息矩阵（协方差的逆）

**对于位姿图（Pose Graph）：**
```
e_ij(x_i, x_j) = t_ij ⊖ (x_i ⊖ x_j)
```

其中 ⊖ 表示位姿复合运算的逆，t_ij 为观测到的相对变换。

### 15.1.3 稀疏性利用

图优化SLAM的关键优势在于**稀疏性**：

```
Hessian矩阵结构:
[x₁ x₂ x₃ ... xₙ l₁ l₂ ... lₘ]
[x₁  X  X  .  .  .  X  .  .  .]
[x₂  X  X  X  .  .  .  X  .  .]
[x₃  .  X  X  X  .  .  .  X  .]
[.   .  .  X  X  .  .  .  .  .]
[l₁  X  .  .  .  .  X  .  .  .]
[l₂  .  X  .  .  .  .  X  .  .]
```

- 每个约束只关联少量节点
- H矩阵大部分为0，可使用稀疏求解器高效求解
- 复杂度从O(n³)降到接近O(n)

```python
import numpy as np
from scipy import sparse

class PoseGraphOptimization:
    """位姿图优化基础框架"""
    def __init__(self):
        self.nodes = {}     # {id: pose}
        self.edges = []     # [(i, j, measurement, information)]
        self.next_node_id = 0
        
    def add_node(self, pose: np.ndarray) -> int:
        """添加位姿节点"""
        node_id = self.next_node_id
        self.nodes[node_id] = pose.copy()
        self.next_node_id += 1
        return node_id
    
    def add_edge(self, i: int, j: int, 
                 measurement: np.ndarray,
                 information: np.ndarray):
        """添加约束边"""
        self.edges.append((i, j, measurement, information))
        
    def compute_error(self) -> float:
        """计算当前总误差"""
        total_error = 0.0
        for (i, j, meas, info) in self.edges:
            if i in self.nodes and j in self.nodes:
                error = self._compute_edge_error(
                    self.nodes[i], self.nodes[j], meas
                )
                total_error += error @ info @ error
        return total_error
    
    def _compute_edge_error(self, xi: np.ndarray, 
                              xj: np.ndarray,
                              measurement: np.ndarray) -> np.ndarray:
        """计算边的误差"""
        # 对于SE2位姿: (x, y, theta)
        dx = xj[0] - xi[0]
        dy = xj[1] - xi[1]
        dtheta = xj[2] - xi[2]
        
        # 预测的相对变换
        predicted = np.array([
            dx * np.cos(-xi[2]) - dy * np.sin(-xi[2]),
            dx * np.sin(-xi[2]) + dy * np.cos(-xi[2]),
            dtheta
        ])
        
        # 误差 = 测量 - 预测
        error = measurement - predicted
        error[2] = np.arctan2(np.sin(error[2]), np.cos(error[2]))
        return error
    
    def optimize(self, num_iterations: int = 10) -> list:
        """
        执行图优化（Gauss-Newton）
        
        返回: 优化后的位姿列表
        """
        for iteration in range(num_iterations):
            # 构建线性系统
            n = len(self.nodes) * 3
            H = sparse.lil_matrix((n, n))
            b = np.zeros(n)
            
            for (i, j, meas, info) in self.edges:
                if i not in self.nodes or j not in self.nodes:
                    continue
                    
                xi = self.nodes[i]
                xj = self.nodes[j]
                
                error = self._compute_edge_error(xi, xj, meas)
                
                # 雅可比矩阵（对xi和xj求导）
                Ji = self._compute_jacobian_i(xi, xj)
                Jj = self._compute_jacobian_j(xi, xj)
                
                # 更新Hessian和b
                idx_i = 3 * i
                idx_j = 3 * j
                
                H[idx_i:idx_i+3, idx_i:idx_i+3] += Ji.T @ info @ Ji
                H[idx_i:idx_i+3, idx_j:idx_j+3] += Ji.T @ info @ Jj
                H[idx_j:idx_j+3, idx_i:idx_i+3] += Jj.T @ info @ Ji
                H[idx_j:idx_j+3, idx_j:idx_j+3] += Jj.T @ info @ Jj
                
                b[idx_i:idx_i+3] += Ji.T @ info @ error
                b[idx_j:idx_j+3] += Jj.T @ info @ error
                
            # 固定第一个节点（固定map坐标系原点）
            H[:3, :] = 0
            H[:, :3] = 0
            H[:3, :3] = np.eye(3)
            b[:3] = 0
            
            # 求解 H·Δx = -b
            H_csc = H.tocsc()
            try:
                delta = sparse.linalg.spsolve(H_csc, -b)
            except:
                break
                
            # 更新位姿
            for node_id in sorted(self.nodes.keys()):
                idx = 3 * node_id
                self.nodes[node_id][0] += delta[idx]
                self.nodes[node_id][1] += delta[idx + 1]
                self.nodes[node_id][2] += delta[idx + 2]
                
            # 收敛检查
            if np.linalg.norm(delta) < 1e-4:
                break
                
        return list(self.nodes.values())
    
    def _compute_jacobian_i(self, xi: np.ndarray, 
                              xj: np.ndarray) -> np.ndarray:
        """计算误差对xi的雅可比"""
        theta = xi[2]
        dx = xj[0] - xi[0]
        dy = xj[1] - xi[1]
        
        J = np.array([
            [-np.cos(theta), -np.sin(theta), 
             -dx * np.sin(-theta) - dy * np.cos(-theta)],
            [np.sin(theta), -np.cos(theta),
             dx * np.cos(-theta) - dy * np.sin(-theta)],
            [0, 0, -1]
        ])
        return J
    
    def _compute_jacobian_j(self, xi: np.ndarray,
                              xj: np.ndarray) -> np.ndarray:
        """计算误差对xj的雅可比"""
        theta = xi[2]
        J = np.array([
            [np.cos(theta), np.sin(theta), 0],
            [-np.sin(theta), np.cos(theta), 0],
            [0, 0, 1]
        ])
        return J
```

### 15.1.4 图优化 vs 滤波方法

| 特性 | 图优化 (GraphSLAM) | 滤波 (EKF-SLAM) |
|------|-------------------|-----------------|
| 状态维护 | 全部位姿+地图 | 当前位姿+地图 |
| 历史修正 | 可修正全部历史 | 不可修正 |
| 回环检测 | 自然支持 | 需要额外处理 |
| 计算复杂度 | O(n³)但可稀疏加速 | O(n²) |
| 实时性 | 批量优化 | 在线实时 |
| 精度 | 高（全优化） | 中（线性化误差） |
| 适用场景 | 大规模、有回环 | 小规模、低噪声 |

## 15.2 Cartographer系统架构

### 15.2.1 总体架构

Cartographer是Google开源的基于图优化的SLAM系统，支持2D和3D建图。其核心架构分为前端和后端两部分：

```
传感器数据流:
┌─────────┐  LaserScan  ┌──────────────────────┐
│ 激光雷达  │──────────→│                      │
└─────────┘            │    Local SLAM        │
┌─────────┐  Imu       │    (前端匹配)         │
│   IMU    │──────────→│    - 体素滤波         │
└─────────┘            │    - 扫描匹配         │
┌─────────┐  Odometry  │    - 子图插入         │
│ 里程计   │──────────→│                      │
└─────────┘            └──────────┬───────────┘
                                  │ Submaps
                                  ↓
┌─────────────────────────────────────────────┐
│         Global SLAM (后端优化)               │
│    - 回环检测 (分支定界加速)                  │
│    - 图优化 (Ceres Solver)                   │
│    - 全局一致的位姿和地图修正                  │
└─────────────────────────────────────────────┘
```

### 15.2.2 核心概念

**Submap（子图）：**
- 由连续多帧激光数据构成的局部栅格地图
- 每个子图包含一定数量的激光扫描帧
- 子图内部通过前端匹配实现局部一致性

**Node（节点）：**
- 关键帧位姿
- 每个子图插入时创建一个节点
- 节点包含：位姿估计、时间戳、相关传感器数据

**Constraint（约束）：**
- 帧间约束：同一子图内相邻帧的匹配
- 子图间约束：不同子图之间的匹配（回环）
- 约束包含：相对变换 + 信息矩阵

```python
class CartographerCore:
    """Cartographer核心概念模拟"""
    def __init__(self):
        self.submaps = []         # 子图列表
        self.nodes = []           # 节点列表
        self.constraints = []     # 约束列表
        self.pose_graph = PoseGraphOptimization()
        
        # 参数
        self.submap_size = 10     # 每个子图的激光帧数
        self.current_submap_frames = 0
        
    def add_laser_scan(self, scan_points: np.ndarray,
                        odom_pose: np.ndarray,
                        timestamp: float) -> int:
        """
        添加激光扫描数据
        
        返回: 节点ID
        """
        # 1. 位姿预测（使用IMU/里程计外推）
        predicted_pose = self.predict_pose(odom_pose, timestamp)
        
        # 2. 扫描匹配（与当前子图匹配，优化位姿）
        if self.submaps:
            matched_pose = self.scan_match(
                scan_points, self.submaps[-1], predicted_pose
            )
        else:
            matched_pose = predicted_pose
            
        # 3. 创建节点
        node_id = len(self.nodes)
        self.nodes.append(matched_pose)
        
        # 4. 将扫描插入当前子图
        if self.submaps:
            self._insert_scan_into_submap(
                self.submaps[-1], scan_points, matched_pose
            )
            self.current_submap_frames += 1
            
            # 添加帧间约束
            if len(self.nodes) > 1:
                self.constraints.append({
                    'type': 'intra_submap',
                    'i': node_id - 1,
                    'j': node_id,
                    'transform': self._compute_relative_transform(
                        self.nodes[node_id-1], matched_pose
                    ),
                    'information': np.eye(3) * 100.0
                })
                
        # 5. 检查是否需要创建新子图
        if (not self.submaps or 
            self.current_submap_frames >= self.submap_size):
            self._start_new_submap()
            
        return node_id
    
    def _start_new_submap(self):
        """创建新子图"""
        new_submap = {
            'id': len(self.submaps),
            'grid': np.zeros((100, 100)),  # 占据栅格
            'origin': self.nodes[-1].copy() if self.nodes else np.zeros(3),
            'frames': 0
        }
        self.submaps.append(new_submap)
        self.current_submap_frames = 0
        
    def _insert_scan_into_submap(self, submap: dict,
                                   scan_points: np.ndarray,
                                   pose: np.ndarray):
        """将激光扫描插入子图"""
        theta = pose[2]
        c, s = np.cos(theta), np.sin(theta)
        
        for point in scan_points:
            wx = pose[0] + point[0]*c - point[1]*s
            wy = pose[1] + point[0]*s + point[1]*c
            
            gx = int((wx - submap['origin'][0]) / 0.05)
            gy = int((wy - submap['origin'][1]) / 0.05)
            
            if 0 <= gx < 100 and 0 <= gy < 100:
                submap['grid'][gy, gx] = 1.0  # 占据
                
        submap['frames'] += 1
        
    def predict_pose(self, odom_pose: np.ndarray,
                      timestamp: float) -> np.ndarray:
        """位姿预测（匀速模型）"""
        if len(self.nodes) < 2:
            return odom_pose
            
        # 使用匀速运动模型外推
        last_pose = self.nodes[-1]
        dt = timestamp - self._last_timestamp
        velocity = (last_pose - self.nodes[-2]) / dt
        
        return last_pose + velocity * dt
    
    def scan_match(self, scan_points: np.ndarray,
                    submap: dict,
                    initial_pose: np.ndarray) -> np.ndarray:
        """扫描-子图匹配（Ceres优化）"""
        # 简化实现：返回初值附近的最优位姿
        # 实际Cartographer使用Ceres Solver进行非线性优化
        best_pose = initial_pose.copy()
        best_score = -float('inf')
        
        # 在初值附近搜索
        for dx in np.linspace(-0.1, 0.1, 5):
            for dy in np.linspace(-0.1, 0.1, 5):
                for dt in np.linspace(-0.05, 0.05, 5):
                    candidate = initial_pose + np.array([dx, dy, dt])
                    score = self._compute_match_score(
                        scan_points, submap, candidate
                    )
                    if score > best_score:
                        best_score = score
                        best_pose = candidate
                        
        return best_pose
    
    def _compute_match_score(self, scan_points: np.ndarray,
                               submap: dict,
                               pose: np.ndarray) -> float:
        """计算扫描匹配得分"""
        theta = pose[2]
        c, s = np.cos(theta), np.sin(theta)
        origin = submap['origin']
        
        score = 0.0
        for point in scan_points:
            wx = pose[0] + point[0]*c - point[1]*s
            wy = pose[1] + point[0]*s + point[1]*c
            
            gx = int((wx - origin[0]) / 0.05)
            gy = int((wy - origin[1]) / 0.05)
            
            if 0 <= gx < 100 and 0 <= gy < 100:
                if submap['grid'][gy, gx] > 0.5:
                    score += 1.0
                    
        return score / max(len(scan_points), 1)
    
    def detect_loop_closure(self, current_node_id: int) -> list:
        """
        回环检测：将当前节点与历史子图匹配
        
        返回: 回环约束列表
        """
        loop_constraints = []
        current_pose = self.nodes[current_node_id]
        
        for i, submap in enumerate(self.submaps[:-1]):  # 跳过当前子图
            # 分支定界加速搜索
            relative_pose = self._fast_correlative_scan_match(
                current_pose, submap
            )
            
            if relative_pose is not None:
                score = self._compute_match_score(
                    self._last_scan, submap, current_pose + relative_pose
                )
                if score > 0.7:  # 匹配得分阈值
                    loop_constraints.append({
                        'type': 'loop_closure',
                        'submap_id': i,
                        'node_id': current_node_id,
                        'transform': relative_pose,
                        'score': score
                    })
                    
        return loop_constraints
```

### 15.2.3 Local SLAM前端流程

Cartographer的前端Local SLAM负责构建局部一致的子图：

```
传感器输入
    ↓
体素滤波 (Voxel Filter)
- 降低点云密度，去除离群点
- 自适应体素大小
    ↓
位姿外推器 (Pose Extrapolator)
- 基于匀速模型预测初始位姿
- 融合IMU数据（如有）
    ↓
扫描匹配 (Scan Match)
- Ceres Solver优化
- 最小化扫描与子图的差异
    ↓
运动滤波器 (Motion Filter)
- 时间/位移/角度阈值
- 筛选关键帧
    ↓
子图插入 (Submap Insertion)
- 将关键帧融入当前子图
- 更新子图占据概率
```

```python
class LocalSLAM:
    """Cartographer前端局部SLAM"""
    def __init__(self, resolution: float = 0.05):
        self.resolution = resolution
        self.current_submap = None
        self.pose_extrapolator = PoseExtrapolator()
        self.voxel_filter = VoxelFilter(0.05)
        self.motion_filter = MotionFilter(
            max_distance=0.3,
            max_angle=np.radians(30),
            max_time=1.0
        )
        
    def process_scan(self, scan_points: np.ndarray,
                      odom_pose: np.ndarray,
                      imu_data: np.ndarray = None,
                      timestamp: float = 0.0) -> dict:
        """处理一帧激光扫描"""
        # 1. 体素滤波
        filtered_scan = self.voxel_filter.filter(scan_points)
        
        # 2. 位姿外推
        predicted_pose = self.pose_extrapolator.extrapolate(
            odom_pose, imu_data, timestamp
        )
        
        # 3. 扫描匹配（Ceres优化）
        if self.current_submap is not None:
            matched_pose = self.ceres_scan_match(
                filtered_scan, self.current_submap, predicted_pose
            )
        else:
            matched_pose = predicted_pose
            
        # 4. 运动滤波
        if not self.motion_filter.should_insert(
            matched_pose, timestamp
        ):
            return {'inserted': False}
            
        # 5. 插入子图
        if self.current_submap is None:
            self.current_submap = Submap(
                self.resolution, matched_pose
            )
        
        self.current_submap.insert_scan(
            filtered_scan, matched_pose
        )
        
        # 6. 更新位姿外推器
        self.pose_extrapolator.add_pose(
            matched_pose, timestamp
        )
        
        result = {
            'inserted': True,
            'pose': matched_pose,
            'submap_id': self.current_submap.id,
            'scan_points': filtered_scan
        }
        
        # 7. 检查子图是否完成
        if self.current_submap.is_complete():
            result['submap_completed'] = True
            self.current_submap = None
            
        return result
    
    def ceres_scan_match(self, scan_points: np.ndarray,
                          submap: 'Submap',
                          initial_pose: np.ndarray) -> np.ndarray:
        """
        Ceres Solver扫描匹配
        
        目标函数:
        argmin Σ (1 - M(T·p_i))²
        其中M为子图的连续双线性插值
        """
        # 简化实现：Gauss-Newton优化
        pose = initial_pose.copy()
        
        for _ in range(10):
            theta = pose[2]
            R = np.array([[np.cos(theta), -np.sin(theta)],
                          [np.sin(theta), np.cos(theta)]])
            
            # 计算残差和雅可比
            J = np.zeros((3,))
            residual = 0.0
            
            for p in scan_points:
                transformed = R @ p + pose[:2]
                grid_pt = transformed / self.resolution
                
                # 双线性插值获取子图占据值和梯度
                occupancy, grad = submap.bilinear_interpolate(grid_pt)
                
                residual += (1 - occupancy)**2
                J[:2] += -2 * (1 - occupancy) * grad
                
            # Gauss-Newton更新
            H = np.outer(J, J) + np.eye(3) * 1e-6
            delta = -np.linalg.solve(H, J * residual)
            
            pose[:2] += delta[:2]
            pose[2] += delta[2]
            
            if np.linalg.norm(delta) < 1e-4:
                break
                
        return pose


class VoxelFilter:
    """体素滤波器"""
    def __init__(self, voxel_size: float):
        self.voxel_size = voxel_size
        
    def filter(self, points: np.ndarray) -> np.ndarray:
        """体素滤波降采样"""
        if len(points) == 0:
            return points
            
        # 计算体素坐标
        voxel_indices = np.floor(points / self.voxel_size).astype(int)
        
        # 去重（每个体素保留一个点）
        _, unique_indices = np.unique(
            voxel_indices, axis=0, return_index=True
        )
        
        return points[unique_indices]


class PoseExtrapolator:
    """位姿外推器"""
    def __init__(self):
        self.poses = []       # 历史位姿
        self.timestamps = []  # 对应时间戳
        self.linear_velocity = np.zeros(2)
        self.angular_velocity = 0.0
        
    def add_pose(self, pose: np.ndarray, timestamp: float):
        """添加新位姿"""
        if self.poses:
            dt = timestamp - self.timestamps[-1]
            if dt > 0:
                self.linear_velocity = (pose[:2] - self.poses[-1][:2]) / dt
                self.angular_velocity = (pose[2] - self.poses[-1][2]) / dt
                
        self.poses.append(pose)
        self.timestamps.append(timestamp)
        
        # 限制历史长度
        if len(self.poses) > 10:
            self.poses.pop(0)
            self.timestamps.pop(0)
            
    def extrapolate(self, odom_pose: np.ndarray,
                     imu_data: np.ndarray = None,
                     timestamp: float = 0.0) -> np.ndarray:
        """外推当前位姿"""
        if not self.poses:
            return odom_pose
            
        dt = 0.1  # 假设固定时间步长
        last_pose = self.poses[-1]
        
        # 匀速运动外推
        extrapolated = last_pose.copy()
        extrapolated[:2] += self.linear_velocity * dt
        extrapolated[2] += self.angular_velocity * dt
        
        return extrapolated


class MotionFilter:
    """运动滤波器：筛选关键帧"""
    def __init__(self, max_distance: float = 0.3,
                 max_angle: float = np.radians(30),
                 max_time: float = 1.0):
        self.max_distance = max_distance
        self.max_angle = max_angle
        self.max_time = max_time
        self.last_pose = None
        self.last_time = 0.0
        
    def should_insert(self, pose: np.ndarray,
                       timestamp: float) -> bool:
        """判断是否应该插入为新关键帧"""
        if self.last_pose is None:
            self.last_pose = pose
            self.last_time = timestamp
            return True
            
        dt = timestamp - self.last_time
        dx = np.linalg.norm(pose[:2] - self.last_pose[:2])
        dtheta = abs(pose[2] - self.last_pose[2])
        
        # 超过任一阈值则插入
        if dx > self.max_distance or dtheta > self.max_angle or dt > self.max_time:
            self.last_pose = pose
            self.last_time = timestamp
            return True
            
        return False


class Submap:
    """Cartographer子图"""
    def __init__(self, resolution: float, origin: np.ndarray):
        self.id = id(self)
        self.resolution = resolution
        self.origin = origin.copy()
        self.grid = np.zeros((200, 200), dtype=np.float32)  # 概率
        self.scans = []
        self.is_full = False
        self.max_scans = 50
        
    def insert_scan(self, scan_points: np.ndarray,
                     pose: np.ndarray):
        """插入激光帧到子图"""
        self.scans.append({
            'points': scan_points,
            'pose': pose
        })
        
        # 使用Bresenham更新占据概率
        theta = pose[2]
        R = np.array([[np.cos(theta), -np.sin(theta)],
                      [np.sin(theta), np.cos(theta)]])
        
        for point in scan_points:
            world_pt = R @ point + pose[:2]
            grid_pt = (world_pt - self.origin[:2]) / self.resolution
            gx, gy = int(grid_pt[0]), int(grid_pt[1])
            
            if 0 <= gx < 200 and 0 <= gy < 200:
                # log-odds更新
                self.grid[gy, gx] = min(self.grid[gy, gx] + 0.5, 5.0)
                
        self.is_full = len(self.scans) >= self.max_scans
        
    def is_complete(self) -> bool:
        """子图是否已完成"""
        return self.is_full or len(self.scans) >= self.max_scans
    
    def bilinear_interpolate(self, pt: np.ndarray) -> tuple:
        """双线性插值获取占据值和梯度"""
        x, y = pt[0], pt[1]
        x0, y0 = int(np.floor(x)), int(np.floor(y))
        x1, y1 = x0 + 1, y0 + 1
        
        if not (0 <= x0 < 199 and 0 <= y0 < 199):
            return 0.5, np.zeros(2)
            
        dx, dy = x - x0, y - y0
        
        v00 = self.grid[y0, x0]
        v10 = self.grid[y0, x1]
        v01 = self.grid[y1, x0]
        v11 = self.grid[y1, x1]
        
        # 双线性插值
        prob = (v00 * (1-dx) * (1-dy) +
                v10 * dx * (1-dy) +
                v01 * (1-dx) * dy +
                v11 * dx * dy)
        
        prob = 1.0 - 1.0 / (1.0 + np.exp(-prob))
        
        # 梯度
        grad_x = ((v10 - v00) * (1-dy) + (v11 - v01) * dy) / self.resolution
        grad_y = ((v01 - v00) * (1-dx) + (v11 - v10) * dx) / self.resolution
        
        return prob, np.array([grad_x, grad_y])
```

## 15.3 全局SLAM后端优化

### 15.3.1 回环检测

Cartographer使用**分支定界法（Branch and Bound）**加速回环检测的搜索过程。

**回环检测流程：**
```
1. 当子图完成，进入后端优化队列
2. 对每个新节点，在所有已完成子图中搜索匹配
3. 使用分支定界法快速搜索最优匹配
4. 匹配得分超过阈值时，添加回环约束到图
5. 执行全局图优化
```

```python
class FastCorrelativeScanMatcher:
    """分支定界快速相关扫描匹配"""
    def __init__(self, submap: Submap):
        self.submap = submap
        self.precomputed_grid = self._build_precomputed_grid()
        
    def _build_precomputed_grid(self) -> np.ndarray:
        """构建预计算栅格（多分辨率）"""
        # 生成栅格概率的积分图，加速匹配评分计算
        prob = 1.0 - 1.0 / (1.0 + np.exp(-self.submap.grid))
        binary = (prob > 0.5).astype(np.float32)
        
        # 积分图像
        integral = np.cumsum(np.cumsum(binary, axis=0), axis=1)
        return integral
        
    def match(self, scan_points: np.ndarray,
              initial_pose: np.ndarray,
              search_window: tuple = (3.0, 3.0, 0.5)) -> tuple:
        """
        分支定界搜索
        
        参数:
            scan_points: 激光扫描点云
            initial_pose: 初始位姿估计
            search_window: (linear_x, linear_y, angular) 搜索范围
            
        返回:
            (best_pose, best_score)
        """
        best_score = -float('inf')
        best_pose = initial_pose.copy()
        
        # 生成搜索候选
        n_x = int(search_window[0] / 0.05)
        n_y = int(search_window[1] / 0.05)
        n_theta = int(search_window[2] / 0.01)
        
        # 多分辨率搜索：
        # 1. 粗分辨率搜索（大步长）
        # 2. 最优候选的细分辨率搜索
        for theta_idx in range(n_theta):
            theta = initial_pose[2] - search_window[2]/2 + theta_idx * 0.01
            
            for x_idx in range(0, n_x, 5):  # 粗步长
                for y_idx in range(0, n_y, 5):
                    x = (initial_pose[0] - search_window[0]/2 + 
                         x_idx * 0.05)
                    y = (initial_pose[1] - search_window[1]/2 + 
                         y_idx * 0.05)
                    
                    candidate = np.array([x, y, theta])
                    score = self._compute_score(
                        scan_points, candidate
                    )
                    
                    if score > best_score:
                        best_score = score
                        best_pose = candidate
                        
        # 精细搜索（在最优候选附近）
        if best_score > 0.5:
            for x_idx in range(-5, 6):  # 精细搜索
                for y_idx in range(-5, 6):
                    candidate = best_pose + np.array([
                        x_idx * 0.01, y_idx * 0.01, 0.0
                    ])
                    score = self._compute_score(
                        scan_points, candidate
                    )
                    if score > best_score:
                        best_score = score
                        best_pose = candidate
                        
        return best_pose, best_score
    
    def _compute_score(self, scan_points: np.ndarray,
                        pose: np.ndarray) -> float:
        """计算匹配得分"""
        theta = pose[2]
        R = np.array([[np.cos(theta), -np.sin(theta)],
                      [np.sin(theta), np.cos(theta)]])
        origin = self.submap.origin
        
        score = 0.0
        for point in scan_points:
            world_pt = R @ point + pose[:2]
            grid_pt = ((world_pt - origin[:2]) / 
                       self.submap.resolution)
            gx, gy = int(grid_pt[0]), int(grid_pt[1])
            
            if 0 <= gx < 200 and 0 <= gy < 200:
                prob = 1.0 - 1.0 / (1.0 + np.exp(-self.submap.grid[gy, gx]))
                if prob > 0.5:
                    score += 1.0
                    
        return score / len(scan_points)


class GlobalSLAM:
    """Cartographer全局SLAM后端"""
    def __init__(self):
        self.completed_submaps = []
        self.nodes = []
        self.constraints = []
        self.pose_graph = PoseGraphOptimization()
        
        # 回环检测参数
        self.min_score_for_loop_closure = 0.6
        self.optimize_every_n_nodes = 3
        
    def add_completed_submap(self, submap: Submap):
        """添加完成的子图到后端"""
        self.completed_submaps.append(submap)
        
    def add_node(self, pose: np.ndarray, scan: np.ndarray,
                  timestamp: float) -> int:
        """添加新节点并检测回环"""
        node_id = len(self.nodes)
        self.nodes.append({
            'pose': pose,
            'scan': scan,
            'timestamp': timestamp,
            'submap_id': len(self.completed_submaps) - 1
        })
        
        # 回环检测
        loop_closures = self._detect_loop_closures(node_id)
        
        # 添加回环约束
        for lc in loop_closures:
            self.constraints.append({
                'type': 'loop_closure',
                'i': lc['node_id'],
                'j': node_id,
                'transform': lc['transform'],
                'information': np.eye(3) * 50.0
            })
            
        # 周期性全局优化
        if len(self.nodes) % self.optimize_every_n_nodes == 0:
            self.run_global_optimization()
            
        return node_id
    
    def _detect_loop_closures(self, node_id: int) -> list:
        """检测回环"""
        closures = []
        node = self.nodes[node_id]
        
        for i, submap in enumerate(self.completed_submaps):
            # 跳过当前子图
            if i == node['submap_id']:
                continue
                
            matcher = FastCorrelativeScanMatcher(submap)
            matched_pose, score = matcher.match(
                node['scan'], node['pose']
            )
            
            if score > self.min_score_for_loop_closure:
                relative_transform = self._compute_relative(
                    node['pose'], matched_pose
                )
                closures.append({
                    'node_id': node_id,
                    'submap_id': i,
                    'transform': relative_transform,
                    'score': score
                })
                
        return closures
    
    def _compute_relative(self, pose_a: np.ndarray,
                           pose_b: np.ndarray) -> np.ndarray:
        """计算相对变换"""
        return np.array([
            pose_b[0] - pose_a[0],
            pose_b[1] - pose_a[1],
            pose_b[2] - pose_a[2]
        ])
    
    def run_global_optimization(self):
        """执行全局图优化"""
        # 构建位姿图
        self.pose_graph = PoseGraphOptimization()
        
        # 添加节点
        node_ids = []
        for node in self.nodes:
            nid = self.pose_graph.add_node(node['pose'])
            node_ids.append(nid)
            
        # 添加里程计约束（连续节点间）
        for i in range(len(self.nodes) - 1):
            meas = self._compute_relative(
                self.nodes[i]['pose'],
                self.nodes[i+1]['pose']
            )
            self.pose_graph.add_edge(
                node_ids[i], node_ids[i+1],
                meas, np.eye(3) * 100.0
            )
            
        # 添加回环约束
        for constraint in self.constraints:
            if constraint['type'] == 'loop_closure':
                self.pose_graph.add_edge(
                    constraint['i'], constraint['j'],
                    constraint['transform'],
                    constraint['information']
                )
                
        # 执行优化
        optimized_poses = self.pose_graph.optimize(
            num_iterations=20
        )
        
        # 更新节点位姿
        for i, opt_pose in enumerate(optimized_poses):
            if i < len(self.nodes):
                self.nodes[i]['pose'] = opt_pose
```

### 15.3.2 Ceres Solver在Cartographer中的应用

Cartographer使用Google的Ceres Solver进行非线性最小二乘优化：

```cpp
// Cartographer中Ceres优化问题的构建（伪代码）
struct ScanMatchCostFunction {
    ScanMatchCostFunction(const Eigen::Vector2d& point,
                          const Grid2D& grid)
        : point_(point), grid_(grid) {}
    
    template <typename T>
    bool operator()(const T* const pose,
                    T* residual) const {
        // 变换激光点
        const T transformed_x = pose[0] + cos(pose[2]) * T(point_.x()) 
                                        - sin(pose[2]) * T(point_.y());
        const T transformed_y = pose[1] + sin(pose[2]) * T(point_.x()) 
                                        + cos(pose[2]) * T(point_.y());
        
        // 双线性插值获取地图占据概率
        const T occupancy = grid_.GetOccupancy(transformed_x, transformed_y);
        
        // 残差 = 1 - occupancy
        residual[0] = T(1.0) - occupancy;
        return true;
    }
    
private:
    Eigen::Vector2d point_;
    const Grid2D& grid_;
};
```

### 15.3.3 全局优化效果

全局优化的核心作用是消除累积误差：

```
优化前（漂移明显）:
┌───────────────────┐
│  x1 → x2 → x3     │
│       ↓          │
│       x4 → x5     │
│            ↓      │
│  ←--- 回环 ---→   │
│  误差积累 → 地图错位 │
└───────────────────┘

优化后（全局一致）:
┌───────────────────┐
│ x1 → x2 → x3     │
│  ↓    ↓    ↓     │
│ x4 → x5 → x1 ←---│  ← 回环闭合
│  所有约束均衡分布     │
└───────────────────┘
```

## 15.4 Cartographer配置与ROS2集成

### 15.4.1 LUA配置文件

Cartographer使用LUA脚本进行配置：

```lua
-- cartographer_2d.lua
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D = {
  min_range = 0.05,
  max_range = 8.0,
  min_z = -0.5,
  max_z = 1.5,
  missing_data_ray_length = 5.,
  num_accumulated_range_data = 1,
  voxel_filter_size = 0.05,
  
  adaptive_voxel_filter = {
    max_length = 0.5,
    min_num_points = 200,
    max_range = 8.,
  },
  
  use_imu_data = false,
  use_online_correlative_scan_matching = true,
  
  real_time_correlative_scan_matcher = {
    linear_search_window = 0.1,
    angular_search_window = math.rad(20.),
    translation_delta_cost_weight = 1e-1,
    rotation_delta_cost_weight = 1e-1,
  },
  
  ceres_scan_matcher = {
    occupied_space_weight = 20.,
    translation_weight = 10.,
    rotation_weight = 40.,
    ceres_solver_options = {
      use_nonmonotonic_steps = true,
      max_num_iterations = 10,
      num_threads = 1,
    },
  },
  
  motion_filter = {
    max_time_seconds = 1.0,
    max_distance_meters = 0.3,
    max_angle_radians = math.rad(30.),
  },
}

POSE_GRAPH = {
  optimize_every_n_nodes = 90,
  constraint_builder = {
    sampling_ratio = 0.3,
    max_constraint_distance = 15.,
    min_score = 0.55,
    global_localization_min_score = 0.6,
    loop_closure_translation_weight = 1.1e4,
    loop_closure_rotation_weight = 1e5,
    
    log_residual_histogram = true,
    
    ceres_scan_matcher = {
      occupied_space_weight = 30.,
      translation_weight = 10.,
      rotation_weight = 40.,
      ceres_solver_options = {
        use_nonmonotonic_steps = true,
        max_num_iterations = 10,
        num_threads = 1,
      },
    },
    
    fast_correlative_scan_matcher = {
      linear_search_window = 7.,
      angular_search_window = math.rad(30.),
      branch_and_bound_depth = 7,
    },
  },
  
  matcher_translation_weight = 5e2,
  matcher_rotation_weight = 1.6e3,
  
  optimization_problem = {
    huber_scale = 1e1,
    acceleration_weight = 1e3,
    rotation_weight = 3e5,
    local_slam_pose_translation_weight = 1e5,
    local_slam_pose_rotation_weight = 1e5,
    odometry_translation_weight = 1e5,
    odometry_rotation_weight = 1e5,
    fixed_frame_pose_translation_weight = 1e1,
    fixed_frame_pose_rotation_weight = 1e2,
    log_solver_summary = false,
    use_online_imu_extrinsics_in_3d = true,
    fix_z_in_3d = false,
  },
  
  max_num_final_iterations = 200,
  global_sampling_ratio = 0.003,
  log_residual_histogram = true,
  global_constraint_search_after_n_seconds = 10.,
}
```

### 15.4.2 ROS2启动配置

```bash
# 安装 Cartographer ROS 2
sudo apt install ros-jazzy-cartographer-ros ros-jazzy-cartographer-rviz

# 启动2D建图
ros2 launch slam_lab cartographer_mapping.launch.py \
  configuration_directory:=src/lab_code/ch10_lab/slam_lab/config/cartographer \
  configuration_basename:=xbot_2d.lua \
  use_sim_time:=true

# 或使用自定义启动文件
```

```python
# cartographer_demo_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            arguments=[
                '-configuration_directory', '/path/to/config',
                '-configuration_basename', 'office_2d.lua'
            ],
            remappings=[
                ('/scan', '/robot/scan'),
                ('/odom', '/robot/odom'),
            ]
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid',
            parameters=[{
                'publish_period_sec': 1.0,
                'resolution': 0.05,
            }]
        ),
    ])
```

### 15.4.3 参数调优指南

**前端匹配参数：**

| 参数 | 说明 | 调优建议 |
|------|------|---------|
| voxel_filter_size | 体素滤波大小 | 0.05(室内)~0.2(室外) |
| ceres_scan_matcher.translation_weight | 平移匹配权重 | 增大提高平移精度 |
| ceres_scan_matcher.rotation_weight | 旋转匹配权重 | 增大提高旋转精度 |
| min_range/max_range | 激光有效范围 | 根据传感器型号设置 |
| use_online_correlative_scan_matching | 在线相关匹配 | 复杂环境启用 |

**后端优化参数：**

| 参数 | 说明 | 调优建议 |
|------|------|---------|
| optimize_every_n_nodes | 优化间隔 | 小值更频繁但计算量大 |
| min_score | 回环检测最低得分 | 0.5~0.7，太高漏检 |
| linear_search_window | 回环搜索窗口(m) | 大范围回环需要增大 |
| angular_search_window | 回环搜索窗口(rad) | 旋转不确定时增大 |
| branch_and_bound_depth | 分支定界深度 | 7~12，深度大速度快 |

### 15.4.4 多传感器融合配置

Cartographer支持激光+IMU+里程计的多传感器融合：

```lua
-- 启用IMU
TRAJECTORY_BUILDER_2D.use_imu_data = true

-- 配置IMU参数
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 10.

-- 启用里程计
options.use_odometry = true

-- 多激光雷达配置
options.num_laser_scans = 2  -- 使用两个激光雷达

-- 配置每个激光雷达的topic映射
-- 通过remapping实现
```

```bash
# 启动多传感器配置（使用自定义 Lua 配置）
ros2 run cartographer_ros cartographer_node \
  -configuration_directory ./config \
  -configuration_basename multi_sensor_2d.lua

# 话题重映射
# 在launch文件中添加remappings
```

## 15.5 建图实践与案例

### 15.5.1 完整建图工作流

```bash
# 1. 启动仿真或机器人
ros2 launch robot_sim_demo gazebo2.launch.py drive:=false

# 2. 启动Cartographer建图
ros2 launch slam_lab cartographer_mapping.launch.py \
  configuration_directory:=src/lab_code/ch10_lab/slam_lab/config/cartographer \
  configuration_basename:=xbot_2d.lua \
  use_sim_time:=true

# 3. 启动RViz2可视化
rviz2 -d src/robot_sim_demo/rviz/museum.rviz

# 4. 控制机器人探索
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 5. 建图完成后的处理
# 保存pbstream
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
  "{filename: '${HOME}/maps/my_map.pbstream'}"

# 将pbstream转为pgm/yaml
ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
  -pbstream_filename ${HOME}/maps/my_map.pbstream \
  -map_filestem ${HOME}/maps/my_map

# 6. 使用生成的地图进行导航
ros2 launch nav2_bringup navigation_launch.py \
  map:=${HOME}/maps/my_map.yaml
```

### 15.5.2 建图质量评估

```python
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np

class CartographerMapQuality(Node):
    """Cartographer建图质量评估"""
    def __init__(self):
        super().__init__('cartographer_map_quality')
        
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        self.submap_list_sub = self.create_subscription(
            OccupancyGrid, '/submap_list', self.submap_callback, 10)
            
        self.map_count = 0
        self.submap_count = 0
        
    def map_callback(self, msg: OccupancyGrid):
        self.map_count += 1
        data = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        
        # 计算建图覆盖率和质量指标
        total = data.size
        mapped = np.sum(data >= 0)
        occupied = np.sum(data == 100)
        free = np.sum(data == 0)
        
        coverage = mapped / total * 100
        occupied_ratio = occupied / mapped * 100 if mapped > 0 else 0
        
        self.get_logger().info(
            f'建图进度: {coverage:.1f}% '
            f'(占据={occupied_ratio:.1f}%, '
            f'空闲={free/mapped*100:.1f}%)'
        )
        
        # 建图完成判定
        if coverage > 90 and self.map_count > 100:
            self.get_logger().info(
                '建图覆盖率达标，建议完成建图并保存地图'
            )
            
    def submap_callback(self, msg):
        """监控子图数量"""
        self.submap_count = len(msg.submap_entry)
        self.get_logger().info(
            f'子图数量: {self.submap_count}', 
            throttle_duration_sec=5.0
        )
```

### 15.5.3 纯定位模式

在建图完成后，Cartographer也可以运行在纯定位模式：

```lua
-- localization.lua
-- 在已有地图上定位
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = true,
  use_odometry = true,
  num_laser_scans = 1,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- 定位模式：不建图
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false
POSE_GRAPH.optimize_every_n_nodes = 0
```

```bash
# 在已有 pbstream 地图上启动定位
ros2 run cartographer_ros cartographer_node \
  -configuration_directory ./config \
  -configuration_basename localization.lua \
  -load_state_filename ${HOME}/maps/my_map.pbstream
```

### 15.5.4 大规模建图优化策略

对于大规模环境（>10000m²）的建图，需要特殊的优化策略：

```lua
-- large_scale_mapping.lua
-- 大规模建图参数优化

TRAJECTORY_BUILDER_2D = {
  -- 增大体素滤波以降低计算量
  voxel_filter_size = 0.15,
  
  -- 降低关键帧插入频率
  motion_filter = {
    max_time_seconds = 3.0,
    max_distance_meters = 0.5,
    max_angle_radians = math.rad(45.),
  },
  
  ceres_scan_matcher = {
    -- 减少Ceres迭代次数
    ceres_solver_options = {
      max_num_iterations = 5,
    },
  },
}

POSE_GRAPH = {
  -- 降低后端优化频率
  optimize_every_n_nodes = 150,
  
  -- 增大回环搜索范围
  constraint_builder = {
    max_constraint_distance = 30.,
    fast_correlative_scan_matcher = {
      linear_search_window = 10.,
      angular_search_window = math.rad(45.),
    },
  },
}
```

### 15.5.5 常见问题排查

**问题1：建图漂移**
- 检查激光频率是否足够（建议>5Hz）
- 检查IMU标定是否正确
- 降低移动速度
- 增大 `ceres_scan_matcher` 的权重

**问题2：回环未闭合**
```lua
-- 增强回环检测
POSE_GRAPH.constraint_builder.min_score = 0.4  -- 降低回环阈值
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 10.0
POSE_GRAPH.optimize_every_n_nodes = 30
```

**问题3：CPU负载过高**
```lua
-- 降低计算负载
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.1
POSE_GRAPH.optimize_every_n_nodes = 200
POSE_GRAPH.constraint_builder.sampling_ratio = 0.1
```

## 课后练习

1. **原理题:** 阐述图优化SLAM的基本原理，说明为什么图优化能够消除SLAM中的累积误差，推导Gauss-Newton优化的核心公式。

2. **编程题:** 实现一个简化的位姿图优化系统，支持添加位姿节点、添加里程计约束和回环约束，使用Gauss-Newton法进行全局优化。

3. **分析题:** 分析Cartographer前后端分离设计的优势，说明Local SLAM和Global SLAM各自的作用和相互协作机制。

4. **配置题:** 编写Cartographer的LUA配置文件，使得在10000m²的仓库环境中实现高质量建图，包括前端匹配、后端优化和回环检测参数。

5. **操作题:** 描述完整的Cartographer建图工作流程：从启动仿真、开始建图、控制探索到最终保存地图并转换为PGM/YAML格式。

6. **设计题:** 某高层建筑需要3D建图，机器人配备3D激光雷达、IMU和轮式里程计。设计基于Cartographer 3D的建图方案，包括传感器配置、参数调优策略、多楼层建图方案和地图配准方法。
