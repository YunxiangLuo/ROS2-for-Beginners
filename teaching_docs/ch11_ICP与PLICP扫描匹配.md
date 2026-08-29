# 第11章 ICP与PLICP扫描匹配

## 仿真结合实例（当前仓库）：采集相邻激光帧作为扫描匹配输入

### 目标与知识点对应

ICP/PLICP 需要连续的激光扫描和位姿初值。本仓库没有独立 ICP/PLICP 实现，因此实例聚焦于从 Gazebo 获得稳定的 `/scan`、`/odom`、`/tf` 输入，并验证数据可供外部匹配器消费。

### 运行步骤

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_sim_demo gazebo2.launch.py \
  gui:=false rviz:=false drive:=true
```

```bash
# 另一个终端记录短时间扫描和里程计
source install/setup.bash
ros2 bag record -o /tmp/scan_match_input /scan /odom /tf /tf_static
# 约 10 秒后 Ctrl+C
ros2 bag info /tmp/scan_match_input
```

### 观察结果

- Bag 中同时包含扫描帧、里程计和 TF，可用于比较相邻帧的初始位姿与匹配增量。
- 在 RViz 中显示 `LaserScan`，机器人运动时扫描相对环境发生变化。

### 源码与边界

- 仿真桥：`src/robot_sim_demo/config/gazebo2_bridge.yaml`
- SLAM 输入配置：`src/slam_sim_demo_ros2/params/slam_toolbox_params.yaml`

仓库没有 ICP/PLICP 求解器；本实例不宣称完成配准或输出算法精度，只完成可复现输入数据采集。

## 学习目标
- 理解ICP扫描匹配算法的基本原理与数学推导
- 掌握PLICP(Point-to-Line ICP)算法及其优势
- 熟悉Gauss-Newton优化方法在扫描匹配中的应用
- 能够使用Python实现基本的ICP和PLICP算法
- 了解扫描匹配在ROS2 SLAM中的实际应用

## 11.1 ICP扫描匹配算法

### 11.1.1 扫描匹配问题定义

扫描匹配 (Scan Matching) 是SLAM前端的核心步骤，目标是根据两帧激光点云之间的几何关系，求解机器人在这两帧之间的相对位姿变化。

**输入：**
- 参考帧点云 P = {p₁, p₂, ..., pₙ}
- 当前帧点云 Q = {q₁, q₂, ..., qₘ}

**输出：**
- 变换矩阵 T = (R, t)，将Q映射到P的坐标系

**数学形式：**
```
T* = argmin Σ || p_i - T(q_i) ||²
```

其中 T(q_i) = R · q_i + t，R为旋转矩阵，t为平移向量。

### 11.1.2 ICP算法步骤

ICP (Iterative Closest Point) 通过迭代方式求解点云配准问题：

```
算法流程:
1. 初始化变换 T₀ (通常使用里程计或匀速模型)
2. 对Q中每个点，在P中寻找最近邻点
3. 计算使匹配点对距离和最小的变换 T
4. 应用T到Q，重复步骤2-3
5. 当收敛或达到最大迭代次数时停止
```

```python
import numpy as np
from scipy.spatial import KDTree

class ICP:
    """Iterative Closest Point 扫描匹配算法"""
    def __init__(self, max_iterations: int = 50, tolerance: float = 1e-6):
        self.max_iterations = max_iterations
        self.tolerance = tolerance
        
    def align(self, source: np.ndarray, target: np.ndarray, 
              initial_guess: np.ndarray = None) -> tuple:
        """
        执行ICP扫描匹配
        
        参数:
            source: 当前帧点云 (N, 2)
            target: 参考帧点云 (M, 2)
            initial_guess: 初始变换估计 [x, y, theta]
            
        返回:
            (R, t): 旋转矩阵和平移向量
            stats: 收敛信息
        """
        if initial_guess is None:
            R = np.eye(2)
            t = np.zeros(2)
        else:
            theta = initial_guess[2]
            R = np.array([[np.cos(theta), -np.sin(theta)],
                          [np.sin(theta), np.cos(theta)]])
            t = initial_guess[:2]
            
        tree = KDTree(target)
        prev_error = float('inf')
        stats = {'iterations': 0, 'errors': []}
        
        for iteration in range(self.max_iterations):
            # Step 1: 变换源点云
            transformed = source @ R.T + t
            
            # Step 2: 寻找最近邻对应点
            distances, indices = tree.query(transformed)
            
            # Step 3: 计算最优变换
            matched_target = target[indices]
            R_new, t_new = self._compute_optimal_transform(
                source, matched_target
            )
            
            # Step 4: 更新变换
            R = R_new @ R
            t = R_new @ t + t_new
            
            # Step 5: 检查收敛
            mean_error = np.mean(distances)
            stats['errors'].append(mean_error)
            stats['iterations'] = iteration + 1
            
            if abs(prev_error - mean_error) < self.tolerance:
                break
            prev_error = mean_error
            
        return (R, t), stats
    
    def _compute_optimal_transform(self, source: np.ndarray, 
                                    target: np.ndarray) -> tuple:
        """计算最优变换（SVD解法）"""
        # 计算质心
        mu_s = np.mean(source, axis=0)
        mu_t = np.mean(target, axis=0)
        
        # 去质心
        source_centered = source - mu_s
        target_centered = target - mu_t
        
        # SVD求解旋转
        W = source_centered.T @ target_centered
        U, _, Vt = np.linalg.svd(W)
        
        R = U @ Vt
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = U @ Vt
            
        t = mu_t - R @ mu_s
        return R, t

def test_icp():
    """测试ICP算法"""
    # 生成参考点云
    np.random.seed(42)
    target = np.random.rand(100, 2) * 10
    
    # 生成变换后的源点云
    true_R = np.array([[np.cos(0.3), -np.sin(0.3)],
                       [np.sin(0.3), np.cos(0.3)]])
    true_t = np.array([1.0, 0.5])
    source = target @ true_R.T + true_t
    source += np.random.randn(100, 2) * 0.1  # 添加噪声
    
    # ICP配准
    icp = ICP()
    (R_est, t_est), stats = icp.align(source, target)
    
    print(f"真实旋转: {np.arctan2(true_R[1,0], true_R[0,0]):.4f} rad")
    print(f"估计旋转: {np.arctan2(R_est[1,0], R_est[0,0]):.4f} rad")
    print(f"真实平移: {true_t}")
    print(f"估计平移: {t_est}")
    print(f"迭代次数: {stats['iterations']}")
    print(f"最终误差: {stats['errors'][-1]:.6f}")
```

### 11.1.3 ICP的数学推导

ICP的核心是最小化对应点之间的欧氏距离平方和：

```
E(R, t) = Σ || p_i - (R·q_i + t) ||²
```

**求解旋转：** 使用SVD分解

```
W = Σ (q_i - μ_q)(p_i - μ_p)ᵀ
W = U·Σ·Vᵀ
R = V·Uᵀ
```

**求解平移：** 旋转后计算

```
t = μ_p - R·μ_q
```

### 11.1.4 ICP的局限性

- **局部最优：** ICP对初始位姿敏感，容易陷入局部最小值
- **点云稀疏性：** 在结构简单环境中匹配质量差
- **计算效率：** 最近邻搜索是O(n²)复杂度（使用KDTree可优化为O(n·log n)）
- **对应关系误差：** 简单最近邻假设可能导致错误匹配

```python
import numpy as np
import matplotlib.pyplot as plt

class ICPWithAnalysis(ICP):
    """带分析功能的ICP"""
    def analyze_convergence(self, source: np.ndarray, 
                             target: np.ndarray) -> dict:
        """分析ICP收敛行为"""
        analysis = {}
        
        # 不同初始位姿下的收敛分析
        initial_offsets = [0.0, 0.5, 1.0, 2.0]
        results = []
        
        for offset in initial_offsets:
            init_guess = np.array([offset, offset, offset * 0.1])
            (R, t), stats = self.align(source, target, init_guess)
            results.append({
                'init_offset': offset,
                'final_error': stats['errors'][-1],
                'iterations': stats['iterations'],
                'converged': stats['errors'][-1] < 0.01
            })
            
        analysis['convergence_test'] = results
        
        # 噪声灵敏度分析
        noise_levels = [0.0, 0.05, 0.1, 0.2, 0.5]
        noise_results = []
        
        for noise in noise_levels:
            noisy_source = source + np.random.randn(*source.shape) * noise
            (R, t), stats = self.align(noisy_source, target)
            angle_error = abs(np.arctan2(R[1,0], R[0,0]) - 0.3)
            translation_error = np.linalg.norm(np.array([1.0, 0.5]) - t)
            noise_results.append({
                'noise_level': noise,
                'angle_error': angle_error,
                'translation_error': translation_error
            })
            
        analysis['noise_sensitivity'] = noise_results
        return analysis
```

## 11.2 PLICP点线ICP算法

### 11.2.1 PLICP原理

PLICP (Point-to-Line ICP) 是ICP的改进版本，使用点到直线的距离代替点到点的距离。这使得收敛速度更快、精度更高，尤其适用于结构化环境。

**目标函数：**
```
E(T) = Σ (n_iᵀ · (T(q_i) - p_i))²
```

其中 n_i 为目标点 p_i 处的法向量。

**点到线距离示意：**
```
      线(L)
        |
  q_i   |    p_i (参考点)
   ×----|----×
        |    ↑
        |    n_i (法向量)
  距离 = n_i · (T(q_i) - p_i)
```

### 11.2.2 法向量估计

法向量估计是PLICP的关键步骤：

```python
import numpy as np
from scipy.spatial import KDTree

class PointToLineICP:
    """Point-to-Line ICP扫描匹配"""
    def __init__(self, max_iterations: int = 20, tolerance: float = 1e-6):
        self.max_iterations = max_iterations
        self.tolerance = tolerance
        
    def estimate_normals(self, points: np.ndarray, k: int = 5) -> np.ndarray:
        """
        估计点云中每个点的法向量
        
        参数:
            points: (N, 2) 点云
            k: 近邻数量
            
        返回:
            normals: (N, 2) 法向量
        """
        tree = KDTree(points)
        normals = np.zeros_like(points)
        
        for i, point in enumerate(points):
            # 寻找k个近邻
            _, indices = tree.query(point, k=k + 1)
            neighbors = points[indices[1:]]  # 排除自身
            
            # PCA计算法向量
            centered = neighbors - np.mean(neighbors, axis=0)
            cov = centered.T @ centered / (k - 1)
            eigenvalues, eigenvectors = np.linalg.eigh(cov)
            
            # 最小特征值对应的特征向量为法向量
            normal = eigenvectors[:, 0]
            
            # 归一化并确保指向一致（朝向传感器）
            if np.linalg.norm(normal) > 0:
                normal = normal / np.linalg.norm(normal)
                # 确保法向量指向外部（远离传感器）
                if normal @ point < 0:
                    normal = -normal
                    
            normals[i] = normal
            
        return normals
    
    def align(self, source: np.ndarray, target: np.ndarray,
              initial_guess: np.ndarray = None) -> tuple:
        """
        执行PLICP扫描匹配
        """
        if initial_guess is None:
            pose = np.zeros(3)
        else:
            pose = initial_guess.copy()
            
        target_normals = self.estimate_normals(target)
        tree = KDTree(target)
        
        prev_error = float('inf')
        stats = {'iterations': 0, 'errors': []}
        
        for iteration in range(self.max_iterations):
            # 变换源点云
            theta = pose[2]
            R = np.array([[np.cos(theta), -np.sin(theta)],
                          [np.sin(theta), np.cos(theta)]])
            transformed = source @ R.T + pose[:2]
            
            # 寻找最近邻和法向量
            distances, indices = tree.query(transformed)
            matched_target = target[indices]
            matched_normals = target_normals[indices]
            
            # 计算点到线距离
            diff = transformed - matched_target
            point_to_line_dist = np.sum(diff * matched_normals, axis=1)
            error = np.sum(point_to_line_dist ** 2)
            
            # Gauss-Newton迭代
            H = np.zeros((3, 3))
            b = np.zeros(3)
            
            for i in range(len(source)):
                q = source[i]
                n = matched_normals[i]
                d = point_to_line_dist[i]
                
                theta = pose[2]
                R_i = np.array([[-np.sin(theta), -np.cos(theta)],
                                [np.cos(theta), -np.sin(theta)]])
                
                # 雅可比: ∂e/∂(x, y, theta)
                J = np.zeros(3)
                J[:2] = n
                J[2] = n @ (R_i @ q)
                
                H += np.outer(J, J)
                b += J * d
                
            # 求解 H·Δ = -b
            try:
                H_inv = np.linalg.inv(H + np.eye(3) * 1e-6)
                delta = -H_inv @ b
            except np.linalg.LinAlgError:
                break
                
            # 更新位姿
            pose[:2] += delta[:2]
            pose[2] += delta[2]
            pose[2] = np.arctan2(np.sin(pose[2]), np.cos(pose[2]))
            
            mean_error = error / len(source)
            stats['errors'].append(mean_error)
            stats['iterations'] = iteration + 1
            
            if abs(prev_error - mean_error) < self.tolerance:
                break
            prev_error = mean_error
            
        return pose, stats
```

### 11.2.3 PLICP的数学推导

PLICP的目标函数（点到线距离的平方和）：

```
E(T) = Σ (n_iᵀ · (R·q_i + t - p_i))²
```

**Gauss-Newton优化：**

对变换参数 (x, y, θ) 进行线性化：

```
e_i(ξ + Δξ) ≈ e_i(ξ) + J_i · Δξ
```

其中雅可比 J_i：

```
J_i = [∂e_i/∂x, ∂e_i/∂y, ∂e_i/∂θ]
    = [n_i_x, n_i_y, n_i · (∂R/∂θ · q_i)]
```

Gauss-Newton更新：

```
Δξ = -(JᵀJ)⁻¹ · Jᵀ · e(ξ)
```

### 11.2.4 PLICP vs ICP对比

```python
import numpy as np
import time

def compare_icp_plicp():
    """对比ICP和PLICP的性能"""
    np.random.seed(42)
    
    # 生成参考点云（结构化环境模拟）
    wall_points = []
    for x in np.linspace(-5, 5, 100):
        wall_points.append([x, -3])  # 水平墙
        wall_points.append([x, 3])   # 水平墙
    for y in np.linspace(-3, 3, 60):
        wall_points.append([-5, y])  # 垂直墙
        wall_points.append([5, y])   # 垂直墙
    target = np.array(wall_points)
    
    # 生成变换后的点云
    true_pose = np.array([0.5, 0.3, 0.15])
    theta = true_pose[2]
    R = np.array([[np.cos(theta), -np.sin(theta)],
                  [np.sin(theta), np.cos(theta)]])
    source = target @ R.T + true_pose[:2]
    source += np.random.randn(*source.shape) * 0.05
    
    # ICP测试
    icp = ICP(max_iterations=50)
    start = time.time()
    (R_icp, t_icp), stats_icp = icp.align(source, target)
    icp_time = time.time() - start
    
    # PLICP测试
    plicp = PointToLineICP(max_iterations=20)
    start = time.time()
    pose_plicp, stats_plicp = plicp.align(source, target)
    plicp_time = time.time() - start
    
    print("=" * 60)
    print(f"{'指标':<20} {'ICP':<18} {'PLICP':<18}")
    print("-" * 60)
    print(f"{'迭代次数':<20} {stats_icp['iterations']:<18} {stats_plicp['iterations']:<18}")
    print(f"{'最终误差':<20} {stats_icp['errors'][-1]:<18.6f} {stats_plicp['errors'][-1]:<18.6f}")
    print(f"{'耗时(ms)':<20} {icp_time*1000:<18.2f} {plicp_time*1000:<18.2f}")
    
    # 位姿误差
    if isinstance(pose_plicp, np.ndarray):
        angle_err = abs(pose_plicp[2] - true_pose[2])
        trans_err = np.linalg.norm(pose_plicp[:2] - true_pose[:2])
        print(f"PLICP角度误差: {np.degrees(angle_err):.4f}°")
        print(f"PLICP平移误差: {trans_err:.4f}m")
```

## 11.3 Gauss-Newton优化方法

### 11.3.1 Gauss-Newton原理

Gauss-Newton法是一种用于非线性最小二乘问题的迭代优化算法，特别适用于扫描匹配。

**问题形式：**
```
min F(x) = Σ f_i(x)² = ||f(x)||²
```

**迭代更新：**
```
x_{k+1} = x_k - (JᵀJ)⁻¹ · Jᵀ · f(x_k)
```

其中 J 为雅可比矩阵，f_i(x) 为残差函数。

### 11.3.2 在扫描匹配中的应用

```python
import numpy as np

class GaussNewtonScanMatcher:
    """Gauss-Newton扫描匹配器"""
    def __init__(self, max_iterations: int = 10, convergence_delta: float = 1e-4):
        self.max_iterations = max_iterations
        self.convergence_delta = convergence_delta
        
    def compute_residuals(self, points: np.ndarray, pose: np.ndarray,
                           map_values, map_gradients) -> np.ndarray:
        """
        计算残差: 1 - M(S_i(T))
        
        M为栅格地图，返回占据概率值
        """
        theta = pose[2]
        R = np.array([[np.cos(theta), -np.sin(theta)],
                      [np.sin(theta), np.cos(theta)]])
        transformed = points @ R.T + pose[:2]
        
        residuals = np.zeros(len(points))
        for i, pt in enumerate(transformed):
            # 双线性插值获取地图值和梯度
            value = self.bilinear_interpolate(map_values, pt)
            residuals[i] = 1.0 - value
            
        return residuals
    
    def compute_jacobian(self, points: np.ndarray, pose: np.ndarray,
                          map_gradients) -> np.ndarray:
        """
        计算残差对位姿参数的雅可比矩阵
        """
        theta = pose[2]
        R = np.array([[np.cos(theta), -np.sin(theta)],
                      [np.sin(theta), np.cos(theta)]])
        R_theta = np.array([[-np.sin(theta), -np.cos(theta)],
                           [np.cos(theta), -np.sin(theta)]])
        
        J = np.zeros((len(points), 3))
        for i, q in enumerate(points):
            # 地图梯度（通过双线性插值获取）
            transformed = R @ q + pose[:2]
            grad = self.bilinear_interpolate_gradient(map_gradients, transformed)
            
            # 雅可比: ∂(1-M)/∂(x,y,θ)
            J[i, :2] = -grad
            J[i, 2] = -grad @ (R_theta @ q)
            
        return J
    
    def optimize(self, points: np.ndarray, initial_pose: np.ndarray,
                  map_values, map_gradients) -> np.ndarray:
        """
        执行Gauss-Newton优化
        """
        pose = initial_pose.copy()
        
        for iteration in range(self.max_iterations):
            residuals = self.compute_residuals(points, pose, map_values, map_gradients)
            J = self.compute_jacobian(points, pose, map_gradients)
            
            # 求解正规方程
            H = J.T @ J
            b = J.T @ residuals
            
            try:
                delta = np.linalg.solve(H + np.eye(3) * 1e-6, -b)
            except np.linalg.LinAlgError:
                break
                
            pose[:2] += delta[:2]
            pose[2] += delta[2]
            
            if np.linalg.norm(delta) < self.convergence_delta:
                break
                
        return pose
    
    def bilinear_interpolate(self, map_values, point: np.ndarray) -> float:
        """双线性插值获取地图值"""
        x, y = point
        x0, y0 = int(np.floor(x)), int(np.floor(y))
        x1, y1 = x0 + 1, y0 + 1
        
        if not (0 <= x0 < map_values.shape[1] and 
                0 <= y0 < map_values.shape[0] and
                0 <= x1 < map_values.shape[1] and 
                0 <= y1 < map_values.shape[0]):
            return 0.5  # 地图外返回未知概率
            
        dx = x - x0
        dy = y - y0
        
        v00 = map_values[y0, x0]
        v10 = map_values[y0, x1]
        v01 = map_values[y1, x0]
        v11 = map_values[y1, x1]
        
        return (v00 * (1-dx) * (1-dy) + 
                v10 * dx * (1-dy) +
                v01 * (1-dx) * dy + 
                v11 * dx * dy)
    
    def bilinear_interpolate_gradient(self, map_gradients, 
                                        point: np.ndarray) -> np.ndarray:
        """双线性插值获取梯度"""
        x, y = point
        x0, y0 = int(np.floor(x)), int(np.floor(y))
        x1, y1 = x0 + 1, y0 + 1
        
        if not (0 <= x0 < map_gradients.shape[1] and 
                0 <= y0 < map_gradients.shape[0]):
            return np.zeros(2)
            
        dx = x - x0
        dy = y - y0
        
        g00 = map_gradients[y0, x0]
        g10 = map_gradients[y0, x1]
        g01 = map_gradients[y1, x0]
        g11 = map_gradients[y1, x1]
        
        return (g00 * (1-dx) * (1-dy) + 
                g10 * dx * (1-dy) +
                g01 * (1-dx) * dy + 
                g11 * dx * dy)
```

### 11.3.3 Gauss-Newton vs 其他优化方法

| 方法 | 收敛速度 | 实现难度 | 需二阶导 | 适用场景 |
|------|---------|---------|---------|---------|
| 梯度下降 | 慢 | 易 | 否 | 远离最优值 |
| Gauss-Newton | 快 | 中 | 否 | 残差较小时 |
| Levenberg-Marquardt | 快 | 中 | 否 | 通用推荐 |
| 牛顿法 | 最快 | 难 | 是 | 小规模问题 |

## 11.4 ROS2中的扫描匹配应用

### 11.4.1 slam_toolbox中的扫描匹配

slam_toolbox内部使用Ceres Solver进行扫描匹配优化：

```yaml
# slam_toolbox参数中的扫描匹配配置
slam_toolbox:
  ros__parameters:
    solver_plugin: solver_plugins::CeresSolver
    ceres_scan_matcher:
      ceres_scan_matcher:
        translation_weight: 10.0      # 平移权重
        rotation_weight: 40.0         # 旋转权重
        occupied_space_weight: 20.0   # 占据空间权重
        ceres_loss_function: None     # 损失函数
    minimum_time_interval: 0.5        # 最小帧间隔
    minimum_travel_distance: 0.3      # 最小移动距离触发放射匹配
    resolution: 0.05                  # 地图分辨率
    max_laser_range: 8.0             # 最大激光范围
```

### 11.4.2 ROS2扫描匹配节点示例

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np

class ScanMatcherNode(Node):
    """ROS2扫描匹配节点"""
    def __init__(self):
        super().__init__('scan_matcher')
        
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
            
        self.latest_scan = None
        self.reference_scan = None
        self.odom_pose = np.zeros(3)
        
        self.icp = PointToLineICP()
        self.scan_count = 0
        
        # 触发扫描匹配的定时器
        self.timer = self.create_timer(0.5, self.do_scan_matching)
        
    def scan_callback(self, msg: LaserScan):
        """处理激光扫描数据"""
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        
        # 过滤无效点
        valid = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < msg.range_max)
        
        # 转换为笛卡尔坐标
        x = ranges[valid] * np.cos(angles[valid])
        y = ranges[valid] * np.sin(angles[valid])
        
        self.latest_scan = np.column_stack((x, y))
        self.scan_count += 1
        
        if self.reference_scan is None and self.scan_count > 5:
            self.reference_scan = self.latest_scan.copy()
            self.get_logger().info(f"参考扫描已初始化: {len(self.reference_scan)}点")
            
    def odom_callback(self, msg: Odometry):
        """更新里程计位姿"""
        pose = msg.pose.pose
        self.odom_pose[0] = pose.position.x
        self.odom_pose[1] = pose.position.y
        # 四元数转欧拉角
        q = pose.orientation
        self.odom_pose[2] = np.arctan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y**2 + q.z**2)
        )
        
    def do_scan_matching(self):
        """执行扫描匹配"""
        if self.latest_scan is None or self.reference_scan is None:
            return
            
        # 使用里程计作为初始猜测
        init_guess = self.odom_pose.copy()
        
        try:
            # PLICP扫描匹配
            matched_pose, stats = self.icp.align(
                self.latest_scan, self.reference_scan, init_guess
            )
            
            self.get_logger().info(
                f"扫描匹配: 位姿=({matched_pose[0]:.3f}, {matched_pose[1]:.3f}, "
                f"{np.degrees(matched_pose[2]):.1f}°) "
                f"误差={stats['errors'][-1]:.6f} "
                f"迭代={stats['iterations']}"
            )
            
            # 更新参考帧（滑动窗口）
            if stats['errors'][-1] < 0.01:
                self.reference_scan = self.latest_scan.copy()
                
        except Exception as e:
            self.get_logger().error(f"扫描匹配失败: {e}")
            
    def point_cloud_to_scan(self, points: np.ndarray, 
                              frame_id: str = 'laser') -> LaserScan:
        """将点云转换为LaserScan消息"""
        msg = LaserScan()
        msg.header.frame_id = frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.angle_min = -np.pi
        msg.angle_max = np.pi
        msg.angle_increment = 2 * np.pi / len(points)
        msg.range_min = 0.05
        msg.range_max = 30.0
        msg.ranges = [np.linalg.norm(p) for p in points]
        return msg
```

### 11.4.3 扫描匹配性能评估

```bash
# 使用rosbag测试扫描匹配
ros2 bag play slam_data_bag --loop

# 查看扫描匹配输出
ros2 topic echo /scan_matcher/matched_pose

# 评估配准精度
ros2 run tf2_tools view_frames.py
```

### 11.4.4 扫描匹配调优建议

1. **点云预处理：**
   - 体素滤波降采样（voxel_filter_size=0.05）
   - 去除离群点（半径滤波）
   - 提取特征点（角点、边缘点）

2. **初始位姿估计：**
   - 使用里程计作为初值
   - 匀速运动模型外推
   - IMU预积分辅助

3. **匹配参数调整：**
   ```
   场景类型      → 平移权重   → 旋转权重
   结构化走廊   → 10.0      → 40.0
   开阔广场     → 20.0      → 20.0
   狭窄通道     → 5.0       → 60.0
   ```

4. **收敛条件：**
   - 最大迭代次数：10-20次
   - 误差变化阈值：1e-4 - 1e-6
   - 变换增量阈值：0.001m / 0.001rad

## 11.5 实际案例分析

### 11.5.1 走廊环境扫描匹配

走廊环境是扫描匹配的典型场景，具有明显的直线特征：

```python
def corridor_matching_example():
    """走廊环境扫描匹配示例"""
    # 模拟走廊点云（两条平行线）
    corridor_width = 2.0
    corridor_length = 10.0
    
    target = []
    for x in np.linspace(-corridor_length/2, corridor_length/2, 200):
        target.append([x, -corridor_width/2])  # 左墙
        target.append([x, corridor_width/2])    # 右墙
    target = np.array(target)
    
    # 模拟机器人移动后的扫描
    displacement = np.array([0.5, 0.0, 0.05])
    theta = displacement[2]
    R = np.array([[np.cos(theta), -np.sin(theta)],
                  [np.sin(theta), np.cos(theta)]])
    source = target @ R.T + displacement[:2]
    source += np.random.randn(*source.shape) * 0.02
    
    # ICP和PLICP对比
    icp = ICP(max_iterations=30)
    plicp = PointToLineICP(max_iterations=15)
    
    (R_icp, t_icp), stats_icp = icp.align(source, target)
    pose_plicp, stats_plicp = plicp.align(source, target)
    
    print("走廊环境扫描匹配结果:")
    print(f"  ICP 误差: {stats_icp['errors'][-1]:.6f} ({stats_icp['iterations']}次)")
    print(f"  PLICP误差: {stats_plicp['errors'][-1]:.6f} ({stats_plicp['iterations']}次)")
    
    # 在走廊环境中PLICP通常比ICP快2-3倍收敛
```

### 11.5.2 动态环境中的鲁棒匹配

```python
import numpy as np
from scipy.spatial import KDTree

class RobustScanMatcher:
    """鲁棒扫描匹配 - 使用M估计抑制动态障碍物"""
    def __init__(self, max_iterations: int = 30):
        self.max_iterations = max_iterations
        
    def align(self, source: np.ndarray, target: np.ndarray,
              initial_pose: np.ndarray = None) -> np.ndarray:
        """执行鲁棒扫描匹配"""
        pose = initial_pose.copy() if initial_pose is not None else np.zeros(3)
        tree = KDTree(target)
        
        for iteration in range(self.max_iterations):
            theta = pose[2]
            R = np.array([[np.cos(theta), -np.sin(theta)],
                          [np.sin(theta), np.cos(theta)]])
            transformed = source @ R.T + pose[:2]
            
            distances, indices = tree.query(transformed)
            
            # Huber损失函数：抑制大误差（动态障碍物）
            weights = self.huber_weights(distances, delta=0.5)
            
            # 加权最小二乘
            matched = target[indices]
            R_new, t_new = self._weighted_least_squares(
                source, matched, weights
            )
            
            pose[:2] = R_new @ pose[:2] + t_new
            pose[2] += np.arctan2(
                R_new[1, 0] * np.cos(pose[2]) + R_new[1, 1] * np.sin(pose[2]),
                R_new[0, 0] * np.cos(pose[2]) - R_new[0, 1] * np.sin(pose[2])
            )
            
        return pose
    
    def huber_weights(self, residuals: np.ndarray, delta: float = 1.0) -> np.ndarray:
        """Huber损失函数权重"""
        weights = np.ones_like(residuals)
        outlier = residuals > delta
        weights[outlier] = delta / residuals[outlier]
        return weights
    
    def _weighted_least_squares(self, source: np.ndarray, 
                                  target: np.ndarray,
                                  weights: np.ndarray) -> tuple:
        """加权最小二乘求解变换"""
        W = np.diag(weights)
        mu_s = np.sum(weights[:, np.newaxis] * source, axis=0) / np.sum(weights)
        mu_t = np.sum(weights[:, np.newaxis] * target, axis=0) / np.sum(weights)
        
        source_centered = source - mu_s
        target_centered = target - mu_t
        
        W_source = W @ source_centered
        H = source_centered.T @ W_source
        
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        t = mu_t - R @ mu_s
        
        return R, t
```

## 课后练习

1. **推导题:** 推导ICP算法中基于SVD的最优变换求解过程，说明为什么SVD分解可以求得旋转矩阵的最优解。

2. **编程题:** 使用Python实现一个完整的ICP算法，能够处理二维点云的配准，支持KDTree加速最近邻搜索，并绘制配准前后对比图。

3. **分析题:** 比较ICP和PLICP的异同点，从目标函数、收敛速度、对噪声的鲁棒性三个方面进行分析。

4. **推导题:** 推导PLICP中Gauss-Newton优化的雅可比矩阵，给出完整的数学推导过程。

5. **配置题:** 在slam_toolbox中调整扫描匹配参数（平移权重、旋转权重、最小移动距离等），说明不同参数对建图质量的影响。

6. **设计题:** 设计一个融合里程计和激光扫描匹配的位姿估计系统，说明在何种情况下应该更信任里程计，在何种情况下应该更信任扫描匹配。
