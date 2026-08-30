# 第13章 gmapping粒子滤波SLAM

> **课程**：ROS2 Python 编程  
> **章节**：第13章  
> **课时**：2 课时（90 分钟）  
> **教学方式**：讲授 + 演示  

---

## 学习目标

本章学习目标包括：理解粒子滤波SLAM的基本原理和Rao-Blackwellized分解，掌握FastSLAM框架的算法流程，熟悉gmapping的提议分布优化和自适应重采样策略，能够在ROS2中使用slam_toolbox进行建图，掌握建图参数调优方法。

## 13.1 粒子滤波SLAM原理

### 13.1.1 粒子滤波基本概念

粒子滤波 (Particle Filter) 是一种基于蒙特卡洛方法的贝叶斯滤波实现。它使用一组加权粒子来近似后验概率分布。

**粒子的定义：**每个粒子表示一个假设的机器人轨迹；粒子包含位姿 `x_t` 和权重 `w_t`；所有粒子共同近似后验分布 `p(x_t | z_{1:t}, u_{1:t-1})`。

**粒子滤波的优势：**可以表示任意复杂的概率分布（非高斯、多模态），易于实现、不需要线性化，计算复杂度可控（通过粒子数平衡精度和效率）。

**粒子滤波的三大步骤：**
```
1. 预测 (Prediction):   x_t ~ p(x_t | x_{t-1}, u_t)
2. 更新 (Update):       w_t = w_{t-1} · p(z_t | x_t)
3. 重采样 (Resampling): 根据权重重新采样粒子
```

### 13.1.2 粒子滤波的SLAM应用

在SLAM中，状态空间包含机器人的完整路径和环境地图，维度极高。直接使用粒子滤波估计完整SLAM后验是不现实的。

**解决方案：Rao-Blackwellized Particle Filter (RBPF)**

RBPF利用条件独立性将SLAM分解为两个子问题：

```
p(x_{1:t}, m | z_{1:t}, u_{1:t-1}) = 
p(m | x_{1:t}, z_{1:t}) · p(x_{1:t} | z_{1:t}, u_{1:t-1})
```

分解含义：使用粒子滤波估计机器人路径 `x_{1:t}`，每个粒子根据其估计的路径独立维护一个地图 `m`，给定路径后地图估计可以解析计算（条件高斯）。

```python
import numpy as np
from collections import defaultdict

class RBPFSLAMParticle:
    """Rao-Blackwellized粒子滤波SLAM的单个粒子"""
    def __init__(self, map_size: tuple, resolution: float):
        # 机器人轨迹
        self.trajectory = []
        self.pose = np.zeros(3)  # x, y, theta
        self.weight = 1.0
        
        # 每个粒子维护独立的地图
        self.map = np.zeros(map_size)  # log-odds地图
        self.resolution = resolution
        self.map_size = map_size
        
    def predict(self, control: np.ndarray, noise: np.ndarray):
        """粒子位姿预测"""
        v, omega = control
        theta = self.pose[2]
        dt = 0.1
        
        # 速度运动模型 + 噪声
        self.pose[0] += v * np.cos(theta) * dt + np.random.normal(0, noise[0])
        self.pose[1] += v * np.sin(theta) * dt + np.random.normal(0, noise[1])
        self.pose[2] += omega * dt + np.random.normal(0, noise[2])
        self.pose[2] = np.arctan2(np.sin(self.pose[2]), np.cos(self.pose[2]))
        
        self.trajectory.append(self.pose.copy())
        
    def update_map(self, scan_points: np.ndarray):
        """使用观测更新粒子维护的地图"""
        # Bresenham线段更新地图
        for point in scan_points:
            wx = self.pose[0] + point[0] * np.cos(self.pose[2]) - point[1] * np.sin(self.pose[2])
            wy = self.pose[1] + point[0] * np.sin(self.pose[2]) + point[1] * np.cos(self.pose[2])
            
            gx = int(wx / self.resolution)
            gy = int(wy / self.resolution)
            
            if 0 <= gx < self.map_size[1] and 0 <= gy < self.map_size[0]:
                self.map[gy, gx] += 1.0  # log-odds更新
                
    def compute_likelihood(self, scan_points: np.ndarray, 
                            sensor_noise: float = 0.1) -> float:
        """计算观测似然"""
        likelihood = 1.0
        for point in scan_points:
            wx = self.pose[0] + point[0] * np.cos(self.pose[2]) - point[1] * np.sin(self.pose[2])
            wy = self.pose[1] + point[0] * np.sin(self.pose[2]) + point[1] * np.cos(self.pose[2])
            
            gx = int(wx / self.resolution)
            gy = int(wy / self.resolution)
            
            if 0 <= gx < self.map_size[1] and 0 <= gy < self.map_size[0]:
                map_value = self.map[gy, gx]
                prob = 1.0 / (1.0 + np.exp(-map_value))
                likelihood *= prob
                
        return likelihood
```

### 13.1.3 粒子退化问题

粒子滤波面临的核心挑战是**粒子退化**：经过若干次迭代后，大部分粒子的权重会变得极小，只有少数粒子具有显著的权重。

**退化现象的度量：**

```python
def compute_effective_sample_size(weights: np.ndarray) -> float:
    """计算有效样本数 Neff"""
    total = np.sum(weights)
    if total == 0:
        return 0
    normalized = weights / total
    neff = 1.0 / np.sum(normalized ** 2)
    return neff


def detect_particle_depletion(particles: list) -> bool:
    """检测粒子退化"""
    weights = np.array([p.weight for p in particles])
    neff = compute_effective_sample_size(weights)
    n = len(particles)
    
    depletion_ratio = neff / n
    print(f"有效粒子比: {depletion_ratio:.3f} (Neff={neff:.1f}/{n})")
    
    # 当neff < n/2时，认为发生了退化
    return depletion_ratio < 0.5
```

**解决退化的方法：重采样**

### 13.1.4 官方要点——RBPF 的理论根基与 Grisetti 2007 年关键论文

本章 13.1.2 的 Rao-Blackwellized 分解对应的是 Grisetti、Stachniss 与 Burgard 于 2007 年发表的经典论文《Improved Techniques for Grid Mapping with Rao-Blackwellized Particle Filters》（TER 期刊）及其 2005 年 ICRA 会议版本。论文提出的三项改进正是本章 13.3 的三板斧：①用激光似然优化提议分布（13.3.2，只在观测信息量大时用激光修正采样，否则退回里程计模型）；②自适应重采样（13.3.3，用有效样本数 Neff 阈值替代每帧重采样，避免粒子多样性骤减）；③选择性扫描匹配（仅当移动/旋转超过阈值才执行）。理解本章代码后重读该论文的推导，是深入粒子滤波 SLAM 最直接的一条路径。

## 13.2 FastSLAM框架

### 13.2.1 FastSLAM 1.0

FastSLAM 1.0 是最早将RBPF应用于SLAM的算法：

```
FastSLAM 1.0 算法流程:

每接收到一组观测和控制:
1. 对每个粒子:
   a. 根据运动模型采样新位姿
   b. 计算观测似然（权重）
2. 归一化粒子权重
3. 根据权重重采样
4. 对每个粒子更新EKF路标点观测
```

```python
import numpy as np

class FastSLAM1_0:
    """FastSLAM 1.0 实现"""
    def __init__(self, num_particles: int = 100, 
                 num_landmarks: int = 100):
        self.num_particles = num_particles
        self.particles = []
        
        for _ in range(num_particles):
            particle = {
                'pose': np.zeros(3),
                'weight': 1.0 / num_particles,
                'landmarks': {
                    'mean': np.zeros((num_landmarks, 2)),
                    'cov': np.zeros((num_landmarks, 2, 2)),
                    'seen': np.zeros(num_landmarks, dtype=bool)
                }
            }
            self.particles.append(particle)
            
    def predict(self, control: np.ndarray, noise: np.ndarray):
        """运动模型预测 - 从提议分布采样"""
        for particle in self.particles:
            v, omega = control
            theta = particle['pose'][2]
            dt = 0.1
            
            # 速度运动模型
            particle['pose'][0] += (v * np.cos(theta) * dt + 
                                     np.random.normal(0, noise[0]))
            particle['pose'][1] += (v * np.sin(theta) * dt + 
                                     np.random.normal(0, noise[1]))
            particle['pose'][2] += (omega * dt + 
                                     np.random.normal(0, noise[2]))
            
    def update_weights(self, observation: np.ndarray, 
                        landmark_id: int,
                        sensor_noise: np.ndarray):
        """根据观测更新粒子权重"""
        for particle in self.particles:
            # 计算观测似然
            likelihood = self._compute_likelihood(
                particle, observation, landmark_id, sensor_noise
            )
            particle['weight'] *= likelihood
            
        # 归一化
        weights = np.array([p['weight'] for p in self.particles])
        weights /= weights.sum()
        for i, particle in enumerate(self.particles):
            particle['weight'] = weights[i]
            
    def resample(self):
        """低方差重采样"""
        num = self.num_particles
        weights = np.array([p['weight'] for p in self.particles])
        
        # 系统重采样
        positions = (np.arange(num) + np.random.random()) / num
        cumsum = np.cumsum(weights)
        cumsum[-1] = 1.0  # 避免浮点误差
        
        new_particles = []
        idx = 0
        for pos in positions:
            while cumsum[idx] < pos:
                idx += 1
            new_particle = {
                'pose': self.particles[idx]['pose'].copy(),
                'weight': 1.0 / num,
                'landmarks': {
                    'mean': self.particles[idx]['landmarks']['mean'].copy(),
                    'cov': self.particles[idx]['landmarks']['cov'].copy(),
                    'seen': self.particles[idx]['landmarks']['seen'].copy()
                }
            }
            new_particles.append(new_particle)
            
        self.particles = new_particles
        
    def _compute_likelihood(self, particle: dict, observation: np.ndarray,
                             landmark_id: int, 
                             sensor_noise: np.ndarray) -> float:
        """计算观测似然"""
        lm = particle['landmarks']
        if not lm['seen'][landmark_id]:
            return 1.0  # 新路标点，似然为1
            
        robot_pose = particle['pose']
        lm_mean = lm['mean'][landmark_id]
        lm_cov = lm['cov'][landmark_id]
        
        # 预测观测
        dx = lm_mean[0] - robot_pose[0]
        dy = lm_mean[1] - robot_pose[1]
        predicted_range = np.sqrt(dx**2 + dy**2)
        predicted_bearing = np.arctan2(dy, dx) - robot_pose[2]
        
        # 观测残差
        innovation = np.array([
            observation[0] - predicted_range,
            observation[1] - predicted_bearing
        ])
        innovation[1] = np.arctan2(np.sin(innovation[1]), 
                                    np.cos(innovation[1]))
        
        # 协方差
        S = lm_cov + np.diag(sensor_noise)
        
        # 多元高斯似然
        dim = 2
        det = np.linalg.det(S)
        if det <= 0:
            return 0.001
            
        likelihood = (1.0 / np.sqrt((2*np.pi)**dim * det) * 
                      np.exp(-0.5 * innovation @ np.linalg.inv(S) @ innovation))
        
        return max(likelihood, 0.001)
```

### 13.2.2 FastSLAM 2.0

FastSLAM 2.0 的关键改进是：使用激光观测优化提议分布，而不仅仅依赖运动模型。

**改进点：**提议分布融合当前观测信息，直接在提议分布中融入激光观测，使采样更集中到高概率区域：

  ```
  x_t ~ p(x_t | x_{t-1}, u_t, z_t, m)
  ```

```python
class FastSLAM2_0:
    """FastSLAM 2.0 - 带优化提议分布"""
    def __init__(self, num_particles: int = 100):
        self.num_particles = num_particles
        self.particles = []
        self._init_particles()
        
    def _init_particles(self):
        for _ in range(self.num_particles):
            self.particles.append({
                'pose': np.zeros(3),
                'weight': 1.0 / self.num_particles,
                'map': np.zeros((100, 100)),
                'last_scan': None
            })
            
    def improved_proposal_distribution(self, particle: dict,
                                         control: np.ndarray,
                                         scan: np.ndarray,
                                         noise: np.ndarray) -> np.ndarray:
        """
        优化提议分布：融合激光观测的运动模型采样
        
        FastSLAM 2.0的核心改进：
        当激光观测具有高信息量时，使用激光优化位姿
        当激光观测信息量低时，使用标准运动模型
        """
        # 尝试多个候选位姿
        num_candidates = 30
        candidates = []
        weights = []
        
        for _ in range(num_candidates):
            # 从运动模型采样
            v, omega = control
            theta = particle['pose'][2]
            
            candidate = particle['pose'].copy()
            candidate[0] += (v * np.cos(theta) * 0.1 + 
                             np.random.normal(0, noise[0]))
            candidate[1] += (v * np.sin(theta) * 0.1 + 
                             np.random.normal(0, noise[1]))
            candidate[2] += (omega * 0.1 + 
                             np.random.normal(0, noise[2]))
            
            # 计算每个候选的激光匹配得分
            score = self._scan_match_score(candidate, scan, 
                                            particle['map'])
            candidates.append(candidate)
            weights.append(score)
            
        weights = np.array(weights)
        weights = np.exp(weights - np.max(weights))  # 数值稳定
        weights /= weights.sum()
        
        # 根据权重采样
        idx = np.random.choice(num_candidates, p=weights)
        return candidates[idx]
    
    def _scan_match_score(self, pose: np.ndarray, 
                           scan: np.ndarray, 
                           grid_map: np.ndarray) -> float:
        """计算激光扫描匹配得分"""
        theta = pose[2]
        c, s = np.cos(theta), np.sin(theta)
        
        total_score = 0.0
        for point in scan:
            wx = pose[0] + point[0] * c - point[1] * s
            wy = pose[1] + point[0] * s + point[1] * c
            
            gx = int(wx / 0.05)
            gy = int(wy / 0.05)
            
            if 0 <= gx < grid_map.shape[1] and 0 <= gy < grid_map.shape[0]:
                prob = 1.0 / (1.0 + np.exp(-grid_map[gy, gx]))
                total_score += np.log(max(prob, 0.01))
                
        return total_score / len(scan) if len(scan) > 0 else -np.inf
```

### 13.2.3 FastSLAM vs EKF-SLAM

| 特性 | FastSLAM (粒子滤波) | EKF-SLAM |
|------|-------------------|-----------|
| 状态表示 | 离散粒子群 | 高斯分布 |
| 非高斯分布 | 支持 | 不支持 |
| 数据关联 | 每个粒子独立处理 | 需要全局关联 |
| 计算复杂度 | O(M·N) M粒子数,N路标数 | O(N²) |
| 实现难度 | 中等 | 简单 |
| 应用场景 | 大场景、非高斯  | 小场景、高斯噪声 |

### 13.2.4 官方要点——粒子滤波的工程代价与替代路线

粒子滤波的三重代价在 galactic 之后的文献与 The Construct 课程中被反复讨论：粒子多样性丧失（重采样后粒子快速趋同，对应 13.1.3 的退化度量 Neff）、计算量随粒子数线性增长、以及对里程计漂移的敏感。RBPF 在里程计差时会让提议分布严重偏离，粒子被迫用数量弥补——这正是本章 13.5.1 建议「噪声大→加粒子」的原因。

作为替代，图优化 SLAM（slam_toolbox 的 pose graph + Ceres、Cartographer 的前端配准+后端优化）把「一次解算近似」换成「最小二乘再求解」，不依赖粒子数量，劣势是把漂移责任转移给回环检测与参数调优。Robotics Back-End 的对比文章给出结论性建议：中小场景、REAL 低噪声里程计上 RBPF 已够用；场地大、要求回环修正时直接上 slam_toolbox（本章 13.4 的实践路线）。

## 13.3 gmapping实现详解

### 13.3.1 gmapping算法特点

gmapping 是基于RBPF的2D SLAM算法，在FastSLAM基础上做了重要改进：

**核心改进：**使用激光观测的似然来优化粒子的提议分布，使粒子集中在高概率区域；仅在必要时进行自适应重采样，减少粒子退化；只对关键帧进行选择性扫描匹配，提高效率。

**gmapping的算法流程：**
```
1. 读取激光扫描数据
2. 根据运动模型预测粒子位姿
3. 计算每个粒子的权重（观测似然）
4. 使用优化提议分布重新采样
5. 更新每个粒子的占据栅格地图
6. 自适应重采样决策
7. 发布建图结果
```

### 13.3.2 gmapping的提议分布优化

**标准粒子滤波的问题：** 提议分布仅基于运动模型，当运动模型噪声大而观测噪声小时，采样效率低。

**gmapping的改进：** 使用激光观测来"修正"提议分布，在观测似然尖锐时使用激光数据。

```python
class GmappingProposalDistribution:
    """gmapping优化提议分布"""
    def __init__(self, scan_matcher=None):
        self.scan_matcher = scan_matcher or HectorScanMatcher()
        
    def compute_optimized_proposal(self, particle_pose: np.ndarray,
                                     odom: np.ndarray,
                                     scan: np.ndarray,
                                     map_data: np.ndarray,
                                     map_resolution: float) -> tuple:
        """
        计算优化提议分布的均值和协方差
        
        参数:
            particle_pose: 当前粒子的位姿
            odom: 里程计测量
            scan: 激光扫描点云
            map_data: 地图数据
            
        返回:
            (mean, cov): 优化提议分布的均值和协方差
        """
        # 使用扫描匹配获取激光观测的似然分布
        try:
            optimized_pose = self.scan_matcher.match(
                scan, map_data, map_resolution, particle_pose
            )
        except:
            # 匹配失败，退回使用里程计
            return particle_pose, np.diag([0.5, 0.5, 0.3])
        
        # 计算激光观测的不确定性
        delta = optimized_pose - particle_pose
        uncertainty = np.outer(delta, delta) + np.eye(3) * 0.01
        
        # 融合里程计和激光观测
        odom_cov = np.diag([0.02, 0.02, 0.01])
        laser_cov = uncertainty
        
        # 信息融合
        info = np.linalg.inv(odom_cov) + np.linalg.inv(laser_cov)
        cov = np.linalg.inv(info)
        mean = cov @ (np.linalg.inv(odom_cov) @ particle_pose + 
                       np.linalg.inv(laser_cov) @ optimized_pose)
        
        return mean, cov
    
    def sample_from_proposal(self, mean: np.ndarray, 
                               cov: np.ndarray) -> np.ndarray:
        """从优化提议分布采样"""
        return np.random.multivariate_normal(mean, cov)
```

### 13.3.3 自适应重采样

gmapping使用**Neff（有效样本数）**来判断是否需要进行重采样：

```python
class AdaptiveResampler:
    """gmapping自适应重采样"""
    def __init__(self, threshold_ratio: float = 0.5):
        self.threshold_ratio = threshold_ratio
        self.resample_count = 0
        
    def should_resample(self, particles: list) -> bool:
        """
        判断是否需要重采样
        
        只有当Neff < N/2时才执行重采样
        这样可以减少粒子退化，同时避免不必要的重采样
        """
        weights = np.array([p.weight for p in particles])
        weights_sum = np.sum(weights)
        if weights_sum == 0:
            return True
            
        normalized = weights / weights_sum
        neff = 1.0 / np.sum(normalized ** 2)
        n = len(particles)
        
        self.neff = neff
        need = neff < n * self.threshold_ratio
        
        if need:
            self.resample_count += 1
            
        return need
    
    def stratified_resample(self, particles: list) -> list:
        """分层重采样"""
        n = len(particles)
        weights = np.array([p.weight for p in particles])
        weights /= weights.sum()
        
        # 分层采样
        new_particles = []
        for i in range(n):
            u = (i + np.random.random()) / n
            cumsum = 0
            for j, w in enumerate(weights):
                cumsum += w
                if cumsum >= u:
                    new_particle = {
                        'pose': particles[j]['pose'].copy(),
                        'weight': 1.0 / n,
                        'map': particles[j]['map'].copy()
                    }
                    new_particles.append(new_particle)
                    break
                    
        return new_particles
    
    def get_resample_stats(self) -> dict:
        """获取重采样统计信息"""
        return {
            'resample_count': self.resample_count,
            'current_neff': getattr(self, 'neff', 0)
        }
```

### 13.3.4 完整gmapping实现

```python
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import tf2_ros

class GmappingSLAM:
    """gmapping SLAM核心算法"""
    def __init__(self, num_particles: int = 30, 
                 map_size: tuple = (200, 200),
                 resolution: float = 0.05):
        self.num_particles = num_particles
        self.resolution = resolution
        self.map_size = map_size
        
        # 初始化粒子
        self.particles = []
        for _ in range(num_particles):
            self.particles.append({
                'pose': np.zeros(3),
                'weight': 1.0 / num_particles,
                'map': np.zeros(map_size),  # log-odds
                'trajectory': []
            })
            
        self.resampler = AdaptiveResampler()
        self.proposal = GmappingProposalDistribution()
        
        # 最佳粒子的地图
        self.best_map = np.zeros(map_size)
        self.map_updated = False
        
    def process_scan(self, scan_points: np.ndarray, 
                      odometry: np.ndarray):
        """处理一帧激光扫描"""
        # 步骤1: 对每个粒子计算优化提议分布并采样
        for i, particle in enumerate(self.particles):
            # 计算提议分布
            mean, cov = self.proposal.compute_optimized_proposal(
                particle['pose'], odometry, scan_points,
                particle['map'], self.resolution
            )
            
            # 从提议分布采样新位姿
            new_pose = self.proposal.sample_from_proposal(mean, cov)
            particle['pose'] = new_pose
            particle['trajectory'].append(new_pose.copy())
            
            # 步骤2: 计算权重（观测似然）
            likelihood = self._compute_scan_likelihood(
                new_pose, scan_points, particle['map']
            )
            particle['weight'] *= (likelihood + 1e-10)
            
            # 步骤3: 更新地图
            self._update_particle_map(particle, scan_points)
            
        # 步骤4: 归一化权重
        weights = np.array([p['weight'] for p in self.particles])
        total = weights.sum()
        if total > 0:
            weights /= total
            for i, p in enumerate(self.particles):
                p['weight'] = weights[i]
                
        # 步骤5: 自适应重采样
        if self.resampler.should_resample(self.particles):
            self.particles = self.resampler.stratified_resample(
                self.particles
            )
            
        # 步骤6: 更新最佳地图（权重最高的粒子的地图）
        best_idx = np.argmax([p['weight'] for p in self.particles])
        self.best_map = self.particles[best_idx]['map'].copy()
        self.map_updated = True
        
    def _compute_scan_likelihood(self, pose: np.ndarray,
                                   scan: np.ndarray,
                                   grid_map: np.ndarray) -> float:
        """计算激光扫描的似然"""
        theta = pose[2]
        c, s = np.cos(theta), np.sin(theta)
        
        likelihood = 0.0
        count = 0
        
        for point in scan:
            wx = pose[0] + point[0] * c - point[1] * s
            wy = pose[1] + point[0] * s + point[1] * c
            
            gx = int(wx / self.resolution)
            gy = int(wy / self.resolution)
            
            if 0 <= gx < self.map_size[1] and 0 <= gy < self.map_size[0]:
                prob = 1.0 / (1.0 + np.exp(-grid_map[gy, gx]))
                # 占据区域的似然更高
                match = 1.0 - abs(prob - 1.0)
                likelihood += np.log(max(match, 0.01))
                count += 1
                
        return np.exp(likelihood / max(count, 1))
    
    def _update_particle_map(self, particle: dict, scan: np.ndarray):
        """更新粒子的占据栅格地图"""
        pose = particle['pose']
        theta = pose[2]
        c, s = np.cos(theta), np.sin(theta)
        
        rx = int(pose[0] / self.resolution)
        ry = int(pose[1] / self.resolution)
        
        for point in scan:
            wx = pose[0] + point[0] * c - point[1] * s
            wy = pose[1] + point[0] * s + point[1] * c
            
            gx = int(wx / self.resolution)
            gy = int(wy / self.resolution)
            
            if 0 <= gx < self.map_size[1] and 0 <= gy < self.map_size[0]:
                if 0 <= rx < self.map_size[1] and 0 <= ry < self.map_size[0]:
                    # 简单更新
                    particle['map'][gy, gx] = min(
                        particle['map'][gy, gx] + 0.5, 5.0
                    )
                    
    def get_occupancy_grid(self) -> np.ndarray:
        """获取占据栅格地图 (0-100)"""
        prob = 1.0 - 1.0 / (1.0 + np.exp(-self.best_map))
        occ_grid = (prob * 100).astype(np.int8)
        occ_grid[prob > 0.7] = 100
        occ_grid[prob < 0.3] = 0
        occ_grid[(prob >= 0.3) & (prob <= 0.7)] = -1
        return occ_grid
```

## 13.4 ROS2中的建图工具

### 13.4.1 slam_toolbox概述

在ROS2中，slam_toolbox替代了ROS1的gmapping，提供了更丰富的功能：

**slam_toolbox支持的工作模式：**`mapping` 为在线建图模式，`localization` 在已有地图上定位，`mapping+localization` 同时建图和定位，`localize_in_partial_map` 支持部分地图定位。

**slam_toolbox的核心参数：**

```yaml
slam_toolbox:
  ros__parameters:
    mode: mapping
    solver_plugin: solver_plugins::CeresSolver
    map_frame: map
    odom_frame: odom
    base_frame: base_footprint
    scan_topic: /scan
    resolution: 0.05
    map_update_interval: 1.0
    minimum_time_interval: 0.2
    transform_timeout: 0.2
    max_laser_range: 8.0
    do_loop_closing: true
    loop_match_minimum_chain_size: 10
    loop_match_maximum_variance_big: 0.003
    correlation_search_space_smear_deviation: 0.03
    minimum_travel_distance: 0.3
    minimum_travel_heading: 0.3
```

### 13.4.2 slam_toolbox参数详解

**建图相关参数：**

| 参数 | 默认值 | 说明 | 调优建议 |
|------|--------|------|---------|
| resolution | 0.05 | 地图分辨率(m) | 大场景0.1，精细0.025 |
| map_update_interval | 1.0 | 地图更新间隔(秒) | 移动快则减小 |
| minimum_time_interval | 0.2 | 最小帧间隔(秒) | 根据激光频率调整 |
| max_laser_range | 8.0 | 最大有效激光距离(m) | 根据激光型号设置 |

**回环检测参数：**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| do_loop_closing | true | 启用回环检测 |
| loop_match_minimum_chain_size | 10 | 最小闭环匹配链大小 |
| loop_match_maximum_variance_big | 0.003 | 闭环匹配方差阈值 |
| correlation_search_space_smear_deviation | 0.03 | 搜索空间涂片偏差 |

**扫描匹配参数：**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| minimum_travel_distance | 0.3 | 触发匹配的最小移动距离(m) |
| minimum_travel_heading | 0.3 | 触发匹配的最小旋转角度(rad) |
| scan_matcher_ceres_initial_scale | 1.0 | Ceres匹配初始尺度 |
| scan_matcher_ceres_scale_step | 1.25 | Ceres匹配尺度步长 |

### 13.4.3 建图操作流程

```bash
# 1. 启动仿真环境
ros2 launch robot_sim_demo gazebo2.launch.py drive:=false

# 2. 启动slam_toolbox建图
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=./config/mapper_params_online_async.yaml \
  use_sim_time:=true

# 3. 控制机器人探索环境
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 4. 保存地图
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map

# 5. 验证地图
# 查看地图文件: my_map.pgm + my_map.yaml
```

### 13.4.4 建图质量控制

```python
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np

class MapQualityMonitor(Node):
    """建图质量监控节点"""
    def __init__(self):
        super().__init__('map_quality_monitor')
        
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
            
        self.map_count = 0
        
    def map_callback(self, msg: OccupancyGrid):
        self.map_count += 1
        if self.map_count % 10 != 0:
            return
            
        data = np.array(msg.data).reshape(
            msg.info.height, msg.info.width
        )
        
        # 计算统计数据
        total_cells = data.size
        free_cells = np.sum(data == 0)
        occupied_cells = np.sum(data == 100)
        unknown_cells = np.sum(data < 0)
        
        coverage = (free_cells + occupied_cells) / total_cells * 100
        
        self.get_logger().info(
            f'建图质量 [#{self.map_count}]: '
            f'覆盖={coverage:.1f}% '
            f'(空闲={free_cells}, 占据={occupied_cells}, '
            f'未知={unknown_cells}) '
            f'分辨率={msg.info.resolution:.3f}m'
        )
        
        # 建图完成判断
        if coverage > 95:
            self.get_logger().info('环境已充分探索，建议保存地图!')
```

### 13.4.5 从gmapping到slam_toolbox

ROS1中的gmapping在ROS2中被slam_toolbox替代，两者的参数映射关系：

| gmapping参数 | slam_toolbox参数 | 说明 |
|-------------|-----------------|------|
| particles | (solver_plugins配置) | 粒子数由求解器管理 |
| iterations | minimum_time_interval | 帧间隔代替迭代次数 |
| srr/srt/stt | (里程计噪声模型) | 噪声模型内置 |
| llsamplerange/llsamplestep | correlation_search_space_* | 搜索空间参数 |
| lsigma | (Ceres Loss Function) | 损失函数参数 |
| ogain | (Ceres权重) | 优化的权重 |
| linearUpdate | minimum_travel_distance | 触发更新的移动距离 |
| angularUpdate | minimum_travel_heading | 触发更新的旋转角度 |

```bash
# ROS1 gmapping启动命令 (对比参考)
rosrun gmapping slam_gmapping scan:=/scan _particles:=30 _linearUpdate:=0.1

# ROS2 slam_toolbox启动命令
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=./config/mapper_params_online_async.yaml
```

### 13.4.6 官方要点——gmapping 的官方文档与参数体系

gmapping 是 ROS 1 时代应用最广的 2D SLAM 包，其 ROS Wiki 页面提供了完整的参数表，与本章 13.4.5 的映射关系一一对应：`particles`（粒子数）、`linearUpdate`/`angularUpdate`（触发扫描匹配的移动/旋转阈值，对应 slam_toolbox 的 `minimum_travel_distance`/`minimum_travel_heading`）、`srr`/`srt`/`str`/`stt`（里程计运动模型噪声）、`lsigma`/`ogain`（激光似然参数）、`llsamplerange`（局部搜索范围）。

openslam.org 的 gmapping 项目页则给出原始实现的说明：它要求输入 `/scan`、里程计与 TF（`map`→`odom`→`base`→`laser`），并强调「粒子数越多建图越稳定，但每帧代价线性增长」——这是本章 13.5.1 粒子数选择公式的理论根据。官方还指出 gmapping 对大场景有地图尺寸限制（内存预分配），这也是 ROS 2 生态最终转向 slam_toolbox 的原因之一。

## 13.5 参数调优与最佳实践

### 13.5.1 粒子数选择

粒子数量直接影响建图质量和计算效率：

```python
def suggest_particle_count(environment_size: float, 
                            sensor_noise: float) -> int:
    """
    根据环境特征推荐粒子数
    
    原则：
    - 环境越大 → 需要越多粒子
    - 传感器噪声越大 → 需要越多粒子
    - 计算资源有限 → 适当减少粒子数
    """
    base_particles = 30
    
    # 环境大小因子
    if environment_size < 100:  # <100m²
        size_factor = 0.5
    elif environment_size < 500:  # <500m²
        size_factor = 1.0
    elif environment_size < 2000:  # <2000m²
        size_factor = 2.0
    else:  # >2000m²
        size_factor = 3.0
        
    # 噪声因子
    noise_factor = 1.0 + sensor_noise * 5
    
    particles = int(base_particles * size_factor * noise_factor)
    return max(20, min(particles, 200))
```

### 13.5.2 建图路径规划

好的建图路径可以显著提高建图质量：

```python
def generate_mapping_path(environment_type: str) -> str:
    """根据环境类型推荐建图路径策略"""
    strategies = {
        'office': """
        办公环境建图策略:
        1. 沿走廊中轴线前进，保证两侧墙壁被扫描到
        2. 在每个房间门口做"回"字形扫描
        3. 避免急转弯，保持匀速
        4. 完成所有房间后，沿外环走一圈巩固闭环
        """,
        'warehouse': """
        仓库环境建图策略:
        1. 沿货架通道穿行，逐行扫描
        2. 每个通道两端做180°旋转覆盖
        3. 注意货架间的狭窄通道
        4. 保持货架两侧都有扫描覆盖
        """,
        'large_hall': """
        大展厅建图策略:
        1. 沿墙壁走一圈建立基本轮廓
        2. 使用"S"形路径覆盖内部空间
        3. 在空旷区域适当提高移动速度
        4. 回到起点的闭环至关重要
        """,
    }
    return strategies.get(environment_type, '通用策略')
```

### 13.5.3 常见问题排查

**问题1：地图重影/偏移**

原因：回环检测未正确触发，或扫描匹配错误。

解决：降低移动速度并手动控制速度（线性 0.15、角速度 0.3），同时减少环境速度相关参数：

  ```bash
  # 降低移动速度
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
  # 手动控制速度：线性0.15，角速度0.3
  
  # 减少环境速度相关的参数
  # 在参数文件中调整
  minimum_travel_distance: 0.15
  minimum_travel_heading: 0.15
  ```

**问题2：地图缺失区域**

原因：未充分探索，激光扫描范围有限。

解决：使用自动探索算法，或手动补充扫描并检查激光范围设置：

  ```bash
  # 使用自动探索算法
  ros2 run exploration exploration_node
  
  # 或手动补充扫描
  # 检查激光范围设置
  max_laser_range: 12.0  # 增大激光有效范围
  ```

**问题3：定位丢失**

原因：快速移动或环境特征不足。

解决：增加角点检测灵敏度：

  ```yaml
  # 增加角点检测灵敏度
  correlation_search_space_smear_deviation: 0.05
  loop_match_maximum_variance_big: 0.005
  ```

### 13.5.4 官方要点——粒子滤波 SLAM 的调优与质量监控纪律

结合官方文档与课程实践，粒子滤波建图的三条纪律值得牢记：第一，录制 bag（本章开头实例的 `/scan`、`/odom`、`/tf`）离线试参，先检查 `ros2 bag info` 确认话题频率与时间戳连续，再谈算法；第二，用本章 13.4.4 的 MapQualityMonitor 思路量化覆盖度（unknown 比例）与重影（同一区域占据栅格离散度），而不是凭肉眼判断；第三，SimTime 环境强调 `use_sim_time:=true` 与时钟对齐，仿真中更容易复现的时间戳错位问题在真机上会因网络延迟放大。建议按本章练习第 4 题系统对比 `resolution`、`minimum_travel_distance` 与回环参数对最终地图的影响，并记录每组参数下的覆盖度数值。

## 课后练习

1. **原理题:** 说明Rao-Blackwellized粒子滤波如何将SLAM问题分解，为什么这种分解可以降低状态空间的维度？

2. **编程题:** 实现一个简化版的gmapping粒子滤波SLAM系统，包含粒子初始化、运动模型预测、权重计算和重采样四个核心步骤。

3. **分析题:** 比较gmapping和slam_toolbox在算法原理上的异同，说明为什么ROS2选择了slam_toolbox而不是继续使用gmapping。

4. **配置题:** 在Gazebo仿真中使用slam_toolbox进行建图，配置参数使得建图质量最优，包括分辨率、更新频率、回环检测等。

5. **推导题:** 推导gmapping中优化提议分布的数学形式，说明为什么融合激光观测的提议分布可以提高采样效率。

6. **设计题:** 某2000m²的仓库需要高精度建图，机器人配备2D激光雷达和轮式里程计。设计完整的建图方案，包括SLAM算法选择、参数配置、建图路径规划和地图质量评估方法。

---

## 仿真结合实例（当前仓库）：用 Gazebo 数据验证粒子滤波 SLAM 接口

### 目标与知识点对应

gmapping 的粒子滤波更新需要激光观测、里程计和 TF。本仓库没有 gmapping 节点，实例用 `robot_sim_demo` 产生同样的数据接口，并用 rosbag 检查数据连续性。

### 运行步骤

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_sim_demo gazebo2.launch.py \
  gui:=false rviz:=false drive:=true
```

```bash
ros2 bag record -o /tmp/gmapping_input /scan /odom /tf /tf_static
# 运行约 10 秒后 Ctrl+C
ros2 bag info /tmp/gmapping_input
```

### 观察结果

检查 bag 中 `/scan` 和 `/odom` 的消息数量、时间范围与 frame_id；这些是粒子预测和观测更新的输入条件。

### 源码与边界

数据桥配置位于 `src/robot_sim_demo/config/gazebo2_bridge.yaml`，在线建图对照位于 `src/slam_sim_demo_ros2/slam_sim_demo_ros2/slam_map_runner.py`。

本例不运行 gmapping 算法，也不把 `slam_toolbox` 的地图作为 gmapping 结果。

> 参考来源：
> - ROS Wiki —— gmapping 包文档与参数说明：https://wiki.ros.org/gmapping
> - OpenSLAM —— gmapping 项目页（RBPF 网格建图）：https://openslam-org.github.io/gmapping.html
> - SLAM Toolbox Wiki —— Steve Macenski 维护的参数与架构说明：https://github.com/SteveMacenski/slam_toolbox/wiki
> - The Construct —— ROS 2 SLAM 课程（gmapping 到 slam_toolbox 的迁移）：https://www.theconstructsim.com/
> - Robotics Back-End —— gMapping vs Cartographer 对比与建图实践：https://roboticsbackend.com/
