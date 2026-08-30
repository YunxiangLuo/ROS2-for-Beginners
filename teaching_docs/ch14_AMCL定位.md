# 第14章 AMCL定位

> **课程**：ROS2 Python 编程  
> **章节**：第14章  
> **课时**：2 课时（90 分钟）  
> **教学方式**：讲授 + 演示  

---

## 学习目标

本章学习目标包括：理解自适应蒙特卡洛定位(AMCL)的原理和算法流程，掌握粒子滤波定位的核心技术，熟悉KLD自适应采样和重采样策略，能够在ROS2中配置和使用AMCL进行机器人定位，掌握AMCL参数调优方法。

## 14.1 AMCL定位原理

### 14.1.1 定位问题定义

机器人定位是在已知地图的情况下，根据传感器数据估计自身位姿的过程。这是导航三大问题中的"我在哪里？"。

**定位问题的数学形式：**
```
p(x_t | z_{1:t}, u_{1:t-1}, m)
```

其中：`x_t` 是机器人在时刻 t 的位姿，`z_{1:t}` 是从起始到 t 时刻的所有观测，`u_{1:t-1}` 是从起始到 t-1 时刻的所有控制输入，`m` 是已知环境地图。

**定位的三种类型：**全局定位是机器人不知道初始位姿，需要在地图中定位自己；位置追踪是已知初始近似位姿，持续跟踪位置变化；绑架问题是机器人被意外移动到新位置，需要重新定位。

### 14.1.2 蒙特卡洛定位

蒙特卡洛定位 (Monte Carlo Localization, MCL) 使用粒子滤波实现机器人在已知地图上的定位。

**MCL核心思想：**使用一组加权粒子表示机器人位姿的后验分布；每个粒子是一个可能的位姿假设 (x, y, θ)；粒子权重表示该位姿与传感器观测的匹配程度。

```python
import numpy as np
from scipy.spatial import KDTree
import rclpy
from rclpy.node import Node

class MCLParticle:
    """蒙特卡洛定位单个粒子"""
    def __init__(self):
        self.pose = np.zeros(3)   # x, y, theta
        self.weight = 1.0
        
class MonteCarloLocalization:
    """蒙特卡洛定位基础实现"""
    def __init__(self, num_particles: int = 500):
        self.num_particles = num_particles
        self.particles = []
        self.map_data = None
        self.map_resolution = None
        self.map_origin = None
        self.global_localization = True
        
    def initialize_global(self, map_data: np.ndarray, 
                          map_resolution: float,
                          map_origin: tuple):
        """全局初始化：粒子均匀分布在整个地图的自由空间"""
        self.map_data = map_data
        self.map_resolution = map_resolution
        self.map_origin = map_origin
        
        free_cells = np.where(map_data == 0)
        num_free = len(free_cells[0])
        
        self.particles = []
        for _ in range(self.num_particles):
            idx = np.random.randint(num_free)
            gy, gx = free_cells[0][idx], free_cells[1][idx]
            
            # 栅格坐标转世界坐标
            wx = gx * map_resolution + map_origin[0]
            wy = gy * map_resolution + map_origin[1]
            
            particle = MCLParticle()
            particle.pose = np.array([
                wx + np.random.uniform(-0.1, 0.1),
                wy + np.random.uniform(-0.1, 0.1),
                np.random.uniform(-np.pi, np.pi)
            ])
            particle.weight = 1.0 / self.num_particles
            self.particles.append(particle)
            
        self.global_localization = False
        
    def initialize_pose(self, x: float, y: float, yaw: float,
                        noise: np.ndarray = None):
        """基于初始位姿估计初始化粒子"""
        if noise is None:
            noise = np.array([0.1, 0.1, 0.1])
            
        self.particles = []
        for _ in range(self.num_particles):
            particle = MCLParticle()
            particle.pose = np.array([
                x + np.random.normal(0, noise[0]),
                y + np.random.normal(0, noise[1]),
                yaw + np.random.normal(0, noise[2])
            ])
            particle.weight = 1.0 / self.num_particles
            self.particles.append(particle)
            
    def predict(self, control: np.ndarray, noise: np.ndarray):
        """运动模型预测"""
        v, omega = control
        dt = 0.1
        
        for particle in self.particles:
            theta = particle.pose[2]
            
            # 速度运动模型 + 噪声
            particle.pose[0] += (v * np.cos(theta) * dt + 
                                 np.random.normal(0, noise[0]))
            particle.pose[1] += (v * np.sin(theta) * dt + 
                                 np.random.normal(0, noise[1]))
            particle.pose[2] += (omega * dt + 
                                 np.random.normal(0, noise[2]))
            
    def update_weights(self, scan_points: np.ndarray):
        """根据激光观测更新粒子权重"""
        for particle in self.particles:
            likelihood = self._compute_scan_likelihood(
                particle.pose, scan_points
            )
            particle.weight *= (likelihood + 1e-10)
            
        # 归一化
        weights = np.array([p.weight for p in self.particles])
        total = weights.sum()
        if total > 0:
            weights /= total
            for i, p in enumerate(self.particles):
                p.weight = weights[i]
                
    def _compute_scan_likelihood(self, pose: np.ndarray,
                                   scan_points: np.ndarray) -> float:
        """计算激光扫描的似然（似然域模型）"""
        if self.map_data is None:
            return 1.0
            
        # 预计算地图中每个栅格到最近障碍物的距离
        if not hasattr(self, '_distance_map'):
            self._build_distance_map()
            
        theta = pose[2]
        c, s = np.cos(theta), np.sin(theta)
        
        total_log_likelihood = 0.0
        count = 0
        
        for point in scan_points:
            # 激光点到世界坐标
            wx = pose[0] + point[0] * c - point[1] * s
            wy = pose[1] + point[0] * s + point[1] * c
            
            # 世界坐标到栅格坐标
            gx = int((wx - self.map_origin[0]) / self.map_resolution)
            gy = int((wy - self.map_origin[1]) / self.map_resolution)
            
            if 0 <= gx < self._distance_map.shape[1] and \
               0 <= gy < self._distance_map.shape[0]:
                dist = self._distance_map[gy, gx]
                
                # 高斯似然
                sigma = 0.1
                likelihood = (1.0 / np.sqrt(2 * np.pi * sigma**2) * 
                             np.exp(-dist**2 / (2 * sigma**2)))
                total_log_likelihood += np.log(max(likelihood, 1e-10))
                count += 1
                
        if count == 0:
            return 1e-10
            
        return np.exp(total_log_likelihood / count)
    
    def _build_distance_map(self):
        """构建距离场地图（每个栅格到最近障碍物的距离）"""
        H, W = self.map_data.shape
        self._distance_map = np.ones((H, W)) * 100.0
        
        # 障碍物栅格位置
        obstacle_cells = np.where(self.map_data == 100)
        obstacle_points = np.column_stack(
            (obstacle_cells[1], obstacle_cells[0])
        )
        
        if len(obstacle_points) == 0:
            return
            
        tree = KDTree(obstacle_points)
        
        for y in range(H):
            for x in range(W):
                dist, _ = tree.query([x, y])
                self._distance_map[y, x] = dist * self.map_resolution
```

### 14.1.3 AMCL的核心改进

AMCL (Adaptive Monte Carlo Localization) 在标准MCL基础上增加了两个关键改进：

**KLD自适应采样：**根据粒子分布的熵动态调整粒子数量

**重采样策略改进：**结合短期和长期平均权重，检测和处理绑架问题

### 14.1.4 官方要点——ROS Wiki amcl 页面与绑架恢复的历史脉络

ROS Wiki 的 amcl 页面（ROS 1 遗产文档）解释了 `alpha_slow`/`alpha_fast` 的原始定义：两者分别是权重慢速/快速指数平均的衰减率，当快速平均低于慢速平均时（说明观测似然整体骤降，即被绑架或重定位失败），按两者比值插入随机粒子。本章 14.5.1 问题 3 的配置正源于此机制，Nav2 版本沿用了这一语义。

该页面同样记录了 `laser_max_beams` 的作用——把一次扫描均匀降采样为固定数量束参与似然计算。束数越多抗噪性越强但单次更新越慢，180～360 是常用区间；这与 14.5.1 问题 2「漂移时提高 beam 数」的建议互为印证。

## 14.2 粒子滤波定位详解

### 14.2.1 运动模型

AMCL支持两种运动模型：

**速度运动模型 (Velocity Motion Model)：**
```
x_t = f(x_{t-1}, u_t) + noise
```

其中：
```
x_t.x = x_{t-1}.x + v/ω · sin(θ + ω·Δt) - v/ω · sin(θ)
x_t.y = x_{t-1}.y - v/ω · cos(θ + ω·Δt) + v/ω · cos(θ)
x_t.θ = x_{t-1}.θ + ω·Δt + γ·Δt
```

**里程计运动模型 (Odometry Motion Model)：**
直接使用里程计测量的相对运动作为控制输入。

### 14.2.2 观测模型

AMCL支持两种传感器模型：

**波束模型 (Beam Model)：**
综合考虑四种测量情况：
1. 正确测量（高斯噪声）
2. 意外障碍物（指数分布）
3. 测量失败（均匀分布）
4. 随机测量（均匀分布）

```python
def beam_model_likelihood(z: float, z_expected: float, 
                           z_max: float, params: dict) -> float:
    """
    波束模型似然计算
    
    参数:
        z: 实际测量值
        z_expected: 期望测量值（根据地图计算）
        z_max: 最大测量距离
        params: 模型参数 {zhit, zshort, zmax, zrand}
    """
    zhit = params.get('zhit', 0.95)
    zshort = params.get('zshort', 0.01)
    zmax = params.get('zmax', 0.01)
    zrand = params.get('zrand', 0.03)
    
    sigma_hit = 0.1  # 测量噪声
    
    # 1. 正确测量（高斯分布）
    p_hit = zhit * (1.0 / np.sqrt(2 * np.pi * sigma_hit**2) *
                    np.exp(-(z - z_expected)**2 / (2 * sigma_hit**2)))
    
    # 2. 意外障碍物
    if z < z_expected:
        p_short = zshort * (2.0 / z_expected * 
                            (1 - z / z_expected))
    else:
        p_short = 0.0
    
    # 3. 测量失败
    if z >= z_max - 0.01:
        p_max = zmax * 1.0
    else:
        p_max = 0.0
    
    # 4. 随机测量
    p_rand = zrand * 1.0 / z_max
    
    return p_hit + p_short + p_max + p_rand
```

**似然域模型 (Likelihood Field Model):**
计算激光点末端到地图障碍物的距离，转为概率。

```python
class LikelihoodFieldModel:
    """似然域模型"""
    def __init__(self, map_data: np.ndarray, resolution: float):
        self.map_data = map_data
        self.resolution = resolution
        self.sigma = 0.1  # 测量噪声
        self.z_hit = 0.95
        self.z_rand = 0.05
        self.z_max = 30.0
        self.distance_map = self._build_distance_map()
        
    def _build_distance_map(self) -> np.ndarray:
        """构建距离场"""
        H, W = self.map_data.shape
        distance_map = np.ones((H, W)) * self.z_max
        
        # 找障碍物像素
        obstacle_y, obstacle_x = np.where(self.map_data == 100)
        obstacle_pts = np.column_stack((obstacle_x, obstacle_y))
        
        if len(obstacle_pts) == 0:
            return distance_map
            
        tree = KDTree(obstacle_pts)
        
        # 对每个像素计算到最近障碍物的距离
        ys, xs = np.mgrid[0:H, 0:W]
        pixels = np.column_stack((xs.flatten(), ys.flatten()))
        
        distances, _ = tree.query(pixels)
        distance_map = distances.reshape(H, W) * self.resolution
        
        return distance_map
    
    def probability(self, point: np.ndarray, 
                    robot_pose: np.ndarray) -> float:
        """
        计算激光点的似然概率
        
        参数:
            point: 激光点在激光坐标系的坐标
            robot_pose: 机器人位姿 (x, y, theta)
        """
        theta = robot_pose[2]
        c, s = np.cos(theta), np.sin(theta)
        
        # 激光点在世界坐标系下的位置
        wx = robot_pose[0] + point[0]*c - point[1]*s
        wy = robot_pose[1] + point[0]*s + point[1]*c
        
        # 转栅格坐标
        gx = int(wx / self.resolution)
        gy = int(wy / self.resolution)
        
        if not (0 <= gx < self.distance_map.shape[1] and
                0 <= gy < self.distance_map.shape[0]):
            return self.z_rand / self.z_max
            
        dist = self.distance_map[gy, gx]
        
        if dist == 0:
            return 1.0
            
        # 高斯似然 + 均匀随机
        p_hit = (self.z_hit * 
                 np.exp(-dist**2 / (2 * self.sigma**2)) / 
                 (np.sqrt(2 * np.pi) * self.sigma))
        p_rand = self.z_rand / self.z_max
        
        return p_hit + p_rand
```

### 14.2.3 重采样策略

**低方差重采样 (Low Variance Resampling)：**
```python
def low_variance_resampling(particles: list, num: int = None) -> list:
    """低方差重采样算法"""
    if num is None:
        num = len(particles)
        
    weights = np.array([p.weight for p in particles])
    weights /= weights.sum() if weights.sum() > 0 else 1
    
    new_particles = []
    step = 1.0 / num
    start = np.random.uniform(0, step)
    cumsum = np.cumsum(weights)
    
    i = 0
    for _ in range(num):
        while cumsum[i] < start:
            i += 1
            if i >= len(cumsum):
                i = len(cumsum) - 1
                break
        new_particle = MCLParticle()
        new_particle.pose = particles[i].pose.copy()
        new_particle.weight = 1.0 / num
        new_particles.append(new_particle)
        start += step
        
    return new_particles
```

## 14.3 KLD自适应采样

### 14.3.1 KLD采样原理

KLD (Kullback-Leibler Distance) 采样根据粒子分布的熵动态调整粒子数量：粒子发散时（不确定性高）增加粒子数，粒子收敛时（不确定性低）减少粒子数。

**KLD采样公式：**
```
n = (k-1) / (2·ε) · z²_{1-δ}
```

其中：`k` 是粒子分布覆盖的栅格数（离散化位姿空间），`ε` 是近似误差阈值，`z_{1-δ}` 是标准正态分布的 1-δ 分位数，`n` 是所需粒子数。

```python
class KLDSampling:
    """KLD自适应采样"""
    def __init__(self, min_particles: int = 100, 
                 max_particles: int = 2000,
                 epsilon: float = 0.05,
                 delta: float = 0.01):
        self.min_particles = min_particles
        self.max_particles = max_particles
        self.epsilon = epsilon
        self.delta = delta
        
        # KLD参数
        self.bin_size = np.array([0.2, 0.2, 0.1])  # 位姿空间离散化
        self.kld_bins = {}  # 占用栅格记录
        
    def compute_kld_sample_size(self, particles: list) -> int:
        """
        计算KLD所需的粒子数
        
        根据粒子分布覆盖的栅格数自适应调整
        """
        if len(particles) <= self.min_particles:
            return self.min_particles
            
        # 统计粒子分布覆盖的栅格数
        covered_bins = self._count_covered_bins(particles)
        k = len(covered_bins)
        
        # KLD公式
        z_score = 2.57  # 对应delta=0.01的z值
        n = int((k - 1) / (2 * self.epsilon) * z_score**2)
        
        return np.clip(n, self.min_particles, self.max_particles)
    
    def _count_covered_bins(self, particles: list) -> set:
        """统计粒子分布覆盖的位姿空间栅格"""
        covered = set()
        
        for p in particles:
            pose = p.pose
            bx = int(pose[0] / self.bin_size[0])
            by = int(pose[1] / self.bin_size[1])
            bt = int(pose[2] / self.bin_size[2])
            covered.add((bx, by, bt))
            
        return covered
    
    def adaptive_resample(self, particles: list) -> list:
        """执行自适应重采样"""
        n_required = self.compute_kld_sample_size(particles)
        
        # 只有当粒子数不足或超过阈值时才重采样
        neff = self._compute_neff(particles)
        
        if neff < len(particles) * 0.5:
            # 执行低方差重采样
            new_particles = low_variance_resampling(particles, n_required)
            
            # 添加随机粒子（防止绑架问题）
            if np.random.random() < 0.05:
                self._add_random_particles(new_particles)
                
            return new_particles
        
        return particles
    
    def _compute_neff(self, particles: list) -> float:
        """计算有效样本数"""
        weights = np.array([p.weight for p in particles])
        weights_sum = weights.sum()
        if weights_sum == 0:
            return 0
        normalized = weights / weights_sum
        return 1.0 / np.sum(normalized ** 2)
    
    def _add_random_particles(self, particles: list):
        """添加随机粒子以处理绑架问题"""
        num_random = max(1, len(particles) // 20)
        for _ in range(num_random):
            idx = np.random.randint(len(particles))
            particle = particles[idx]
            # 随机扰动
            particle.pose[0] += np.random.uniform(-1.0, 1.0)
            particle.pose[1] += np.random.uniform(-1.0, 1.0)
            particle.pose[2] += np.random.uniform(-0.5, 0.5)
```

### 14.3.2 官方要点——KLD 采样的信息论出处与实验建议

本章 14.3 的 KLD 自适应采样出自 Fox 与 Dieter Fox 团队的 KLD-Sampling 工作（蒙特卡洛定位系列论文），其核心结论是：所需样本数只与粒子分布覆盖的"有效栅格数 k"相关，收敛后 k 急剧缩小，因此可以安全降粒子数。文献中的实验表明，在约 1000 ㎡的办公楼里，KLD 采样能把平均粒子数从数千降到数百而保持同等定位精度。

按 The Construct 课程的建议复现：分别在开阔大厅与狭窄走廊录制 bag，观察 `/particlecloud` 数量的动态变化；开阔处粒子应明显变少。这可以直接验证 14.5.2 的收敛过程示意，也是课后练习 4 的天然素材。

## 14.4 ROS2 AMCL配置

### 14.4.1 Nav2 AMCL节点

在ROS2中，AMCL是Nav2框架的一部分：

```bash
# 启动AMCL定位
ros2 launch nav2_bringup localization_launch.py \
  map:=./maps/my_map.yaml \
  params_file:=./config/nav2_params.yaml \
  use_sim_time:=true
```

### 14.4.2 AMCL参数配置

```yaml
amcl:
  ros__parameters:
    # 粒子滤波参数
    max_particles: 2000
    min_particles: 500
    
    # 更新阈值
    update_min_a: 0.2       # 最小旋转更新(rad)
    update_min_d: 0.1       # 最小平移更新(m)
    
    # 激光模型
    laser_model_type: "likelihood_field"  # 或 "beam"
    laser_min_range: 0.1
    laser_max_range: 8.0
    laser_max_beams: 180
    
    # 似然域模型参数
    z_hit: 0.95
    z_rand: 0.05
    sigma_hit: 0.2
    
    # 波束模型参数
    lambda_short: 0.1
    z_short: 0.01
    z_max: 0.01
    
    # 初始位姿
    initial_pose.x: 0.0
    initial_pose.y: 0.0
    initial_pose.yaw: 0.0
    
    # 初始位姿不确定性
    initial_pose.covariance_x: 0.5
    initial_pose.covariance_y: 0.5
    initial_pose.covariance_yaw: 0.5
    
    # TF坐标系
    base_frame_id: "base_link"
    global_frame_id: "map"
    odom_frame_id: "odom"
    
    # 重采样参数
    resample_interval: 1   # 每次更新都重采样
    alpha_slow: 0.001      # 慢速平均权重衰减
    alpha_fast: 0.1        # 快速平均权重衰减
    
    # KLD参数
    kld_epsilon: 0.05
    kld_delta: 0.01
    
    # 变换
    set_initial_pose: true
    always_reset_initial_pose: false
    first_map_only: false
```

### 14.4.3 AMCL启动与交互

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import tf2_ros
import numpy as np

class AMCLInterface(Node):
    """AMCL交互接口节点"""
    def __init__(self):
        super().__init__('amcl_interface')
        
        # 初始位姿发布器
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
            
        # AMCL位姿订阅
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose',
            self.amcl_pose_callback, 10)
            
        # 粒子云订阅（可视化AMCL内部状态）
        self.particle_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/particlecloud',
            self.particle_callback, 10)
            
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.current_pose = None
        self.particle_cloud = []
        
    def set_initial_pose(self, x: float, y: float, yaw: float):
        """设置AMCL初始位姿"""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        
        qz = np.sin(yaw / 2.0)
        qw = np.cos(yaw / 2.0)
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        
        # 设置协方差（初始不确定性）
        msg.pose.covariance[0] = 0.25   # x方差
        msg.pose.covariance[7] = 0.25   # y方差
        msg.pose.covariance[35] = 0.068 # yaw方差
        
        self.initial_pose_pub.publish(msg)
        self.get_logger().info(
            f'初始位姿已设置: ({x:.2f}, {y:.2f}, {np.degrees(yaw):.1f}°)')
        
    def amcl_pose_callback(self, msg):
        """AMCL估计位姿回调"""
        pose = msg.pose.pose
        # 四元数转欧拉角
        q = pose.orientation
        yaw = np.arctan2(2*(q.w*q.z + q.x*q.y), 
                         1-2*(q.y**2 + q.z**2))
        
        self.current_pose = {
            'x': pose.position.x,
            'y': pose.position.y,
            'yaw': yaw,
            'covariance': np.array(msg.pose.covariance).reshape(6, 6)
        }
        
        # 计算定位不确定性
        uncertainty = np.sqrt(np.trace(self.current_pose['covariance'][:2, :2]))
        self.get_logger().info(
            f'AMCL位姿: ({pose.position.x:.3f}, {pose.position.y:.3f}, '
            f'{np.degrees(yaw):.1f}°) 不确定度={uncertainty:.4f}',
            throttle_duration_sec=1.0)
        
    def particle_callback(self, msg):
        """粒子云回调"""
        self.particle_cloud = []
        for pose in msg.poses:
            q = pose.orientation
            yaw = np.arctan2(2*(q.w*q.z + q.x*q.y),
                             1-2*(q.y**2 + q.z**2))
            self.particle_cloud.append({
                'x': pose.position.x,
                'y': pose.position.y,
                'yaw': yaw
            })
            
    def get_localization_quality(self) -> float:
        """
        获取定位质量评估
        
        返回: 0~1 的质量评分
        """
        if self.current_pose is None:
            return 0.0
            
        # 根据协方差评估
        cov = self.current_pose['covariance']
        trace_xy = cov[0, 0] + cov[1, 1]
        
        # 根据粒子分布评估
        if len(self.particle_cloud) < 10:
            return 0.0
            
        poses = np.array([[p['x'], p['y']] 
                          for p in self.particle_cloud])
        spread = np.std(poses, axis=0).mean()
        
        # 综合评分
        quality = 1.0 / (1.0 + trace_xy * 10 + spread * 5)
        return np.clip(quality, 0.0, 1.0)
    
    def check_amcl_status(self) -> str:
        """检查AMCL状态"""
        if self.current_pose is None:
            return "AMCL未初始化"
            
        quality = self.get_localization_quality()
        if quality > 0.7:
            return f"定位良好 (质量={quality:.2f})"
        elif quality > 0.3:
            return f"定位一般 (质量={quality:.2f})"
        else:
            return f"定位差，建议重新初始化 (质量={quality:.2f})"
```

### 14.4.4 定位性能监控

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
import numpy as np
import matplotlib.pyplot as plt

class AMCLMonitor(Node):
    """AMCL性能监控器"""
    def __init__(self):
        super().__init__('amcl_monitor')
        
        self.particle_sub = self.create_subscription(
            PoseArray, '/particlecloud', self.particle_callback, 10)
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose',
            self.pose_callback, 10)
            
        self.particle_history = []
        self.pose_history = []
        
    def particle_callback(self, msg):
        """记录粒子分布"""
        positions = [(p.position.x, p.position.y) for p in msg.poses]
        if positions:
            positions = np.array(positions)
            spread = np.std(positions, axis=0)
            self.particle_history.append({
                'time': self.get_clock().now().nanoseconds,
                'num_particles': len(positions),
                'spread_x': spread[0],
                'spread_y': spread[1]
            })
            
    def pose_callback(self, msg):
        """记录位姿估计"""
        pose = msg.pose.pose
        q = pose.orientation
        yaw = np.arctan2(2*(q.w*q.z + q.x*q.y),
                         1-2*(q.y**2 + q.z**2))
        self.pose_history.append({
            'time': self.get_clock().now().nanoseconds,
            'x': pose.position.x,
            'y': pose.position.y,
            'yaw': yaw,
            'cov_trace': np.trace(
                np.array(msg.pose.covariance).reshape(6, 6)[:3, :3])
        })
        
    def get_statistics(self) -> dict:
        """获取定位统计信息"""
        if not self.pose_history:
            return {}
            
        recent_poses = self.pose_history[-100:]
        recent_particles = self.particle_history[-100:]
        
        stats = {
            'current_cov_trace': recent_poses[-1]['cov_trace'],
            'avg_particle_spread': (
                np.mean([p['spread_x'] for p in recent_particles]),
                np.mean([p['spread_y'] for p in recent_particles])
            ),
            'position_variance': (
                np.var([p['x'] for p in recent_poses]),
                np.var([p['y'] for p in recent_poses])
            )
        }
        return stats
```

### 14.4.5 多传感器定位

AMCL可以融合多个传感器信息提高定位精度：

```python
class MultiSensorAMCL(Node):
    """多传感器AMCL定位"""
    def __init__(self):
        super().__init__('multi_sensor_amcl')
        
        # 主激光雷达
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
            
        # 辅助激光雷达（如后方）
        self.scan_back_sub = self.create_subscription(
            LaserScan, '/scan_back', self.scan_back_callback, 10)
            
        # IMU辅助（航向观测）
        self.imu_sub = self.create_subscription(
            Imu, '/imu', self.imu_callback, 10)
            
        # 里程计融合
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
            
        self.particles = []
        self.imu_yaw = None
        self.odom_pose = np.zeros(3)
        
    def imu_callback(self, msg: Imu):
        """IMU数据：提供航向约束"""
        q = msg.orientation
        yaw = np.arctan2(2*(q.w*q.z + q.x*q.y),
                         1-2*(q.y**2 + q.z**2))
        self.imu_yaw = yaw
        
        # 利用IMU航向更新粒子位姿
        if self.imu_yaw is not None:
            for particle in self.particles:
                # IMU观测似然（航向约束）
                yaw_diff = abs(particle.pose[2] - self.imu_yaw)
                yaw_diff = min(yaw_diff, 2*np.pi - yaw_diff)
                yaw_likelihood = np.exp(-yaw_diff**2 / (2 * 0.05**2))
                particle.weight *= yaw_likelihood
```

### 14.4.6 官方要点——nav2_amcl 官方文档的参数组与生命周期

docs.ros.org 的 nav2_amcl 页面把参数分为四组：通用（`scan_topic`、`set_initial_pose`、`always_reset_initial_pose`、`first_map_only`）、粒子滤波（`min_particles`、`max_particles`、`resample_interval`、`alpha_slow`/`alpha_fast`）、激光模型（`laser_model_type`、`laser_max_beams`、`z_hit`/`z_rand`/`sigma_hit`、`lambda_short`）与运动模型（`odom_model_type`，可选 `diff`、`omni`、`diff_c`、`omni_c`，对应 14.2.1 的两种模型及其协方差变体）。文档特别强调：`update_min_d`/`update_min_a` 决定"多久执行一次滤波更新"，过大导致快速转弯时更新不及时，过小浪费算力——这是 14.5.1 调参表背后的语义。

作为 Nav2 生命周期的受管节点，AMCL 必须在 map_server 之后激活（依赖 `/map`）；`ros2 lifecycle set /amcl activate` 的顺序错误是新手定位失败的常见原因。官方教程还提供 `nav2_simple_commander` 中配合 initial pose 的标准初始化流程，可对照本章 14.4.3 的接口代码阅读。

## 14.5 定位问题与解决方案

### 14.5.1 常见定位问题

**问题1：全局定位失败**
```yaml
# 解决方案：增加初始粒子数和分散度
initial_pose.covariance_x: 1.0
initial_pose.covariance_y: 1.0
initial_pose.covariance_yaw: 1.0
max_particles: 5000
```

**问题2：定位漂移**
```yaml
# 解决方案：强化观测模型
laser_model_type: "beam"  # 波束模型更精确
laser_max_beams: 360  # 使用全部激光束
sigma_hit: 0.1  # 减小观测噪声
```

**问题3：绑架问题**
```yaml
# 解决方案：增加随机粒子
alpha_slow: 0.001
alpha_fast: 0.1
# 当快速平均权重远小于慢速平均权重时，AMCL会添加随机粒子
```

### 14.5.2 粒子收敛过程可视化

```
初始阶段（全局均匀分布）:
o o o o o o o o o o
o o o o o o o o o o
o o o o o o o o o o
o o o o o o o o o o
o o o o o o o o o o

运动后（粒子开始收敛）:
. . . . o . . . . .
. . . o o o . . . .
. . o o o o o . . .
. . . o o o . . . .
. . . . o . . . . .

收敛后（粒子聚拢）:
. . . . . . . . . .
. . . . . . . . . .
. . . o o o . . . .
. . . . . . . . . .
. . . . . . . . . .

成功定位后（单点估计）:
. . . . . . . . . .
. . . . × . . . . .
. . . . . . . . . .
```
(× 表示AMCL估计的最终位姿)

### 14.5.3 定位质量自动监控

```python
class LocalizationHealthMonitor(Node):
    """定位健康状态监控"""
    def __init__(self):
        super().__init__('localization_health')
        
        self.particle_sub = self.create_subscription(
            PoseArray, '/particlecloud', self.particle_callback, 10)
            
        self.pose_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
            
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
            
        self.consecutive_bad_readings = 0
        self.auto_recovery_enabled = True
        
    def particle_callback(self, msg):
        """监控粒子分布健康状态"""
        if len(msg.poses) < 10:
            return
            
        positions = np.array([[p.position.x, p.position.y] 
                              for p in msg.poses])
        spread = np.std(positions, axis=0)
        mean_spread = np.mean(spread)
        
        # 检测定位异常
        if mean_spread > 2.0:  # 粒子散布过大
            self.consecutive_bad_readings += 1
            self.get_logger().warn(
                f'粒子分布过大: spread={mean_spread:.2f}m')
            
            if self.consecutive_bad_readings > 10 and self.auto_recovery_enabled:
                self.auto_recover()
        else:
            self.consecutive_bad_readings = 0
            
    def auto_recover(self):
        """自动恢复定位"""
        self.get_logger().warn('尝试自动恢复定位...')
        
        # 重新初始化位姿（使用最后一次可靠估计）
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = self.last_known_pose[0]
        msg.pose.pose.position.y = self.last_known_pose[1]
        msg.pose.covariance[0] = 1.0
        msg.pose.covariance[7] = 1.0
        msg.pose.covariance[35] = 0.5
        
        self.initial_pose_pub.publish(msg)
        self.consecutive_bad_readings = 0
        self.get_logger().info('自动恢复定位完成')
```

### 14.5.4 定位精度评估

```bash
# 使用tf树评估定位精度
ros2 run tf2_ros tf2_monitor

# 查看AMCL发布的位姿话题
ros2 topic echo /amcl_pose

# 查看粒子云
ros2 topic echo /particlecloud

# RViz2可视化
rviz2
# 添加: ParticleCloud, PoseWithCovariance
```

### 14.5.5 官方要点——定位健康监控与失败恢复的工程纪律

Robotics Back-End 与 Nav2 文档共同强调三条定位健康纪律：第一，监控 `/amcl_pose` 协方差对角元素（尤其是 x、y 方差），方差持续增大说明粒子发散，应结合 `stddev_scan_diff`（本章 14.4.4 的思路）建立自动告警；第二，重定位失败时不要盲目重启节点，先用 RViz2 的 "2D Pose Estimate" 或发布 `/initialpose` 带大协方差试探性收敛，再逐步收紧；第三，`always_reset_initial_pose: false` 与 `first_map_only: false` 组合适用于多地图切换场景（练习第 6 题），而单地图固定起点部署可设 `set_initial_pose: true` 让启动即收敛。另外提醒：AMCL 只估计 `map`→`odom` 变换，航迹推算质量由 `odom_frame_id` 下游的里程计决定，定位异常时应先排除里程计漂移再调 AMCL 参数。

## 课后练习

1. **原理题:** 阐述AMCL中KLD自适应采样的原理，说明为什么在粒子收敛时可以减少粒子数而在发散时需要增加粒子数。

2. **编程题:** 实现一个简化版的AMCL系统，包含粒子初始化、运动模型预测、似然域权重更新和低方差重采样四个步骤。

3. **分析题:** 比较似然域模型和波束模型两种传感器模型的优缺点，说明在什么场景下应该选择哪种模型。

4. **配置题:** 在已知地图上启动AMCL定位，配置参数使得在以下场景中定位性能最优：a) 对称走廊 b) 开阔大厅 c) 狭窄通道。

5. **操作题:** 描述当AMCL定位丢失时，如何通过ROS2命令行手动重置初始位姿并恢复定位。

6. **设计题:** 某物流机器人需要在多个楼层间自动切换定位，设计一个跨楼层定位方案，包括地图切换机制、AMCL参数自适应调整和定位质量评估方法。

---

## 仿真结合实例（当前仓库）：预置地图上的 AMCL 初始位姿

### 目标与知识点对应

AMCL 需要地图、激光、里程计和初始位姿。本实例通过 `navigation_sim_demo_ros2` 启动 `map_server` 与 `amcl`，然后观察初始位姿发布和 `/amcl_pose` 输出。

### 运行步骤

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch navigation_sim_demo_ros2 nav2_demo.launch.py \
  use_gazebo:=true use_rviz:=true gz_headless:=false \
  initial_pose_x:=0.0 initial_pose_y:=0.0 initial_pose_yaw:=0.0
```

```bash
ros2 topic echo /amcl_pose --once
ros2 topic echo /particlecloud --once
ros2 topic info /map
```

### 观察结果

RViz 可显示地图、LaserScan 和 AMCL 位姿；调整 `initial_pose_x/y/yaw` 后重启，比较初始估计对定位过程的影响。

### 源码与边界

Nav2 Launch 位于 `src/navigation_sim_demo_ros2/launch/nav2_demo.launch.py`，初始位姿节点位于 `src/navigation_sim_demo_ros2/navigation_sim_demo_ros2/initial_pose_publisher.py`，地图位于 `src/navigation_sim_demo_ros2/maps/Software_Museum.yaml`。

粒子云是否收敛应以本地 RViz 和 `/amcl_pose` 实际输出判断；启动日志本身不等于定位精度验证。

学习材料：
- docs.ros.org —— nav2_amcl 参数与节点文档：https://docs.ros.org/en/jazzy/p/nav2_amcl/
- Nav2 官方文档 —— Navigation System 与定位教程：https://docs.nav2.org/
- ROS Wiki —— amcl 包文档（历史参数语义）：https://wiki.ros.org/amcl
- The Construct —— ROS 2 Localization with AMCL 课程：https://www.theconstructsim.com/
- Robotics Back-End —— AMCL 定位实战与调参指南：https://roboticsbackend.com/
