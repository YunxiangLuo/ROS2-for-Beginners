# 第10章 SLAM基本概念与贝叶斯框架

## 学习目标
- 理解SLAM问题的数学定义与核心挑战
- 掌握贝叶斯滤波框架在SLAM中的应用
- 熟悉SLAM系统中的传感器模型与TF坐标变换体系
- 了解SLAM方法的分类及其适用场景
- 能够推导基本贝叶斯滤波公式

## 10.1 SLAM问题定义

### 10.1.1 SLAM问题的起源与意义

SLAM (Simultaneous Localization and Mapping) 是移动机器人在未知环境中同时进行自身定位和环境地图构建的核心技术。这是机器人领域公认的基础性难题，被誉为"机器人学的圣杯"之一。

SLAM需要同时解决两个相互耦合的问题：
- **定位:** 机器人根据传感器数据确定自身在环境中的位姿 (x, y, θ)
- **建图:** 建立周围环境的栅格地图，用于后续导航和避障

定位与建图之间存在鸡生蛋蛋生鸡的关系：
- 要精确定位，需要知道环境地图
- 要构建精确地图，需要知道机器人位姿

SLAM的核心挑战在于打破这种耦合关系，在没有任何先验信息的情况下同时完成两项任务。

### 10.1.2 SLAM问题的数学形式化

SLAM的概率公式化表达为：

```
p(x_{1:t}, m | z_{1:t}, u_{1:t-1})
```

各符号含义：
- `x_{1:t} = {x_1, x_2, ..., x_t}`：机器人从时刻1到t的完整位姿序列
- `m`：环境地图（栅格地图或特征地图）
- `z_{1:t} = {z_1, z_2, ..., z_t}`：从时刻1到t的所有观测数据
- `u_{1:t-1} = {u_1, u_2, ..., u_{t-1}}`：从时刻1到t-1的所有控制输入

该后验概率的意义是：在给定所有观测数据和控制输入的情况下，同时估计机器人路径和地图的联合概率分布。

### 10.1.3 SLAM问题的分解

SLAM问题可以分解为两个递归步骤：

**预测步骤（运动模型）：**
```
p(x_t, m | z_{1:t-1}, u_{1:t-1}) = 
∫ p(x_t | x_{t-1}, u_t) · p(x_{t-1}, m | z_{1:t-1}, u_{1:t-2}) dx_{t-1}
```

**更新步骤（观测模型）：**
```
p(x_t, m | z_{1:t}, u_{1:t-1}) = 
η · p(z_t | x_t, m) · p(x_t, m | z_{1:t-1}, u_{1:t-1})
```

其中 η 为归一化常数，确保概率和为1。

## 10.2 贝叶斯滤波框架

### 10.2.1 贝叶斯滤波基本原理

贝叶斯滤波是SLAM问题的核心数学工具，它提供了根据观测数据递归估计状态的概率框架。

**基本思想：** 利用状态转移模型预测当前状态，然后利用观测模型更新预测。

```python
import numpy as np
from scipy.stats import norm

class BayesianFilter:
    """贝叶斯滤波基础框架"""
    def __init__(self, dim_state: int = 3):
        self.dim_state = dim_state
        self.belief_mean = np.zeros(dim_state)
        self.belief_cov = np.eye(dim_state) * 0.1
        
    def predict(self, control: np.ndarray, motion_noise: np.ndarray):
        """预测步骤：根据运动模型更新信念"""
        # 运动模型：x_t = f(x_{t-1}, u_t) + noise
        G = np.eye(self.dim_state)  # 运动模型雅可比
        self.belief_mean = self.motion_model(self.belief_mean, control)
        self.belief_cov = G @ self.belief_cov @ G.T + np.diag(motion_noise)
        
    def update(self, observation: np.ndarray, obs_noise: np.ndarray):
        """更新步骤：根据观测修正信念"""
        H = np.eye(self.dim_state)  # 观测模型雅可比
        innovation = observation - self.observation_model(self.belief_mean)
        S = H @ self.belief_cov @ H.T + np.diag(obs_noise)
        K = self.belief_cov @ H.T @ np.linalg.inv(S)
        self.belief_mean = self.belief_mean + K @ innovation
        self.belief_cov = (np.eye(self.dim_state) - K @ H) @ self.belief_cov
        
    def motion_model(self, state: np.ndarray, control: np.ndarray) -> np.ndarray:
        """运动模型：根据控制输入更新位姿"""
        x, y, theta = state
        v, omega = control
        dt = 0.1
        new_x = x + v * np.cos(theta) * dt
        new_y = y + v * np.sin(theta) * dt
        new_theta = theta + omega * dt
        return np.array([new_x, new_y, new_theta])
    
    def observation_model(self, state: np.ndarray) -> np.ndarray:
        """观测模型：从位姿预测观测值"""
        return state  # 简化：直接观测位姿
```

### 10.2.2 马尔可夫假设

贝叶斯滤波有效的前提是马尔可夫假设：
- **状态完备性：** 当前状态包含了预测未来所需的所有信息
- **观测独立性：** 给定当前状态，当前观测与过去观测独立

在SLAM中，这意味着：
- 机器人未来位姿只取决于当前位姿和控制输入
- 当前传感器观测只取决于当前位姿和地图

### 10.2.3 贝叶斯滤波的SLAM实现

```python
import numpy as np

class SLAMBayesianFilter:
    """基于贝叶斯滤波的SLAM框架"""
    def __init__(self, num_landmarks: int = 100):
        self.num_landmarks = num_landmarks
        # 状态向量：[robot_x, robot_y, robot_theta, lm1_x, lm1_y, lm2_x, lm2_y, ...]
        self.state_dim = 3 + 2 * num_landmarks
        self.mean = np.zeros(self.state_dim)
        self.cov = np.eye(self.state_dim) * 0.1
        self.landmarks_seen = np.zeros(num_landmarks, dtype=bool)
        
    def predict(self, odom: np.ndarray, noise: np.ndarray):
        """运动模型预测"""
        x, y, theta = self.mean[:3]
        v = odom[0]
        omega = odom[1]
        dt = 0.1
        
        # 更新机器人位姿
        self.mean[0] += v * np.cos(theta) * dt
        self.mean[1] += v * np.sin(theta) * dt
        self.mean[2] += omega * dt
        
        # 运动雅可比
        G = np.eye(self.state_dim)
        G[0, 2] = -v * np.sin(theta) * dt
        G[1, 2] = v * np.cos(theta) * dt
        
        # 协方差更新
        motion_cov = np.zeros((self.state_dim, self.state_dim))
        motion_cov[:3, :3] = np.diag(noise)
        self.cov = G @ self.cov @ G.T + motion_cov
        
    def update_with_landmark(self, z: np.ndarray, landmark_id: int, 
                              sensor_noise: np.ndarray):
        """使用路标点观测更新"""
        if not self.landmarks_seen[landmark_id]:
            # 初始化新路标点
            idx = 3 + 2 * landmark_id
            x_r, y_r, theta_r = self.mean[:3]
            range_meas, bearing_meas = z
            
            # 将极坐标观测转换为全局坐标
            lm_x = x_r + range_meas * np.cos(theta_r + bearing_meas)
            lm_y = y_r + range_meas * np.sin(theta_r + bearing_meas)
            self.mean[idx] = lm_x
            self.mean[idx + 1] = lm_y
            self.landmarks_seen[landmark_id] = True
        else:
            # EKF更新已知路标点
            idx = 3 + 2 * landmark_id
            x_r, y_r, theta_r = self.mean[:3]
            lm_x, lm_y = self.mean[idx], self.mean[idx + 1]
            
            # 预测观测
            dx = lm_x - x_r
            dy = lm_y - y_r
            predicted_range = np.sqrt(dx**2 + dy**2)
            predicted_bearing = np.arctan2(dy, dx) - theta_r
            
            # 观测雅可比
            H = np.zeros((2, self.state_dim))
            H[0, 0] = -dx / predicted_range
            H[0, 1] = -dy / predicted_range
            H[0, idx] = dx / predicted_range
            H[0, idx + 1] = dy / predicted_range
            H[1, 0] = dy / (dx**2 + dy**2)
            H[1, 1] = -dx / (dx**2 + dy**2)
            H[1, 2] = -1
            H[1, idx] = -dy / (dx**2 + dy**2)
            H[1, idx + 1] = dx / (dx**2 + dy**2)
            
            # EKF更新
            innovation = np.array([z[0] - predicted_range, 
                                    z[1] - predicted_bearing])
            innovation[1] = np.arctan2(np.sin(innovation[1]), 
                                        np.cos(innovation[1]))
            
            S = H @ self.cov @ H.T + np.diag(sensor_noise)
            K = self.cov @ H.T @ np.linalg.inv(S)
            self.mean = self.mean + K @ innovation
            self.cov = (np.eye(self.state_dim) - K @ H) @ self.cov
```

### 10.2.4 贝叶斯滤波的局限性

纯贝叶斯滤波在SLAM中面临以下挑战：
- **高维状态空间：** 包含地图点后，状态维度呈线性增长
- **非线性问题：** 运动模型和观测模型高度非线性
- **数据关联：** 需要正确匹配观测与地图点
- **计算复杂度：** 协方差矩阵更新为O(n²)复杂度

这些局限性催生了GraphSLAM、粒子滤波SLAM等更先进的SLAM方法。

## 10.3 传感器与坐标变换

### 10.3.1 SLAM常用传感器

**2D激光雷达 (LaserScan)：**
- 室内导航主要传感器，提供270°-360°范围内距离测量
- 发布 `sensor_msgs/LaserScan` 话题
- 典型参数：角度分辨率0.25°-1°，最大测距8-30m

**3D激光雷达 (PointCloud2)：**
- 室外环境和大场景SLAM使用
- 发布 `sensor_msgs/PointCloud2` 话题
- 代表：Velodyne VLP-16, Ouster OS系列

**深度相机 (RGB-D)：**
- 同时提供RGB图像和深度图
- 代表：Intel RealSense D415/D435, Microsoft Kinect
- 适用于视觉SLAM和室内精细建图

**IMU (惯性测量单元)：**
- 提供加速度和角速度测量
- 高频(100-1000Hz)但存在漂移
- 通常与激光/视觉融合使用

**里程计 (Odometry)：**
- 通过轮式编码器估算机器人位移
- 短时间内精度高，长时间存在累积误差

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu, Image
from nav_msgs.msg import Odometry

class SensorMonitor(Node):
    """多传感器数据监控节点"""
    def __init__(self):
        super().__init__('sensor_monitor')
        
        # 订阅激光雷达
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
            
        # 订阅IMU
        self.imu_sub = self.create_subscription(
            Imu, '/imu', self.imu_callback, 10)
            
        # 订阅里程计
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
            
        self.scan_count = 0
        self.imu_count = 0
        self.odom_count = 0
        
    def scan_callback(self, msg: LaserScan):
        self.scan_count += 1
        angles = len(msg.ranges)
        valid = sum(1 for r in msg.ranges if 0 < r < msg.range_max)
        self.get_logger().info(
            f'激光: {angles}条射线, {valid}个有效点, '
            f'范围[{msg.range_min:.2f}, {msg.range_max:.2f}]',
            throttle_duration_sec=2.0
        )
        
    def imu_callback(self, msg: Imu):
        self.imu_count += 1
        acc = msg.linear_acceleration
        gyro = msg.angular_velocity
        self.get_logger().info(
            f'IMU: acc=({acc.x:.2f},{acc.y:.2f},{acc.z:.2f}) '
            f'gyro=({gyro.x:.2f},{gyro.y:.2f},{gyro.z:.2f})',
            throttle_duration_sec=2.0
        )
        
    def odom_callback(self, msg: Odometry):
        self.odom_count += 1
        pose = msg.pose.pose
        twist = msg.twist.twist
        self.get_logger().info(
            f'里程计: pos=({pose.position.x:.2f},{pose.position.y:.2f}) '
            f'vel=({twist.linear.x:.2f}m/s)',
            throttle_duration_sec=2.0
        )

def main():
    rclpy.init()
    node = SensorMonitor()
    rclpy.spin(node)
    rclpy.shutdown()
```

### 10.3.2 TF坐标变换体系

SLAM系统依赖TF坐标变换树来关联不同传感器和本体坐标系：

```
map → odom → base_footprint → base_link → laser
```

各坐标系含义：
- **map（地图坐标系）：** 全局世界坐标系，SLAM构建地图所在的固定参考系
- **odom（里程计坐标系）：** 机器人起始位置，由里程计连续更新，局部精度高
- **base_footprint（机器人足迹坐标系）：** 机器人在地面的投影，常作为机器人本体坐标系
- **base_link（机器人基座坐标系）：** 机器人本体的中心坐标系
- **laser（激光雷达坐标系）：** 激光传感器的坐标系，通常位于机器人顶部

map→odom的变换由SLAM/AMCL持续更新，以消除里程计累积误差。

### 10.3.3 TF树在SLAM中的作用

```python
import rclpy
from rclpy.node import Node
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped, PointStamped

class TFMonitor(Node):
    """TF变换监控与查询节点"""
    def __init__(self):
        super().__init__('tf_monitor')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.query_tf)
        
    def query_tf(self):
        try:
            # 查询 map → base_link 变换
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time()
            )
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            self.get_logger().info(f'map→base_link: ({x:.3f}, {y:.3f})')
            
            # 坐标变换：将激光点转换到map坐标系
            laser_point = PointStamped()
            laser_point.header.frame_id = 'laser'
            laser_point.point.x = 1.0
            laser_point.point.y = 0.0
            laser_point.point.z = 0.0
            
            map_point = self.tf_buffer.transform(laser_point, 'map')
            self.get_logger().info(
                f'激光点(1,0,0)在map下: '
                f'({map_point.point.x:.3f}, {map_point.point.y:.3f})'
            )
        except Exception as e:
            self.get_logger().warn(f'TF查询失败: {e}')
            
    def can_transform_check(self, source: str, target: str) -> bool:
        """检查TF变换是否可用"""
        try:
            self.tf_buffer.can_transform(target, source, 
                                          rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1))
            return True
        except:
            return False
```

### 10.3.4 传感器标定与时间同步

SLAM系统中多传感器的时间同步至关重要：

```bash
# 检查传感器话题频率
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic hz /imu

# 检查传感器时间戳
ros2 topic echo /scan --once | grep stamp
ros2 topic echo /odom --once | grep stamp

# 查看TF树
ros2 run tf2_tools view_frames.py

# 查看TF广播关系
ros2 run tf2_ros tf2_monitor
```

## 10.4 SLAM方法分类

### 10.4.1 基于滤波的方法

基于滤波的SLAM方法使用概率滤波器（EKF、粒子滤波等）在线递归估计机器人位姿和地图。

**代表算法：**
| 算法 | 滤波器类型 | 特点 |
|------|-----------|------|
| EKF-SLAM | 扩展卡尔曼滤波 | 最早的方法，计算O(n²) |
| FastSLAM 1.0/2.0 | Rao-Blackwellized粒子滤波 | 粒子表示路径，EKF维护地图 |
| gmapping | RBPF + 自适应提议分布 | 广泛使用的2D SLAM |
| UKF-SLAM | 无迹卡尔曼滤波 | 处理强非线性 |

**优点：**
- 在线计算，实时性强
- 自然处理不确定性
- 易于嵌入控制回路

**缺点：**
- 线性化误差（EKF）
- 粒子退化问题（粒子滤波）
- 对大规模环境扩展性有限

### 10.4.2 基于优化的方法

基于优化的方法将SLAM构建为图优化问题，通过最小化误差函数求解。

**代表算法：**
| 算法 | 优化方法 | 特点 |
|------|----------|------|
| GraphSLAM | 稀疏矩阵分解 | 全图优化，离线批处理 |
| Cartographer | Ceres Solver | 子图和回环检测，Google开源 |
| SLAM Toolbox | Ceres Solver | ROS2官方2D SLAM工具 |
| KartoSLAM | SPA (Sparse Pose Adjustment) | 基于图优化 |

**图优化框架：**
```
节点 (Vertex):    边 (Edge):
- 机器人位姿 x_t   - 里程计约束 (连续位姿间)
- 路标点 l_j      - 观测约束 (位姿-路标点)
                  - 回环约束 (非连续位姿间)
```

```python
import numpy as np

class GraphSLAM:
    """GraphSLAM图优化SLAM框架"""
    def __init__(self):
        self.poses = []        # 机器人位姿序列
        self.landmarks = {}    # 路标点字典 {id: (x, y)}
        self.constraints = []  # 约束列表
        self.landmark_id_counter = 0
        
    def add_odometry_constraint(self, pose_i: int, pose_j: int, 
                                  transform: np.ndarray, information: np.ndarray):
        """添加里程计约束"""
        self.constraints.append({
            'type': 'odometry',
            'i': pose_i,
            'j': pose_j,
            'measurement': transform,
            'information': information,
        })
        
    def add_observation_constraint(self, pose_id: int, landmark_id: int,
                                     observation: np.ndarray, information: np.ndarray):
        """添加观测约束"""
        self.constraints.append({
            'type': 'observation',
            'pose': pose_id,
            'landmark': landmark_id,
            'measurement': observation,
            'information': information,
        })
        
    def add_new_landmark(self, position: np.ndarray) -> int:
        """添加新路标点"""
        idx = self.landmark_id_counter
        self.landmarks[idx] = position
        self.landmark_id_counter += 1
        return idx
        
    def build_linear_system(self):
        """构建线性系统 H·Δx = b"""
        n = 3 * len(self.poses) + 2 * len(self.landmarks)
        H = np.zeros((n, n))
        b = np.zeros(n)
        
        for constraint in self.constraints:
            if constraint['type'] == 'odometry':
                i = constraint['i']
                j = constraint['j']
                meas = constraint['measurement']
                info = constraint['information']
                
                # 误差计算和线性化
                xi = self.poses[i] if i < len(self.poses) else np.zeros(3)
                xj = self.poses[j] if j < len(self.poses) else np.zeros(3)
                
                # 残差
                error = self.compute_odometry_error(xi, xj, meas)
                
                # 雅可比矩阵
                Ji = np.eye(3)
                Jj = -np.eye(3)
                
                # 添加到Hessian矩阵
                idx_i = 3 * i
                idx_j = 3 * j
                H[idx_i:idx_i+3, idx_i:idx_i+3] += Ji.T @ info @ Ji
                H[idx_i:idx_i+3, idx_j:idx_j+3] += Ji.T @ info @ Jj
                H[idx_j:idx_j+3, idx_i:idx_i+3] += Jj.T @ info @ Ji
                H[idx_j:idx_j+3, idx_j:idx_j+3] += Jj.T @ info @ Jj
                b[idx_i:idx_i+3] += Ji.T @ info @ error
                b[idx_j:idx_j+3] += Jj.T @ info @ error
                
        return H, b
    
    def compute_odometry_error(self, xi, xj, meas):
        """计算里程计误差"""
        return meas - (xj - xi)
    
    def optimize(self):
        """求解图优化"""
        H, b = self.build_linear_system()
        # 固定第一个位姿（固定map坐标系）
        H[:3, :] = 0
        H[:, :3] = 0
        H[:3, :3] = np.eye(3)
        b[:3] = 0
        
        # 求解线性系统
        delta = np.linalg.solve(H + 1e-6 * np.eye(len(H)), b)
        
        # 更新位姿
        for i in range(len(self.poses)):
            self.poses[i] += delta[3*i:3*i+3]
```

### 10.4.3 基于视觉的方法

视觉SLAM使用相机作为主要传感器，提取图像特征进行定位和建图。

**代表算法：**
- **ORB-SLAM2/3:** 特征法视觉SLAM，三线程架构
- **DSO (Direct Sparse Odometry):** 直接法，利用像素亮度
- **SVO (Semi-direct Visual Odometry):** 半直接法
- **VINS-Mono:** 视觉惯性融合SLAM

### 10.4.4 混合方法

现代SLAM系统往往结合多种方法的优势：

| 方法 | 代表 | 前端 | 后端 |
|------|------|------|------|
| 激光+IMU融合 | Cartographer | 扫描匹配 | 图优化 |
| 视觉+惯性融合 | VINS-Fusion | 特征跟踪 | 滑窗BA |
| 激光+视觉融合 | LVI-SAM | 多传感器前端 | 因子图优化 |

### 10.4.5 方法选择指南

选择SLAM算法时需考虑以下因素：

```python
def suggest_slam_algorithm(
    environment: str = 'indoor',
    has_odom: bool = True,
    has_imu: bool = False,
    sensor_type: str = 'lidar2d',
    precision_requirement: str = 'high',
    realtime: bool = True
) -> str:
    """根据场景推荐SLAM算法"""
    if sensor_type == 'lidar2d':
        if environment == 'indoor' and not has_odom:
            return 'Hector-SLAM (不依赖里程计)'
        elif environment == 'indoor' and has_odom:
            return 'slam_toolbox (ROS2推荐) / gmapping'
        elif environment == 'large_scale':
            return 'Cartographer (大场景图优化)'
    elif sensor_type == 'lidar3d':
        return 'Cartographer 3D / A-LOAM'
    elif sensor_type == 'camera':
        if has_imu:
            return 'VINS-Fusion / ORB-SLAM3'
        else:
            return 'ORB-SLAM2 / DSO'
    elif sensor_type == 'mixed':
        return 'LVI-SAM / R3LIVE'
    return 'slam_toolbox (默认推荐)'
```

## X.5 贝叶斯框架的数学推导

### 10.5.1 贝叶斯定理回顾

贝叶斯定理是SLAM的概率基础：

```
p(A|B) = p(B|A) · p(A) / p(B)
```

在SLAM上下文中：
- `p(x_t, m | z_{1:t}, u_{1:t-1})`：后验概率（给定观测和控制的状态信念）
- `p(z_t | x_t, m)`：似然（给定位姿和地图的观测概率）
- `p(x_t, m | z_{1:t-1}, u_{1:t-1})`：先验（预测）

### 10.5.2 递归贝叶斯滤波推导

SLAM使用递归贝叶斯滤波，通过预测-更新循环实现：

**预测步骤：**
```
bel(x_t, m) = ∫ p(x_t | x_{t-1}, u_t) · bel(x_{t-1}, m) dx_{t-1}
```

**更新步骤：**
```
bel(x_t, m) = η · p(z_t | x_t, m) · bel(x_t, m)
```

### 10.5.3 独立性假设与简化

在FastSLAM中，利用条件独立性做出关键简化：

```
p(x_{1:t}, m | z_{1:t}, u_{1:t-1}) = 
p(m | x_{1:t}, z_{1:t}) · p(x_{1:t} | z_{1:t}, u_{1:t-1})
```

这意味着：
- 给定机器人轨迹，地图点条件独立
- 可先估计轨迹，再计算地图

### 10.5.4 信息形式 vs 协方差形式

SLAM中两种等价的概率表示：

**协方差形式（EKF-SLAM）：**
- 维护状态均值和协方差矩阵
- 更新复杂度O(n²)
- 直观表示不确定性

**信息形式（GraphSLAM）：**
- 维护信息矩阵(Ω)和信息向量(ξ)
- 稀疏信息矩阵利于大规模求解
- 使用迭代线性求解器

```python
import numpy as np
from scipy import sparse

class InformationFormSLAM:
    """信息形式的SLAM滤波器"""
    def __init__(self, num_landmarks: int = 100):
        self.dim = 3 + 2 * num_landmarks
        # 信息矩阵 (协方差的逆)
        self.Omega = sparse.eye(self.dim, format='csr') * 0.01
        # 信息向量
        self.xi = np.zeros(self.dim)
        self.num_landmarks = num_landmarks
        
    def predict(self, motion_info: np.ndarray):
        """信息形式的预测"""
        # 信息形式的预测相当于添加先验约束
        pass
        
    def update_with_measurement(self, z: np.ndarray, 
                                  landmark_id: int,
                                  R: np.ndarray):
        """信息形式的更新"""
        idx = 3 + 2 * landmark_id
        
        # 构建观测约束的信息贡献
        H = self._compute_jacobian(landmark_id)
        H_R = H.T @ np.linalg.inv(R) @ H
        
        # 更新信息矩阵
        self.Omega += H_R
        
        # 更新信息向量
        innovation = z - self._predict_observation(landmark_id)
        self.xi += H.T @ np.linalg.inv(R) @ innovation
        
    def get_mean(self) -> np.ndarray:
        """从信息形式恢复均值"""
        return sparse.linalg.spsolve(self.Omega, self.xi)
    
    def get_covariance(self) -> np.ndarray:
        """从信息形式恢复协方差"""
        return sparse.linalg.inv(self.Omega).toarray()
    
    def _compute_jacobian(self, landmark_id: int):
        """计算观测雅可比"""
        H = np.zeros((2, self.dim))
        # 简化实现
        idx = 3 + 2 * landmark_id
        H[0, 0] = -1.0
        H[0, idx] = 1.0
        H[1, 1] = -1.0
        H[1, idx + 1] = 1.0
        return H
    
    def _predict_observation(self, landmark_id: int) -> np.ndarray:
        """预测观测值"""
        mean = self.get_mean()
        robot_pose = mean[:3]
        landmark_pos = mean[3 + 2*landmark_id:3 + 2*landmark_id + 2]
        return landmark_pos - robot_pose[:2]
```

## 10.6 ROS2 SLAM工具链概览

### 10.6.1 ROS2 SLAM生态

ROS2提供了完整的SLAM工具链：

**slam_toolbox:**
- ROS2官方2D SLAM解决方案
- 基于图优化，支持在线/离线建图
- 替代ROS1中的gmapping和karto

**nav2_amcl:**
- 基于已知地图的粒子滤波定位
- KLD自适应采样
- 多种传感器模型

**cartographer_ros:**
- Google开源的多传感器SLAM
- 支持2D和3D建图
- 激光+IMU融合

### 10.6.2 常见启动命令

```bash
# slam_toolbox 在线异步建图
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=./config/slam_toolbox_params.yaml \
  use_sim_time:=true

# 课程 Cartographer 2D 建图
ros2 launch slam_lab cartographer_mapping.launch.py \
  configuration_directory:=src/lab_code/ch10_lab/slam_lab/config/cartographer \
  configuration_basename:=xbot_2d.lua \
  use_sim_time:=true

# Nav2 AMCL定位
ros2 launch nav2_bringup localization_launch.py \
  map:=./maps/my_map.yaml \
  use_sim_time:=true
```

### 10.6.3 数据录制与离线处理

```bash
# 录制传感器数据
ros2 bag record -o slam_data /scan /tf /tf_static /odom /imu

# 离线建图（使用rosbag）
ros2 launch slam_toolbox offline_launch.py \
  bag_filename:=slam_data use_sim_time:=true

# 评估建图结果
ros2 run nav2_map_server map_saver_cli -f final_map
```

## 课后练习

1. **原理题:** 简要说明SLAM问题的核心挑战，为什么定位和建图需要同时进行？用数学公式给出SLAM的后验概率表达。

2. **推导题:** 推导贝叶斯滤波的预测步骤和更新步骤公式，解释每个符号的物理含义。

3. **编程题:** 编写一个Python函数，实现二维空间中基于贝叶斯滤波的位姿估计，输入为里程计数据和激光观测数据，输出为位姿后验均值。

4. **配置题:** 在一个已知环境的rosbag上，使用slam_toolbox进行离线建图，写出完整的命令行流程。

5. **分析题:** 比较基于滤波的SLAM方法和基于优化的SLAM方法的优缺点，说明各自适用场景。

6. **设计题:** 某巡检机器人需要在2000m²的仓库环境中运行，配备2D激光雷达、轮式里程计和IMU。为其设计SLAM方案，包括算法选择、传感器融合策略和参数配置建议。
