# 第22章 多传感器融合SLAM

## 仿真结合实例（当前仓库）：同步 LiDAR、相机与里程计数据

### 目标与知识点对应

使用 `robot_sim_demo` 同时输出 `/scan`、`/camera/image_raw`、`/camera/camera_info`、`/odom` 和 `/tf`，检查多传感器融合前的时间戳、frame_id 和话题频率。

### 运行步骤

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_sim_demo gazebo2.launch.py \
  gui:=false rviz:=false drive:=true
```

```bash
ros2 topic hz /scan
ros2 topic hz /camera/image_raw
ros2 topic echo /camera/camera_info --once
ros2 topic echo /odom --once
```

### 观察结果

比较各消息的时间戳和 frame_id，识别融合节点需要的同步策略及 TF 外参；RViz 可分别显示激光和机器人 TF。

### 源码与边界

- Bridge：`src/robot_sim_demo/config/gazebo2_bridge.yaml`
- 相机内参：`src/robot_sim_demo/robot_sim_demo/camera_info_publisher.py`
- 模型：`src/robot_sim_demo/models/wheeltec_robot/model.sdf`

当前仓库没有 FAST-LIO/VINS 等完整融合算法，本例只验证输入和标定信息。

## 学习目标
- 理解多传感器融合SLAM的必要性和基本框架
- 掌握IMU+LiDAR融合原理与方法
- 掌握视觉+IMU融合（VIO）原理
- 熟悉Kalman滤波和图优化融合框架
- 了解实际工程中的多传感器融合案例

## 22.1 多传感器融合概述

### 22.1.1 为什么需要多传感器融合

单一传感器在SLAM中存在固有的局限性：

| 传感器 | 优势 | 局限性 |
|--------|------|--------|
| 激光雷达 | 精度高、不受光照影响 | 稀疏、无纹理、无语义 |
| 相机 | 信息丰富、低成本 | 受光照、弱纹理、模糊 |
| IMU | 高频、短期精度高 | 漂移大、无绝对参考 |
| 里程计 | 局部精度高 | 累积误差、打滑 |
| GPS | 全局无漂移 | 室内不可用、精度低 |

**多传感器融合的核心优势：**
- 互补性：不同传感器的优势互补
- 冗余性：某传感器失效时仍有其他传感器可用
- 鲁棒性：融合后可应对更广泛的环境条件
- 精度提升：多源信息约束提高估计精度

### 22.1.2 融合框架分类

**松耦合 (Loosely Coupled)：**
```
传感器各自独立处理 → 融合结果
- 各传感器运行各自的SLAM/里程计
- 在状态估计层面融合（EKF）
- 实现简单，模块化
- 精度相对较低
```

**紧耦合 (Tightly Coupled)：**
```
传感器原始数据 → 统一优化
- 在因子图/优化中统一处理各传感器约束
- 使用传感器原始测量（非处理后的位姿）
- 精度高，复杂度高
- 代表：LVI-SAM, MSCKF
```

## 22.2 IMU+LiDAR融合

### 22.2.1 IMU的作用

IMU在LiDAR SLAM中的关键作用：

1. **运动畸变校正：** 激光扫描过程中若机器人运动，导致点云畸变，IMU可校正
2. **位姿预测：** 提供高频位姿预测，作为扫描匹配的初值
3. **状态约束：** 重力方向约束，使地图水平
4. **快速运动恢复：** 激光匹配失败时，IMU可短暂维持定位

```python
import numpy as np

class IMUPreintegrator:
    """IMU预积分器"""
    def __init__(self):
        self.dt = 0.01  # IMU采样周期
        self.g = np.array([0, 0, -9.81])  # 重力
        
        # 预积分状态
        self.delta_p = np.zeros(3)   # 位置增量
        self.delta_v = np.zeros(3)   # 速度增量
        self.delta_R = np.eye(3)     # 旋转增量
        
        # 协方差
        self.covariance = np.zeros((9, 9))
        
    def integrate(self, acc: np.ndarray, gyro: np.ndarray):
        """
        IMU预积分
        
        参数:
            acc: 加速度计测量 (含重力)
            gyro: 陀螺仪测量
        """
        # 预积分更新（不考虑偏置）
        dt = self.dt
        
        # 旋转增量
        omega = self._skew(gyro)
        self.delta_R = self.delta_R @ (np.eye(3) + omega * dt)
        
        # 速度增量
        self.delta_v += self.delta_R @ acc * dt
        
        # 位置增量
        self.delta_p += (self.delta_v * dt + 
                         0.5 * self.delta_R @ acc * dt**2)
        
    def predict_state(self, p_i: np.ndarray, v_i: np.ndarray,
                       R_i: np.ndarray, 
                       bg: np.ndarray, ba: np.ndarray) -> tuple:
        """
        根据预积分预测状态
        
        参数:
            p_i, v_i, R_i: 时刻i的状态
            bg, ba: 陀螺仪和加速度计偏置
            
        返回:
            p_j, v_j, R_j: 时刻j的预测状态
        """
        # 预积分的偏置校正
        dR = self._correct_rotation(self.delta_R, bg)
        dv = self._correct_velocity(self.delta_v, ba)
        dp = self._correct_position(self.delta_p, ba)
        
        # 预测
        p_j = p_i + v_i * self._total_time + R_i @ dp + 0.5 * self.g * self._total_time**2
        v_j = v_i + R_i @ dv + self.g * self._total_time
        R_j = R_i @ dR
        
        return p_j, v_j, R_j
    
    def _skew(self, v: np.ndarray) -> np.ndarray:
        """向量→反对称矩阵"""
        return np.array([[0, -v[2], v[1]],
                        [v[2], 0, -v[0]],
                        [-v[1], v[0], 0]])
    
    def _correct_rotation(self, dR: np.ndarray, bg: np.ndarray) -> np.ndarray:
        """旋转的偏置校正（一阶近似）"""
        return dR @ self._skew(-bg * self._total_time)
    
    def _correct_velocity(self, dv: np.ndarray, ba: np.ndarray) -> np.ndarray:
        return dv + ba * self._total_time
    
    def _correct_position(self, dp: np.ndarray, ba: np.ndarray) -> np.ndarray:
        return dp + 0.5 * ba * self._total_time**2
    
    @property
    def _total_time(self):
        return self.dt
```

### 22.2.2 LiDAR-IMU标定

LiDAR和IMU之间的外参标定是融合的前提：

```python
class LidarIMUCalibration:
    """LiDAR-IMU外参标定"""
    def __init__(self):
        # 待标定参数
        self.R_lidar_imu = np.eye(3)   # 旋转外参
        self.t_lidar_imu = np.zeros(3)  # 平移外参
        self.time_offset = 0.0          # 时间偏移
        
    def calibrate_rotation(self, lidar_poses: list, 
                             imu_rotations: list) -> np.ndarray:
        """
        通过旋转约束标定旋转外参
        
        参数:
            lidar_poses: LiDAR位姿序列
            imu_rotations: IMU旋转序列
            
        返回:
            R_lidar_imu: 优化的旋转外参
        """
        # 构建约束：R_imu = R_lidar @ R_lidar_imu
        A = []
        b = []
        
        for i in range(1, len(lidar_poses)):
            # LiDAR的相对旋转
            R_lidar_rel = lidar_poses[i][:3, :3] @ lidar_poses[i-1][:3, :3].T
            
            # IMU的相对旋转
            R_imu_rel = imu_rotations[i] @ imu_rotations[i-1].T
            
            # 约束：R_imu_rel = R_lidar_imu @ R_lidar_rel @ R_lidar_imu.T
            # 线性化后构建最小二乘问题
            # ...
            
        # SVD求解
        # ...
        return self.R_lidar_imu
    
    def calibrate_translation(self, lidar_poses: list,
                               imu_data: list) -> np.ndarray:
        """通过加速度约束标定平移外参"""
        # 利用杠杆臂效应：IMU加速度包含LiDAR角速度引起的向心加速度
        # a_imu = R_lidar_imu^T @ (a_lidar + w_lidar × (w_lidar × t) + w_dot × t)
        # ...
        return self.t_lidar_imu
```

### 22.2.3 FAST-LIO2框架

FAST-LIO2是经典的LiDAR-IMU紧耦合SLAM系统：

```python
import numpy as np

class FASTLIO2:
    """FAST-LIO2核心框架（简化）"""
    def __init__(self):
        # 状态向量
        self.state = {
            'position': np.zeros(3),
            'velocity': np.zeros(3),
            'rotation': np.eye(3),
            'gyro_bias': np.zeros(3),
            'acc_bias': np.zeros(3)
        }
        
        # 协方差
        self.cov = np.eye(15) * 0.01
        
        # 地图（ikd-Tree）
        self.map = None
        
        # 参数
        self.lidar_freq = 10.0
        self.imu_freq = 200.0
        
    def imu_callback(self, acc: np.ndarray, gyro: np.ndarray, 
                      timestamp: float):
        """IMU数据处理 - 状态预测"""
        dt = 1.0 / self.imu_freq
        
        # 去除偏置
        acc_corrected = acc - self.state['acc_bias']
        gyro_corrected = gyro - self.state['gyro_bias']
        
        # 运动学预测
        R = self.state['rotation']
        
        # 位置
        self.state['position'] += self.state['velocity'] * dt
        
        # 速度
        self.state['velocity'] += (R @ acc_corrected + np.array([0, 0, -9.81])) * dt
        
        # 姿态
        omega = np.array([[0, -gyro_corrected[2], gyro_corrected[1]],
                          [gyro_corrected[2], 0, -gyro_corrected[0]],
                          [-gyro_corrected[1], gyro_corrected[0], 0]])
        self.state['rotation'] = R @ (np.eye(3) + omega * dt)
        
        # 协方差预测
        self._predict_covariance(dt)
        
    def lidar_callback(self, points: np.ndarray, timestamp: float):
        """LiDAR数据处理 - 状态更新"""
        # 1. 运动畸变校正（使用IMU预测）
        corrected_points = self._undistort_points(points)
        
        # 2. 特征提取
        features = self._extract_features(corrected_points)
        
        # 3. 扫描-地图匹配
        H, z = self._scan_to_map_matching(features)
        
        # 4. Kalman更新
        self._kalman_update(H, z)
        
        # 5. 地图更新
        self._update_map(corrected_points)
        
    def _extract_features(self, points: np.ndarray) -> dict:
        """特征提取（边缘点+平面点）"""
        # 计算曲率
        curvatures = []
        for i, point in enumerate(points):
            neighbors = points[max(0,i-5):min(len(points),i+5)]
            curv = np.linalg.norm(point - neighbors.mean(axis=0))
            curvatures.append(curv)
            
        curvatures = np.array(curvatures)
        
        # 边缘点（高曲率）
        edge_threshold = np.percentile(curvatures, 80)
        edge_points = points[curvatures > edge_threshold]
        
        # 平面点（低曲率）
        flat_threshold = np.percentile(curvatures, 30)
        flat_points = points[curvatures < flat_threshold]
        
        return {
            'edge': edge_points,
            'flat': flat_points
        }
    
    def _undistort_points(self, points: np.ndarray) -> np.ndarray:
        """运动畸变校正"""
        # 使用IMU预测的姿态序列
        corrected = points.copy()
        # 对每个点根据其扫描时间补偿旋转和平移
        return corrected
    
    def _scan_to_map_matching(self, features: dict) -> tuple:
        """扫描-地图匹配"""
        H = np.zeros((6, 15))  # 观测雅可比
        z = np.zeros(6)        # 残差
        return H, z
    
    def _kalman_update(self, H: np.ndarray, z: np.ndarray):
        """Kalman状态更新"""
        # Kalman增益
        P = self.cov
        R = np.eye(len(z)) * 0.1  # 观测噪声
        
        S = H @ P @ H.T + R
        K = P @ H.T @ np.linalg.inv(S)
        
        # 状态更新
        dx = K @ z
        self._update_state(dx)
        
        # 协方差更新
        I_KH = np.eye(len(dx)) - K @ H
        self.cov = I_KH @ P @ I_KH.T + K @ R @ K.T
        
    def _update_state(self, dx: np.ndarray):
        """更新状态向量"""
        self.state['position'] += dx[0:3]
        self.state['velocity'] += dx[3:6]
        
        # 旋转更新（使用so3指数映射）
        omega = dx[6:9]
        theta = np.linalg.norm(omega)
        if theta > 1e-6:
            n = omega / theta
            K = np.array([[0, -n[2], n[1]],
                          [n[2], 0, -n[0]],
                          [-n[1], n[0], 0]])
            dR = np.eye(3) + np.sin(theta) * K + (1-np.cos(theta)) * K @ K
            self.state['rotation'] = self.state['rotation'] @ dR
            
        self.state['gyro_bias'] += dx[9:12]
        self.state['acc_bias'] += dx[12:15]
    
    def _predict_covariance(self, dt: float):
        """协方差预测"""
        F = np.eye(15)  # 状态转移矩阵
        Q = np.eye(15) * 0.001  # 过程噪声
        
        self.cov = F @ self.cov @ F.T + Q
```

## 22.3 视觉+IMU融合(VIO)

### 22.3.1 VIO算法分类

| 算法 | 方法 | 特点 |
|------|------|------|
| MSCKF | 多状态约束KF | 计算量小，不优化地图点 |
| VINS-Mono | 滑窗优化+IMU预积分 | 精度高，单目VIO |
| OKVIS | 滑窗BA+IMU | 关键帧滑窗优化 |
| ROVIO | 扩展卡尔曼滤波 | 直接法+IMU |
| ORB-SLAM3 | 视觉SLAM+IMU | 支持视觉惯性模式 |

### 22.3.2 VINS-Mono框架

```python
import numpy as np

class VINSMono:
    """VINS-Mono核心框架"""
    def __init__(self):
        # 滑窗
        self.window_size = 10
        self.frame_buffer = []
        
        # IMU预积分器
        self.preintegrator = IMUPreintegrator()
        
        # 状态
        self.velocity = np.zeros(3)
        self.gravity = np.array([0, 0, -9.81])
        self.gyro_bias = np.zeros(3)
        self.acc_bias = np.zeros(3)
        
        # 特征点
        self.features = {}
        
    def process_image(self, image: np.ndarray, timestamp: float):
        """处理图像帧"""
        # 1. 特征提取和跟踪
        new_features = self._track_features(image)
        
        # 2. 添加到帧缓冲
        self.frame_buffer.append({
            'timestamp': timestamp,
            'features': new_features,
            'preintegration': self.preintegrator.clone()
        })
        
        # 3. 滑窗优化
        if len(self.frame_buffer) >= self.window_size:
            self._sliding_window_optimization()
            
        # 4. 边缘化
        self._marginalize()
        
    def _track_features(self, image: np.ndarray) -> dict:
        """KLT光流特征跟踪"""
        # 使用OpenCV的calcOpticalFlowPyrLK
        # ...
        return {}
    
    def _sliding_window_optimization(self):
        """滑窗BA优化"""
        # 构建因子图
        # IMU因子（预积分约束）
        # 视觉因子（重投影约束）
        # 先验因子（边缘化信息）
        
        # Ceres / g2o 优化
        pass
    
    def _marginalize(self):
        """边缘化旧帧"""
        if len(self.frame_buffer) > self.window_size:
            oldest = self.frame_buffer.pop(0)
            # 舒尔补边缘化
            pass
```

### 22.3.3 视觉-惯性BA

```python
class VisualInertialBA:
    """视觉-惯性光束平差法"""
    def __init__(self):
        pass
        
    def build_factor_graph(self, frames: list, 
                           landmarks: dict) -> dict:
        """构建视觉-惯性因子图"""
        factors = {
            'imu': [],      # IMU预积分约束
            'visual': [],   # 视觉重投影约束
            'prior': []     # 先验约束
        }
        
        # IMU因子
        for i in range(len(frames) - 1):
            if 'preintegration' in frames[i+1]:
                factors['imu'].append({
                    'i': i,
                    'j': i + 1,
                    'preint': frames[i+1]['preintegration']
                })
                
        # 视觉因子
        for lm_id, lm in landmarks.items():
            for frame_id, obs in lm['observations'].items():
                factors['visual'].append({
                    'frame': frame_id,
                    'landmark': lm_id,
                    'observation': obs
                })
                
        return factors
    
    def optimize(self, factors: dict, 
                 initial_states: dict) -> dict:
        """执行视觉-惯性BA优化"""
        # 使用非线性最小二乘求解器
        # (ceres, g2o, gtsam)
        
        # 残差包括:
        # 1. IMU预积分残差
        # 2. 视觉重投影残差
        # 3. 先验残差
        
        # ...
        return initial_states
```

## 22.4 Kalman滤波与图优化融合框架

### 22.4.1 多传感器EKF融合

扩展卡尔曼滤波是多传感器融合的基础框架：

```python
import numpy as np

class MultiSensorEKF:
    """多传感器EKF融合"""
    def __init__(self, dim_state: int = 15):
        self.dim_state = dim_state
        self.x = np.zeros(dim_state)  # [位置,速度,姿态,偏置...]
        self.P = np.eye(dim_state) * 0.1
        self.Q = np.eye(dim_state) * 0.01  # 过程噪声
        self.R = {}  # 各传感器的观测噪声
        
    def predict(self, dt: float, control: np.ndarray = None):
        """状态预测"""
        # 状态转移矩阵（简化为常速模型）
        F = np.eye(self.dim_state)
        F[0:3, 3:6] = np.eye(3) * dt
        
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q
        
    def update_imu(self, acc: np.ndarray, gyro: np.ndarray):
        """IMU观测更新"""
        # IMU测量模型
        z = np.concatenate([acc, gyro])
        H = self._compute_imu_jacobian()
        R = np.eye(6) * 0.1
        
        self._kalman_update(z, H, R, 'imu')
        
    def update_lidar_odom(self, delta_pose: np.ndarray):
        """激光里程计更新"""
        z = delta_pose
        H = np.zeros((6, self.dim_state))
        H[:6, :6] = np.eye(6)
        R = np.eye(6) * 0.05
        
        self._kalman_update(z, H, R, 'lidar')
        
    def update_visual_odom(self, delta_pose: np.ndarray):
        """视觉里程计更新"""
        z = delta_pose
        H = np.zeros((6, self.dim_state))
        H[:6, :6] = np.eye(6)
        R = np.eye(6) * 0.2  # 视觉噪声大于激光
        
        self._kalman_update(z, H, R, 'visual')
        
    def update_gps(self, position: np.ndarray):
        """GPS更新"""
        z = position
        H = np.zeros((3, self.dim_state))
        H[:3, :3] = np.eye(3)
        R = np.eye(3) * 1.0  # GPS噪声较大
        
        self._kalman_update(z, H, R, 'gps')
        
    def _kalman_update(self, z: np.ndarray, H: np.ndarray,
                        R: np.ndarray, sensor_name: str):
        """通用Kalman更新步骤"""
        # 观测残差
        z_pred = H @ self.x
        residual = z - z_pred
        
        # Kalman增益
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # 状态更新
        self.x = self.x + K @ residual
        self.P = (np.eye(self.dim_state) - K @ H) @ self.P
        
    def _compute_imu_jacobian(self) -> np.ndarray:
        """计算IMU观测雅可比"""
        H = np.zeros((6, self.dim_state))
        # 加速度计部分与姿态相关
        # 陀螺仪部分与角速度相关
        return H
```

### 22.4.2 因子图优化(GTSAM)

现代SLAM系统广泛使用因子图进行多传感器融合：

```python
# 伪代码：使用GTSAM进行因子图优化
from gtsam import *

class FactorGraphFusion:
    """因子图多传感器融合"""
    def __init__(self):
        self.graph = NonlinearFactorGraph()
        self.initial = Values()
        self.result = None
        
    def add_prior_factor(self, key: int, pose: np.ndarray,
                          noise: np.ndarray):
        """添加先验因子"""
        prior_noise = noise_model.Diagonal.Sigmas(noise)
        self.graph.add(PriorFactorPose2(key, pose, prior_noise))
        
    def add_odometry_factor(self, key_i: int, key_j: int,
                              delta: np.ndarray, noise: np.ndarray):
        """添加里程计因子（激光/视觉/轮式）"""
        odom_noise = noise_model.Diagonal.Sigmas(noise)
        self.graph.add(BetweenFactorPose2(
            key_i, key_j, delta, odom_noise))
        
    def add_imu_factor(self, key_i: int, key_j: int,
                        preintegrated: dict, noise: np.ndarray):
        """添加IMU预积分因子"""
        # IMU因子（需要gtsam的PreintegratedImuMeasurements）
        imu_noise = noise_model.Diagonal.Sigmas(noise)
        # self.graph.add(ImuFactor(...))
        
    def add_loop_closure_factor(self, key_i: int, key_j: int,
                                  relative_pose: np.ndarray,
                                  noise: np.ndarray):
        """添加回环因子"""
        lc_noise = noise_model.Diagonal.Sigmas(noise)
        self.graph.add(BetweenFactorPose2(
            key_i, key_j, relative_pose, lc_noise))
        
    def optimize(self, num_iterations: int = 100):
        """执行优化"""
        optimizer = LevenbergMarquardtOptimizer(
            self.graph, self.initial)
        self.result = optimizer.optimize()
        return self.result
```

## 22.5 实际工程案例

### 22.5.1 仓库AGV融合SLAM案例

```python
class WarehouseAGVFusion:
    """仓库AGV多传感器融合SLAM"""
    def __init__(self):
        # 传感器
        self.lidar = '2D激光雷达 × 2（前后）'
        self.imu = '6轴IMU'
        self.encoder = '轮式编码器'
        self.rfid = 'RFID标签（辅助定位）'
        
        # 融合策略
        self.fusion = MultiSensorEKF()
        
    def fusion_pipeline(self):
        """融合处理流程"""
        steps = """
        ┌─────────────────────────────────────────────┐
        │  传感器输入                                   │
        ├────────────┬────────────┬────────────────────┤
        │ 激光雷达    │    IMU     │  轮式编码器         │
        │ 10Hz       │  200Hz     │  100Hz             │
        ├────────────┴────────────┴────────────────────┤
        │  传感器预处理                                  │
        │  - 激光：体素滤波、特征提取                      │
        │  - IMU：预积分、偏置估计                        │
        │  - 编码器：速度积分                            │
        ├──────────────────────────────────────────────┤
        │  多传感器EKF融合                               │
        │  - 高频IMU传播 (200Hz)                        │
        │  - 激光匹配更新 (10Hz)                         │
        │  - 编码器约束更新 (100Hz)                      │
        │  - RFID绝对修正 (1Hz)                         │
        ├──────────────────────────────────────────────┤
        │  输出                                         │
        │  - 位姿估计: 100Hz                            │
        │  - 占据栅格地图: /map                         │
        │  - 里程计: /odom                              │
        └──────────────────────────────────────────────┘
        """
        return steps
    
    def parameter_recommendation(self) -> dict:
        """参数推荐"""
        return {
            'laser_noise': 0.02,     # 激光噪声(m)
            'imu_acc_noise': 0.01,   # IMU加速度噪声(m/s²)
            'imu_gyro_noise': 0.001, # IMU角速度噪声(rad/s)
            'encoder_noise': 0.05,   # 编码器噪声(m)
            'rfid_noise': 0.3,       # RFID噪声(m)
            'fusion_frequency': 100, # 融合输出频率(Hz)
            'map_resolution': 0.05,  # 地图分辨率(m)
        }
```

### 22.5.2 自动驾驶融合定位案例

```python
class AutonomousVehicleLocalization:
    """自动驾驶多传感器融合定位"""
    def __init__(self):
        self.sensors = {
            'gps': {"精度": "3m (单点) / 0.1m (RTK)", "频率": "10Hz"},
            'imu': {"精度": "0.01°/s (光纤陀螺)", "频率": "200Hz"},
            'lidar_3d': {"精度": "0.03m", "频率": "10Hz"},
            'camera': {"精度": "像素级", "频率": "30Hz"},
            'wheel_encoder': {"精度": "0.001m (脉冲)", "频率": "100Hz"},
        }
        
    def localization_strategy(self) -> str:
        """多模式定位策略"""
        return """
        1. 城区道路:
           GPS RTK + IMU + 轮式里程计
           (RTK可用时：cm级精度)
           
        2. 隧道/地下:
           LiDAR SLAM + IMU + 轮式里程计
           (无GPS信号，纯SLAM)
           
        3. 高速路段:
           GPS + IMU 松耦合
           (高动态，SLAM可靠性下降)
           
        4. 多楼层停车:
           LiDAR SLAM + 视觉回环
           (楼层间地图切换)
        """
```

### 22.5.3 传感器时间同步

多传感器融合的时间同步是关键工程问题：

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan, Image
import numpy as np

class SensorTimeSynchronizer(Node):
    """多传感器时间同步器"""
    def __init__(self):
        super().__init__('sensor_sync')
        
        # 时间戳对齐
        self.approximate_sync = ApproximateTimeSynchronizer(
            [self.create_subscription(Imu, '/imu', self.imu_callback, 10),
             self.create_subscription(LaserScan, '/scan', self.scan_callback, 10),
             self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)],
            queue_size=10,
            slop=0.05  # 最大时间差50ms
        )
        self.approximate_sync.registerCallback(self.sync_callback)
        
        # 时间偏移估计
        self.time_offsets = {
            'imu_lidar': [],
            'lidar_camera': []
        }
        
    def sync_callback(self, imu_msg: Imu, scan_msg: LaserScan, 
                       image_msg: Image):
        """同步后的传感器数据"""
        # 此处三个传感器时间戳已对齐（误差<50ms）
        imu_time = imu_msg.header.stamp.sec + imu_msg.header.stamp.nanosec * 1e-9
        scan_time = scan_msg.header.stamp.sec + scan_msg.header.stamp.nanosec * 1e-9
        image_time = image_msg.header.stamp.sec + image_msg.header.stamp.nanosec * 1e-9
        
        self.get_logger().info(
            f'传感器同步: IMU={imu_time:.3f} '
            f'LiDAR={scan_time:.3f} '
            f'Camera={image_time:.3f}'
        )
        
        # 硬件同步触发（如果支持）
        self.hardware_trigger()
        
    def estimate_time_offset(self):
        """估计传感器间时间偏移"""
        # 使用互相关方法
        if len(self.time_offsets['imu_lidar']) > 100:
            mean_offset = np.mean(self.time_offsets['imu_lidar'])
            std_offset = np.std(self.time_offsets['imu_lidar'])
            self.get_logger().info(
                f'IMU-LiDAR时间偏移: {mean_offset*1000:.2f}ms ± {std_offset*1000:.2f}ms'
            )
    
    def hardware_trigger(self):
        """硬件同步触发"""
        # 如果传感器支持硬件触发线
        # 通过GPIO发送同步脉冲
        pass
```

### 22.5.4 故障检测与切换

```python
class SensorHealthMonitor:
    """传感器健康监控与自动切换"""
    def __init__(self):
        self.sensor_status = {
            'lidar': {'healthy': True, 'last_ok': 0},
            'camera': {'healthy': True, 'last_ok': 0},
            'imu': {'healthy': True, 'last_ok': 0},
            'gps': {'healthy': True, 'last_ok': 0}
        }
        
        # 故障阈值
        self.timeout = {
            'lidar': 0.5,    # 500ms无数据
            'camera': 1.0,   # 1s无数据
            'imu': 0.2,      # 200ms无数据
            'gps': 2.0       # 2s无数据
        }
        
    def check_health(self, sensor: str, timestamp: float):
        """检查传感器健康状态"""
        status = self.sensor_status[sensor]
        time_since_last = timestamp - status['last_ok']
        
        if time_since_last > self.timeout[sensor]:
            if status['healthy']:
                status['healthy'] = False
                print(f'警告: {sensor} 故障 (超时{time_since_last:.1f}s)')
                self.handle_failure(sensor)
        else:
            if not status['healthy']:
                print(f'恢复: {sensor} 已恢复')
                status['healthy'] = True
            status['last_ok'] = timestamp
            
    def handle_failure(self, failed_sensor: str):
        """处理传感器故障"""
        strategies = {
            'lidar': '切换到视觉+IMU紧耦合模式',
            'camera': '切换到激光+IMU模式，禁用视觉回环',
            'imu': '切换到轮式里程计+激光模式，降低动态性能',
            'gps': '切换到纯SLAM定位模式'
        }
        
        print(f'故障处理: {strategies.get(failed_sensor, "未知传感器")}')
```

## 课后练习

1. **原理题:** 比较松耦合和紧耦合多传感器融合的异同，分析为什么紧耦合通常能获得更高的精度。

2. **编程题:** 实现一个简化的EKF多传感器融合系统，融合激光里程计和IMU数据，输出融合后的位姿估计。

3. **分析题:** 分析IMU在LiDAR SLAM中的三个重要作用：运动畸变校正、位姿预测、重力约束，说明各自的原理和实现方式。

4. **配置题:** 在Cartographer中配置激光+IMU+里程计的多传感器融合建图，写出对应的LUA配置文件。

5. **操作题:** 录制包含激光、IMU和里程计数据的rosbag，使用FAST-LIO2或同类算法进行多传感器融合SLAM建图，评估结果。

6. **设计题:** 设计一个面向机场候机楼的清洁机器人SLAM方案，环境特征包括：大面积玻璃幕墙（激光穿透）、高挑空间（视觉特征少）、金属结构（IMU干扰）。给出传感器选型、融合策略和应急处理方案。
