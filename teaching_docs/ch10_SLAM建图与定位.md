# 第10章 SLAM 建图与定位

## 10.1 知识要点

### 10.1.1 SLAM 基本原理

SLAM (Simultaneous Localization and Mapping) 是机器人同时进行自身定位和环境地图构建的核心技术。

**贝叶斯滤波框架:**
```
p(x_{1:t}, m | z_{1:t}, u_{1:t-1})
```
其中 x 为位姿, m 为地图, z 为观测, u 为控制输入。

### 10.1.2 GraphSLAM 图优化

GraphSLAM 将 SLAM 问题建模为图优化问题：
- **节点 (Vertex):** 机器人位姿 x_t 和路标点 l_j
- **边 (Edge):** 里程计约束和观测约束
- **优化目标:** 最小化所有约束误差

```python
# GraphSLAM 图构建伪代码
def build_graph(poses, odometry, observations):
    graph = g2o.SparseOptimizer()
    # 添加位姿顶点
    for i, pose in enumerate(poses):
        v = g2o.VertexSE2()
        v.set_estimate(pose)
        graph.add_vertex(v)
    # 添加里程计边 (连续位姿间约束)
    for i in range(len(poses) - 1):
        edge = g2o.EdgeSE2()
        edge.set_measurement(odometry[i])
        edge.set_vertex(0, graph.vertex(i))
        edge.set_vertex(1, graph.vertex(i + 1))
        edge.set_information(np.eye(3) * info_odom)
        graph.add_edge(edge)
    # 添加观测边 (位姿-路标点约束)
    for obs in observations:
        edge = g2o.EdgeSE2PointXY()
        edge.set_measurement(obs.measurement)
        graph.add_edge(edge)
    graph.optimize()
    return graph
```

### 10.1.3 粒子滤波 SLAM (FastSLAM)

FastSLAM 使用 Rao-Blackwellized 粒子滤波，将 SLAM 分解为：
- 粒子滤波器估计机器人路径
- 每个粒子维护独立的 EKF 地图

```python
import numpy as np

class FastSLAMParticle:
    """FastSLAM 单个粒子实现"""
    def __init__(self, num_landmarks: int):
        self.pose = np.zeros(3)           # x, y, theta
        self.weight = 1.0
        # 每个路标点的 EKF: mean[2], cov[2,2]
        self.landmarks_mean = np.zeros((num_landmarks, 2))
        self.landmarks_cov = np.zeros((num_landmarks, 2, 2))
        self.landmarks_seen = [False] * num_landmarks

    def predict(self, control: np.ndarray, noise: np.ndarray):
        """运动模型预测"""
        self.pose[0] += control[0] * np.cos(self.pose[2]) + np.random.normal(0, noise[0])
        self.pose[1] += control[0] * np.sin(self.pose[2]) + np.random.normal(0, noise[1])
        self.pose[2] += control[1] + np.random.normal(0, noise[2])

    def update(self, z: np.ndarray, j: int, sensor_noise: np.ndarray):
        """EKF 更新单个路标点"""
        if not self.landmarks_seen[j]:
            # 初始化新路标点
            self.landmarks_mean[j] = self.pose[:2] + z
            self.landmarks_cov[j] = np.diag(sensor_noise[:2])
            self.landmarks_seen[j] = True
        else:
            # EKF 更新已知路标点
            H = np.eye(2)
            S = H @ self.landmarks_cov[j] @ H.T + np.diag(sensor_noise[:2])
            K = self.landmarks_cov[j] @ H.T @ np.linalg.inv(S)
            self.landmarks_mean[j] += K @ (z - self.landmarks_mean[j] + self.pose[:2])
            self.landmarks_cov[j] = (np.eye(2) - K @ H) @ self.landmarks_cov[j]


class FastSLAM:
    """FastSLAM 2.0 粒子滤波 SLAM"""
    def __init__(self, num_particles: int = 50, num_landmarks: int = 100):
        self.particles = [FastSLAMParticle(num_landmarks) for _ in range(num_particles)]
        self.num_particles = num_particles

    def resample(self):
        """低方差重采样"""
        weights = np.array([p.weight for p in self.particles])
        weights /= weights.sum()
        indices = np.random.choice(
            self.num_particles, self.num_particles, p=weights
        )
        self.particles = [self.particles[i] for i in indices]
        for p in self.particles:
            p.weight = 1.0 / self.num_particles
```

### 10.1.4 slam_toolbox 在线/离线建图

slam_toolbox 是 ROS 2 主流 2D SLAM 方案：

**在线建图 (Online Async):**
```python
import rclpy
from rclpy.node import Node
from slam_toolbox.srv import SaveMap
from nav_msgs.msg import OccupancyGrid
import subprocess

class OnlineSlamManager(Node):
    """在线 SLAM 建图管理器"""
    def __init__(self):
        super().__init__('slam_manager')
        # 订阅地图更新
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10
        )
        # 保存地图服务客户端
        self.save_client = self.create_client(SaveMap, '/slam_toolbox/save_map')
        self.get_logger().info('SLAM管理器已就绪')

    def map_callback(self, msg: OccupancyGrid):
        """接收实时建图结果"""
        resolution = msg.info.resolution
        width = msg.info.width
        height = msg.info.height
        free = sum(1 for c in msg.data if c == 0)
        occupied = sum(1 for c in msg.data if c == 100)
        self.get_logger().info(
            f'地图: {width}x{height}, 分辨率={resolution:.3f}m, '
            f'空闲={free}, 占据={occupied}'
        )

    async def save_map(self, name: str = 'my_map'):
        """保存建图结果"""
        # 通过 ROS 2 service 保存地图
        req = SaveMap.Request()
        req.name.data = name
        future = self.save_client.call_async(req)
        await future
        self.get_logger().info(f'地图已保存: {name}')


def spawn_slam_toolbox_online():
    """启动 slam_toolbox 在线建图 Lifecycle 节点"""
    cmd = [
        'ros2', 'launch', 'slam_toolbox',
        'online_async_launch.py',
        'params_file:=config/mapper_params_online_async.yaml',
        'use_sim_time:=true'
    ]
    subprocess.Popen(cmd)
```

**离线建图 (手册建图, 读取 rosbag):**
```bash
# 录制数据包
ros2 bag record -o my_mapping_bag /scan /tf /tf_static /odom

# 离线建图
ros2 launch slam_toolbox offline_launch.py \
  bag_filename:=my_mapping_bag use_sim_time:=true
```

### 10.1.5 AMCL 蒙特卡洛定位

AMCL (Adaptive Monte Carlo Localization) 使用粒子滤波器进行全局定位：

```python
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose

class AMCLManager(Node):
    """AMCL 定位管理器"""
    def __init__(self):
        super().__init__('amcl_manager')
        # 初始位姿发布器
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10
        )
        # 当前估计位姿
        self.current_pose = None

    def set_initial_pose(self, x: float, y: float, yaw: float):
        """设置 AMCL 初始位姿估计"""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        qz = np.sin(yaw / 2.0)
        qw = np.cos(yaw / 2.0)
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        # 协方差: 表示不确定性
        msg.pose.covariance[0] = 0.25   # x 方差
        msg.pose.covariance[7] = 0.25   # y 方差
        msg.pose.covariance[35] = 0.05  # yaw 方差
        self.pose_pub.publish(msg)
        self.get_logger().info(f'初始位姿已设置: ({x:.2f}, {y:.2f}, {yaw:.2f})')


def launch_amcl(use_sim_time: bool = True):
    """启动 AMCL 定位节点"""
    import subprocess
    cmd = [
        'ros2', 'launch', 'nav2_bringup',
        'localization_launch.py',
        'map:=src/maps/my_map.yaml',
        'params_file:=config/nav2_params.yaml',
        f'use_sim_time:={str(use_sim_time).lower()}'
    ]
    subprocess.Popen(cmd)
```

**AMCL 粒子收敛示意:**
```
初始分布 (全局均匀):    收敛后 (聚类):
o o o o o               . . . . .
o o o o o               . . o . .
o o o o o      -->      . o o o .
o o o o o               . . o . .
o o o o o               . . . . .
```

### 10.1.6 map_server 地图保存与加载

```python
from nav2_msgs.srv import LoadMap, SaveMap
from nav_msgs.msg import OccupancyGrid
import yaml

class MapServer(Node):
    """地图服务器 Python 接口"""
    def __init__(self):
        super().__init__('map_server_manager')
        self.load_client = self.create_client(
            LoadMap, '/map_server/load_map'
        )
        self.save_client = self.create_client(
            SaveMap, '/map_server/save_map'
        )
        self.current_map: OccupancyGrid = None

    async def load_map(self, yaml_path: str):
        """加载地图"""
        req = LoadMap.Request()
        req.map_url = yaml_path
        future = self.load_client.call_async(req)
        result = await future
        self.get_logger().info(f'地图已加载: {yaml_path} (result={result.result})')
        return result

    async def save_map(self, map_name: str):
        """保存当前地图"""
        req = SaveMap.Request()
        req.map_topic = 'map'
        req.map_url = f'src/maps/{map_name}'
        future = self.save_client.call_async(req)
        await future
        self.get_logger().info(f'地图已保存: {map_name}')

    def read_map_yaml(self, yaml_path: str) -> dict:
        """读取地图 YAML 元数据"""
        with open(yaml_path, 'r') as f:
            return yaml.safe_load(f)
```

### 10.1.7 Cartographer 多传感器 SLAM

Cartographer 支持激光 + IMU + 里程计多传感器融合，采用子图 (Submap) 和回环检测机制。

**Cartographer 核心流程:**
```
传感器数据 → 局部SLAM (构建子图 submap)
                  ↓
            全局SLAM (回环检测 + 后端优化)
                  ↓
            输出全局一致地图
```

```python
import subprocess
import yaml

def generate_cartographer_config(
    num_lasers: int = 1,
    provide_odom: bool = True,
    use_imu: bool = False,
    tracking_frame: str = 'base_link',
    published_frame: str = 'odom',
    map_frame: str = 'map'
) -> str:
    """动态生成 Cartographer 配置文件"""
    config = {
        'options': {
            'use_sim_time': True,
            'map_frame': map_frame,
            'tracking_frame': tracking_frame,
            'published_frame': published_frame,
            'odom_frame': 'odom',
            'provide_odom_frame': provide_odom,
            'use_odometry': provide_odom,
            'use_nav_sat': False,
            'num_range_sensors': num_lasers,
            'use_landmarks': False,
            'num_laser_scans': num_lasers,
            'num_point_clouds': 0,
        },
        'trajectory_builder_options': {
            'use_imu_data': use_imu,
            'min_range': 0.15,
            'max_range': 25.0,
            'voxel_filter_size': 0.05,
        },
        'pose_graph_options': {
            'optimize_every_n_nodes': 90,
            'constraint_builder_options': {
                'sampling_ratio': 0.3,
                'max_constraint_distance': 15.0,
                'ceres_scan_matcher_options': {
                    'occupied_space_weight': 20.0,
                    'translation_weight': 10.0,
                    'rotation_weight': 1.0,
                },
                'fast_correlative_scan_matcher_options': {
                    'linear_search_window': 7.0,
                    'angular_search_window': 0.5236,
                    'branch_and_bound_depth': 7,
                },
            },
            'matcher_translation_weight': 5e2,
            'matcher_rotation_weight': 1.6e3,
            'optimization_problem_options': {
                'huber_scale': 1e1,
            },
        },
    }
    config_yaml = yaml.dump(config, default_flow_style=False, allow_unicode=True)
    return config_yaml


def launch_cartographer(rviz: bool = True):
    """启动 Cartographer 建图"""
    cmd = [
        'ros2', 'launch', 'turtlebot3_cartographer',
        'cartographer.launch.py'
    ]
    if not rviz:
        cmd.append('open_rviz:=false')
    subprocess.Popen(cmd)
```

### 10.1.8 slam_sim_demo 仿真建图演示

参考 `ros2_ws/src/slam_sim_demo_ros2/` 包结构，使用 teleop 控制机器人在 Gazebo 仿真环境中建图：

```python
# 从 slam_sim_demo_ros2 包核心建图流程
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class AutoExplore(Node):
    """自动探索建图节点 - 基于激光避障的边界探索"""
    def __init__(self):
        super().__init__('auto_explore')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.front_dist = float('inf')
        self.timer = self.create_timer(0.1, self.control_loop)

    def scan_callback(self, msg: LaserScan):
        """分析激光数据"""
        ranges = np.array(msg.ranges)
        # 前方 30 度扇形区域
        idx = len(ranges) // 2
        half_window = len(ranges) // 12
        front_sector = ranges[idx - half_window: idx + half_window]
        self.front_dist = np.min(front_sector[np.isfinite(front_sector)])

    def control_loop(self):
        """基于激光的避障巡航"""
        cmd = Twist()
        if self.front_dist < 0.8:
            cmd.angular.z = 0.8  # 障碍物过近, 旋转
            cmd.linear.x = 0.0
        elif self.front_dist < 1.5:
            cmd.linear.x = 0.15
            cmd.angular.z = 0.3  # 慢速避障
        else:
            cmd.linear.x = 0.3   # 前进探索
            cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = AutoExplore()
    rclpy.spin(node)
    rclpy.shutdown()
```

---

## 10.2 练习题

**1. 原理题:** 简述 GraphSLAM 图优化模型中顶点和边的物理含义, 并说明为什么回环检测对 SLAM 至关重要。

**2. 编程题:** 编写一个 ROS 2 节点, 订阅 `/scan` 话题并统计激光雷达的测量极值 (最近/最远有效距离), 将结果以 ROS 2 日志形式输出。

**3. 配置题:** 编写 slam_toolbox 在线建图的 YAML 配置文件, 要求设置地图分辨率为 0.05m, 更新频率 5Hz, 使用 scan_matching 方法, 并调整回环检测阈值。

**4. 分析题:** AMCL 定位中, 粒子数增加会带来什么影响? 说明 KLD 采样如何自适应调整粒子数量。

**5. 操作题:** 描述从 Gazebo 仿真到保存最终地图的完整 ROS 2 命令行流程, 包括启动仿真、建图、teleop 控制、保存地图及验证。

**6. 设计题:** 某仓库分 3 层楼, 每层约 500m², 需要全自主建图。设计方案包括传感器选型、SLAM 算法选择、多楼层地图配准方法, 并给出 ROS 2 Launch 文件框架。
