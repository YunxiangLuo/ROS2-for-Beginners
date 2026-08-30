# 第32章 AR标签检测与手眼标定

## 学习目标
本章学习目标包括：掌握AprilTag和ArUco标签的检测方法；理解标签位姿估计原理；掌握手眼标定的两种方式（eye-in-hand/eye-to-hand）；学会使用easy_handeye进行手眼标定；理解坐标变换在视觉系统中的作用。

## 32.1 AR标签检测

### 32.1.1 什么是AR标签

AR（Augmented Reality）标签是用于视觉定位的标记图案，通过图像处理可以快速检测并计算出标签在三维空间中的位姿。常见的AR标签类型有：**ArUco**是OpenCV集成的开源标签系统；**AprilTag**比ArUco精度更高、误检率更低；**QR Code**是通用二维码，但位姿估计精度较低。

ArUco标签的配置参数：

| 字典 | 标记数 | 标记尺寸 |
|------|--------|---------|
| DICT_4X4_50 | 50 | 4×4 bits |
| DICT_5X5_100 | 100 | 5×5 bits |
| DICT_6X6_250 | 250 | 6×6 bits |
| DICT_7X7_1000 | 1000 | 7×7 bits |

### 32.1.2 ArUco标签生成

```python
import cv2
import cv2.aruco as aruco
import numpy as np

def create_aruco_markers():
    """生成ArUco标记"""
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)

    # 生成多个标记
    for marker_id in range(1, 5):
        marker_size = 200  # 像素
        image = aruco.generateImageMarker(
            dictionary, marker_id, marker_size
        )
        cv2.imwrite(f'aruco_marker_{marker_id}.png', image)
        print(f'生成: aruco_marker_{marker_id}.png')

    # 生成标定板（多个标记的组合）
    board = aruco.GridBoard(
        size=(3, 2),           # 3列2行
        markerLength=0.04,     # 每个标记边长4cm
        markerSeparation=0.01, # 标记间距1cm
        dictionary=dictionary
    )
    board_image = board.generateImage(outSize=(600, 400))
    cv2.imwrite('aruco_board.png', board_image)
    print('生成: aruco_board.png')

if __name__ == '__main__':
    create_aruco_markers()
```

### 32.1.3 ArUco检测与位姿估计

```python
#!/usr/bin/env python3
"""ArUco标签检测与位姿估计ROS2节点"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
import yaml

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.bridge = CvBridge()

        # ArUco配置
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.dictionary, self.parameters)

        # 标记实际尺寸（米）
        self.declare_parameter('marker_size', 0.065)
        self.marker_size = self.get_parameter('marker_size').value

        # 相机参数（需标定得到）
        self.camera_matrix = None
        self.dist_coeffs = None

        # 发布者
        self.marker_pose_pub = self.create_publisher(
            PoseStamped, '/aruco/pose', 10
        )

        # 订阅者
        self.image_sub = self.create_subscription(
            Image, '/camera/color/image_raw',
            self.image_callback, 10
        )
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/color/camera_info',
            self.info_callback, 10
        )

        self.get_logger().info('ArUco检测节点已启动')

    def info_callback(self, msg):
        """读取相机内参"""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info('相机参数已加载')

    def image_callback(self, msg):
        if self.camera_matrix is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 检测ArUco标记
        corners, ids, rejected = self.detector.detectMarkers(gray)

        if ids is not None:
            # 绘制检测到的标记
            aruco.drawDetectedMarkers(frame, corners, ids)

            # 估计位姿
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, self.marker_size,
                self.camera_matrix, self.dist_coeffs
            )

            for i in range(len(ids)):
                # 绘制坐标系轴
                aruco.drawAxis(
                    frame, self.camera_matrix, self.dist_coeffs,
                    rvecs[i], tvecs[i], 0.03
                )

                # 构建PoseStamped消息
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = 'camera_color_optical_frame'
                pose_msg.pose.position.x = float(tvecs[i][0][0])
                pose_msg.pose.position.y = float(tvecs[i][0][1])
                pose_msg.pose.position.z = float(tvecs[i][0][2])

                # 旋转向量 → 四元数
                rmat, _ = cv2.Rodrigues(rvecs[i])
                q = self.rotation_matrix_to_quaternion(rmat)
                pose_msg.pose.orientation.x = q[0]
                pose_msg.pose.orientation.y = q[1]
                pose_msg.pose.orientation.z = q[2]
                pose_msg.pose.orientation.w = q[3]

                self.marker_pose_pub.publish(pose_msg)

                self.get_logger().info(
                    f'标记 {ids[i][0]}: '
                    f't={tvecs[i][0]}, r={rvecs[i][0]}'
                )

        cv2.imshow('ArUco Detection', frame)
        cv2.waitKey(1)

    @staticmethod
    def rotation_matrix_to_quaternion(r):
        """旋转矩阵 → 四元数"""
        q = np.empty(4)
        t = np.trace(r)
        if t > 0:
            s = np.sqrt(1.0 + t) * 2
            q[3] = 0.25 * s
            q[0] = (r[2, 1] - r[1, 2]) / s
            q[1] = (r[0, 2] - r[2, 0]) / s
            q[2] = (r[1, 0] - r[0, 1]) / s
        else:
            i = np.argmax([r[0, 0], r[1, 1], r[2, 2]])
            s = np.sqrt(1.0 + r[i, i]) * 2
            q[3] = (r[2, 1] - r[1, 2]) / s if i == 0 else \
                   (r[0, 2] - r[2, 0]) / s if i == 1 else \
                   (r[1, 0] - r[0, 1]) / s
            q[i] = 0.25 * s
        return q

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 32.1.4 AprilTag检测

AprilTag是另一种流行的视觉标签系统：

```bash
# 安装AprilTag
pip install apriltag

# 或安装ROS2 wrapper
# Jazzy 下先确认发行版是否提供该 wrapper；也可直接使用 pip apriltag
apt-cache policy ros-jazzy-apriltag-ros
```

```python
# 使用AprilTag进行检测
import apriltag
import cv2
import numpy as np

class AprilTagDetector:
    def __init__(self):
        self.detector = apriltag.Detector()

    def detect(self, gray_image):
        """检测AprilTag标签"""
        results = self.detector.detect(gray_image)
        detections = []

        for r in results:
            # 标签角点
            corners = r.corners

            # 标签中心
            center = (r.center[0], r.center[1])

            # 标签ID
            tag_id = r.tag_id

            # 位姿估计（需要相机内参）
            pose = self.estimate_pose(r, self.camera_matrix)

            detections.append({
                'id': tag_id,
                'center': center,
                'corners': corners,
                'pose': pose,
            })

        return detections

    def estimate_pose(self, detection, camera_matrix, tag_size=0.065):
        """估计AprilTag的3D位姿"""
        # 3D点（标签坐标系）
        half = tag_size / 2
        object_points = np.array([
            [-half, -half, 0],
            [half, -half, 0],
            [half, half, 0],
            [-half, half, 0],
        ])

        # 2D点（图像坐标系）
        image_points = detection.corners.astype(np.float32)

        # PnP求解
        ret, rvec, tvec = cv2.solvePnP(
            object_points, image_points,
            camera_matrix, None
        )

        if ret:
            return {
                'translation': tvec.flatten(),
                'rotation': rvec.flatten(),
            }
        return None
```

### 32.1.5 多标签检测

```python
class MultiArucoDetector(Node):
    """多ArUco标签同时检测"""

    def __init__(self):
        super().__init__('multi_aruco_detector')
        self.bridge = CvBridge()
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.detector = aruco.ArucoDetector(self.dictionary)

        self.marker_sizes = {
            1: 0.065,  # 标记1边长6.5cm
            2: 0.065,
            3: 0.040,  # 标记3边长4.0cm
            4: 0.040,
        }

        self.pose_publishers = {}
        for marker_id in range(1, 5):
            self.pose_publishers[marker_id] = self.create_publisher(
                PoseStamped, f'/aruco/{marker_id}/pose', 10
            )

    def image_callback(self, msg):
        # ... 检测逻辑
        pass
```

## 32.2 手眼标定原理

### 32.2.1 为什么需要手眼标定

手眼标定解决相机坐标系与机械臂坐标系之间的变换关系。视觉系统检测到目标在相机坐标系下的位姿后，需要将其变换到机械臂基座坐标系，才能由MoveIt2进行运动规划。

变换链：
```
object → camera_link → base_link → (MoveIt2) → end_effector
```

### 32.2.2 眼在手外（Eye-to-Hand）

相机固定安装在机械臂外部，与机械臂基座的相对位置不变。

```
标定板(固定于末端) → 相机视角 → 机械臂运动多个位姿
```

变换关系为：A表示机械臂末端在基座下的位姿（已知，正运动学），B表示标定板在相机下的位姿（已知，视觉检测），X表示相机在基座下的位姿（待求解）。

求解方程：AX = XB

### 32.2.3 眼在手上（Eye-in-Hand）

相机安装在机械臂末端，随机械臂运动。

```
标定板(固定于外部) → 相机视角 → 机械臂运动多个位姿
```

同样求解AX = XB，此时X为相机与机械臂末端的变换关系。

### 32.2.4 手眼标定数学原理

```
A * X = X * B

其中：
A = T_base_endeffector_current⁻¹ * T_base_endeffector_next
  = 机械臂末端的相对运动

B = T_camera_board_current * T_camera_board_next⁻¹
  = 标定板的相对运动

X = T_endeffector_camera（eye-in-hand）或 T_base_camera（eye-to-hand）
```

求解分为两步：先进行旋转部分求解（R_A * R_X = R_X * R_B），再进行平移部分求解（(R_A - I) * t_X = R_X * t_B - t_A）。

```python
import numpy as np

def solve_handeye(A_matrices, B_matrices, method='tsai'):
    """
    手眼标定求解 AX = XB
    参数:
        A_matrices: [4x4] 机械臂末端相对运动列表
        B_matrices: [4x4] 标定板相对运动列表
    返回:
        X: [4x4] 手眼变换矩阵
    """
    from scipy.spatial.transform import Rotation

    n = len(A_matrices)
    M = np.zeros((3 * n, 4))
    C = np.zeros((3 * n, 1))

    for i in range(n):
        R_A = A_matrices[i][:3, :3]
        t_A = A_matrices[i][:3, 3]
        R_B = B_matrices[i][:3, :3]
        t_B = B_matrices[i][:3, 3]

        # 旋转向量
        r_A = Rotation.from_matrix(R_A).as_rotvec()
        r_B = Rotation.from_matrix(R_B).as_rotvec()

        # 构建线性方程组
        M[3*i:3*i+3, :] = np.hstack([
            R_A - np.eye(3),
            np.zeros((3, 1))
        ])
        C[3*i:3*i+3, 0] = R_A @ t_B - t_A

    # 求解最小二乘
    X_vec, _, _, _ = np.linalg.lstsq(M, C, rcond=None)
    R_X = Rotation.from_rotvec(X_vec[:3].flatten()).as_matrix()
    t_X = X_vec[3:].flatten()

    X = np.eye(4)
    X[:3, :3] = R_X
    X[:3, 3] = t_X
    return X
```

## 32.3 手眼标定实践

### 32.3.1 easy_handeye功能包

ROS2中有easy_handeye功能包简化手眼标定流程：

```bash
# Jazzy 下 easy_handeye 的二进制可用性取决于发行版仓库，先检查
apt-cache policy ros-jazzy-easy-handeye

# 或源码安装
cd ~/ros2_course_ws/src
git clone https://github.com/IFL-CAMP/easy_handeye.git
cd ..
colcon build --packages-select easy_handeye
```

### 32.3.2 眼在手外标定流程

**步骤1：配置launch文件**

```python
# handeye_calibration.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. 启动相机驱动
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            parameters=[{
                'video_device': '/dev/video0',
                'frame_id': 'camera_link',
            }]
        ),

        # 2. 启动ArUco检测
        Node(
            package='aruco_ros',
            executable='aruco_node',
            name='aruco_detector',
            parameters=[{
                'marker_size': 0.065,
                'reference_frame': 'camera_link',
            }]
        ),

        # 3. 启动easy_handeye标定
        Node(
            package='easy_handeye',
            executable='calibrate',
            name='handeye_calibration',
            parameters=[{
                'eye_on_hand': False,  # Eye-to-Hand
                'robot_base_frame': 'base_link',
                'robot_effector_frame': 'tool0',
                'tracking_base_frame': 'camera_link',
                'tracking_marker_frame': 'aruco_marker_frame',
            }]
        ),
    ])
```

**步骤2：启动标定**

```bash
ros2 launch handeye_calibration.launch.py

# 在另一个终端启动标定GUI
ros2 run easy_handeye handeye_calibration_client
```

**步骤3：采集数据**

通过GUI操作完成数据采集：先控制机械臂运动到第一个位姿，点击"Take Sample"采集；再控制机械臂运动到不同位姿并重复采集（建议至少采集17个样本）；最后点击"Compute"计算结果，并点击"Save"保存标定结果。

**步骤4：保存结果**

标定结果保存为YAML文件：

```yaml
# handeye_calibration.yaml
transformation:
  header:
    frame_id: base_link
    child_frame_id: camera_link
  transform:
    translation:
      x: 1.004
      y: -0.628
      z: 0.553
    rotation:
      x: 0.482
      y: -0.072
      z: 0.118
      w: 0.866
```

### 32.3.3 发布标定结果

```python
#!/usr/bin/env python3
"""发布手眼标定结果TF"""
import rclpy
from rclpy.node import Node
import tf2_ros
import geometry_msgs.msg
import yaml

class CalibrationPublisher(Node):
    """发布手眼标定结果的静态TF变换"""

    def __init__(self):
        super().__init__('calibration_publisher')
        self.declare_parameter('calibration_file', '')
        calib_file = self.get_parameter('calibration_file').value

        # 加载标定文件
        with open(calib_file, 'r') as f:
            calib = yaml.safe_load(f)

        # 创建静态TF发布器
        self.br = tf2_ros.StaticTransformBroadcaster(self)
        t = geometry_msgs.msg.TransformStamped()

        trans = calib['transformation']
        t.header.frame_id = trans['header']['frame_id']
        t.child_frame_id = trans['child_frame_id']

        tr = trans['transform']['translation']
        t.transform.translation.x = tr['x']
        t.transform.translation.y = tr['y']
        t.transform.translation.z = tr['z']

        rot = trans['transform']['rotation']
        t.transform.rotation.x = rot['x']
        t.transform.rotation.y = rot['y']
        t.transform.rotation.z = rot['z']
        t.transform.rotation.w = rot['w']

        self.br.sendTransform(t)
        self.get_logger().info(
            f'发布静态TF: {t.header.frame_id} → {t.child_frame_id}'
        )

        # 持续发布（static broadcaster只需要发布一次）

def main(args=None):
    rclpy.init(args=args)
    node = CalibrationPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 32.3.4 眼在手上标定

```python
# eye_in_hand_calibration.launch.py
def generate_launch_description():
    return LaunchDescription([
        # ...

        # eye-in-hand配置
        Node(
            package='easy_handeye',
            executable='calibrate',
            name='handeye_calibration',
            parameters=[{
                'eye_on_hand': True,   # Eye-in-Hand
                'robot_base_frame': 'base_link',
                'robot_effector_frame': 'tool0',
                'tracking_base_frame': 'camera_link',
                'tracking_marker_frame': 'aruco_marker_frame',
            }]
        ),
    ])
```

### 32.3.5 手动标定步骤

如果无法使用easy_handeye，可以手动完成标定：

```python
#!/usr/bin/env python3
"""手动手眼标定（Eye-to-Hand）"""
import numpy as np
import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import PoseStamped
import tf_transformations

class ManualHandEyeCalibration(Node):
    def __init__(self):
        super().__init__('manual_handeye_calibration')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.samples = []
        self.get_logger().info(
            '手动标定: 移动机械臂到不同位姿后按Enter采集'
        )

    def collect_sample(self):
        """采集一个样本点"""
        try:
            # 获取机械臂末端在基座下的位姿 (A)
            base_to_ee = self.tf_buffer.lookup_transform(
                'base_link', 'tool0', rclpy.time.Time()
            )

            # 获取标定板在相机下的位姿 (B)
            camera_to_board = self.tf_buffer.lookup_transform(
                'camera_color_optical_frame', 'aruco_marker_frame',
                rclpy.time.Time()
            )

            sample = {
                'base_to_ee': base_to_ee,
                'camera_to_board': camera_to_board,
            }
            self.samples.append(sample)
            self.get_logger().info(
                f'采集样本 {len(self.samples)}: '
                f'末端({base_to_ee.transform.translation.x:.3f}, '
                f'{base_to_ee.transform.translation.y:.3f}, '
                f'{base_to_ee.transform.translation.z:.3f})'
            )

        except Exception as e:
            self.get_logger().error(f'采集失败: {e}')

    def compute_calibration(self):
        """计算标定结果"""
        if len(self.samples) < 3:
            self.get_logger().error('至少需要3个样本')
            return

        A_matrices = []
        B_matrices = []

        for i in range(len(self.samples) - 1):
            s1 = self.samples[i]
            s2 = self.samples[i + 1]

            # A: 机械臂末端的相对运动
            A1 = self.transform_to_matrix(s1['base_to_ee'].transform)
            A2 = self.transform_to_matrix(s2['base_to_ee'].transform)
            A = np.linalg.inv(A1) @ A2
            A_matrices.append(A)

            # B: 标定板的相对运动
            B1 = self.transform_to_matrix(s1['camera_to_board'].transform)
            B2 = self.transform_to_matrix(s2['camera_to_board'].transform)
            B = B1 @ np.linalg.inv(B2)
            B_matrices.append(B)

        # 求解
        X = solve_handeye(A_matrices, B_matrices)
        self.get_logger().info(f'标定结果:\n{X}')

    @staticmethod
    def transform_to_matrix(transform):
        """Transform → 4x4矩阵"""
        T = tf_transformations.quaternion_matrix([
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        ])
        T[0, 3] = transform.translation.x
        T[1, 3] = transform.translation.y
        T[2, 3] = transform.translation.z
        return T
```

### 32.3.6 TF树与坐标变换

完整的TF树结构：

```
world
  └── base_link (虚拟关节)
       ├── arm_base (URDF)
       │   ├── link1
       │   │   ├── link2
       │   │   │   ├── link3
       │   │   │   │   ├── link4
       │   │   │   │   │   ├── link5
       │   │   │   │   │   │   └── tool0 (末端)
       │   │   │   │   │   └── camera_link (手眼标定)
       │   │   │   │   │       └── camera_color_optical_frame
       │   │   │   │   │           └── aruco_marker_frame (检测结果)
```

```python
class TFVisualizer(Node):
    """TF树查看与坐标变换"""
    def __init__(self):
        super().__init__('tf_visualizer')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.query_transforms)

    def query_transforms(self):
        try:
            # 查询相机到基座的变换
            camera_to_base = self.tf_buffer.lookup_transform(
                'base_link', 'camera_color_optical_frame',
                rclpy.time.Time()
            )
            pos = camera_to_base.transform.translation
            self.get_logger().info(
                f'相机在基座下位置: '
                f'({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})'
            )

            # 查询物体到基座的变换
            object_to_base = self.tf_buffer.lookup_transform(
                'base_link', 'aruco_marker_frame',
                rclpy.time.Time()
            )
            obj_pos = object_to_base.transform.translation
            self.get_logger().info(
                f'物体在基座下位置: '
                f'({obj_pos.x:.3f}, {obj_pos.y:.3f}, {obj_pos.z:.3f})'
            )

        except Exception as e:
            self.get_logger().debug(f'TF查询失败: {e}')
```

### 32.3.7 官方要点——OpenCV 官方 ArUco 文档、easy_handeye 标定教程与 TF2 官方坐标变换教程

**OpenCV 官方 ArUco 模块：字典、检测与位姿估计。**OpenCV 官方 ArUco 教程定义了练习第 1 题的全部细节：`cv.aruco.Dictionary_get(aruco.DICT_6X6_250)` 生成 6x6 位纹、250 个标记的字典（H/maxCorrection 决定误检率），`cv.aruco.detectMarkers` 返回角点与 ID，`cv.aruco.estimatePoseSingleMarkers` 用已知 `markerLength`（米）求解板系位姿（rvec/tvec）。官方文档特别提醒一个"规范"问题：正方形标记存在 180° 镜像歧义（rvec 有两个对称解），因此单面 ArUco 的 roll 方向可能整体翻转——工程上常用"对角角点编号排序 + 已知朝上轴"消歧。AprilTag（MIT 项目）在 OpenCV 4.7 后同样纳入 `cv.aruco.detectMarkers` 的字典选项，其四大家族（tag36h11 等）误检率更低，间距依赖更弱，是 ArUco 之外的另一主流选择。

**手眼标定的官方数学约定：AX=XB 与两种构型。**本章 32.2 节的核心方程在机器人视觉文献中即经典 A X = X B 问题：A 是相机两次观测间的变换（由标记位姿相除得到），B 是机械臂两次位姿间的变换（从 TF/正运动学读取），X 是待求手眼矩阵；官方文档普遍强调，平移与旋转需分解求解（先旋转后平移，即本章 32.2 节的方程组），且最小化重投影误差的优化解优于解析解。easy_handeye 官方文档把两种构型讲得很清楚：eye-in-hand（相机装末端，X = 相机→末端）与 eye-to-hand（相机装外部固定位，X = 相机→基座），两者只需在启动时选对参数头文件，算法一致，但 motions 建议不同——eye-in-hand 要求大旋转、小平移，eye-to-hand 反之，两条经验都对应练习第 3 题"至少 17 个样本、位姿差异尽量大"的采集策略。

**easy_handeye2 官方流程：界面、服务与参数。**easy_handeye2（ROS 2 维护版，原 easy_handeye 移植）官方 README 给出了练习第 3/4 题的完整走法：`ros2 launch easy_handeye2 publish.launch.py eye:=on_robot/off_robot` 启动标定界面（MoveIt 侧 RViz 面板 Take Sample / Compute / Save），每次采样自动读取当前机械臂位姿与标记位姿，采集满 17 个以上样本后点 Compute 估算 X；保存时生成 `handeye.yaml`（含 transformation 矩阵），`publish.launch.py` 的 `eye_on_hand` 参数决定发布到 TF 的静态变换名称。官方文档强调标定结果应做交叉验证：采集一组"验证样本"（不参与求解），检查重投影误差与在 RViz 中把相机看到的标记框与机械臂模型对齐；`tsai-lenz`、`park-martin` 等求解器可在配置中切换，官方建议多种解法互相佐证。

**TF2 官方教程：把标定结果变成可用坐标链。**ROS 2 官方 TF2 教程（Writing/Reading Frames）覆盖了练习第 2/4/5 题涉及的变换链：`tf2_ros.StaticTransformBroadcaster` 发布"标定后固定不变"的 camera→base 关系，`tf2_ros.Buffer.lookup_transform` 在任意时刻查询 marker→base 的合成变换；官方教程强调查询应选 `tf2_time` 对齐的同步帧，且静态变换发布后其他节点才能做 look-at 链式运算。练习第 5 题的"标记→机械臂基座"终点在官方工程里就是 Image→camera_info（第 30 章反投影）→ArUco 位姿（第 32.1 节）→lookup（第 32.4 节）四段链路的串接，The Construct 的手眼标定课程与 Robotics Back-End 的教程分别演示了 Gazebo 仿真与真实 UR5 上验证该链条的完整工程案例。

> 本节内容综合翻译自 OpenCV 官方文档（ArUco 模块教程）、easy_handeye / easy_handeye2 官方仓库文档、ROS 2 官方 TF2 教程（Writing/Reading Frame）与 MoveIt 官方 "Hand-Eye Calibration" 相关文档，另参考 AprilTag 项目官方文档与 The Construct 的手眼标定课程。原文均为英文，此处为中文编译，供课后巩固与进阶阅读。

## 课后练习

1. 生成4个不同的ArUco标记（DICT_6X6_250），打印并固定在物体表面。编写ROS2节点检测并输出每个标记的ID和位姿。

2. 编写节点，将ArUco标记检测到的位姿发布为TF变换，使标记坐标系在Rviz中可见。

3. 使用easy_handeye进行Eye-to-Hand手眼标定，至少采集17个样本，保存标定结果。

4. 加载手眼标定结果，编写节点发布静态TF变换，验证camera_link到base_link的变换是否正确。

5. 编写完整的坐标变换节点：检测ArUco标记 → 查询TF变换 → 输出目标在base_link坐标系下的位姿。

---

## 仿真结合实例（当前仓库）：相机坐标系到机械臂基座的标定接口

### 目标与知识点对应

先用 `robot_sim_demo` 验证相机图像和 `camera_link` TF，再在 xArm MoveIt/RViz 中验证目标位姿转换后的基座坐标接口，理解手眼标定结果如何进入抓取规划。

### 运行步骤

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_sim_demo gazebo2.launch.py \
  gui:=false rviz:=false drive:=false
ros2 run tf2_ros tf2_echo base_link camera_link
ros2 topic echo /camera/image_raw --once
```

在提供兼容 `xarm_description` 2.0.0 的环境中，再启动 xArm 的 MoveIt/RViz，将检测节点输出的 `PoseStamped` 转换到 `base_link` 后送给 `xarm` 规划组。

### 观察结果与边界

TF 查询应返回相机相对机器人基座的变换；ArUco 检测和手眼求解是否成功取决于外部视觉节点与标定样本。当前仓库没有预置检测器或手眼标定结果。

### 源码

相机/TF：`src/robot_sim_demo/models/wheeltec_robot/model.sdf`、`src/robot_sim_demo/config/gazebo2_bridge.yaml`；xArm MoveIt：`src/xarm/launch/arm_only_move_group.launch.py`；视觉实验参考：`src/lab_code/ch19_lab/vision_detection_lab/`。

> 参考来源：
> - OpenCV 官方文档 —— ArUco 模块教程：https://docs.opencv.org/
> - AprilTag 官方项目文档：https://april.eecs.umich.edu/
> - easy_handeye / easy_handeye2 官方仓库：https://github.com/ros4hri/easy_handeye2 、https://github.com/ffrisi/easy_handeye_ros
> - ROS 2 官方 TF2 教程：https://docs.ros.org/
> - The Construct / Robotics Back-End —— 手眼标定课程：https://www.theconstructsim.com/ 、https://roboticsbackend.com/
