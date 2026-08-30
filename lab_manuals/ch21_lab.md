# 第21章 实验：视觉抓取综合项目

## 当前仓库仿真验证：相机目标接口与 xArm 规划分层

### 实验目标

用移动机器人仿真验证相机、内参和 TF，用 xArm6 MoveIt2/RViz 验证目标 Pose 到规划组的转换，形成“检测→定位→规划”的可检查链路。

### 运行步骤

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_sim_demo gazebo2.launch.py \
  gui:=true rviz:=false drive:=false
ros2 topic echo /camera/camera_info --once
ros2 run tf2_ros tf2_echo base_link camera_link
```

在提供兼容 `xarm_description` 2.0.0 的环境中另开终端：

```bash
source /path/to/xarm_description_workspace/install/setup.bash
ros2 launch xarm_ros2_arm_only arm_only.launch.py
```

将实验视觉节点输出的 `PoseStamped` 转换到 xArm 基座 frame，在 RViz MotionPlanning 中先验证预抓取和避障轨迹。

### 观察与边界

分开检查相机话题、TF、目标位姿、规划结果和夹爪执行状态。源码：`src/lab_code/ch21_lab/vision_pickup_lab/`、`src/robot_sim_demo/`、`src/xarm/`。当前仓库没有真实 ArUco/YOLO 检测和成功抓取证据，不将接口验证写成完整闭环成功。

## 实际运行证据

真实运行的 Gazebo 相机桥接、xArm 控制器与 MoveIt 规划服务接口检查：

![ch21 相机目标接口与 xArm 规划分层](images/runtime/ch21_vision_pickup.gif)

原始录制：[ch21_vision_pickup.cast](images/runtime/ch21_vision_pickup.cast)。相机 TF 查询结果仍需按本地环境单独检查。

> **对应理论章节**：第34章《视觉抓取完整流程》、第35章《综合实训：智能机器人产线》
> **实验课时**：6课时  
> **实验代码**：`src/lab_code/ch21_lab/vision_pickup_lab/`  

## 实验目标
- 整合视觉检测和机械臂运动规划
- 实现"检测→定位→抓取→放置"全流程自动化
- 掌握视觉服务和MoveIt2的协同工作方式
- 完成基于ArUco标签的视觉抓取演示
- 综合运用课程所学知识构建完整的智能机器人系统
- 实现多组件集成: 视觉检测 + 运动规划 + 抓取控制
- 掌握Action Server在复杂任务编排中的应用
- 完成从场景理解到任务执行的全自动化流程

## 实验环境
- ROS 2 Jazzy + MoveIt2
- USB摄像头 + ArUco标签
- OpenCV + cv_bridge
- YOLOv8 / VLM API (可选，见第20章)
- Python3

**MoveIt 前置依赖（统一说明）**：本章实验开始前，需先启动 xArm 仿真与 MoveIt：

```bash
# 方式1: 纯 MoveIt + RViz 仿真（不含 Gazebo）
ros2 launch xarm_ros2_arm_only arm_only_move_group.launch.py

# 方式2: 含 Gazebo 的完整仿真（本章参考代码 README 使用的命令）
ros2 launch xarm_ros2_arm_only arm_only.launch.py
```

默认使用上述 `xarm_ros2_arm_only` 启动命令。若按第17章17.1节自行生成配置包，请使用该包的实际名称及其生成的 launch 文件。

## 参考代码说明
`src/lab_code/ch21_lab/vision_pickup_lab/`（ament_python 包）提供以下程序：

| 程序 | 功能 |
|------|------|
| `tf2_camera_broadcaster.py` | 相机TF广播器。发布 `camera_link` 到 `base_link` 的静态坐标变换，支持参数或YAML标定文件配置 |
| `vision_pickup_pipeline.py` | 视觉抓取流水线。检测ArUco标签，将位姿转换到 `base_link` 并发布标签位姿 |
| `aruco_pick_server.py` | AR码引导抓取服务。订阅标签位姿，将物体加入规划场景，生成多种抓取/放置姿态，通过MoveIt2执行多目标抓取放置，成功后清理场景并回到Home |

构建：
```bash
cd <工作区根目录>
colcon build --symlink-install --packages-select vision_pickup_lab
source install/setup.bash
```

## 21.1 视觉抓取完整流程

### 21.1.1 运行参考代码 vision_pickup_lab
```bash
# 终端1: 启动xArm仿真（见"实验环境"前置说明）
ros2 launch xarm_ros2_arm_only arm_only.launch.py

# 终端2: 发布相机TF（另开终端）
ros2 run vision_pickup_lab tf2_camera_broadcaster --ros-args \
  -p x:=0.15 -p z:=0.25 -p child_frame:=camera_link

# 终端3: 启动抓取服务与流水线
ros2 run vision_pickup_lab aruco_pick_server
ros2 run vision_pickup_lab vision_pickup_pipeline
```

流水线检测到AR码后广播其3D位姿，机械臂规划并执行抓取，终端输出 `Pick succeeded`。

`tf2_camera_broadcaster.py` 支持从参数或YAML标定文件加载变换（可使用第19章19.3节手眼标定的结果）:
```bash
# 从参数
python3 tf2_camera_broadcaster.py --ros-args \
  -p x:=0.3 -p y:=0.0 -p z:=0.15 \
  -p roll:=0.0 -p pitch:=0.0 -p yaw:=1.57

# 从标定文件
python3 tf2_camera_broadcaster.py --ros-args \
  -p calibration_file:=/path/to/calib.yaml
```

### 21.1.2 创建实验包
```bash
cd ~/ros2_arm_ws/src
ros2 pkg create ch21_vision_grasp --build-type ament_python --dependencies rclpy sensor_msgs geometry_msgs cv_bridge tf2_ros moveit
cd ch21_vision_grasp
mkdir -p ch21_vision_grasp launch
```

### 21.1.3 编写视觉抓取Pipeline节点
创建 `ch21_vision_grasp/vision_grasp_pipeline.py`:
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
from moveit.planning import MoveItPy, PlanningComponent
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from shape_msgs.msg import SolidPrimitive
import cv2
import cv2.aruco as aruco
import numpy as np
import tf2_ros
import tf2_geometry_msgs
import math
import time
import threading

class VisionGraspPipeline(Node):
    def __init__(self):
        super().__init__('vision_grasp_pipeline')
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.declare_parameter('marker_size', 0.065)
        self.marker_size = self.get_parameter('marker_size').value

        dict_aruco = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        params = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(dict_aruco, params)

        self.moveit = MoveItPy(node_name='moveit_py')
        self.arm = PlanningComponent(
            self.moveit, 'xarm_group', 'gripper_centor_link'
        )
        self.gripper = PlanningComponent(
            self.moveit, 'gripper_group', 'gripper_centor_link'
        )
        self.psm = self.moveit.get_planning_scene_monitor()
        self.arm.set_pose_reference_frame('base_link')
        time.sleep(1)

        self.camera_matrix = None
        self.dist_coeffs = None
        self.latest_pose = None
        self.latest_ids = None

        self.image_sub = self.create_subscription(
            Image, '/image_raw', self.image_cb, 10
        )
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera_info', self.info_cb, 10
        )

        self.srv = self.create_service(Trigger, '/vision_grasp/trigger', self.trigger_cb)
        self.pose_pub = self.create_publisher(PoseStamped, '/vision_grasp/target_pose', 10)
        self.get_logger().info('视觉抓取Pipeline已就绪')
        self.get_logger().info('调用: ros2 service call /vision_grasp/trigger std_srvs/srv/Trigger')

    def info_cb(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info('相机参数已接收')

    def image_cb(self, msg):
        if self.camera_matrix is None:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        self.latest_ids = ids

        if ids is not None:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.camera_matrix, self.dist_coeffs
            )
            aruco.drawDetectedMarkers(frame, corners, ids)

            for i in range(len(ids)):
                aruco.drawAxis(frame, self.camera_matrix, self.dist_coeffs,
                               rvecs[i], tvecs[i], 0.03)

                pose_cam = PoseStamped()
                pose_cam.header.stamp = self.get_clock().now().to_msg()
                pose_cam.header.frame_id = 'camera_link'
                pose_cam.pose.position.x = float(tvecs[i][0][0])
                pose_cam.pose.position.y = float(tvecs[i][0][1])
                pose_cam.pose.position.z = float(tvecs[i][0][2])

                rmat, _ = cv2.Rodrigues(rvecs[i])
                q = self.rotmat_to_quat(rmat)
                pose_cam.pose.orientation.x = q[0]
                pose_cam.pose.orientation.y = q[1]
                pose_cam.pose.orientation.z = q[2]
                pose_cam.pose.orientation.w = q[3]

                try:
                    transform = self.tf_buffer.lookup_transform(
                        'base_link', 'camera_link', rclpy.time.Time()
                    )
                    pose_base = tf2_geometry_msgs.do_transform_pose(pose_cam, transform)
                    self.latest_pose = pose_base
                    self.pose_pub.publish(pose_base)
                except Exception as e:
                    self.get_logger().warn(f'TF变换失败: {e}')

        cv2.putText(frame, f'Markers: {len(ids) if ids is not None else 0}',
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow('Vision Grasp', frame)
        cv2.waitKey(1)

    def trigger_cb(self, request, response):
        if self.latest_pose is None:
            response.success = False
            response.message = '未检测到ArUco标签'
            return response

        self.get_logger().info('触发视觉抓取...')
        threading.Thread(target=self.execute_grasp, daemon=True).start()
        response.success = True
        response.message = '抓取任务已启动'
        return response

    def execute_grasp(self):
        target = self.latest_pose.pose
        self.get_logger().info(f'目标位置: ({target.position.x:.3f}, {target.position.y:.3f}, {target.position.z:.3f})')

        home = [0.0]*6
        pre_grasp = [target.position.x * 0.8, -0.5, 0.5, 0.0, 0.0, 0.0]

        self.arm.set_start_state_to_current_state()
        self.arm.set_joint_value_target(home)
        plan = self.arm.plan()
        if plan: self.arm.execute(plan.trajectory)
        time.sleep(1)

        self.gripper.set_start_state_to_current_state()
        self.gripper.set_joint_value_target([0.65, 0.65])
        plan = self.gripper.plan()
        if plan: self.gripper.execute(plan.trajectory)
        time.sleep(0.5)

        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = 'base_link'
        grasp_pose.pose.position.x = target.position.x
        grasp_pose.pose.position.y = target.position.y
        grasp_pose.pose.position.z = target.position.z + 0.15
        grasp_pose.pose.orientation = target.orientation

        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(grasp_pose.pose, 'gripper_centor_link')
        plan = self.arm.plan()
        if plan:
            self.arm.execute(plan.trajectory)
            time.sleep(1)

        lower_pose = grasp_pose
        lower_pose.pose.position.z = target.position.z + 0.02
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(lower_pose.pose, 'gripper_centor_link')
        plan = self.arm.plan()
        if plan:
            self.arm.execute(plan.trajectory)
            time.sleep(0.5)

        self.gripper.set_start_state_to_current_state()
        self.gripper.set_joint_value_target([0.0, 0.0])
        plan = self.gripper.plan()
        if plan: self.gripper.execute(plan.trajectory)
        time.sleep(0.5)

        # attach
        aco = AttachedCollisionObject()
        aco.link_name = 'gripper_centor_link'
        aco.object.id = 'target_object'
        aco.object.operation = CollisionObject.ADD
        aco.touch_links = ['gripper_1_link', 'gripper_2_link']
        self.psm.process_attached_collision_object(aco)
        time.sleep(0.5)

        lift_pose = grasp_pose
        lift_pose.pose.position.z += 0.15
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(lift_pose.pose, 'gripper_centor_link')
        plan = self.arm.plan()
        if plan: self.arm.execute(plan.trajectory)
        time.sleep(1)

        place_pose = PoseStamped()
        place_pose.header.frame_id = 'base_link'
        place_pose.pose.position.x = target.position.x - 0.2
        place_pose.pose.position.y = target.position.y + 0.2
        place_pose.pose.position.z = target.position.z + 0.15
        place_pose.pose.orientation = target.orientation

        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(place_pose.pose, 'gripper_centor_link')
        plan = self.arm.plan()
        if plan: self.arm.execute(plan.trajectory)
        time.sleep(1)

        place_lower = place_pose
        place_lower.pose.position.z = target.position.z + 0.02
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(place_lower.pose, 'gripper_centor_link')
        plan = self.arm.plan()
        if plan: self.arm.execute(plan.trajectory)
        time.sleep(0.5)

        aco2 = AttachedCollisionObject()
        aco2.link_name = 'gripper_centor_link'
        aco2.object.id = 'target_object'
        aco2.object.operation = CollisionObject.REMOVE
        self.psm.process_attached_collision_object(aco2)

        self.gripper.set_start_state_to_current_state()
        self.gripper.set_joint_value_target([0.65, 0.65])
        plan = self.gripper.plan()
        if plan: self.gripper.execute(plan.trajectory)
        time.sleep(1)

        self.arm.set_start_state_to_current_state()
        self.arm.set_joint_value_target(home)
        plan = self.arm.plan()
        if plan: self.arm.execute(plan.trajectory)

        self.get_logger().info('视觉抓取完成!')

    @staticmethod
    def rotmat_to_quat(r):
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
    node = VisionGraspPipeline()
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

> **注意**：该Pipeline依赖 `base_link`→`camera_link` 的TF变换，可使用参考代码 `tf2_camera_broadcaster.py`（见21.1.1）或第19章19.3.11节的 `calibration_tf_publisher.py` 提供。

### 21.1.4 编写launch文件
创建 `launch/vision_grasp.launch.py`:
```python
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            parameters=[{
                'video_device': '/dev/video0',
                'image_width': 640,
                'image_height': 480,
                'camera_frame_id': 'camera_link',
                'pixel_format': 'yuyv',
            }],
        ),
        Node(
            package='ch21_vision_grasp',
            executable='vision_grasp_pipeline',
            name='vision_grasp_pipeline',
            parameters=[{'marker_size': 0.065}],
        ),
    ])
```

### 21.1.5 配置setup.py并编译运行
```python
entry_points={
    'console_scripts': [
        'vision_grasp_pipeline = ch21_vision_grasp.vision_grasp_pipeline:main',
    ],
},
```

```bash
cd ~/ros2_arm_ws
colcon build --packages-select ch21_vision_grasp
source install/setup.bash

# 终端1: 启动MoveIt2仿真（见"实验环境"前置说明）
ros2 launch xarm_ros2_arm_only arm_only_move_group.launch.py
# 或 ros2 launch xarm_ros2_arm_only arm_only.launch.py

# 终端2: 发布相机TF（参考代码或第19章标定结果）
ros2 run vision_pickup_lab tf2_camera_broadcaster --ros-args \
  -p x:=0.15 -p z:=0.25 -p child_frame:=camera_link

# 终端3: 启动视觉抓取Pipeline (包含相机驱动)
ros2 launch ch21_vision_grasp vision_grasp.launch.py

# 或分开启动:
# ros2 run usb_cam usb_cam_node_exe
# ros2 run ch21_vision_grasp vision_grasp_pipeline --ros-args -p marker_size:=0.065

# 将ArUco标签放在相机视野内
# 终端4: 触发抓取
ros2 service call /vision_grasp/trigger std_srvs/srv/Trigger
```

### 21.1.6 测试不同场景
```bash
# 使用不同尺寸的标签
ros2 run ch21_vision_grasp vision_grasp_pipeline --ros-args -p marker_size:=0.094

# 查看目标位姿
ros2 topic echo /vision_grasp/target_pose

# 查看检测图像
rqt_image_view /image_raw
```

### 21.1.7 编写简化测试节点
创建 `ch21_vision_grasp/simple_vision_test.py`:
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np

class SimpleVisionTest(Node):
    def __init__(self):
        super().__init__('simple_vision_test')
        self.bridge = CvBridge()
        self.declare_parameter('marker_size', 0.065)
        self.marker_size = self.get_parameter('marker_size').value

        dict_aruco = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        params = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(dict_aruco, params)

        self.camera_matrix = np.array([[530, 0, 320], [0, 530, 240], [0, 0, 1]], dtype=float)
        self.dist_coeffs = np.zeros(5)

        self.sub = self.create_subscription(Image, '/image_raw', self.image_cb, 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/vision/test_pose', 10)
        self.get_logger().info('视觉测试节点已启动')

    def image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            return
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is not None:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.camera_matrix, self.dist_coeffs
            )
            aruco.drawDetectedMarkers(frame, corners, ids)
            for i in range(len(ids)):
                aruco.drawAxis(frame, self.camera_matrix, self.dist_coeffs,
                               rvecs[i], tvecs[i], 0.03)
                p = PoseStamped()
                p.header.stamp = self.get_clock().now().to_msg()
                p.header.frame_id = 'camera_link'
                p.pose.position.x = float(tvecs[i][0][0])
                p.pose.position.y = float(tvecs[i][0][1])
                p.pose.position.z = float(tvecs[i][0][2])
                p.pose.orientation.w = 1.0
                self.pose_pub.publish(p)
                self.get_logger().info(
                    f'标签{ids[i][0]}: ({tvecs[i][0][0]:.3f}, {tvecs[i][0][1]:.3f}, {tvecs[i][0][2]:.3f})'
                )
        cv2.imshow('Test', frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleVisionTest()
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

## 21.2 综合实训：智能机器人产线

### 21.2.1 创建综合实训包
```bash
cd ~/ros2_arm_ws/src
ros2 pkg create ch21_factory --build-type ament_python --dependencies rclpy sensor_msgs geometry_msgs std_srvs action_msgs cv_bridge tf2_ros moveit
cd ch21_factory
mkdir -p ch21_factory launch actions
```

### 21.2.2 定义Action接口
创建 `actions/Pipeline.action`:
```
# 智能产线任务编排
string recipe_text  # 任务描述文本
---
bool success
string message
---
# 反馈
int32 current_step
int32 total_steps
string step_name
```

创建 `actions/DetectObject.action`:
```
# 视觉检测
string target_name
---
bool detected
geometry_msgs/PoseStamped object_pose
float32 confidence
---
string status
```

### 21.2.3 编写视觉检测模块
创建 `ch21_factory/vision_module.py`:
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
import tf2_ros
import tf2_geometry_msgs

class VisionModule(Node):
    def __init__(self):
        super().__init__('vision_module')
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.declare_parameter('marker_size', 0.065)
        self.marker_size = self.get_parameter('marker_size').value

        dict_aruco = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        params = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(dict_aruco, params)

        self.camera_matrix = None
        self.dist_coeffs = None
        self.latest_frame = None

        self.image_sub = self.create_subscription(Image, '/image_raw', self.image_cb, 10)
        self.info_sub = self.create_subscription(CameraInfo, '/camera_info', self.info_cb, 10)

    def info_cb(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)

    def image_cb(self, msg):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            pass

    def detect_marker(self, target_id=None):
        if self.latest_frame is None or self.camera_matrix is None:
            return None

        gray = cv2.cvtColor(self.latest_frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        if ids is None:
            return None

        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
            corners, self.marker_size, self.camera_matrix, self.dist_coeffs
        )

        results = []
        for i in range(len(ids)):
            marker_id = int(ids[i][0])
            if target_id is not None and marker_id != target_id:
                continue

            pose_cam = PoseStamped()
            pose_cam.header.frame_id = 'camera_link'
            pose_cam.pose.position.x = float(tvecs[i][0][0])
            pose_cam.pose.position.y = float(tvecs[i][0][1])
            pose_cam.pose.position.z = float(tvecs[i][0][2])
            pose_cam.pose.orientation.w = 1.0

            try:
                transform = self.tf_buffer.lookup_transform(
                    'base_link', 'camera_link', rclpy.time.Time()
                )
                pose_base = tf2_geometry_msgs.do_transform_pose(pose_cam, transform)
                results.append((marker_id, pose_base))
            except Exception:
                results.append((marker_id, pose_cam))

        return results

def main(args=None):
    rclpy.init(args=args)
    node = VisionModule()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 21.2.4 编写运动控制模块
创建 `ch21_factory/motion_module.py`:
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy, PlanningComponent
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from shape_msgs.msg import SolidPrimitive
import time

class MotionModule(Node):
    def __init__(self):
        super().__init__('motion_module')
        self.moveit = MoveItPy(node_name='moveit_py')
        self.arm = PlanningComponent(self.moveit, 'xarm_group', 'gripper_centor_link')
        self.gripper = PlanningComponent(self.moveit, 'gripper_group', 'gripper_centor_link')
        self.psm = self.moveit.get_planning_scene_monitor()
        self.arm.set_pose_reference_frame('base_link')
        time.sleep(1)
        self.get_logger().info('运动模块已初始化')

    def go_home(self):
        self.arm.set_start_state_to_current_state()
        self.arm.set_joint_value_target([0.0]*6)
        plan = self.arm.plan()
        if plan:
            self.arm.execute(plan.trajectory)
            return True
        return False

    def open_gripper(self):
        self.gripper.set_start_state_to_current_state()
        self.gripper.set_joint_value_target([0.65, 0.65])
        plan = self.gripper.plan()
        if plan:
            self.gripper.execute(plan.trajectory)

    def close_gripper(self):
        self.gripper.set_start_state_to_current_state()
        self.gripper.set_joint_value_target([0.0, 0.0])
        plan = self.gripper.plan()
        if plan:
            self.gripper.execute(plan.trajectory)

    def move_to_pose(self, x, y, z, roll=0, pitch=math.pi/2, yaw=0):
        import math
        from tf_transformations import quaternion_from_euler
        target = PoseStamped()
        target.header.frame_id = 'base_link'
        target.pose.position.x = x
        target.pose.position.y = y
        target.pose.position.z = z
        q = quaternion_from_euler(roll, pitch, yaw)
        target.pose.orientation.x = q[0]
        target.pose.orientation.y = q[1]
        target.pose.orientation.z = q[2]
        target.pose.orientation.w = q[3]

        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(target.pose, 'gripper_centor_link')
        plan = self.arm.plan()
        if plan:
            self.arm.execute(plan.trajectory)
            return True
        return False

    def move_to_joints(self, joints):
        self.arm.set_start_state_to_current_state()
        self.arm.set_joint_value_target(joints)
        plan = self.arm.plan()
        if plan:
            self.arm.execute(plan.trajectory)
            return True
        return False

    def pick_object(self, pose_stamped):
        self.open_gripper()
        time.sleep(0.5)

        pre_pick = PoseStamped()
        pre_pick.header.frame_id = 'base_link'
        pre_pick.pose.position.x = pose_stamped.pose.position.x
        pre_pick.pose.position.y = pose_stamped.pose.position.y
        pre_pick.pose.position.z = pose_stamped.pose.position.z + 0.1
        pre_pick.pose.orientation = pose_stamped.pose.orientation

        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(pre_pick.pose, 'gripper_centor_link')
        plan = self.arm.plan()
        if plan:
            self.arm.execute(plan.trajectory)
        time.sleep(0.5)

        pick_pose = PoseStamped()
        pick_pose.header = pre_pick.header
        pick_pose.pose.position.x = pose_stamped.pose.position.x
        pick_pose.pose.position.y = pose_stamped.pose.position.y
        pick_pose.pose.position.z = pose_stamped.pose.position.z
        pick_pose.pose.orientation = pose_stamped.pose.orientation

        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(pick_pose.pose, 'gripper_centor_link')
        plan = self.arm.plan()
        if plan:
            self.arm.execute(plan.trajectory)
        time.sleep(0.5)

        self.close_gripper()
        time.sleep(0.5)

        aco = AttachedCollisionObject()
        aco.link_name = 'gripper_centor_link'
        aco.object.id = 'workpiece'
        aco.object.operation = CollisionObject.ADD
        aco.touch_links = ['gripper_1_link', 'gripper_2_link']
        self.psm.process_attached_collision_object(aco)

        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(pre_pick.pose, 'gripper_centor_link')
        plan = self.arm.plan()
        if plan:
            self.arm.execute(plan.trajectory)
        return True

    def place_object(self, x, y, z):
        place_pose = PoseStamped()
        place_pose.header.frame_id = 'base_link'
        place_pose.pose.position.x = x
        place_pose.pose.position.y = y
        place_pose.pose.position.z = z + 0.1
        place_pose.pose.orientation.w = 1.0

        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(place_pose.pose, 'gripper_centor_link')
        plan = self.arm.plan()
        if plan:
            self.arm.execute(plan.trajectory)
        time.sleep(0.5)

        down_pose = place_pose
        down_pose.pose.position.z = z
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(down_pose.pose, 'gripper_centor_link')
        plan = self.arm.plan()
        if plan:
            self.arm.execute(plan.trajectory)
        time.sleep(0.5)

        aco = AttachedCollisionObject()
        aco.link_name = 'gripper_centor_link'
        aco.object.id = 'workpiece'
        aco.object.operation = CollisionObject.REMOVE
        self.psm.process_attached_collision_object(aco)

        self.open_gripper()
        time.sleep(0.5)

        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(place_pose.pose, 'gripper_centor_link')
        plan = self.arm.plan()
        if plan:
            self.arm.execute(plan.trajectory)
        return True

import math
from tf_transformations import quaternion_from_euler

def main(args=None):
    rclpy.init(args=args)
    node = MotionModule()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 21.2.5 编写Pipeline编排服务器
创建 `ch21_factory/pipeline_server.py`:
```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
import time
import json

class FactoryPipelineServer(Node):
    def __init__(self):
        super().__init__('factory_pipeline_server')
        self.vision = None
        self.motion = None
        self.current_step = 0
        self.total_steps = 5

        self.srv = self.create_service(Trigger, '/factory/start_pipeline', self.start_cb)
        self.get_logger().info('智能产线Pipeine Server已就绪')
        self.get_logger().info('启动: ros2 service call /factory/start_pipeline std_srvs/srv/Trigger')

    def start_cb(self, request, response):
        self.get_logger().info('启动智能产线流程...')
        try:
            self.run_pipeline()
            response.success = True
            response.message = '产线流程完成'
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def run_pipeline(self):
        self.get_logger().info(f'=== 步骤1/{self.total_steps}: 初始化系统 ===')
        self.current_step = 1
        self.init_modules()
        time.sleep(1)

        self.get_logger().info(f'=== 步骤2/{self.total_steps}: 视觉检测 ===')
        self.current_step = 2
        detected_objects = self.detect_objects()
        if not detected_objects:
            raise Exception('未检测到目标物体')
        target_pose = detected_objects[0]
        self.get_logger().info(f'检测到目标: ({target_pose.pose.position.x:.3f}, {target_pose.pose.position.y:.3f}, {target_pose.pose.position.z:.3f})')
        time.sleep(1)

        self.get_logger().info(f'=== 步骤3/{self.total_steps}: 规划抓取路径 ===')
        self.current_step = 3
        self.motion.open_gripper()
        self.motion.go_home()
        time.sleep(1)

        self.get_logger().info(f'=== 步骤4/{self.total_steps}: 执行抓取 ===')
        self.current_step = 4
        self.motion.pick_object(target_pose)
        time.sleep(1)

        self.get_logger().info(f'=== 步骤5/{self.total_steps}: 放置到目标区域 ===')
        self.current_step = 5
        place_x = target_pose.pose.position.x - 0.2
        place_y = target_pose.pose.position.y + 0.2
        self.motion.place_object(place_x, place_y, 0.02)
        time.sleep(1)

        self.get_logger().info('回到Home位姿')
        self.motion.go_home()

        self.get_logger().info('=== 智能产线流程完成! ===')

    def init_modules(self):
        from ch21_factory.vision_module import VisionModule
        from ch21_factory.motion_module import MotionModule
        self.vision = VisionModule()
        self.motion = MotionModule()

    def detect_objects(self):
        results = self.vision.detect_marker()
        if results:
            return [pose for _, pose in results]
        return []

def main(args=None):
    rclpy.init(args=args)
    node = FactoryPipelineServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 21.2.6 编写产线动作客户端
创建 `ch21_factory/pipeline_client.py`:
```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_srvs.srv import Trigger

class PipelineClient(Node):
    def __init__(self):
        super().__init__('pipeline_client')
        self.client = self.create_client(Trigger, '/factory/start_pipeline')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待Pipeline Server...')
        self.get_logger().info('Pipeline Client就绪')

    def start(self):
        req = Trigger.Request()
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result():
            self.get_logger().info(f'结果: {future.result().message}')
        else:
            self.get_logger().error('调用失败')

def main(args=None):
    rclpy.init(args=args)
    node = PipelineClient()
    node.start()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 21.2.7 编写多物体分拣演示
创建 `ch21_factory/sorting_demo.py`:
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
import time

class SortingDemo(Node):
    def __init__(self):
        super().__init__('sorting_demo')
        self.bridge = CvBridge()
        dict_aruco = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        params = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(dict_aruco, params)

        self.sub = self.create_subscription(Image, '/image_raw', self.image_cb, 10)
        self.targets = {
            1: {'name': '红色零件', 'place_zone': 'Zone_A'},
            2: {'name': '蓝色零件', 'place_zone': 'Zone_B'},
            3: {'name': '绿色零件', 'place_zone': 'Zone_C'},
        }

    def image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)
            for i in range(len(ids)):
                mid = int(ids[i][0])
                if mid in self.targets:
                    c = corners[i][0]
                    cx = int(c[:, 0].mean())
                    cy = int(c[:, 1].mean())
                    info = f'{self.targets[mid]["name"]} → {self.targets[mid]["place_zone"]}'
                    cv2.putText(frame, info, (cx-30, cy),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        cv2.putText(frame, 'Sorting Demo', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
        cv2.imshow('Sorting Demo', frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = SortingDemo()
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

### 21.2.8 编写launch文件
创建 `launch/factory_system.launch.py`:
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            parameters=[{
                'video_device': '/dev/video0',
                'image_width': 640,
                'image_height': 480,
                'camera_frame_id': 'camera_link',
            }],
        ),
        Node(
            package='ch21_factory',
            executable='vision_module',
            name='vision_module',
            parameters=[{'marker_size': 0.065}],
        ),
        Node(
            package='ch21_factory',
            executable='pipeline_server',
            name='pipeline_server',
        ),
    ])
```

### 21.2.9 配置setup.py并编译运行
```python
entry_points={
    'console_scripts': [
        'vision_module = ch21_factory.vision_module:main',
        'motion_module = ch21_factory.motion_module:main',
        'pipeline_server = ch21_factory.pipeline_server:main',
        'pipeline_client = ch21_factory.pipeline_client:main',
        'sorting_demo = ch21_factory.sorting_demo:main',
    ],
},
```

```bash
cd ~/ros2_arm_ws
colcon build --packages-select ch21_factory
source install/setup.bash

# 终端1: 启动MoveIt2仿真（见"实验环境"前置说明）
ros2 launch xarm_ros2_arm_only arm_only_move_group.launch.py
# 或 ros2 launch xarm_ros2_arm_only arm_only.launch.py

# 终端2: 启动产线系统
ros2 launch ch21_factory factory_system.launch.py

# 终端3: 启动产线流程
ros2 service call /factory/start_pipeline std_srvs/srv/Trigger

# 或运行分拣演示
ros2 run ch21_factory sorting_demo
```

### 21.2.10 扩展练习
```bash
# 练习1: 添加更多ArUco标签和对应的放置区域
# 修改targets字典, 增加4-6号标签

# 练习2: 集成YOLO检测替代ArUco
# 使用第19章(19.2节)中的YOLO节点检测物体类别, 输入到Pipeline

# 练习3: 添加碰撞检测和异常处理
# 在抓取过程中检查机械臂是否与障碍物碰撞

# 练习4: 实现连续分拣多个物体
# 循环检测-抓取-放置, 直到所有物体分拣完成
```

## 实验结果与分析
- 视觉抓取Pipeline完整实现了检测→定位→抓取→放置流程
- ArUco标签提供了稳定的视觉特征用于位姿估计
- TF坐标变换将相机坐标系中的目标位姿转换到机器人基坐标系
- MoveIt2根据目标位姿进行逆运动学求解和避障规划
- 完整的智能产线系统集成了视觉检测、运动规划和抓取控制
- Action/Service机制实现了模块间的解耦和协同
- Pipeline编排服务器统一管理多步骤任务流程
- 系统具有良好的可扩展性, 可以添加更多功能模块
- 参考代码 `vision_pickup_lab` 的 `tf2_camera_broadcaster`/`vision_pickup_pipeline`/`aruco_pick_server` 分别对应本章的相机TF发布、标签检测与抓取执行环节，可对照验证自实现系统

## 思考题
1. 在视觉抓取流程中, 哪个环节最容易出错? 如何处理异常?
2. 如果相机视野中有多个ArUco标签, 如何选择正确的抓取目标?
3. 如何提高视觉抓取的成功率? 哪些因素会影响精度?
4. 在没有深度相机的情况下, 如何获取目标的深度信息?
5. 在大规模产线场景中, 如何实现多机械臂的协同作业?
6. 如何处理抓取失败的异常情况? 如何实现错误恢复?
7. 产线系统的实时性能如何优化? 哪些环节是性能瓶颈?
8. 如何将VLM集成到产线系统中, 实现更智能的决策?
