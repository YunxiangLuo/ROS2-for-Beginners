#!/usr/bin/env python3

"""
第31章 综合项目 - 城区自动驾驶主管线节点

该节点是整个自动驾驶系统的核心管线，负责任务调度、模块协调和状态管理。
按顺序执行：传感器采集 → 感知 → 定位 → 规划 → 控制 → 安全监控。
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

import numpy as np
from enum import Enum, auto
import threading
import time
from collections import deque

import std_srvs.srv
from std_msgs.msg import Float32, Float32MultiArray, Bool, String, Header
from geometry_msgs.msg import PoseStamped, TwistStamped, Point
from sensor_msgs.msg import Image, PointCloud2, NavSatFix, Imu
from nav_msgs.msg import OccupancyGrid, Path
from visualization_msgs.msg import Marker, MarkerArray
from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray



class PipelineState(Enum):
    INITIALIZING = auto()
    SENSOR_CHECK = auto()
    LOCALIZATION_READY = auto()
    PLANNING_READY = auto()
    DRIVING = auto()
    EMERGENCY_STOP = auto()
    PAUSED = auto()
    COMPLETED = auto()
    FAILED = auto()


class AutonomousDrivingPipeline(Node):
    """
    自动驾驶主管线：协调所有模块的运行，管理系统状态，监控性能指标。
    """

    def __init__(self):
        super().__init__('autonomous_driving_pipeline')

        # ── 系统状态 ──
        self.state = PipelineState.INITIALIZING
        self.state_lock = threading.Lock()
        self.error_count = 0
        self.start_time = None
        self.last_cycle_time = None

        # ── 性能监控 ──
        self.cycle_times = deque(maxlen=100)
        self.module_latencies = {}
        self.pipeline_fps = 0.0

        # ── 数据缓冲 ──
        self.latest_sensor_data = {}
        self.latest_perception = {}
        self.latest_localization = {}
        self.latest_planning = {}
        self.latest_control = {}
        self.data_buffer_lock = threading.Lock()

        # ── 安全状态 ──
        self.emergency_brake_active = False
        self.collision_warning = False
        self.safety_status = DiagnosticStatus()

        # ── 运行控制 ──
        self.enable_autonomous = False
        self.target_goal = None
        self.pipeline_rate = 20.0

        # ── 回调组 ──
        self.sensor_cb_group = MutuallyExclusiveCallbackGroup()
        self.perception_cb_group = MutuallyExclusiveCallbackGroup()
        self.planning_cb_group = MutuallyExclusiveCallbackGroup()
        self.control_cb_group = MutuallyExclusiveCallbackGroup()
        self.main_cb_group = ReentrantCallbackGroup()

        # ── 创建各引擎模块 ──
        self.get_logger().info('Initializing autonomous driving pipeline...')
        self._init_modules()

        # ── 系统状态发布 ──
        self.status_pub = self.create_publisher(
            DiagnosticStatus, '/system/pipeline_status', 10)
        self.metrics_pub = self.create_publisher(
            MarkerArray, '/system/performance_metrics', 10)

        # ── 主循环定时器 ──
        self.main_timer = self.create_timer(
            1.0 / self.pipeline_rate, self.pipeline_cycle, callback_group=self.main_cb_group)

        # ── 服务 ──
        self.enable_srv = self.create_service(
            std_srvs.srv.SetBool, '/pipeline/enable',
            self.enable_callback, callback_group=self.main_cb_group)
        self.set_goal_srv = self.create_service(
            std_srvs.srv.SetBool, '/pipeline/set_goal',
            self.set_goal_callback, callback_group=self.main_cb_group)

        self.get_logger().info('Pipeline initialized. Waiting for enable signal.')
        self.state = PipelineState.SENSOR_CHECK

    def _init_modules(self):
        """初始化所有功能模块"""
        try:
            from .carla_sensor_driver.sensor_driver import SensorDriver
            from .perception_node.perception_node import PerceptionEngine
            from .localization_node.localization_node import LocalizationEngine
            from .planning_node.planning_node import PlanningEngine
            from .control_node.control_node import ControlEngine
            from .safety_monitor_node.safety_monitor import SafetyMonitorEngine

            self.sensor_driver = SensorDriver(self, self.sensor_cb_group)
            self.perception = PerceptionEngine(self, self.perception_cb_group)
            self.localization = LocalizationEngine(self, self.sensor_cb_group)
            self.planning = PlanningEngine(self, self.planning_cb_group)
            self.control = ControlEngine(self, self.control_cb_group)
            self.safety_monitor = SafetyMonitorEngine(self, self.main_cb_group)

            self.get_logger().info('All modules initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize modules: {e}')
            self.state = PipelineState.FAILED

    def pipeline_cycle(self):
        """主管线循环：协调各模块按序执行"""
        cycle_start = time.perf_counter()

        if not self.enable_autonomous:
            return

        with self.state_lock:
            if self.state not in [PipelineState.DRIVING, PipelineState.LOCALIZATION_READY,
                                  PipelineState.PLANNING_READY]:
                return

        try:
            # 1. 检查传感器数据新鲜度
            if not self._check_sensor_freshness():
                self.get_logger().warn('Sensor data stale, entering safe mode')
                self._safe_stop()
                return

            # 2. 执行感知（绑定传感器回调触发）
            self._run_perception()

            # 3. 执行定位更新
            self._run_localization()

            # 4. 规划（依赖定位和感知结果）
            if self.latest_localization and self.latest_perception:
                self._run_planning()

            # 5. 检查安全状态
            safety_ok = self._run_safety_check()
            if not safety_ok:
                return

            # 6. 执行控制
            if self.latest_planning:
                self._run_control()

            # 7. 发布状态和指标
            self._publish_status()
            self._compute_metrics(cycle_start)

        except Exception as e:
            self.get_logger().error(f'Pipeline cycle error: {e}')
            self.error_count += 1
            if self.error_count > 10:
                self.state = PipelineState.FAILED

    def _check_sensor_freshness(self):
        """检查传感器数据是否在超时范围内"""
        max_age = 0.5
        now = self.get_clock().now()
        with self.data_buffer_lock:
            for topic, data in self.latest_sensor_data.items():
                age = (now - data['stamp']).nanoseconds * 1e-9
                if age > max_age:
                    return False
        return True

    def _run_perception(self):
        """执行感知处理"""
        start = time.perf_counter()
        with self.data_buffer_lock:
            rgb_image = self.latest_sensor_data.get('camera_rgb')
            lidar_cloud = self.latest_sensor_data.get('lidar')
            depth_image = self.latest_sensor_data.get('camera_depth')

        if rgb_image is not None:
            result = self.perception.process(rgb_image, lidar_cloud)
            with self.data_buffer_lock:
                self.latest_perception = result

        latency = (time.perf_counter() - start) * 1000
        self.module_latencies['perception'] = latency

    def _run_localization(self):
        """执行定位融合"""
        start = time.perf_counter()
        with self.data_buffer_lock:
            gnss = self.latest_sensor_data.get('gnss')
            imu = self.latest_sensor_data.get('imu')
            speed = self.latest_sensor_data.get('speed')

        if all(v is not None for v in [gnss, imu, speed]):
            pose = self.localization.update(gnss, imu, speed)
            with self.data_buffer_lock:
                self.latest_localization = pose

        latency = (time.perf_counter() - start) * 1000
        self.module_latencies['localization'] = latency

    def _run_planning(self):
        """执行路径规划"""
        start = time.perf_counter()

        obstacles = self.latest_perception.get('obstacles', [])
        traffic_light = self.latest_perception.get('traffic_light', None)
        lane_markers = self.latest_perception.get('lane_markers', None)
        occupancy = self.latest_perception.get('occupancy_grid', None)

        trajectory = self.planning.update(
            pose=self.latest_localization,
            obstacles=obstacles,
            traffic_light=traffic_light,
            lane_markers=lane_markers,
            occupancy_grid=occupancy,
            target_goal=self.target_goal
        )

        with self.data_buffer_lock:
            self.latest_planning = trajectory

        latency = (time.perf_counter() - start) * 1000
        self.module_latencies['planning'] = latency

    def _run_safety_check(self):
        """运行安全检查，必要时触发紧急制动"""
        start = time.perf_counter()

        safety_result = self.safety_monitor.check(
            pose=self.latest_localization,
            trajectory=self.latest_planning,
            obstacles=self.latest_perception.get('obstacles', [])
        )

        self.safety_status = safety_result['status']

        if safety_result['emergency']:
            self.get_logger().warn(f"Emergency stop triggered: {safety_result['reason']}")
            self.emergency_brake_active = True
            self.control.emergency_stop()
            self.state = PipelineState.EMERGENCY_STOP
            return False

        if self.emergency_brake_active and not safety_result['emergency']:
            self.emergency_brake_active = False
            self.get_logger().info('Emergency stop cleared, resuming')
            self.state = PipelineState.DRIVING

        latency = (time.perf_counter() - start) * 1000
        self.module_latencies['safety'] = latency
        return True

    def _run_control(self):
        """执行运动控制"""
        start = time.perf_counter()

        control_cmd = self.control.update(
            trajectory=self.latest_planning,
            pose=self.latest_localization,
            speed=self.latest_sensor_data.get('speed', Float32())
        )

        with self.data_buffer_lock:
            self.latest_control = control_cmd

        latency = (time.perf_counter() - start) * 1000
        self.module_latencies['control'] = latency

    def _safe_stop(self):
        """安全停车"""
        self.get_logger().info('Performing safe stop')
        self.control.emergency_stop()
        self.state = PipelineState.EMERGENCY_STOP

    def _publish_status(self):
        """发布系统诊断状态"""
        status = DiagnosticStatus()
        status.level = DiagnosticStatus.OK if self.state == PipelineState.DRIVING else \
            DiagnosticStatus.WARN if self.state == PipelineState.EMERGENCY_STOP else \
            DiagnosticStatus.ERROR
        status.name = 'autonomous_driving_pipeline'
        status.message = f'State: {self.state.name}, Errors: {self.error_count}'

        for module, latency in self.module_latencies.items():
            kv = DiagnosticStatus.KeyValue()
            kv.key = f'{module}_latency_ms'
            kv.value = f'{latency:.1f}'
            status.values.append(kv)

        self.status_pub.publish(status)

    def _compute_metrics(self, cycle_start):
        """计算管线性能指标"""
        cycle_time = (time.perf_counter() - cycle_start) * 1000
        self.cycle_times.append(cycle_time)
        self.pipeline_fps = 1000.0 / (sum(self.cycle_times) / len(self.cycle_times) + 1e-6)

        if len(self.cycle_times) % 50 == 0:
            avg_cycle = sum(self.cycle_times) / len(self.cycle_times)
            self.get_logger().info(
                f'Pipeline avg cycle: {avg_cycle:.1f}ms ({self.pipeline_fps:.1f} FPS) | '
                f'Modules: {self.module_latencies}'
            )

    def enable_callback(self, request, response):
        """启用/禁用自动驾驶"""
        self.enable_autonomous = request.data
        if self.enable_autonomous:
            if self.state in [PipelineState.PAUSED, PipelineState.SENSOR_CHECK]:
                self.state = PipelineState.DRIVING
                self.start_time = self.get_clock().now()
                self.get_logger().info('Autonomous driving ENABLED')
        else:
            self.state = PipelineState.PAUSED
            self.control.emergency_stop()
            self.get_logger().info('Autonomous driving DISABLED')
        response.success = True
        response.message = f'Pipeline {"enabled" if request.data else "disabled"}'
        return response

    def set_goal_callback(self, request, response):
        """设置目标点"""
        if hasattr(request, 'goal'):
            self.target_goal = {
                'x': request.goal.position.x,
                'y': request.goal.position.y,
                'z': request.goal.position.z,
            }
            self.planning.set_goal(self.target_goal)
            self.get_logger().info(f'Target goal set to: {self.target_goal}')
            response.success = True
            response.message = 'Goal set'
        else:
            response.success = False
            response.message = 'Invalid goal'
        return response

    def on_carla_tick(self, sensor_data):
        """CARLA tick回调：更新传感器数据缓冲"""
        with self.data_buffer_lock:
            self.latest_sensor_data.update(sensor_data)
            self.latest_sensor_data = {
                k: v for k, v in self.latest_sensor_data.items()
                if (self.get_clock().now() - v['stamp']).nanoseconds * 1e-9 < 2.0
            }

    def get_state(self):
        """获取当前管线状态"""
        with self.state_lock:
            return self.state

    def shutdown(self):
        """安全关闭所有模块"""
        self.get_logger().info('Shutting down pipeline...')
        self.enable_autonomous = False
        self.control.emergency_stop()
        self.sensor_driver.shutdown()
        self.perception.shutdown()
        self.localization.shutdown()
        self.planning.shutdown()
        self.control.shutdown()
        self.safety_monitor.shutdown()


def main(args=None):
    rclpy.init(args=args)

    pipeline = AutonomousDrivingPipeline()

    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(pipeline)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pipeline.get_logger().info('Pipeline interrupted by user')
    finally:
        pipeline.shutdown()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
