#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool, Float32, String, Header
from geometry_msgs.msg import Twist, Pose, PoseStamped
from nav_msgs.msg import Odometry, Path
import numpy as np
import json
import time
import math
import os
import csv
from datetime import datetime
from dataclasses import dataclass, field, asdict
from typing import List, Optional


INF = float('inf')


@dataclass
class ScenarioConfig:
    name: str
    map_name: str = 'Town03'
    duration_s: int = 120
    max_speed_ms: float = 10.0
    fault_profile: str = 'baseline'
    waypoints: List[tuple] = field(default_factory=list)
    weather: str = 'clear'
    traffic_density: int = 0


@dataclass
class TestMetrics:
    __test__ = False

    avg_speed: float = 0.0
    max_speed: float = 0.0
    min_ttc: float = INF
    avg_ttc: float = 0.0
    deviation_rate: float = 0.0
    rms_jerk: float = 0.0
    max_jerk: float = 0.0
    planning_latency_ms: float = 0.0
    control_rmse: float = 0.0
    task_completed: bool = False
    collision_count: int = 0
    recovery_time_s: float = 0.0


class IntegrationTestNode(Node):

    def __init__(self):
        super().__init__('integration_test_node')

        self.declare_parameter('scenario', 'straight_lane')
        self.declare_parameter('duration', 120.0)
        self.declare_parameter('output_dir', 'results')
        self.declare_parameter('fault_profile', 'baseline')

        self.scenario_name = self.get_parameter('scenario').value
        self.test_duration = self.get_parameter('duration').value
        self.output_dir = self.get_parameter('output_dir').value
        self.fault_profile = self.get_parameter('fault_profile').value

        os.makedirs(self.output_dir, exist_ok=True)

        self.test_start_time = None
        self.test_active = False

        self.velocity_log = []
        self.time_log = []
        self.ttc_log = []
        self.deviation_log = []
        self.collision_events = []
        self.control_errors = []
        self.planning_latencies = []

        self.odom_sub = self.create_subscription(
            Odometry, '/vehicle/odometry',
            self.odom_callback, 10)
        self.collision_sub = self.create_subscription(
            Bool, '/safety/collision_warning',
            self.collision_callback, 10)
        self.deviation_sub = self.create_subscription(
            Float32, '/safety/deviation',
            self.deviation_callback, 10)
        self.status_sub = self.create_subscription(
            String, '/vehicle/status',
            self.status_callback, 10)

        self.cmd_pub = self.create_publisher(
            Twist, '/control/cmd', 10)

        self.timer = self.create_timer(1.0 / 50.0, self.test_loop)
        self.status_timer = self.create_timer(1.0, self.report_progress)

        self.get_logger().info(
            f'IntegrationTest starting: scenario={self.scenario_name}, '
            f'duration={self.test_duration}s')

    def odom_callback(self, msg):
        if not self.test_active:
            return
        now = time.time()
        vel = math.sqrt(
            msg.twist.twist.linear.x ** 2 +
            msg.twist.twist.linear.y ** 2
        )
        self.velocity_log.append(vel)
        self.time_log.append(now)

    def collision_callback(self, msg):
        if msg.data and self.test_active:
            now = time.time()
            self.collision_events.append(now)
            self.get_logger().warn(f'碰撞事件 @ {now:.1f}s')

    def deviation_callback(self, msg):
        if self.test_active:
            now = time.time()
            self.deviation_log.append((now, msg.data))

    def status_callback(self, msg):
        pass

    def run_test(self):
        self.test_active = True
        self.test_start_time = time.time()
        self.get_logger().info('测试开始')
        self.run_scenario()
        self.wait_for_completion()
        metrics = self.compute_metrics()
        self.save_results(metrics)
        return metrics

    def run_scenario(self):
        scenarios = {
            'straight_lane': self.scenario_straight_lane,
            'city_junction': self.scenario_city_junction,
            'pedestrian_crossing': self.scenario_pedestrian_crossing,
            'emergency_brake': self.scenario_emergency_brake,
            'sensor_failure': self.scenario_sensor_failure,
        }
        func = scenarios.get(self.scenario_name, self.scenario_default)
        func()

    def scenario_straight_lane(self):
        self.get_logger().info('场景: 直线车道巡航')
        cmd = Twist()
        cmd.linear.x = 5.0
        self.cmd_pub.publish(cmd)

    def scenario_city_junction(self):
        self.get_logger().info('场景: 城市路口')
        cmd = Twist()
        cmd.linear.x = 3.0
        self.cmd_pub.publish(cmd)

    def scenario_pedestrian_crossing(self):
        self.get_logger().info('场景: 行人横穿')
        cmd = Twist()
        cmd.linear.x = 4.0
        self.cmd_pub.publish(cmd)

    def scenario_emergency_brake(self):
        self.get_logger().info('场景: 紧急制动')
        cmd = Twist()
        cmd.linear.x = 8.0
        self.cmd_pub.publish(cmd)

    def scenario_sensor_failure(self):
        self.get_logger().info('场景: 传感器失效')
        cmd = Twist()
        cmd.linear.x = 3.0
        self.cmd_pub.publish(cmd)

    def scenario_default(self):
        cmd = Twist()
        cmd.linear.x = 4.0
        self.cmd_pub.publish(cmd)

    def wait_for_completion(self):
        elapsed = 0.0
        while elapsed < self.test_duration and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed = time.time() - self.test_start_time
        self.test_active = False
        self.get_logger().info('测试完成')

    def compute_metrics(self) -> TestMetrics:
        metrics = TestMetrics()

        if self.velocity_log and self.time_log:
            metrics.avg_speed = float(np.mean(self.velocity_log))
            metrics.max_speed = float(np.max(self.velocity_log))

        if self.ttc_log:
            ttc_values = [t for _, t in self.ttc_log if t < INF]
            if ttc_values:
                metrics.min_ttc = float(min(ttc_values))
                metrics.avg_ttc = float(np.mean(ttc_values))

        if self.deviation_log:
            deviations = [d for _, d in self.deviation_log]
            threshold = 0.4 * 3.5
            deviation_count = sum(1 for d in deviations if d > threshold)
            metrics.deviation_rate = (
                deviation_count / len(deviations) if deviations else 0.0)

        if len(self.velocity_log) > 2 and len(self.time_log) > 2:
            acc = np.diff(self.velocity_log) / np.diff(self.time_log)
            jerk = np.diff(acc) / np.diff(self.time_log[1:])
            if len(jerk) > 0:
                metrics.rms_jerk = float(np.sqrt(np.mean(jerk ** 2)))
                metrics.max_jerk = float(np.max(np.abs(jerk)))

        metrics.collision_count = len(self.collision_events)
        metrics.task_completed = metrics.collision_count == 0

        return metrics

    def save_results(self, metrics: TestMetrics):
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'{self.scenario_name}_{self.fault_profile}_{timestamp}'

        log_file = os.path.join(self.output_dir, f'{filename}_log.csv')
        with open(log_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'velocity', 'deviation'])
            for i in range(min(len(self.time_log), len(self.velocity_log))):
                t = self.time_log[i] - self.test_start_time if self.test_start_time else 0
                v = self.velocity_log[i] if i < len(self.velocity_log) else 0
                d = self.deviation_log[i][1] if i < len(self.deviation_log) else 0
                writer.writerow([t, v, d])

        metrics_dict = asdict(metrics)
        metrics_file = os.path.join(self.output_dir, f'{filename}_metrics.json')
        with open(metrics_file, 'w') as f:
            json.dump({
                'scenario': self.scenario_name,
                'fault_profile': self.fault_profile,
                'duration_s': self.test_duration,
                'metrics': metrics_dict,
                'timestamp': timestamp
            }, f, indent=2)

        self.get_logger().info(f'结果保存到 {metrics_file}')

        return metrics_dict

    def report_progress(self):
        if self.test_active and self.test_start_time:
            elapsed = time.time() - self.test_start_time
            progress = min(100.0, elapsed / self.test_duration * 100)
            if self.velocity_log:
                current_vel = self.velocity_log[-1]
                self.get_logger().info(
                    f'[{progress:.0f}%] 速度={current_vel:.1f}m/s, '
                    f'碰撞={len(self.collision_events)}, '
                    f'数据点={len(self.velocity_log)}')

    def cleanup(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        self.cmd_pub.publish(cmd)
        self.get_logger().info('测试结束，车辆停止')


class TestRunner:
    __test__ = False

    def __init__(self, output_dir='results'):
        self.output_dir = output_dir
        self.results = {}

    def run_single(self, scenario, duration, fault_profile):
        print(f'\n=== 运行测试: {scenario} [{fault_profile}] ===')
        node = IntegrationTestNode()
        node.scenario_name = scenario
        node.test_duration = duration
        node.fault_profile = fault_profile
        node.output_dir = self.output_dir

        executor = MultiThreadedExecutor()
        executor.add_node(node)

        try:
            metrics = node.run_test()
            self.results[f'{scenario}_{fault_profile}'] = metrics
            return metrics
        except Exception as e:
            print(f'测试失败: {e}')
        finally:
            node.cleanup()
            node.destroy_node()

        return None

    def run_all(self):
        scenarios = ['straight_lane', 'city_junction', 'pedestrian_crossing']
        profiles = ['baseline', 'lidar_drop_5pct', 'camera_latency_200ms']

        for scenario in scenarios:
            for profile in profiles:
                self.run_single(scenario, 60, profile)
                time.sleep(2.0)

        self.save_summary()

    def save_summary(self):
        summary_file = os.path.join(self.output_dir, 'summary.json')
        summary = {}
        for key, metrics in self.results.items():
            if metrics:
                summary[key] = asdict(metrics)
        with open(summary_file, 'w') as f:
            json.dump(summary, f, indent=2)
        print(f'\n测试汇总保存到 {summary_file}')
        return summary


def main(args=None):
    rclpy.init(args=args)

    import sys
    if '--run-all' in sys.argv:
        runner = TestRunner(output_dir='results/ch30_eval')
        runner.run_all()
    elif '--scenario' in sys.argv:
        idx = sys.argv.index('--scenario')
        scenario = sys.argv[idx + 1] if idx + 1 < len(sys.argv) else 'straight_lane'
        duration = 120
        if '--duration' in sys.argv:
            didx = sys.argv.index('--duration')
            duration = float(sys.argv[didx + 1]) if didx + 1 < len(sys.argv) else 120
        profile = 'baseline'
        if '--fault-profile' in sys.argv:
            pidx = sys.argv.index('--fault-profile')
            profile = sys.argv[pidx + 1] if pidx + 1 < len(sys.argv) else 'baseline'

        node = IntegrationTestNode()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        try:
            node.run_test()
        finally:
            node.cleanup()
            node.destroy_node()
    else:
        print('用法: python integration_test.py --scenario <name> [--duration <s>]')
        print('       python integration_test.py --run-all')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
