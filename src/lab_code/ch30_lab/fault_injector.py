#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String, Header
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
import numpy as np
import random
import math
import json
import time
from enum import Enum
from collections import deque


class FaultType(Enum):
    DROP = 'drop'
    NOISE = 'noise'
    BIAS = 'bias'
    STALL = 'stall'
    LATENCY = 'latency'


class FaultState:
    def __init__(self, fault_type, params=None):
        self.fault_type = fault_type
        self.params = params or {}
        self.active = True
        self.start_time = None
        self.stall_end_time = None


class BaseFault:
    def __init__(self, node, fault_config):
        self.node = node
        self.config = fault_config
        self.fault_type = fault_config['type']
        self.active = True

    def apply(self, msg):
        raise NotImplementedError

    def get_status(self):
        return {'type': self.fault_type, 'active': self.active}


class DropFrameFault(BaseFault):
    def __init__(self, node, config):
        super().__init__(node, config)
        self.drop_rate = config.get('drop_rate', 0.1)
        self.frame_count = 0

    def apply(self, msg):
        self.frame_count += 1
        if random.random() < self.drop_rate:
            self.node.get_logger().debug(
                f'DropFrameFault: 丢弃第{self.frame_count}帧')
            return None
        return msg

    def get_status(self):
        return {'type': 'drop', 'active': self.active, 'rate': self.drop_rate}


class NoiseFault(BaseFault):
    def __init__(self, node, config):
        super().__init__(node, config)
        self.stddev = config.get('stddev', 0.3)
        self.fields = config.get('fields', ['position.x', 'position.y'])

    def apply(self, msg):
        noise = np.random.normal(0, self.stddev)
        if hasattr(msg, 'pose') and hasattr(msg.pose, 'position'):
            if 'position.x' in self.fields:
                msg.pose.position.x += noise * random.choice([-1, 1])
            if 'position.y' in self.fields:
                msg.pose.position.y += noise * random.choice([-1, 1])
            if 'position.z' in self.fields:
                msg.pose.position.z += abs(noise) * random.choice([-1, 1])
        if hasattr(msg, 'twist') and hasattr(msg.twist, 'linear'):
            msg.twist.linear.x += np.random.normal(0, self.stddev * 0.5)
            msg.twist.linear.y += np.random.normal(0, self.stddev * 0.5)
        return msg

    def get_status(self):
        return {
            'type': 'noise', 'active': self.active,
            'stddev': self.stddev, 'fields': self.fields
        }


class BiasFault(BaseFault):
    def __init__(self, node, config):
        super().__init__(node, config)
        self.offset = config.get('offset', 1.5)
        self.dimension = config.get('dimension', 'x')

    def apply(self, msg):
        if hasattr(msg, 'pose') and hasattr(msg.pose, 'position'):
            pos = msg.pose.position
            if self.dimension == 'x':
                pos.x += self.offset
            elif self.dimension == 'y':
                pos.y += self.offset
            elif self.dimension == 'z':
                pos.z += self.offset
        return msg


class StallFault(BaseFault):
    def __init__(self, node, config):
        super().__init__(node, config)
        self.duration = config.get('duration', 2.0)
        self.stall_until = 0.0

    def apply(self, msg):
        now = time.time()
        if now < self.stall_until:
            return None
        return msg

    def trigger_stall(self):
        self.stall_until = time.time() + self.duration
        self.node.get_logger().warn(
            f'StallFault: 传感器失效 {self.duration}s')

    def get_status(self):
        remaining = max(0.0, self.stall_until - time.time())
        return {
            'type': 'stall', 'active': self.active,
            'duration': self.duration, 'remaining': remaining
        }


class LatencyFault(BaseFault):
    def __init__(self, node, config):
        super().__init__(node, config)
        self.delay_ms = config.get('delay_ms', 200)
        self.buffer = deque()

    def apply(self, msg):
        self.buffer.append((time.time(), msg))
        return None

    def flush_ready(self):
        now = time.time()
        ready_messages = []
        while self.buffer and (now - self.buffer[0][0]) * 1000 >= self.delay_ms:
            ready_messages.append(self.buffer.popleft()[1])
        return ready_messages

    def get_status(self):
        return {
            'type': 'latency', 'active': self.active,
            'delay_ms': self.delay_ms,
            'buffer_size': len(self.buffer)
        }


class FaultInjector(Node):

    def __init__(self):
        super().__init__('fault_injector')

        self.declare_parameter('faults', ['lidar_drop'])
        self.declare_parameter('drop_rate', 0.05)
        self.declare_parameter('noise_stddev', 0.3)
        self.declare_parameter('bias_offset', 1.5)
        self.declare_parameter('stall_duration', 2.0)
        self.declare_parameter('latency_ms', 200)
        self.declare_parameter('target_topics', [
            '/perception/objects',
            '/vehicle/odometry'
        ])

        self.fault_configs = self.get_parameter('faults').value
        self.target_topics = self.get_parameter('target_topics').value
        self.fault_instances = []
        self.active_faults = []

        self.status_pub = self.create_publisher(
            String, '/fault_injector/status', 10)
        self.active_faults_pub = self.create_publisher(
            String, '/fault_injector/active_faults', 10)

        self.setup_faults()
        self.setup_interception()

        self.timer = self.create_timer(1.0, self.publish_status)
        self.latency_timer = self.create_timer(0.05, self.flush_latency_buffers)

        self.get_logger().info(
            f'FaultInjector started with faults: {self.fault_configs}')

    def setup_faults(self):
        for fault_type in self.fault_configs:
            config = {'type': fault_type}
            if fault_type == 'lidar_drop' or fault_type == 'drop':
                config['drop_rate'] = self.get_parameter('drop_rate').value
                config['target'] = 'lidar'
                self.fault_instances.append(DropFrameFault(self, config))
            elif fault_type == 'noise':
                config['stddev'] = self.get_parameter('noise_stddev').value
                self.fault_instances.append(NoiseFault(self, config))
            elif fault_type == 'bias':
                config['offset'] = self.get_parameter('bias_offset').value
                self.fault_instances.append(BiasFault(self, config))
            elif fault_type == 'stall':
                config['duration'] = self.get_parameter('stall_duration').value
                instance = StallFault(self, config)
                instance.trigger_stall()
                self.fault_instances.append(instance)
            elif fault_type == 'latency':
                config['delay_ms'] = self.get_parameter('latency_ms').value
                self.fault_instances.append(LatencyFault(self, config))

    def setup_interception(self):
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        for i, topic in enumerate(self.target_topics):
            sub = self.create_subscription(
                String, topic,
                lambda msg, t=topic: self.intercept_callback(msg, t),
                qos)
            pub = self.create_publisher(
                String,
                f'{topic}_injected',
                10)
            self.get_logger().info(f'监听: {topic} → 发布到: {topic}_injected')

    def intercept_callback(self, msg, topic):
        processed = msg
        for fault in self.fault_instances:
            if not fault.active:
                continue
            result = fault.apply(processed)
            if result is None:
                return
            processed = result

        injected_topic = f'{topic}_injected'
        for pub in self.publishers:
            if pub.topic_name == injected_topic:
                pub.publish(processed)
                break

    def flush_latency_buffers(self):
        for fault in self.fault_instances:
            if isinstance(fault, LatencyFault) and fault.active:
                messages = fault.flush_ready()
                for msg in messages:
                    pass

    def publish_status(self):
        status_list = [f.get_status() for f in self.fault_instances]
        status_msg = String()
        status_msg.data = json.dumps(status_list, ensure_ascii=False)
        self.status_pub.publish(status_msg)

        active_names = []
        for f in self.fault_instances:
            if f.active:
                active_names.append(f.fault_type)
        active_msg = String()
        active_msg.data = json.dumps(active_names)
        self.active_faults_pub.publish(active_msg)

    def print_status(self):
        print(f'\n=== 故障注入器状态 ===')
        for f in self.fault_instances:
            status = f.get_status()
            print(f'  [{status["type"]}] active={status["active"]}', end='')
            if 'rate' in status:
                print(f' rate={status["rate"]:.1%}', end='')
            if 'stddev' in status:
                print(f' stddev={status["stddev"]}', end='')
            if 'delay_ms' in status:
                print(f' delay={status["delay_ms"]}ms', end='')
            if 'remaining' in status:
                print(f' remaining={status["remaining"]:.1f}s', end='')
            print()
        print('=' * 40)


def main(args=None):
    rclpy.init(args=args)
    node = FaultInjector()

    from threading import Thread

    def status_printer():
        while rclpy.ok():
            time.sleep(5)
            node.print_status()

    printer_thread = Thread(target=status_printer, daemon=True)
    printer_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
