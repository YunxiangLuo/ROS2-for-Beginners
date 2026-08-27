import random
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy

from std_msgs.msg import String
from std_srvs.srv import SetBool


class FaultInjector(Node):

    def __init__(self):
        super().__init__('fault_injector')

        self.declare_parameter('fault_type', 'drop')
        self.declare_parameter('target_topic', '/plan')
        self.declare_parameter('fault_probability', 0.1)

        self._fault_type = self.get_parameter('fault_type').value
        self._target_topic = self.get_parameter('target_topic').value
        self._fault_probability = self.get_parameter('fault_probability').value
        self._active = True
        self._stall_active = False
        self._buffer = []

        self._srv = self.create_service(SetBool, '/inject_fault', self._inject_fault_callback)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=10,
        )

        self._pub = self.create_publisher(String, self._target_topic + '_injected', 10)
        self._sub = self.create_subscription(
            String,
            self._target_topic,
            self._fault_sub_callback,
            qos,
        )

        self._latency_thread = None
        self._stall_thread = None

        self.get_logger().info(
            f'FaultInjector initialized: type={self._fault_type}, '
            f'topic={self._target_topic}, prob={self._fault_probability}'
        )

    def _inject_fault_callback(self, request, response):
        if request.data:
            self._active = True
            self.get_logger().info('Fault injection enabled')
            response.message = 'Fault injection enabled'
        else:
            self._active = False
            self.get_logger().info('Fault injection disabled')
            response.message = 'Fault injection disabled'
        response.success = True
        return response

    def _apply_drop(self, msg):
        if random.random() < self._fault_probability:
            self.get_logger().warn(f'Dropping message on {self._target_topic}')
            return True
        return False

    def _apply_noise(self, msg):
        if random.random() < self._fault_probability:
            noisy_msg = String()
            noisy_msg.data = msg.data
            noise = random.gauss(0.0, 1.0)
            noisy_msg.data += f' [noise:{noise:.3f}]'
            self._pub.publish(noisy_msg)
            self.get_logger().info(f'Applied noise to message: {noise:.3f}')
            return True
        return False

    def _apply_bias(self, msg):
        if random.random() < self._fault_probability:
            biased_msg = String()
            bias = 5.0
            biased_msg.data = msg.data + f' [bias:{bias:.1f}]'
            self._pub.publish(biased_msg)
            self.get_logger().info(f'Applied bias to message: {bias:.1f}')
            return True
        return False

    def _apply_latency(self, msg):
        if random.random() < self._fault_probability:
            delay = random.uniform(0.5, 3.0)
            self.get_logger().warn(f'Injecting latency: {delay:.2f}s')

            def delayed_publish(m, d):
                time.sleep(d)
                if self._active:
                    self._pub.publish(m)

            t = threading.Thread(target=delayed_publish, args=(msg, delay), daemon=True)
            t.start()
            return True
        return False

    def _apply_stall(self, msg):
        if not self._stall_active and random.random() < self._fault_probability:
            self._stall_active = True
            duration = random.uniform(2.0, 5.0)
            self.get_logger().warn(f'Stalling publishing for {duration:.2f}s')

            def stall(d):
                time.sleep(d)
                self._stall_active = False

            t = threading.Thread(target=stall, args=(duration,), daemon=True)
            t.start()

    def _fault_sub_callback(self, msg):
        if not self._active:
            self._pub.publish(msg)
            return

        if self._fault_type == 'drop':
            if self._apply_drop(msg):
                return
        elif self._fault_type == 'noise':
            if self._apply_noise(msg):
                return
        elif self._fault_type == 'bias':
            if self._apply_bias(msg):
                return
        elif self._fault_type == 'latency':
            if self._apply_latency(msg):
                return
        elif self._fault_type == 'stall':
            self._apply_stall(msg)
            if self._stall_active:
                return

        self._pub.publish(msg)

    def configure(self, fault_type=None, target_topic=None, fault_probability=None):
        if fault_type is not None:
            self._fault_type = fault_type
        if target_topic is not None:
            self._target_topic = target_topic
            self._sub.destroy()
            qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                depth=10,
            )
            self._sub = self.create_subscription(
                String,
                self._target_topic,
                self._fault_sub_callback,
                qos,
            )
        if fault_probability is not None:
            self._fault_probability = fault_probability
        self.get_logger().info(
            f'Reconfigured: type={self._fault_type}, '
            f'topic={self._target_topic}, prob={self._fault_probability}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = FaultInjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
