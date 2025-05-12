import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import numpy as np
import os
import psutil
import sys
import argparse
from rclpy.qos import QoSProfile, QoSReliabilityPolicy


class Listener(Node):
    def __init__(self, msg_count):
        super().__init__('dds_listener')
        self.msg_count = msg_count
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.RELIABLE
        self.sub = self.create_subscription(String, 'dds_test', self.callback, 10)
        self.latencies = []
        self.count = 0
        self.process = psutil.Process(os.getpid())
        self.done = False  # NEW FLAG
        self.get_logger().info("Starting listener...")

    def callback(self, msg):
        try:
            sent_time_str, _ = msg.data.split("|", 1)
            sent_time_ns = int(sent_time_str)
            now_ns = time.time_ns()
            latency_ms = (now_ns - sent_time_ns) / 1e6
            self.latencies.append(latency_ms)
            self.count += 1

            _ = np.fft.fft(np.random.rand(5000))  # simulate load

            if self.count % (self.msg_count//10) == 0:
                cpu = self.process.cpu_percent(interval=None)
                mem = self.process.memory_info().rss / (1024 * 1024)
                self.get_logger().info(f"[{self.count}] CPU: {cpu:.2f}% | MEM: {mem:.2f} MB")

            if self.count >= self.msg_count:
                self.report()
                self.done = True
        except Exception as e:
            self.get_logger().error(f"Error processing message: {e}")

    def report(self):
        arr = np.array(self.latencies)
        self.get_logger().info(f"Samples: {len(arr)}")
        self.get_logger().info(f"Avg: {arr.mean():.3f} ms | Min: {arr.min():.3f} ms | Max: {arr.max():.3f} ms | Std: {arr.std():.3f} ms")


def main():
    parser = argparse.ArgumentParser(description='Listener Node Parameters')
    parser.add_argument('msg_count', type=int, help='Number of messages to receive')
    args = parser.parse_args()
    rclpy.init()
    node = Listener(args.msg_count)
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
