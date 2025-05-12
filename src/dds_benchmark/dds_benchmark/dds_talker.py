import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import numpy as np
import os
import psutil
import argparse
import sys
from rclpy.qos import QoSProfile, QoSReliabilityPolicy


class Talker(Node):
    def __init__(self, msg_count, frequency):
        super().__init__('dds_talker')
        self.msg_count = msg_count
        self.frequency = frequency

        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.RELIABLE
        self.pub = self.create_publisher(String, 'dds_test', 10)
        self.count = 0
        self.process = psutil.Process(os.getpid())
        self.done = False
        self.get_logger().info(f"Starting talker! {self.frequency}Hz with {self.msg_count}msgs")

        timer_period = 1.0 / self.frequency
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        now_ns = time.time_ns()
        msg = String()
        msg.data = f"{now_ns}|payload"
        self.pub.publish(msg)
        self.count += 1
        _ = np.fft.fft(np.random.rand(3000))  # simulate load

        if self.count % (self.msg_count//10) == 0:
            cpu = self.process.cpu_percent(interval=None)
            mem = self.process.memory_info().rss / (1024 * 1024)
            self.get_logger().info(f"[{self.count}] CPU: {cpu:.2f}% | MEM: {mem:.2f} MB")

        if self.count >= self.msg_count:
            self.get_logger().info(f"Sent {self.count} messages.")
            self.timer.cancel()
            self.done = True
            return


def main():
    parser = argparse.ArgumentParser(description='Talker Node Parameters')
    parser.add_argument('msg_count', type=int, help='Number of messages to send')
    parser.add_argument('frequency', type=float, help='Publish frequency in Hz')
    args = parser.parse_args()

    rclpy.init()
    node = Talker(args.msg_count, args.frequency)
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
