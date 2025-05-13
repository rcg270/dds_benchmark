import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import numpy as np
import os
import psutil
import argparse
import csv
from rclpy.qos import QoSProfile, QoSReliabilityPolicy


class Listener(Node):
    def __init__(self, msg_count, timeout):
        super().__init__('dds_listener')
        self.msg_count = msg_count
        self.expect_msg_count = msg_count
        self.first_msg_time_ns = None
        self.last_msg_received_ns = time.time_ns()
        self.timeout_s = timeout

        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.RELIABLE
        self.sub = self.create_subscription(String, 'dds_test', self.callback, qos_profile)

        self.latencies = []
        self.cpu_samples = []
        self.mem_samples = []
        self.count = 0
        self.process = psutil.Process(os.getpid())
        self.done = False

        self.get_logger().info("Starting listener...")

    def callback(self, msg):
        try:
            sent_time_str, _ = msg.data.split("|", 1)
            sent_time_ns = int(sent_time_str)
            now_ns = time.time_ns()
            latency_ms = (now_ns - sent_time_ns) / 1e6
            self.latencies.append(latency_ms)

            if self.first_msg_time_ns is None:
                self.first_msg_time_ns = sent_time_ns

            self.last_msg_received_ns = now_ns
            self.count += 1

            _ = np.fft.fft(np.random.rand(5000))  # simulate load

            self.cpu_samples.append(self.process.cpu_percent(interval=None))
            self.mem_samples.append(self.process.memory_info().rss / (1024 * 1024))

            if self.count % (self.msg_count // 10) == 0:
                self.get_logger().info(
                    f"[{self.count}] CPU: {self.cpu_samples[-1]:.2f}%"
                    f" | MEM: {self.mem_samples[-1]:.2f} MB"
                )

            if self.count >= self.msg_count:
                self.report()
                self.done = True

        except Exception as e:
            self.get_logger().error(f"Error processing message: {e}")

    def report(self):
        arr = np.array(self.latencies)
        duration_s = (self.last_msg_received_ns - self.first_msg_time_ns) / 1e9
        throughput = self.count / duration_s if duration_s > 0 else 0
        drops = self.expect_msg_count - self.count
        drop_rate = drops / self.expect_msg_count * 100 if self.expect_msg_count > 0 else 0

        self.get_logger().info(f"Samples: {len(arr)} | "
                               f"Avg: {arr.mean():.3f} ms | Min: {arr.min():.3f} ms | "
                               f"Max: {arr.max():.3f} ms | Std: {arr.std():.3f} ms | "
                               f"Throughput: {throughput:.3f} msg/s | Dropped: {drops} | "
                               f"Drop rate: {drop_rate:.3f}% | Duration: {duration_s:.3f} s")
        self.get_logger().info("Listener finished.")
        self.save_to_csv(arr, throughput, drops, drop_rate)

    def save_to_csv(self, arr, throughput, drops, drop_rate):
        csv_path = os.path.join("logs", os.environ.get("RMW_IMPLEMENTATION", "unknown") + ".csv")
        file_exists = os.path.isfile(csv_path)

        with open(csv_path, "a", newline='') as csvfile:
            writer = csv.writer(csvfile)
            if not file_exists:
                writer.writerow([
                    "rmw_impl", "msg_count", "payload_size", "frequency_hz",
                    "avg_latency_ms", "min_latency_ms", "max_latency_ms", "std_latency_ms",
                    "throughput_msgs_per_sec", "drops", "drop_rate",
                    "cpu_avg", "mem_avg_mb"
                ])
            writer.writerow([
                os.environ.get("RMW_IMPLEMENTATION", "unknown"),
                self.expect_msg_count,
                os.environ.get("PAYLOAD_SIZE", "unknown"),
                os.environ.get("FREQUENCY_HZ", "unknown"),
                round(arr.mean(), 3), round(arr.min(), 3), round(arr.max(), 3), round(arr.std(), 3),
                round(throughput, 2), drops, round(drop_rate, 2),
                round(np.mean(self.cpu_samples), 2),
                round(np.mean(self.mem_samples), 2)
            ])


def main():
    parser = argparse.ArgumentParser(description='Listener Node Parameters')
    parser.add_argument('msg_count', type=int, help='Number of messages to receive')
    parser.add_argument('timeout', type=int, default=10, help='Timeout in seconds')
    args = parser.parse_args()

    rclpy.init()
    node = Listener(args.msg_count, args.timeout)

    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time_ns() - node.last_msg_received_ns > args.timeout * 1e9:
            node.get_logger().info("Timeout reached, stopping listener.")
            node.report()
            node.done = True

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
